/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 * 【逐行注释版】esp-radar 旧版主组件 —— WiFi CSI 感知的完整数据通路
 * ============================================================================
 * 数据流水线总览：
 *   WiFi 硬件(每帧回调) ──► esp_radar_csi_rx_cb          [WiFi 任务上下文]
 *        │ MAC 过滤 / 子载波重建 / 增益补偿 / 时间限流
 *        ▼  csi_info_queue
 *   csi_preprocessing_task                               [任务1: 预处理]
 *        │ 取 LTF / I,Q→振幅 / 写环形缓冲 / 离群过滤 / 滑窗推进
 *        ▼  csi_data_queue
 *   csi_detection_task                                   [任务2: 特征计算]
 *        │ PCA 指纹 → jitter(帧间) → wander(对背景) → 训练采样 → 语义翻转
 *        ▼
 *   wifi_radar_cb 用户回调（wifi_radar_info_t{jitter, wander}）
 *
 * 配套：old_pca.c(指纹数学) / old_utils.c(corr 等工具)
 * ============================================================================
 */
#include "freertos/FreeRTOS.h"          /* FreeRTOS 核心：任务/队列/事件组 */
#include "freertos/task.h"              /* 任务 API：xTaskCreate/vTaskDelete */
#include "freertos/event_groups.h"      /* 事件组：任务退出同步 */
#include "freertos/queue.h"             /* 队列：两级流水线的消息通道 */

#include "esp_mac.h"                    /* MAC 地址工具 */
#include "esp_system.h"                 /* 系统工具（堆内存查询等） */
#include "esp_wifi.h"                   /* WiFi API：CSI 配置/回调/信道 */
#include "esp_log.h"                    /* 日志 */
#include "esp_check.h"                  /* ESP_RETURN_ON_ERROR 等检查宏 */
#include "esp_private/esp_wifi_he_types_private.h"   /* 11ax(HE) 私有类型：SIG 字段解析 */
#include "esp_radar.h"                  /* 本组件公共头：配置结构/API 声明 */
#include "esp_csi_gain_ctrl.h"          /* CSI 增益控制器：AGC/FFT 增益补偿 */
#include "utils.h"                      /* corr/avg 等数学工具（见 old_utils.c） */

static const char *TAG                          = "esp_radar";            /* 主日志标签 */
static const char *TAG_DETECTION                = "csi_detection_task";   /* 特征任务日志标签 */
static const char *TAG_TRAIN                     = "esp_radar_train";      /* 训练日志标签 */
#define CSI_DETECTION_EXIT_BIT      BIT0    /* 事件位：特征任务已退出 */
#define CSI_PREPROCESSING_EXIT_BIT  BIT1    /* 事件位：预处理任务已退出 */

#define RADAR_PCA_WINDOW_DEFAULT      4     /* PCA 指纹滑动窗口默认大小（帧数） */

/* ----------------------------------------------------------------------------
 * 数据结构定义
 * -------------------------------------------------------------------------- */

/**
 * CSI 振幅环形缓冲：整个算法的"原始数据池"
 * 每帧 CSI 算出振幅后写入这里，特征计算按窗口区间取数据。
 */
typedef struct {
    float **amplitude;      /* 二维振幅数组 [buff_size][subcarrier_len]，一行=一帧 */
    uint32_t *timestamp;    /* 每帧时间戳（硬件 us ÷1000 = ms），滑窗时间校验用 */
    uint32_t *seq_id;       /* 每帧包序号，排查丢帧用 */
} csi_data_buff_t;

/**
 * 滑窗上下文：管理环形缓冲与处理窗口的推进
 * seq 是绝对递增号，对 buff_size 取模才是环形物理下标。
 */
typedef struct {
    uint32_t buff_size;         /* 环形缓冲容量（帧数） */
    uint32_t handle_window;     /* 目标处理窗口大小（帧数） */
    uint32_t window_start_seq;  /* 当前窗口起始绝对序号 */
    uint32_t next_seq;          /* 下一帧写入的绝对序号（=已收帧总数） */
    uint32_t last_timestamp;    /* 上一帧时间戳(ms)，检测断流/跳变 */
} csi_window_ctx_t;

/**
 * 雷达总上下文：系统全部状态与配置都在这一个结构里
 */
typedef struct {
    EventGroupHandle_t          task_exit_group;     /* 任务退出同步事件组 */
    QueueHandle_t               csi_info_queue;      /* 队列1：回调→预处理，传 filtered_info* */
    QueueHandle_t               csi_data_queue;      /* 队列2：预处理→特征，传窗口索引 */
    esp_radar_dec_config_t      dec_config;          /* 算法配置（窗口/阈值/回调等） */
    esp_radar_csi_config_t      csi_config;          /* CSI 硬件采集配置 */
    esp_radar_wifi_config_t     wifi_config;         /* WiFi 信道/协议配置 */
    esp_radar_espnow_config_t   espnow_config;       /* ESPNOW 发射端配置 */
    bool                        init_flag;           /* 已初始化标志（防重复 init） */
    bool                        run_flag;            /* 运行中标志（start/stop 翻转） */
    csi_window_ctx_t            window_ctx;          /* 滑窗管理上下文 */
    uint16_t                    subcarrier_len;      /* 子载波数（首帧自动探测确定） */
    csi_data_buff_t             *csi_data_buff;      /* CSI 振幅环形缓冲 */
    bool                        lltf_bit_mode;       /* LLTF 12bit 紧凑格式开关（C5/C61） */
} radar_ctx_t;

/**
 * 窗口索引："提货单"。union 技巧使整个结构可作为 32 位整数过队列
 * （队列传 4 字节而不是整个结构，省拷贝）。
 */
typedef struct {
    union {
        struct {
            uint8_t begin;      /* 窗口起始环形下标 */
            uint8_t end;        /* 窗口结束环形下标 */
            uint8_t window;     /* 窗口帧数 */
            uint8_t reserved;   /* 对齐保留 */
        };
        uint32_t data;          /* 整体 32 位传输 */
    };
} csi_data_buff_index_t;

/* ---- 训练常量 ---- */
#define CSI_CORR_NUM              10        /* 背景指纹库容量（10 条 PCA 指纹） */
#define CSI_CORR_THRESHOLD        0.998f    /* 训练期"环境稳定"判定：corr > 0.998 */
#define RADAR_BUFF_NUM            3         /* jitter 三点滑窗长度（离群剔除） */
#define RADAR_OUTLIERS_THRESHOLD  0.005f    /* 三点离群阈值：中间值两侧低 0.005 即丢弃 */

/**
 * 标定状态机：NO(未训练) → PROGRESS(采背景中) → COMPLETE(阈值有效)
 */
typedef enum {
    RADAR_CALIBRATE_NO = 0,         /* 未训练 */
    RADAR_CALIBRATE_PROGRESS,       /* 训练进行中 */
    RADAR_CALIBRATE_COMPLETE,       /* 训练完成，阈值可用 */
} radar_calibrate_status_t;

/**
 * 标定（训练）数据 —— Presence 阈值的来源
 * 训练做两件事：
 *   1) 挑稳定时刻的 PCA 指纹存入 data[]（背景指纹库）
 *   2) 累计背景 corr 统计 → train_stop 时换算成阈值
 */
typedef struct {
    radar_calibrate_status_t calibrate_status;  /* 当前标定状态 */
    float *data[CSI_CORR_NUM];                  /* 背景指纹库（环形复用 10 条） */
    size_t buff_size;                            /* 三点滑窗已填充个数 */
    float waveform_jitter_buff[RADAR_BUFF_NUM];  /* jitter 三点滑窗缓存 */
    size_t data_num;                             /* 已收集背景指纹条数 */
    float none_corr_sum;                         /* 背景 corr 总和 */
    float none_corr_count;                       /* 背景 corr 计数 */
    float none_corr;                             /* 最近一次背景 corr（诊断） */
    float static_corr;                           /* 静止参考 corr（训练期最小 jitter corr） */
    uint16_t subcarrier_len;                     /* 训练时子载波数（变了要重训） */
} radar_calibrate_t;

static radar_ctx_t s_ctx = {0};                     /* 全局唯一实例 */
static uint32_t s_csi_seq = 0;                      /* CSI 帧全局序号计数器 */
static uint32_t s_pca_subcarrier_buff_num = 0;      /* 已算出的 PCA 指纹总数（环形索引基准） */
static radar_calibrate_t *s_radar_calibrate = NULL; /* 标定数据（train_start 时分配） */
static float s_waveform_wander_last = 0.0f;         /* 上一次 wander 值（训练决策滞后一拍用） */

/* ----------------------------------------------------------------------------
 * 回调层：从 WiFi 硬件到干净数据
 * -------------------------------------------------------------------------- */

/**
 * 从 WiFi 硬件寄存器视图(rx_ctrl)提取本帧元信息
 *
 * 这些信息决定"这帧 CSI 是什么格式"（11b/g/n/ax、带宽、STBC...），
 * rebuild_csi_data 靠它去子载波表查这帧数据的字节布局。
 * C5/C6/C61 走新寄存器布局（支持 HE/11ax），老芯片走旧布局。
 */
static esp_err_t esp_radar_extract_rx_ctrl_info(const wifi_pkt_rx_ctrl_t *rx_ctrl, esp_radar_rx_ctrl_info_t *info)
{
    if (!rx_ctrl || !info) {                    /* 参数保护：空指针直接返回 */
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    info->rssi = rx_ctrl->rssi;                 /* 接收信号强度 */
    info->rate = rx_ctrl->rate;                 /* 物理层速率 */
    info->rx_format = rx_ctrl->cur_bb_format;   /* 基带格式（11b/g/n/ac/ax） */
    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &info->agc_gain, &info->fft_gain);  /* 读 AGC/FFT 增益（后面补偿用） */
    info->timestamp = rx_ctrl->timestamp;       /* 硬件时间戳(us) */
    info->noise_floor = rx_ctrl->noise_floor;   /* 底噪 */
    info->channel = rx_ctrl->channel;           /* 主信道号 */
    info->secondary_channel = rx_ctrl->second;  /* 次信道（40MHz 用） */
    switch (rx_ctrl->cur_bb_format) {           /* 按格式解析 SIG 字段里的 MCS/带宽/STBC */
    case RX_FORMAT_11B:
    case RX_FORMAT_11G:  /* 11A 与 11G 数值相同，共用分支 */
        info->signal_mode = WIFI_SIGNAL_MODE_NON_HT;                 /* legacy 非 HT 模式 */
        info->mcs = ((esp_wifi_htsig_t *)&rx_ctrl->he_siga1)->mcs;   /* 速率字段 */
        info->cwb = ((esp_wifi_htsig_t *)&rx_ctrl->he_siga1)->cbw;   /* 带宽 */
        info->stbc = ((esp_wifi_htsig_t *)&rx_ctrl->he_siga1)->stbc; /* 空时块编码 */
        break;
    case RX_FORMAT_HT:                          /* 11n (HT) */
        info->signal_mode = WIFI_SIGNAL_MODE_HT;
        info->mcs = ((esp_wifi_htsig_t *)&rx_ctrl->he_siga1)->mcs;
        info->cwb = ((esp_wifi_htsig_t *)&rx_ctrl->he_siga1)->cbw;
        info->stbc = ((esp_wifi_htsig_t *)&rx_ctrl->he_siga1)->stbc;
        break;
    case RX_FORMAT_VHT:                         /* 11ac (VHT) */
        info->signal_mode = WIFI_SIGNAL_MODE_HT; /* 按 HT 布局近似处理 */
        info->mcs = ((esp_wifi_vht_siga1_t *)&rx_ctrl->he_siga1)->su_mcs;  /* VHT 的 SU MCS */
        info->cwb = ((esp_wifi_vht_siga1_t *)&rx_ctrl->he_siga1)->cbw;
        info->stbc = ((esp_wifi_vht_siga1_t *)&rx_ctrl->he_siga1)->stbc;
        break;
    case RX_FORMAT_HE_SU:                       /* 11ax 单用户 */
        info->signal_mode = WIFI_SIGNAL_MODE_HE;
        info->mcs = ((esp_wifi_su_siga1_t *)&rx_ctrl->he_siga1)->he_mcs;
        info->cwb = ((esp_wifi_su_siga1_t *)&rx_ctrl->he_siga1)->bw;
        info->stbc = ((esp_wifi_su_siga2_t *)&rx_ctrl->he_siga2)->stbc;
        break;
    case RX_FORMAT_HE_MU:                       /* 11ax 多用户 */
        info->signal_mode = WIFI_SIGNAL_MODE_HE;
        info->mcs = ((esp_wifi_mu_siga1_t *)&rx_ctrl->he_siga1)->sigb_mcs; /* SIG-B 的 MCS */
        info->cwb = ((esp_wifi_mu_siga1_t *)&rx_ctrl->he_siga1)->bw;
        info->stbc = ((esp_wifi_mu_siga2_t *)&rx_ctrl->he_siga2)->stbc;
        break;
    default:
        return ESP_FAIL;                        /* 未知格式：布局无法解析 */
    }

#elif CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
    /* 老芯片：寄存器布局简单，只有 HT/非HT 两种 */
    info->rssi = rx_ctrl->rssi;
    info->rate = rx_ctrl->rate;
    info->rx_format = rx_ctrl->sig_mode;        /* 0=非HT 1=HT */
    if (rx_ctrl->sig_mode == RX_FORMAT_NON_HT) {
        info->signal_mode = WIFI_SIGNAL_MODE_NON_HT;
    } else {
        info->signal_mode = WIFI_SIGNAL_MODE_HT;
    }
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
    info->agc_gain = 0;                         /* 这两颗芯片读不到增益：不补偿 */
    info->fft_gain = 0;
#else
    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &info->agc_gain, &info->fft_gain);
#endif
    info->timestamp = rx_ctrl->timestamp;
    info->noise_floor = rx_ctrl->noise_floor;
    info->channel = rx_ctrl->channel;
    info->secondary_channel = rx_ctrl->secondary_channel;
    info->mcs = rx_ctrl->mcs;
    info->cwb = rx_ctrl->cwb;
    info->stbc = rx_ctrl->stbc;
#endif
    return ESP_OK;
}

/**
 * MAC 地址过滤：只放行"目标发送端"的帧
 *
 * 规则：
 *   1) 配了 filter_mac → 只收源 MAC 匹配的帧
 *   2) 没配 filter_mac → 只收"空数据帧"(payload=14B)：ESPNOW/keep-alive
 *      空帧最稳定，适合当 CSI 探测信号
 *   3) 配了 dmac 过滤 → 再过一层目的 MAC
 * 返回 ESP_FAIL 表示这帧不要。
 */
static esp_err_t esp_radar_mac_addr_filter(wifi_csi_info_t *info)
{
    static const char *TAG_FILTER = "esp_radar_filter";

    if ((!ADDR_IS_FULL(s_ctx.csi_config.filter_mac) &&  memcmp(info->mac, s_ctx.csi_config.filter_mac, 6) != 0)   /* 源 MAC 不匹配 */
#if WIFI_CSI_SEND_NULL_DATA_ENABLE
            || (ADDR_IS_EMPTY(s_ctx.csi_config.filter_mac) && info->payload_len != 14)   /* 或未配 MAC 但不是空帧 */
#endif
       ) {
        if ((ADDR_IS_EMPTY(s_ctx.csi_config.filter_mac) && info->payload_len != 14)) {
            ESP_LOGD(TAG_FILTER, "Not null data packet - payload_len: %d", info->payload_len);   /* 非空帧：丢 */
            return ESP_FAIL;
        }
        ESP_LOGD(TAG_FILTER, "Source MAC mismatch - mac: " MACSTR ", filter_mac: " MACSTR,  MAC2STR(info->mac), MAC2STR(s_ctx.csi_config.filter_mac));   /* 源 MAC 不符：丢 */
        return ESP_FAIL;
    }

    if (s_ctx.csi_config.filter_dmac_flag && memcmp(info->dmac, s_ctx.csi_config.filter_dmac, 6) != 0) {  /* 目的 MAC 过滤 */
        ESP_LOGD(TAG_FILTER, "Dest MAC mismatch - dmac: " MACSTR ", filter_dmac: " MACSTR, MAC2STR(info->dmac), MAC2STR(s_ctx.csi_config.filter_dmac));
        return ESP_FAIL;
    }

    return ESP_OK;      /* 通过：这帧留下 */
}

/**
 * 帧间隔限流：两帧时间戳太近就丢（间隔 < 配置接收间隔的 1/5）
 * 防突发帧（重传/AP 突发）打乱窗口的时间特性。
 */
static esp_err_t esp_radar_check_timestamp_interval(uint32_t timestamp)
{
    static uint32_t s_last_timestamp = 0;   /* 上一帧时间戳（静态局部） */

    uint32_t min_interval_us = s_ctx.csi_config.csi_recv_interval * 1000 / 5;   /* 最小间隔 = 配置间隔的 20% */

    if (timestamp - s_last_timestamp <= min_interval_us) {   /* 距上帧太近 → 丢 */
        return ESP_FAIL;
    }

    s_last_timestamp = timestamp;           /* 记录本帧时间 */
    return ESP_OK;
}

/**
 * 按子载波区间表从原始 CSI 拷贝有效数据段
 *
 * 原始 CSI 缓冲里混着导频/保护等无效字节。sub_carrier_table
 * （csi_sub_carrier_table.c）按信号格式登记了各 LTF 段有效子载波的
 * 字节区间，本函数逐段拷贝到输出紧凑缓冲。
 * lltf_bit_mode=true：源是 12bit 紧凑排列，按 16bit 单位做符号扩展
 * （左移 4 再算术右移 4）。
 */
static void copy_subcarrier_data(wifi_csi_filtered_info_t *out_filtered_info, const int8_t *src_buf, const sub_carrier_range_t *range_array, uint16_t target_bytes, uint16_t *field_len)
{
    uint8_t i = 0;                                  /* 区间数组游标 */
    while (*field_len < target_bytes) {             /* 拷满目标字节数为止 */
        uint16_t size = range_array[i].stop - range_array[i].start;   /* 本区间字节数 */

        if (s_ctx.lltf_bit_mode) {                  /* 12bit 紧凑模式（C5/C61 LLTF） */
            int16_t *dest = (int16_t *)(out_filtered_info->valid_data + out_filtered_info->valid_len);   /* 目标按 16bit 写 */
            const uint16_t *src = (const uint16_t *)(src_buf + range_array[i].start);
            for (uint16_t j = 0; j < size / 2; j++) {
                dest[j] = ((int16_t)(src[j] << 4)) >> 4;    /* 左移4算术右移4 = 符号扩展 12→16bit */
            }
        } else {
            memcpy(out_filtered_info->valid_data + out_filtered_info->valid_len, src_buf + range_array[i].start, size);  /* 普通：直接 memcpy */
        }

        *field_len += size;                         /* 本字段累计长度 */
        out_filtered_info->valid_len += size;       /* 总有效长度 */
        i++;                                        /* 下一个区间 */
    }
}

/**
 * CSI 数据重建：把"硬件原始布局"翻译成"按 LTF 分段的有效子载波数据"
 *
 * 这是理解 CSI 数据布局的关键函数：
 *   - 一帧 CSI 的原始 buf 里 LLTF/HT-LTF/STBC/HE-LTF 各段交织排列，
 *     且混着无效子载波（导频/保护）
 *   - sub_carrier_table 按 (信号模式, 带宽, STBC, 数据长度) 索引，
 *     命中哪一行就知道各段有效子载波的字节区间
 *   - 命中失败（格式不在表里）→ 丢帧
 */
static esp_err_t esp_radar_rebuild_csi_data(const wifi_csi_info_t *info, const esp_radar_rx_ctrl_info_t *rx_ctrl_info, wifi_csi_filtered_info_t **out_filtered_info)
{
    static const char *TAG_FILTER_CSI = "esp_radar_csi_data_rebuild";
    if (!info || !rx_ctrl_info || !out_filtered_info) {     /* 参数保护 */
        return ESP_ERR_INVALID_ARG;
    }

    *out_filtered_info = NULL;

    for (size_t i = 0; i < sub_carrier_table_size; ++i) {   /* 遍历布局表找匹配行 */
        bool int16_mismatch = (s_ctx.lltf_bit_mode && sub_carrier_table[i].total_bytes != info->len);   /* 16bit 模式：先比总长 */

        bool int8_mismatch = (!s_ctx.lltf_bit_mode &&
                              (rx_ctrl_info->signal_mode != sub_carrier_table[i].signal_mode ||   /* 信号模式匹配 */
                               rx_ctrl_info->cwb != sub_carrier_table[i].channel_bandwidth ||     /* 带宽匹配 */
                               rx_ctrl_info->stbc != sub_carrier_table[i].stbc                    /* STBC 匹配 */
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
                               || rx_ctrl_info->secondary_channel != sub_carrier_table[i].second  /* 老芯片还要比次信道 */
#endif
                              ));

        if (int16_mismatch || int8_mismatch) {      /* 不匹配：看下一行 */
            continue;
        }

        const csi_sub_carrier_table_t *sub_carrier_index =  sub_carrier_table + i;   /* 命中此布局 */
        *out_filtered_info = RADAR_MALLOC_RETRY(sizeof(wifi_csi_filtered_info_t) + sub_carrier_index->valid_bytes);   /* 按有效字节数分配（尾部柔性区） */
        memset(*out_filtered_info, 0, sizeof(wifi_csi_filtered_info_t) + sub_carrier_index->valid_bytes);

        wifi_csi_filtered_info_t *filtered_info = *out_filtered_info;

        filtered_info->rx_ctrl_info = *rx_ctrl_info;    /* 带走元信息（RSSI/增益/时间戳） */
        filtered_info->data_type = s_ctx.lltf_bit_mode ? WIFI_CSI_DATA_TYPE_INT16 : WIFI_CSI_DATA_TYPE_INT8;   /* 记录数据格式 */
        filtered_info->raw_data = info->buf;            /* 原始缓冲指针（调试用） */
        filtered_info->raw_len  = info->len;
        memcpy(filtered_info->mac, info->mac, 6);       /* 源 MAC（分 peer 用） */
        memcpy(filtered_info->dmac, info->dmac, 6);     /* 目的 MAC */

        if (sub_carrier_index->lltf_bytes > 0) {        /* LLTF 段（legacy 长训练字段，子载波最多） */
            copy_subcarrier_data(filtered_info, info->buf, sub_carrier_index->lltf,
                                 sub_carrier_index->lltf_bytes, &filtered_info->valid_lltf_len);
        }

        if (sub_carrier_index->ht_ltf_bytes > 0) {      /* HT-LTF 段（11n） */
            copy_subcarrier_data(filtered_info, info->buf, sub_carrier_index->ht_ltf,
                                 sub_carrier_index->ht_ltf_bytes, &filtered_info->valid_ht_ltf_len);
        }

        if (sub_carrier_index->stbc_ht_ltf_bytes > 0) { /* STBC-HT-LTF 段 */
            copy_subcarrier_data(filtered_info, info->buf, sub_carrier_index->stbc_ht_ltf,
                                 sub_carrier_index->stbc_ht_ltf_bytes, &filtered_info->valid_stbc_ht_ltf_len);
        }
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
        if (sub_carrier_index->he_ltf_bytes > 0) {      /* HE-LTF 段（11ax，C5/C6/C61） */
            copy_subcarrier_data(filtered_info, info->buf, sub_carrier_index->he_ltf,
                                 sub_carrier_index->he_ltf_bytes, &filtered_info->valid_he_ltf_len);
        }
        if (sub_carrier_index->stbc_he_ltf_bytes > 0) { /* STBC-HE-LTF 段 */
            copy_subcarrier_data(filtered_info, info->buf, sub_carrier_index->stbc_he_ltf,
                                 sub_carrier_index->stbc_he_ltf_bytes, &filtered_info->valid_stbc_he_ltf_len);
        }
#endif

        ESP_LOGD(TAG_FILTER_CSI, "raw_len: %d, valid_len: %d, valid_lltf_len: %d, valid_ht_ltf_len: %d, valid_stbc_ht_ltf_len: %d"
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
                 ", valid_he_ltf_len: %d, valid_stbc_he_ltf_len: %d"
#endif
                 , filtered_info->raw_len, filtered_info->valid_len, filtered_info->valid_lltf_len, filtered_info->valid_ht_ltf_len, filtered_info->valid_stbc_ht_ltf_len
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
                 , filtered_info->valid_he_ltf_len, filtered_info->valid_stbc_he_ltf_len
#endif
                );
        return ESP_OK;      /* 成功重建一帧 */
    }

    ESP_LOGW(TAG_FILTER_CSI, "value fail, len: %d, secondary_channel: %d, sig_mode: %d, cwb: %d, stbc: %d",
             info->len, rx_ctrl_info->secondary_channel, rx_ctrl_info->rx_format, rx_ctrl_info->cwb, rx_ctrl_info->stbc);   /* 无匹配布局 */

    return ESP_FAIL;
}

/**
 * ★ CSI 硬件回调：WiFi 驱动每收到一帧带 CSI 的包就调一次 ★
 *
 * 运行在 WiFi 任务上下文，必须快：
 * 过滤 → 元信息 → 重建 → 增益补偿 → 用户回调 → 入队（不等待，满则丢）
 * 全部失败路径都要 free，防止内存泄漏。
 */
static void esp_radar_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    static const char *TAG_CB = "esp_radar_csi_rx_cb";
    if (!info || !info->buf) {                  /* 空帧保护 */
        ESP_LOGE(TAG_CB, "<%s> esp_radar_csi_rx_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    if (esp_radar_mac_addr_filter(info) != ESP_OK) {    /* MAC/payload 过滤：不是目标发送端就丢 */
        return;
    }

    esp_radar_rx_ctrl_info_t rx_ctrl_info;
    esp_radar_extract_rx_ctrl_info(&info->rx_ctrl, &rx_ctrl_info);   /* 提取元信息（格式/RSSI/增益/时间戳） */
    ESP_LOGD(TAG_CB, "timestamp: %ld, mac: " MACSTR", channel: %d, secondary_channel: %d, rssi: %d, rx_format: %d, cwb: %d, rate: %d, mcs: %d, stbc: %d, noise_floor: %d, len: %d, agc_gain: %d, fft_gain: %d",
             rx_ctrl_info.timestamp, MAC2STR(info->mac), rx_ctrl_info.channel, rx_ctrl_info.secondary_channel, rx_ctrl_info.rssi, rx_ctrl_info.rx_format, rx_ctrl_info.cwb, rx_ctrl_info.rate, rx_ctrl_info.mcs,
             rx_ctrl_info.stbc, rx_ctrl_info.noise_floor, info->len, rx_ctrl_info.agc_gain, rx_ctrl_info.fft_gain);

    wifi_csi_filtered_info_t *filtered_info = NULL;
    ESP_RETURN_VOID_ON_ERROR(esp_radar_rebuild_csi_data(info, &rx_ctrl_info, &filtered_info), TAG_CB, "Failed to filter CSI data");   /* 布局重建（失败已丢帧） */

    uint32_t seq = s_csi_seq++;             /* 分配全局递增序号（每帧唯一） */
    filtered_info->seq_id = seq;
    filtered_info->info = info;             /* 保留原始 info 指针（用户回调可能用） */

    float compensate_gain = 0;              /* 补偿量（dB，诊断记录用） */
#if WIFI_CSI_PHY_GAIN_ENABLE
    esp_csi_gain_ctrl_record_rx_gain(rx_ctrl_info.agc_gain, rx_ctrl_info.fft_gain);   /* 记录本帧增益到增益控制器 */
    if (s_ctx.csi_config.csi_compensate_en) {           /* 若开启增益补偿 */
        bool samples_are_16bit = (filtered_info->data_type == WIFI_CSI_DATA_TYPE_INT16);
        esp_csi_gain_ctrl_compensate_rx_gain(filtered_info->valid_data, filtered_info->valid_len, samples_are_16bit,
                                             &compensate_gain, rx_ctrl_info.agc_gain, rx_ctrl_info.fft_gain);   /* 原地补偿 AGC/FFT 增益差异 */
    }
#endif
    filtered_info->rx_gain_compensation = compensate_gain;
    if (s_ctx.csi_config.csi_filtered_cb) {             /* 可选：先给用户回调看一眼干净帧 */
        s_ctx.csi_config.csi_filtered_cb(s_ctx.csi_config.csi_filtered_cb_ctx, filtered_info);
    }
    if (!s_ctx.run_flag) {                              /* 已 stop：只回调不入队 */
        static bool run_flag_warned = false;
        if (!run_flag_warned) {
            ESP_LOGW(TAG_CB, "esp_radar not running, CSI data dropped");   /* 注意：原版没有置位 run_flag_warned，实际会每帧都打警告 */
        }
        RADAR_FREE(filtered_info);
    } else if (!s_ctx.csi_info_queue || xQueueSend(s_ctx.csi_info_queue, &filtered_info, 0) == pdFALSE) {   /* 入队（0 超时=不能堵 WiFi 任务，满则丢） */
        ESP_LOGW(TAG_CB, "Failed to send CSI data to queue, data dropped");
        RADAR_FREE(filtered_info);
    }
}
#define my_hypotf(a, b) sqrtf((a) * (a) + (b) * (b))    /* 手写 hypot：sqrt(I²+Q²)，比库函数快 */

/* ----------------------------------------------------------------------------
 * 预处理层：LTF 切段 / 离群过滤 / 滑窗推进
 * -------------------------------------------------------------------------- */

/**
 * 按配置的 LTF 类型从重建数据里切出对应段
 *
 * valid_data 布局 = [LLTF|HT-LTF|STBC-HT-LTF|HE-LTF|STBC-HE-LTF] 顺序拼接，
 * 按 ltf_type 定位偏移和长度。demo 用 LLTF（子载波最多、最灵敏）。
 */
static esp_err_t esp_radar_get_ltf_data(const wifi_csi_filtered_info_t *filtered_info, void **data_ptr, uint16_t *data_len)
{
    if (!filtered_info || !data_ptr || !data_len) {     /* 参数保护 */
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t *ptr = (uint8_t *)filtered_info->valid_data;   /* 数据区起点 */
    uint16_t offset = 0;                                /* 各段字节偏移累加器 */

    switch (s_ctx.dec_config.ltf_type) {
    case RADAR_LTF_TYPE_LLTF:               /* LLTF：就在开头 */
        *data_ptr = ptr;
        *data_len = filtered_info->valid_lltf_len;
        break;

    case RADAR_LTF_TYPE_HTLTF:              /* HT-LTF：跳过 LLTF */
        offset = filtered_info->valid_lltf_len;
        *data_ptr = ptr + offset;
        *data_len = filtered_info->valid_ht_ltf_len;
        break;

    case RADAR_LTF_TYPE_STBC_HTLTF:         /* STBC-HT-LTF：跳过前两段 */
        offset = filtered_info->valid_lltf_len + filtered_info->valid_ht_ltf_len;
        *data_ptr = ptr + offset;
        *data_len = filtered_info->valid_stbc_ht_ltf_len;
        break;

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    case RADAR_LTF_TYPE_HELTF:              /* HE-LTF：跳过前三段 */
        offset = filtered_info->valid_lltf_len + filtered_info->valid_ht_ltf_len +
                 filtered_info->valid_stbc_ht_ltf_len;
        *data_ptr = ptr + offset;
        *data_len = filtered_info->valid_he_ltf_len;
        break;

    case RADAR_LTF_TYPE_STBC_HELTF:         /* STBC-HE-LTF：跳过前四段 */
        offset = filtered_info->valid_lltf_len + filtered_info->valid_ht_ltf_len +
                 filtered_info->valid_stbc_ht_ltf_len + filtered_info->valid_he_ltf_len;
        *data_ptr = ptr + offset;
        *data_len = filtered_info->valid_stbc_he_ltf_len;
        break;
#endif

    default:
        ESP_LOGE(TAG, "Unsupported LTF type: %d", s_ctx.dec_config.ltf_type);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (*data_len == 0) {                   /* 该段没数据（如没配采集 LLTF） */
        ESP_LOGW(TAG, "LTF type %d has no data", s_ctx.dec_config.ltf_type);
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

/**
 * 离群帧过滤（软丢弃）：判断当前帧是否"整体异常"
 *
 * 方法：当前帧每个子载波与前 3 帧均值比，偏差超阈值且超过一半子载波
 *       → 判为离群帧。旧版只记 streak 不丢数据（"Soft-updated"）；
 *       连续 3 帧离群 = 环境真变了，接受为新基线。
 *       （新版闭源库改成了真丢帧。）
 */
static esp_err_t csi_outlier_filter_process(uint16_t subcarrier_len)
{
    const uint8_t threshold = s_ctx.dec_config.outliers_threshold;   /* 单子载波偏差阈值 */
    uint32_t curr_seq = s_ctx.window_ctx.next_seq;   /* 当前帧序号 */
    const uint8_t max_streak = 3;                    /* 连续多少帧算新基线 */
    static uint8_t outlier_streak = 0;               /* 连续离群计数 */
    if (curr_seq < 3) {                              /* 历史不足 3 帧 */
        outlier_streak = 0;
        return ESP_OK;
    }

    /* 取前 3 帧与当前帧的振幅行（环形取模） */
    float *hist0 = s_ctx.csi_data_buff->amplitude[(curr_seq - 3) % s_ctx.window_ctx.buff_size];
    float *hist1 = s_ctx.csi_data_buff->amplitude[(curr_seq - 2) % s_ctx.window_ctx.buff_size];
    float *hist2 = s_ctx.csi_data_buff->amplitude[(curr_seq - 1) % s_ctx.window_ctx.buff_size];
    float *curr  = s_ctx.csi_data_buff->amplitude[curr_seq % s_ctx.window_ctx.buff_size];

    uint32_t outliers_count = 0;                     /* 异常子载波计数 */

    for (int i = 0; i < subcarrier_len; ++i) {
        float ref = (hist0[i] + hist1[i] + hist2[i]) / 3.0f;   /* 参考 = 前 3 帧均值 */
        float diff = curr[i] - ref;                            /* 当前偏差 */

        if (diff > threshold || diff < -threshold) {           /* 超阈值计数 */
            outliers_count++;
        }
    }
    bool is_outlier_frame = (outliers_count >= subcarrier_len / 2);   /* 过半异常 = 整帧离群 */

    if (!is_outlier_frame) {                         /* 正常帧：清 streak */
        outlier_streak = 0;
        return ESP_OK;
    }
    outlier_streak++;                                /* 离群：streak+1 */

    if (outlier_streak >= max_streak) {              /* 连续多帧离群 → 环境变了 */
        ESP_LOGW(TAG, "Consecutive outliers (%u), accept as new baseline, seq=%lu", outlier_streak, curr_seq);
        outlier_streak = 0;
        return ESP_OK;                               /* 接受为新基线 */
    }
    ESP_LOGD(TAG, "Soft-updated outlier frame: %lu/%d at seq=%lu", outliers_count, subcarrier_len, curr_seq);

    return ESP_OK;      /* 软处理：不丢帧 */
}

/**
 * 滑窗推进：决定"攒够一批可以送去算特征了"
 *
 * 返回 ESP_OK 时 buff_index 装着本批 [begin,end,window]。
 * 策略（半窗重叠滑动）：
 *   - 攒够 handle_window 帧或跨 2 倍 csi_handle_time → 触发
 *   - 处理后窗口起点前移 window/2（50% 重叠，时间连续）
 *   - 帧数 < handle_window/3 不值得算 → 丢弃重开
 *   - 时间戳回退/断流太久 → 窗口作废重开
 */
static esp_err_t csi_window_update(csi_data_buff_index_t *buff_index)
{
    if (!s_ctx.csi_data_buff || s_ctx.window_ctx.buff_size == 0) {   /* 缓冲未就绪 */
        return ESP_FAIL;
    }

    buff_index->begin  = s_ctx.window_ctx.window_start_seq % s_ctx.window_ctx.buff_size;   /* 起始环形下标 */
    buff_index->end    = s_ctx.window_ctx.next_seq % s_ctx.window_ctx.buff_size;           /* 结束环形下标 */
    buff_index->window = s_ctx.window_ctx.next_seq - s_ctx.window_ctx.window_start_seq;    /* 窗口帧数 */

    int32_t spent_time = s_ctx.csi_data_buff->timestamp[buff_index->end] - s_ctx.csi_data_buff->timestamp[buff_index->begin];   /* 窗口时间跨度(ms) */
    int32_t time_tamp  = s_ctx.csi_data_buff->timestamp[buff_index->end] - (int32_t)(s_ctx.window_ctx.last_timestamp);          /* 距上次处理间隔(ms) */

    esp_err_t ret = ESP_FAIL;

    if (time_tamp < 0 || time_tamp > (int32_t)(s_ctx.dec_config.csi_handle_time / 2)) {   /* 时间异常（回退/断流） */
        ESP_LOGW(TAG, "time_tamp: %d, spent_time: %d,csi_handle_time: %d, end: %d, last: %d, window: %d,handle_window: %d",
                 time_tamp, spent_time, s_ctx.dec_config.csi_handle_time, s_ctx.csi_data_buff->timestamp[buff_index->end], s_ctx.window_ctx.last_timestamp,
                 buff_index->window, s_ctx.window_ctx.handle_window);

        if (buff_index->window > s_ctx.window_ctx.handle_window / 3) {   /* 攒的够多：先处理掉这批 */
            buff_index->window--;                        /* 最后一点留给下个窗口 */
            buff_index->end = (s_ctx.window_ctx.next_seq - 1) % s_ctx.window_ctx.buff_size;
            ret = ESP_OK;
        }
        s_ctx.window_ctx.window_start_seq = s_ctx.window_ctx.next_seq;   /* 窗口重开 */
        goto UPDATE_STATE;
    }

    if (spent_time >= (int32_t)(s_ctx.dec_config.csi_handle_time * 2) || buff_index->window >= s_ctx.window_ctx.handle_window) {   /* 时间/帧数达标 */
        if (buff_index->window < s_ctx.window_ctx.handle_window / 3) {   /* 帧数太少：不算 PCA */
            s_ctx.window_ctx.window_start_seq = s_ctx.window_ctx.next_seq;
            ESP_LOGW(TAG, "buff_index.window: %d, spent_time: %d, csi_handle_time: %d, handle_window: %d",
                     buff_index->window, spent_time, s_ctx.dec_config.csi_handle_time, s_ctx.window_ctx.handle_window);
            goto UPDATE_STATE;
        }
        ESP_LOGD(TAG, "buff_index.window: %d, time_tamp: %d, spent_time: %d, csi_handle_time: %d, handle_window: %d,buff_index-begin: %d, buff_index-end: %d",
                 buff_index->window, time_tamp, spent_time, s_ctx.dec_config.csi_handle_time, s_ctx.window_ctx.handle_window, buff_index->begin, buff_index->end);
        ret = ESP_OK;                                    /* 本批有效！ */
        s_ctx.window_ctx.window_start_seq += buff_index->window / 2;   /* 前移半窗（50% 重叠） */
    }

UPDATE_STATE:
    s_ctx.window_ctx.last_timestamp = s_ctx.csi_data_buff->timestamp[buff_index->end];   /* 更新上次处理时间 */
    s_ctx.window_ctx.next_seq++;         /* 本帧处理完毕，序号推进 */
    return ret;
}

/**
 * 分配振幅环形缓冲（指针数组+数据块合一的连续内存）
 * 布局：[buff_size 个 float*][buff_size × subcarrier_len 个 float]
 * 一次分配对 cache 友好，避免碎片。
 */
static void csi_prepare_amplitude(uint16_t subcarrier_len)
{
    size_t pointer_bytes = (size_t)s_ctx.window_ctx.buff_size * sizeof(float *);                 /* 指针区大小 */
    size_t data_bytes    = (size_t)s_ctx.window_ctx.buff_size * subcarrier_len * sizeof(float);  /* 数据区大小 */
    size_t total_bytes   = pointer_bytes + data_bytes;
    uint8_t *amplitude_block = RADAR_MALLOC_RETRY(total_bytes);   /* 一次申请（失败自动重试） */
    memset(amplitude_block, 0, total_bytes);

    s_ctx.csi_data_buff->amplitude = (float **)amplitude_block;   /* 前半：指针数组 */
    float *data_base = (float *)(amplitude_block + pointer_bytes); /* 后半：数据块 */
    for (uint32_t i = 0; i < s_ctx.window_ctx.buff_size; i++) {
        s_ctx.csi_data_buff->amplitude[i] = data_base + (i * subcarrier_len);   /* 每指针指向一行 */
    }
}

/**
 * 一帧 CSI 写入环形缓冲：解 I/Q → 振幅 → 存入
 *
 * I/Q 布局：
 *   INT16 模式：每子载波 4B = [imag_lo, imag_hi, real_lo, real_hi]，12bit 符号数
 *   INT8  模式：每子载波 2B = [imag, real]
 * step = sub_carrier_step_size：子载波抽取步长（2=隔一个取一个，减计算量）
 */
static void csi_write_frame_to_ring(const void *ltf_data, uint16_t subcarrier_len, wifi_csi_filtered_info_t *filtered_info)
{
    if (!s_ctx.csi_data_buff || !s_ctx.csi_data_buff->amplitude ||
            !s_ctx.csi_data_buff->seq_id || !s_ctx.csi_data_buff->timestamp) {   /* 缓冲未就绪 */
        return;
    }

    uint32_t write_index = s_ctx.window_ctx.next_seq % s_ctx.window_ctx.buff_size;   /* 环形写位置 */

    s_ctx.csi_data_buff->seq_id[write_index] = filtered_info->seq_id;                /* 记包序号 */
    s_ctx.csi_data_buff->timestamp[write_index] = filtered_info->rx_ctrl_info.timestamp / 1000;   /* us→ms */

    float *csi_data = s_ctx.csi_data_buff->amplitude[write_index];   /* 本帧目标行 */
    const uint8_t step = s_ctx.dec_config.sub_carrier_step_size;
    const uint8_t *data = (const uint8_t *)ltf_data;

    for (int i = 0; i < subcarrier_len; i++) {       /* 逐子载波：I/Q → 振幅 */
        if (filtered_info->data_type == WIFI_CSI_DATA_TYPE_INT16) {
            /* 12bit 紧凑格式：符号扩展 = 左移4算术右移4 */
            int16_t imag = ((int16_t)(((int16_t)data[i * step * 4 + 1]) << 12) >> 4 | (uint8_t)data[i * step * 4]) >> 4;
            int16_t real = ((int16_t)(((int16_t)data[i * step * 4 + 3]) << 12) >> 4 | (uint8_t)data[i * step * 4 + 2]) >> 4;
            csi_data[i] = my_hypotf(real, imag);     /* 振幅 = sqrt(real²+imag²) */
        } else {
            csi_data[i] = my_hypotf(((const int8_t *)ltf_data)[i * step * 2], ((const int8_t *)ltf_data)[i * step * 2 + 1]);   /* INT8: [imag, real] */
        }
    }
}

/**
 * ★ 任务1：CSI 预处理主循环 ★（消费回调入队的帧）
 * 取帧 → 切 LTF → 探测/校验子载波数 → 写环形缓冲 → 离群过滤
 * → 推进滑窗 →（攒够一批）把窗口索引发给特征任务
 */
static void csi_preprocessing_task(void *arg)
{
    wifi_csi_filtered_info_t *filtered_info = NULL;

    while (xQueueReceive(s_ctx.csi_info_queue, &filtered_info, portMAX_DELAY) && s_ctx.run_flag) {   /* 阻塞等帧 */
        void *ltf_data = NULL;
        uint16_t ltf_len = 0;
        uint16_t subcarrier_len = 0;
        if (esp_radar_get_ltf_data(filtered_info, &ltf_data, &ltf_len) != ESP_OK) {   /* 切出配置的 LTF 段 */
            goto FREE_MEM;
        }

        uint8_t component_bytes = (filtered_info->data_type == WIFI_CSI_DATA_TYPE_INT16) ? sizeof(int16_t) : sizeof(int8_t);   /* I/Q 每分量字节数 */
        subcarrier_len = (uint16_t)((ltf_len / (2U * component_bytes)) / s_ctx.dec_config.sub_carrier_step_size);   /* 子载波数 = 字节÷(2×分量字节)÷步长 */
        if (s_ctx.subcarrier_len == 0) {                /* 首帧：确定子载波数并分配缓冲 */
            s_ctx.subcarrier_len = subcarrier_len;
            csi_prepare_amplitude(subcarrier_len);
            ESP_LOGI(TAG, "First frame detected: type=%d, LTF length=%d, subcarrier length=%d, step size=%d, allocated buffer: %d x %d",
                     s_ctx.dec_config.ltf_type, ltf_len, subcarrier_len, s_ctx.dec_config.sub_carrier_step_size, s_ctx.window_ctx.buff_size, subcarrier_len);
        } else if (subcarrier_len != s_ctx.subcarrier_len) {   /* 子载波数变了（AP 换带宽/格式）→ 丢帧 */
            ESP_LOGE(TAG, "Frame length mismatch detected! Expected type=%d, subcarrier_len=%d, got=%d (LTF len=%d). Discarding frame.",
                     s_ctx.dec_config.ltf_type, s_ctx.subcarrier_len, subcarrier_len, ltf_len);
            goto FREE_MEM;
        }

        csi_write_frame_to_ring(ltf_data, subcarrier_len, filtered_info);   /* 写入环形缓冲 */
        if (s_ctx.dec_config.outliers_threshold > 0) {  /* 离群过滤（可配置关闭） */
            if (csi_outlier_filter_process(subcarrier_len) != ESP_OK) {
                goto FREE_MEM;
            }
        }

        csi_data_buff_index_t buff_index = {0};
        if (csi_window_update(&buff_index) == ESP_OK) { /* 滑窗攒够一批 */
            if (!s_ctx.csi_data_queue || xQueueSend(s_ctx.csi_data_queue, &buff_index, portMAX_DELAY) == pdFALSE) {   /* 发给特征任务（可阻塞：队列容量1） */
                ESP_LOGW(TAG, "The buffer is full");
            }
        }

FREE_MEM:
        RADAR_FREE(filtered_info);                      /* 无论走哪条路都释放本帧 */
    }

    ESP_LOGW(TAG, "csi_preprocessing_task exit");
    xEventGroupSetBits(s_ctx.task_exit_group, CSI_PREPROCESSING_EXIT_BIT);   /* 通知 stop 已退出 */
    vTaskDelete(NULL);                                  /* 自删除 */
}

/* ----------------------------------------------------------------------------
 * 训练（标定）：采集"无人背景"，生成 Presence 阈值
 * -------------------------------------------------------------------------- */

/**
 * 释放训练指纹库全部内存
 */
static void radar_calibrate_free_entries(radar_calibrate_t *cal)
{
    if (!cal) {
        return;
    }
    for (int i = 0; i < CSI_CORR_NUM; ++i) {    /* 逐条释放 10 条指纹 */
        if (cal->data[i]) {
            RADAR_FREE(cal->data[i]);
            cal->data[i] = NULL;
        }
    }
}

/**
 * 复位训练统计（不释放指纹库内存）
 * static_corr 初始化为 1：后面用"取更小值"方式收集静止参考。
 */
static void radar_calibrate_reset_stats(radar_calibrate_t *cal)
{
    if (!cal) {
        return;
    }

    cal->buff_size = 0;             /* 三点滑窗清零 */
    cal->none_corr_sum = 0.0f;      /* 背景统计清零 */
    cal->none_corr_count = 0.0f;
    cal->none_corr = 1.0f;          /* 当前背景 corr（1=无效初值） */
    cal->static_corr = 1.0f;        /* 静止参考（训练期往下压） */
    cal->subcarrier_len = 0;        /* 子载波数重新记录 */
}

/**
 * 训练开始（用户调，如 HMSCMD RESET_BASELINE）
 * 前提：现场必须静止！训练期间有人走动会污染背景样本。
 */
esp_err_t esp_radar_train_start(void)
{
    if (!s_radar_calibrate) {                       /* 首次：分配标定结构 */
        s_radar_calibrate = RADAR_MALLOC_RETRY(sizeof(radar_calibrate_t));
        memset(s_radar_calibrate, 0, sizeof(radar_calibrate_t));
        s_radar_calibrate->data_num = 0;
    }

    radar_calibrate_reset_stats(s_radar_calibrate); /* 统计复位（指纹库保留） */
    s_radar_calibrate->calibrate_status = RADAR_CALIBRATE_PROGRESS;   /* 进入训练态 */
    s_waveform_wander_last = 0.0f;                  /* wander 历史复位 */
    s_pca_subcarrier_buff_num = 0;                  /* PCA 环形索引复位（训练从干净窗口序列开始） */
    ESP_LOGI(TAG_TRAIN, "esp_radar_train_start");
    return ESP_OK;
}

/**
 * 训练作废（子载波数变了/用户清除）：释放指纹库，状态回 NO
 */
esp_err_t esp_radar_train_remove(void)
{
    if (!s_radar_calibrate) {
        return ESP_OK;
    }

    radar_calibrate_free_entries(s_radar_calibrate);    /* 释放 10 条指纹 */
    radar_calibrate_reset_stats(s_radar_calibrate);
    s_radar_calibrate->data_num = 0;
    s_radar_calibrate->calibrate_status = RADAR_CALIBRATE_NO;
    s_waveform_wander_last = 0.0f;
    ESP_LOGI(TAG_TRAIN, "esp_radar_train_remove");
    return ESP_OK;
}

/**
 * 训练结束：生成 Presence 的两个阈值
 *   wander_threshold = 1 - 背景corr均值  → 新版 train_wander_threshold
 *   jitter_threshold = 1 - static_corr   → 新版 train_jitter_threshold
 */
esp_err_t esp_radar_train_stop(float *wander_threshold, float *jitter_threshold)
{
    if (!s_radar_calibrate) {
        return ESP_ERR_NOT_SUPPORTED;               /* 没训练过 */
    }

    if (!s_radar_calibrate->data_num || s_radar_calibrate->none_corr_count == 0.0f) {   /* 没有效样本 */
        return ESP_ERR_INVALID_STATE;
    }

    s_radar_calibrate->calibrate_status = RADAR_CALIBRATE_COMPLETE;   /* 训练完成 */

    if (wander_threshold) {
        *wander_threshold = 1.0f - (s_radar_calibrate->none_corr_sum / s_radar_calibrate->none_corr_count);   /* 背景均值→阈值 */
    }
    if (jitter_threshold) {
        *jitter_threshold = 1.0f - s_radar_calibrate->static_corr;   /* 静止参考→阈值 */
    }

    ESP_LOGI(TAG_TRAIN, "esp_radar_train_stop");
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * 特征计算：PCA 指纹 → jitter → wander
 * -------------------------------------------------------------------------- */

/**
 * 对一个窗口调 PCA 产指纹（处理环形缓冲两段拼接）
 * 窗口跨"尾部→头部"回绕时拆两段：begin→缓冲尾(data_0)、缓冲头→窗口尾(data_1)。
 */
static esp_err_t csi_detection_compute_pca(uint16_t cols, const csi_data_buff_index_t *buff_index, float *pca_subcarrier_buff_current)
{
    float (*csi_data_0)[cols] = (float (*)[cols])s_ctx.csi_data_buff->amplitude[buff_index->begin];   /* 第一段首行（二维数组视图） */
    float (*csi_data_1)[cols] = (float (*)[cols])s_ctx.csi_data_buff->amplitude[0];                   /* 第二段从物理 0 开始 */

    uint16_t csi_data_0_len = (buff_index->begin <= buff_index->end) ? buff_index->window : (s_ctx.window_ctx.buff_size - buff_index->begin);   /* 不回绕=全窗口；回绕=begin 到末尾 */
    uint16_t csi_data_1_len = buff_index->window - csi_data_0_len;    /* 回绕部分长度 */

    if (csi_data_0_len == 0) {
        return ESP_FAIL;
    }

    return pca(cols, csi_data_0_len, csi_data_0, csi_data_1_len, csi_data_1, pca_subcarrier_buff_current);   /* → old_pca.c */
}

/**
 * ★ Motion 特征：jitter（波形抖动）★
 * jitter(corr 口径) = 当前指纹与滑窗内最近 (W-1) 个指纹的最大 |corr|
 * 直觉：窗口内两两都高度相关(corr≈1)→静止；有一个低相关→波形剧变→在动。
 * 取 max：只要有"一瞬间变化"就该报。输出前统一翻转为 (1-corr)。
 */
static void csi_detection_update_waveform_metrics(wifi_radar_info_t *radar_info, float *pca_subcarrier_buff_current, float **pca_subcarrier_buff, uint16_t cols)
{
    uint8_t move_buffer_size = s_ctx.dec_config.pca_window_size;    /* 指纹窗口大小（默认4） */
    if (s_pca_subcarrier_buff_num < move_buffer_size) {             /* 窗口未攒满：不算（数据不足） */
        return;
    }

    radar_info->waveform_jitter = 0.0f;                             /* 从 0 开始取 max */

    for (int i = 0; i < move_buffer_size - 1; ++i) {                /* 与窗口内其他指纹比 */
        float corr_val = fabs(corr(pca_subcarrier_buff_current, pca_subcarrier_buff[(s_pca_subcarrier_buff_num - 2 - i) % move_buffer_size], cols));   /* 从"上上帧"往回取（-2 跳过当前槽位） */

        if (radar_info->waveform_jitter < corr_val) {               /* 取最大相关 */
            radar_info->waveform_jitter = corr_val;
        }
    }

}

/**
 * ★ Presence 特征：wander（波形漂移）★
 * wander(corr 口径) = 当前指纹与训练背景指纹库(≤10条)的最大 |corr|
 * 直觉：与"任何一条无人背景"最像的 corr = 当前与无人状态的最高相似度。
 * 相似度高→没人；掉下去→环境变了（有人）。
 */
static void csi_detection_compute_wander(wifi_radar_info_t *radar_info, float *pca_subcarrier_buff_current, uint16_t cols)
{
    if (s_radar_calibrate->data_num == 0) {         /* 未训练：wander 无意义，置 1（翻转后=0） */
        radar_info->waveform_wander = 1.0f;
        return;
    }

    radar_info->waveform_wander = 0.0f;             /* 从 0 开始取 max */

    for (size_t i = 0; i < s_radar_calibrate->data_num && i < CSI_CORR_NUM; ++i) {   /* 遍历背景库（环形最近10条） */
        float *record = s_radar_calibrate->data[i % CSI_CORR_NUM];
        if (!record) {
            continue;
        }
        float corr_val = fabs(corr(record, pca_subcarrier_buff_current, cols));   /* 与该背景指纹相关 */
        if (radar_info->waveform_wander < corr_val) {   /* 取最大相似度 */
            radar_info->waveform_wander = corr_val;
        }
    }
}

/**
 * ★ 训练采样：判定该不该把指纹存入背景库 ★
 * 1) jitter 三点离群剔除：中间值比两侧低 0.005+ → 是尖峰间歇，丢弃
 * 2) 收集静止参考：static_corr 取训练期最小 jitter corr（最静时刻）
 * 3) 背景采样决策（滞后一拍逻辑）：
 *    - 上一拍 wander corr < 0.998（刚扰动过）→ 把"上上帧"指纹存入背景库
 *    - 上一拍持续稳定 → 累计背景 corr 统计（无人基线的样本）
 *    滞后一拍 + 取上上帧：确保存入的确实是稳定时刻的指纹。
 */
static void csi_training_collect_sample(wifi_radar_info_t *radar_info, float **pca_subcarrier_buff, uint16_t cols)
{
    uint8_t move_buffer_size = s_ctx.dec_config.pca_window_size;    /* 指纹窗口大小 */

    size_t idx_first = (s_radar_calibrate->buff_size + RADAR_BUFF_NUM - 2) % RADAR_BUFF_NUM;    /* 三点滑窗：前前帧 */
    size_t idx_second = (s_radar_calibrate->buff_size + RADAR_BUFF_NUM - 1) % RADAR_BUFF_NUM;   /* 前一帧 */
    size_t idx_third = s_radar_calibrate->buff_size % RADAR_BUFF_NUM;                           /* 当前帧 */

    s_radar_calibrate->waveform_jitter_buff[idx_third] = radar_info->waveform_jitter;   /* 存当前 jitter（corr 口径） */
    s_radar_calibrate->buff_size++;

    if (s_radar_calibrate->buff_size < RADAR_BUFF_NUM) {    /* 三点没凑齐：等 */
        return;
    }

    float first = s_radar_calibrate->waveform_jitter_buff[idx_first];    /* 三点值 */
    float second = s_radar_calibrate->waveform_jitter_buff[idx_second];
    float third = s_radar_calibrate->waveform_jitter_buff[idx_third];

    if ((first - second > RADAR_OUTLIERS_THRESHOLD) && (third - second > RADAR_OUTLIERS_THRESHOLD)) {   /* 两侧都比中间高 0.005+ → 中间是 V 型谷底=正常，两侧尖峰 */
        ESP_LOGI(TAG_TRAIN, "Jitter outlier detected: %.4f < %.4f, %.4f", second, first, third);
        return;                                             /* 丢弃本次采样 */
    }

    if (s_radar_calibrate->static_corr > radar_info->waveform_jitter) {   /* 静止参考取更稳定（更小）的 */
        s_radar_calibrate->static_corr = second;
    }

    if (s_waveform_wander_last < CSI_CORR_THRESHOLD) {      /* 上一拍环境有扰动（corr<0.998） */
        float *pca_prev = pca_subcarrier_buff[(s_pca_subcarrier_buff_num + move_buffer_size - 2) % move_buffer_size];   /* 取"上上帧"指纹（稳定帧） */
        if (pca_prev) {
            size_t index = s_radar_calibrate->data_num % CSI_CORR_NUM;   /* 背景库环形槽位 */

            if (!s_radar_calibrate->data[index]) {       /* 槽位首次使用先分配 */
                s_radar_calibrate->data[index] = RADAR_MALLOC_RETRY((size_t)cols * sizeof(float));
            }

            memcpy(s_radar_calibrate->data[index], pca_prev, (size_t)cols * sizeof(float));   /* 存入背景库 */
            s_radar_calibrate->data_num++;
            s_radar_calibrate->none_corr = 1.0f;         /* 本帧 wander 无效化 */
            radar_info->waveform_wander = 1.0f;          /* 翻转后=0：刚采样不报偏离 */

            ESP_LOGI(TAG_TRAIN, "Training sample collected: num=%zu, corr=%.4f",
                     s_radar_calibrate->data_num, s_waveform_wander_last);
        }
    } else {                                                /* 上一拍持续稳定 */
        s_radar_calibrate->none_corr = s_waveform_wander_last;
        if (s_waveform_wander_last < 0.99999f) {            /* corr<1 才有统计意义（1 是初值） */
            s_radar_calibrate->none_corr_sum += s_waveform_wander_last;   /* 累计背景 corr */
            s_radar_calibrate->none_corr_count += 1.0f;

            ESP_LOGI(TAG_TRAIN, "Training stats: sum=%.4f, count=%.0f, avg=%.4f",
                     s_radar_calibrate->none_corr_sum, s_radar_calibrate->none_corr_count,
                     s_radar_calibrate->none_corr_sum / s_radar_calibrate->none_corr_count);
        }
    }

    s_waveform_wander_last = radar_info->waveform_wander;   /* 滞后一拍保存供下次决策 */
}

/**
 * ★ 任务2：特征计算主循环 ★（消费窗口索引，产出 jitter/wander）
 * PCA 指纹 → jitter → wander → 训练采样 → 语义翻转 → 用户回调
 */
static void csi_detection_task(void *arg)
{
    uint8_t move_buffer_size = s_ctx.dec_config.pca_window_size;    /* 指纹窗口大小 */
    float **pca_subcarrier_buff = RADAR_MALLOC_RETRY(move_buffer_size * sizeof(float *));   /* 指纹环形缓冲（W 条） */
    memset(pca_subcarrier_buff, 0, move_buffer_size * sizeof(float *));
    uint16_t cols = 0;
    bool pca_buff_allocated = false;                /* 指纹缓冲惰性分配标记 */
    csi_data_buff_index_t buff_index = {0};

    while (xQueueReceive(s_ctx.csi_data_queue, &buff_index, portMAX_DELAY) && s_ctx.run_flag) {   /* 阻塞等窗口 */
        cols = s_ctx.subcarrier_len;
        if (!s_ctx.csi_data_buff->amplitude || cols == 0) {   /* 振幅缓冲未就绪 */
            ESP_LOGW(TAG_DETECTION, "CSI buffer not ready");
            continue;
        }

        if (!pca_buff_allocated) {                  /* 首次：按子载波数分配指纹缓冲 */
            for (int i = 0; i < move_buffer_size; ++i) {
                pca_subcarrier_buff[i] = RADAR_MALLOC_RETRY(cols * sizeof(float));
            }
            pca_buff_allocated = true;
            ESP_LOGI(TAG_DETECTION, "Allocated PCA subcarrier buffer: %d x %d", move_buffer_size, cols);
        }

        wifi_radar_info_t radar_info = {            /* 本窗口输出（1=最不稳定初值） */
            .waveform_jitter = 1,
            .waveform_wander = 1,
        };

        uint32_t timestamp_start = esp_log_timestamp();   /* 耗时统计起点 */

        float *pca_subcarrier_buff_current = pca_subcarrier_buff[s_pca_subcarrier_buff_num % move_buffer_size];   /* 当前指纹写入槽位（环形） */

        if (csi_detection_compute_pca(cols, &buff_index, pca_subcarrier_buff_current) != ESP_OK) {   /* 第1步：PCA 指纹 */
            ESP_LOGW(TAG_DETECTION, "PCA calculation failed");   /* 幂法不收敛：跳过本窗口 */
            continue;
        }
        s_pca_subcarrier_buff_num++;                /* 指纹计数+1（环形推进） */

        csi_detection_update_waveform_metrics(&radar_info, pca_subcarrier_buff_current, pca_subcarrier_buff, cols);   /* 第2步：jitter（对窗口内历史） */

        if (s_radar_calibrate) {                    /* 有训练结构才做 wander/训练 */
            if (s_radar_calibrate->subcarrier_len == 0) {    /* 训练首次见数据：记子载波数 */
                s_radar_calibrate->subcarrier_len = cols;
            } else if (s_radar_calibrate->subcarrier_len != cols) {  /* 子载波数变了→训练作废重训 */
                ESP_LOGW(TAG_TRAIN, "Subcarrier length changed from %u to %u, reset training data",
                         s_radar_calibrate->subcarrier_len, cols);
                esp_radar_train_remove();
                s_radar_calibrate->subcarrier_len = cols;
            }

            csi_detection_compute_wander(&radar_info, pca_subcarrier_buff_current, cols);   /* 第3步：wander（对背景库） */

            if (s_radar_calibrate->calibrate_status == RADAR_CALIBRATE_PROGRESS && s_pca_subcarrier_buff_num >= 2) {   /* 训练态且≥2条指纹（保证能取上上帧） */
                csi_training_collect_sample(&radar_info, pca_subcarrier_buff, cols);   /* 第4步：训练采样 */
            }
        }

        radar_info.waveform_wander = 1.0f - radar_info.waveform_wander;   /* 第5步：语义翻转！corr→偏离度（0=像背景） */
        radar_info.waveform_jitter = 1.0f - radar_info.waveform_jitter;   /*            corr→抖动度（0=稳定） */

        int32_t time_spent  = s_ctx.csi_data_buff->timestamp[buff_index.end] - s_ctx.csi_data_buff->timestamp[buff_index.begin];   /* 窗口时间跨度（诊断） */
        if (time_spent > 0) {
            ESP_LOGD(TAG_DETECTION, "det_time: %u/%u, free_heap: %d, wander: %f, jitter: %f, window: %d, begin: %d, end: %d, freq: %dHz",
                     time_spent, esp_log_timestamp() - timestamp_start, esp_get_free_heap_size(),
                     radar_info.waveform_wander, radar_info.waveform_jitter, buff_index.window,
                     buff_index.begin, buff_index.end, buff_index.window * 1000 / time_spent);
        }

        if (s_ctx.dec_config.wifi_radar_cb) {       /* 第6步：交给用户回调（FSM/应用层） */
            s_ctx.dec_config.wifi_radar_cb(s_ctx.dec_config.wifi_radar_cb_ctx, &radar_info);
        }
    }

    for (int i = 0; i < move_buffer_size; ++i) {    /* 退出清理：释放指纹缓冲 */
        if (pca_subcarrier_buff[i]) {
            RADAR_FREE(pca_subcarrier_buff[i]);
        }
    }
    RADAR_FREE(pca_subcarrier_buff);
    s_pca_subcarrier_buff_num = 0;
    ESP_LOGW(TAG_DETECTION, "csi_detection_task  exit");
    xEventGroupSetBits(s_ctx.task_exit_group, CSI_DETECTION_EXIT_BIT);   /* 通知 stop 已退出 */
    vTaskDelete(NULL);                                  /* 自删除 */
}

/* ----------------------------------------------------------------------------
 * 生命周期：启停 / 配置 / 初始化
 * -------------------------------------------------------------------------- */

/**
 * 启动雷达：分配缓冲/队列，起两个任务，算窗口参数
 */
esp_err_t esp_radar_start()
{
    if (s_ctx.run_flag) {               /* 已在跑：幂等返回 */
        return ESP_OK;
    }

    s_ctx.run_flag = true;
    s_csi_seq = 0;                      /* 帧序号从 0 重新计 */

    s_ctx.csi_data_buff = RADAR_MALLOC_RETRY(sizeof(csi_data_buff_t));   /* 分配环形缓冲结构 */
    memset(s_ctx.csi_data_buff, 0, sizeof(csi_data_buff_t));

    s_ctx.csi_info_queue = xQueueCreate(5, sizeof(void *));              /* 队列1：5 个指针槽（回调→预处理） */
    s_ctx.csi_data_queue = xQueueCreate(1, sizeof(csi_data_buff_index_t));   /* 队列2：1 个窗口索引（预处理→特征） */
    s_ctx.task_exit_group  = xEventGroupCreate();
    if (s_ctx.dec_config.csi_handle_time < s_ctx.csi_config.csi_recv_interval * (s_ctx.dec_config.pca_window_size)) {   /* 处理时间至少够一个完整指纹窗口 */
        ESP_LOGE(TAG, "csi_handle_time is too short, will set to %d", s_ctx.csi_config.csi_recv_interval * (s_ctx.dec_config.pca_window_size));
        s_ctx.dec_config.csi_handle_time = s_ctx.csi_config.csi_recv_interval * (s_ctx.dec_config.pca_window_size);   /* 自动放大 */
    }
    s_ctx.window_ctx.handle_window     = (s_ctx.dec_config.csi_handle_time / s_ctx.csi_config.csi_recv_interval) * 2;    /* 处理窗口 = 2 倍处理时间内的帧数 */
    s_ctx.window_ctx.buff_size         = (s_ctx.dec_config.csi_handle_time / s_ctx.csi_config.csi_recv_interval) * 2 + 20;   /* 缓冲 = 窗口 + 20 帧余量 */
    s_ctx.window_ctx.window_start_seq  = 0;
    s_ctx.window_ctx.next_seq          = 0;
    s_ctx.window_ctx.last_timestamp    = 0;

    ESP_LOGI(TAG, "[%s, %d] csi_recv_interval: %d, csi_handle_time: %d, csi_handle_window: %d, csi_handle_buffer: %d",
             __func__, __LINE__,
             s_ctx.csi_config.csi_recv_interval, s_ctx.dec_config.csi_handle_time,
             s_ctx.window_ctx.handle_window, s_ctx.window_ctx.buff_size);

    s_ctx.csi_data_buff->amplitude = NULL;      /* 振幅区首帧再分配（子载波数未知） */
    s_ctx.csi_data_buff->timestamp = RADAR_MALLOC_RETRY(s_ctx.window_ctx.buff_size * sizeof(uint32_t));   /* 时间戳数组 */
    memset(s_ctx.csi_data_buff->timestamp, 0, s_ctx.window_ctx.buff_size * sizeof(uint32_t));
    s_ctx.csi_data_buff->seq_id = RADAR_MALLOC_RETRY(s_ctx.window_ctx.buff_size * sizeof(uint32_t));      /* 序号数组 */
    memset(s_ctx.csi_data_buff->seq_id, 0, s_ctx.window_ctx.buff_size * sizeof(uint32_t));

    xTaskCreate(csi_detection_task, "csi_handle", 3 * 1024, NULL, s_ctx.dec_config.csi_handle_priority, NULL);       /* 任务2：特征计算 */
    xTaskCreate(csi_preprocessing_task, "csi_combine", 3 * 1024, NULL, s_ctx.dec_config.csi_combine_priority, NULL); /* 任务1：预处理 */

    return ESP_OK;
}

/**
 * 停止雷达：停任务、清队列、释放全部缓冲
 * 技巧：向两个队列发 NULL/空消息唤醒阻塞的任务让其退出循环。
 */
esp_err_t esp_radar_stop()
{
    csi_data_buff_index_t buff_index = {0};
    wifi_csi_filtered_info_t *filtered_info = NULL;
    s_ctx.run_flag = false;             /* 先翻标志：任务循环条件失效 */
    xQueueSend(s_ctx.csi_info_queue, &filtered_info, 0);    /* 发 NULL 唤醒预处理任务 */
    xQueueSend(s_ctx.csi_data_queue, &buff_index, 0);       /* 发空索引唤醒特征任务 */

    xEventGroupWaitBits(s_ctx.task_exit_group, CSI_DETECTION_EXIT_BIT | CSI_PREPROCESSING_EXIT_BIT,
                        pdTRUE, pdTRUE, portMAX_DELAY);     /* 等两个任务都退出 */

    if (s_ctx.csi_info_queue) {         /* 排空队列1残留的帧并释放 */
        while (xQueueReceive(s_ctx.csi_info_queue, &filtered_info, 0)) {
            if (filtered_info) {
                RADAR_FREE(filtered_info);
            }
        }
    }

    if (s_ctx.csi_data_queue) {         /* 排空队列2并释放环形缓冲 */
        while (xQueueReceive(s_ctx.csi_data_queue, &buff_index, 0)) ;

        if (s_ctx.csi_data_buff) {
            RADAR_FREE(s_ctx.csi_data_buff->amplitude);
            RADAR_FREE(s_ctx.csi_data_buff->timestamp);
            RADAR_FREE(s_ctx.csi_data_buff->seq_id);
            RADAR_FREE(s_ctx.csi_data_buff);
            s_ctx.csi_data_buff = NULL;
            s_ctx.subcarrier_len = 0;   /* 下次 start 重新探测子载波数 */
        }
        vQueueDelete(s_ctx.csi_data_queue);
        s_ctx.csi_data_queue = NULL;
    }

    if (s_ctx.csi_info_queue) {
        vQueueDelete(s_ctx.csi_info_queue);
        s_ctx.csi_info_queue = NULL;
    }

    vEventGroupDelete(s_ctx.task_exit_group);
    s_ctx.task_exit_group = NULL;
    s_ctx.init_flag = false;
    return ESP_OK;
}

/**
 * CSI 硬件采集配置：按芯片代际填 wifi_csi_config_t 并使能 CSI
 * C5/C6/C61 用新字段（含 HE），老芯片用旧字段。lltf_bit_mode 决定
 * 数据格式（12bit 紧凑 vs 8bit）。
 */
esp_err_t esp_radar_csi_init(esp_radar_csi_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    wifi_csi_config_t wifi_csi_config;
    s_ctx.lltf_bit_mode = false;
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    wifi_csi_config = (wifi_csi_config_t) {
        .enable                   = true,                       /* 使能 CSI */
        .acquire_csi_legacy       = config->acquire_csi_lltf,   /* LLTF（legacy 帧） */
        .acquire_csi_force_lltf   = config->acquire_csi_lltf,   /* 强制只收 LLTF */
        .acquire_csi_ht20         = config->acquire_csi_ht20,   /* HT20 帧 */
        .acquire_csi_ht40         = config->acquire_csi_ht40,   /* HT40 帧 */
        .acquire_csi_vht          = config->acquire_csi_vht,    /* VHT 帧 */
        .acquire_csi_su           = config->acquire_csi_su,     /* HE SU 帧 */
        .acquire_csi_mu           = config->acquire_csi_mu,     /* HE MU 帧 */
        .acquire_csi_dcm          = config->acquire_csi_dcm,    /* DCM 帧 */
        .acquire_csi_beamformed   = config->acquire_csi_beamformed, /* 波束成形帧 */
        .acquire_csi_he_stbc_mode = config->acquire_csi_he_stbc_mode,
        .val_scale_cfg            = config->val_scale_cfg,      /* 数值缩放 */
        .dump_ack_en              = config->dump_ack_en,        /* 是否也采 ACK 的 CSI */
        .reserved                 = 0
    };
    if (config->acquire_csi_lltf) {
        ESP_LOGW(TAG, "%s: LLTF collection is enabled, other collection settings will be ignored", CONFIG_IDF_TARGET);   /* C5/C61 开 LLTF 时其它选项被硬件忽略 */
    }
    s_ctx.lltf_bit_mode = config->acquire_csi_lltf;   /* LLTF=12bit 紧凑格式 */
#elif CONFIG_IDF_TARGET_ESP32C6
    wifi_csi_config = (wifi_csi_config_t) {           /* C6 无 force_lltf 字段 */
        .enable                 = true,
        .acquire_csi_legacy     = config->acquire_csi_lltf,
        .acquire_csi_ht20       = config->acquire_csi_ht20,
        .acquire_csi_ht40       = config->acquire_csi_ht40,
        .acquire_csi_su         = config->acquire_csi_su,
        .acquire_csi_mu         = config->acquire_csi_mu,
        .acquire_csi_dcm        = config->acquire_csi_dcm,
        .acquire_csi_beamformed = config->acquire_csi_beamformed,
        .acquire_csi_he_stbc    = config->acquire_csi_he_stbc_mode,
        .val_scale_cfg          = config->val_scale_cfg,
        .dump_ack_en            = config->dump_ack_en,
        .reserved               = 0
    };
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32
    wifi_csi_config = (wifi_csi_config_t) {           /* 老芯片字段完全不同 */
        .lltf_en                = config->lltf_en,            /* LLTF 使能 */
        .htltf_en               = config->htltf_en,           /* HT-LTF 使能 */
        .stbc_htltf2_en         = config->stbc_htltf2_en,     /* STBC 第二 HT-LTF */
        .ltf_merge_en           = config->ltf_merge_en,       /* 多段 LTF 合并 */
        .channel_filter_en      = config->channel_filter_en,  /* 信道滤波 */
        .manu_scale             = config->manu_scale,         /* 手动缩放 */
        .shift                  = config->shift,              /* 数值右移（防溢出） */
        .dump_ack_en            = config->dump_ack_en,
    };
#else
    ESP_LOGE(TAG, "CSI functionality is not supported for %s.", CONFIG_IDF_TARGET);
    return ESP_ERR_NOT_SUPPORTED;
#endif
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));    /* 混杂模式（收非本机帧的 CSI 需要） */
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&wifi_csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(esp_radar_csi_rx_cb, config->csi_filtered_cb_ctx));   /* 注册硬件回调 */
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));            /* 总开关打开 */

    memcpy(&s_ctx.csi_config, config, sizeof(esp_radar_csi_config_t));

    return ESP_OK;
}

/**
 * WiFi STA 初始化：信道/带宽/协议配置
 * C5 支持 5GHz（2.4G 信道≤13，5G 信道>13），老芯片强制 2.4G。
 * 感知场景必须关省电（WIFI_PS_NONE），否则 STA 打盹会漏帧。
 */
esp_err_t esp_radar_wifi_init(esp_radar_wifi_config_t *config)
{
    static bool s_wifi_inited = false;
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_wifi_inited) {               /* 事件循环/netif 只初始化一次 */
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        ESP_ERROR_CHECK(esp_netif_init());
        s_wifi_inited = true;
    }
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));      /* STA 模式 */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM)); /* 配置存 RAM（不落 flash） */
    if (!ADDR_IS_EMPTY(config->mac_address)) {              /* 可自定义本机 MAC */
        ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, config->mac_address));
    }

#if !CONFIG_IDF_TARGET_ESP32C5
    if (config->band_mode == WIFI_BAND_MODE_5G_ONLY || config->band_mode == WIFI_BAND_MODE_AUTO) {   /* 老芯片不支持 5G */
        ESP_LOGW(TAG, "%s does not support 5GHz band_mode, forcing to WIFI_BAND_MODE_2G_ONLY", CONFIG_IDF_TARGET);
        config->band_mode = WIFI_BAND_MODE_2G_ONLY;
    }
    if (config->protocols.ghz_5g != 0) {
        ESP_LOGW(TAG, "%s does not support 5GHz protocols, clearing 5GHz protocol configuration", CONFIG_IDF_TARGET);
        config->protocols.ghz_5g = 0;
    }
    if (config->bandwidths.ghz_5g != 0) {
        ESP_LOGW(TAG, "%s does not support 5GHz bandwidths, clearing 5GHz bandwidth configuration", CONFIG_IDF_TARGET);
        config->bandwidths.ghz_5g = 0;
    }
#endif
    esp_netif_create_default_wifi_sta();
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    ESP_ERROR_CHECK(esp_wifi_start());                  /* 新芯片：先 start 再配射频参数 */
    ESP_ERROR_CHECK(esp_wifi_set_band_mode(config->band_mode));
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &config->protocols));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &config->bandwidths));
    bool is_20mhz = (config->channel <= 13 && config->bandwidths.ghz_2g == WIFI_BW_HT20) ||
                    (config->channel > 13 && config->bandwidths.ghz_5g == WIFI_BW_HT20);   /* 2.4G 信道≤13 / 5G 信道>13 */
#else
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &config->protocols));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, config->bandwidths.ghz_2g));
    ESP_ERROR_CHECK(esp_wifi_start());                  /* 老芯片：先配置再 start */
    bool is_20mhz = (config->bandwidths.ghz_2g == WIFI_BW_HT20);
#endif

    if (is_20mhz) {                                     /* 20MHz 不允许次信道 */
        if (config->second_chan != WIFI_SECOND_CHAN_NONE) {
            ESP_LOGW(TAG, "20MHz bandwidth configured but secondary channel is set to %d, forcing to NONE (channel: %d)",
                     config->second_chan, config->channel);
        }
        ESP_ERROR_CHECK(esp_wifi_set_channel(config->channel, WIFI_SECOND_CHAN_NONE));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_channel(config->channel, config->second_chan));
    }
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));     /* 关省电：漏帧会毁掉时间连续性 */
    memcpy(&s_ctx.wifi_config, config, sizeof(esp_radar_wifi_config_t));
    return ESP_OK;
}

/**
 * ESPNOW 发射端初始化：让本机按固定速率向 peer 发空数据帧当 CSI 探测源
 * （感知需要持续 CSI 流；没有 AP 主动发帧时，自己发最可控）
 */
esp_err_t esp_radar_espnow_init(esp_radar_espnow_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx.wifi_config.channel == 0) {       /* 信道未配置：先跑 wifi_init */
        ESP_LOGW(TAG, "channel is not set, please first run esp_radar_wifi_init");
        return ESP_ERR_INVALID_STATE;
    }
    esp_now_peer_info_t peer = {
        .channel   = s_ctx.wifi_config.channel, /* peer 在同信道 */
        .ifidx     = WIFI_IF_STA,
        .encrypt   = false,                     /* 不加密（省 CPU） */
    };
    memcpy(peer.peer_addr, config->peer_addr, sizeof(peer.peer_addr));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)config->pmk));
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    wifi_phy_mode_t phy_mode = 0;
    if (s_ctx.wifi_config.channel <= 13) {      /* 2.4G：按协议选 PHY 模式 */
        switch (s_ctx.wifi_config.protocols.ghz_2g) {
        case WIFI_PROTOCOL_11B:
            ESP_LOGW(TAG, "11b not supported csi");     /* 11b 没有 CSI */
            phy_mode = WIFI_PHY_MODE_11B;
            break;
        case WIFI_PROTOCOL_11G:
            phy_mode = WIFI_PHY_MODE_11G;
            break;
        case WIFI_PROTOCOL_11N:
            if (s_ctx.wifi_config.bandwidths.ghz_2g == WIFI_BW_HT20) {
                phy_mode = WIFI_PHY_MODE_HT20;
            } else {
                phy_mode = WIFI_PHY_MODE_HT40;
            }
            break;
        case WIFI_PROTOCOL_11AX:
            phy_mode = WIFI_PHY_MODE_HE20;      /* 11ax 用 HE20 */
            break;
        }
    } else {                                    /* 5G：只有 a/ac/ax */
        switch (s_ctx.wifi_config.protocols.ghz_5g) {
        case WIFI_PROTOCOL_11A:
        case WIFI_PROTOCOL_11AC:
            if (s_ctx.wifi_config.bandwidths.ghz_5g == WIFI_BW_HT20) {
                phy_mode = WIFI_PHY_MODE_HT20;
            } else {
                phy_mode = WIFI_PHY_MODE_HT40;
            }
            break;
        case WIFI_PROTOCOL_11AX:
            phy_mode = WIFI_PHY_MODE_HE20;
            break;
        }
    }

    esp_now_rate_config_t rate_config = {       /* 固定发射速率：CSI 子载波布局与速率绑定，速率变则子载波数变 */
        .phymode = phy_mode,
        .rate = config->rate,
        .ersu = false,
        .dcm = false
    };
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr, &rate_config));
#else
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(ESP_IF_WIFI_STA, config->rate));   /* 老芯片：简单 API */
#endif
    memcpy(&s_ctx.espnow_config, config, sizeof(esp_radar_espnow_config_t));
    return ESP_OK;
}

/**
 * 读取当前全部配置（Wi-Fi/CSI/ESPNOW/算法）
 */
esp_err_t esp_radar_get_config(esp_radar_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&config->wifi_config, &s_ctx.wifi_config, sizeof(esp_radar_wifi_config_t));
    memcpy(&config->csi_config, &s_ctx.csi_config, sizeof(esp_radar_csi_config_t));
    memcpy(&config->espnow_config, &s_ctx.espnow_config, sizeof(esp_radar_espnow_config_t));
    memcpy(&config->dec_config, &s_ctx.dec_config, sizeof(esp_radar_dec_config_t));
    return ESP_OK;
}

/**
 * WiFi 重新初始化（运行中改信道等场景）
 */
esp_err_t esp_radar_wifi_reinit(esp_radar_wifi_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_wifi_stop();                    /* 停 WiFi */
    esp_wifi_deinit();                  /* 反初始化 */
    esp_err_t ret = esp_radar_wifi_init(config);   /* 按新配置重来 */
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

/**
 * 运行中热更新配置：逐项 memcmp，变了哪项就重 init 哪项；
 * 之前在跑的话 stop 后再 start。
 */
esp_err_t esp_radar_change_config(esp_radar_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_radar_wifi_config_t *wifi_config = &config->wifi_config;
    esp_radar_csi_config_t *csi_config = &config->csi_config;
    esp_radar_espnow_config_t *espnow_config = &config->espnow_config;
    esp_radar_dec_config_t *dec_config = &config->dec_config;
    bool need_start = false;
    if (s_ctx.run_flag) {               /* 正在跑：先停，改完再启 */
        esp_radar_stop();
        need_start = true;
    }
    if (wifi_config && memcmp(wifi_config, &s_ctx.wifi_config, sizeof(esp_radar_wifi_config_t)) != 0) {   /* WiFi 配置变了 */
        esp_radar_wifi_reinit(wifi_config);
    }
    if (espnow_config && memcmp(espnow_config, &s_ctx.espnow_config, sizeof(esp_radar_espnow_config_t)) != 0) {   /* ESPNOW 配置变了 */
        esp_radar_espnow_init(espnow_config);
    }
    if (csi_config && memcmp(csi_config, &s_ctx.csi_config, sizeof(esp_radar_csi_config_t)) != 0) {   /* CSI 配置变了 */
        esp_radar_csi_init(csi_config);
    }
    if (dec_config && memcmp(dec_config, &s_ctx.dec_config, sizeof(esp_radar_dec_config_t)) != 0) {   /* 算法配置变了 */
        esp_radar_dec_init(dec_config);
    }
    if (need_start) {
        esp_radar_start();
    }
    return ESP_OK;
}

/**
 * 算法参数初始化：拷贝配置 + 窗口大小下限保护
 */
esp_err_t esp_radar_dec_init(esp_radar_dec_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx.init_flag) {              /* 防重复初始化 */
        ESP_LOGW(TAG, "esp_radar already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    s_ctx.init_flag = true;
    memcpy(&s_ctx.dec_config, config, sizeof(esp_radar_dec_config_t));
    if (s_ctx.dec_config.pca_window_size < RADAR_PCA_WINDOW_DEFAULT) {   /* 窗口太小没意义 */
        ESP_LOGW(TAG, "pca_window_size < 2, fallback to default: %d", RADAR_PCA_WINDOW_DEFAULT);
        s_ctx.dec_config.pca_window_size = RADAR_PCA_WINDOW_DEFAULT;     /* 回退默认 4 */
    }

#if WIFI_CSI_PHY_GAIN_ENABLE
    esp_csi_gain_ctrl_reset_rx_gain_baseline();   /* 增益基线复位 */
#endif
    return ESP_OK;
}

/**
 * 一站式初始化：WiFi + ESPNOW + CSI + 算法参数
 */
esp_err_t esp_radar_init(esp_radar_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_radar_wifi_init(&config->wifi_config);
    esp_radar_espnow_init(&config->espnow_config);
    esp_radar_csi_init(&config->csi_config);
    esp_radar_dec_init(&config->dec_config);
    return ESP_OK;
}

/**
 * 反初始化：关 CSI、释放训练数据、清空全局上下文
 */
esp_err_t esp_radar_deinit()
{
    ESP_ERROR_CHECK(esp_wifi_set_csi(false));   /* 关 CSI 硬件开关 */
    if (s_radar_calibrate) {
        radar_calibrate_free_entries(s_radar_calibrate);   /* 释放背景指纹库 */
        RADAR_FREE(s_radar_calibrate);
    }
    s_waveform_wander_last = 0.0f;
    memset(&s_ctx, 0, sizeof(s_ctx));           /* 全局上下文清零 */

    return ESP_OK;
}
