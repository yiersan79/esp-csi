/* ============================================================================
 * 【逐行注释版】esp_radar_motion_dec.h —— 本地开源替代组件的 API 头文件
 * ----------------------------------------------------------------------------
 * 本组件替代闭源预编译库 espressif__esp_radar_motion_dec，
 * 三个 API 与闭源库签名完全一致（同名同参），项目无需改动调用方：
 *   compute()        PCA 指纹：CSI 窗口 → 每子载波一个指纹值
 *   wander_compute() Presence 特征：指纹 vs 背景指纹的偏离度（0=像背景）
 *   compute_jitter() Motion 特征：指纹 vs 窗口内历史的抖动度（0=稳定）
 * ============================================================================
 */
#pragma once                /* 防头文件重复包含（MSVC/GCC 通用写法） */

#include "esp_err.h"        /* esp_err_t / ESP_OK / ESP_FAIL 等错误码 */
#include <stdint.h>         /* uint8_t / uint16_t / uint32_t */
#include <stddef.h>         /* size_t */

#ifdef __cplusplus          /* C++ 兼容：按 C 链接导出符号 */
extern "C" {
#endif

/**
 * @brief 计算一个 CSI 时间窗口的 PCA 指纹（Motion/Presence 共同的输入）
 *
 * @param cols    子载波数（= 输出指纹长度 = 组矩阵后的行数）
 * @param row_0   第一段帧数（环形缓冲 begin→缓冲尾）
 * @param data_0  第一段数据 [row_0][cols]，每行 = 一帧的子载波振幅
 * @param row_1   第二段帧数（环形缓冲回绕部分 头→end）
 * @param data_1  第二段数据 [row_1][cols]
 * @param output  输出指纹 [cols]，每子载波一个值
 * @return ESP_OK 成功；ESP_FAIL 幂法未收敛/零矩阵；ESP_ERR_NO_MEM 内存不足
 */
esp_err_t esp_radar_motion_dec_compute(uint32_t cols,
                                       uint32_t row_0, const float data_0[row_0][cols],
                                       uint32_t row_1, const float data_1[row_1][cols],
                                       float output[cols]);

/**
 * @brief 计算波形 wander（Presence 特征）
 *
 * 内部：wander = 1 − |Pearson corr(a, b)|
 *
 * @param a    向量 A（通常是当前 PCA 指纹）
 * @param b    向量 B（通常是训练期背景指纹）
 * @param len  向量长度（= 子载波数）
 * @return float 偏离度 [0.0, 1.0]：0=与背景完全一致（无人），越大=偏离越远
 */
float esp_radar_motion_dec_wander_compute(const float *a, const float *b, size_t len);

/**
 * @brief 计算波形 jitter（Motion 特征）
 *
 * 内部：jitter = 1 − max|corr(当前指纹, 窗口内历史指纹)|
 *
 * @param current      当前指纹 [cols]
 * @param history      历史指纹环形缓冲，history[i] 为一条指纹 [cols]
 * @param cols         子载波数
 * @param window_size  环形缓冲槽位数（指纹窗口大小）
 * @param buff_num     当前已产出的指纹总数（用于推算 current 所在槽位）
 * @return float 抖动度 [0.0, 1.0]：0=窗口内波形稳定（静止），越大=剧变（有人动）；
 *               数据不足（历史少于 1 条）时返回 0.0（保守输出）
 */
float esp_radar_motion_dec_compute_jitter(float *current,
                                          float **history,
                                          uint16_t cols,
                                          uint8_t window_size,
                                          uint32_t buff_num);

#ifdef __cplusplus
}
#endif
