# ESP Wi-Fi Sensing 算法移植 Amlogic 平台 — 详细设计文档

- 版本：v1.0
- 日期：2026-10-14
- 目标平台：Amlogic Linux 电视盒子 + 自研 Wi-Fi 芯片（CSI 已可从驱动导出）
- 源工程：`esp-csi/examples/esp-radar/wifi_sensing_demo`
- 状态：待评审

---

## 1. 目标与范围

将 ESP `esp_wifi_sensing` 的 Wi-Fi 感知算法（运动检测 + 存在检测 + FSM 状态机）移植到
Amlogic Linux 电视盒子，以**用户态守护进程**方式运行，输出运动/存在事件给盒子主业务。

范围：

- 在范围内：CSI 采集适配、核心检测算法、FSM 状态机、事件输出、离线验证工具。
- 不在范围内：Amlogic Wi-Fi 芯片/驱动的 CSI 导出功能本身（已具备）；ESP 侧改动。

关键前提（已确认）：

1. 自研 Wi-Fi 芯片驱动**已提供 CSI 导出接口**（vendor cmd / netlink / debugfs 之一）。
2. Amlogic 盒子为 Linux 用户态环境（可用 pthread / syslog / clock_gettime 等 POSIX API）。
3. 核心算法 `esp_radar_motion_dec` 为 ESP 闭源预编译库（仅 Xtensa/RISC-V），**必须重写**。

---

## 2. 源工程架构与数据链路（逆向梳理）

ESP 侧算法栈自上而下：

```
应用层   app_main.c / led_control / web_serial_monitor
编排层   esp_wifi_sensing.c    —— FSM 状态机、baseline、debounce、presence、ping
雷达层   esp_radar.c           —— CSI 环形缓冲、训练管线、增益补偿、子载波重建
算法层   esp_radar_motion_dec  —— 闭源 .a：compute / wander / jitter 三个函数
增益层   esp_csi_gain_ctrl     —— AGC/FFT gain 基线与补偿
数据源   ESP Wi-Fi 驱动        —— esp_wifi_set_csi_rx_cb + wifi_csi_info_t
```

完整数据链路（已对照源码确认）：

```
1. Wi-Fi 收包 → wifi_csi_info_t（rx_ctrl 元数据 + CSI payload）
2. esp_radar_csi_rx_cb
   ├─ esp_radar_extract_rx_ctrl_info：取 timestamp/channel/rssi/noise_floor/agc_gain/fft_gain
   └─ esp_radar_rebuild_csi_data：按 LTF 类型(LLTF/HT-LTF/HE-LTF)过滤出有效子载波
3. esp_csi_gain_ctrl_compensate_rx_gain：用 gain 基线做幅度补偿（可选）
4. 入队 → csi_detection_task 消费一个滑动窗口：
   ├─ esp_radar_motion_dec_compute(cols, rows0, data0, rows1, data1, out[cols])
   │     对窗口内两段 CSI 幅度做差分 → 每子载波“运动特征”
   ├─ esp_radar_motion_dec_compute_jitter(current, history, cols, window, n)
   │     当前特征 vs 历史窗口 → waveform_jitter（越小越稳定）★运动主特征
   └─ esp_radar_motion_dec_wander_compute(template, current, cols)
         当前特征 vs 训练模板 → waveform_wander（越小越相似）★存在/训练特征
5. 回调 radar_cb_ex → FSM 侧：
   ├─ jitter_value = waveform_jitter
   ├─ wander_value = waveform_wander
   └─ raw_feature = log1pf(jitter*1000) * 1000    （log 量化，送入 FSM raw 环形缓冲）
6. FSM 每 tick（hms_preprocess_channel + hms_process_channel_state）
   → 输出 ACTIVE / INACTIVE 事件
```

> 结论：`esp_wifi_sensing.c`（FSM）与 `esp_radar.c`（缓冲/训练）是开源 C，可整体移植；
> 只有 `esp_radar_motion_dec` 三个函数需要重写，且它们的输入/输出语义在头文件与调用点
> 中完整可推导。

---

## 3. 目标架构（Amlogic Linux）

```
┌─────────────────────────────────────────────────────────────────┐
│  Amlogic Linux 用户态守护进程  wifi_sensingd                     │
│                                                                 │
│  csi_adapter   （平台相关，唯一接触 Wi-Fi 驱动的模块）            │
│      │  vendor cmd / netlink / debugfs / ioctl                  │
│      ▼                                                          │
│  csi_frame_t 统一帧 ──► 环形缓冲 (csi_ring)                      │
│                              │                                  │
│  algo 层（纯 C + POSIX，可 x86 离线编译）                        │
│   ├─ motion_dec   （重写闭源三函数）                             │
│   ├─ radar_pipe   （esp_radar.c 移植：窗口/训练/增益）            │
│   └─ fsm          （esp_wifi_sensing.c 移植：状态机）            │
│                              │                                  │
│  event 层 ──► 运动/存在事件 ──► 主业务 / LED / 日志 / 诊断 JSON   │
└─────────────────────────────────────────────────────────────────┘
```

设计原则：

- **数据源解耦**：只有 `csi_adapter` 知道驱动细节；算法层只认识 `csi_frame_t`。
- **可离线验证**：algo 层不依赖任何 Amlogic 硬件，可在 x86 主机编译成回放工具。
- **多信道**：延续 ESP 的 per-peer 模型，一条 FSM 句柄管理多条信道（AP + 若干参考对端）。

---

## 4. 接口定义

### 4.1 统一 CSI 帧

```c
#define CSI_SUB_MAX  256

typedef enum {
    CSI_SRC_LINUX_NETLINK = 0,   /* 驱动 netlink 导出 */
    CSI_SRC_VENDOR_CMD,          /* cfg80211 vendor command */
    CSI_SRC_DEBUGFS,             /* debugfs 文件读 */
    CSI_SRC_IOCTL,               /* 私有 ioctl */
} csi_source_kind_t;

typedef struct {
    uint8_t  mac[6];          /* 对端 MAC（peer） */
    uint32_t seq;             /* 帧序号 */
    uint64_t tstamp_us;       /* 单调时钟时间戳（CLOCK_MONOTONIC） */
    uint8_t  channel;         /* 主信道 */
    int8_t   secondary_channel;
    int8_t   rssi;
    int8_t   noise_floor;
    uint8_t  agc_gain;        /* 驱动能提供则填，否则 0 */
    int8_t   fft_gain;        /* 同上 */
    uint16_t subcarrier_num;  /* 有效子载波数 */
    int8_t   ltf_type;        /* LLTF/HT-LTF/HE-LTF，对应有效字段 */
    float    amp[CSI_SUB_MAX];   /* 幅度（已按 int8/int16 归一化到浮点） */
    float    phase[CSI_SUB_MAX]; /* 相位（可空） */
} csi_frame_t;
```

> 字段对齐 ESP 的 `wifi_csi_info_t` + `wifi_pkt_rx_ctrl_t` 语义，方便后续算法逻辑
> 一比一对照。若驱动不提供 gain/noise_floor，则增益补偿步骤整体关闭（见 5.4）。

### 4.2 CSI 采集适配器 API

```c
typedef int (*csi_frame_cb_t)(const csi_frame_t *frame, void *ctx);

typedef struct {
    csi_source_kind_t kind;
    csi_frame_cb_t    cb;          /* 收到一帧时回调 */
    void             *cb_ctx;
    uint32_t          rx_depth;    /* 内核→用户态缓冲深度 */
} csi_adapter_cfg_t;

int  csi_adapter_init(const csi_adapter_cfg_t *cfg);   /* 打开驱动接口、注册回调 */
int  csi_adapter_start(void);                          /* 开始接收 */
int  csi_adapter_stop(void);
void csi_adapter_deinit(void);
```

平台相关实现按 `kind` 选择：netlink 用 `socket(AF_NETLINK,...)` + 专用 family；
vendor cmd 用 `nl80211` vendor command 或驱动私有 netlink；debugfs 用定时读文件。

### 4.3 算法层 API（对齐 ESP 原语）

```c
/* —— motion_dec（重写，语义对齐 esp_radar_motion_dec.h） —— */
int motion_dec_compute(uint32_t cols,
                       uint32_t row_0, const float data_0[][cols],
                       uint32_t row_1, const float data_1[][cols],
                       float output[cols]);
float motion_dec_wander_compute(const float *a, const float *b, size_t len);
float motion_dec_compute_jitter(const float *current, float *const *history,
                                uint16_t cols, uint8_t window_size, uint32_t buff_num);

/* —— radar_pipe（对齐 esp_radar.c 核心） —— */
int  radar_pipe_feed(const csi_frame_t *frame);              /* 幅度入环形缓冲 */
void radar_pipe_set_decode_cb(void (*cb)(void*, const uint8_t mac[6],
                                         float jitter, float wander), void *ctx);
int  radar_pipe_train_start(const uint8_t mac[6]);
int  radar_pipe_train_stop(const uint8_t mac[6], float *wander_thr, float *jitter_thr);

/* —— fsm（对齐 esp_wifi_sensing.h 公开 API，保持同名便于对照） —— */
int fsm_create(const fsm_config_t *cfg, fsm_handle_t *h);
int fsm_add_channel(fsm_handle_t h, const uint8_t mac[6]);
int fsm_update_data(fsm_handle_t h, const uint8_t mac[6], uint32_t raw_feature);
int fsm_register_event_cb(fsm_handle_t h, fsm_event_t ev, fsm_event_cb_t cb, void *ctx);
int fsm_control(fsm_handle_t h, fsm_ctrl_t ctrl);            /* START/STOP/RESET_BASELINE */
int fsm_get_channel_diag(fsm_handle_t h, const uint8_t mac[6], fsm_channel_diag_t *d);
```

### 4.4 事件输出

```c
typedef enum {
    SENSING_EVENT_INACTIVE = 0,
    SENSING_EVENT_ACTIVE,
} sensing_event_t;

typedef void (*sensing_event_cb_t)(const uint8_t mac[6], sensing_event_t ev, void *ctx);
```

同时输出诊断 JSON（对齐 demo 的 `HMS:` 协议），便于用同一套浏览器面板或调试工具对拍。

---

## 5. 模块分解与移植映射

### 5.1 模块总览

| 模块 | 源 | 平台依赖 | 改动量 |
|------|----|---------|--------|
| `csi_adapter` | 新写 | Linux 驱动接口 | 小（对接一个接口） |
| `csi_ring` | 新写（参考 esp_radar 环形缓冲） | 无 | 小 |
| `motion_dec` | **重写**（闭源 .a） | 无 | 中（需对拍） |
| `radar_pipe` | 移植 esp_radar.c | FreeRTOS→pthread | 中 |
| `fsm` | 移植 esp_wifi_sensing.c | FreeRTOS→pthread，去 lwip ping | 中 |
| `gain_ctrl` | 移植 esp_csi_gain_ctrl | gain 提取换成驱动字段 | 小 |
| `event_out` / `diag` | 移植 demo 的 web_serial_monitor 思想 | 无 | 小 |
| `replay_tool`（x86） | 新写 | 无 | 小 |

### 5.2 motion_dec 重写规格（关键）

三个函数的行为规格（供重写 + 对拍）：

1. **`motion_dec_compute`**
   - 输入：`cols` 子载波数；两段连续 CSI 幅度 `data_0[row_0][cols]`、`data_1[row_1][cols]`。
   - 输出：`output[cols]`，每子载波一个“运动特征值”。
   - 语义：对两个相邻时间窗的幅度做差分/能量型度量，值越大表示该子载波变化越剧烈。
   - 验证：构造已知差异的数据（如 data_1 = data_0 + 常数），确认输出单调、非负。

2. **`motion_dec_wander_compute(a, b, len)`**
   - 语义：向量相似度，返回 `[0,1]`，**越小越相似/越稳定**。
   - 用途：当前特征 vs 训练模板（存在检测 + 训练样本判定）。

3. **`motion_dec_compute_jitter(current, history, cols, window, buff_num)`**
   - 语义：当前特征 vs 历史窗口的稳定性度量，返回 `[0,1]`，**越小越稳定**；
     历史不足时返回 0。
   - 用途：`waveform_jitter`，是运动检测的主特征，最终经 `log1pf(jitter*1000)*1000`
     量化后进入 FSM。

> 重写建议：先用**幅度差分 + 归一化 + 滑窗方差/均值**类基线实现，重点保证单调性与
> `[0,1]` 值域，再用 6.2 的金标准数据迭代逼近 ESP 输出趋势；不必逐位一致，只需
> 检测时刻与阈值余量一致。

### 5.3 radar_pipe 移植要点

- 将 `esp_radar.c` 的 `csi_info_queue` / `csi_data_queue`（FreeRTOS 队列）换成
  有界环形缓冲 + `pthread_cond`。
- 滑动窗口参数：`handle_window = csi_handle_time / csi_recv_interval * 2`；
  `buff_size = handle_window + 20`。移植后按驱动实际帧率重设 `csi_recv_interval`。
- 训练管线（train_start/stop/remove）整体照搬，含：
  - 抖动离群点判定（`RADAR_OUTLIERS_THRESHOLD`）；
  - `static_wander`（静态抖动参考）作为 **jitter_threshold**；
  - wander > 阈值时 `COLLECT_SAMPLE` 存模板，否则 `ACCUMULATE_BACKGROUND` 累计
    `none_wander_sum/count`，其均值作为 **wander_threshold**。

### 5.4 gain_ctrl 移植要点

- 数学部分（基线中位数、补偿因子计算）照搬 `esp_csi_gain_ctrl.c`。
- `esp_csi_gain_ctrl_get_rx_gain` 对 ESP `wifi_pkt_rx_ctrl_t` 结构体的字段提取，
  改为从 `csi_frame_t.agc_gain/fft_gain` 读取。
- 若驱动**不提供 gain**：关闭补偿（`csi_compensate_en=false`），幅度直接使用；
  但这会增加环境差异，标定阶段需要更大的阈值余量。

### 5.5 FSM 移植要点（算法规格）

FSM 是移植核心，逻辑照搬 `esp_wifi_sensing.c`，关键参数与状态如下。

**简配参数 → 运行时参数展开**（`hms_expand_channel_config`）：

| 运行时参数 | 取值规则 |
|-----------|---------|
| `smooth_coef` | lerp(0.06, 0.38, sensitivity) |
| `baseline_coef` | 0.020 固定 |
| `gold_baseline_coef` | baseline_coef × 0.05 |
| `noise_p` / `noise_n` | 0.30 / 0.21 |
| `noise_attack_coef` / `noise_release_coef` | 0.18 / 0.03 |
| `threshold_p` | lerp(0.90, 0.20, sensitivity) |
| `hysteresis_active` / `hysteresis_inactive` | 0.08 / 0.28 |
| `noise_threshold_gain_enter` / `_exit` | 0.18 / 0.08 |
| `debounce_active` | confirm_count（默认 2） |
| `debounce_inactive` | confirm_count × 2 |
| `active_filter_ms` | 300 |
| 固定点缩放 `SCALE_FACTOR` | 1000 |

**初始化三阶段**：

```
START      → 2000ms 窗口内求平均（≥50 包）→ 得到 initial baseline → GOLD_FOUND
GOLD_FOUND → 连续落在噪声带内 → STABLE；连续 reset_calibration(6) 次越界 → 回 START
STABLE     → 进入正常检测
```

**每 tick 预处理**（`hms_preprocess_channel`）：

1. raw 特征 ×1000 固定点缩放。
2. 一阶平滑：`smooth += (raw - smooth) * smooth_coef`；下降时系数 ×2（快速回落）。
3. 双基线更新：
   - `baseline`（工作基线）：向 smooth 缓慢跟随，`baseline_coef`（背景态）/×0.25（非背景态）；
   - `gold_baseline`（长期中心）：跟随 baseline，系数 `gold_baseline_coef`（背景态）/×0.25；
   - 越界量被 clamp 在噪声带限内（`baseline_clamp_gain=0.35`）。
4. 噪声电平自适应：`noise += (|smooth-baseline| - noise) * coef`，上升用 attack、下降用 release。

**状态判定**（`hms_process_channel_state`）：

```
diff = smooth - baseline
enter_thr = threshold_p * ref * (1 + hysteresis_active) + noise * gain_enter
exit_thr  = threshold_p * ref * (1 - hysteresis_inactive) + noise * gain_exit

IDLE / DEBOUNCE_ACTIVE：diff > enter_thr 连续 debounce_active 次 → ACTIVE
ACTIVE / DEBOUNCE_INACTIVE：diff < exit_thr 连续 debounce_inactive 次 → INACTIVE
方向翻转 → debounce 清零；active_filter_ms 内不判 INACTIVE（最小保持）
jitter < active_jitter_min → 强制 INACTIVE
```

**存在检测（presence）**：

- 用 `waveform_wander` 历史（25 窗）做 trimmed mean（去掉较大 50% 后平均）；
- 样本 ≥5 才 ready；
- `someone_threshold = train_wander_threshold × lerp(1.0, 30.0, presence_sensitivity)`；
- 平均 wander 高于阈值 → `someone` 状态。

**可去掉的依赖**：lwip `ping_sock` 辅助采样（`ping_router_start`）在 Amlogic 上
默认关闭；若需要保持 CSI 流量，用 raw socket ICMP 或让驱动持续发包代替。

---

## 6. 验证计划

### 6.1 分阶段验证

| 阶段 | 验证内容 | 通过标准 |
|------|---------|---------|
| V0 | `csi_adapter` 能持续收到 CSI 并 dump | 稳定帧率 ≥ 目标值，字段完整 |
| V1 | `motion_dec` 三函数单元测试 | 值域、单调性、边界（历史不足返回 0） |
| V2 | 离线回放：x86 跑 algo，喂 ESP 金标准 CSI | jitter/wander 曲线与 ESP 输出同趋势 |
| V3 | 离线回放：喂 Amlogic 实测 CSI | 检测时刻与人工标注一致 |
| V4 | 真机实时 + 标定 | 漏检/误检率达标，延迟达标 |
| V5 | 集成 | 事件正确送达主业务，重启自恢复 |

### 6.2 金标准数据采集

1. ESP32-C5 跑现有固件，在受控场景（无人 / 静止有人 / 走动 / 进出）录制：
   - 原始 CSI（幅度，保存为 CSV/二进制）；
   - 每帧的 `jitter_value`、`wander_value`；
   - ACTIVE/INACTIVE 事件时间戳。
2. Amlogic 盒子放**同一位置、同一环境**录同场景 CSI。
3. 对拍：把两路数据喂给重写后的 algo，对比：
   - jitter/wander 曲线相关系数与趋势；
   - 事件发生时刻偏差；
   - 阈值余量（事件时刻的 diff 距 enter/exit 阈值的距离）。

### 6.3 指标与标定

- 检测延迟：目标 ≤ 300ms（受 `active_filter_ms` 与 debounce 影响）；
- 漏检率 / 误检率：按业务定义（建议无人环境下误检 < 1 次/小时起步）；
- 标定对象：`sensitivity`、`active_jitter_min`、`active_filter_ms`、`presence_sensitivity`；
- 标定方法：离线回放网格搜索 → 真机复核。

---

## 7. 里程碑

| 里程碑 | 内容 | 输出 |
|--------|------|------|
| M1 | CSI 适配器 + 采集 dump 工具 | 可稳定录 CSI |
| M2 | motion_dec 重写 + 单元测试 + x86 回放对拍 | 曲线趋势一致 |
| M3 | algo 整体移植到盒子真机 + 标定 | 真机事件输出 |
| M4 | 集成主业务 + 自启/看护/日志 | 可交付 |

---

## 8. 风险与对策

| 风险 | 影响 | 对策 |
|------|------|------|
| motion_dec 重写难对齐 ESP 效果 | 检测精度下降 | 金标准数据迭代；必要时向 Espressif 申请源码/授权 |
| 驱动 CSI 帧率/量纲与 ESP 差异大 | 阈值失效 | 增益补偿 + 重新标定；V2/V3 提前暴露 |
| 驱动不提供 gain/noise_floor | 环境鲁棒性差 | 关闭补偿，增大阈值余量，或推动驱动补齐字段 |
| CSI 采样率抖动影响窗口 | 特征不稳 | 以时间戳重采样/对齐窗口，而非按帧数 |
| FreeRTOS→pthread 时序差异 | debounce/hold 行为漂移 | 用 CLOCK_MONOTONIC 绝对时间实现 hold 与窗口 |

---

## 9. 交付物清单

1. `wifi_sensingd` 守护进程源码（模块 5.1 各模块）。
2. x86 离线回放/对拍工具 `replay_tool`。
3. CSI 采集 dump 工具。
4. 标定脚本（网格搜索参数）。
5. 本设计文档 + 接口头文件。
