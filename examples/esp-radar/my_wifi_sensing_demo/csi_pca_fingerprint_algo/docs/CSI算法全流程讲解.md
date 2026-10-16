# WiFi CSI 感知算法全流程讲解

> 基于 esp-csi 旧版开源实现（commit `6dc275f`）整理，配套注释版源码：
> `old_esp_radar_annotated.c` / `old_pca_annotated.c` / `old_utils_annotated.c(h)`

---

## 0. 一分钟总览

WiFi CSI（Channel State Information，信道状态信息）记录的是 WiFi 信号从发射端到接收端**每个 OFDM 子载波上的幅度和相位**。人在环境中活动会改变多径反射，从而改变 CSI —— 这就是"用 WiFi 当雷达"的物理基础。

本算法把 CSI 变成两个输出：

```
jitter（抖动）  → Motion 检测：有没有人在"动"
wander（漂移）  → Presence 检测：环境是否"偏离无人基线"（静坐的人也算）
```

两者共享同一个中间产物 —— **PCA 空间指纹**。

```
WiFi CSI 帧流（每 20ms 一帧）
  │ ①回调层：过滤/重建/补偿
  ▼
环形缓冲（每帧 = 子载波振幅向量）
  │ ②预处理层：滑窗攒批
  ▼
窗口（N 帧 × 子载波 振幅矩阵）
  │ ③PCA：压成"空间指纹"向量
  ▼
指纹序列
  ├─④a jitter = 当前指纹 vs 窗口内近期指纹的相关性   → Motion
  └─④b wander = 当前指纹 vs 训练期背景指纹的相关性   → Presence
  ▼
⑤语义翻转 (1-corr) → 用户回调 / FSM 状态机
```

---

## 1. 什么是 CSI：物理输入长什么样

一帧 CSI 数据包含一次 WiFi 传输中**长训练字段（LTF）**上各子载波的复数响应：

```
CSI 帧 = [子载波0: (I₀,Q₀)] [子载波1: (I₁,Q₁)] ... [子载波N-1: (Iₙ₋₁,Qₙ₋₁)]
```

- `I`（实部）、`Q`（虚部）是有符号整数（老芯片 int8，C5/C61 的 LLTF 是 12bit 紧凑排列）
- 每个子载波的**振幅** = `sqrt(I² + Q²)` —— 算法只用振幅，不用相位
- 20MHz 11n 模式约 56 个有效子载波；11ax (HE) 更多

物理含义：`|Hₖ|`（第 k 个子载波的信道增益）由直射径 + 所有反射径在该频率上的矢量叠加决定。**人移动 → 反射路径长度变化 → 各子载波振幅出现特定模式的起伏**。不同位置的人造成的"起伏模式"（跨子载波的形状）不同 —— 这正是后面用 PCA 和相关性来捕捉的东西。

---

## 2. 回调层：从 WiFi 驱动到干净数据（`esp_radar_csi_rx_cb`）

WiFi 驱动每收到一帧满足条件的包就调用一次回调，发生在 WiFi 任务上下文，必须快：

### 2.1 MAC 过滤（`esp_radar_mac_addr_filter`）

- 配了 `filter_mac`：只收源 MAC 匹配的帧
- 没配：只收**空数据帧**（payload=14 字节）—— ESPNOW/keep-alive 空帧最稳定，适合当探测信号
- 可选目的 MAC 二级过滤

### 2.2 元信息提取（`esp_radar_extract_rx_ctrl_info`）

从硬件寄存器视图提取：RSSI、时间戳、**信号格式**（11b/g/n/ac/ax）、带宽、STBC、AGC/FFT 增益。格式信息决定下一步查哪行布局表。

### 2.3 子载波重建（`esp_radar_rebuild_csi_data` + `copy_subcarrier_data`）

**这是理解 CSI 数据布局的关键。** 原始 buf 里不是所有字节都有用（导频、保护子载波混在其中），且 LLTF/HT-LTF/HE-LTF 各段交织排列。

`csi_sub_carrier_table.c` 按 `(信号模式, 带宽, STBC, 数据长度)` 登记了每行布局：各段有效子载波的字节区间。重建 = 按区间把有效字节抄到新的紧凑缓冲，得到：

```
filtered_info->valid_data = [LLTF 段 | HT-LTF 段 | STBC 段 | HE-LTF 段]
```

查表不命中（格式不在表里）→ 丢帧。12bit 模式下按 16bit 单位做符号扩展（左移4 + 算术右移4）。

### 2.4 增益补偿 + 入队

AGC 会动态调接收增益，导致相邻帧振幅基准不同。`esp_csi_gain_ctrl` 记录每帧增益并可原地补偿（可选）。最后把干净帧指针塞进 `csi_info_queue`（容量 5，**不等待**，满则丢 —— CSI 是流数据不能堵 WiFi 任务）。

---

## 3. 预处理任务（`csi_preprocessing_task`）：字节 → 振幅 → 环形缓冲

### 3.1 切 LTF 段（`esp_radar_get_ltf_data`）

按配置的 `ltf_type` 从拼接缓冲里定位对应段（demo 用 LLTF：子载波最多、时间敏感性最好）。

### 3.2 I/Q → 振幅（`csi_write_frame_to_ring`）

```c
INT16(12bit): 每子载波 4 字节 [imag_lo, imag_hi, real_lo, real_hi]
              符号扩展: ((int16_t)(x << 12)) >> 4 | 低8位 → 右移4
INT8:         每子载波 2 字节 [imag, real]
振幅 = sqrt(I² + Q²)      （step=子载波抽取步长，可隔点采样减计算量）
```

结果写入环形缓冲 `csi_data_buff`（行=帧，列=子载波），同时记时间戳(ms)和包序号。

### 3.3 离群帧过滤（`csi_outlier_filter_process`）

```
对每个子载波: ref = 前3帧均值;  |curr - ref| > threshold 记一次异常
异常子载波 ≥ 一半 → 整帧离群
离群 streak ≥ 3  → 环境真变了，接受为新基线（防止换环境后永远过滤）
```

旧版"软处理"只记录不丢帧；新版闭源库改为真丢帧。

### 3.4 滑窗推进（`csi_window_update`）—— 半窗重叠滑动

窗口攒够 `handle_window` 帧或时间跨 2 倍 `csi_handle_time` 才触发一次处理：

```
触发后: 窗口起点前移 window/2   （50% 重叠 → 相邻窗口共享一半数据，输出更平滑）
帧数 < window/3        → 太少不值得算 PCA，丢弃重开
时间戳回退/断流超时    → 窗口作废重开（防跨空洞计算假相关）
```

触发时把 `[begin, end, window]` 压缩成 32 位索引（union 技巧）发进 `csi_data_queue`（容量 1）。

---

## 4. PCA 指纹（`old_pca.c`）：矩阵 → 一个向量

特征计算任务拿到窗口索引后，第一步是把窗口的 N 帧振幅压成**一条指纹向量**。

### 4.1 组矩阵

把 `[帧][子载波]` 的数据**转置**成 `[子载波][帧]`：

```
matrix[k][j] = 第 k 个子载波在第 j 帧的振幅
行（k）= 子载波维度    列（j）= 时间维度
```

即：每一行是"一个子载波随时间的起伏曲线"。

### 4.2 协方差（`compute_cov`）

```c
cov[i][j] = (1/(row×col)) × Σₖ matrix[k][i]·matrix[k][j]
```

帧×帧 的相似度矩阵（注意：未减均值的缩放 XᵀX 版本 —— 对"找最大方差方向"目的等价，省一次遍历）。只算下三角再镜像，省一半计算。

**直觉**：如果两个时刻的多径结构相似，对应列的内积就大。

### 4.3 幂法求主特征向量（`matrix_power_method_eigen`）

```c
v = 全1向量
循环 ≤30 次:
    u = M·v                    // 矩阵乘向量
    eigenvalue = max(u)        // 取最大分量做缩放因子
    v = u / eigenvalue         // 归一化
直到 |eigenvalue - eigenvalue_last| < 0.0001
```

收敛后 `v` ≈ 协方差矩阵最大特征值对应的特征向量 = **窗口内变化最剧烈的方向**（主成分）。

### 4.4 投影

```c
output[i] = (Σⱼ matrix[i][j]·v[j]) / column
```

每个子载波的时间序列在主成分方向上的加权平均 —— 得到长度=子载波数的**指纹**。

**为什么 PCA 有效**：20MHz 下有 56 个子载波，逐子载波比对太吵。PCA 把 N 帧里"共同变化模式"提取成 1 维，等于做了**空时联合降噪**：噪声（各子载波独立起伏）在投影中被平均掉，人体运动（多子载波相干起伏）被保留。

---

## 5. Motion：jitter（帧间相关性）

`csi_detection_update_waveform_metrics`（old_esp_radar.c:737）

```
jitter(corr 口径) = max over 窗口内近 (W-1) 条指纹 的 |corr(当前指纹, 历史指纹)|
```

- `corr` = Pearson 相关系数（utils.c:109），度量两条指纹"形状"的相似度，与幅度 scale 无关
- 窗口 W=4：当前指纹与前面 3 条逐一算相关，**取最大**（只要有"一瞬间稳定"就算稳定 → 抗单帧毛刺）

**语义**（corr 口径，越大越稳定）→ 输出前统一翻转 `jitter = 1 - corr`：

| 场景 | 窗口内指纹关系 | corr | jitter (1-corr) |
|---|---|---|---|
| 完全静止 | 4 条指纹几乎相同 | ≈1 | ≈0 |
| 有人走动 | 每帧多径都在变 | 低 | 高 |
| 静止但刚走动完 | 部分指纹已重新稳定 | 中 | 中 |

之后 FSM 层（新版 `esp_wifi_sensing.c`）做：自适应平滑（灵敏度映射平滑系数 0.06~0.38）→ 自适应基线跟踪 → 迟滞阈值 + 连续命中防抖 → IDLE/ACTIVE 状态翻转 → `motion_status`。

---

## 6. Presence：wander（对训练背景的相关性）

### 6.1 训练期（`csi_training_collect_sample`，环境必须静止）

每窗口一次，三件事：

1. **jitter 三点离群剔除**：连续 3 个 jitter 中，中间值比两侧都低 0.005 → 两侧是瞬时尖峰，丢弃本次采样
2. **收集静止参考**：`static_corr` 取训练期见过的**最小** jitter corr（最安静时刻）
3. **背景采样决策**（关键，滞后一拍逻辑）：
   - 上一拍 `wander corr < 0.998`（刚发生过扰动但已平息）→ 把**上上帧**的 PCA 指纹存入背景库（环形 10 条）
   - 上一拍持续稳定（corr ≥ 0.998）→ 累计背景统计 `none_corr_sum / none_corr_count`

   滞后一拍 + 取上上帧的设计：确保存入库里的确实是"稳定时刻"的指纹，而不是扰动瞬间的。

### 6.2 生成阈值（`esp_radar_train_stop`）

```
train_wander_threshold = 1 - (none_corr_sum / none_corr_count)   ← 无人基线的偏离度
train_jitter_threshold = 1 - static_corr                          ← 静止参考的抖动度
```

### 6.3 运行期判定

特征层（`csi_detection_compute_wander`）：

```
wander(corr 口径) = max over 背景库 ≤10 条 的 |corr(当前指纹, 背景指纹)|
→ 与"任何一条无人时指纹"最像的程度
→ 翻转: wander = 1 - corr   （0=与背景一致，1=完全不像）
```

判定层（新版 FSM `hms_update_presence_state`）：

```c
// 25 个 wander 样本的历史环 → 截尾均值（丢大的一半再平均，抗瞬时尖峰）
presence_wander_average = trimmean(history, 25, 0.5);
// 灵敏度 [0,1] → 增益 1~30
gain = 1 + presence_sensitivity × 29;
// 判定：均值放大增益后超过训练阈值 → 有人
presence_someone_status = ready && (avg × gain > train_wander_threshold);
```

**为什么静坐的人能被 presence 抓到而 motion 不能**：静坐时帧间指纹几乎不变（jitter≈0，Motion 报"无运动"），但人的身体改变了多径结构 → 当前指纹与"无人背景库"的相关性下降 → wander 高 → Presence 报"有人"。

---

## 7. 语义翻转的细节（容易踩的坑）

旧版代码内部统一用 **corr 口径**（值越大 = 越相似 = 越稳定），只在**输出给用户前**统一翻转：

```c
radar_info.waveform_wander = 1.0f - radar_info.waveform_wander;   // esp_radar.c:894
radar_info.waveform_jitter = 1.0f - radar_info.waveform_jitter;   // esp_radar.c:895
```

我们写的本地替代组件（`local_replacement_component`）已在函数内部完成翻转，与新版闭源库语义一致（0=稳定/像背景，大=动/偏离）。若自己魔改算法，务必先确认当前在"哪一侧"：

| 量 | corr 口径（内部） | 输出口径（对用户） |
|---|---|---|
| jitter | 大 = 稳定 | **0 = 稳定，大 = 在动** |
| wander | 大 = 像背景 | **0 = 像背景，大 = 偏离** |
| static_corr | 训练期最小 jitter corr | `1-static_corr` = jitter 阈值 |
| none_corr_avg | 背景相关均值 | `1-avg` = wander 阈值 |

---

## 8. 完整数据流走查（以一帧 CSI 为例）

```
1. AP 发出一帧 11ax 数据，ESP32-C5 硬件解出 CSI
2. esp_radar_csi_rx_cb（WiFi 上下文）
   ├ MAC 过滤：不是目标 peer → 丢弃
   ├ 查 rx_ctrl：HE 格式、20MHz、无 STBC
   ├ 查子载波表：命中 HE20 行 → 抄出 LLTF/HE-LTF 有效子载波
   ├ （可选）增益补偿
   └ filtered_info 入队 csi_info_queue
3. csi_preprocessing_task 取出帧
   ├ 取 LLTF 段 → 解 12bit I/Q → sqrt(I²+Q²) → 56 维振幅向量
   ├ 写入环形缓冲（next_seq % buff_size 位置）
   ├ 离群过滤（与前 3 帧均值比）
   └ csi_window_update：窗口攒够 → 索引入 csi_data_queue；窗口起点前移半窗
4. csi_detection_task 取出窗口索引
   ├ 取出 [begin..end] 的 N×56 振幅矩阵（处理环形回绕两段）
   ├ pca(): 转置 → 协方差(帧×帧) → 幂法(≤30 迭代) → 投影 → 56 维指纹
   ├ jitter = 1 - max|corr(当前指纹, 窗口内前 3 条指纹)|
   ├ wander = 1 - max|corr(当前指纹, 背景库 10 条)|
   ├ 训练态：三点离群剔除 → 存背景指纹 / 累计背景统计
   └ 回调 wifi_radar_cb(jitter, wander)
5. FSM 层（esp_wifi_sensing 组件）
   ├ Motion: 平滑 → 基线 → 迟滞+防抖 → IDLE/ACTIVE → LED/事件
   └ Presence: 25 点截尾均值 × 灵敏度增益 > train_wander_threshold → 有人/无人
```

---

## 9. 参数速查（在哪调什么）

| 参数 | 默认 | 作用 | 影响谁 |
|---|---|---|---|
| `csi_recv_interval` | 20ms | 发帧间隔 | 全部（决定时间分辨率） |
| `csi_handle_time` | — | 窗口时间基准 | 窗口/缓冲大小 = handle_time/interval×2 (+20) |
| `pca_window_size` | 4 | 指纹滑动窗口 | jitter（越大越抗噪但响应慢） |
| `sub_carrier_step_size` | 1 | 子载波抽取 | 计算量 vs 空间分辨率 |
| `outliers_threshold` | — | 离群帧判定阈值 | 预处理（0=关闭） |
| `CSI_CORR_THRESHOLD` | 0.998 | 训练"稳定"判定 | Presence 背景采样 |
| `RADAR_OUTLIERS_THRESHOLD` | 0.005 | 三点离群阈值 | 训练质量 |
| `presence_sensitivity` | 0.25 (demo) | Presence 灵敏度 → gain 1~30 | Presence 判定 |
| `sensitivity`（motion） | — | → 平滑系数 0.06~0.38 | Motion 响应速度 |

---

## 10. 已知局限与改进方向

1. **PCA 幂法可能不收敛**（30 次上限）→ 该窗口直接跳过（输出保持旧值）。改进：Jacobi 特征分解或固定投影向量（训练期学一次，运行期只做矩阵乘，省 90% CPU）
2. **未减均值的协方差**：对均值漂移（如温度引起的慢变）敏感。改进：减均值的标准协方差
3. **corr 不区分正负相关**：取绝对值后，"完全反向的多径变化"会被当成"相似"。`dis()`（对称欧氏距离）是为此准备的备选，主流程未用
4. **背景指纹库只有 10 条**：环境缓慢变化（家具挪动）会让库过时 → 需要定期重训（demo 的 `RESET_BASELINE`）
5. **单 peer 串行计算**：新版按 peer 分了独立缓冲（每 peer 一套指纹环），旧版是全局单份 —— 移植时注意

---

## 附：文件导读顺序

1. `old_utils_annotated.c` → `corr()`（30 行，5 分钟看懂相似度度量）
2. `old_pca_annotated.c` → `pca()`（协方差+幂法+投影，算法心脏）
3. `old_esp_radar_annotated.c` → 按 CHUNK 顺序：回调层 → 预处理任务 → 训练/特征 → 启停/配置
4. 对照 `local_replacement_component/src/esp_radar_motion_dec.c` 看语义适配（翻转发生在哪）
