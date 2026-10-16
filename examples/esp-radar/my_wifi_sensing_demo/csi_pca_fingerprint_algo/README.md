# csi_pca_fingerprint_algo

WiFi CSI 感知算法包：PCA 指纹 + corr 相似度，实现 Motion(jitter) 与 Presence(wander) 判定。

## 目录结构（每个代码文件只有一份，全部带逐行中文注释）

```
csi_pca_fingerprint_algo\
├── CMakeLists.txt                  # 组件构建脚本（部署时与 src/include 一起复制）
├── README.md                       # 本文件
├── include\
│   └── esp_radar_motion_dec.h      # 组件 API 头文件（逐行注释）
├── src\
│   └── esp_radar_motion_dec.c      # 组件实现：PCA 指纹 / jitter / wander（逐行注释）
├── esp_radar.c                     # 参考实现：主组件全流程（回调→预处理→特征→训练）
├── pca.c                           # 参考实现：PCA 指纹数学（协方差 + 幂法）
├── utils.c / utils.h               # 参考实现：corr / trimmean 等统计工具
└── docs\
    └── CSI算法全流程讲解.md         # 10 章算法讲解文档
```

- `src\ + include\ + CMakeLists.txt`：可部署的正式组件，与 D:\ESP 工程里的
  `components\esp_radar_motion_dec\` 保持一致（部署后哈希校验通过）。
- 根目录的 `esp_radar.c / pca.c / utils.c/.h`：来自 esp-csi commit 6dc275f 的
  上层算法流程参考实现，算法迭代时以它们为蓝本迁移逻辑到 `src\`。

## 数据流水线

```
WiFi 硬件回调 ──► MAC过滤/子载波重建/增益补偿 ──► 预处理(I/Q→振幅/滑窗)
      ──► PCA 指纹 ──► jitter(帧间 corr) ──► wander(对背景库 corr)
      ──► 语义翻转(1-corr) ──► wifi_radar_cb 用户回调
```

## 语义约定（输出口径）

| 量 | 公式 | 0 表示 | 1 表示 |
|----|------|--------|--------|
| jitter | 1 − max\|corr(当前指纹, 窗口内历史指纹)\| | 稳定 | 剧变（Motion） |
| wander | 1 − max\|corr(当前指纹, 背景指纹库)\| | 与无人环境一致 | 偏离（Presence） |

## 部署方式

把 `CMakeLists.txt + include\ + src\` 复制到
`wifi_sensing_demo\components\esp_radar_motion_dec\`，
组件名与托管闭源库 `espressif__esp_radar_motion_dec` 同名即自动覆盖。

## 后续迭代注意

- 改算法只改 `src\esp_radar_motion_dec.c`（并同步到 D:\ESP 部署目录）；
  根目录参考实现不要直接改，新逻辑迁入组件后对应参考文件可删。
- 语义口径与旧版相反（旧版 corr 大=稳定，本版输出前已翻转为 1−corr），
  与旧版对拍数值时先统一口径。
- 训练（背景指纹库采集）逻辑目前在 `esp_radar.c` 的
  `csi_training_collect_sample` / `esp_radar_train_*`，迁移新实现时一并带走。
