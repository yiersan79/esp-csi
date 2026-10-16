// WiFi CSI 感知算法 —— 逐行注释版工具库头文件
//
// 这些函数是 old_esp_radar.c / old_pca.c 依赖的数学积木：
//   corr()      —— Pearson 相关系数，jitter/wander 的相似度核心
//   trimmean()  —— 截尾均值，抗离群平均
//   其余为排序/离散度等通用统计工具

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* MAC 字符串 "aa:bb:cc:dd:ee:ff" → 6 字节十六进制。解析失败返回 false。 */
bool mac_str2hex(const char *mac_str, uint8_t *mac_hex);

/* 算术平均 avg = Σ/n；len=0 返回 0 防除零 */
float avg(const float *array, size_t len);

/* 求和 Σ */
float sum(const float *a, size_t len);

/* 分位数型最大值：排序后跳过尾部 percent/2 个极端大值再取最大 */
float max(const float *array, size_t len, float percent);

/* 分位数型最小值：排序后跳过头部 percent/2 个极端小值再取最小 */
float min(const float *array, size_t len, float percent);

/* 截尾均值：排序后丢掉大的 percent 侧，对剩余 (1-percent) 部分求平均
 * —— Presence 的 wander 历史窗口用这个平均，抗瞬时尖峰 */
float trimmean(const float *array, size_t len, float percent);

/* 样本协方差 cov(x,y) = Σ(x-x̄)(y-ȳ)/(n-1)（减均值、无偏估计） */
float cov(const float *x, const float *y, size_t len);

/* ★ Pearson 相关系数 [-1,1]：1=线性同向，0=不相关
 * jitter（指纹帧间）与 wander（指纹对背景）都用它量"像不像"
 * 调用方一律取 |corr|，符号无关紧要 */
float corr(const float *a, const float *b, size_t len);

/* 总体标准差（除 n 不是 n-1） */
float std(const float *a, size_t len);

/* 对称化欧氏距离：min(|a-b|, |a+b|)/n —— 对 PCA 符号翻转不变（备用工具） */
float dis(const float *a, const float *b, size_t len);

/* qsort 的 float 升序比较回调 */
int cmp_float(const void *_a, const void *_b);

/* 中位数：奇取中，偶取中间两数平均 */
float median(const float *a, size_t len);

/* ★ PCA 指纹主入口（实现在 old_pca.c）★
 * 输入：一个时间窗口的 CSI 振幅（可能拆成环形缓冲的两段）
 * 输出：output[cols] —— 每子载波一个"空间指纹"值 */
esp_err_t pca(uint32_t cols,
              uint32_t row_0, const float data_0[row_0][cols],
              uint32_t row_1, const float data_1[row_1][cols],
              float output[cols]);

#ifdef __cplusplus
}
#endif
