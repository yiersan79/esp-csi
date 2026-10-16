/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 * 【逐行注释版】数学统计工具库
 * ----------------------------------------------------------------------------
 * 本文件是整套感知算法的"数学积木"：
 *   - corr()      Pearson 相关系数 —— jitter / wander 的相似度度量核心
 *   - trimmean()  截尾均值         —— Presence 判定的抗干扰平均
 *   - median/std/trimstd —— 排序类统计，用于调试与离群分析
 * 所有函数都是纯计算，无副作用，可直接单测对拍。
 * ============================================================================
 */

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <math.h>
#include <stdbool.h>

#include "esp_radar.h"

/**
 * 算术平均：avg = (1/n) Σ a[i]
 * len=0 时返回 0，避免除零。
 */
float avg(const float *a, size_t len)
{
    if (!len) {                 /* 空数组保护：返回 0 而不是 NaN */
        return 0.0;
    }

    float sum = 0;              /* 累加器 */

    for (int i = 0; i < len; i++) {   /* 逐元素累加 */
        sum += a[i];
    }

    return sum / len;           /* 除以元素个数得到平均 */
}

/**
 * 求和：Σ a[i]。最基础的累加，供其它统计函数复用。
 */
float sum(const float *a, size_t len)
{
    float sum = 0;              /* 累加器 */

    for (int i = 0; i < len; i++) {
        sum += a[i];
    }

    return sum;

}

/**
 * qsort 比较回调：float 升序。
 * 注意：相等时返回 -1 也不影响 qsort 正确性（非严格弱序但不破坏结果）。
 */
int cmp_float(const void *a, const void *b)
{
    return *(float *)a > *(float *)b ? 1 : -1;   /* a>b 返回 1 → a 排后面 */
}

/**
 * 截尾均值（trimmed mean）
 *
 * 做法：先排序，再只对"小的那一侧"的 (1-percent) 个元素求平均——
 *       即丢掉较大的 percent 部分，对剩余求均值。
 * 例：len=25, percent=0.5 → 取排序后前 13 个的平均。
 *
 * 在本算法中的用途：Presence 的 wander 历史窗口用它平均，
 * 大的离群值（突变瞬间）被丢弃，均值反映"持续偏离"而不是"瞬时尖峰"。
 */
float trimmean(const float *array, size_t len, float percent)
{
    float trimmean_data = 0;
    float *tmp = RADAR_MALLOC_RETRY(sizeof(float) * len);   /* 副本，避免改原数组 */

    memcpy(tmp, array, sizeof(float) * len);    /* 复制原数组 */
    qsort(tmp, len, sizeof(float), cmp_float);  /* 升序排序 */
    trimmean_data = avg(tmp, (int)(len * (1 - percent) + 0.5));  /* 对前 (1-p)*n 个求平均（+0.5 四舍五入） */

    RADAR_FREE(tmp);            /* 释放副本 */

    return trimmean_data;
}

/**
 * 分位数型最大值：排序后取"倒数第 percent/2 位置"的值。
 * 即把最极端的 percent/2 个大值切掉后，再取当前最大值。
 * （比裸 max 更抗离群，本项目主流程未用，属于备用工具）
 */
float max(const float *array, size_t len, float percent)
{
    float max_data = 0;
    float *tmp = RADAR_MALLOC_RETRY(sizeof(float) * len);

    memcpy(tmp, array, sizeof(float) * len);    /* 复制原数组 */
    qsort(tmp, len, sizeof(float), cmp_float);  /* 升序排序 */
    max_data = tmp[len - (int)(len * percent / 2 + 0.5)];   /* 从尾部往回跳过 percent/2 个再取 */

    RADAR_FREE(tmp);
    return max_data;
}

/**
 * 分位数型最小值：与 max 对称——切掉最小的 percent/2 个再取当前最小。
 */
float min(const float *array, size_t len, float percent)
{
    float min_data = 0;
    float *tmp = RADAR_MALLOC_RETRY(sizeof(float) * len);

    memcpy(tmp, array, sizeof(float) * len);
    qsort(tmp, len, sizeof(float), cmp_float);
    min_data = tmp[(int)(len * percent / 2 + 0.5)];   /* 从头部跳过 percent/2 个再取 */

    RADAR_FREE(tmp);
    return min_data;
}

/**
 * 样本协方差（单对向量）：cov(x,y) = Σ(x-x̄)(y-ȳ) / (n-1)
 * 与 pca.c 里 compute_cov 的区别：这里是"两个向量的协方差"（减了均值），
 * pca 里是"矩阵列间"（未减均值）。corr() 内部其实不需要显式协方差，
 * 这个函数是独立提供的统计工具。
 */
float cov(const float *x, const float *y, size_t len)
{
    float aver_x, aver_y, sum_x = 0, sum_y = 0;
    float sum = 0;

    for (int i = 0; i < len; i++) {     /* 第一遍：求两个均值 */
        sum_x += x[i];
        sum_y += y[i];
    }

    aver_x = sum_x / len;
    aver_y = sum_y / len;

    for (int i = 0; i < len; i++) {     /* 第二遍：累加离差乘积 */
        sum += (x[i] - aver_x) * (y[i] - aver_y);
    }

    return sum / (len - 1);             /* n-1：无偏估计（样本协方差） */
}

/**
 * ★ Pearson 相关系数 —— 整个感知算法最重要的相似度度量 ★
 *
 * corr(a,b) = Σ(a-ā)(b-b̄) / sqrt(Σ(a-ā)² · Σ(b-b̄)²)
 * 值域 [-1, 1]：1=完全同向线性相关，0=不相关，-1=反向。
 * 调用方一律取 |corr|，所以符号不重要。
 *
 * 在算法中的角色（两个地方都用它）：
 *   1) jitter：当前 PCA 指纹 vs 滑窗内近期指纹 → 波形变化快慢
 *   2) wander：当前 PCA 指纹 vs 训练期背景指纹 → 与无人环境的偏离
 *
 * 退化保护：某个向量是常数（方差=0）时除零 → 返回 NaN。
 * 实测静止场景 PCA 指纹波动极小但非零，一般不会触发。
 */
float corr(const float *a, const float *b, size_t len)
{
    float aver_a = 0, aver_b = 0, sum_a = 0, sum_b = 0;
    float cov_sum = 0, var_sum_a = 0, var_sum_b = 0;

    for (int i = 0; i < len; i++) {     /* 第一遍：求两个向量的均值 */
        sum_a += a[i];
        sum_b += b[i];
    }

    aver_a = sum_a / len;
    aver_b = sum_b / len;

    for (int i = 0; i < len; i++) {     /* 第二遍：累加协方差与方差 */
        cov_sum += (a[i] - aver_a) * (b[i] - aver_b);   /* 分子：离差乘积 */
        var_sum_a += powf((a[i] - aver_a), 2);          /* 分母左：a 的方差和 */
        var_sum_b += powf((b[i] - aver_b), 2);          /* 分母右：b 的方差和 */
    }

    return cov_sum / sqrtf(var_sum_a * var_sum_b);      /* Pearson 公式 */
}

/**
 * 总体标准差：sqrt( Σ(a-ā)² / n )（除 n 不是 n-1，总体口径）。
 * 用于衡量一组值的离散程度。
 */
float std(const float *a, size_t len)
{
    float avg_data = avg(a, len);       /* 先求均值 */
    float pow_sum  = 0;                 /* 平方差累加器 */

    for (int i = 0; i < len; i++) {
        pow_sum += powf(a[i] - avg_data, 2);   /* Σ(a-ā)² */
    }

    return sqrtf(pow_sum / len);        /* 开方得标准差 */
}

/**
 * 对称化欧氏距离（自定义）：取 |a-b| 和 |a+b| 两种距离的较小者再除以 n。
 * 设计意图：指纹向量的整体符号可能翻转（PCA 主成分方向不唯一），
 * 同时考虑 a-b 和 a+b 就对"符号翻转"不变。本项目主流程未用。
 */
float dis(const float *a, const float *b, size_t len)
{
    float pow_sum[2]  = {0};            /* [0]=|a-b|² 累加, [1]=|a+b|² 累加 */

    for (int i = 0; i < len; i++) {
        pow_sum[0] += powf((a[i] - b[i]), 2);   /* 差距离分量 */
        pow_sum[1] += powf((a[i] + b[i]), 2);   /* 和距离分量（等价于符号翻转后的差） */
    }

    return sqrtf(MIN(pow_sum[0], pow_sum[1])) / len;   /* 取小的那个（符号无关） */
}

/**
 * 中位数：排序后取中间值（偶数个取中间两个的平均）。
 */
float median(const float *a, size_t len)
{
    float median_data = 0;
    float *tmp        = RADAR_MALLOC_RETRY(sizeof(float) * len);   /* 副本 */
    memcpy(tmp, a, sizeof(float) * len);

    qsort(tmp, len, sizeof(float), cmp_float);          /* 升序排序 */
    median_data = (len % 2) ? tmp[len / 2] : (tmp[len / 2 - 1] + tmp[len % 2]) / 2;   /* 奇取中，偶取平均 */

    RADAR_FREE(tmp);

    return median_data;
}

/**
 * 截尾标准差：排序 → 只保留小的 (1-percent) 部分 → 求标准差。
 * 与 trimmean 配套的离散度指标，用于调试分析。
 */
float trimstd(const float *array, size_t len, float percent)
{
    float trimstd_data = 0;
    float *tmp = RADAR_MALLOC_RETRY(sizeof(float) * len);

    memcpy(tmp, array, sizeof(float) * len);    /* 复制 */
    qsort(tmp, len, sizeof(float), cmp_float);  /* 排序 */
    trimstd_data = std(tmp, (int)(len * (1 - percent) + 0.5));   /* 对保留部分求 std */

    RADAR_FREE(tmp);

    return trimstd_data;
}
