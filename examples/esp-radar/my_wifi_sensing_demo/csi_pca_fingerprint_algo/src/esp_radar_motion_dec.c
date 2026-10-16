/*
 * 【逐行注释版】esp_radar_motion_dec.c —— 本地开源替代组件
 * ============================================================================
 * 背景：原 demo 用的是闭源预编译库 espressif__esp_radar_motion_dec.a，
 * 本文件按 esp-csi 历史 commit 6dc275f 的开源算法重写，API 完全一致：
 *   - esp_radar_motion_dec_compute()        PCA 指纹（= 旧版 old_pca.c）
 *   - esp_radar_motion_dec_wander_compute() wander（语义已翻转为 0=相似）
 *   - esp_radar_motion_dec_compute_jitter() jitter（语义已翻转为 0=稳定）
 *
 * 与旧版的两个关键差异：
 *   1) 语义反转：旧版输出 corr 口径（越大越稳定），本版输出前统一做
 *      1-corr，直接给出"偏离/抖动度"（0=稳定，1=剧烈）——与闭源库语义对齐
 *   2) jitter 索引方式：改为显式计算环形槽位 + 越界保护（见函数内详注）
 * ============================================================================
 */
#include <string.h>         /* memset */
#include <math.h>           /* fabsf / sqrtf / powf */
#include <stdlib.h>         /* malloc / free（组件独立，不再用 RADAR_MALLOC_RETRY 宏） */

#include "esp_err.h"        /* esp_err_t / ESP_OK / ESP_FAIL */
#include "esp_radar_motion_dec.h"   /* 本组件 API 声明 */

#define ITRS_MAX     30     /* 幂法最大迭代次数（防不收敛死循环） */
#define PRECISION    0.0001 /* 幂法收敛阈值：相邻两次特征值差小于此即认为收敛 */

/**
 * Pearson 相关系数（组件内私有版本，等价于 old_utils.c 的 corr()）
 * 唯一改进：分母为零（某向量是常数）时返回 0 而不是 NaN——
 * 静止场景 PCA 指纹可能几乎恒定，NaN 会污染下游 max 比较逻辑。
 */
static float local_corr(const float *a, const float *b, size_t len)
{
    float aver_a = 0, aver_b = 0, sum_a = 0, sum_b = 0;     /* 均值累加器 */
    float cov_sum = 0, var_sum_a = 0, var_sum_b = 0;        /* 协方差/方差累加器 */

    for (size_t i = 0; i < len; i++) {          /* 第一遍：求两个均值 */
        sum_a += a[i];
        sum_b += b[i];
    }

    aver_a = sum_a / len;                       /* a 的均值 */
    aver_b = sum_b / len;                       /* b 的均值 */

    for (size_t i = 0; i < len; i++) {          /* 第二遍：协方差与方差累加 */
        cov_sum += (a[i] - aver_a) * (b[i] - aver_b);   /* 分子：离差乘积和 */
        var_sum_a += powf((a[i] - aver_a), 2);          /* 分母左：a 方差和 */
        var_sum_b += powf((b[i] - aver_b), 2);          /* 分母右：b 方差和 */
    }

    if (var_sum_a == 0 || var_sum_b == 0) {     /* 零方差保护：常数向量相关无意义 */
        return 0;
    }

    return cov_sum / sqrtf(var_sum_a * var_sum_b);      /* Pearson 公式，值域 [-1,1] */
}

/**
 * ★ PCA 指纹主入口 ★（与旧版 old_pca.c 的 pca() 逐句等价，合并成单函数）
 *
 * 输入：一个时间窗口的 CSI 振幅（环形缓冲可能拆成 data_0/data_1 两段）
 * 输出：output[cols] —— 每子载波一个"空间指纹"值
 *
 * 数学流程三步：
 *   1) 转置组矩阵：matrix[子载波][帧]，行=子载波、列=帧
 *   2) 帧间协方差：cov[i][j] = Σ_k matrix[k][i]·matrix[k][j] / (row·column)
 *      （未减均值的 X^T·X 缩放版，对找最大方差方向等价）
 *   3) 幂法求主特征向量 + 投影：把每个子载波的时间序列投影到主成分方向
 */
esp_err_t esp_radar_motion_dec_compute(uint32_t cols,
                                       uint32_t row_0, const float data_0[row_0][cols],
                                       uint32_t row_1, const float data_1[row_1][cols],
                                       float output[cols])
{
    esp_err_t ret   = ESP_OK;
    uint32_t row    = cols;             /* 组矩阵后行数 = 子载波数 */
    uint32_t column = row_0 + row_1;    /* 组矩阵后列数 = 窗口内总帧数 */

    float (*matrix)[column] = malloc(row * column * sizeof(float));         /* 转置矩阵 */
    float (*matrix_cov)[column] = malloc(column * column * sizeof(float));  /* 协方差矩阵（帧×帧） */
    float *eigenvector = malloc(column * sizeof(float));                    /* 主特征向量 */
    float *eigenvalue_list = malloc(column * sizeof(float));                /* M·v 结果暂存 */

    if (!matrix || !matrix_cov || !eigenvector || !eigenvalue_list) {       /* 任一分配失败 */
        ret = ESP_ERR_NO_MEM;
        goto EXIT;                          /* 统一出口释放 */
    }

    for (uint32_t i = 0; i < row; i++) {            /* 第 1 步：转置组矩阵 */
        for (uint32_t j = 0; j < row_0; j++) {      /* 第一段：环形缓冲后半 begin→尾 */
            matrix[i][j] = data_0[j][i];
        }
        for (uint32_t j = 0; j < row_1; j++) {      /* 第二段：环形回绕部分 头→end */
            matrix[i][j + row_0] = data_1[j][i];
        }
    }

    memset(matrix_cov, 0, column * column * sizeof(float));     /* 协方差矩阵清零（是累加目标） */
    {                                   /* 第 2 步：帧间协方差（内联了 compute_cov） */
        uint32_t zoom_out = row * column;   /* 归一化分母 = 总元素个数 */

        for (uint32_t i = 0; i < column; i++) {         /* 列 i（帧 i） */
            for (uint32_t j = 0; j < i + 1; j++) {      /* 只算下三角（对称矩阵） */
                for (uint32_t k = 0; k < row; k++) {    /* 对所有子载波行累加内积 */
                    matrix_cov[i][j] += matrix[k][i] * matrix[k][j];    /* 帧 i · 帧 j */
                }
                matrix_cov[i][j] /= zoom_out;           /* 归一化防数值爆炸 */

                if (i != j) {
                    matrix_cov[j][i] = matrix_cov[i][j];    /* 对称位直接复制（省一半计算） */
                }
            }
        }
    }

    /* 第 3 步：幂法求主特征向量（与旧版 matrix_power_method_eigen 等价，内联） */
    {
        double eigenvalue = 1.0;                /* 当前最大分量（缩放因子），double 减浮点误差 */
        double eigenvalue_last = 0;             /* 上一轮缩放因子，判收敛用 */
        uint32_t iterate = 0;                   /* 迭代计数 */

        for (uint32_t i = 0; i < column; i++) {
            eigenvector[i] = 1;                 /* 初始向量取全 1（经典幂法起点） */
        }

        for (iterate = 0; fabs(eigenvalue - eigenvalue_last) > PRECISION && iterate < ITRS_MAX; iterate++) {    /* 未收敛且未超限就继续 */
            eigenvalue_last = eigenvalue;       /* 记录上一轮缩放因子 */
            eigenvalue      = 0;                /* 本轮重新找最大分量 */

            for (uint32_t i = 0; i < column; i++) {     /* 矩阵-向量乘：list = M·v */
                eigenvalue_list[i] = 0;
                for (uint32_t j = 0; j < column; j++) {
                    eigenvalue_list[i] += matrix_cov[i][j] * eigenvector[j];
                }
                if (eigenvalue_list[i] > eigenvalue) {  /* 顺便追踪最大分量 */
                    eigenvalue = eigenvalue_list[i];
                }
            }

            if (eigenvalue == 0) {              /* 零矩阵保护：除零 → 放弃本窗口 */
                ret = ESP_FAIL;
                goto EXIT;
            }
            for (uint32_t i = 0; i < column; i++) {
                eigenvector[i] = eigenvalue_list[i] / eigenvalue;   /* 除最大分量归一化 */
            }
        }

        if (iterate == ITRS_MAX) {              /* 跑满迭代仍未收敛 → 失败 */
            ret = ESP_FAIL;
            goto EXIT;
        }
    }

    for (uint32_t i = 0; i < row; i++) {        /* 投影：指纹[i] = Σ_j matrix[i][j]·v[j] / 帧数 */
        output[i] = 0;
        for (uint32_t j = 0; j < column; j++) {
            output[i] += matrix[i][j] * eigenvector[j];     /* 主成分方向加权和 */
        }
        output[i] /= column;                    /* 除以帧数：量纲不随窗口大小变化 */
    }

EXIT:
    free(matrix);                               /* 统一出口释放四块内存 */
    free(matrix_cov);
    free(eigenvector);
    free(eigenvalue_list);
    return ret;
}

/**
 * ★ wander（Presence 特征）★
 * 内部先算 corr 口径（1=完全像），再翻转为偏离度：wander = 1 - |corr|。
 * 0 = 与背景完全一样（无人），越大 = 环境偏离无人状态越远（可能有人）。
 */
float esp_radar_motion_dec_wander_compute(const float *a, const float *b, size_t len)
{
    float c = fabsf(local_corr(a, b, len));     /* |corr|：符号无关（PCA 主成分方向不唯一） */
    return 1.0f - c;                            /* 语义翻转：corr→偏离度 */
}

/**
 * ★ jitter（Motion 特征）★
 * 当前指纹与滑窗内最近 (window_size-1) 条历史指纹的最大 |corr|，
 * 再翻转为抖动度：jitter = 1 - max|corr|。
 * 0 = 窗口内波形完全稳定（静止），越大 = 波形剧变（有人活动）。
 *
 * 环形索引逻辑：
 *   current_idx = (buff_num-1) % window_size  当前指纹所在槽位
 *   filled      = min(buff_num-1, window_size) 已填充的槽位数
 *   从 current 往回绕一圈遍历历史，未填充槽位直接停止。
 */
float esp_radar_motion_dec_compute_jitter(float *current,
                                          float **history,
                                          uint16_t cols,
                                          uint8_t window_size,
                                          uint32_t buff_num)
{
    /* 数据不足（当前帧 + 至少 1 条历史）：返回 0 = 最稳定（保守输出） */
    if (buff_num < 2 || window_size < 2 || !history) {
        return 0.0f;
    }

    uint32_t current_idx = (buff_num - 1) % window_size;    /* 当前指纹的环形槽位号 */
    uint32_t filled = (buff_num - 1 < (uint32_t)window_size) ? (buff_num - 1) : (uint32_t)window_size;  /* 已填充槽数（上限=窗口大小） */

    float corr_max = 0.0f;                      /* 最大相关（从 0 开始取 max） */
    for (int i = 0; i < window_size - 1; ++i) { /* 与窗口内其余指纹两两比 */
        uint32_t hist_idx = (current_idx + window_size - 1 - i) % window_size;  /* 往回第 i+1 个槽位（环形回退） */
        if (hist_idx >= filled) {               /* 未填充槽位：后面也不会有更早数据，停 */
            break;
        }
        float corr_val = fabsf(local_corr(current, history[hist_idx], cols));   /* 与该历史指纹的相关 */
        if (corr_val > corr_max) {              /* 取最大相似度 */
            corr_max = corr_val;
        }
    }

    /* 语义翻转：corr 口径（大=稳定）→ 抖动度（0=稳定，1=剧变） */
    return 1.0f - corr_max;
}
