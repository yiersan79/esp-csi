/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 * 【逐行注释版】PCA 指纹计算 —— 整套 WiFi 感知算法的数学核心
 * ----------------------------------------------------------------------------
 * 原理一句话：把"一个时间窗口内 N 帧 CSI"压缩成一个长度=子载波数的
 * "空间指纹向量"。人动 → 多径结构变 → 指纹变；环境静止 → 指纹稳定。
 * 后续 Motion（帧间比指纹）和 Presence（对训练背景比指纹）都在这份指纹上工作。
 *
 * 数学流程（三步）：
 *   1) 组矩阵   : 行=子载波(cols)，列=帧(row_0+row_1)   —— 即"每个子载波
 *                 在时间轴上的取值序列"
 *   2) 协方差   : 计算帧与帧之间的协方差矩阵 column×column
 *                 （哪几帧的变化模式彼此相关）
 *   3) 幂法求主特征向量 + 投影：找出方差最大的方向（主成分），
 *                 把矩阵投影上去 → 得到指纹 output[cols]
 * ============================================================================
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/param.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_radar.h"

#define ITRS_MAX 30          /* 幂法最大迭代次数：防止不收敛死循环 */
#define PRECISION 0.0001     /* 幂法收敛精度：特征值相邻两次差小于此值即停 */

#define DOUBLE_CALCULATE     /* 用 double 做特征值运算，减少浮点累计误差 */

static const char *TAG = "algorithm";

/**
 * 计算协方差矩阵（未减均值的"原始二阶矩"版本）
 *
 * 输入 matrix[row][column]：行=子载波，列=帧
 * 输出 matrix_cov[column][column]：帧×帧 的协方差
 *
 * 公式：cov[i][j] = (1/(row*column)) * Σ_k matrix[k][i] * matrix[k][j]
 * 注意：这里没有减去均值（不是标准协方差，而是缩放后的 X^T·X），
 *       对"找最大方差方向"这个目的来说等价（均值项只是加常数），
 *       省一次遍历，嵌入式上更省时间。
 */
esp_err_t compute_cov(uint32_t row, uint32_t column,
                      const float matrix[row][column],
                      float matrix_cov[column][column])
{
    uint32_t zoom_out = row * column;   /* 归一化分母：总元素个数（旧版实现如此） */

    for (int i = 0; i < column; i++) {          /* 遍历列 i（帧 i） */
        for (int j = 0; j < i + 1; j++) {       /* 只算下三角 j<=i（对称矩阵） */
            for (int k = 0; k < row; k++) {     /* 对所有子载波行 k 累加乘积 */
                matrix_cov[i][j] += matrix[k][i] * matrix[k][j];  /* 帧 i·帧 j 内积 */
            }

            matrix_cov[i][j] /= zoom_out;       /* 归一化，防止数值随帧数无限增大 */

            if (i != j) {
                matrix_cov[j][i] = matrix_cov[i][j];  /* 对称位置直接复制（省一半计算量） */
            }
        }
    }

    return ESP_OK;
}

// #define PYTHON
// #define MATRIX_PRINT
#ifdef MATRIX_PRINT

/* 调试用：打印矩阵内容（可按 Python 列表格式打印，便于与 numpy 对拍） */
static void matrix_print(int row, int column, const float data[row][column])
{
    printf("===  matrix, row: %d, column: %d\n", row, column);

#ifdef PYTHON
    printf("data = [\n");

    for (int i = 0; i < row; i++) {
        printf("\t[");

        for (int j = 0; j < column; j++) {
            printf("\t%.2f,", data[i][j]);
        }

        printf("\b],\n");           /* \b 回退删掉行尾多余逗号 */
    }

    printf("]\n");

    printf("const float matrix_0[%d][%d] = {\n", row, column);

    for (int i = 0; i < row; i++) {
        printf("\t{");

        for (int j = 0; j < column; j++) {
            printf("\t%.2f,", data[i][j]);
        }

        printf("\b},\n");
    }

    printf("};\n");
#else

#endif
}
#endif

/**
 * 幂法（Power Iteration）求协方差矩阵的主特征向量
 *
 * 原理：任取初始向量 v，反复做 v <- M·v 并归一化，v 会收敛到
 *       最大特征值对应的特征向量（主成分方向）。
 * 这里用的"归一化"是除以当前最大分量（无穷范数），数值上更稳。
 *
 * 参数：matrix      协方差矩阵 [column][column]
 *       itrs_max    最大迭代次数（ITRS_MAX=30）
 *       delta_min   收敛阈值（PRECISION=0.0001）
 *       eigenvector 输出的主特征向量 [column]
 *
 * 返回：迭代达到上限仍未收敛 → ESP_FAIL（调用方放弃本次指纹计算）
 */
static esp_err_t matrix_power_method_eigen(uint32_t row, uint32_t column, const float matrix[row][column],
                                           uint32_t itrs_max, float delta_min, float *eigenvector)
{
    uint32_t iterate = 0;
#ifdef DOUBLE_CALCULATE
    double eigenvalue = 1.0;            /* 当前最大分量值（作为缩放因子） */
    double eigenvalue_last = 0;         /* 上次的最大分量值，用于判断收敛 */
    double *eigenvalue_list = RADAR_MALLOC_RETRY(column * sizeof(double));  /* M·v 结果暂存 */
#else
    float eigenvalue = 1.0;
    float eigenvalue_last = 0;
    float *eigenvalue_list = RADAR_MALLOC_RETRY(column * sizeof(float));
#endif

    for (int i = 0; i < column; i++) {
        eigenvector[i] = 1;             /* 初始向量取全 1（经典幂法起点） */
    }

    /* 迭代条件：特征值变化还没收敛 且 没超过最大迭代次数 */
    for (iterate = 0; fabs(eigenvalue - eigenvalue_last) > delta_min && iterate < itrs_max;
            iterate++) {
        eigenvalue_last = eigenvalue;   /* 记录上一轮的缩放因子 */
        eigenvalue      = 0;            /* 本轮重新找最大分量 */

        for (int i = 0; i < row; i++) {         /* 矩阵-向量乘法：list = M·v */
            eigenvalue_list[i] = 0;

            for (int j = 0; j < column; j++) {
                eigenvalue_list[i] += matrix[i][j] * eigenvector[j];
            }

            if (eigenvalue_list[i] > eigenvalue) {   /* 顺便找出最大分量 */
                eigenvalue =  eigenvalue_list[i];
            }
        }

        for (int i = 0; i < row; i++) {
            eigenvector[i] = eigenvalue_list[i] / eigenvalue;  /* 除以最大分量归一化 */
        }
    }

    RADAR_FREE(eigenvalue_list);

    ESP_LOGD(TAG, "iterate: %ld, itrs_max: %ld", iterate, itrs_max);
    return (iterate == itrs_max) ? ESP_FAIL : ESP_OK;   /* 跑满迭代=没收敛=失败 */
}

/**
 * 主入口：计算一个 CSI 时间窗口的 PCA 指纹
 *
 * 参数：
 *   cols    子载波数（矩阵的"行"数，也是输出指纹长度）
 *   row_0   环形缓冲中第一段连续帧数（begin..缓冲尾）
 *   data_0  第一段数据 [row_0][cols]，每行=一帧的子载波振幅
 *   row_1   第二段帧数（缓冲头..end，环形回绕部分）
 *   data_1  第二段数据 [row_1][cols]
 *   output  输出指纹 [cols]，每子载波一个值
 *
 * 环形缓冲会被拆成两段传入（这是 PCA 接口带 data_0/data_1 两段的原因）
 */
esp_err_t pca(uint32_t cols,
              uint32_t row_0, const float data_0[row_0][cols],
              uint32_t row_1, const float data_1[row_1][cols],
              float output[cols])
{
    esp_err_t ret   = ESP_OK;
    uint32_t row    = cols;             /* 组矩阵后：行 = 子载波数 */
    uint32_t column = row_0 + row_1;    /* 组矩阵后：列 = 窗口内总帧数 */
    // int64_t start_time = esp_timer_get_time();

    // printf("cols: %d, row_0: %d, row_1: %d\n", cols, row_0, row_1);

    /* 一次性申请三块内存：矩阵、协方差、特征向量（失败则直接退出） */
    float (*matrix)[column] = RADAR_MALLOC_RETRY(row * column * sizeof(float));

    /* 组装矩阵：matrix[子载波][帧] = data[帧][子载波]（即转置） */
    for (int i = 0; i < row; i++) {             /* 遍历子载波 i */
        for (int j = 0; j < row_0; j++) {       /* 第一段（环形缓冲后半） */
            matrix[i][j] = data_0[j][i];
        }

        for (int j = 0; j < row_1; j++) {       /* 第二段（环形缓冲回绕部分） */
            matrix[i][j + row_0] = data_1[j][i];
        }
    }

    float (*matrix_cov)[column] = RADAR_MALLOC_RETRY(column * column * sizeof(float));
    memset(matrix_cov, 0, column * column * sizeof(float));   /* 协方差先清零（累加目标） */

    compute_cov(row, column, matrix, matrix_cov);   /* 第 2 步：帧×帧协方差 */

    float *eigenvector = RADAR_MALLOC_RETRY(column * sizeof(float));
    ret = matrix_power_method_eigen(column, column, matrix_cov,
                                    ITRS_MAX, PRECISION, eigenvector);  /* 第 3 步：主特征向量 */

    if (ret != ESP_OK) {
        // ESP_LOGW(TAG, "[%d] fail", __LINE__);
        // matrix_print(row_0, cols, data_0);
        // matrix_print(row_1, cols, data_1);

        goto EXIT;      /* 未收敛：放弃本次指纹（output 保持上一次内容，由调用方丢弃） */
    }

    /* 投影：output[i] = Σ_j matrix[i][j]·eigenvector[j] / column
     * 把每个子载波在时间轴上的序列，往主成分方向做加权平均，
     * 得到该子载波在主成分上的"得分"——这就是指纹的第 i 个元素 */
    for (int i = 0; i < row; i++) {
        output[i] = 0;

        for (int j = 0; j < column; j++) {
            output[i] +=  matrix[i][j] * eigenvector[j];
        }

        output[i] /= column;    /* 除以帧数=取平均，让量纲不随窗口大小变化 */
    }

    // printf("pca = [");
    // for (int i = 0; i < row; i++) {
    //     printf("\t%.2f,", output[i]);
    // }
    // printf("\b]\n");

EXIT:
    RADAR_FREE(matrix);
    RADAR_FREE(matrix_cov);
    RADAR_FREE(eigenvector);

    // ESP_LOGD(TAG, "[%s, %d] spend time: %lld", __FUNCTION__, __LINE__, esp_timer_get_time() - start_time);
    return ret;
}
