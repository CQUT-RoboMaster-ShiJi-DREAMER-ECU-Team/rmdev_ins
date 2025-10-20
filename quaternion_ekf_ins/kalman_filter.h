/**
 ******************************************************************************
 * @file    kalman_filter.h
 * @author  Wang Hongxi, DuYicheng
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   C implementation of kalman filter
 * @attention 目前这个文件内的函数仅供 rmdev_ins 模块内部使用
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "arm_math.h"
#include "stdint.h"

#include "emdevif/attributes_and_useful_macros.h"

EMDEVIF_EXTERN_C_BEGIN

#define KALMAN_FILTER_MAT              arm_matrix_instance_f32
#define KALMAN_FILTER_Matrix_Init      arm_mat_init_f32
#define KALMAN_FILTER_Matrix_Add       arm_mat_add_f32
#define KALMAN_FILTER_Matrix_Subtract  arm_mat_sub_f32
#define KALMAN_FILTER_Matrix_Multiply  arm_mat_mult_f32
#define KALMAN_FILTER_Matrix_Transpose arm_mat_trans_f32
#define KALMAN_FILTER_Matrix_Inverse   arm_mat_inverse_f32

typedef struct kf_t {
    float* FilteredValue;
    float* MeasuredVector;
    float* ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t* MeasurementMap;       // 量测与状态的关系 how measurement relates to the state
    float* MeasurementDegree;      // 测量值对应H矩阵元素值 elements of each measurement in H
    float* MatR_DiagonalElements;  // 量测方差 variance for each measurement
    float* StateMinVariance;       // 最小方差 避免方差过度收敛 suppress filter excessive convergence
    uint8_t* temp;

    // 配合用户定义函数使用,作为标志位用于判断是否要跳过标准KF中五个环节中的任意一个
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    // definiion of struct mat: rows & cols & pointer to vars
    KALMAN_FILTER_MAT xhat;       // x(k|k)
    KALMAN_FILTER_MAT xhatminus;  // x(k|k-1)
    KALMAN_FILTER_MAT u;          // control vector u
    KALMAN_FILTER_MAT z;          // measurement vector z
    KALMAN_FILTER_MAT P;          // covariance matrix P(k|k)
    KALMAN_FILTER_MAT Pminus;     // covariance matrix P(k|k-1)
    KALMAN_FILTER_MAT F, FT;      // state transition matrix F FT
    KALMAN_FILTER_MAT B;          // control matrix B
    KALMAN_FILTER_MAT H, HT;      // measurement matrix H
    KALMAN_FILTER_MAT Q;          // process noise covariance matrix Q
    KALMAN_FILTER_MAT R;          // measurement noise covariance matrix R
    KALMAN_FILTER_MAT K;          // kalman gain  K
    KALMAN_FILTER_MAT S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    // 用户定义函数,可以替换或扩展基准KF的功能
    void (*User_Func0_f)(struct kf_t* kf);
    void (*User_Func1_f)(struct kf_t* kf);
    void (*User_Func2_f)(struct kf_t* kf);
    void (*User_Func3_f)(struct kf_t* kf);
    void (*User_Func4_f)(struct kf_t* kf);
    void (*User_Func5_f)(struct kf_t* kf);
    void (*User_Func6_f)(struct kf_t* kf);

    // 用于用户自定义函数的其他参数，可用于保存上下文信息
    void* user_external_arg;

    // 矩阵存储空间指针
    float* xhat_data;
    float* xhatminus_data;
    float* u_data;
    float* z_data;
    float* P_data;
    float* Pminus_data;
    float* F_data;
    float* FT_data;
    float* B_data;
    float* H_data;
    float* HT_data;
    float* Q_data;
    float* R_data;
    float* K_data;

    float* S_data;
    float* temp_matrix_data;
    float* temp_matrix_data1;
    float* temp_vector_data;
    float* temp_vector_data1;
} KalmanFilter_t;

void Kalman_Filter_Init(KalmanFilter_t* kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
void Kalman_Filter_DeInit(KalmanFilter_t* kf);
void Kalman_Filter_Measure(KalmanFilter_t* kf);
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t* kf);
void Kalman_Filter_PminusUpdate(KalmanFilter_t* kf);
void Kalman_Filter_SetK(KalmanFilter_t* kf);
void Kalman_Filter_xhatUpdate(KalmanFilter_t* kf);
void Kalman_Filter_P_Update(KalmanFilter_t* kf);
float* Kalman_Filter_Update(KalmanFilter_t* kf);

EMDEVIF_EXTERN_C_END

#endif  // KALMAN_FILTER_H
