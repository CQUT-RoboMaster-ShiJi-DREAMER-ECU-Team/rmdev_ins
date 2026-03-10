/**
 * @file rmdev_matrix_adapter.hpp
 * @brief
 */

#pragma once

#include "arm_math.h"

// #include "rmdev/matrix.hpp"

namespace rmdev::ins::detail::qekf_ins {

// using KALMAN_FILTER_MAT = rmdev::Matrix<float>;

#define KALMAN_FILTER_MAT              arm_matrix_instance_f32
#define KALMAN_FILTER_Matrix_Init      arm_mat_init_f32
#define KALMAN_FILTER_Matrix_Add       arm_mat_add_f32
#define KALMAN_FILTER_Matrix_Subtract  arm_mat_sub_f32
#define KALMAN_FILTER_Matrix_Multiply  arm_mat_mult_f32
#define KALMAN_FILTER_Matrix_Transpose arm_mat_trans_f32
#define KALMAN_FILTER_Matrix_Inverse   arm_mat_inverse_f32

}  // namespace rmdev::ins::detail::qekf_ins
