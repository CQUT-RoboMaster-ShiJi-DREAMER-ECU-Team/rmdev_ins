/**
 * @file ins.cppm
 * @brief 姿态解算模块
 */

module;

#include <cstdint>
#include <cstring>

#include <type_traits>

#include "emdevif/core/fatal_handler.h"

#include "rmdev/ins/detail/quaternion_ekf_ins/kalman_filter.hpp"
#include "rmdev/ins.hpp"

export module rmdev.ins;

export namespace rmdev {
    using ::rmdev::InsAlgorithm;
    using ::rmdev::Ins;
}
