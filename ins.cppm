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

#define EMDEVIF_MODULE_INTERFACE_UNIT

export module rmdev.ins;

import rmdev.device_model.sensor.imu;
import rmdev.matrix;
import emdevif.core.error_handler;
import emdevif.core.concepts;
import emdevif.timeline;

#ifdef __clang__
    #pragma clang diagnostic ignored "-Winclude-angled-in-module-purview"
#endif

#include "rmdev/ins.hpp"
