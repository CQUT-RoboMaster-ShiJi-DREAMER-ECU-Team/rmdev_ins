/**
 * @file ins.cppm
 * @module rmdev.ins:base
 * @author DuYicheng
 * @date 2025-10-18
 * @brief 姿态解算模块 - 基本定义
 */

module;

#include <cstdint>

export module rmdev.ins:base;

export namespace rmdev {

enum class InsAlgorithm : uint_fast8_t {
    QuaternionEKF = 0
};

template<typename T = float, InsAlgorithm algorithm = InsAlgorithm::QuaternionEKF>
class Ins;

}  // namespace rmdev
