/**
 * @file ins.cppm
 * @author DuYicheng
 * @date 2025-10-18
 * @brief 姿态解算模块 - 基本定义
 */

module;

#include <cstdint>

export module rmdev.ins:base;

export namespace rmdev {

/**
 * 姿态解算的算法
 */
enum class InsAlgorithm : uint_fast8_t {
    QuaternionEKF = 0  ///< 基于四元数扩展卡尔曼滤波的姿态更新算法（Wang Hongxi,
                       ///< https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example）
};

/**
 * 姿态解算
 * @note 这个空类不进行任何操作，具体实现在根据 @ref algorithm 的偏特化中进行。
 * 特化的版本需要提供以下公有成员：
 *     -- 类型别名 ScaleType。值始终等于第一个模板参数，即 using ScaleType = T;
 *     -- 方法 void init() 用于实例初始化
 *     -- 方法 void deInit() 用于销毁实例
 *     -- 方法 void insUpdate(rmdev::Imu<ScaleType>& current_imu_data) 用于实现姿态解算数据更新
 * @tparam T 数据类型
 * @tparam algorithm 实现的算法，参数取 @ref rmdev::InsAlgorithm
 */
template<typename T = float, InsAlgorithm algorithm = InsAlgorithm::QuaternionEKF>
class Ins;

}  // namespace rmdev
