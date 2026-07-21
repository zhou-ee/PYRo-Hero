#pragma once
#include <optional>
#include <cmath>

namespace pyro {
namespace kinematics {

/**
 * @brief 解算理想俯仰角
 * @param delta_x X方向距离差
 * @param delta_y Y方向距离差
 * @param delta_z Z方向距离差
 * @param param 参数（射速/子弹速度等）
 * @return 俯仰角（弧度），如果无法解算则返回nullopt
 */
std::optional<float> solveIdealPitch(float delta_x, float delta_y,
                                     float delta_z, float param);

/**
 * @brief 线速度转转速
 */
float mps_to_rpm(const float mps, const float radius);

/**
 * @brief 转速转线速度
 */
float rpm_to_mps(const float rpm, const float radius);

} // namespace kinematics
} // namespace pyro