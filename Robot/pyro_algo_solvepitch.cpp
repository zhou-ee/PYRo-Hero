#include "pyro_algo_solvepitch.h"
#include <cmath>  // 需要这个

namespace pyro {
namespace kinematics {

std::optional<float> solveIdealPitch(float delta_x, float delta_y,
                                     float delta_z, float param)
{
    // 计算水平距离
    float horizontal_dist = std::sqrt(delta_x * delta_x + delta_y * delta_y);

    // 检查是否可解
    if (horizontal_dist < 0.001f || param < 0.001f) {
        return std::nullopt;  // 无法解算
    }

    // 俯仰角解算（atan2得到角度）
    float pitch = std::atan2(delta_z, horizontal_dist);

    // 对俯仰角进行约束（-45° 到 45°）
    const float MAX_PITCH = 45.0f * 3.141592653589793f / 180.0f;
    if (std::abs(pitch) > MAX_PITCH) {
        return std::nullopt;  // 超出范围
    }

    return pitch;
}

float mps_to_rpm(const float mps, const float radius)
{
    if (radius < 1.0e-4f) {
        return 0.0f;
    }
    return (mps / radius) * 9.5492966f;
}

float rpm_to_mps(const float rpm, const float radius)
{
    return rpm * radius / 9.5492966f;
}

} // namespace kinematics
} // namespace pyro