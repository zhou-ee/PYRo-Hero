/**
 * @file pyro_leso_fric_controller.h
 * @brief LESO + 单P 摩擦轮控制器
 *
 * 设计原则：
 * 1. 控制输出为电流指令 (A)
 * 2. LESO 物理模型: dot(omega) = b0 * I + f, 其中 b0 = KT / J
 * 3. 物理扭矩仅用于监控/日志，不作为控制量
 */

#ifndef __PYRO_LESO_FRIC_CONTROLLER_H__
#define __PYRO_LESO_FRIC_CONTROLLER_H__

#include "pyro_algo_leso.h"
#include <array>
#include <cstdint>
#include "pyro_algo_pid.h"

namespace pyro
{

/**
 * @brief 单个摩擦轮的 LESO 配置（电流控制模式）
 */
struct fric_leso_per_wheel_config_t
{
    // === 物理参数 ===
    float J;            // 转动惯量 (kg·m²)
    float KT;           // 力矩常数 (N·m / A)
    float b0;           // 控制增益 = KT / J (rad/(s²·A))

    // === LESO 参数 ===
    float omega_o;      // 观测器带宽 (rad/s)
    float z_limit;      // 扰动限幅 (rad/s²) - 加速度域

    // === 控制器参数 ===
    float kp;           // 比例增益 (1/s) —— 传给 pid_t 的 kp
    float kd;           // 微分增益 (无量纲) —— 传给 pid_t 的 kd
    float ki;           // 积分增益 (1/s²) —— 微量，用于补偿残余误差
    float integral_limit; // 积分限幅 (A)
    float current_max;  // 最大电流指令 (A)
};

/**
 * @brief 4 个摩擦轮的 LESO 配置
 */
struct fric_leso_config_t
{
    fric_leso_per_wheel_config_t wheel[4];
};

/**
 * @brief 4 个摩擦轮的 LESO + 单P 控制器
 *
 * 物理量纲对齐版本：
 * - 内部计算使用加速度 (rad/s²) 和电流 (A)
 * - 输出为电流指令 (A)，直接发给电调
 * - 物理扭矩 = KT * current_cmd，仅用于监控
 */
class fric_leso_controller_t
{
public:
    explicit fric_leso_controller_t(const fric_leso_config_t& config);
    ~fric_leso_controller_t();

    void reset();
    void set_params(float omega_o, float kp);
    void set_wheel_params(uint32_t index, float omega_o, float kp);

    /**
     * @brief 计算 4 个摩擦轮的控制输出
     *
     * @param target   目标角速度 (rad/s)
     * @param feedback 实际角速度 (rad/s)
     * @param output   输出电流指令 (A)
     * @param dt       采样周期 (s)
     */
    void compute(const float target[4],
                 const float feedback[4],
                 float output[4],
                 float dt);

    // Getters
    float get_disturbance(uint32_t index) const;  // 扰动加速度 (rad/s²)
    float get_z(uint32_t index, uint32_t state) const;
    float get_b0(uint32_t index) const;
    float get_kp(uint32_t index) const;
    float get_current_cmd(uint32_t index) const;  // 当前电流指令 (A)
    float get_torque_physical(uint32_t index) const;  // 物理扭矩 (N·m)

private:
    fric_leso_config_t _config;
    leso_t<2>* _leso[4];
    pid_t* _pid[4];                  // 4 个 PD/PID 控制器（复用现有库）
    float _last_output[4] = {0.0f};   // 上一周期电流指令 (A)
    float _last_torque[4] = {0.0f};   // 上一周期物理扭矩 (N·m) - 监控用
};

} // namespace pyro

#endif // __PYRO_LESO_FRIC_CONTROLLER_H__