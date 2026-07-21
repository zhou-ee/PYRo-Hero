/**
 * @file pyro_leso_fric_controller.cpp
 * @brief LESO + 单P 摩擦轮控制器实现
 */

#include "pyro_leso_fric_controller.h"
#include <algorithm>
#include <cmath>

namespace pyro
{

fric_leso_controller_t::fric_leso_controller_t(const fric_leso_config_t& config)
    : _config(config)
{
    for (int i = 0; i < 4; i++)
    {
        const auto& w = _config.wheel[i];
        _leso[i] = new leso_t<2>(w.omega_o, w.b0, w.z_limit);
        _last_output[i] = 0.0f;
        _last_torque[i] = 0.0f;
    }
}

fric_leso_controller_t::~fric_leso_controller_t()
{
    for (int i = 0; i < 4; i++)
    {
        delete _leso[i];
        _leso[i] = nullptr;
    }
}

void fric_leso_controller_t::reset()
{
    for (int i = 0; i < 4; i++)
    {
        if (_leso[i] != nullptr)
        {
            _leso[i]->clear();
        }
        _last_output[i] = 0.0f;
        _last_torque[i] = 0.0f;
    }
}

void fric_leso_controller_t::set_params(float omega_o, float kp)
{
    for (int i = 0; i < 4; i++)
    {
        _config.wheel[i].omega_o = omega_o;
        _config.wheel[i].kp = kp;
        if (_leso[i] != nullptr)
        {
            _leso[i]->set_params(omega_o, _config.wheel[i].b0);
        }
    }
}

void fric_leso_controller_t::set_wheel_params(uint32_t index, float omega_o, float kp)
{
    if (index >= 4) return;

    _config.wheel[index].omega_o = omega_o;
    _config.wheel[index].kp = kp;
    if (_leso[index] != nullptr)
    {
        _leso[index]->set_params(omega_o, _config.wheel[index].b0);
    }
}

void fric_leso_controller_t::compute(const float target[4],
                                     const float feedback[4],
                                     float output[4],
                                     float dt)
{
    if (dt < 1e-9f)
    {
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        if (_leso[i] == nullptr)
        {
            output[i] = 0.0f;
            continue;
        }

        const auto& w = _config.wheel[i];

        // =========================================================
        // Step 1: 用上一拍实际输出的电流更新 LESO
        //         物理模型: dot(omega) = b0 * I + f
        //         其中 b0 = KT / J
        // =========================================================
        _leso[i]->update(feedback[i], _last_output[i]);

        // =========================================================
        // Step 2: 获取当前状态估计（已对齐到当前时刻）
        // =========================================================
        float estimated_speed = _leso[i]->get_z(0);      // z1: 估计速度 (rad/s)
        float disturbance = _leso[i]->get_disturbance(); // z3: 扰动加速度 (rad/s²)

        // =========================================================
        // Step 3: 计算电流指令 (A)
        //         控制律: I = (kp * error - disturbance) / b0
        //         物理含义: 电流 = (期望加速度 - 扰动加速度) / 电流->加速度增益
        // =========================================================
        float error = target[i] - estimated_speed;       // 速度误差 (rad/s)
        float u0 = w.kp * error;                         // 期望加速度 (rad/s²)
        float current_cmd = (u0 - disturbance) / w.b0;   // 电流指令 (A)
        current_cmd = std::clamp(current_cmd, -w.current_max, w.current_max);

        // =========================================================
        // Step 4: (可选) 计算物理扭矩用于监控/日志
        //         物理扭矩 T = KT * I (N·m)
        // =========================================================
        float torque_physical = w.KT * current_cmd;

        // =========================================================
        // Step 5: 输出电流指令给电调
        // =========================================================
        output[i] = current_cmd;

        // =========================================================
        // Step 6: 保存当前值供下一周期使用
        // =========================================================
        _last_output[i] = current_cmd;
        _last_torque[i] = torque_physical;
    }
}

float fric_leso_controller_t::get_disturbance(uint32_t index) const
{
    if (index < 4 && _leso[index] != nullptr)
    {
        return _leso[index]->get_disturbance();
    }
    return 0.0f;
}

float fric_leso_controller_t::get_z(uint32_t index, uint32_t state) const
{
    if (index < 4 && _leso[index] != nullptr)
    {
        return _leso[index]->get_z(state);
    }
    return 0.0f;
}

float fric_leso_controller_t::get_b0(uint32_t index) const
{
    if (index < 4)
    {
        return _config.wheel[index].b0;
    }
    return 0.0f;
}

float fric_leso_controller_t::get_kp(uint32_t index) const
{
    if (index < 4)
    {
        return _config.wheel[index].kp;
    }
    return 0.0f;
}

float fric_leso_controller_t::get_current_cmd(uint32_t index) const
{
    if (index < 4)
    {
        return _last_output[index];
    }
    return 0.0f;
}

float fric_leso_controller_t::get_torque_physical(uint32_t index) const
{
    if (index < 4)
    {
        return _last_torque[index];
    }
    return 0.0f;
}

} // namespace pyro