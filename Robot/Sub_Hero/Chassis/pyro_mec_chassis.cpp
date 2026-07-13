#include "pyro_mec_chassis.h"

#include "pyro_algo_common.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_referee.h"
#include "mec_config.h"
#include <algorithm>
#include <arm_math.h>
#include <cmath>

namespace pyro
{

namespace
{
constexpr uint16_t CAP_EXTRA_POWER_ENABLE_CV = 1900;
constexpr uint16_t CAP_EXTRA_POWER_DISABLE_CV = 1750;
constexpr float CAP_EXTRA_POWER_W = 80.0f;

constexpr uint32_t SUPERCAP_ENABLE_DELAY_TICKS = 1000;
constexpr uint32_t SUPERCAP_REFRESH_TICKS = 10;
constexpr uint32_t SUPERCAP_DISABLE_DEBOUNCE_TICKS = 30;
} // namespace

mec_chassis_t::mec_chassis_t() : module_base_t("mec_chassis")
{
    _ctx = {};
}

status_t mec_chassis_t::_init()
{
    _ctx.motor = _module_deps.motor_deps;
    _ctx.pid   = _module_deps.pid_deps;

    _kinematics = new mecanum_kin_t(WHEELBASE, TRACK_WIDTH);

    _ctx.powermeter = new powermeter_drv_t(0x212, can_hub_t::can2);
    _ctx.powermeter->init();

    _power_control_init();

    return PYRO_OK;
}

mec_chassis_t::mec_context_t &mec_chassis_t::get_ctx()
{
    return _ctx;
}

void mec_chassis_t::_power_control_init()
{
    power_fit_params_t params;

    params.k1    = 0.03912453f;
    params.k2    = 0.06985056f;
    params.k3    = 0.00001723f;
    params.k4    = 0.00917522f;
    params.k5    = 0.75000000f;
    params.alpha = 0.00393f;

    for (auto *&node : _ctx.power_motor_data)
    {
        node = power_controller_t::get_instance().register_motor(params);
    }

    power_controller_t::get_instance().config_buffer_loop(40.0f);
}

void mec_chassis_t::_update_feedback()
{
    for (auto *motor : _ctx.motor.wheels)
    {
        motor->update_feedback();
    }
    _ctx.motor.yaw->update_feedback();

    for (int i = 0; i < 4; i++)
    {
        _ctx.data.current_wheel_rpm[i] =
            radps_to_rpm(_ctx.motor.wheels[i]->get_current_rotate() *
                         dji_m3508_motor_drv_t::reciprocal_reduction_ratio);
        _ctx.data.current_wheel_torque[i] =
            _ctx.motor.wheels[i]->get_current_torque();
        _ctx.data.current_wheel_temp[i] =
            _ctx.motor.wheels[i]->get_temperature();
        _ctx.data.wheel_online[i] = _ctx.motor.wheels[i]->is_online();
    }

    mecanum_kin_t::wheel_speeds_t current_speed{};
    current_speed.fl =
        rpm_to_mps(_ctx.data.current_wheel_rpm[0], WHEEL_RADIUS);
    current_speed.fr =
        rpm_to_mps(-_ctx.data.current_wheel_rpm[1], WHEEL_RADIUS);
    current_speed.bl =
        rpm_to_mps(_ctx.data.current_wheel_rpm[2], WHEEL_RADIUS);
    current_speed.br =
        rpm_to_mps(-_ctx.data.current_wheel_rpm[3], WHEEL_RADIUS);
    _kinematics->compute_odometry(current_speed, _ctx.data.real_vx,
                                  _ctx.data.real_vy, _ctx.data.real_wz);

    float current_angle =
        _ctx.motor.yaw->get_current_position() - YAW_OFFSET_RAD;
    current_angle               = loop_fp32_constrain(current_angle, -PI, PI);
    _ctx.data.current_yaw_error = loop_fp32_constrain(current_angle, -PI, PI);

    auto *referee = referee_drv_t::get_instance();
    const auto &ref_data = referee->get_data();

    _ctx.data.buffer_energy =
        ref_data.power_heat.buffer_energy;
    _ctx.data.total_predicted_power =
        power_controller_t::get_instance().get_total_predicted_power();

    _ctx.supercap_cmd.power_referee = 0.0f;
    _ctx.supercap_cmd.power_limit_referee =
        ref_data.robot_status.chassis_power_limit;
    _ctx.supercap_cmd.power_buffer_limit_referee = 60.0f;
    _ctx.supercap_cmd.power_buffer_referee =
        ref_data.power_heat.buffer_energy;
    _ctx.supercap_cmd.kill_chassis_user = 0;
    _ctx.supercap_cmd.speed_up_user_now = 0;

    _ctx.cap_feedback = supercap_drv_t::get_instance()->get_feedback();
    _ctx.powermeter->get_data(_ctx.powermeter_feedback);
}

void mec_chassis_t::_kinematics_solve()
{
    float follow_wz =
        _ctx.pid.follow_yaw_pid->calculate(_ctx.data.current_yaw_error, 0.0f);

    float final_wz      = follow_wz;
    const float theta   = -_ctx.data.current_yaw_error;
    const float c_theta = arm_cos_f32(theta);
    const float s_theta = arm_sin_f32(theta);

    float vx_chassis = _ctx.cmd->vx * c_theta + _ctx.cmd->vy * s_theta;
    float vy_chassis = -_ctx.cmd->vx * s_theta + _ctx.cmd->vy * c_theta;

    if (_ctx.cmd->wz != 0.0f)
    {
        final_wz = _ctx.cmd->wz;
    }
    else if (std::abs(final_wz) > 3.0f)
    {
        vx_chassis *= 0.22f;
        vy_chassis *= 0.22f;
    }

    int offline_count  = 0;
    auto missing_wheel = mecanum_kin_t::missing_mec_e::NONE;

    if (!_ctx.data.wheel_online[0])
    {
        offline_count++;
        missing_wheel = mecanum_kin_t::missing_mec_e::FL;
    }
    if (!_ctx.data.wheel_online[1])
    {
        offline_count++;
        missing_wheel = mecanum_kin_t::missing_mec_e::FR;
    }
    if (!_ctx.data.wheel_online[2])
    {
        offline_count++;
        missing_wheel = mecanum_kin_t::missing_mec_e::BL;
    }
    if (!_ctx.data.wheel_online[3])
    {
        offline_count++;
        missing_wheel = mecanum_kin_t::missing_mec_e::BR;
    }

    if (offline_count >= 2)
    {
        vx_chassis    = 0.0f;
        vy_chassis    = 0.0f;
        final_wz      = 0.0f;
        missing_wheel = mecanum_kin_t::missing_mec_e::NONE;
    }

    const auto wheel_speeds =
        _kinematics->solve(vx_chassis, vy_chassis, final_wz, missing_wheel);

    _ctx.data.target_wheel_rpm[0] =
        mps_to_rpm(wheel_speeds.fl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[1] =
        -mps_to_rpm(wheel_speeds.fr, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[2] =
        mps_to_rpm(wheel_speeds.bl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[3] =
        -mps_to_rpm(wheel_speeds.br, WHEEL_RADIUS);
}

void mec_chassis_t::_mecanum_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_wheel_torque[i] = _ctx.pid.wheel_pid[i]->calculate(
            _ctx.data.target_wheel_rpm[i], _ctx.data.current_wheel_rpm[i]);
    }
}

void mec_chassis_t::_power_control()
{
    for (int i = 0; i < 4; i++)
    {
        if (_ctx.power_motor_data[i] == nullptr)
        {
            continue;
        }

        _ctx.power_motor_data[i]->target_cmd =
            _ctx.data.out_wheel_torque[i];
        _ctx.power_motor_data[i]->rpm  = _ctx.data.current_wheel_rpm[i];
        _ctx.power_motor_data[i]->temp = _ctx.data.current_wheel_temp[i];
    }

    auto *referee = referee_drv_t::get_instance();
    const auto &ref_data = referee->get_data();
    static bool cap_extra_power_enabled = false;
    if (_ctx.cap_feedback.vot_cap >= CAP_EXTRA_POWER_ENABLE_CV)
    {
        cap_extra_power_enabled = true;
    }
    else if (_ctx.cap_feedback.vot_cap <= CAP_EXTRA_POWER_DISABLE_CV)
    {
        cap_extra_power_enabled = false;
    }

    const float cap_extra_power =
        cap_extra_power_enabled ? CAP_EXTRA_POWER_W : 0.0f;

    power_controller_t::get_instance().solve(
        ref_data.robot_status.chassis_power_limit,
        ref_data.power_heat.buffer_energy, cap_extra_power);

    for (int i = 0; i < 4; i++)
    {
        if (_ctx.power_motor_data[i] != nullptr)
        {
            _ctx.data.out_wheel_torque[i] =
                _ctx.power_motor_data[i]->safe_cmd;
        }
    }
}

void mec_chassis_t::_supercap_control()
{
    static bool cap_enabled = false;
    static uint32_t enable_timer = 0;
    static uint32_t refresh_timer = 0;
    static uint32_t disable_timer = 0;

    const bool current_status =
        referee_drv_t::get_instance()
            ->get_data()
            .robot_status.power_management_chassis_output;

    if (current_status)
    {
        disable_timer = 0;

        if (!cap_enabled)
        {
            refresh_timer = 0;
            if (++enable_timer >= SUPERCAP_ENABLE_DELAY_TICKS)
            {
                enable_timer             = 0;
                cap_enabled              = true;
                _ctx.supercap_cmd.use_cap = 1;
                supercap_drv_t::get_instance()->send_cmd(_ctx.supercap_cmd);
            }
        }
        else if (++refresh_timer >= SUPERCAP_REFRESH_TICKS)
        {
            refresh_timer           = 0;
            _ctx.supercap_cmd.use_cap = 1;
            supercap_drv_t::get_instance()->send_cmd(_ctx.supercap_cmd);
        }
    }
    else
    {
        enable_timer  = 0;
        refresh_timer = 0;

        if (cap_enabled &&
            ++disable_timer >= SUPERCAP_DISABLE_DEBOUNCE_TICKS)
        {
            disable_timer            = 0;
            cap_enabled              = false;
            _ctx.supercap_cmd.use_cap = 0;
            supercap_drv_t::get_instance()->send_cmd(_ctx.supercap_cmd);
        }
        else if (!cap_enabled)
        {
            disable_timer = 0;
        }
    }
}

void mec_chassis_t::_send_motor_command() const
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.wheels[i]->send_torque(_ctx.data.out_wheel_torque[i]);
        // _ctx.motor.wheels[i]->send_torque(0);
    }
}

void mec_chassis_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
    {
        _main_fsm.change_state(&_state_active);
    }
    else
    {
        _main_fsm.change_state(&_state_passive);
    }

    _main_fsm.execute(this);
}

} // namespace pyro
