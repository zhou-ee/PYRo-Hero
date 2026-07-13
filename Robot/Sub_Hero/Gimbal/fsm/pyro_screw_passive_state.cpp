#include "pyro_screw_gimbal.h"
#include "pyro_algo_common.h"

namespace pyro
{
void screw_gimbal_t::fsm_passive_t::on_enter(owner *owner)
{
    owner->_ctx.pid.pitch_pos->clear();
    owner->_ctx.pid.pitch_spd->clear();
    owner->_ctx.pid.yaw_pos->clear();
    owner->_ctx.pid.yaw_spd->clear();

    owner->_ctx.motor.pitch->disable();
    owner->_ctx.motor.yaw->disable();

    _last_calib_flag = owner->_ctx.cmd->trigger_calibration;
    change_state(&idle_state);
}

void screw_gimbal_t::fsm_passive_t::on_execute(owner *owner)
{
    owner->_ctx.data.out_pitch_torque = 0;
    owner->_ctx.data.out_yaw_torque   = 0;

    // 【防跳变保护】：对齐目标值
    if (owner->_ctx.data.has_initial_calibrated) {
        owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_pitch_motor_rad;
    } else {
        owner->_ctx.data.target_pitch_rad = owner->_ctx.data.pitch_imu_rad;
    }

    owner->_ctx.data.target_yaw_rad = pyro::loop_fp32_constrain(owner->_ctx.data.yaw_imu_rad, -PI, PI);
    owner->_ctx.data.target_relative_yaw_rad =
        owner->_ctx.data.relative_yaw_motor_wrapped_rad;

    owner->_ctx.motor.pitch->send_torque(0);
    owner->_ctx.motor.yaw->send_torque(0);
}

void screw_gimbal_t::fsm_passive_t::on_exit(owner *owner) {}
} // namespace pyro
