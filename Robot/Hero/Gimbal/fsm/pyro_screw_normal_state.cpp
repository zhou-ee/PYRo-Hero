#include "pyro_screw_gimbal.h"
#include "screw_config.h"
#include "pyro_algo_common.h"
#include <algorithm>

namespace pyro
{
void screw_gimbal_t::fsm_active_t::normal_state_t::enter(owner *owner)
{
    // 切换到 Normal 模式时，对齐目标防止跳变
    owner->_ctx.data.target_yaw_rad = owner->_ctx.data.yaw_imu_rad;
    owner->_ctx.data.target_pitch_rad =
        owner->_ctx.data.current_pitch_motor_rad;

    // 清除切入切出时可能残留的 PID 积分，防止由于历史积分导致的猛烈弹跳
    owner->_ctx.pid.yaw_pos->clear();
    owner->_ctx.pid.yaw_spd->clear();

    if (owner->_fsm_active._last_state != &owner->_fsm_active.sling_state)
    {
        owner->_ctx.pid.pitch_pos->clear();
        owner->_ctx.pid.pitch_spd->clear();
    }
}

void screw_gimbal_t::fsm_active_t::normal_state_t::execute(owner *owner)
{
    if (owner->_ctx.cmd->track_en)
    {
        owner->_ctx.data.target_pitch_rad = GIMBAL_CROSSING_PITCH_RAD;
    }
    else
    {
        owner->_ctx.data.target_pitch_rad += owner->_ctx.cmd->pitch_delta_angle;
    }
    owner->_ctx.data.target_yaw_rad += owner->_ctx.cmd->yaw_delta_angle;

    // ==========================================
    // 1. Pitch 绝对限幅
    // ==========================================
    owner->_ctx.data.target_pitch_rad = std::clamp(
        owner->_ctx.data.target_pitch_rad, PITCH_MIN_RELATIVE_RAD, PITCH_MAX_RELATIVE_RAD);


    owner->_ctx.data.target_yaw_rad =
        pyro::loop_fp32_constrain(owner->_ctx.data.target_yaw_rad, -PI, PI);

    // ==========================================
    // 3. 执行底层控制与发送指令
    // 底层的 _gimbal_control 算误差时自带了 loop_fp32_constrain，所以步骤 f 给出的数据完全合法
    // ==========================================
    owner->_gimbal_control();
    _send_motor_command(&owner->_ctx);
}

void screw_gimbal_t::fsm_active_t::normal_state_t::exit(owner *owner)
{
}
} // namespace pyro
