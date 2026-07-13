//
// Created by Clean on 2026/4/6.
//
#include "pyro_screw_gimbal.h"
#include "screw_config.h"
#include "pyro_algo_common.h"
#include <algorithm>

namespace pyro
{
void screw_gimbal_t::fsm_active_t::autoaim_state_t::enter(owner *owner)
{
    // 切换到 Autoaim 模式时，直接对齐 IMU 绝对角，防止跳变
    owner->_ctx.data.target_yaw_rad   = owner->_ctx.data.yaw_imu_rad;
    owner->_ctx.data.target_pitch_rad = owner->_ctx.data.pitch_imu_rad;

    // 清除自瞄专用的绝对角度环和速度环积分
    owner->_ctx.pid.pitch_auto_pos->clear();
    owner->_ctx.pid.pitch_auto_spd->clear();

    // 清理 Yaw 环积分
    owner->_ctx.pid.yaw_pos->clear();
    owner->_ctx.pid.yaw_spd->clear();

    owner->_ctx.data.allow_dynamic_calib = false;
}

void screw_gimbal_t::fsm_active_t::autoaim_state_t::execute(owner *owner)
{
    // 自瞄模式下，直接读取外部传入的绝对目标角度并进行限幅
    if (fabs(owner->_ctx.cmd->target_pitch) < PI &&
        fabs(owner->_ctx.cmd->target_yaw) < PI)
    {
        owner->_ctx.data.target_pitch_rad = owner->_ctx.cmd->target_pitch;
        owner->_ctx.data.target_yaw_rad   = owner->_ctx.cmd->target_yaw;
    }
    else
    {
        //视觉掉线或数据异常时的兜底步进
        owner->_ctx.data.target_pitch_rad += owner->_ctx.cmd->pitch_delta_angle;
        owner->_ctx.data.target_yaw_rad   += owner->_ctx.cmd->yaw_delta_angle;
    }

    // ==========================================
    // 1. Pitch 绝对限幅 (使用 IMU 配置限幅阈值)
    // ==========================================
    owner->_ctx.data.target_pitch_rad =
        std::clamp(owner->_ctx.data.target_pitch_rad, PITCH_MIN_IMU_RAD,
                   PITCH_MAX_IMU_RAD);

    // ==========================================
    // 2. Yaw 环化限幅 (-PI, PI)
    // ==========================================
    owner->_ctx.data.target_yaw_rad =
        pyro::loop_fp32_constrain(owner->_ctx.data.target_yaw_rad, -PI, PI);

    // ==========================================
    // 3. 执行自瞄专用的绝对角度闭环控制并发送指令
    // ==========================================
    owner->_gimbal_autoaim_control();
    _send_motor_command(&owner->_ctx);
}

void screw_gimbal_t::fsm_active_t::autoaim_state_t::exit(owner *owner)
{
    owner->_ctx.data.allow_dynamic_calib = true;
}
} // namespace pyro