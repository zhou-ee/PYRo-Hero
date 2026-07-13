#include "pyro_hybrid_chassis.h"

namespace pyro
{

void hybrid_chassis_t::fsm_active_t::cruising_state_t::enter(owner *owner)
{
    // 进入巡航模式时，清空麦轮PID，防止状态切换瞬间的积分突变导致抽搐
    for (auto *pid : owner->_ctx.pid.mecanum_pid)
    {
        if (pid) pid->clear();
    }

    // 巡航模式下履带不工作，清空履带PID
    for (auto *pid : owner->_ctx.pid.track_pid)
    {
        if (pid) pid->clear();
    }
}

void hybrid_chassis_t::fsm_active_t::cruising_state_t::execute(owner *owner)
{

    owner->_leg_vmc();

    // owner->_ctx.data.target_leg_rad[0] = LEG_LENGTH_MAX_POS - LEG_POS_BUFFER_RAD;
    // owner->_ctx.data.target_leg_rad[1] = LEG_LENGTH_MAX_POS - LEG_POS_BUFFER_RAD;
    // owner->_leg_length_control();

    // 2. 麦轮速度环控制 (提供平面移动的主动力)
    owner->_mecanum_control();

    owner->_power_control();

    // 3. 巡航模式：强制关闭履带输出，省电并防止干扰
    owner->_ctx.data.out_track_torque[0] = 0.0f;
    owner->_ctx.data.out_track_torque[1] = 0.0f;

    // 4. 统一发送所有电机指令
    owner->_send_motor_command();
}

void hybrid_chassis_t::fsm_active_t::cruising_state_t::exit(owner *owner)
{
    // 退出巡航模式时的清理工作（当前可留空）
}

} // namespace pyro