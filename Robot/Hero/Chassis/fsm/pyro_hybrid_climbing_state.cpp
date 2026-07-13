#include "pyro_hybrid_chassis.h"

namespace pyro
{

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::on_enter(owner *owner)
{
    // 进入爬坡模式，重新初始化所有驱动机构的 PID
    for (auto *pid : owner->_ctx.pid.mecanum_pid)
    {
        if (pid) pid->clear();
    }
    for (auto *pid : owner->_ctx.pid.track_pid)
    {
        if (pid) pid->clear();
    }

    owner->_ctx.motor.track[0]  ->enable();
    owner->_ctx.motor.track[1]  ->enable();
    owner->_ctx.motor.leg[0]    ->enable();
    owner->_ctx.motor.leg[1]    ->enable();

    // --- 初始化状态 ---
    _auto_retract_flag = false;
    _retract_hold_tick = 0;
}

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::on_execute(owner *owner)
{
    // 1. 麦轮速度环控制 (提供前轮牵引力)
    owner->_mecanum_control();

    // 2. 履带速度环控制 (履带正式介入，提供主要越障/爬坡推进力)
    owner->_track_control();

    owner->_power_control();


    // ================= 自动收腿：双测距空间判定逻辑 =================

    // 1. 提取 _update_feedback 中已做完三阶低通滤波的数据
    bool is_front_on_step  = (owner->_ctx.data.filtered_front_distance < CLIMB_DIST_THRES_LOW);
    bool is_back_suspended = (owner->_ctx.data.filtered_back_distance > CLIMB_DIST_THRES_HIGH);
    bool is_back_on_step   = (owner->_ctx.data.filtered_back_distance < CLIMB_DIST_THRES_LOW);

    // 2. 状态转移逻辑
    if (!_auto_retract_flag)
    {
        // a. 触发判定：前轮搭上台阶 且 后部被腿支撑悬空 且 车身保持向前速度
        if (is_front_on_step && is_back_suspended && owner->_ctx.data.real_vx > CLIMB_VEL_X_FRONT_THRES)
        {
            _auto_retract_flag = true;
            _retract_hold_tick = 0;    // 触发时清零计时器，开始超时倒数
        }
    }
    else
    {
        _retract_hold_tick++; // 处于收腿状态时，每帧累加时间

        // b. 成功越障判定 (绝对空间确认)
        if (is_back_on_step)
        {
            // 后方导轮也落在了台阶面上，证明整车成功上去，立刻解除收腿
            _auto_retract_flag = false;
            _retract_hold_tick = 0;
        }
        // c. 超时卡死判定
        else if (_retract_hold_tick >= CLIMB_RETRACT_TIMEOUT_TICKS)
        {
            // 长时间未能让后轮上去，证明卡住或履带打滑，强制放下腿重新支撑
            _auto_retract_flag = false;
            _retract_hold_tick = 0;
        }
        // d. 主动放弃判定 (倒车保护)
        else if (owner->_ctx.data.real_vx < -CLIMB_VEL_X_BACK_THRES)
        {
            // 驾驶员剧烈向后拉摇杆，强制解除收腿，重新放下腿部支撑保护底盘
            _auto_retract_flag = false;
            _retract_hold_tick = 0;
        }
    }
    // =============================================================

    // 综合原有的手动控制与自动判定
    if (owner->_ctx.cmd->leg_retract || _auto_retract_flag)
    {
        change_state(&leg_retraction_state);
    }
    else
    {
        change_state(&track_climbing_state);
    }
}

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::on_exit(owner *owner)
{
    // 退出爬坡模式时的清理工作
    owner->_ctx.data.out_leg_torque[0] = 0;
    owner->_ctx.data.out_leg_torque[1] = 0;
    owner->_ctx.data.out_track_torque[0] = 0;
    owner->_ctx.data.out_track_torque[1] = 0;

    owner->_ctx.motor.track[0]  ->disable();
    owner->_ctx.motor.track[1]  ->disable();
    owner->_ctx.motor.leg[0]    ->disable();
    owner->_ctx.motor.leg[1]    ->disable();
}

} // namespace pyro