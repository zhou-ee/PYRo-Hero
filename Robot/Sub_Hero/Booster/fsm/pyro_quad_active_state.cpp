#include "pyro_dm_motor_drv.h"
#include "pyro_quad_booster.h"
#include "pyro_dwt_drv.h"

namespace pyro
{

void quad_booster_t::fsm_active_t::on_enter(owner *owner)
{
    static_cast<dm_motor_drv_t *>(owner->_ctx.motor.trigger_wheel) // NOLINT
        ->clear_error();                                           // NOLINT
    owner->_ctx.motor.trigger_wheel->enable();
    owner->_ctx.motor.fric_wheels[0]->enable();
    owner->_ctx.motor.fric_wheels[1]->enable();
    owner->_ctx.motor.fric_wheels[2]->enable();
    owner->_ctx.motor.fric_wheels[3]->enable();
    owner->_ctx.data.internal_reset_count = owner->_ctx.cmd->reset_count;
    owner->_ctx.data.internal_shoot_data_reset_count =
        owner->_ctx.cmd->shoot_data_reset_count;

    change_state(&_homing_state);
}

void quad_booster_t::fsm_active_t::on_execute(owner *owner)
{
    if (owner->_ctx.cmd->anti_jam)
    {
        if (_active_state != &_anti_jam_state)
        {
            change_state(&_anti_jam_state);
        }
        return;
    }

    if (_active_state == &_anti_jam_state)
    {
        change_state(&_homing_state);
        return;
    }

    if (owner->_ctx.cmd->reset_count != owner->_ctx.data.internal_reset_count)
    {
        owner->_ctx.data.internal_reset_count = owner->_ctx.cmd->reset_count;
        change_state(&_homing_state);
        return;
    }

    // 1. 弹速闭环更新
    if (owner->_ctx.cmd->shoot_data_reset_count !=
        owner->_ctx.data.internal_shoot_data_reset_count)
    {
        owner->_ctx.data.internal_shoot_data_reset_count =
            owner->_ctx.cmd->shoot_data_reset_count;
        owner->_reset_active_shoot_data();
    }

    owner->_speed_control();

    if (owner->_ctx.cmd->fric_on)
    {
        auto &shoot_data                    = owner->_use_deploy_data()
                                                  ? owner->_ctx.shoot_deploy_data
                                                  : owner->_ctx.shoot_normal_data;

        owner->_ctx.data.target_fric_mps[0] = shoot_data.fric2_mps;
        owner->_ctx.data.target_fric_mps[2] = -shoot_data.fric2_mps;
        owner->_ctx.data.target_fric_mps[1] = shoot_data.fric1_mps;
        owner->_ctx.data.target_fric_mps[3] = -shoot_data.fric1_mps;

        owner->_fric_control();
    }
    else
    {
        owner->_ctx.data.target_fric_mps[0] = 0.0f;
        owner->_ctx.data.target_fric_mps[1] = 0.0f;
        owner->_ctx.data.target_fric_mps[2] = 0.0f;
        owner->_ctx.data.target_fric_mps[3] = 0.0f;
        owner->_fric_control();
        for (int i = 0; i < 4; i++)
        {
            if (abs(owner->_ctx.data.current_fric_mps[i]) < 0.3f)
                owner->_ctx.data.out_fric_torque[i] = 0.0f;
        }
    }
    owner->_send_fric_command();

    // 2. 发弹延迟计算
    owner->_launch_delay_calculate();

    // 3. 拨弹盘堵转判断
    constexpr float STALL_TIME_THRESHOLD   = 1000.0f;
    constexpr float STALL_TORQUE_THRESHOLD = 2.5f;
    constexpr float STALL_SPEED_THRESHOLD  = 0.1f;

    static float stall_start_time          = 0.0f;
    static uint16_t clear_stall_counter    = 0;
    if (abs(owner->_ctx.data.current_trig_radps) < STALL_SPEED_THRESHOLD &&
        abs(owner->_ctx.data.current_trig_torque) > STALL_TORQUE_THRESHOLD)
    {
        clear_stall_counter = 0;
        if (stall_start_time == 0.0f)
        {
            stall_start_time = dwt_drv_t::get_timeline_ms();
        }
        else
        {
            const float elapsed_time =
                dwt_drv_t::get_timeline_ms() - stall_start_time;
            if (elapsed_time >= STALL_TIME_THRESHOLD)
            {
                change_state(&_stall_state);
                if (_active_state == &_stall_state)
                {
                    reset();
                }
                stall_start_time = 0.0f;
            }
        }
    }
    else
    {
        if (stall_start_time != 0.0f)
        {
            clear_stall_counter++;
            if (clear_stall_counter >= 8) // 连续8个周期不满足堵转条件
            {
                stall_start_time    = 0.0f; // 真正重置堵转计时
                clear_stall_counter = 0;    // 计数器归零
            }
        }
        else
        {
            clear_stall_counter = 0; // 平时未触发堵转检测时，保持计数器为0
        }
    }
}

void quad_booster_t::fsm_active_t::on_exit(owner *owner)
{
}

} // namespace pyro
