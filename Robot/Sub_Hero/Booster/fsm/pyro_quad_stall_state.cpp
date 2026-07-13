#include "pyro_dwt_drv.h"
#include "pyro_quad_booster.h"
#include "quad_config.h" // 新增：引入预置位宏定义 TRIGGER_OFFSET
#include <cmath>         // 新增：引入数学库以使用 std::floor/std::ceil

namespace pyro
{
void quad_booster_t::fsm_active_t::state_stall_t::enter(owner *owner)
{


    if (&owner->_state_active._homing_state == owner->_state_active._last_state)
    {
        owner->_ctx.data.target_trig_rad   = owner->_ctx.data.current_trig_rad;
        owner->_ctx.data.target_trig_radps = 0;
        owner->_ctx.data.target_trig_rad += TRIGGER_FEED_DIR * TRIGGER_PRESET_DEFORM_THRESHOLD_RAD;
    }
    else if (&owner->_state_active._stall_state ==
             owner->_state_active._last_state)
    {
        if (owner->_ctx.data.out_trig_torque > 1.0f)
        {
            owner->_ctx.data.target_trig_rad =
                owner->_ctx.data.current_trig_rad - 0.05f;
        }
        else if (owner->_ctx.data.out_trig_torque < -1.0f)
        {
            owner->_ctx.data.target_trig_rad =
                owner->_ctx.data.current_trig_rad + 0.05f;
        }
        else
        {
            owner->_ctx.data.target_trig_rad =
                owner->_ctx.data.current_trig_rad;
            owner->_ctx.data.target_trig_radps = 0;
        }
    }
    else
    {
        owner->_ctx.data.target_trig_rad -=
            TRIGGER_FEED_DIR * PI / 3.0f; // 正常发弹时的堵转反转退弹逻辑
    }

    // 新增：保证目标值在 [-PI, PI] 合法范围内
    owner->_ctx.data.target_trig_rad =
        quad_booster_t::_normalize_angle(owner->_ctx.data.target_trig_rad);
}

void quad_booster_t::fsm_active_t::state_stall_t::execute(owner *owner)
{

    // 新增：计算最短路径的误差
    float error =
        owner->_ctx.data.current_trig_rad - owner->_ctx.data.target_trig_rad;
    error = quad_booster_t::_normalize_angle(error);

    // 回到合适角度后，切换回拨弹状态
    if (fabs(error) < 0.3f)
    {
        request_switch(&owner->_state_active._interim_state);
    }

    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void quad_booster_t::fsm_active_t::state_stall_t::exit(owner *owner)
{
}
} // namespace pyro
