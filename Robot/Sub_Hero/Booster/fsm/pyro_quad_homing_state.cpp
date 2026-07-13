#include "pyro_dwt_drv.h"
#include "pyro_quad_booster.h"
#include "quad_config.h"


namespace pyro
{
void quad_booster_t::fsm_active_t::state_homing_t::enter(owner *owner)
{
    _homing_turnback_start_time = dwt_drv_t::get_timeline_ms();
}

void quad_booster_t::fsm_active_t::state_homing_t::execute(owner *owner)
{
    const float turnback_time =
        dwt_drv_t::get_timeline_ms() - _homing_turnback_start_time;
    if (turnback_time > 1200.0f)
    {
        request_switch(&owner->_state_active._interim_state);
        return;
    }
    owner->_ctx.data.target_trig_radps = -TRIGGER_FEED_DIR * 5.0f;
    owner->_trigger_speed_control();
    owner->_send_trigger_command();
}

void quad_booster_t::fsm_active_t::state_homing_t::exit(owner *owner)
{
    owner->_ctx.data.target_trig_rad   = owner->_ctx.data.current_trig_rad;
    owner->_ctx.data.target_trig_radps = 0.0f;
}
} // namespace pyro
