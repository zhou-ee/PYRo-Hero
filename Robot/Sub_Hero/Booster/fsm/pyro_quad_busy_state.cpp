#include "pyro_quad_booster.h"

namespace pyro
{

void quad_booster_t::fsm_active_t::state_busy_t::enter(owner *owner)
{
}

void quad_booster_t::fsm_active_t::state_busy_t::execute(owner *owner)
{
    float error =
        owner->_ctx.data.target_trig_rad - owner->_ctx.data.current_trig_rad;

    // 新增：过零点修正
    error = quad_booster_t::_normalize_angle(error);

    if (abs(error) < 0.3f)
    {
        request_switch(&owner->_state_active._interim_state);
    }
    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void quad_booster_t::fsm_active_t::state_busy_t::exit(owner *owner)
{
}

} // namespace pyro
