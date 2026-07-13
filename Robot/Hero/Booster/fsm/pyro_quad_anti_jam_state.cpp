#include "pyro_quad_booster.h"

namespace pyro
{

void quad_booster_t::fsm_active_t::state_anti_jam_t::enter(owner *owner)
{
    owner->_ctx.pid.trigger_pos_pid->clear();
    owner->_ctx.pid.trigger_spd_pid->clear();
    owner->_ctx.data.anti_jam_active = true;
    owner->_anti_jam_control();
    owner->_send_raw_fric_command();
}

void quad_booster_t::fsm_active_t::state_anti_jam_t::execute(owner *owner)
{
    owner->_anti_jam_control();
    owner->_send_raw_fric_command();
    owner->_send_trigger_command();
}

void quad_booster_t::fsm_active_t::state_anti_jam_t::exit(owner *owner)
{
    owner->_ctx.data.anti_jam_active = false;
}

} // namespace pyro
