#include "pyro_quad_booster.h"

namespace pyro
{

void quad_booster_t::state_temp_t::enter(owner *owner)
{
    owner->_ctx.motor.trigger_wheel->enable();
}

void quad_booster_t::state_temp_t::execute(owner *owner)
{
    // 拨弹盘速度闭环
    owner->_ctx.data.target_trig_radps = owner->_ctx.cmd->trig_target_spd;
    owner->_trigger_speed_control();
    owner->_send_trigger_command();
}

void quad_booster_t::state_temp_t::exit(owner *owner)
{
}

} // namespace pyro