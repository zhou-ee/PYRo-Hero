#include "pyro_hybrid_chassis.h"
#include "hybrid_config.h"
namespace pyro
{

void hybrid_chassis_t::state_passive_t::enter(owner *owner)
{
    owner->_ctx.motor.mecanum[0]->disable();
    owner->_ctx.motor.mecanum[1]->disable();
    owner->_ctx.motor.mecanum[2]->disable();
    owner->_ctx.motor.mecanum[3]->disable();
    owner->_ctx.motor.track[0]->disable();
    owner->_ctx.motor.track[1]->disable();
    owner->_ctx.motor.leg[0]->disable();
    owner->_ctx.motor.leg[1]->disable();
}

void hybrid_chassis_t::state_passive_t::execute(owner *owner)
{
    owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_pitch_rad;

    // LEFT_LEG_OFFSET_RAD  = owner->_ctx.motor.leg[0]->get_current_position();
    // RIGHT_LEG_OFFSET_RAD = owner->_ctx.motor.leg[1]->get_current_position();
    owner->_ctx.motor.mecanum[0]->send_torque(0);
    owner->_ctx.motor.mecanum[1]->send_torque(0);
    owner->_ctx.motor.mecanum[2]->send_torque(0);
    owner->_ctx.motor.mecanum[3]->send_torque(0);
    owner->_ctx.motor.track[0]->send_torque(0);
    owner->_ctx.motor.track[1]->send_torque(0);
    owner->_ctx.motor.leg[0]->send_torque(0);
    owner->_ctx.motor.leg[1]->send_torque(0);
}

void hybrid_chassis_t::state_passive_t::exit(owner *owner)
{
}

} // namespace pyro