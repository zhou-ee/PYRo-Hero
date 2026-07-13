#include "pyro_mec_chassis.h"

namespace pyro
{

void mec_chassis_t::state_active_t::enter(owner *owner)
{
    for (auto *motor : owner->_ctx.motor.wheels)
    {
        motor->enable();
    }

    owner->_ctx.pid.follow_yaw_pid->clear();
    for (auto *pid : owner->_ctx.pid.wheel_pid)
    {
        pid->clear();
    }
}

void mec_chassis_t::state_active_t::execute(owner *owner)
{
    owner->_supercap_control();
    owner->_kinematics_solve();
    owner->_mecanum_control();
    owner->_power_control();
    owner->_send_motor_command();
}

void mec_chassis_t::state_active_t::exit(owner *owner)
{
}

} // namespace pyro
