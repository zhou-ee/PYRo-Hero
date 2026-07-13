#include "pyro_mec_chassis.h"

namespace pyro
{

void mec_chassis_t::state_passive_t::enter(owner *owner)
{
    for (auto *motor : owner->_ctx.motor.wheels)
    {
        motor->disable();
    }

    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.target_wheel_rpm[i] = 0.0f;
        owner->_ctx.data.out_wheel_torque[i] = 0.0f;
    }

    owner->_ctx.pid.follow_yaw_pid->clear();
    for (auto *pid : owner->_ctx.pid.wheel_pid)
    {
        pid->clear();
    }
}

void mec_chassis_t::state_passive_t::execute(owner *owner)
{
    owner->_supercap_control();

    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.out_wheel_torque[i] = 0.0f;
    }
    owner->_send_motor_command();
}

void mec_chassis_t::state_passive_t::exit(owner *owner)
{
}

} // namespace pyro
