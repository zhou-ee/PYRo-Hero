#include "pyro_quad_booster.h"

namespace pyro
{

void quad_booster_t::state_passive_t::enter(owner *owner)
{
    owner->_ctx.pid.trigger_pos_pid->clear();
    owner->_ctx.pid.trigger_spd_pid->clear();
    owner->_ctx.motor.trigger_wheel->disable();
    owner->_ctx.motor.fric_wheels[0]->enable();
    owner->_ctx.motor.fric_wheels[1]->disable();
    owner->_ctx.motor.fric_wheels[2]->disable();
    owner->_ctx.motor.fric_wheels[3]->disable();
    owner->_ctx.data.anti_jam_active = false;
    _trigger_stopped = false;
}

void quad_booster_t::state_passive_t::execute(owner *owner)
{
    // 持续输出0
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
    owner->_ctx.data.out_trig_torque = 0;
    owner->_send_fric_command();

    owner->_ctx.data.target_trig_rad   = owner->_ctx.data.current_trig_rad;
    owner->_ctx.data.target_trig_radps = 0.0f;
    if (abs(owner->_ctx.data.current_trig_radps) < 0.05f)
    {
        owner->_ctx.data.out_trig_torque = 0.0f;
        _trigger_stopped                 = true;
    }
    if (!_trigger_stopped)
        owner->_trigger_speed_control();
    owner->_send_trigger_command();
}

void quad_booster_t::state_passive_t::exit(owner *owner)
{
}

} // namespace pyro
