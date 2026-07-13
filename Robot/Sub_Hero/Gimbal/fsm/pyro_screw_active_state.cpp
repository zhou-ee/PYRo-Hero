#include "pyro_screw_gimbal.h"

namespace pyro
{
void screw_gimbal_t::fsm_active_t::on_enter(owner *owner)
{
    owner->_ctx.motor.pitch->enable();
    owner->_ctx.motor.yaw->enable();

    // 刚进入 Active 模式时，根据指令判定初始子状态
    if (owner->_ctx.cmd->autoaim_mode)
    {
        change_state(&autoaim_state);
    }
    else if (owner->_ctx.cmd->sling_mode)
    {
        change_state(&sling_state);
    }
    else
    {
        change_state(&normal_state);
    }
}

void screw_gimbal_t::fsm_active_t::on_execute(owner *owner)
{
    // 子状态机切换逻辑
    if (owner->_ctx.cmd->autoaim_mode)
    {
        change_state(&autoaim_state);
    }
    else if (owner->_ctx.cmd->sling_mode)
    {
        change_state(&sling_state);
    }
    else
    {
        change_state(&normal_state);
    }
}

void screw_gimbal_t::fsm_active_t::on_exit(owner *owner)
{
}
} // namespace pyro