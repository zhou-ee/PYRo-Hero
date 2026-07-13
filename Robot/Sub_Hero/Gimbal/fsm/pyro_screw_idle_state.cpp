#include "pyro_screw_gimbal.h"

namespace pyro
{
void screw_gimbal_t::fsm_passive_t::idle_state_t::enter(owner *owner)
{
    owner->_ctx.data.is_calibrating = false;
}

void screw_gimbal_t::fsm_passive_t::idle_state_t::execute(owner *owner)
{
    // 检测 trigger_calibration 标志位从 0 到 1 的上升沿
    const bool current_calib_flag = owner->_ctx.cmd->trigger_calibration;
    const bool is_rising_edge = current_calib_flag && !owner->_fsm_passive._last_calib_flag;

    // 触发条件：外部命令的上升沿，或者刚上电还未完成初次校准
    if (is_rising_edge || !owner->_ctx.data.has_initial_calibrated)
    {
        request_switch(&owner->_fsm_passive.calibration_state);
    }

    owner->_fsm_passive._last_calib_flag = current_calib_flag;
}

void screw_gimbal_t::fsm_passive_t::idle_state_t::exit(owner *owner) {}
} // namespace pyro