#include "pyro_screw_gimbal.h"

namespace pyro
{
void screw_gimbal_t::fsm_passive_t::calibration_state_t::enter(owner *owner)
{
    owner->_ctx.data.is_calibrating = true;

    // 清空上次校准的累计数据，确保能重新校准
    owner->_calib_tick = 0;
    owner->_calib_pitch_sum = 0.0f;
}

void screw_gimbal_t::fsm_passive_t::calibration_state_t::execute(owner *owner)
{
    // 同步标志位，防止持续长按触发重复校准
    owner->_fsm_passive._last_calib_flag = owner->_ctx.cmd->trigger_calibration;

    if (owner->_calibrate_pitch_offset())
    {
        // 校准完成，标记初次校准已完成
        owner->_ctx.data.has_initial_calibrated = true;
        request_switch(&owner->_fsm_passive.idle_state);
    }
}

void screw_gimbal_t::fsm_passive_t::calibration_state_t::exit(owner *owner) {}
} // namespace pyro