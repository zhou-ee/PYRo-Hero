#include "pyro_screw_gimbal.h"
#include "screw_config.h"
#include "pyro_algo_common.h"
#include "pyro_board_drv.h"

#include <algorithm>
#include <cmath>

namespace pyro
{
void screw_gimbal_t::fsm_active_t::sling_state_t::enter(owner *owner)
{
    // 切换到 Sling 模式时，对齐相对角目标
    owner->_ctx.data.target_yaw_rad = owner->_ctx.data.relative_yaw_motor_rad;
    owner->_ctx.data.target_pitch_rad =
        owner->_ctx.data.current_pitch_motor_rad;



    // 【重要】吊射模式下禁止动态实时校准，确保角度绝对稳定，防止因大角度震动触发错误校准
    owner->_ctx.data.allow_dynamic_calib = false;
    owner->_ctx.pid.yaw_relative_pos->clear();
    owner->_ctx.pid.yaw_relative_spd->clear();
    owner->_ctx.pid.yaw_pos_leso->clear();
    owner->_ctx.pid.yaw_spd_leso->clear();
}

void screw_gimbal_t::fsm_active_t::sling_state_t::execute(owner *owner)
{
    // 在 Sling 模式下，WASD输入直接映射为相对角度或Pitch角度的累加
    owner->_ctx.data.target_pitch_rad += owner->_ctx.cmd->pitch_delta_angle;
    owner->_ctx.data.target_yaw_rad += owner->_ctx.cmd->yaw_delta_angle;

    static uint32_t last_sling_preaim_seq = 0;
    if (owner->_ctx.cmd->sling_preaim_seq != last_sling_preaim_seq)
    {
        constexpr float target_z = 1.080f;
        constexpr float delta_z = target_z - (0.35f + 0.4f);
        float delta_x = 21.1f;
        float delta_y = 0.0f;

        if (owner->_ctx.cmd->sling_preaim_source ==
            screw_gimbal_cmd_t::sling_preaim_source_t::CHASSIS_COORD)
        {
            const auto &rx_data = board_drv_t::get_instance().get_c2g_rx_data();
            const float robot_x =
                static_cast<float>(rx_data.robot_x) / 65535.0f * 28.0f;
            const float robot_y =
                static_cast<float>(rx_data.robot_y) / 65535.0f * 15.0f;
            const float target_x = rx_data.robot_color ? 25.593f : 2.407f;
            constexpr float target_y = 7.5f;

            delta_x = std::fabs(target_x - robot_x);
            delta_y = std::fabs(target_y - robot_y);
        }
        if (auto pitch = solveIdealPitch(delta_x, delta_y, delta_z, 16.3f))
        {
            float imu_target_pitch = -*pitch;
            float err = owner->_ctx.data.current_pitch_motor_rad - owner->_ctx.data.pitch_imu_rad;
            owner->_ctx.data.target_pitch_rad = imu_target_pitch + err;
        }
        last_sling_preaim_seq = owner->_ctx.cmd->sling_preaim_seq;
    }

    owner->_ctx.data.target_yaw_rad =
        std::clamp(owner->_ctx.data.target_yaw_rad, YAW_MIN_RELATIVE_RAD,
                   YAW_MAX_RELATIVE_RAD);

    // Pitch 绝对限幅
    owner->_ctx.data.target_pitch_rad =
        std::clamp(owner->_ctx.data.target_pitch_rad, PITCH_MIN_RELATIVE_RAD,
                   PITCH_MAX_RELATIVE_RAD);

    // Yaw 相对限幅 (基于配置极值)
    // owner->_ctx.data.target_relative_yaw_rad = std::clamp(
    //     owner->_ctx.data.target_relative_yaw_rad, YAW_MIN_RELATIVE_RAD,
    //     YAW_MAX_RELATIVE_RAD);

    // 执行纯机械角控制与发送指令
    owner->_gimbal_sling_control();
    screw_gimbal_t::_send_motor_command(&owner->_ctx);
}

void screw_gimbal_t::fsm_active_t::sling_state_t::exit(owner *owner)
{
    // 退出 Sling 模式后恢复动态校准
    owner->_ctx.data.allow_dynamic_calib = true;
}
} // namespace pyro
