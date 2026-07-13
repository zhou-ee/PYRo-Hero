#include "pyro_module_base.h"
#include "pyro_board_drv.h"
#include "pyro_hybrid_chassis.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_referee.h"

#include <cstdint>

using namespace pyro;

pyro::hybrid_chassis_t *hybrid_chassis_ptr  = nullptr;
static pyro::hybrid_cmd_t *hybrid_cmd_ptr   = nullptr;
static pyro::hybrid_deps_t *hybrid_deps_ptr = nullptr;
static board_drv_t *board_drv_ptr           = nullptr;

static void chassis_rxcmd();
static void deps_init();

extern "C"
{
    void hero_chassis_thread(void *argument)
    {
        static bool prev_chassis_output = false;

        while (true)
        {
            bool current_chassis_output =
                referee_drv_t::get_instance()
                    ->get_data()
                    .robot_status.power_management_chassis_output;

            if (current_chassis_output && !prev_chassis_output)
            {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            prev_chassis_output = current_chassis_output;

            if (current_chassis_output)
            {
                if (board_drv_ptr->check_online())
                {
                    chassis_rxcmd();
                }
                else
                {
                    hybrid_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
                }
            }
            else
            {
                hybrid_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
            }
            hybrid_chassis_ptr->set_command(*hybrid_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_chassis_init(void *argument)
    {
        board_drv_ptr = &board_drv_t::get_instance(board_drv_t::role_t::CHASSIS,
                                                   can_hub_t::can1);
        hybrid_cmd_ptr     = new pyro::hybrid_cmd_t();
        hybrid_chassis_ptr = pyro::hybrid_chassis_t::instance();

        deps_init();
        hybrid_chassis_ptr->configure(*hybrid_deps_ptr);
        hybrid_chassis_ptr->start();

        xTaskCreate(hero_chassis_thread, "start_hero_chassis_thread", 128,
                    nullptr, configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}

void chassis_rxcmd()
{
    if (!board_drv_ptr || !board_drv_ptr->check_online())
    {
        return;
    }

    const auto &rx_data  = board_drv_ptr->get_g2c_rx_data();
    hybrid_cmd_ptr->vx   = 4.0f * static_cast<float>(rx_data.vx) / 127.0f;
    hybrid_cmd_ptr->vy   = 2.0f * static_cast<float>(rx_data.vy) / 127.0f;
    hybrid_cmd_ptr->wz   = 2.0f * static_cast<float>(rx_data.wz) / 127.0f;
    hybrid_cmd_ptr->mode = rx_data.active ? pyro::cmd_base_t::mode_t::ACTIVE
                                          : pyro::cmd_base_t::mode_t::PASSIVE;
    hybrid_cmd_ptr->pseudo_gyro_en = rx_data.wz > 0;
    hybrid_cmd_ptr->crossing_en = rx_data.track_en;
    hybrid_cmd_ptr->leg_retract = rx_data.leg_retract;
    hybrid_cmd_ptr->leg_calibration = rx_data.leg_calibration;
}


void deps_init()
{
    hybrid_deps_ptr = new pyro::hybrid_deps_t();
    hybrid_deps_ptr->motor_deps.mecanum[0] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                        pyro::can_hub_t::can3); // FL Wheel
    hybrid_deps_ptr->motor_deps.mecanum[1] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,
                                        pyro::can_hub_t::can3); // FR Wheel
    hybrid_deps_ptr->motor_deps.mecanum[2] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,
                                        pyro::can_hub_t::can3); // RL Wheel
    hybrid_deps_ptr->motor_deps.mecanum[3] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4,
                                        pyro::can_hub_t::can3); // RR Wheel
    hybrid_deps_ptr->motor_deps.track[0] =
        new pyro::dm_motor_drv_t(0x11, 0x21, pyro::can_hub_t::can3);
    hybrid_deps_ptr->motor_deps.track[1] =
        new pyro::dm_motor_drv_t(0x12, 0x22, pyro::can_hub_t::can3);
    hybrid_deps_ptr->motor_deps.leg[0] =
        new pyro::dm_motor_drv_t(0x31, 0x41, pyro::can_hub_t::can2);
    hybrid_deps_ptr->motor_deps.leg[1] =
        new pyro::dm_motor_drv_t(0x32, 0x42, pyro::can_hub_t::can2);

    hybrid_deps_ptr->motor_deps.yaw = new pyro::dji_gm_6020_motor_drv_t(
        pyro::dji_motor_tx_frame_t::id_3, pyro::can_hub_t::can1);

    // NOLINTBEGIN(cppcoreguidelines-pro-type-static-cast-downcast)
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[0])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[1])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[0])
        ->set_rotate_range(-30.0f, 30.0f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[1])
        ->set_rotate_range(-30.0f, 30.0f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[0])
        ->set_torque_range(-11.0f, 11.0f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[1])
        ->set_torque_range(-11.0f, 11.0f);

    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[0])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[1])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[0])
        ->set_rotate_range(-5.655f, 5.655f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[1])
        ->set_rotate_range(-5.655f, 5.655f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[0])
        ->set_torque_range(-27.0f, 27.0f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[1])
        ->set_torque_range(-27.0f, 27.0f);
    // NOLINTEND(cppcoreguidelines-pro-type-static-cast-downcast)

    // 3508 手册的最大电流为 10 A
    // C620 电调提供的 3508性能曲线表示在电流 > 10A 时变化不大
    hybrid_deps_ptr->pid_deps.mecanum_pid[0] =
        new pid_t(0.3f, 0.0008f, 0.0002f, 1.0f, 10.0f, 20, 1, 10, 1, 4);
    hybrid_deps_ptr->pid_deps.mecanum_pid[1] =
        new pid_t(0.3f, 0.0008f, 0.0002f, 1.0f, 10.0f, 20, 1, 10, 1, 4);
    hybrid_deps_ptr->pid_deps.mecanum_pid[2] =
        new pid_t(0.3f, 0.0008f, 0.0002f, 1.0f, 10.0f, 20, 1, 10, 1, 4);
    hybrid_deps_ptr->pid_deps.mecanum_pid[3] =
        new pid_t(0.3f, 0.0008f, 0.0002f, 1.0f, 10.0f, 20, 1, 10, 1, 4);

    hybrid_deps_ptr->pid_deps.follow_yaw_pid =
        new pid_t(8.0f, 0.0f, 0.1f, 0.0f, 10.0f, 200, 1, 100, 1, 4);

    hybrid_deps_ptr->pid_deps.track_pid[0] =
        new pid_t(0.02f, 0.0001f, 0.00002f, 0.5f, 11.0f, 20, 1, 10, 1, 4);
    hybrid_deps_ptr->pid_deps.track_pid[1] =
        new pid_t(0.02f, 0.0001f, 0.00002f, 0.5f, 11.0f, 20, 1, 10, 1, 4);

    hybrid_deps_ptr->pid_deps.pitch_pid = new pid_t(
        260.0f, 0.1f, 10.0f, 20.0f, 200.0f, 200, 1, 100, 1, 4,
        pid_t::INTEGRAL_LIMIT | pid_t::OUTPUT_FILTER |
            pid_t::DERIVATIVE_FILTER | pid_t::DERIVATIVE_ON_MEASUREMENT);
    hybrid_deps_ptr->pid_deps.roll_pid = new pid_t(
        360.0f, 0.1f, 10.0f, 20.0f, 200.0f, 200, 1, 100, 1, 4,
        pid_t::INTEGRAL_LIMIT | pid_t::OUTPUT_FILTER |
            pid_t::DERIVATIVE_FILTER | pid_t::DERIVATIVE_ON_MEASUREMENT);

    // 检测传动质量用参数
    // hybrid_deps_ptr->pid_deps.leg_pos_pid[0] =
    // new pid_t(32.2f, 0.005f, 0.008f, 0.01f, 20.0f, 20, 1, 10, 1, 4);
    // hybrid_deps_ptr->pid_deps.leg_pos_pid[1] =
    //     new pid_t(32.2f, 0.005f, 0.008f, 0.01f, 20.0f, 20, 1, 10, 1, 4);
    // hybrid_deps_ptr->pid_deps.leg_vel_pid[0] =
    //     new pid_t(1400.0f, 0.005f, 0.00f, 5.0f, 200.0f, 20, 1, 10, 1, 4);
    // hybrid_deps_ptr->pid_deps.leg_vel_pid[1] =
    //     new pid_t(1400.0f, 0.005f, 0.00f, 5.0f, 200.0f, 20, 1, 10, 1, 4);

    hybrid_deps_ptr->pid_deps.leg_pos_pid[0] =
        new pid_t(11.2f, 0.005f, 0.008f, 0.01f, 0.4f, 20, 1, 10, 1, 4);
    hybrid_deps_ptr->pid_deps.leg_pos_pid[1] =
        new pid_t(11.2f, 0.005f, 0.008f, 0.01f, 0.4f, 20, 1, 10, 1, 4);
    hybrid_deps_ptr->pid_deps.leg_vel_pid[0] =
        new pid_t(260.0f, 0.005f, 0.008f, 5.0f, 200.0f, 20, 1, 10, 1, 4);
    hybrid_deps_ptr->pid_deps.leg_vel_pid[1] =
        new pid_t(260.0f, 0.005f, 0.008f, 5.0f, 200.0f, 20, 1, 10, 1, 4);
}
