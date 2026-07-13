#include "pyro_board_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_mec_chassis.h"
#include "pyro_module_base.h"
#include "pyro_referee.h"

#include <cstdint>

using namespace pyro;

pyro::mec_chassis_t *mec_chassis_ptr  = nullptr;
static pyro::mec_cmd_t *mec_cmd_ptr   = nullptr;
static pyro::mec_deps_t *mec_deps_ptr = nullptr;
static board_drv_t *board_drv_ptr     = nullptr;

static void chassis_rxcmd();
static void deps_init();

extern "C"
{
    void hero_chassis_thread(void *argument)
    {
        static bool prev_chassis_output = false;

        while (true)
        {
            const bool current_chassis_output =
                referee_drv_t::get_instance()
                    ->get_data()
                    .robot_status.power_management_chassis_output;

            if (current_chassis_output && !prev_chassis_output)
            {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            prev_chassis_output = current_chassis_output;

            if (current_chassis_output && board_drv_ptr->check_online())
            {
                chassis_rxcmd();
            }
            else
            {
                mec_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
            }

            mec_chassis_ptr->set_command(*mec_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_chassis_init(void *argument)
    {
        board_drv_ptr = &board_drv_t::get_instance(board_drv_t::role_t::CHASSIS,
                                                   can_hub_t::can2);
        mec_cmd_ptr     = new pyro::mec_cmd_t();
        mec_chassis_ptr = pyro::mec_chassis_t::instance();

        deps_init();
        mec_chassis_ptr->configure(*mec_deps_ptr);
        mec_chassis_ptr->start();

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

    const auto &rx_data = board_drv_ptr->get_g2c_rx_data();
    mec_cmd_ptr->vx     = 4.0f * static_cast<float>(rx_data.vx) / 127.0f;
    mec_cmd_ptr->vy     = 2.0f * static_cast<float>(rx_data.vy) / 127.0f;
    mec_cmd_ptr->wz     = 8.0f * static_cast<float>(rx_data.wz) / 127.0f;
    mec_cmd_ptr->mode   = rx_data.active ? pyro::cmd_base_t::mode_t::ACTIVE
                                         : pyro::cmd_base_t::mode_t::PASSIVE;
}

void deps_init()
{
    mec_deps_ptr = new pyro::mec_deps_t();

    mec_deps_ptr->motor_deps.wheels[0] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                        pyro::can_hub_t::can1);
    mec_deps_ptr->motor_deps.wheels[1] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,
                                        pyro::can_hub_t::can1);
    mec_deps_ptr->motor_deps.wheels[2] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,
                                        pyro::can_hub_t::can1);
    mec_deps_ptr->motor_deps.wheels[3] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4,
                                        pyro::can_hub_t::can1);

    mec_deps_ptr->motor_deps.yaw = new pyro::dji_gm_6020_motor_drv_t(
        pyro::dji_motor_tx_frame_t::id_2, pyro::can_hub_t::can2);

    for (auto *&pid : mec_deps_ptr->pid_deps.wheel_pid)
    {
        pid = new pid_t(0.2f, 0.0f, 0.0f, 1.0f, 20.0f);
    }

    mec_deps_ptr->pid_deps.follow_yaw_pid =
        new pid_t(5.0f, 0.0f, 0.002f, 0.0f, 6.0f, 10.0f, 1, 10.0f, 1, 4);
}
