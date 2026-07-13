/**
 * @file pyro_board_com.cpp
 */

#include "pyro_board_drv.h"
#include "pyro_ins.h"
#include "pyro_mec_chassis.h"
#include "pyro_module_base.h"
#include "pyro_referee.h"

using namespace pyro;

static TaskHandle_t board_com_task_handle = nullptr;
static board_drv_t *board_drv_ptr         = nullptr;

static void process_chassis_logic()
{
    // 1. 填充周期数据 (四元数)
    auto &tx_data = board_drv_ptr->get_c2g_tx_data();
    float q[4]    = {0.0f};
    ins_drv_t::get_instance()->get_quaternion(&q[0], &q[1], &q[2], &q[3]);

    for (int i = 0; i < 4; ++i)
    {
        tx_data.chassis_q[i] = static_cast<int16_t>(q[i] * 32767.0f);
    }

    static uint16_t last_launching_num = 0;
    auto *referee                      = referee_drv_t::get_instance();
    if (referee)
    {
        const auto &ref_data      = referee->get_data();
        const auto &referee_shoot = referee->get_data().shoot;
        if (ref_data.shoot_launching_count != last_launching_num)
        {
            board_drv_t::event_shoot_t shoot_event{};
            shoot_event.shoot_speed   = referee_shoot.initial_speed;
            shoot_event.launching_num = ref_data.shoot_launching_count;

            status_t ret              = board_drv_ptr->send_event(
                board_drv_t::EVENT_C2G_SHOOT, shoot_event);
            (void)ret;
            last_launching_num = ref_data.shoot_launching_count;
        }
        tx_data.chassis_output =
            ref_data.robot_status.power_management_chassis_output;
        tx_data.gimbal_output =
            ref_data.robot_status.power_management_gimbal_output;
        tx_data.booster_output =
            ref_data.robot_status.power_management_shooter_output;
        tx_data.chassis_power_limit = ref_data.robot_status.chassis_power_limit;
        tx_data.chassis_buffer_energy =
            static_cast<int16_t>(ref_data.power_heat.buffer_energy * 100.0f);
        tx_data.supercap_voltage = static_cast<uint16_t>(
            mec_chassis_t::instance()->get_ctx().cap_feedback.vot_cap);
        tx_data.robot_color = ref_data.robot_status.robot_id >= 100 ? 1 : 0;
        tx_data.heat_limit = ref_data.robot_status.shooter_barrel_heat_limit;
        tx_data.heat       = ref_data.power_heat.shooter_42mm_barrel_heat;
        tx_data.robot_x = static_cast<uint16_t>(ref_data.robot_pos.x / 28.0f * 65535.0f);
        tx_data.robot_y = static_cast<uint16_t>(ref_data.robot_pos.y / 15.0f * 65535.0f);
    }
    // 3. 接收逻辑
    if (board_drv_ptr->check_online())
    {
        const auto &rx_data = board_drv_ptr->get_g2c_rx_data();
        (void)rx_data;
    }
}

extern "C"
{
    void hero_board_com_thread(void *argument)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        while (true)
        {
            process_chassis_logic();
            board_drv_ptr->send_data(); // 发送周期包
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void hero_board_com_init(void *argument)
    {
        board_drv_ptr = &board_drv_t::get_instance(board_drv_t::role_t::CHASSIS,
                                                   can_hub_t::can2);
        board_drv_ptr->start_rx();

        xTaskCreate(hero_board_com_thread, "board_com_app", 256, nullptr,
                    configMAX_PRIORITIES - 3, &board_com_task_handle);
        vTaskDelete(nullptr);
    }
}
