/**
 * @file pyro_board_com.cpp
 * @brief 板间通信业务逻辑层 - 云台端 (Gimbal)
 */

#include "pyro_board_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_module_base.h"
#include "pyro_quad_booster.h"
#include "pyro_rc_base_drv.h"
#include "pyro_screw_gimbal.h"
#include "pyro_vt03_rc_drv.h"

using namespace pyro;

constexpr uint32_t EVENT_BIT_SLING_TOGGLE = (1 << 0);
constexpr uint32_t EVENT_BIT_C_PRESS      = (1 << 1);

static TaskHandle_t board_com_task_handle = nullptr;
static board_drv_t *board_drv_ptr         = nullptr;

static void process_gimbal_logic(uint32_t notify_val)
{
    auto &tx_data = board_drv_ptr->get_g2c_tx_data();

    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc              = pyro::rc_drv_t::read();

    static bool sling_mode = false;

    if (vt03_drv_t::instance().check_online())
    {
        if (notify_val & EVENT_BIT_SLING_TOGGLE)
        {
            sling_mode = !sling_mode;
        }

        if (sling_mode)
        {
            tx_data.active = false;
            tx_data.vx     = 0;
            tx_data.vy     = 0;
            tx_data.wz     = 0;
        }
        else
        {
            if (sw_pos_t::UP == vrc.switches.gear.current_pos)
            {
                tx_data.active = false;
                tx_data.vx     = 0;
                tx_data.vy     = 0;
                tx_data.wz     = 0;
            }
            else
            {
                tx_data.active = true;
                tx_data.vx = static_cast<int8_t>(vrc.keys.w.current_level ? 127
                                                 : vrc.keys.s.current_level
                                                     ? -127
                                                     : vrc.axes.ly * 127);
                tx_data.vy = static_cast<int8_t>(vrc.keys.a.current_level ? 127
                                                 : vrc.keys.d.current_level
                                                     ? -127
                                                     : -vrc.axes.lx * 127);
                tx_data.wz = static_cast<int8_t>(
                    vrc.keys.shift.current_level ? 127 : vrc.axes.wheel * 127);
            }
        }
        if (notify_val & EVENT_BIT_C_PRESS)
        {
            tx_data.ui_refresh = !tx_data.ui_refresh;
        }
    }
    else if (dr16_drv_t::instance().check_online())
    {
        if (sw_pos_t::MID != vrc.switches.right.current_pos)
        {
            tx_data.active = false;
            tx_data.vx     = 0;
            tx_data.vy     = 0;
            tx_data.wz     = 0;
        }
        else
        {
            tx_data.active = true;
            tx_data.vx     = static_cast<int8_t>(vrc.axes.ly * 127);
            tx_data.vy     = static_cast<int8_t>(-vrc.axes.lx * 127);
            tx_data.wz     = 0;
        }
    }
    else
    {
        tx_data.active = false;
        tx_data.vx     = 0;
        tx_data.vy     = 0;
        tx_data.wz     = 0;
    }
    // tx_data.active      = false;


    if (abs(screw_gimbal_t::instance()->get_ctx().data.relative_pitch_rad) >=
        1.0f)
    {
        tx_data.pitch_rad = 32767;
    }
    else
    {
        tx_data.pitch_rad = static_cast<int16_t>(
            screw_gimbal_t::instance()->get_ctx().data.relative_pitch_rad *
            32767.0f);
    }
    if (quad_booster_t::instance()->get_ctx().data.target_shoot_speed < 0.0f ||
        quad_booster_t::instance()->get_ctx().data.target_shoot_speed > 20.0f)
    {
        tx_data.target_shoot_spd = 250;
    }
    else
    {
        tx_data.target_shoot_spd = static_cast<uint8_t>(
            quad_booster_t::instance()->get_ctx().data.target_shoot_speed *
            10.0f);
    }
    tx_data.fric_en    = quad_booster_t::instance()->get_ctx().cmd->fric_on;
    tx_data.fric_err   = quad_booster_t::instance()->get_ctx().data.fric_err;
    tx_data.sling_mode = sling_mode;
    tx_data.trigger_located =
        quad_booster_t::instance()->get_ctx().data.trigger_located;
    tx_data.fire_ready =
        quad_booster_t::instance()->get_ctx().data.ready_state_flag;

    if (board_drv_ptr->check_online())
    {
        // 预留：读取底盘发来的反馈信息并处理
        // const auto &rx_data = board_drv_ptr->get_c2g_rx_data();
    }
}

extern "C"
{
    void hero_board_com_thread(void *argument)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        while (true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);
            process_gimbal_logic(notify_val);
            board_drv_ptr->send_data();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void hero_board_com_init(void *argument)
    {


        board_drv_ptr = &board_drv_t::get_instance(board_drv_t::role_t::GIMBAL,
                                                   can_hub_t::can1);
        board_drv_ptr->start_rx();

        xTaskCreate(hero_board_com_thread, "board_com_app", 256, nullptr,
                    configMAX_PRIORITIES - 3, &board_com_task_handle);

        auto &vrc = pyro::rc_drv_t::read();
        pyro::btn_broker::subscribe(&vrc.keys.r, pyro::btn_event_t::PRESS_DOWN,
                                    board_com_task_handle,
                                    EVENT_BIT_SLING_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.keys.c, pyro::btn_event_t::PRESS_DOWN,
                                    board_com_task_handle, EVENT_BIT_C_PRESS);
        vTaskDelete(nullptr);
    }
}
