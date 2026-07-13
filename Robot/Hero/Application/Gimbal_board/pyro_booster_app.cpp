#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_quad_booster.h"
#include "pyro_com_cantx.h"
#include "pyro_quad_booster.h"
#include "pyro_com_canrx.h"
#include "pyro_autoaim_drv.h"
#include "pyro_board_drv.h"
#include "pyro_dm_motor_drv.h"

namespace pyro
{
class dm_motor_drv_t;
}
using namespace pyro;

extern float read_time;

// 定义任务通知的位掩码 (Event Bits)
constexpr uint32_t EVENT_BIT_FRIC_TOGGLE         = (1 << 0);
constexpr uint32_t EVENT_BIT_FIRE                = (1 << 1);
constexpr uint32_t EVENT_BIT_FORCE_DEPLOY_TOGGLE = (1 << 2);
constexpr uint32_t EVENT_BIT_FRIC_ON             = (1 << 3);
constexpr uint32_t EVENT_BIT_FRIC_OFF            = (1 << 4);
constexpr uint32_t EVENT_BIT_TRIGGER_RESET       = (1 << 5);
constexpr uint32_t EVENT_BIT_SHOOT_DATA_RESET    = (1 << 6);
constexpr uint32_t EVENT_BIT_ANTI_JAM_START      = (1 << 7);
constexpr uint32_t EVENT_BIT_ANTI_JAM_STOP       = (1 << 8);

static TaskHandle_t booster_task_handle   = nullptr;
static pyro::quad_booster_t *quad_booster_ptr         = nullptr;
static pyro::quad_booster_cmd_t *quad_booster_cmd_ptr = nullptr;
static pyro::quad_deps_t *quad_deps_ptr               = nullptr;
static pyro::board_drv_t *board_drv_ptr               = nullptr;

static void deps_init();

extern "C"
{
    void booster_dr162cmd(uint32_t notify_val)
    {
        pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
        auto &vrc = pyro::rc_drv_t::read();

        if (pyro::sw_pos_t::DOWN == vrc.switches.right.current_pos)
        {
            quad_booster_cmd_ptr->mode    = pyro::cmd_base_t::mode_t::PASSIVE;
            quad_booster_cmd_ptr->fric_on = false;
            quad_booster_cmd_ptr->anti_jam = false;
            return;
        }

        quad_booster_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;

        if (notify_val & EVENT_BIT_FRIC_TOGGLE)
        {
            quad_booster_cmd_ptr->fric_on = !quad_booster_cmd_ptr->fric_on;
            quad_booster_cmd_ptr->anti_jam = false;
        }

        if (pyro::sw_pos_t::MID == vrc.switches.right.current_pos)
        {
            if (notify_val & EVENT_BIT_FIRE)
            {
                quad_booster_cmd_ptr->fire_count++;
            }
        }
    }

    void booster_vt032cmd(uint32_t notify_val)
    {
        pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
        auto &vrc                   = pyro::rc_drv_t::read();

        static uint8_t last_pc_fire = 0;
        static bool force_deploy    = false;

        if (pyro::sw_pos_t::UP == vrc.switches.gear.current_pos)
        {
            quad_booster_cmd_ptr->mode    = pyro::cmd_base_t::mode_t::PASSIVE;
            quad_booster_cmd_ptr->fric_on = false;
            quad_booster_cmd_ptr->anti_jam = false;
            last_pc_fire                  = 0;
            return;
        }

        quad_booster_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;

        if (notify_val & EVENT_BIT_FRIC_TOGGLE)
        {
            quad_booster_cmd_ptr->fric_on = !quad_booster_cmd_ptr->fric_on;
            quad_booster_cmd_ptr->anti_jam = false;
        }

        // Q/E set fric state, Ctrl+F forces deploy shoot-data, B resets trigger wheel.
        if (notify_val & EVENT_BIT_FRIC_ON)
        {
            quad_booster_cmd_ptr->fric_on = true;
            quad_booster_cmd_ptr->anti_jam = false;
        }
        if (notify_val & EVENT_BIT_FRIC_OFF)
        {
            quad_booster_cmd_ptr->fric_on = false;
            quad_booster_cmd_ptr->anti_jam = false;
        }
        if (notify_val & EVENT_BIT_ANTI_JAM_START)
        {
            quad_booster_cmd_ptr->fric_on = false;
            quad_booster_cmd_ptr->anti_jam = true;
        }
        if (notify_val & EVENT_BIT_ANTI_JAM_STOP)
        {
            quad_booster_cmd_ptr->anti_jam = false;
        }

        if ((notify_val & EVENT_BIT_FORCE_DEPLOY_TOGGLE) &&
            vrc.keys.ctrl.current_level)
        {
            force_deploy = !force_deploy;
        }
        quad_booster_cmd_ptr->force_deploy = force_deploy;
        if (notify_val & EVENT_BIT_TRIGGER_RESET)
        {
            quad_booster_cmd_ptr->reset_count++;
        }
        if (notify_val & EVENT_BIT_SHOOT_DATA_RESET)
        {
            quad_booster_cmd_ptr->shoot_data_reset_count++;
        }

        if (pyro::sw_pos_t::DOWN == vrc.switches.gear.current_pos)
        {
            if (pyro::autoaim_drv_t::get_instance().check_online())
            {
                const auto &rx_data =
                    pyro::autoaim_drv_t::get_instance().get_target_data();

                if (rx_data.fire && !last_pc_fire)
                {
                    quad_booster_cmd_ptr->fire_count++;
                }
                last_pc_fire = rx_data.fire;
            }
            else
            {
                last_pc_fire = 0;
            }
        }
        else
        {
            last_pc_fire = 0;
        }

        quad_booster_cmd_ptr->trig_target_spd = 14.0f * vrc.axes.rx;

        if (notify_val & EVENT_BIT_FIRE)
        {
            quad_booster_cmd_ptr->fire_count++;
        }
    }

    void hero_booster_thread(void *argument)
    {
        static bool prev_booster_output = false;

        while (true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

            bool current_booster_output = false;
            if (board_drv_ptr->check_online())
            {
                current_booster_output =
                    board_drv_ptr->get_c2g_rx_data().booster_output;
            }

            if (current_booster_output && !prev_booster_output)
            {
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
            prev_booster_output = current_booster_output;

            if (board_drv_ptr->check_online())
            {
                if (current_booster_output)
                {
                    if (vt03_drv_t::instance().check_online())
                    {
                        booster_vt032cmd(notify_val);
                    }
                    else if (dr16_drv_t::instance().check_online())
                    {
                        booster_dr162cmd(notify_val);
                    }
                    else
                    {
                        quad_booster_cmd_ptr->mode =
                            pyro::cmd_base_t::mode_t::PASSIVE;
                        quad_booster_cmd_ptr->anti_jam = false;
                    }
                }
                else
                {
                    quad_booster_cmd_ptr->mode =
                        pyro::cmd_base_t::mode_t::PASSIVE;
                    quad_booster_cmd_ptr->anti_jam = false;
                }
            }
            else
            {
                quad_booster_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
                quad_booster_cmd_ptr->anti_jam = false;
            }


            quad_booster_ptr->set_command(*quad_booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_booster_init(void *argument)
    {
        board_drv_ptr        = &pyro::board_drv_t::get_instance();
        quad_booster_ptr     = pyro::quad_booster_t::instance();
        quad_booster_cmd_ptr = new pyro::quad_booster_cmd_t();

        deps_init();
        quad_booster_ptr->configure(*quad_deps_ptr);

        xTaskCreate(hero_booster_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, &booster_task_handle);

        auto &vrc = pyro::rc_drv_t::read();

        // --- VT03 按键绑定 ---
        pyro::btn_broker::subscribe(&vrc.buttons.fn_l,
                                    pyro::btn_event_t::PRESS_DOWN,
                                    booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.keys.q, pyro::btn_event_t::PRESS_DOWN,
                                    booster_task_handle, EVENT_BIT_FRIC_ON);
        pyro::btn_broker::subscribe(&vrc.keys.e, pyro::btn_event_t::PRESS_DOWN,
                                    booster_task_handle, EVENT_BIT_FRIC_OFF);
        pyro::btn_broker::subscribe(&vrc.keys.e,
                                    pyro::btn_event_t::LONG_PRESS_START,
                                    booster_task_handle,
                                    EVENT_BIT_ANTI_JAM_START);
        pyro::btn_broker::subscribe(&vrc.keys.e, pyro::btn_event_t::PRESS_UP,
                                    booster_task_handle,
                                    EVENT_BIT_ANTI_JAM_STOP);

        // Ctrl+F forces the booster deploy shoot-data set.
        pyro::btn_broker::subscribe(&vrc.keys.f, pyro::btn_event_t::PRESS_DOWN,
                                    booster_task_handle,
                                    EVENT_BIT_FORCE_DEPLOY_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.keys.f,
                                    pyro::btn_event_t::LONG_PRESS_START,
                                    booster_task_handle,
                                    EVENT_BIT_SHOOT_DATA_RESET);
        pyro::btn_broker::subscribe(&vrc.keys.b, pyro::btn_event_t::PRESS_DOWN,
                                    booster_task_handle,
                                    EVENT_BIT_TRIGGER_RESET);

        pyro::btn_broker::subscribe(&vrc.buttons.trigger,
                                    pyro::btn_event_t::PRESS_DOWN,
                                    booster_task_handle, EVENT_BIT_FIRE);
        pyro::btn_broker::subscribe(&vrc.buttons.press_l,
                                    pyro::btn_event_t::PRESS_DOWN,
                                    booster_task_handle, EVENT_BIT_FIRE);

        // --- DR16 拨杆绑定 ---
        pyro::sw_broker::subscribe(&vrc.switches.left,
                                   pyro::sw_event_t::UP_TO_MID,
                                   booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
        pyro::sw_broker::subscribe(&vrc.switches.left,
                                   pyro::sw_event_t::DOWN_TO_MID,
                                   booster_task_handle, EVENT_BIT_FIRE);

        quad_booster_ptr->start();
        vTaskDelete(nullptr);
    }
}

void deps_init()
{
    quad_deps_ptr                            = new pyro::quad_deps_t();
    quad_deps_ptr->motor_deps.fric_wheels[0] = new pyro::dji_m3508_motor_drv_t(
        pyro::dji_motor_tx_frame_t::id_1, pyro::can_hub_t::can2);
    quad_deps_ptr->motor_deps.fric_wheels[1] = new pyro::dji_m3508_motor_drv_t(
        pyro::dji_motor_tx_frame_t::id_2, pyro::can_hub_t::can2);
    quad_deps_ptr->motor_deps.fric_wheels[2] = new pyro::dji_m3508_motor_drv_t(
        pyro::dji_motor_tx_frame_t::id_3, pyro::can_hub_t::can2);
    quad_deps_ptr->motor_deps.fric_wheels[3] = new pyro::dji_m3508_motor_drv_t(
        pyro::dji_motor_tx_frame_t::id_4, pyro::can_hub_t::can2);
    quad_deps_ptr->motor_deps.trigger_wheel =
        new pyro::dm_motor_drv_t(0x51, 0x61, pyro::can_hub_t::can1);

    static_cast<dm_motor_drv_t *>(quad_deps_ptr->motor_deps.trigger_wheel)
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(quad_deps_ptr->motor_deps.trigger_wheel)
        ->set_rotate_range(-30.0f, 30.0f);
    static_cast<dm_motor_drv_t *>(quad_deps_ptr->motor_deps.trigger_wheel)
        ->set_torque_range(-7.0f, 7.0f);

    quad_deps_ptr->pid_deps.fric_pid[0] =
        new pid_t(6.375f, 0.02f, 0.02f, 2.5f, 20, 320, 1, 80, 1, 4);
    quad_deps_ptr->pid_deps.fric_pid[1] =
        new pid_t(11.315f, 0.03f, 0.004f, 2.5f, 20, 240, 1, 80, 1, 4);
    quad_deps_ptr->pid_deps.fric_pid[2] =
        new pid_t(6.410f, 0.02f, 0.02f, 2.5f, 20, 320, 1, 80, 1, 4);
    quad_deps_ptr->pid_deps.fric_pid[3] =
        new pid_t(11.315f, 0.03f, 0.004f, 2.5f, 20, 240, 1, 80, 1, 4);

    quad_deps_ptr->pid_deps.trigger_pos_pid =
        new pid_t(8.0f, 0.03f, 0.0015f, 0.3f, 2.0f, 40, 1, 20, 1, 4);
    quad_deps_ptr->pid_deps.trigger_spd_pid =
        new pid_t(0.9f, 0.9f, 0.0015f, 1.0f, 5.0f, 30, 1, 20, 1, 4);
}
