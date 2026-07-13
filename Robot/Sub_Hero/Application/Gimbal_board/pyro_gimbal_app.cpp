#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_screw_gimbal.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_autoaim_drv.h" // 新增：引入自瞄驱动头文件
#include "pyro_board_drv.h"

using namespace pyro;

// 定义任务通知的位掩码 (Event Bits)

constexpr uint32_t EVENT_BIT_SLING_TOGGLE             = (1 << 1);
constexpr uint32_t EVENT_BIT_SLING_PITCH_PREAIM       = (1 << 2);

static TaskHandle_t gimbal_task_handle                = nullptr;
static pyro::screw_gimbal_t *screw_gimbal_ptr         = nullptr;
static pyro::screw_gimbal_cmd_t *screw_gimbal_cmd_ptr = nullptr;
static pyro::screw_gimbal_deps_t *screw_gimbal_deps   = nullptr;
static pyro::board_drv_t *board_drv_ptr               = nullptr;

// 追踪当前是否处于吊射模式
static bool is_sling_mode                             = false;

static void gimbal_dr162cmd();
static void gimbal_vt032cmd();
static void deps_init();

extern "C"
{
    void hero_gimbal_thread(void *argument)
    {
        static bool prev_gimbal_output = false;

        while (true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

            // 检测按键 R 触发，翻转吊射模式状态
            if (notify_val & EVENT_BIT_SLING_TOGGLE)
            {
                is_sling_mode = !is_sling_mode;
            }
            if ((notify_val & EVENT_BIT_SLING_PITCH_PREAIM) && is_sling_mode)
            {
                screw_gimbal_cmd_ptr->sling_pitch_flag =
                    !screw_gimbal_cmd_ptr->sling_pitch_flag;
            }
            // is_sling_mode = true;


            // 同步给底层 HFSM 状态机
            screw_gimbal_cmd_ptr->sling_mode = is_sling_mode;

            bool current_gimbal_output = false;
            if (board_drv_ptr->check_online())
            {
                current_gimbal_output =
                    board_drv_ptr->get_c2g_rx_data().gimbal_output;
            }

            if (current_gimbal_output && !prev_gimbal_output)
            {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            prev_gimbal_output = current_gimbal_output;

            if (board_drv_ptr->check_online())
            {
                if (current_gimbal_output)
                {
                    if (vt03_drv_t::instance().check_online())
                    {
                        gimbal_vt032cmd();
                    }
                    else if (dr16_drv_t::instance().check_online())
                    {
                        gimbal_dr162cmd();
                    }
                    else
                    {
                        screw_gimbal_cmd_ptr->mode =
                            pyro::cmd_base_t::mode_t::PASSIVE;
                    }
                }
                else
                {
                    screw_gimbal_cmd_ptr->mode =
                        pyro::cmd_base_t::mode_t::PASSIVE;
                }
            }
            else
            {
                screw_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
            }



            screw_gimbal_ptr->set_command(*screw_gimbal_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_gimbal_init(void *argument)
    {
        board_drv_ptr        = &pyro::board_drv_t::get_instance();
        screw_gimbal_cmd_ptr = new pyro::screw_gimbal_cmd_t();
        screw_gimbal_ptr     = pyro::screw_gimbal_t::instance();

        deps_init();
        screw_gimbal_ptr->configure(*screw_gimbal_deps);
        screw_gimbal_ptr->start();

        xTaskCreate(hero_gimbal_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, &gimbal_task_handle);

        auto &vrc = pyro::rc_drv_t::read();

        // 绑定键盘 R 键到吊射模式切换事件
        pyro::btn_broker::subscribe(&vrc.keys.r, pyro::btn_event_t::PRESS_DOWN,
                                    gimbal_task_handle, EVENT_BIT_SLING_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.keys.x, pyro::btn_event_t::PRESS_DOWN,
                                    gimbal_task_handle,
                                    EVENT_BIT_SLING_PITCH_PREAIM);

        vTaskDelete(nullptr);
    }
}

void gimbal_dr162cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::MID != vrc.switches.right.current_pos)
    {
        screw_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
        screw_gimbal_cmd_ptr->pitch_delta_angle = 0;
        screw_gimbal_cmd_ptr->yaw_delta_angle   = 0;
        return;
    }
    screw_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;
    screw_gimbal_cmd_ptr->pitch_delta_angle = -vrc.axes.ry * 0.0025f;
    screw_gimbal_cmd_ptr->yaw_delta_angle   = -vrc.axes.rx * 0.0035f;
}


void gimbal_vt032cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::UP == vrc.switches.gear.current_pos)
    {
        screw_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
        screw_gimbal_cmd_ptr->pitch_delta_angle = 0;
        screw_gimbal_cmd_ptr->yaw_delta_angle   = 0;
        screw_gimbal_cmd_ptr->autoaim_mode      = false;
        return;
    }

    screw_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;

    // --- 新增：判断拨动到 DOWN (右侧) 时进入自瞄状态 ---
    if (pyro::sw_pos_t::DOWN == vrc.switches.gear.current_pos)
    {
        screw_gimbal_cmd_ptr->autoaim_mode = true;

        if (pyro::autoaim_drv_t::get_instance().check_online())
        {
            // 解析并赋值 PC 下发的目标角度
            const auto &rx_data =
                pyro::autoaim_drv_t::get_instance().get_target_data();
            screw_gimbal_cmd_ptr->target_yaw   = rx_data.shoot_yaw;
            screw_gimbal_cmd_ptr->target_pitch = -rx_data.shoot_pitch;
            screw_gimbal_cmd_ptr->pitch_delta_angle =
                -vrc.axes.ry * 0.0025f - vrc.mouse_axes.y * 0.25f;
            screw_gimbal_cmd_ptr->yaw_delta_angle =
                -vrc.axes.rx * 0.0025f - vrc.mouse_axes.x * 0.6f;
        }
        else
        {
            screw_gimbal_cmd_ptr->pitch_delta_angle =
                -vrc.axes.ry * 0.0025f - vrc.mouse_axes.y * 0.25f;
            screw_gimbal_cmd_ptr->yaw_delta_angle =
                -vrc.axes.rx * 0.0025f - vrc.mouse_axes.x * 0.6f;
        }
    }
    else // MID 档位为纯手动控制
    {
        screw_gimbal_cmd_ptr->autoaim_mode =
            false; // 清除外部视觉依赖，仅走手控

        if (is_sling_mode)
        {
            // 吊射模式下：WASD 直接接管云台控制 (W/S控制Pitch, A/D控制Yaw)
            float wasd_pitch = static_cast<float>(
                vrc.keys.w.current_level ? 1
                                         : (vrc.keys.s.current_level ? -1 : 0)) + -vrc.axes.ry * 2.0f;
            float wasd_yaw = static_cast<float>(
                vrc.keys.a.current_level ? 1
                                         : (vrc.keys.d.current_level ? -1 : 0)) + -vrc.axes.rx * 2.0f;

            // 步进系数设为 0.0025f，以保证手感相对平滑
            screw_gimbal_cmd_ptr->pitch_delta_angle = -wasd_pitch * 0.00001f;
            screw_gimbal_cmd_ptr->yaw_delta_angle   = wasd_yaw * 0.000028f;
        }
        else
        {
            // 正常模式：遥控器拨杆或纯鼠标控制
            screw_gimbal_cmd_ptr->pitch_delta_angle =
                -vrc.axes.ry * 0.0025f - vrc.mouse_axes.y * 0.25f;
            screw_gimbal_cmd_ptr->yaw_delta_angle =
                -vrc.axes.rx * 0.0025f - vrc.mouse_axes.x * 0.6f;
        }
    }
}


void deps_init()
{
    screw_gimbal_deps = new pyro::screw_gimbal_deps_t();
    // 1. 初始化电机

    // Pitch motor
    screw_gimbal_deps->motor_deps.pitch =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, can_hub_t::can3);

    // Yaw: 使用 DJI GM6020 (ID 2, CAN1)

    screw_gimbal_deps->motor_deps.yaw = new dji_gm_6020_motor_drv_t(
        dji_motor_tx_frame_t::id_2, can_hub_t::can1);


    // 3. 初始化串级 PID
    // screw_gimbal_deps->pid_deps.pitch_pos =
    //     new pid_t(11.5f, 0.108f, 0.01f, 0.5f, 10.0f, 40, 10,
    //               4); // 位置环输出为 rad/s，限制在电机可接受范围内
    // screw_gimbal_deps->pid_deps.pitch_spd =
    //     new pid_t(22.0f, 0.102f, 0.014f, 1.0f, 20.0f, 20, 10,
    //               4); // 输出限制匹配电机 Nm 级
    screw_gimbal_deps->pid_deps.pitch_pos =
        new pid_t(11.5f, 0.108f, 0.01f, 0.5f, 10.0f, 40, 1, 10, 1,
                  4); // 位置环输出为 rad/s，限制在电机可接受范围内
    screw_gimbal_deps->pid_deps.pitch_spd =
        new pid_t(11000.0f, 51.0f, 7.0f, 500.0f, 11000.0f, 20, 1, 10, 1,
                  4); // 输出限制匹配电机 Nm 级

    screw_gimbal_deps->pid_deps.pitch_auto_pos =
        new pid_t(12.5f, 0.027f, 0.01f, 0.1f, 10.0f, 15, 1, 5, 1,
                  4); // 位置环输出为 rad/s，限制在电机可接受范围内
    screw_gimbal_deps->pid_deps.pitch_auto_spd =
        new pid_t(11500.0f, 25.0f, 7.0f, 300.0f, 10000.0f, 10, 1, 5, 1,
                  4); // 输出限制匹配电机 Nm 级


    // Yaw 轴 (DJI GM6020，输出为电流值/电压值，通常量级较大，如 +/- 30000)
    // screw_gimbal_deps->pid_deps.yaw_pos =
    //     new pid_t(6.2f, 0.01f, 0.22f, 0.8f, 10.0f,100,50,4);
    // screw_gimbal_deps->pid_deps.yaw_spd =
    //     new pid_t(4.0f, 0.0003f, 0.0001f, 0.2f, 3.0f,100,50,4);
    screw_gimbal_deps->pid_deps.yaw_pos =
        new pid_t(8.2f, 0.01f, 0.22f, 0.8f, 10.0f, 100, 1, 50, 1, 4);
    screw_gimbal_deps->pid_deps.yaw_spd =
        new pid_t(5.0f, 0.0003f, 0.0001f, 0.2f, 3.0f, 100, 1, 50, 1, 4);

    screw_gimbal_deps->pid_deps.yaw_relative_pos =
        new pid_t(5.0f, 0.0f, 0.0f, 0.3f, 3.0f, 60, 2, 20, 3, 4);
    screw_gimbal_deps->pid_deps.yaw_relative_spd =
        new pid_t(12.0f, 0.0f, 0.0f, 0.3f, 3.0f, 60, 1, 10, 2, 4);
    screw_gimbal_deps->pid_deps.yaw_pos_leso =
        new leso_t<3>(40,12.221f,20.0f); // LESO 参数配置
    screw_gimbal_deps->pid_deps.yaw_spd_leso =
        new leso_t<2>(50,12.221f,20.0f); // LESO 参数配置
    screw_gimbal_deps->pid_deps.yaw_pos_imu_leso =
        new leso_t<3>(50,12.221f,20.0f); // LESO 参数配置
    screw_gimbal_deps->pid_deps.yaw_spd_imu_leso =
        new leso_t<2>(60,12.221f,20.0f);

    // screw_gimbal_deps->pid_deps.yaw_leso =
    //     new leso_t<3>(50.0f, 14.0f, 20.0f);
    //
    // // 2. 位置环 PID: 保持 Kp=4.0 不变。
    // // 关键修改：将输出滤波从 30Hz 提高到 80Hz (一阶)，减少相位延迟。
    // screw_gimbal_deps->pid_deps.yaw_relative_pos =
    //     new pid_t(4.0f, 0.0f, 0.0f, 0.3f, 5.0f, 80.0f, 1, 0.0f, 1, 2);
    //
    // // 3. 速度环 PID: 保持 Kp=10.0 不变。
    // // 关键修改：将输出滤波提高到 80Hz (一阶)，给电流输出更快的响应速度。
    // screw_gimbal_deps->pid_deps.yaw_relative_spd =
    //     new pid_t(10.0f, 0.0f, 0.0f, 0.3f, 5.0f, 80.0f, 1, 0.0f, 1, 2);
}
