/**
 * @file pyro_autoaim_com.cpp
 * @brief Autoaim Application Task Framework
 * * 负责处理与 MiniPC 的上层业务逻辑：
 * 1. 定期将云台姿态打包发送给 PC
 * 2. 接收 PC 传回的自瞄目标角度并下发给云台控制器
 */

#include "pyro_autoaim_drv.h"
#include "pyro_module_base.h"
#include "pyro_screw_gimbal.h"
#include "pyro_quad_booster.h"

using namespace pyro;

/* ========================================================================== */
/* Static Variables                                                           */
/* ========================================================================== */
static TaskHandle_t autoaim_app_task_handle = nullptr;
static pyro::autoaim_drv_t *autoaim_drv_ptr = nullptr;

/* ========================================================================== */
/* Helper Function Declarations                                               */
/* ========================================================================== */
static void
process_pc_target_data(const pyro::autoaim_drv_t::rx_data_t &rx_data);
static void update_and_send_feedback();

/* ========================================================================== */
/* FreeRTOS Task Entries                                                      */
/* ========================================================================== */
extern "C"
{
    /**
     * @brief 自动瞄准应用层主循环线程
     */
    void hero_autoaim_app_thread(void *argument)
    {
        // 延时等待底层设备初始化完成
        vTaskDelay(pdMS_TO_TICKS(500));

        while (true)
        {
            // 1. 检查 PC 视觉是否在线
            if (autoaim_drv_ptr->check_online())
            {
                // 获取 PC 下发的目标数据
                const auto &rx_data = autoaim_drv_ptr->get_target_data();

                // 处理接收到的数据，将目标角度和射击指令下发给模块
                process_pc_target_data(rx_data);
            }

            // 2. 无论是否收到数据，都以固定频率向 PC 发送当前云台姿态和弹速
            update_and_send_feedback();

            // 3. 延时让出 CPU
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    /**
     * @brief 自动瞄准应用层初始化函数 (由 FreeRTOS 启动时调用)
     */
    void hero_autoaim_init(void *argument)
    {
        // 1. 获取底层驱动实例
        autoaim_drv_ptr = &pyro::autoaim_drv_t::get_instance();

        // 2. 启动驱动层的接收和解析任务
        autoaim_drv_ptr->start_rx();

        // 3. 创建应用层业务线程
        xTaskCreate(hero_autoaim_app_thread, "autoaim_app_thread", 256, nullptr,
                    configMAX_PRIORITIES - 3, &autoaim_app_task_handle);

        // 4. 初始化完成，删除自身
        vTaskDelete(nullptr);
    }
}

/* ========================================================================== */
/* Helper Function Implementations                                            */
/* ========================================================================== */

/**
 * @brief 解析并处理来自 PC 的自瞄控制数据
 */
static void
process_pc_target_data(const pyro::autoaim_drv_t::rx_data_t &rx_data)
{

}

/**
 * @brief 获取当前 MCU 状态，并将其发送给 PC
 */
static void update_and_send_feedback()
{
    // 1. 获取待发送数据的引用
    auto &tx_data = autoaim_drv_ptr->get_tx_data();

    // 2. 获取当前的运行上下文
    auto gimbal_ctx = pyro::screw_gimbal_t::instance()->get_ctx();
    auto booster_ctx = pyro::quad_booster_t::instance()->get_ctx();

    // 3. 填充云台位姿 (使用 IMU 绝对姿态)
    tx_data.curr_yaw = gimbal_ctx.data.yaw_imu_rad;
    tx_data.curr_pitch = -gimbal_ctx.data.pitch_imu_rad;

    // 4. 填充发弹数据与发弹预测延迟
    tx_data.curr_speed = booster_ctx.shoot_normal_data.avg_ball_speed;
    tx_data.shoot_delay = static_cast<uint16_t>(booster_ctx.data.avg_launch_delay);
    tx_data.fire_count = booster_ctx.data.fire_count;

    // 5. 填充标志位
    tx_data.state       = 0; // 需接入裁判系统
    tx_data.enemy_color = 0; // 需接入裁判系统
    tx_data.autoaim = (gimbal_ctx.cmd && gimbal_ctx.cmd->autoaim_mode) ? 1 : 0;

    // 6. 触发底层 DMA 发送
    autoaim_drv_ptr->send_data();
}