/**
 * @file pyro_custom_com.cpp
 * @brief Custom Client Forwarding Task
 * 负责处理 PC 与 裁判系统图传链路（Image Link）之间的透明双向转发：
 * 1. PC (rx_data) -> 裁判系统 (0x0310) （带队列缓存、重发与严格 50Hz 限流）
 * 2. 裁判系统 (0x0311) -> PC (tx_data)
 */

#include "pyro_custom_drv.h"
#include "pyro_image_drv.h"
#include "pyro_dwt_drv.h" // 引入时间戳功能用于速率控制

#include <cstring>

using namespace pyro;

/* ========================================================================== */
/* Static Variables                                                           */
/* ========================================================================== */
static TaskHandle_t custom_app_task_handle = nullptr;
static custom_drv_t *custom_drv_ptr = nullptr;
static image_drv_t *image_drv_ptr = nullptr;

// 新增：用于速率控制与重发的静态变量
static custom_drv_t::rx_data_t pending_pc_data;
static bool has_pending_data = false;
static float last_send_time_ms = 0.0f;

/* ========================================================================== */
/* Helper Function Declarations                                               */
/* ========================================================================== */
static void forward_pc_to_referee();
static void forward_referee_to_pc();

/* ========================================================================== */
/* FreeRTOS Task Entries                                                      */
/* ========================================================================== */
extern "C"
{
    void hero_custom_app_thread(void *argument)
    {
        // 延时等待底层设备初始化完成
        vTaskDelay(pdMS_TO_TICKS(500));

        while (true)
        {
            // 1. 将 PC 上位机发来的自定义指令转发至裁判系统图传链路
            forward_pc_to_referee();

            // 2. 将裁判系统图传链路收到的自定义指令转发至 PC 上位机
            forward_referee_to_pc();

            // 3. 延时让出 CPU
            // 轮询速率 2ms
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    void hero_custom_init(void *argument)
    {
        custom_drv_ptr = &pyro::custom_drv_t::get_instance();
        image_drv_ptr = &pyro::image_drv_t::get_instance();

        // 启动 PC 串口的接收和解析任务
        custom_drv_ptr->start_rx();

        // 创建应用层数据透传线程
        xTaskCreate(hero_custom_app_thread, "custom_app_thread", 256, nullptr,
                    configMAX_PRIORITIES - 4, &custom_app_task_handle);

        // 初始化完成，删除自身
        vTaskDelete(nullptr);
    }
}

/* ========================================================================== */
/* Helper Function Implementations                                            */
/* ========================================================================== */

/**
 * @brief 将 PC 数据透传给裁判系统 (0x0310)，严格限制最大 50Hz，自带忙碌重试
 */
static void forward_pc_to_referee()
{
    // 如果链路断开，清空积压的队列与状态标志
    if (!custom_drv_ptr->check_online()) {
        has_pending_data = false;
        custom_drv_t::rx_data_t dummy{};
        while(custom_drv_ptr->pop_rx_data(dummy)) {} // 清空陈旧数据
        return;
    }

    // 1. 若当前没有待发送的数据，从队列中取一包新数据
    if (!has_pending_data) {
        has_pending_data = custom_drv_ptr->pop_rx_data(pending_pc_data);
    }

    // 2. 如果手头有待发数据，执行速率控制与发送检测
    if (has_pending_data) {
        float current_time = dwt_drv_t::get_timeline_ms();

        // 进行基础流控，主要流控由pc端完成
        if ((current_time - last_send_time_ms) >= 16.0f) {

            // 拿到图传链路的 DMA 零拷贝发送句柄
            auto &image_tx_data = image_drv_ptr->get_client_tx_data();
            memcpy(image_tx_data.data, &pending_pc_data, sizeof(pyro::custom_drv_t::rx_data_t));

            // 触发图传链路发送
            // 只要 UART 驱动里实现了“DMA 忙碌时返回 PYRO_ERROR”，这里的判定就能完美实现重试
            if (image_drv_ptr->send_client_data() == PYRO_OK) {
                // 发送成功，清除挂起标志位，记录发送时间
                has_pending_data = false;
                last_send_time_ms = current_time;
            }
            // 若返回 ERROR (DMA忙碌)，本轮不更新时间和标志位，下次 2ms 轮询到此时会继续尝试重发
        }
    }
}

/**
 * @brief 将裁判系统数据 (最大 30 字节, 0x0311) 透传给 PC
 */
static void forward_referee_to_pc()
{
    // 检查图传链路是否在线
    if (!image_drv_ptr->is_online()) return;

    // 从图传驱动获取 0x0311 下发指令的只读指针
    const uint8_t* referee_rx_data = image_drv_ptr->get_client_rx_cmd_data();

    // 拿到发往 PC 驱动的待发送数据载荷引用
    auto &pc_tx_data = custom_drv_ptr->get_tx_data();

    // 拷贝裁判系统数据到 PC 发送区
    memcpy(pc_tx_data.data, referee_rx_data, sizeof(pc_tx_data.data));

    // 触发 PC 链路打包并发送（若需要实时回传可取消注释）
    // custom_drv_ptr->send_data();
}
