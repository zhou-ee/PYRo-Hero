/**
 * @file pyro_custom_drv.h
 * @brief Custom Client Communication Driver (MCU <-> MiniPC)
 *
 * 负责与 PC 上位机进行自定义客户端数据的双向通信，采用与自瞄完全相同的架构：
 * 1. 组合模式内部维护 Task
 * 2. MessageBuffer 传递变长或定长包
 * 3. DMA 零拷贝发送
 * 4. 增加 Queue 队列缓存，防止高频突发流量丢包
 */

#ifndef __PYRO_CUSTOM_DRV_H__
#define __PYRO_CUSTOM_DRV_H__

#include "pyro_task.h"
#include "pyro_uart_drv.h"
#include "pyro_core_def.h"
#include "message_buffer.h"
#include "queue.h"

namespace pyro
{

class custom_drv_t
{
  public:
/* ================= User Data Payload ================= */
#pragma pack(push, 1)

    /**
     * @brief [MCU -> PC] 下行透传数据 (对应裁判系统 0x0311，最多 30 bytes)
     */
    struct tx_data_t
    {
        uint8_t data[30];
    };

    /**
     * @brief [PC -> MCU] 上行透传数据 (对应裁判系统 0x0310，最多 300 bytes)
     */
    struct rx_data_t
    {
        uint16_t seq;
        uint64_t timestamp;
        uint8_t data[290];
    };

#pragma pack(pop)

#ifdef CUSTOM_UART
    /* Public Methods --------------------------------------------------------*/
    static custom_drv_t &get_instance();
#endif

    /**
     * @brief 启动 PC 串口接收任务
     */
    void start_rx() const;

    /**
     * @brief 获取待发给 PC 数据的引用 (位于 DMA 内存区)
     */
    tx_data_t &get_tx_data();

    /**
     * @brief 打包发送透传数据到 PC
     */
    status_t send_data() const;

    /**
     * @brief 从队列中获取一包新的 PC 数据
     * @param out_data 输出数据引用
     * @return true 如果成功获取到数据
     */
    [[nodiscard]] bool pop_rx_data(rx_data_t &out_data) const;

    /**
     * @brief 检查 PC 是否在线
     */
    [[nodiscard]] bool check_online() const;

    /**
     * @brief 获取最新通信间隔 (ms)
     */
    [[nodiscard]] float get_comm_interval() const;

  private:
    explicit custom_drv_t(uart_drv_t *uart_handle);
    ~custom_drv_t();

    /* Private Task Implementation -------------------------------------------*/
    class custom_task_t final : public task_base_t
    {
      public:
        explicit custom_task_t(custom_drv_t *owner_ptr)
            : task_base_t("custom_task", 256, 512, priority_t::NORMAL),
              _owner(owner_ptr) {}
      protected:
        status_t init() override;
        void run_loop() override;
      private:
        custom_drv_t *_owner;
    };

/* Private Protocol Types ------------------------------------------------*/
#pragma pack(push, 1)
    struct frame_header_t
    {
        uint8_t sof;
        uint16_t data_length;
        uint8_t seq;
        uint8_t crc8;
    };
    struct frame_tailer_t { uint16_t crc16; uint8_t end; }; // '\n'
    struct rx_frame_tailer_t { uint16_t crc16; };

    struct tx_packet_t
    {
        frame_header_t header;
        tx_data_t payload;
        frame_tailer_t tailer;
    };

    struct rx_packet_t
    {
        frame_header_t header;
        uint16_t cmd_id;
        rx_data_t payload;
        rx_frame_tailer_t tailer;
    };
#pragma pack(pop)

    /* Private Members -------------------------------------------------------*/
    uart_drv_t *_uart_drv;
    custom_task_t *_task;
    tx_packet_t *_tx_buffer; // DMA buffer
    MessageBufferHandle_t _rx_msg_buf;
    QueueHandle_t _rx_data_queue; // 接收数据缓存队列

    tx_data_t _tx_payload{};

    bool _is_online;
    bool _first_frame;      // 用于初始化序列号检测
    uint16_t _last_seq;     // 记录上一次成功接收的序列号

    float _comm_interval_ms{0.0f};
    float _last_rx_time_ms{0.0f};

    mutable uint8_t _tx_seq{0}; // 新增：发送序列号，使用 mutable 以便在 const 方法中自增

    static constexpr uint8_t FRAME_SOF = 0xA5; // 修改：固定的 SOF 为 0xA5
    static constexpr uint16_t CMD_ID = 0x0310;

    /* Private Methods (Logic) -----------------------------------------------*/
    void init_impl();
    void run_loop_impl();

    bool rx_callback(const uint8_t *p_data, uint16_t size, BaseType_t& xHigherPriorityTaskWoken) const;
    static status_t error_check(const rx_packet_t *buf);
    void unpack(const rx_packet_t *buf);
};

} // namespace pyro

#endif // __PYRO_CUSTOM_DRV_H__