/**
 * @file pyro_custom_drv.cpp
 * @brief Implementation of Custom Client Communication Driver.
 */

#include "pyro_custom_drv.h"
#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"
#include "pyro_core_dma_heap.h"
#include "pyro_crc.h"
#include "pyro_dwt_drv.h"
#include <cstring>

namespace pyro
{

status_t custom_drv_t::custom_task_t::init()
{
    if (_owner)
    {
        _owner->init_impl();
        return PYRO_OK;
    }
    return PYRO_ERROR;
}

void custom_drv_t::custom_task_t::run_loop()
{
    if (_owner)
    {
        _owner->run_loop_impl();
    }
}

#ifdef CUSTOM_UART
custom_drv_t &custom_drv_t::get_instance()
{
    static custom_drv_t instance(&CUSTOM_UART);
    return instance;
}
#endif

custom_drv_t::custom_drv_t(uart_drv_t *uart_handle)
    : _uart_drv(uart_handle), _task(nullptr), _tx_buffer(nullptr),
      _rx_msg_buf(nullptr), _rx_data_queue(nullptr), _is_online(false),
      _first_frame(true), _last_seq(0)
{
    memset(&_tx_payload, 0, sizeof(_tx_payload));

    constexpr size_t buf_size = sizeof(tx_packet_t);
    _tx_buffer = static_cast<tx_packet_t *>(pvPortDmaMalloc(buf_size));

    if (_tx_buffer)
        memset(_tx_buffer, 0, buf_size);

    // 创建能容纳 15 包数据的队列，作为重发和缓冲池
    _rx_data_queue = xQueueCreate(15, sizeof(rx_data_t));

    _task          = new custom_task_t(this);
}

custom_drv_t::~custom_drv_t()
{
    if (_task)
    {
        _task->stop();
        delete _task;
        _task = nullptr;
    }
    if (_uart_drv)
        _uart_drv->remove_rx_event_callback(reinterpret_cast<uint32_t>(this));
    if (_tx_buffer)
    {
        vPortFree(_tx_buffer);
        _tx_buffer = nullptr;
    }
    if (_rx_msg_buf)
    {
        vMessageBufferDelete(_rx_msg_buf);
        _rx_msg_buf = nullptr;
    }
    if (_rx_data_queue)
    {
        vQueueDelete(_rx_data_queue);
        _rx_data_queue = nullptr;
    }
}

void custom_drv_t::start_rx() const
{
    if (_task && _uart_drv && _tx_buffer)
        _task->start();
}

void custom_drv_t::init_impl()
{
    // 将 Buffer 容量从 1024 扩大到 4096，防止 ISR 层突发溢出丢包
    if (_rx_msg_buf == nullptr)
        _rx_msg_buf = xMessageBufferCreate(8192);
    if (_rx_msg_buf == nullptr)
        return;

    _uart_drv->add_rx_event_callback(
        [this](const uint8_t *p, uint16_t size, BaseType_t &task_woken) -> bool
        { return this->rx_callback(p, size, task_woken); },
        reinterpret_cast<uint32_t>(this));
}

void custom_drv_t::run_loop_impl()
{
    while (true)
    {
        static rx_packet_t pkt;
        static size_t xReceivedBytes;

        if (xMessageBufferReceive(_rx_msg_buf, &pkt, sizeof(pkt),
                                  portMAX_DELAY) == sizeof(rx_packet_t))
        {
            _is_online       = true;
            if (error_check(&pkt) == PYRO_OK)
            {
                unpack(&pkt);
            }
            _last_rx_time_ms = dwt_drv_t::get_timeline_ms();
        }

        while (_is_online)
        {
            xReceivedBytes = xMessageBufferReceive(
                _rx_msg_buf, &pkt, sizeof(pkt), pdMS_TO_TICKS(1000));

            if (xReceivedBytes == sizeof(rx_packet_t))
            {
                if (error_check(&pkt) == PYRO_OK)
                {
                    unpack(&pkt);
                    float current_time = dwt_drv_t::get_timeline_ms();
                    if (_last_rx_time_ms > 0.0f)
                        _comm_interval_ms = current_time - _last_rx_time_ms;
                    _last_rx_time_ms = current_time;
                }
            }
            else if (xReceivedBytes == 0)
            {
                _is_online        = false;
                _first_frame      = true; // 掉线后重置首次标志
                _comm_interval_ms = 0.0f;
                _last_rx_time_ms  = 0.0f;
            }
        }
    }
}

bool custom_drv_t::rx_callback(const uint8_t *p_data, uint16_t size,
                               BaseType_t &xHigherPriorityTaskWoken) const
{
    if (size == sizeof(rx_packet_t) && p_data[0] == FRAME_SOF)
    {
        xMessageBufferSendFromISR(_rx_msg_buf, p_data, sizeof(rx_packet_t),
                                  &xHigherPriorityTaskWoken);
        return true;
    }
    return false;
}

status_t custom_drv_t::error_check(const rx_packet_t *buf)
{
    if (!verify_crc16_check_sum(reinterpret_cast<uint8_t const *>(buf),
                                sizeof(rx_packet_t)))
        return PYRO_ERROR;
    return PYRO_OK;
}

void custom_drv_t::unpack(const rx_packet_t *buf)
{
    uint16_t current_seq = buf->payload.seq;

    auto seq_diff =
        static_cast<int16_t>(static_cast<uint16_t>(current_seq - _last_seq));
    // if (current_seq == 0)
    // {
    //     _last_seq = current_seq;
    //     if (_rx_data_queue != nullptr)
    //     {
    //         // 如果队列已满，丢弃最老的一包以保证实时性
    //         if (uxQueueSpacesAvailable(_rx_data_queue) == 0)
    //         {
    //             rx_data_t dummy{};
    //             xQueueReceive(_rx_data_queue, &dummy, 0);
    //         }
    //         xQueueSend(_rx_data_queue, &buf->payload, 0);
    //     }
    // }

    if (_first_frame || seq_diff > 0)
    {
        _first_frame = false;
        _last_seq    = current_seq;

        // 将新数据推入队列
        if (_rx_data_queue != nullptr)
        {
            // 如果队列已满，丢弃最老的一包以保证实时性
            if (uxQueueSpacesAvailable(_rx_data_queue) == 0)
            {
                rx_data_t dummy{};
                xQueueReceive(_rx_data_queue, &dummy, 0);
            }
            xQueueSend(_rx_data_queue, &buf->payload, 0);
        }
    }
}

bool custom_drv_t::pop_rx_data(rx_data_t &out_data) const
{
    if (_rx_data_queue == nullptr)
        return false;
    return xQueueReceive(_rx_data_queue, &out_data, 0) == pdTRUE;
}

custom_drv_t::tx_data_t &custom_drv_t::get_tx_data()
{
    return _tx_payload;
}

status_t custom_drv_t::send_data() const
{
    if (!_tx_buffer || !_uart_drv)
        return PYRO_ERROR;

    // 1. 填充帧头数据
    _tx_buffer->header.sof         = FRAME_SOF;//
    _tx_buffer->header.data_length = sizeof(tx_data_t);
    _tx_buffer->header.seq         = _tx_seq++; // 序列号赋值后自增1

    // 2. 计算帧头 CRC8 校验
    append_crc8_check_sum(reinterpret_cast<uint8_t *>(&_tx_buffer->header),
                          sizeof(frame_header_t));

    // 3. 填充数据域与帧尾
    memcpy(&_tx_buffer->payload, &_tx_payload, sizeof(tx_data_t));
    _tx_buffer->tailer.end = '\n';

    // 4. 计算整包 CRC16 校验 (注意减去 end 字符的长度)
    append_crc16_check_sum(reinterpret_cast<uint8_t *>(_tx_buffer),
                           sizeof(tx_packet_t) - 1);

    // 5. 调用底层 UART 进行发送
    return _uart_drv->write(reinterpret_cast<uint8_t *>(_tx_buffer),
                            sizeof(tx_packet_t));
}

bool custom_drv_t::check_online() const
{
    return _is_online;
}
float custom_drv_t::get_comm_interval() const
{
    return _comm_interval_ms;
}

} // namespace pyro
