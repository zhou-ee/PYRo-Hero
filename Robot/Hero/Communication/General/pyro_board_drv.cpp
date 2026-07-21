/**
 * @file pyro_board_drv.cpp
 */

#include "pyro_board_drv.h"

#include "pyro_can_drv.h"
#include "pyro_bsp_can.h"

#include "pyro_dwt_drv.h"
#include <cstring>
#include <algorithm>  // for std::min

namespace pyro
{

status_t board_drv_t::board_task_t::init()
{
    if (_owner)
    {
        _owner->init_impl();
        return PYRO_OK;
    }
    return PYRO_ERROR;
}

void board_drv_t::board_task_t::run_loop()
{
    if (_owner)
    {
        _owner->run_loop_impl();
    }
}

board_drv_t &board_drv_t::get_instance(const role_t role,
                                       const bsp_can::which_can can_ch)
{
    static board_drv_t instance(role, can_ch);
    return instance;
}

board_drv_t::board_drv_t(const role_t role, const bsp_can::which_can can_ch)
    : _role(role), _can_ch(can_ch), _task(nullptr), _is_online(false),
      _last_rx_time_ms(0.0f)
{
    for (auto &buf : _g2c_buffers) buf = nullptr;
    for (auto &buf : _c2g_buffers) buf = nullptr;
    for (auto &buf : _event_buffers) buf = nullptr;
    _task = new board_task_t(this);
}

board_drv_t::~board_drv_t()
{
    if (_task)
    {
        _task->stop();
        delete _task;
    }
    // 清理动态分配的 buffer（可选）
    for (auto &buf : _g2c_buffers) delete buf;
    for (auto &buf : _c2g_buffers) delete buf;
    for (auto &buf : _event_buffers) delete buf;
}

void board_drv_t::start_rx() const
{
    if (_task)
        _task->start();
}

void board_drv_t::init_impl()
{
    can_drv_t* can_drv = bsp_can::get_can(_can_ch);
    if (!can_drv) return;

    if (_role == role_t::CHASSIS)
    {
        for (uint8_t i = 0; i < G2C_FRAME_CNT; ++i)
        {
            _g2c_buffers[i] = new can_msg_buffer_t(G2C_BASE_ID + i);
            can_drv->register_rx_msg(_g2c_buffers[i]);
        }
    }
    else // GIMBAL
    {
        for (uint8_t i = 0; i < C2G_FRAME_CNT; ++i)
        {
            _c2g_buffers[i] = new can_msg_buffer_t(C2G_BASE_ID + i);
            can_drv->register_rx_msg(_c2g_buffers[i]);
        }

        const auto shoot_cnt =
            static_cast<uint8_t>((sizeof(event_shoot_t) + 7) / 8);
        for (uint8_t i = 0; i < shoot_cnt; ++i)
        {
            _event_buffers[i] = new can_msg_buffer_t(EVENT_C2G_SHOOT + i);
            can_drv->register_rx_msg(_event_buffers[i]);
        }
    }
}

void board_drv_t::run_loop_impl()
{
    while (true)
    {
        std::array<uint8_t, 8> raw{};
        float now = dwt_drv_t::get_timeline_ms();

        if (_role == role_t::CHASSIS)
        {
            auto *dst = reinterpret_cast<uint8_t *>(&_latest_g2c_rx);
            for (uint8_t i = 0; i < G2C_FRAME_CNT; ++i)
            {
                if (_g2c_buffers[i] && _g2c_buffers[i]->is_fresh()) {
                    _g2c_buffers[i]->get_data(raw);
                    size_t len = std::min<size_t>(sizeof(g2c_data_t) - i*8, 8);
                    memcpy(dst + i*8, raw.data(), len);
                    _is_online = true;
                    _last_rx_time_ms = now;
                    _g2c_buffers[i]->mark_read();
                }
            }
        }
        else // GIMBAL
        {
            // ★ 修复：使用 _latest_c2g_rx，而不是 _latest_g2c_rx
            auto *dst = reinterpret_cast<uint8_t *>(&_latest_c2g_rx);
            for (uint8_t i = 0; i < C2G_FRAME_CNT; ++i)
            {
                if (_c2g_buffers[i] && _c2g_buffers[i]->is_fresh()) {
                    _c2g_buffers[i]->get_data(raw);
                    size_t len = std::min<size_t>(sizeof(c2g_data_t) - i*8, 8);
                    memcpy(dst + i*8, raw.data(), len);
                    _is_online = true;
                    _last_rx_time_ms = now;
                    _c2g_buffers[i]->mark_read();
                }
            }
        }

        if (_is_online && (now - _last_rx_time_ms > 100.0f))
        {
            _is_online = false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ======================== 底层 Raw 独立事件操作 ======================== //

status_t board_drv_t::send_event_raw(const uint32_t event_base_id,
                                     const void *data, const size_t size) const
{
    can_drv_t* can_drv = bsp_can::get_can(_can_ch);
    if (!can_drv || !data)
        return PYRO_ERROR;

    const auto *ptr = static_cast<const uint8_t *>(data);
    const auto frame_cnt = static_cast<uint8_t>((size + 7) / 8);

    for (uint8_t i = 0; i < frame_cnt; ++i)
    {
        const uint32_t current_id = event_base_id + i;
        size_t send_len = size - (i * 8);
        if (send_len > 8) send_len = 8;

        uint8_t tx_data[8] = {0};
        memcpy(tx_data, ptr + (i * 8), send_len);

        status_t ret = can_drv->send_msg(current_id, tx_data);
        if (ret != PYRO_OK) return ret;
    }
    return PYRO_OK;
}

bool board_drv_t::read_event_raw(const uint32_t event_base_id, void *data_out,
                                 const size_t size) const
{
    if (!data_out)
        return false;

    // 只支持 EVENT_C2G_SHOOT
    if (event_base_id != EVENT_C2G_SHOOT)
        return false;

    std::array<uint8_t, 8> raw_data{};
    bool received_any = false;
    auto *ptr = static_cast<uint8_t *>(data_out);
    const uint8_t frame_cnt = static_cast<uint8_t>((size + 7) / 8);

    for (uint8_t i = 0; i < frame_cnt; ++i)
    {
        if (_event_buffers[i] && _event_buffers[i]->is_fresh())
        {
            _event_buffers[i]->get_data(raw_data);
            size_t copy_len = size - (i * 8);
            if (copy_len > 8) copy_len = 8;
            memcpy(ptr + (i * 8), raw_data.data(), copy_len);
            received_any = true;
            _event_buffers[i]->mark_read();
        }
    }
    return received_any;
}

// ======================== 周期数据 Getters & Setters ======================== //

board_drv_t::g2c_data_t &board_drv_t::get_g2c_tx_data()
{
    return _g2c_tx_payload;
}
board_drv_t::c2g_data_t &board_drv_t::get_c2g_tx_data()
{
    return _c2g_tx_payload;
}
const board_drv_t::g2c_data_t &board_drv_t::get_g2c_rx_data() const
{
    return _latest_g2c_rx;
}
const board_drv_t::c2g_data_t &board_drv_t::get_c2g_rx_data() const
{
    return _latest_c2g_rx;
}
bool board_drv_t::check_online() const
{
    return _is_online;
}

status_t board_drv_t::send_data() const
{
    can_drv_t* can_drv = bsp_can::get_can(_can_ch);
    if (!can_drv) return PYRO_ERROR;

    if (_role == role_t::GIMBAL)
    {
        const auto *ptr = reinterpret_cast<const uint8_t *>(&_g2c_tx_payload);
        for (uint8_t i = 0; i < G2C_FRAME_CNT; ++i)
        {
            const uint32_t current_id = G2C_BASE_ID + i;
            size_t send_len = sizeof(g2c_data_t) - (i * 8);
            if (send_len > 8) send_len = 8;

            uint8_t tx_data[8] = {0};
            memcpy(tx_data, ptr + (i * 8), send_len);

            status_t ret = can_drv->send_msg(current_id, tx_data);
            if (ret != PYRO_OK) return ret;
        }
    }
    else // CHASSIS
    {
        const auto *ptr = reinterpret_cast<const uint8_t *>(&_c2g_tx_payload);
        for (uint8_t i = 0; i < C2G_FRAME_CNT; ++i)
        {
            const uint32_t current_id = C2G_BASE_ID + i;
            size_t send_len = sizeof(c2g_data_t) - (i * 8);
            if (send_len > 8) send_len = 8;

            uint8_t tx_data[8] = {0};
            memcpy(tx_data, ptr + (i * 8), send_len);

            status_t ret = can_drv->send_msg(current_id, tx_data);
            if (ret != PYRO_OK) return ret;
        }
    }
    return PYRO_OK;
}

} // namespace pyro