/**
 * @file pyro_board_drv.h
 * @brief 板间通信底层驱动 (支持高频周期数据与多通道独立事件数据)
 */

#ifndef PYRO_BOARD_DRV_H
#define PYRO_BOARD_DRV_H

#include "pyro_can_drv.h"
#include "pyro_core_def.h"
#include "pyro_task.h"
#include <cstdint>

namespace pyro
{

class board_drv_t
{
  public:
    enum class role_t
    {
        GIMBAL,
        CHASSIS
    };

/* ================== 通信协议结构体定义 ================== */
#pragma pack(push, 1)

    /**
     * @brief [周期数据区] 云台 -> 底盘
     */
    struct g2c_data_t
    {
        int8_t vx;
        int8_t vy;
        int8_t wz;

        bool active          : 1;
        bool track_en        : 1;
        bool leg_retract     : 1;
        bool leg_calibration : 1;
        bool ui_refresh      : 1;
        bool fric_en         : 1;
        bool fric_err        : 1;
        bool sling_mode      : 1;

        bool trigger_located : 1;
        bool fire_ready :1;
        bool reserved_bit    : 6;

        int16_t pitch_rad;
        uint8_t target_shoot_spd;
    };

    /**
     * @brief [周期数据区] 底盘 -> 云台
     */
    struct c2g_data_t
    {
        int16_t chassis_q[4];
        bool gimbal_output  : 1;
        bool booster_output : 1;
        bool chassis_output : 1;
        uint8_t robot_color : 1;
        bool reserved_bit   : 4;
        uint8_t heat;
        uint8_t heat_limit;
        uint16_t robot_x;
        uint16_t robot_y;
    };

    /* ---------------------------------------------------- */
    /* [独立事件区] 扩充事件时，新建结构体并分配独立 ID 即可  */
    /* ---------------------------------------------------- */

    // 事件 A：底盘发弹测速
    struct event_shoot_t
    {
        float shoot_speed;
        uint16_t launching_num;
    };


    // 事件 B：预留（例如云台发送 UI 更新）
    // struct event_ui_t { uint8_t ui_mode; };

#pragma pack(pop)
    /* ======================================================= */

    // 周期数据基准 ID
    static constexpr uint32_t G2C_BASE_ID     = 0x101;
    static constexpr uint32_t C2G_BASE_ID     = 0x105;

    // 独立事件基准 ID
    static constexpr uint32_t EVENT_C2G_SHOOT = 0x110;
    static constexpr uint32_t EVENT_G2C_UI    = 0x112;
    // 注意避让上一事件可能占用的多个分包ID

    // 周期帧数计算
    static constexpr uint8_t G2C_FRAME_CNT    = (sizeof(g2c_data_t) + 7) / 8;
    static constexpr uint8_t C2G_FRAME_CNT    = (sizeof(c2g_data_t) + 7) / 8;

    static board_drv_t &
    get_instance(role_t role                 = role_t::GIMBAL,
                 can_hub_t::which_can can_ch = can_hub_t::can1);

    void start_rx() const;

    // 周期数据交互接口 (由后台任务高频维护)
    g2c_data_t &get_g2c_tx_data();
    c2g_data_t &get_c2g_tx_data();
    [[nodiscard]] const g2c_data_t &get_g2c_rx_data() const;
    [[nodiscard]] const c2g_data_t &get_c2g_rx_data() const;
    status_t send_data() const;

    // =========================================================================
    // 独立事件交互接口 (纯解耦：只发送/读取目标 ID 的结构体)
    // =========================================================================

    /**
     * @brief 发送指定事件包
     * @param event_base_id 目标事件的 CAN ID
     * @param data 要发送的独立事件结构体
     */
    template <typename T>
    status_t send_event(uint32_t event_base_id, const T &data) const
    {
        return send_event_raw(event_base_id, &data, sizeof(T));
    }

    /**
     * @brief 主动读取指定事件包
     * @param event_base_id 目标事件的 CAN ID
     * @param data_out [输出] 接收到的事件结构体
     * @return true 代表收到了新事件，false 代表无新数据
     */
    template <typename T>
    bool read_event(uint32_t event_base_id, T &data_out) const
    {
        return read_event_raw(event_base_id, &data_out, sizeof(T));
    }

    [[nodiscard]] bool check_online() const;
    [[nodiscard]] role_t get_role() const
    {
        return _role;
    }

  private:
    explicit board_drv_t(role_t role, can_hub_t::which_can can_ch);
    ~board_drv_t();

    // 隐藏的底层 raw 接口，避免头文件被 CAN 驱动污染
    status_t send_event_raw(uint32_t event_base_id, const void *data,
                            size_t size) const;
    bool read_event_raw(uint32_t event_base_id, void *data_out,
                        size_t size) const;

    class board_task_t final : public task_base_t
    {
      public:
        explicit board_task_t(board_drv_t *owner_ptr)
            : task_base_t("board_drv_task", 128, 256, priority_t::NORMAL),
              _owner(owner_ptr)
        {
        }

      protected:
        status_t init() override;
        void run_loop() override;

      private:
        board_drv_t *_owner;
    };

    role_t _role;
    can_hub_t::which_can _can_ch;
    board_task_t *_task;

    g2c_data_t _g2c_tx_payload{};
    c2g_data_t _c2g_tx_payload{};
    g2c_data_t _latest_g2c_rx{};
    c2g_data_t _latest_c2g_rx{};

    bool _is_online;
    float _last_rx_time_ms;

    void init_impl() const;
    void run_loop_impl();
};

} // namespace pyro

#endif // PYRO_BOARD_DRV_H
