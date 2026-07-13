/**
 * @file pyro_ui_com.h
 * @brief RoboMaster UI Component Architecture
 */
#ifndef PYRO_UI_COM_H
#define PYRO_UI_COM_H

#include "pyro_ui_drv.h"

namespace pyro
{
// ==========================================
// 1. UI 数据上下文 (Context)
// ==========================================
struct ui_ctx_t
{
    bool sling_flag         = false; // 吊射模式标志位
    bool fric_en_flag       = false; // 摩擦轮使能标志位
    bool fric_error_flag    = false; // 摩擦轮错误标志位
    bool track_en_flag      = false; // 自动追踪/自瞄使能标志位
    bool trigger_located    = false;
    bool fire_ready_flag    = false;
    float yaw_rad           = 0.0f; // 云台 Yaw 相对底盘角度 (rad)
    float pitch_rad         = 0.0f; // 云台 Pitch 绝对角度 (rad)
    float target_shoot_spd  = 0.0f; // 目标射速 (m/s)
    float super_cap_voltage = 0.0f; // 超级电容电压 (V)
    float position_x        = 0.0f; // 机器人全场坐标 X
    float position_y        = 0.0f; // 机器人全场坐标 Y
    float distance          = 0.0f;
    float left_leg_rad      = 0.0f;
    float right_leg_rad     = 0.0f;
    bool mecanum_online[4]  = {false}; // 四轮电机在线状态
    bool refresh_flag       = false;   // 刷新触发标志位（沿触发）

    static constexpr float super_cap_voltage_max = 26.0f;
};

// ==========================================
// 2. UI 绘制框架类
// ==========================================
class ui_com
{
  public:
    ui_com() = default;

    // 绑定底层驱动
    void init(ui_drv_t *drv);

    // 更新内部上下文数据并触发刷新检查
    void update_ctx(const ui_ctx_t &new_ctx);

    // 绘制静态元素 (底图、静态文本标签等)
    void draw_static();

    // 绘制动态元素 (数值修改、状态切换)
    void draw_dynamic();

    // 全局刷新 (清空所有 -> 绘制静态 -> 绘制动态)
    void refresh();

    // 检查刷新沿
    bool refresh_check() const;

  private:
    ui_drv_t *_drv = nullptr;

    ui_ctx_t _ctx;      // 当前最新状态
    ui_ctx_t _last_ctx; // 上一次传入的状态 (用于检测外部 refresh_flag 翻转)
    ui_ctx_t _sent_ctx; // 真正成功发送上屏的状态 (用于各模块的增量脏检查)

    bool _force_refresh_flag = true; // 强制重绘图层标志位

    // === 具体功能模块的私有绘制接口 ===
    void draw_fric_state();
    void draw_lob_state();
    void draw_yaw();
    void draw_pitch();
    void draw_spd();
    void draw_super_cap();
    void draw_trigger_located_state();
    void draw_fire_ready_state();
    void draw_track_state();
    void draw_pos();
    void draw_relative_pos();
    void draw_leg();
};
} // namespace pyro

#endif // PYRO_UI_COM_H
