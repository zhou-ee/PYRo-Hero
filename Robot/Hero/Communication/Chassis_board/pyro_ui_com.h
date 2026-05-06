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
// 外部需要上屏幕的数据全部放进这里
// ==========================================
struct ui_ctx_t
{
    bool sling_flag                              = false; // 吊射模式标志位
    bool fric_en_flag                            = false; // 摩擦轮使能标志位
    bool fric_error_flag                         = false; // 摩擦轮错误标志位
    float yaw_rad                                = 0.0f;  // rad
    float pitch_rad                              = 0.0f;  // rad
    float target_shoot_spd                       = 0.0f;  // m/s
    float super_cap_voltage                      = 0.0f;  // V

    static constexpr float super_cap_voltage_max = 26.0f;

    // 辅助判定
    bool is_fric_ready() const
    {
        return fric_en_flag && fric_error_flag;
    }
};

// ==========================================
// 2. UI 绘制框架
// ==========================================
class ui_com
{
  public:
    ui_com() = default;

    // 绑定底层驱动
    void init(ui_drv_t *drv);

    // 更新内部上下文数据
    void update_ctx(const ui_ctx_t &new_ctx);

    // 绘制静态元素 (底图、静态文本标签等)
    void draw_static();

    // 绘制动态元素 (数值修改、状态切换)
    void draw_dynamic();

    // 全局刷新 (清空所有 -> 绘制静态 -> 绘制动态)
    void refresh();

  private:
    ui_drv_t *_drv = nullptr;

    ui_ctx_t _ctx;                   // 当前状态
    ui_ctx_t _last_ctx;              // 上一次的状态 (用于脏检查)
    bool _force_refresh_flag = true; // 强制更新图层的标志位

    // === 具体功能模块的私有绘制接口 ===
    void draw_fric_state();
    void draw_lob_state();
    void draw_yaw();
    void draw_pitch();
    void draw_spd();
    void draw_super_cap();
};

} // namespace pyro
static constexpr float sqrt__2 = 0.707f;
#endif // PYRO_UI_COM_H