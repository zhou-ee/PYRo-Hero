/**
 * @file pyro_ui_com.cpp
 * @brief RoboMaster UI Component Implementation & FreeRTOS Thread
 */

#include "pyro_ui_com.h"

#include "pyro_board_drv.h"
#include "pyro_module_base.h"
#include "pyro_referee.h"
#include "pyro_can_drv.h"
#include "pyro_com_canrx.h"
#include "pyro_hybrid_chassis.h"
#include "pyro_supercap_drv.h"

using namespace pyro;

// ==========================================
// 全局变量定义区
// ==========================================
static pyro::referee_drv_t *referee_ptr = nullptr;
static pyro::ui_drv_t *ui_ptr           = nullptr;
static bool flush_flag                  = false;

static pyro::ui_ctx_t ui_ctx; // 全局数据上下文
static pyro::ui_com hero_ui;  // UI 绘制控制器组件

// ==========================================
// 辅助与测试函数
// ==========================================
static void uirxbooster()
{
    // 预留的 RX booster 逻辑
}

static void info_update_test()
{

    ui_ctx.sling_flag =
        board_drv_t::get_instance().get_g2c_rx_data().sling_mode;
    ui_ctx.fric_en_flag = board_drv_t::get_instance().get_g2c_rx_data().fric_en;
    ui_ctx.fric_error_flag =
        board_drv_t::get_instance().get_g2c_rx_data().firc_err;
    ui_ctx.yaw_rad =
        hybrid_chassis_t::instance()->get_ctx().data.current_yaw_error;
    ui_ctx.pitch_rad =
        static_cast<float>(
            board_drv_t::get_instance().get_g2c_rx_data().pitch_rad) /
        32768.0f;
    ui_ctx.target_shoot_spd =
        static_cast<float>(
            board_drv_t::get_instance().get_g2c_rx_data().target_shoot_spd) /
        10.0f;
    ui_ctx.super_cap_voltage = static_cast<float>(
        hybrid_chassis_t::instance()->get_ctx().cap_feedback.vot_cap);
}

// ==========================================
// UI 布局与外观配置 (隐藏在匿名空间中)
// ==========================================
namespace
{
struct cfg_fric
{
    static constexpr uint16_t x = 1800, y = 730, r = 50;
    static constexpr uint8_t layer = 2, cross_layer = 5;
};
struct cfg_lob
{
    static constexpr uint16_t x = 1800, y = 530, r = 50;
    static constexpr uint8_t layer = 2, cross_layer = 6;
};
struct cfg_text
{
    static constexpr uint16_t yaw_x = 1815, yaw_y = 440;
    static constexpr uint16_t pitch_x = 1815, pitch_y = 380;
    static constexpr uint16_t spd_x = 1790, spd_y = 650;
    static constexpr uint16_t label_offset = 210;
    static constexpr uint8_t layer = 3, val_layer = 4;
};
struct cfg_cap
{
    static constexpr uint16_t cx = 960,
                              cy = 110; // 居中 1920/2, 底部 1080-910-60
    static constexpr uint16_t hw = 350, hh = 30;
    static constexpr uint8_t layer = 2;
};
} // namespace

// ==========================================
// ui_com 类方法实现
// ==========================================
void ui_com::init(ui_drv_t *drv)
{
    _drv = drv;
}

void ui_com::update_ctx(const ui_ctx_t &new_ctx)
{
    _last_ctx = _ctx;
    _ctx      = new_ctx;
}

void ui_com::draw_static()
{
    if (!_drv)
        return;

    // 1. 绘制基础背景圆
    _drv->draw_circle("FRC", ui_operate::ADD, cfg_fric::layer, ui_color::PINK,
                      3, cfg_fric::x, cfg_fric::y, cfg_fric::r)
        .draw_circle("LOB", ui_operate::ADD, cfg_lob::layer, ui_color::WHITE, 3,
                     cfg_lob::x, cfg_lob::y, cfg_lob::r);

    // 2. 绘制超级电容外框和进度条初始点
    _drv->draw_rect("SCP", ui_operate::ADD, cfg_cap::layer, ui_color::ORANGE, 3,
                    cfg_cap::cx - cfg_cap::hw, cfg_cap::cy + cfg_cap::hh,
                    cfg_cap::cx + cfg_cap::hw, cfg_cap::cy - cfg_cap::hh)
        .draw_line("PBR", ui_operate::ADD, cfg_cap::layer + 1, ui_color::ORANGE,
                   50, cfg_cap::cx - cfg_cap::hw, cfg_cap::cy,
                   cfg_cap::cx - cfg_cap::hw, cfg_cap::cy);

    // 3. 绘制静态文本标签
    _drv->draw_string("YAW", ui_operate::ADD, cfg_text::layer, ui_color::GREEN,
                      20, 2, cfg_text::yaw_x - cfg_text::label_offset,
                      cfg_text::yaw_y, "YAW");
    _drv->draw_string("PIH", ui_operate::ADD, cfg_text::layer, ui_color::GREEN,
                      20, 2, cfg_text::pitch_x - cfg_text::label_offset,
                      cfg_text::pitch_y, "PITCH");

    // 4. 浮点数值的 ADD 占位
    _drv->draw_float("YDG", ui_operate::ADD, cfg_text::val_layer,
                     ui_color::GREEN, 20, 2, cfg_text::yaw_x, cfg_text::yaw_y,
                     0.0f)
        .draw_float("PDG", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::GREEN, 20, 2, cfg_text::pitch_x,
                    cfg_text::pitch_y, 0.0f)
        .draw_float("SPD", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::GREEN, 20, 2, cfg_text::spd_x, cfg_text::spd_y,
                    0.0f);
    // 5. 交叉线的ADD占位
    _drv->draw_line("FL1", ui_operate::ADD, cfg_fric::cross_layer,
                           ui_color::MAGENTA, 3, 0,0,0,0)
        .draw_line("FL2", ui_operate::ADD, cfg_fric::cross_layer,
                           ui_color::MAGENTA, 3, 0,0,0,0)
        .draw_line("LL1", ui_operate::ADD, cfg_lob::cross_layer,
                           ui_color::ORANGE, 3,0,0,0,0)
        .draw_line("LL2", ui_operate::ADD, cfg_lob::cross_layer,
                           ui_color::ORANGE, 3, 0,0,0,0);

    _drv->flush();
}

void ui_com::draw_dynamic()
{
    if (!_drv)
        return;

    draw_yaw();
    draw_pitch();
    draw_spd();
    draw_fric_state();
    draw_lob_state();
    draw_super_cap();

    _drv->flush();
    _force_refresh_flag = false;
}

void ui_com::refresh()
{
    if (!_drv)
        return;
    _drv->clear_all();
    _force_refresh_flag = true;
    draw_static();
    draw_dynamic();
}

void ui_com::draw_yaw()
{
    _drv->draw_float("YDG", ui_operate::MODIFY, cfg_text::val_layer,
                     ui_color::GREEN, 20, 2, cfg_text::yaw_x, cfg_text::yaw_y,
                     _ctx.yaw_rad);
}

void ui_com::draw_pitch()
{
    _drv->draw_float("PDG", ui_operate::MODIFY, cfg_text::val_layer,
                     ui_color::GREEN, 20, 2, cfg_text::pitch_x,
                     cfg_text::pitch_y, _ctx.pitch_rad);
}

void ui_com::draw_spd()
{
    _drv->draw_float("SPD", ui_operate::MODIFY, cfg_text::val_layer,
                     ui_color::GREEN, 20, 2, cfg_text::spd_x, cfg_text::spd_y,
                     _ctx.target_shoot_spd);
}

void ui_com::draw_fric_state()
{
    bool curr_ready = _ctx.fric_en_flag;
    bool last_ready = _last_ctx.fric_en_flag;
    float cross_line_reduce_k = sqrt__2;
    bool cross_line = false;
    ui_color circle_color = ui_color::PINK;
    ui_color cross_line_color = ui_color::MAGENTA;
    if (curr_ready!=last_ready || _force_refresh_flag)
    {
        if (curr_ready)
        {
            circle_color = ui_color::GREEN;
            if (_ctx.fric_error_flag)
            {
                cross_line_color = ui_color::ORANGE;
                cross_line_reduce_k = 1;
                cross_line = true;
            }
        }
        else
        {
            cross_line = true;
        }
        _drv->draw_circle("FRC", ui_operate::MODIFY, cfg_fric::layer,
                        circle_color, 3, cfg_fric::x, cfg_fric::y,
                        cfg_fric::r)
            .draw_line("FL1", ui_operate::ADD, cfg_fric::cross_layer,
                        cross_line_color, 3,
                        (cfg_fric::x - cfg_fric::r*cross_line_reduce_k)*cross_line,
                        (cfg_fric::y + cfg_fric::r*cross_line_reduce_k)*cross_line,
                        (cfg_fric::x + cfg_fric::r*cross_line_reduce_k)*cross_line,
                        (cfg_fric::y - cfg_fric::r*cross_line_reduce_k)*cross_line)
            .draw_line("FL2", ui_operate::ADD, cfg_fric::cross_layer,
                        cross_line_color, 3,
                        (cfg_fric::x + cfg_fric::r*cross_line_reduce_k)*cross_line,
                        (cfg_fric::y + cfg_fric::r*cross_line_reduce_k)*cross_line,
                        (cfg_fric::x - cfg_fric::r*cross_line_reduce_k)*cross_line,
                        (cfg_fric::y - cfg_fric::r*cross_line_reduce_k)*cross_line);
    }

}

void ui_com::draw_lob_state()
{
    if (_ctx.sling_flag != _last_ctx.sling_flag || _force_refresh_flag)
    {
        ui_color circle_color = ui_color::WHITE;
        bool cross_line = _ctx.sling_flag;
        if (_ctx.sling_flag)
        {

            circle_color = ui_color::YELLOW;
        }
        _drv->draw_circle("LOB", ui_operate::MODIFY, cfg_lob::layer,
                              circle_color, 3, cfg_lob::x, cfg_lob::y,
                              cfg_lob::r)
            .draw_line("LL1", ui_operate::MODIFY, cfg_lob::cross_layer,
                           ui_color::ORANGE, 3,
                           cfg_lob::x,
                           (cfg_lob::y + cfg_lob::r + 25)*cross_line,
                           cfg_lob::x,
                           (cfg_lob::y - cfg_lob::r - 25)*cross_line)
            .draw_line("LL2", ui_operate::MODIFY, cfg_lob::cross_layer,
                           ui_color::ORANGE, 3,
                           (cfg_lob::x - cfg_lob::r - 25)*cross_line,
                           cfg_lob::y,
                           (cfg_lob::x + cfg_lob::r + 25)*cross_line,
                           cfg_lob::y);
    }
}

void ui_com::draw_super_cap()
{
    float ratio = _ctx.super_cap_voltage / ui_ctx_t::super_cap_voltage_max;
    if (ratio > 1.0f)
        ratio = 1.0f;
    if (ratio < 0.0f)
        ratio = 0.0f;

    uint16_t start_x = cfg_cap::cx - cfg_cap::hw;
    uint16_t end_x   = start_x + static_cast<uint16_t>(2 * cfg_cap::hw * ratio);
    ui_color cap_color = (ratio >= 1.0f) ? ui_color::CYAN : ui_color::ORANGE;

    _drv->draw_line("PBR", ui_operate::MODIFY, cfg_cap::layer + 1, cap_color,
                    50, start_x, cfg_cap::cy, end_x, cfg_cap::cy);
}

// ==========================================
// 线程与初始化 (外部 C 接口保持不变)
// ==========================================
extern "C"
{

    static void hero_ui_thread(void *argument)
    {
        // 1. 阻塞等待裁判系统链路连通
        while (!referee_ptr->is_online() || referee_ptr->get_robot_id() == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // 2. 初始化封装器，并请求全局重绘
        hero_ui.init(ui_ptr);
        hero_ui.refresh();

        // 3. 主循环
        while (true)
        {
            uirxbooster();
            info_update_test(); // 将最新数据写入全局变量 ui_ctx

            if (referee_ptr->is_online())
            {
                if (flush_flag)
                {
                    // 如果请求强制刷新
                    hero_ui.refresh();
                    flush_flag = false;
                }
                else
                {
                    // 常规状态同步更新
                    hero_ui.update_ctx(ui_ctx);
                    hero_ui.draw_dynamic();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    void hero_ui_init(void *argument)
    {
        pyro::can_rx_drv_t::subscribe(can_hub_t::can2, 0x110);
        referee_ptr = pyro::referee_drv_t::get_instance();
        ui_ptr      = new pyro::ui_drv_t(referee_ptr);

        xTaskCreate(hero_ui_thread, "hero_ui_thread", 512, nullptr,
                    configMAX_PRIORITIES - 3, nullptr);

        vTaskDelete(nullptr);
    }

} // extern "C"