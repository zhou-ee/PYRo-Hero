#include "pyro_ui_com.h"

#include "pyro_board_drv.h"
#include "pyro_can_drv.h"
#include "pyro_com_canrx.h"
#include "pyro_core_def.h"
#include "pyro_mec_chassis.h"
#include "pyro_module_base.h"
#include "pyro_referee.h"
#include "pyro_supercap_drv.h"
#include "pyro_ui_drv.h"

#include <cmath>
#include <cstring>

using namespace pyro;

namespace
{
static constexpr float sqrt_2                       = 1.4142135f;
static constexpr float sqrt_2_2                     = 0.7071068f;
static constexpr float rad_update_threshold         = 0.01f;
static constexpr float speed_update_threshold       = 0.1f;
static constexpr float position_update_threshold    = 0.01f;
static constexpr float cap_voltage_update_threshold = 0.3f;

inline bool value_changed(float current, float sent, float threshold)
{
    return std::fabs(current - sent) > threshold;
}

inline bool mecanum_online_changed(const bool current[4], const bool sent[4])
{
    return (current[0] != sent[0]) || (current[1] != sent[1]) ||
           (current[2] != sent[2]) || (current[3] != sent[3]);
}

inline uint16_t ui_coord(float value)
{
    if (value < 0.0f)
        return 0;
    if (value > 2047.0f)
        return 2047;
    return static_cast<uint16_t>(value);
}

struct cfg_relative_pos
{
    static constexpr uint16_t x = 100, y = 600, chassis_scale = 70;
    static constexpr uint8_t chassis_layer = 2, gimbal_layer = 3;
    static constexpr uint8_t gimbal_r = 20, cannon_length = 50,
                             cannon_width    = 15;
    static constexpr uint8_t cannon_offset_r = 18;
};

struct cfg_fric
{
    static constexpr uint16_t x = 1770, y = 730, r = 50;
    static constexpr uint8_t layer = 2, cross_layer = 5;
};

struct cfg_trigger_located
{
    static constexpr uint16_t x         = cfg_fric::x;
    static constexpr uint16_t y         = cfg_fric::y - cfg_fric::r - 45;
    static constexpr uint16_t half_size = 28;
    static constexpr uint8_t layer      = 7;
    static constexpr uint8_t width      = 5;
};

struct cfg_fire_ready
{
    static constexpr uint16_t x         = cfg_fric::x;
    static constexpr uint16_t y         = cfg_fric::y + cfg_fric::r + 45;
    static constexpr uint16_t half_size = 28;
    static constexpr uint8_t layer      = 7;
    static constexpr uint8_t width      = 5;
};

struct cfg_lob
{
    static constexpr uint16_t x = 1770, y = 530, r = 50;
    static constexpr uint8_t layer = 2, cross_layer = 6;
};

struct cfg_text
{
    static constexpr uint16_t yaw_x = 1770, yaw_y = 440;
    static constexpr uint16_t pitch_x = 1770, pitch_y = 380;
    static constexpr uint16_t spd_x = 885, spd_y = 220;
    static constexpr uint16_t pos_x = 100, pos_y = 800;
    static constexpr uint16_t label_offset = 100;
    static constexpr uint8_t layer = 3, val_layer = 4;
};

struct cfg_cap
{
    static constexpr uint16_t cx = 960, cy = 110;
    static constexpr uint16_t hw = 350, hh = 30;
    static constexpr uint8_t layer = 2;
};

struct cfg_outpost
{
    static constexpr uint16_t start_x = 900, end_x = 1020;
    static constexpr uint16_t start_y = 500, end_y = 500;
    static constexpr uint8_t width = 3;
    static constexpr uint8_t layer = 2;
};

struct cfg_base
{
    static constexpr uint16_t start_x = 900, end_x = 1020;
    static constexpr uint16_t start_y = 412, end_y = 412;
    static constexpr uint8_t width = 3;
    static constexpr uint8_t layer = 2;
};

struct ui_point_t
{
    uint16_t x;
    uint16_t y;
};

inline ui_point_t point_on_angle(float center_x, float center_y, float radius,
                                 float angle_rad)
{
    return {ui_coord(center_x + radius * std::cos(angle_rad)),
            ui_coord(center_y + radius * std::sin(angle_rad))};
}

inline ui_point_t relative_pos_center()
{
    return {
        ui_coord(cfg_relative_pos::x + cfg_relative_pos::chassis_scale / 2.0f),
        ui_coord(cfg_relative_pos::y +
                 cfg_relative_pos::chassis_scale * sqrt_2 / 2.0f)};
}

void draw_cannon_line(ui_drv_t *drv, const char name[3], ui_operate op,
                      uint8_t layer, ui_color color, uint16_t width,
                      float angle_rad, float length_offset = 0.0f)
{
    const auto center = relative_pos_center();
    const auto start  = point_on_angle(
        center.x, center.y, cfg_relative_pos::cannon_offset_r, angle_rad);
    const auto end =
        point_on_angle(center.x, center.y,
                       cfg_relative_pos::cannon_offset_r +
                           cfg_relative_pos::cannon_length + length_offset,
                       angle_rad);

    drv->draw_line(name, op, layer, color, width, start.x, start.y, end.x,
                   end.y);
}
} // namespace

static pyro::referee_drv_t *referee_ptr = nullptr;
static pyro::ui_drv_t *ui_ptr           = nullptr;
static pyro::ui_ctx_t ui_ctx;
static pyro::ui_com hero_ui;

static void info_update_test()
{
    auto &board             = board_drv_t::get_instance();
    const auto &rx_data     = board.get_g2c_rx_data();
    auto *chassis           = mec_chassis_t::instance();
    const auto &chassis_ctx = chassis->get_ctx();
    const auto &ref_data    = referee_ptr->get_data();

    ui_ctx.sling_flag       = rx_data.sling_mode;
    ui_ctx.fric_en_flag     = rx_data.fric_en;
    ui_ctx.fric_error_flag  = rx_data.fric_err;
    ui_ctx.trigger_located  = rx_data.trigger_located;
    ui_ctx.fire_ready_flag  = rx_data.fire_ready;
    ui_ctx.yaw_rad          = chassis_ctx.data.current_yaw_error;
    ui_ctx.pitch_rad        = static_cast<float>(rx_data.pitch_rad) / 32768.0f;
    ui_ctx.target_shoot_spd =
        static_cast<float>(rx_data.target_shoot_spd) / 10.0f;
    ui_ctx.super_cap_voltage =
        static_cast<float>(chassis_ctx.cap_feedback.vot_cap);
    ui_ctx.position_x   = ref_data.robot_pos.x;
    ui_ctx.position_y   = ref_data.robot_pos.y;
    ui_ctx.distance     = std::sqrt(ui_ctx.position_x * ui_ctx.position_x +
                                    ui_ctx.position_y * ui_ctx.position_y);
    ui_ctx.refresh_flag = rx_data.ui_refresh;

    std::memcpy(ui_ctx.mecanum_online, chassis_ctx.data.wheel_online,
                sizeof(ui_ctx.mecanum_online));
}

void ui_com::init(ui_drv_t *drv)
{
    _drv = drv;
}

void ui_com::update_ctx(const ui_ctx_t &new_ctx)
{
    _last_ctx = _ctx;
    _ctx      = new_ctx;
    if (refresh_check())
    {
        refresh();
    }
}

bool ui_com::refresh_check() const
{
    return _last_ctx.refresh_flag != _ctx.refresh_flag;
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

void ui_com::draw_static()
{
    if (!_drv)
        return;

    _drv->draw_circle("FRC", ui_operate::ADD, cfg_fric::layer, ui_color::PINK,
                      3, cfg_fric::x, cfg_fric::y, cfg_fric::r)
        .draw_circle("LOB", ui_operate::ADD, cfg_lob::layer, ui_color::WHITE, 3,
                     cfg_lob::x, cfg_lob::y, cfg_lob::r)
        .draw_rect("SCP", ui_operate::ADD, cfg_cap::layer, ui_color::ORANGE, 3,
                   cfg_cap::cx - cfg_cap::hw, cfg_cap::cy + cfg_cap::hh,
                   cfg_cap::cx + cfg_cap::hw, cfg_cap::cy - cfg_cap::hh)
        .draw_line("PBR", ui_operate::ADD, cfg_cap::layer + 1, ui_color::ORANGE,
                   50, cfg_cap::cx - cfg_cap::hw, cfg_cap::cy,
                   cfg_cap::cx - cfg_cap::hw, cfg_cap::cy);

    _drv->draw_string("YAW", ui_operate::ADD, cfg_text::layer, ui_color::GREEN,
                      20, 2, cfg_text::yaw_x - cfg_text::label_offset,
                      cfg_text::yaw_y, "YAW");
    _drv->draw_string("PIH", ui_operate::ADD, cfg_text::layer, ui_color::GREEN,
                      20, 2, cfg_text::pitch_x - cfg_text::label_offset,
                      cfg_text::pitch_y, "PITCH");
    _drv->draw_string("POX", ui_operate::ADD, cfg_text::layer, ui_color::GREEN,
                      20, 2, cfg_text::pos_x, cfg_text::pos_y, "X:");
    _drv->draw_string("POY", ui_operate::ADD, cfg_text::layer, ui_color::GREEN,
                      20, 2, cfg_text::pos_x + 140, cfg_text::pos_y, "Y:");
    _drv->draw_string("POD", ui_operate::ADD, cfg_text::layer, ui_color::GREEN,
                      20, 2, cfg_text::pos_x, cfg_text::pos_y - 50, "DIS:");

    _drv->draw_float("YDG", ui_operate::ADD, cfg_text::val_layer,
                     ui_color::GREEN, 20, 2, cfg_text::yaw_x, cfg_text::yaw_y,
                     0.0f)
        .draw_float("PDG", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::GREEN, 20, 2, cfg_text::pitch_x,
                    cfg_text::pitch_y, 0.0f)
        .draw_float("SPD", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::WHITE, 40, 5, cfg_text::spd_x, cfg_text::spd_y,
                    0.0f)
        .draw_float("PSX", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::GREEN, 20, 2, cfg_text::pos_x + 30,
                    cfg_text::pos_y, 0.0f)
        .draw_float("PSY", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::GREEN, 20, 2, cfg_text::pos_x + 170,
                    cfg_text::pos_y, 0.0f)
        .draw_float("PSD", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::ORANGE, 20, 2, cfg_text::pos_x + 70,
                    cfg_text::pos_y - 50, 0.0f)
        .draw_line("FL1", ui_operate::ADD, cfg_fric::cross_layer,
                   ui_color::MAGENTA, 3, 1, 1, 1, 1)
        .draw_line("FL2", ui_operate::ADD, cfg_fric::cross_layer,
                   ui_color::MAGENTA, 3, 1, 1, 1, 1)
        .draw_line("LL1", ui_operate::ADD, cfg_lob::cross_layer,
                   ui_color::ORANGE, 3, 1, 1, 1, 1)
        .draw_line("LL2", ui_operate::ADD, cfg_lob::cross_layer,
                   ui_color::ORANGE, 3, 1, 1, 1, 1)
        .draw_line("TC1", ui_operate::ADD, cfg_trigger_located::layer,
                   ui_color::GREEN, cfg_trigger_located::width, 1, 1, 1, 1)
        .draw_line("TC2", ui_operate::ADD, cfg_trigger_located::layer,
                   ui_color::GREEN, cfg_trigger_located::width, 1, 1, 1, 1)
        .draw_line("TX1", ui_operate::ADD, cfg_trigger_located::layer,
                   ui_color::PINK, cfg_trigger_located::width, 1, 1, 1, 1)
        .draw_line("TX2", ui_operate::ADD, cfg_trigger_located::layer,
                   ui_color::PINK, cfg_trigger_located::width, 1, 1, 1, 1)
        .draw_line("FC1", ui_operate::ADD, cfg_fire_ready::layer,
                   ui_color::GREEN, cfg_fire_ready::width, 1, 1, 1, 1)
        .draw_line("FC2", ui_operate::ADD, cfg_fire_ready::layer,
                   ui_color::GREEN, cfg_fire_ready::width, 1, 1, 1, 1)
        .draw_line("FX1", ui_operate::ADD, cfg_fire_ready::layer,
                   ui_color::PINK, cfg_fire_ready::width, 1, 1, 1, 1)
        .draw_line("FX2", ui_operate::ADD, cfg_fire_ready::layer,
                   ui_color::PINK, cfg_fire_ready::width, 1, 1, 1, 1);

    const auto relative_center = relative_pos_center();
    _drv->draw_rect(
            "CS1", ui_operate::ADD, cfg_relative_pos::chassis_layer,
            ui_color::WHITE, 5, cfg_relative_pos::x, cfg_relative_pos::y,
            cfg_relative_pos::x + cfg_relative_pos::chassis_scale,
            cfg_relative_pos::y +
                static_cast<uint16_t>(cfg_relative_pos::chassis_scale * sqrt_2))
        .draw_circle("GMA", ui_operate::ADD, cfg_relative_pos::gimbal_layer,
                     ui_color::WHITE, 5, relative_center.x, relative_center.y,
                     cfg_relative_pos::gimbal_r);

    draw_cannon_line(_drv, "GL2", ui_operate::ADD,
                     cfg_relative_pos::gimbal_layer, ui_color::ALLY,
                     cfg_relative_pos::cannon_width - 5, PI / 2 - _ctx.yaw_rad,
                     -5.0f);

    _drv->draw_line("OP1", ui_operate::ADD, cfg_outpost::layer, ui_color::GREEN,
                    cfg_outpost::width, cfg_outpost::start_x,
                    cfg_outpost::start_y, cfg_outpost::end_x,
                    cfg_outpost::end_y)
        .draw_float("OF1", ui_operate::ADD, cfg_outpost::layer, ui_color::PINK,
                    15, cfg_outpost::width, cfg_outpost::start_x + 150,
                    cfg_outpost::start_y, 6.5f)
        .draw_line("BA1", ui_operate::ADD, cfg_base::layer, ui_color::GREEN,
                   cfg_base::width, cfg_base::start_x, cfg_base::start_y,
                   cfg_base::end_x, cfg_base::end_y)
        .draw_float("BF1", ui_operate::ADD, cfg_base::layer, ui_color::PINK, 15,
                    cfg_base::width, cfg_base::start_x + 150, cfg_base::start_y,
                    9.2f);

    _drv->flush();
}

void ui_com::draw_dynamic()
{
    if (!_drv)
        return;

    const bool force = _force_refresh_flag;

    const bool yaw_changed =
        force ||
        value_changed(_ctx.yaw_rad, _sent_ctx.yaw_rad, rad_update_threshold);
    const bool pitch_changed =
        force || value_changed(_ctx.pitch_rad, _sent_ctx.pitch_rad,
                               rad_update_threshold);
    const bool spd_changed = force || value_changed(_ctx.target_shoot_spd,
                                                    _sent_ctx.target_shoot_spd,
                                                    speed_update_threshold);
    const bool pos_changed =
        force ||
        value_changed(_ctx.position_x, _sent_ctx.position_x,
                      position_update_threshold) ||
        value_changed(_ctx.position_y, _sent_ctx.position_y,
                      position_update_threshold) ||
        value_changed(_ctx.distance, _sent_ctx.distance,
                      position_update_threshold);
    const bool fric_changed =
        force || (_ctx.fric_en_flag != _sent_ctx.fric_en_flag) ||
        (_ctx.fric_error_flag != _sent_ctx.fric_error_flag);
    const bool trigger_located_changed =
        force || (_ctx.trigger_located != _sent_ctx.trigger_located);
    const bool fire_ready_changed =
        force || (_ctx.fire_ready_flag != _sent_ctx.fire_ready_flag);
    const bool lob_changed = force || (_ctx.sling_flag != _sent_ctx.sling_flag);
    const bool cap_changed =
        force ||
        value_changed(_ctx.super_cap_voltage, _sent_ctx.super_cap_voltage,
                      cap_voltage_update_threshold);
    const bool relative_changed =
        force || yaw_changed ||
        mecanum_online_changed(_ctx.mecanum_online, _sent_ctx.mecanum_online);

    if (yaw_changed)
        draw_yaw();
    if (pitch_changed)
        draw_pitch();
    if (spd_changed)
        draw_spd();
    if (pos_changed)
        draw_pos();
    if (fric_changed)
        draw_fric_state();
    if (trigger_located_changed)
        draw_trigger_located_state();
    if (fire_ready_changed)
        draw_fire_ready_state();
    if (lob_changed)
        draw_lob_state();
    if (cap_changed)
        draw_super_cap();
    if (relative_changed)
        draw_relative_pos();

    _drv->flush();
    _force_refresh_flag = false;
}

void ui_com::draw_yaw()
{
    _drv->draw_float("YDG", ui_operate::MODIFY, cfg_text::val_layer,
                     ui_color::GREEN, 20, 2, cfg_text::yaw_x, cfg_text::yaw_y,
                     _ctx.yaw_rad);
    _sent_ctx.yaw_rad = _ctx.yaw_rad;
}

void ui_com::draw_pitch()
{
    _drv->draw_float("PDG", ui_operate::MODIFY, cfg_text::val_layer,
                     ui_color::GREEN, 20, 2, cfg_text::pitch_x,
                     cfg_text::pitch_y, _ctx.pitch_rad);
    _sent_ctx.pitch_rad = _ctx.pitch_rad;
}

void ui_com::draw_spd()
{
    _drv->draw_float("SPD", ui_operate::MODIFY, cfg_text::val_layer,
                     ui_color::WHITE, 40, 5, cfg_text::spd_x, cfg_text::spd_y,
                     _ctx.target_shoot_spd);
    _sent_ctx.target_shoot_spd = _ctx.target_shoot_spd;
}

void ui_com::draw_pos()
{
    _drv->draw_float("PSX", ui_operate::MODIFY, cfg_text::val_layer,
                     ui_color::GREEN, 20, 2, cfg_text::pos_x + 30,
                     cfg_text::pos_y, _ctx.position_x)
        .draw_float("PSY", ui_operate::MODIFY, cfg_text::val_layer,
                    ui_color::GREEN, 20, 2, cfg_text::pos_x + 170,
                    cfg_text::pos_y, _ctx.position_y)
        .draw_float("PSD", ui_operate::MODIFY, cfg_text::val_layer,
                    ui_color::ORANGE, 20, 2, cfg_text::pos_x + 70,
                    cfg_text::pos_y - 50, _ctx.distance);
    _sent_ctx.position_x = _ctx.position_x;
    _sent_ctx.position_y = _ctx.position_y;
    _sent_ctx.distance   = _ctx.distance;
}

void ui_com::draw_fric_state()
{
    ui_color circle_color     = ui_color::PINK;
    ui_color cross_line_color = ui_color::MAGENTA;

    uint16_t x1 = 1, y1 = 1, x2 = 1, y2 = 1;

    if (_ctx.fric_en_flag)
    {
        circle_color = ui_color::GREEN;
        if (_ctx.fric_error_flag)
        {
            cross_line_color = ui_color::ORANGE;
            x1               = cfg_fric::x - cfg_fric::r;
            y1               = cfg_fric::y + cfg_fric::r;
            x2               = cfg_fric::x + cfg_fric::r;
            y2               = cfg_fric::y - cfg_fric::r;
        }
    }
    else
    {
        x1 = static_cast<uint16_t>(cfg_fric::x - cfg_fric::r * sqrt_2_2);
        y1 = static_cast<uint16_t>(cfg_fric::y + cfg_fric::r * sqrt_2_2);
        x2 = static_cast<uint16_t>(cfg_fric::x + cfg_fric::r * sqrt_2_2);
        y2 = static_cast<uint16_t>(cfg_fric::y - cfg_fric::r * sqrt_2_2);
    }

    _drv->draw_circle("FRC", ui_operate::MODIFY, cfg_fric::layer, circle_color,
                      3, cfg_fric::x, cfg_fric::y, cfg_fric::r)
        .draw_line("FL1", ui_operate::MODIFY, cfg_fric::cross_layer,
                   cross_line_color, 3, x1, y1, x2, y2)
        .draw_line("FL2", ui_operate::MODIFY, cfg_fric::cross_layer,
                   cross_line_color, 3, x1,
                   static_cast<uint16_t>(cfg_fric::y - (y1 - cfg_fric::y)), x2,
                   static_cast<uint16_t>(cfg_fric::y + (cfg_fric::y - y2)));

    _sent_ctx.fric_en_flag    = _ctx.fric_en_flag;
    _sent_ctx.fric_error_flag = _ctx.fric_error_flag;
}

void ui_com::draw_trigger_located_state()
{
    constexpr uint16_t hidden = 1;
    const auto x              = cfg_trigger_located::x;
    const auto y              = cfg_trigger_located::y;
    const auto s              = cfg_trigger_located::half_size;

    const uint16_t check_x1   = _ctx.trigger_located ? x - s : hidden;
    const uint16_t check_y1   = _ctx.trigger_located ? y - s / 3 : hidden;
    const uint16_t check_x2   = _ctx.trigger_located ? x - s / 4 : hidden;
    const uint16_t check_y2   = _ctx.trigger_located ? y - s : hidden;
    const uint16_t check_x3   = _ctx.trigger_located ? x + s : hidden;
    const uint16_t check_y3   = _ctx.trigger_located ? y + s : hidden;

    const uint16_t cross_x1   = _ctx.trigger_located ? hidden : x - s;
    const uint16_t cross_y1   = _ctx.trigger_located ? hidden : y - s;
    const uint16_t cross_x2   = _ctx.trigger_located ? hidden : x + s;
    const uint16_t cross_y2   = _ctx.trigger_located ? hidden : y + s;

    _drv->draw_line("TC1", ui_operate::MODIFY, cfg_trigger_located::layer,
                    ui_color::GREEN, cfg_trigger_located::width, check_x1,
                    check_y1, check_x2, check_y2)
        .draw_line("TC2", ui_operate::MODIFY, cfg_trigger_located::layer,
                   ui_color::GREEN, cfg_trigger_located::width, check_x2,
                   check_y2, check_x3, check_y3)
        .draw_line("TX1", ui_operate::MODIFY, cfg_trigger_located::layer,
                   ui_color::PINK, cfg_trigger_located::width, cross_x1,
                   cross_y1, cross_x2, cross_y2)
        .draw_line("TX2", ui_operate::MODIFY, cfg_trigger_located::layer,
                   ui_color::PINK, cfg_trigger_located::width, cross_x1,
                   cross_y2, cross_x2, cross_y1);

    _sent_ctx.trigger_located = _ctx.trigger_located;
}

void ui_com::draw_fire_ready_state()
{
    constexpr uint16_t hidden = 1;
    const auto x              = cfg_fire_ready::x;
    const auto y              = cfg_fire_ready::y;
    const auto s              = cfg_fire_ready::half_size;

    const uint16_t check_x1   = _ctx.fire_ready_flag ? x - s : hidden;
    const uint16_t check_y1   = _ctx.fire_ready_flag ? y - s / 3 : hidden;
    const uint16_t check_x2   = _ctx.fire_ready_flag ? x - s / 4 : hidden;
    const uint16_t check_y2   = _ctx.fire_ready_flag ? y - s : hidden;
    const uint16_t check_x3   = _ctx.fire_ready_flag ? x + s : hidden;
    const uint16_t check_y3   = _ctx.fire_ready_flag ? y + s : hidden;

    const uint16_t cross_x1   = _ctx.fire_ready_flag ? hidden : x - s;
    const uint16_t cross_y1   = _ctx.fire_ready_flag ? hidden : y - s;
    const uint16_t cross_x2   = _ctx.fire_ready_flag ? hidden : x + s;
    const uint16_t cross_y2   = _ctx.fire_ready_flag ? hidden : y + s;

    _drv->draw_line("FC1", ui_operate::MODIFY, cfg_fire_ready::layer,
                    ui_color::GREEN, cfg_fire_ready::width, check_x1, check_y1,
                    check_x2, check_y2)
        .draw_line("FC2", ui_operate::MODIFY, cfg_fire_ready::layer,
                   ui_color::GREEN, cfg_fire_ready::width, check_x2, check_y2,
                   check_x3, check_y3)
        .draw_line("FX1", ui_operate::MODIFY, cfg_fire_ready::layer,
                   ui_color::PINK, cfg_fire_ready::width, cross_x1, cross_y1,
                   cross_x2, cross_y2)
        .draw_line("FX2", ui_operate::MODIFY, cfg_fire_ready::layer,
                   ui_color::PINK, cfg_fire_ready::width, cross_x1, cross_y2,
                   cross_x2, cross_y1);

    _sent_ctx.fire_ready_flag = _ctx.fire_ready_flag;
}

void ui_com::draw_lob_state()
{
    ui_color circle_color =
        _ctx.sling_flag ? ui_color::YELLOW : ui_color::WHITE;

    uint16_t lx1 = 0, ly1 = 0, lx2 = 0, ly2 = 0;
    uint16_t kx1 = 0, ky1 = 0, kx2 = 0, ky2 = 0;

    if (_ctx.sling_flag)
    {
        lx1 = cfg_lob::x;
        ly1 = cfg_lob::y + cfg_lob::r + 25;
        lx2 = cfg_lob::x;
        ly2 = cfg_lob::y - cfg_lob::r - 25;
        kx1 = cfg_lob::x - cfg_lob::r - 25;
        ky1 = cfg_lob::y;
        kx2 = cfg_lob::x + cfg_lob::r + 25;
        ky2 = cfg_lob::y;
    }

    _drv->draw_circle("LOB", ui_operate::MODIFY, cfg_lob::layer, circle_color,
                      3, cfg_lob::x, cfg_lob::y, cfg_lob::r)
        .draw_line("LL1", ui_operate::MODIFY, cfg_lob::cross_layer,
                   ui_color::ORANGE, 3, lx1, ly1, lx2, ly2)
        .draw_line("LL2", ui_operate::MODIFY, cfg_lob::cross_layer,
                   ui_color::ORANGE, 3, kx1, ky1, kx2, ky2);

    _sent_ctx.sling_flag = _ctx.sling_flag;
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

    _sent_ctx.super_cap_voltage = _ctx.super_cap_voltage;
}

void ui_com::draw_relative_pos()
{
    ui_color rect_color = ui_color::WHITE;
    for (bool online : _ctx.mecanum_online)
    {
        if (!online)
        {
            rect_color = ui_color::MAGENTA;
            break;
        }
    }

    const float gimbal_angle = PI / 2 - _ctx.yaw_rad;
    draw_cannon_line(_drv, "GL2", ui_operate::MODIFY,
                     cfg_relative_pos::gimbal_layer, ui_color::ALLY,
                     cfg_relative_pos::cannon_width - 5, gimbal_angle, -5.0f);

    _drv->draw_rect(
        "CS1", ui_operate::MODIFY, cfg_relative_pos::chassis_layer, rect_color,
        5, cfg_relative_pos::x, cfg_relative_pos::y,
        cfg_relative_pos::x + cfg_relative_pos::chassis_scale,
        cfg_relative_pos::y +
            static_cast<uint16_t>(cfg_relative_pos::chassis_scale * sqrt_2));

    std::memcpy(_sent_ctx.mecanum_online, _ctx.mecanum_online,
                sizeof(_ctx.mecanum_online));
}

extern "C"
{
    static void hero_ui_thread(void *argument)
    {
        while (!referee_ptr->is_online() || referee_ptr->get_robot_id() == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        hero_ui.init(ui_ptr);
        hero_ui.refresh();

        while (true)
        {
            info_update_test();

            if (referee_ptr->is_online())
            {
                hero_ui.update_ctx(ui_ctx);
                hero_ui.draw_dynamic();
            }
            vTaskDelay(pdMS_TO_TICKS(50));
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
