#ifndef PYRO_UI_COM_H
#define PYRO_UI_COM_H

#include "pyro_ui_drv.h"

namespace pyro
{

struct ui_ctx_t
{
    bool sling_flag                              = false;
    bool fric_en_flag                            = false;
    bool fric_error_flag                         = false;
    bool trigger_located                         = false;
    bool fire_ready_flag                         = false;
    float yaw_rad                                = 0.0f;
    float pitch_rad                              = 0.0f;
    float target_shoot_spd                       = 0.0f;
    float super_cap_voltage                      = 0.0f;
    float position_x                             = 0.0f;
    float position_y                             = 0.0f;
    float distance                               = 0.0f;
    bool mecanum_online[4]                       = {false};
    bool refresh_flag                            = false;

    static constexpr float super_cap_voltage_max = 26.0f;
};

class ui_com
{
  public:
    ui_com() = default;

    void init(ui_drv_t *drv);
    void update_ctx(const ui_ctx_t &new_ctx);
    void draw_static();
    void draw_dynamic();
    void refresh();
    bool refresh_check() const;

  private:
    ui_drv_t *_drv = nullptr;

    ui_ctx_t _ctx;
    ui_ctx_t _last_ctx;
    ui_ctx_t _sent_ctx;
    bool _force_refresh_flag = true;

    void draw_fric_state();
    void draw_lob_state();
    void draw_yaw();
    void draw_pitch();
    void draw_spd();
    void draw_super_cap();
    void draw_trigger_located_state();
    void draw_fire_ready_state();
    void draw_pos();
    void draw_relative_pos();
};

} // namespace pyro

#endif // PYRO_UI_COM_H
