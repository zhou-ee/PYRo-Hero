#include "pyro_quad_booster.h"
#include "pyro_algo_common.h"
#include "pyro_bsp_uart.h"
#include "pyro_board_drv.h"
#include "pyro_dwt_drv.h"
#include <cmath>
#include "quad_config.h"
#include <algorithm>
#include "arm_math.h"

namespace pyro
{

quad_booster_t::quad_booster_t() : module_base_t("quad_booster")
{
}

status_t quad_booster_t::_init()
{
    _ctx.motor              = _module_deps.motor_deps;
    _ctx.pid                = _module_deps.pid_deps;
    _ctx.pid.ball_speed_pid = new pid_t(0.6f, 0.0f, 0.005f, 0.0f, 2.0f);

    return PYRO_OK;
}

float quad_booster_t::_normalize_angle(float angle)
{
    while (angle > PI)
        angle -= 2.0f * PI;
    while (angle < -PI)
        angle += 2.0f * PI;
    return angle;
}

bool quad_booster_t::_is_trigger_located(float trigger_rad)
{
    constexpr float TRIGGER_SLOT_RAD = PI / 3.0f;
    const float delta = _normalize_angle(trigger_rad - TRIGGER_OFFSET);
    const float nearest_slot_delta =
        delta - std::round(delta / TRIGGER_SLOT_RAD) * TRIGGER_SLOT_RAD;
    return std::fabs(nearest_slot_delta) < TRIGGER_LOCATED_THRESHOLD_RAD;
}

float quad_booster_t::_get_next_trigger_preset(float trigger_rad,
                                               float min_advance_rad)
{
    constexpr float TRIGGER_SLOT_RAD = PI / 3.0f;
    const float delta        = _normalize_angle(trigger_rad - TRIGGER_OFFSET);
    const bool feed_positive = TRIGGER_FEED_DIR > 0.0f;
    float preset_index  = feed_positive ? std::ceil(delta / TRIGGER_SLOT_RAD)
                                        : std::floor(delta / TRIGGER_SLOT_RAD);
    const float advance = feed_positive
                              ? preset_index * TRIGGER_SLOT_RAD - delta
                              : delta - preset_index * TRIGGER_SLOT_RAD;

    if (advance <= min_advance_rad)
    {
        preset_index += feed_positive ? 1.0f : -1.0f;
    }

    return _normalize_angle(TRIGGER_OFFSET + preset_index * TRIGGER_SLOT_RAD);
}

void quad_booster_t::_update_feedback()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->update_feedback();
        _ctx.data.current_fric_torque[i] =
            _ctx.motor.fric_wheels[i]->get_current_torque();
    }
    _ctx.data.current_fric_mps[0] =
        _ctx.motor.fric_wheels[0]->get_current_rotate() * FRIC2_RADIUS;
    _ctx.data.current_fric_mps[1] =
        _ctx.motor.fric_wheels[1]->get_current_rotate() * FRIC1_RADIUS;
    _ctx.data.current_fric_mps[2] =
        _ctx.motor.fric_wheels[2]->get_current_rotate() * FRIC2_RADIUS;
    _ctx.data.current_fric_mps[3] =
        _ctx.motor.fric_wheels[3]->get_current_rotate() * FRIC1_RADIUS;

    for (int i = 0; i < 4; i++)
    {
        _ctx.data.abs_current_fric_mps[i] = abs(_ctx.data.current_fric_mps[i]);
    }

    _ctx.motor.trigger_wheel->update_feedback();
    _ctx.data.current_trig_radps =
        _ctx.motor.trigger_wheel->get_current_rotate();
    _ctx.data.current_trig_torque =
        _ctx.motor.trigger_wheel->get_current_torque();
    _ctx.data.current_trig_rad =
        _ctx.motor.trigger_wheel->get_current_position();
    _ctx.data.trigger_located = _is_trigger_located(_ctx.data.current_trig_rad);

    auto &board_drv           = board_drv_t::get_instance();
    if (board_drv.check_online())
    {
        auto board_com_data = board_drv.get_c2g_rx_data();
        _ctx.data.deploy_mode =
            board_com_data.booster_output && !board_com_data.chassis_output;
    }
    else
    {
        _ctx.data.deploy_mode = false;
    }
}

void quad_booster_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
        _main_fsm.change_state(&_state_active);
    else
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

#include <cstdint>

__attribute__((section(".dma_heap"))) char shoot_speed[10];

/**
 * @brief 轻量级浮点数转字符函数（保留5位小数）
 * @param value 要转换的浮点数
 * @param buffer 输出的字符数组
 * @param max_len 数组最大长度（防止越界）
 */
void float_to_char_5_decimals(float value, char *buffer, int max_len)
{
    int idx = 0;

    // 1. 处理符号
    if (value < 0)
    {
        if (idx < max_len - 1)
            buffer[idx++] = '-';
        value = -value;
    }

    // 2. 分离整数和小数部分
    int int_part  = (int)value;
    // 加 0.5f 用于实现最后一位的四舍五入
    int frac_part = (int)((value - (float)int_part) * 100000.0f + 0.5f);

    // 处理四舍五入导致的进位
    if (frac_part >= 100000)
    {
        int_part++;
        frac_part -= 100000;
    }

    // 3. 计算整数部分的位数
    int temp       = int_part;
    int num_digits = 0;
    do
    {
        num_digits++;
        temp /= 10;
    } while (temp > 0);

    // 4. 边界安全检查：符号位 + 整数位数 + 小数点(1) + 5位小数 + 结束符(1)
    if (idx + num_digits + 1 + 5 + 1 > max_len)
    {
        // 如果越界（例如弹速异常到了三位数），默认安全返回全0
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }

    // 5. 提取整数部分（逆序写入）
    for (int i = num_digits - 1; i >= 0; i--)
    {
        buffer[idx + i] = '0' + (int_part % 10);
        int_part /= 10;
    }
    idx += num_digits;

    // 6. 写入小数点
    buffer[idx++] = '.';

    // 7. 提取小数部分（固定提取5位）
    for (int i = 4; i >= 0; i--)
    {
        buffer[idx + i] = '0' + (frac_part % 10);
        frac_part /= 10;
    }
    idx += 5;

    // 8. 添加字符串结束符
    buffer[idx]     = '\n';
    buffer[idx + 1] = '\0';
}

void quad_booster_t::_speed_control()
{
    static uint16_t last_launching_num = 0;
    auto &board_drv =
        board_drv_t::get_instance(board_drv_t::role_t::GIMBAL, bsp_can::can1);
    board_drv_t::event_shoot_t shoot_event{};

    auto &shoot_data =
        _use_deploy_data() ? _ctx.shoot_deploy_data : _ctx.shoot_normal_data;
    _ctx.data.target_shoot_speed = shoot_data.target_speed;

    if (!board_drv.read_event(board_drv_t::EVENT_C2G_SHOOT, shoot_event))
    {
        return;
    }

    if (shoot_event.launching_num == last_launching_num)
    {
        return;
    }
    last_launching_num = shoot_event.launching_num;

    for (int i = 7; i > 0; --i)
    {
        shoot_data.real_ball_speed[i] = shoot_data.real_ball_speed[i - 1];
    }
    shoot_data.real_ball_speed[0]     = shoot_event.shoot_speed;

    constexpr float real_speed_weight = 1.0f / 8.0f;
    shoot_data.avg_real_ball_speed    = 0.0f;
    for (float speed : shoot_data.real_ball_speed)
    {
        shoot_data.avg_real_ball_speed += real_speed_weight * speed;
    }

    shoot_data.ball_speed[2] = shoot_data.ball_speed[1];
    shoot_data.ball_speed[1] = shoot_data.ball_speed[0];
    shoot_data.ball_speed[0] = shoot_event.shoot_speed;

    float_to_char_5_decimals(shoot_data.ball_speed[0], shoot_speed,
                             sizeof(shoot_speed));

    bsp_uart::get_uart10().write(reinterpret_cast<const uint8_t *>(shoot_speed),
                                 strlen(shoot_speed));

    for (float &i : shoot_data.ball_speed)
    {
        if (i == 0.0f)
            i = shoot_data.ball_speed[0];
    }
    for (float &i : shoot_data.real_ball_speed)
    {
        if (i == 0.0f)
            i = shoot_data.real_ball_speed[0];
    }

    constexpr float outlier_threshold = 0.1f;
    const bool real_speed_stable =
        std::abs(shoot_data.avg_real_ball_speed - shoot_data.target_speed) <
        outlier_threshold;

    if (real_speed_stable &&
        std::abs(shoot_data.ball_speed[0] - shoot_data.target_speed) >
            outlier_threshold)
    {
        shoot_data.ball_speed[0] =
            0.7f * shoot_data.ball_speed[1] + 0.3f * shoot_data.ball_speed[2];
    }

    constexpr float w0        = 0.65f;
    constexpr float w1        = 0.25f;
    constexpr float w2        = 0.10f;

    shoot_data.avg_ball_speed = w0 * shoot_data.ball_speed[0] +
                                w1 * shoot_data.ball_speed[1] +
                                w2 * shoot_data.ball_speed[2];

    float e0 = shoot_data.ball_speed[0] - shoot_data.target_speed;
    float e1 = shoot_data.ball_speed[1] - shoot_data.target_speed;
    float e2 = shoot_data.ball_speed[2] - shoot_data.target_speed;

    float signed_weighted_mse = (w0 * e0 * std::abs(e0)) +
                                (w1 * e1 * std::abs(e1)) +
                                (w2 * e2 * std::abs(e2));

    [[maybe_unused]] float speed_increment =
        _ctx.pid.ball_speed_pid->calculate(0.0f, signed_weighted_mse);

    shoot_data.fric1_mps += speed_increment;

    // 共用限幅 9-17
    shoot_data.fric1_mps = std::clamp(shoot_data.fric1_mps, 9.0f, 17.0f);
}

void quad_booster_t::_launch_delay_calculate()
{
    auto &shoot_data =
        _use_deploy_data() ? _ctx.shoot_deploy_data : _ctx.shoot_normal_data;

    _ctx.data.fresh_timer++;

    if (shoot_data.fric1_mps - std::abs(_ctx.data.current_fric_mps[1]) > 0.8f &&
        shoot_data.fric1_mps - std::abs(_ctx.data.current_fric_mps[3]) > 0.8f &&
        std::abs(_ctx.data.current_fric_torque[1]) > 3.0f &&
        std::abs(_ctx.data.current_fric_torque[2]) > 3.0f &&
        _ctx.data.fresh_timer > 220)
    {
        _ctx.data.launch_delay_timer[2] = _ctx.data.launch_delay_timer[1];
        _ctx.data.launch_delay_timer[1] = _ctx.data.launch_delay_timer[0];
        _ctx.data.launch_delay_timer[0] =
            (dwt_drv_t::get_timeline_ms() - _ctx.data.signal_timer > 200.0f)
                ? _ctx.data.avg_launch_delay
                : (dwt_drv_t::get_timeline_ms() - _ctx.data.signal_timer +
                   20.0f);

        _ctx.data.avg_launch_delay = 0.7f * _ctx.data.launch_delay_timer[0] +
                                     0.2f * _ctx.data.launch_delay_timer[1] +
                                     0.1f * _ctx.data.launch_delay_timer[2];
        _ctx.data.fresh_timer = 0;
        _ctx.data.fire_count++;
    }
}

bool quad_booster_t::_use_deploy_data() const
{
    return _ctx.data.deploy_mode ||
           (_ctx.cmd != nullptr && _ctx.cmd->force_deploy);
}

void quad_booster_t::_reset_active_shoot_data()
{
    auto &shoot_data =
        _use_deploy_data() ? _ctx.shoot_deploy_data : _ctx.shoot_normal_data;
    shoot_data.reset();
}

void quad_booster_t::_fric_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_fric_torque[i] = _ctx.pid.fric_pid[i]->calculate(
            _ctx.data.target_fric_mps[i], _ctx.data.current_fric_mps[i]);
    }
}

void quad_booster_t::_anti_jam_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.target_fric_mps[i] = 0.0f;
        _ctx.data.out_fric_torque[i] = 0.0f;
    }

    _ctx.data.out_fric_torque[1] = -FRIC1_ANTI_JAM_REVERSE_TORQUE;
    _ctx.data.out_fric_torque[3] = FRIC1_ANTI_JAM_REVERSE_TORQUE;
    _ctx.data.target_trig_rad    = _ctx.data.current_trig_rad;
    _ctx.data.target_trig_radps  = 0.0f;
    _ctx.data.out_trig_torque    = 0.0f;
}

void quad_booster_t::_trigger_position_control()
{
    float error = _ctx.data.target_trig_rad - _ctx.data.current_trig_rad;
    error       = _normalize_angle(error);

    _ctx.data.target_trig_radps =
        _ctx.pid.trigger_pos_pid->calculate(error, 0.0f);

    static float ff_torque                 = 0.0f;
    constexpr float TRIG_FF_SPEED_DEADBAND = 1.0f;
    constexpr float TRIG_FF_TORQUE         = 0.505f;

    const float feed_speed = _ctx.data.target_trig_radps * TRIGGER_FEED_DIR;
    if (feed_speed > TRIG_FF_SPEED_DEADBAND)
    {
        ff_torque = TRIGGER_FEED_DIR * TRIG_FF_TORQUE;
    }
    else if (feed_speed < 0.0f)
    {
        ff_torque = 0.0f;
    }

    _ctx.data.out_trig_torque =
        _ctx.pid.trigger_spd_pid->calculate(_ctx.data.target_trig_radps,
                                            _ctx.data.current_trig_radps) +
        ff_torque;

    _ctx.data.out_trig_torque =
        std::clamp(_ctx.data.out_trig_torque, -7.0f, 7.0f);
}

void quad_booster_t::_trigger_speed_control()
{
    float ff_torque                        = 0.0f;
    constexpr float TRIG_FF_SPEED_DEADBAND = 0.5f;
    constexpr float TRIG_FF_TORQUE         = 0.505f;

    if (_ctx.data.target_trig_radps * TRIGGER_FEED_DIR > TRIG_FF_SPEED_DEADBAND)
    {
        ff_torque = TRIGGER_FEED_DIR * TRIG_FF_TORQUE;
    }

    _ctx.data.out_trig_torque =
        _ctx.pid.trigger_spd_pid->calculate(_ctx.data.target_trig_radps,
                                            _ctx.data.current_trig_radps) +
        ff_torque;
}

void quad_booster_t::_send_fric_command() const
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->send_torque(
            _ctx.data.out_fric_torque[i] +
            0.08f * _ctx.data.current_fric_torque[i]);
    }
}

void quad_booster_t::_send_raw_fric_command() const
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->send_torque(_ctx.data.out_fric_torque[i]);
    }
}

void quad_booster_t::_send_trigger_command() const
{
    _ctx.motor.trigger_wheel->send_torque(_ctx.data.out_trig_torque);
}

namespace {

float safeSqrt(const float value)
{
    float result = 0.0f;
    // CMSIS-DSP sqrt keeps this path in f32 and avoids accidental double math.
    arm_sqrt_f32(value > 0.0f ? value : 0.0f, &result);
    return result;
}

float trajectoryFunc(const float p)
{
    // p is dy/dx, i.e. tan(theta). This is the primitive function that appears
    // after rewriting the drag model with slope as the integration variable.
    const float sq = safeSqrt(1.0f + p * p);
    return p * sq + std::log(p + sq);
}

float calcIntegralX(const float p0, const float p1, const float c,
                    const float k)
{
    // Simpson integration from terminal slope p1 to initial slope p0. The
    // result is the horizontal displacement predicted by the current state.
    const float dp = (p0 - p1) / static_cast<float>(kIntegralSteps);
    float sum      = 0.0f;

    for (int i = 0; i <= kIntegralSteps; ++i)
    {
        const float p     = p1 + static_cast<float>(i) * dp;
        float denominator = c - trajectoryFunc(p);
        if (denominator < kDenominatorMin)
        {
            denominator = kDenominatorMin;
        }

        const float weight = (i == 0 || i == kIntegralSteps) ? 1.0f
                             : ((i & 1) != 0)                ? 4.0f
                                                             : 2.0f;
        sum += weight / denominator;
    }

    return (dp / 3.0f) * sum / k;
}

float calcIntegralY(const float p0, const float p1, const float c,
                    const float k)
{
    // Same integration interval as X, with an extra slope multiplier to obtain
    // vertical displacement.
    const float dp = (p0 - p1) / static_cast<float>(kIntegralSteps);
    float sum      = 0.0f;

    for (int i = 0; i <= kIntegralSteps; ++i)
    {
        const float p     = p1 + static_cast<float>(i) * dp;
        float denominator = c - trajectoryFunc(p);
        if (denominator < kDenominatorMin)
        {
            denominator = kDenominatorMin;
        }

        const float weight = (i == 0 || i == kIntegralSteps) ? 1.0f
                             : ((i & 1) != 0)                ? 4.0f
                                                             : 2.0f;
        sum += weight * p / denominator;
    }

    return (dp / 3.0f) * sum / k;
}

float calcIntegralC(const float p0, const float v0)
{
    // c is the conserved term derived from initial slope and muzzle speed.
    return kGravity * (1.0f + p0 * p0) / (kDrag * v0 * v0) + trajectoryFunc(p0);
}

bool calcJacobianInv(const float p0, const float p1, const float v0,
                     const float x0, const float y0, float (&j_inv_data)[4])
{
    // Residual D = target displacement - predicted displacement.
    // The Newton state is [p1, p0]^T, so columns are finite differences with
    // respect to terminal slope p1 and initial slope p0.
    const float c_base  = calcIntegralC(p0, v0);
    const float d0_base = x0 - calcIntegralX(p0, p1, c_base, kDrag);
    const float d1_base = y0 - calcIntegralY(p0, p1, c_base, kDrag);

    // First column: perturb terminal slope p1 while c stays unchanged.
    const float p1_eps  = p1 + kJacobianStep;
    const float x_p1    = calcIntegralX(p0, p1_eps, c_base, kDrag);
    const float y_p1    = calcIntegralY(p0, p1_eps, c_base, kDrag);
    const float dD0_dp1 = (x0 - x_p1 - d0_base) / kJacobianStep;
    const float dD1_dp1 = (y0 - y_p1 - d1_base) / kJacobianStep;

    // Second column: perturb initial slope p0, which also changes c.
    const float p0_eps  = p0 + kJacobianStep;
    const float c_eps   = calcIntegralC(p0_eps, v0);
    const float x_p0    = calcIntegralX(p0_eps, p1, c_eps, kDrag);
    const float y_p0    = calcIntegralY(p0_eps, p1, c_eps, kDrag);
    const float dD0_dp0 = (x0 - x_p0 - d0_base) / kJacobianStep;
    const float dD1_dp0 = (y0 - y_p0 - d1_base) / kJacobianStep;

    float j_data[4]     = {
        dD0_dp1,
        dD0_dp0,
        dD1_dp1,
        dD1_dp0,
    };

    const float det = dD0_dp1 * dD1_dp0 - dD0_dp0 * dD1_dp1;
    if (std::fabs(det) < kSingularEpsilon)
    {
        return false;
    }

    arm_matrix_instance_f32 j{};
    arm_matrix_instance_f32 j_inv{};
    arm_mat_init_f32(&j, 2, 2, j_data);
    arm_mat_init_f32(&j_inv, 2, 2, j_inv_data);

    // Use CMSIS-DSP for the matrix inverse so this remains consistent with the
    // rest of the control and filter code on Cortex-M.
    return arm_mat_inverse_f32(&j, &j_inv) == ARM_MATH_SUCCESS;
}

bool calcNewtonStep(float (&j_inv_data)[4], const float d0, const float d1,
                    float &dp1, float &dp0)
{
    // step = inv(J) * D. The Newton update subtracts this step from [p1, p0].
    float d_data[2]    = {d0, d1};
    float step_data[2] = {};

    arm_matrix_instance_f32 j_inv{};
    arm_matrix_instance_f32 d{};
    arm_matrix_instance_f32 step{};
    arm_mat_init_f32(&j_inv, 2, 2, j_inv_data);
    arm_mat_init_f32(&d, 2, 1, d_data);
    arm_mat_init_f32(&step, 2, 1, step_data);

    if (arm_mat_mult_f32(&j_inv, &d, &step) != ARM_MATH_SUCCESS)
    {
        return false;
    }

    dp1 = step_data[0];
    dp0 = step_data[1];
    return true;
}

bool isValidSlopeState(const float p0, const float p1)
{
    return std::isfinite(p0) && std::isfinite(p1) &&
           std::fabs(p0) < kSlopeLimit && std::fabs(p1) < kSlopeLimit &&
           p0 > p1 + kSlopeGapMin;
}

bool calcResidual(const float p0, const float p1, const float v0,
                  const float x0, const float y0, float &d0, float &d1,
                  float &residual)
{
    if (!isValidSlopeState(p0, p1))
    {
        return false;
    }

    const float c = calcIntegralC(p0, v0);
    const float x = calcIntegralX(p0, p1, c, kDrag);
    const float y = calcIntegralY(p0, p1, c, kDrag);
    if (!std::isfinite(c) || !std::isfinite(x) || !std::isfinite(y))
    {
        return false;
    }

    d0       = x0 - x;
    d1       = y0 - y;
    residual = safeSqrt(d0 * d0 + d1 * d1);
    return std::isfinite(residual);
}

}

std::optional<float> solveParabolicPitch(const float delta_x,
                                         const float delta_y,
                                         const float delta_z, const float v0,
                                         const bool use_high_root)
{
    if (v0 < 1.0e-3f)
    {
        return std::nullopt;
    }

    const float x = safeSqrt(delta_x * delta_x + delta_y * delta_y);
    if (x < 1.0e-4f)
    {
        return std::nullopt;
    }

    // y = x*tan(theta) - g*x^2/(2*v0^2) * (1 + tan(theta)^2)
    // Let u = tan(theta), a = g*x^2/(2*v0^2):
    // a*u^2 - x*u + (a + y) = 0.
    const float v0_sq = v0 * v0;
    const float a     = kGravity * x * x / (2.0f * v0_sq);
    const float disc  = x * x - 4.0f * a * (a + delta_z);
    if (disc < 0.0f || a < 1.0e-6f)
    {
        return std::nullopt;
    }

    const float sqrt_disc = safeSqrt(disc);
    const float tan_pitch = use_high_root ? ((x + sqrt_disc) / (2.0f * a))
                                          : ((x - sqrt_disc) / (2.0f * a));

    if (!std::isfinite(tan_pitch))
    {
        return std::nullopt;
    }

    return std::atan(tan_pitch);
}

std::optional<float> solveIdealPitch(const float delta_x, const float delta_y,
                                     const float delta_z, const float v0,
                                     std::optional<float> pitch_guess)
{
    if (v0 < 1.0e-3f)
    {
        return std::nullopt;
    }

    if (!pitch_guess.has_value())
    {
        pitch_guess = solveParabolicPitch(delta_x, delta_y, delta_z, v0, false);
        if (!pitch_guess.has_value())
        {
            return std::nullopt;
        }
    }

    // Convert the 3D target offset into the 2D ballistic plane.
    const float x0         = safeSqrt(delta_x * delta_x + delta_y * delta_y);
    const float y0         = delta_z;

    // A near-vertical initial guess makes the horizontal time estimate
    // unstable.
    const float v_x_approx = v0 * std::cos(*pitch_guess);
    if (v_x_approx < 1.0e-3f)
    {
        return std::nullopt;
    }

    // Initial slopes: p0 is muzzle slope, p1 is an ideal no-drag terminal slope
    // used only to seed Newton iteration.
    const float t_approx = x0 / v_x_approx;
    float p0             = std::tan(*pitch_guess);
    float p1 = (v0 * std::sin(*pitch_guess) - kGravity * t_approx) / v_x_approx;

    for (int i = 0; i < kMaxIterations; ++i)
    {
        // D = target - prediction. Once D is small enough, atan(p0) is the
        // solved pitch angle.
        float d0       = 0.0f;
        float d1       = 0.0f;
        float residual = 0.0f;
        if (!calcResidual(p0, p1, v0, x0, y0, d0, d1, residual))
        {
            return std::nullopt;
        }

        if (residual < kSolveTolerance)
        {
            return std::atan(p0);
        }

        // Solve the local linear system using DSP matrix primitives.
        float j_inv_data[4] = {};
        if (!calcJacobianInv(p0, p1, v0, x0, y0, j_inv_data))
        {
            return std::nullopt;
        }

        float dp1 = 0.0f;
        float dp0 = 0.0f;
        if (!calcNewtonStep(j_inv_data, d0, d1, dp1, dp0))
        {
            return std::nullopt;
        }

        bool accepted = false;
        float scale   = 1.0f;
        for (int j = 0; j < kLineSearchSteps; ++j)
        {
            const float next_p1 = p1 - scale * dp1;
            const float next_p0 = p0 - scale * dp0;

            float next_d0       = 0.0f;
            float next_d1       = 0.0f;
            float next_residual = 0.0f;
            if (calcResidual(next_p0, next_p1, v0, x0, y0, next_d0, next_d1,
                             next_residual) &&
                next_residual < residual)
            {
                p1       = next_p1;
                p0       = next_p0;
                accepted = true;
                break;
            }

            scale *= 0.5f;
        }

        if (!accepted)
        {
            return std::nullopt;
        }
    }

    return std::nullopt;
}

} // namespace pyro
