#include "pyro_screw_gimbal.h"
#include "pyro_ins.h"
#include "pyro_board_drv.h"
#include "pyro_algo_common.h"
#include "pyro_dwt_drv.h"
#include "screw_config.h"

#include <algorithm>
namespace pyro
{

// =========================================================
// 构造与初始化
// =========================================================

screw_gimbal_t::screw_gimbal_t() : module_base_t("screw_gimbal")
{
    _ctx = {};
}

status_t screw_gimbal_t::_init()
{
    _ctx.motor = _module_deps.motor_deps;
    _ctx.pid   = _module_deps.pid_deps;

    return PYRO_OK;
}

// =========================================================
// 核心循环回调
// =========================================================

void screw_gimbal_t::_update_feedback()
{
    _ctx.motor.pitch->update_feedback();
    _ctx.motor.yaw->update_feedback();

    // =========================================================
    // 处理 Pitch 增量式电机过零点套圈逻辑
    // =========================================================
    float now_pitch_rotor_rad = _ctx.motor.pitch->get_current_position();
    float delta_rotor = now_pitch_rotor_rad - _ctx.data.last_pitch_rotor_rad;

    if (delta_rotor > PI)
    {
        delta_rotor -= 2.0f * PI;
    }
    else if (delta_rotor < -PI)
    {
        delta_rotor += 2.0f * PI;
    }

    _ctx.data.total_pitch_motor_rad += delta_rotor;
    _ctx.data.last_pitch_rotor_rad = now_pitch_rotor_rad;

    // Yaw相对角与相对角速度解算
    const float now_yaw_rotor_rad = _ctx.motor.yaw->get_current_position();
    if (!_ctx.data.yaw_wrap_initialized)
    {
        _ctx.data.last_yaw_rotor_rad   = now_yaw_rotor_rad;
        _ctx.data.total_yaw_motor_rad  = now_yaw_rotor_rad;
        _ctx.data.yaw_wrap_initialized = true;
    }
    else
    {
        float delta_yaw_rotor =
            now_yaw_rotor_rad - _ctx.data.last_yaw_rotor_rad;
        if (delta_yaw_rotor > PI)
        {
            delta_yaw_rotor -= 2.0f * PI;
        }
        else if (delta_yaw_rotor < -PI)
        {
            delta_yaw_rotor += 2.0f * PI;
        }

        _ctx.data.total_yaw_motor_rad += delta_yaw_rotor;
        _ctx.data.last_yaw_rotor_rad = now_yaw_rotor_rad;
    }

    _ctx.data.relative_yaw_motor_rad =
        _ctx.data.total_yaw_motor_rad - YAW_OFFSET_RAD;
    _ctx.data.relative_yaw_motor_wrapped_rad =
        loop_fp32_constrain(now_yaw_rotor_rad - YAW_OFFSET_RAD, -PI, PI);
    _ctx.data.relative_yaw_motor_radps = _ctx.motor.yaw->get_current_rotate();

    // 读取 IMU 数据
    ins_drv_t::get_instance()->get_rads_n(&_ctx.data.yaw_imu_rad,
                                          &_ctx.data.pitch_imu_rad,
                                          &_ctx.data.roll_imu_rad);

    ins_drv_t::get_instance()->get_gyro_b(&_ctx.data.yaw_imu_radps,
                                          &_ctx.data.pitch_imu_radps,
                                          &_ctx.data.roll_imu_radps);

    ins_drv_t::get_instance()->get_accel_b(
        &_ctx.data.x_accel_imu, &_ctx.data.y_accel_imu, &_ctx.data.z_accel_imu);

    ins_drv_t::get_instance()->get_quaternion(
        &_ctx.data.gimbal_q[0], &_ctx.data.gimbal_q[1], &_ctx.data.gimbal_q[2],
        &_ctx.data.gimbal_q[3]);

    _communicate_chassis();
    _calculate_relative_angles();

    // =========================================================
    // 集中式非线性解算与缓存 (无参化设计的数据源头)
    // =========================================================
    // 1. 位置反馈解算
    _ctx.data.current_pitch_motor_rad =
        _motor_rad_to_pitch_rad(_ctx.data.total_pitch_motor_rad);

    // 2. 提前计算并缓存雅可比(传动比)
    // 【关键修改】在自瞄模式下，采用 IMU 解算的 relative_pitch_rad 作为雅可比输入
    float jacobian_input_rad = (_current_cmd.autoaim_mode) ? _ctx.data.relative_pitch_rad : _ctx.data.current_pitch_motor_rad;
    _ctx.data.current_jacobian = _get_motor_to_pitch_jacobian(jacobian_input_rad);

    // 3. 速度反馈解算
    if (std::abs(_ctx.data.current_jacobian) > 0.001f) {
        _ctx.data.current_pitch_motor_radps =
            _ctx.motor.pitch->get_current_rotate() / _ctx.data.current_jacobian;
    } else {
        _ctx.data.current_pitch_motor_radps = 0.0f;
    }

    // 4. 动态实时校准 (抗编码器零漂)
    _handle_dynamic_calibration();

    // =========================================================
    // 5. LESO 观测器更新 (全时段连续追踪，防止切入时状态突跳)
    // =========================================================
    const bool use_sling_leso = _current_cmd.sling_mode;
    if (use_sling_leso && _ctx.pid.yaw_pos_leso != nullptr &&
        _ctx.pid.yaw_pos_imu_leso != nullptr)
    {
        // 取上一次计算出的最终力矩作为已知控制输入 u
        float last_yaw_u = _ctx.data.out_yaw_torque;
        _ctx.pid.yaw_pos_leso->update(_ctx.data.relative_yaw_motor_rad, last_yaw_u);
        _ctx.pid.yaw_pos_imu_leso->update(_ctx.data.yaw_imu_rad, last_yaw_u);
        _ctx.data.pos_imu_leso_z1 = _ctx.pid.yaw_pos_imu_leso->get_z(1);
        _ctx.data.pos_leso_z0 = _ctx.pid.yaw_pos_leso->get_z(0);
        _ctx.data.pos_leso_z1 = _ctx.pid.yaw_pos_leso->get_z(1);
        _ctx.data.pos_leso_out =
            -_ctx.pid.yaw_pos_leso->get_z(2) / _ctx.pid.yaw_pos_leso->get_b();
    }
    if (use_sling_leso && _ctx.pid.yaw_spd_leso != nullptr)
    {
        // 取上一次计算出的最终力矩作为已知控制输入 u
        float last_yaw_u = _ctx.data.out_yaw_torque;
        _ctx.pid.yaw_spd_leso->update(_ctx.data.yaw_imu_radps, last_yaw_u);
        _ctx.data.spd_leso_z0 = _ctx.pid.yaw_spd_leso->get_z(0);
    }
    if (use_sling_leso && _ctx.pid.yaw_spd_imu_leso != nullptr)
    {
        float last_yaw_u = _ctx.data.out_yaw_torque;
        _ctx.pid.yaw_spd_imu_leso->update(_ctx.data.yaw_imu_radps, last_yaw_u);
        _ctx.data.spd_imu_leso_z0 = _ctx.pid.yaw_spd_imu_leso->get_z(0);
    }
}

void screw_gimbal_t::_gimbal_control()
{
    // --- Pitch 串级控制 (常规模式基于机械角) ---
    _ctx.data.target_pitch_radps = _ctx.pid.pitch_pos->calculate(
        _ctx.data.target_pitch_rad, _ctx.data.current_pitch_motor_rad);

    float pitch_joint_torque = _ctx.pid.pitch_spd->calculate(
        _ctx.data.target_pitch_radps, _ctx.data.current_pitch_motor_radps);

    float pitch_pid_motor_torque = 0.0f;
    if (std::abs(_ctx.data.current_jacobian) > 0.001f) {
        pitch_pid_motor_torque = pitch_joint_torque / _ctx.data.current_jacobian;
    }

    // 常规模式，传入 false
    float pitch_ff_torque = _calculate_pitch_compensation(false);

    pitch_ff_torque =
        std::clamp(pitch_ff_torque, -SCREW_MAX_TORQUE, SCREW_MAX_TORQUE);
    float pid_torque_max = SCREW_MAX_TORQUE - pitch_ff_torque;
    float pid_torque_min = -SCREW_MAX_TORQUE - pitch_ff_torque;

    pitch_pid_motor_torque = std::clamp(pitch_pid_motor_torque, pid_torque_min, pid_torque_max);

    _ctx.data.out_pitch_torque = pitch_pid_motor_torque + pitch_ff_torque;

    // --- Yaw 串级控制 (基于 IMU) ---
    float raw_yaw_error     = _ctx.data.yaw_imu_rad - _ctx.data.target_yaw_rad;
    _ctx.data.yaw_error_rad = pyro::loop_fp32_constrain(raw_yaw_error, -PI, PI);

    _ctx.data.target_yaw_radps =
        _ctx.pid.yaw_pos->calculate(0.0f, _ctx.data.yaw_error_rad);

    _ctx.data.out_yaw_torque = _ctx.pid.yaw_spd->calculate(
        _ctx.data.target_yaw_radps, _ctx.data.yaw_imu_radps);
}

void screw_gimbal_t::_gimbal_autoaim_control()
{
    // --- Pitch 串级控制 (自瞄模式基于绝对 IMU 角度) ---
    _ctx.data.target_pitch_radps = _ctx.pid.pitch_auto_pos->calculate(
        _ctx.data.target_pitch_rad, _ctx.data.pitch_imu_rad);

    float pitch_joint_torque = _ctx.pid.pitch_auto_spd->calculate(
        _ctx.data.target_pitch_radps, _ctx.data.pitch_imu_radps);

    float pitch_pid_motor_torque = 0.0f;
    if (std::abs(_ctx.data.current_jacobian) > 0.001f) {
        pitch_pid_motor_torque = pitch_joint_torque / _ctx.data.current_jacobian;
    }

    // 自瞄模式，传入 true 增大死区抵抗 IMU 高频噪声
    float pitch_ff_torque = _calculate_pitch_compensation(true);

    pitch_ff_torque =
        std::clamp(pitch_ff_torque, -SCREW_MAX_TORQUE, SCREW_MAX_TORQUE);
    float pid_torque_max = SCREW_MAX_TORQUE - pitch_ff_torque;
    float pid_torque_min = -SCREW_MAX_TORQUE - pitch_ff_torque;

    pitch_pid_motor_torque = std::clamp(pitch_pid_motor_torque, pid_torque_min, pid_torque_max);

    _ctx.data.out_pitch_torque = pitch_pid_motor_torque + pitch_ff_torque;

    // --- Yaw 串级控制 (基于 IMU) ---
    float raw_yaw_error     = _ctx.data.yaw_imu_rad - _ctx.data.target_yaw_rad;
    _ctx.data.yaw_error_rad = pyro::loop_fp32_constrain(raw_yaw_error, -PI, PI);

    _ctx.data.target_yaw_radps =
        _ctx.pid.yaw_pos->calculate(0.0f, _ctx.data.yaw_error_rad);

    _ctx.data.out_yaw_torque = _ctx.pid.yaw_spd->calculate(
        _ctx.data.target_yaw_radps, _ctx.data.yaw_imu_radps);
}

void screw_gimbal_t::_gimbal_sling_control()
{
    // --- Pitch 串级控制 (Sling模式) ---
    _ctx.data.target_pitch_radps = _ctx.pid.pitch_pos->calculate(
        _ctx.data.target_pitch_rad, _ctx.data.current_pitch_motor_rad);

    float pitch_joint_torque = _ctx.pid.pitch_spd->calculate(
        _ctx.data.target_pitch_radps, _ctx.data.current_pitch_motor_radps);

    float pitch_pid_motor_torque = 0.0f;
    if (std::abs(_ctx.data.current_jacobian) > 0.001f) {
        pitch_pid_motor_torque = pitch_joint_torque / _ctx.data.current_jacobian;
    }

    // 吊射模式，传入 false
    float pitch_ff_torque = _calculate_pitch_compensation(false);

    pitch_ff_torque =
        std::clamp(pitch_ff_torque, -SCREW_MAX_TORQUE, SCREW_MAX_TORQUE);
    float pid_torque_max = SCREW_MAX_TORQUE - pitch_ff_torque;
    float pid_torque_min = -SCREW_MAX_TORQUE - pitch_ff_torque;

    pitch_pid_motor_torque = std::clamp(pitch_pid_motor_torque, pid_torque_min, pid_torque_max);
    _ctx.data.out_pitch_torque = pitch_pid_motor_torque + pitch_ff_torque;

    // --- Yaw 纯机械相对角控制 (Sling专用) ---
    _ctx.data.relative_yaw_error_rad =
        _ctx.data.relative_yaw_motor_rad- _ctx.data.target_yaw_rad;

    _ctx.data.target_yaw_radps =
        _ctx.pid.yaw_relative_pos->calculate(0.0f,_ctx.data.relative_yaw_error_rad);

    float yaw_pid_out = _ctx.pid.yaw_relative_spd->calculate(
        _ctx.data.target_yaw_radps, _ctx.data.spd_imu_leso_z0);

    // 【修改点】：使用 LESO 估计的总扰动进行前馈补偿，替代原先的施密特触发器逻辑
    float yaw_leso_comp = 0.0f;
    if (_ctx.pid.yaw_pos_leso != nullptr)
    {
        // 获取实时观测出的阻尼与摩擦总扰动
        float estimated_disturbance = _ctx.pid.yaw_pos_leso->get_disturbance();
        // 控制律: u = PID_out - z3 / b
        yaw_leso_comp = -estimated_disturbance / _ctx.pid.yaw_pos_leso->get_b();
    }

    _ctx.data.out_yaw_torque = yaw_pid_out + yaw_leso_comp;
    // _ctx.data.out_yaw_torque = yaw_leso_comp;
    // _ctx.data.out_yaw_torque = yaw_pid_out ;
    _ctx.data.out_yaw_torque = std::clamp(_ctx.data.out_yaw_torque, -YAW_SLING_TORQUE_LIMIT, YAW_SLING_TORQUE_LIMIT);
}

void screw_gimbal_t::_handle_dynamic_calibration()
{
    // 如果处于吊射模式或尚未完成初始校准，则直接退出
    if (!_ctx.data.allow_dynamic_calib || !_ctx.data.has_initial_calibrated) {
        _dynamic_calib_sum = 0.0f;
        _dynamic_calib_timer = 0;
        return;
    }

    // 严格校准条件 (使用配置常量)：
    const bool condition = (std::abs(_ctx.data.pitch_imu_radps) < CALIB_PITCH_STILL_RADPS) &&
                           (std::abs(_ctx.data.roll_imu_rad) < CALIB_ROLL_LEVEL_RAD) &&
                           (std::abs(_ctx.data.current_pitch_motor_rad - _ctx.data.target_pitch_rad) < CALIB_PITCH_ERROR_RAD) &&
                           (std::abs(_ctx.data.current_pitch_motor_radps) < CALIB_MOTOR_STILL_RADPS);

    if (condition) {
        _dynamic_calib_timer++;
        _dynamic_calib_sum += _ctx.data.relative_pitch_rad; // 使用 IMU 四元数解算的相对角作为基准

        if (_dynamic_calib_timer >= DYNAMIC_CALIB_WINDOW_TICKS) {
            float avg_ref_pitch = _dynamic_calib_sum / static_cast<float>(DYNAMIC_CALIB_WINDOW_TICKS);
            // 覆写当前电机累计值，消除编码器累积误差，不影响初始限位
            _ctx.data.total_pitch_motor_rad = _pitch_rad_to_motor_rad(avg_ref_pitch);
            float current_motor_rad = _motor_rad_to_pitch_rad(_ctx.data.total_pitch_motor_rad);
            _ctx.data.target_pitch_rad = current_motor_rad - _ctx.data.current_pitch_motor_rad + _ctx.data.target_pitch_rad;
            _ctx.data.current_pitch_motor_rad = current_motor_rad;

            _dynamic_calib_timer = 0;
            _dynamic_calib_sum = 0.0f;
        }
    } else {
        _dynamic_calib_timer = 0;
        _dynamic_calib_sum = 0.0f;
    }
}

screw_gimbal_t::gimbal_context_t& screw_gimbal_t::get_ctx()
{
    return _ctx;
}


void screw_gimbal_t::_send_motor_command(gimbal_context_t *ctx)
{
    ctx->motor.pitch->send_torque(ctx->data.out_pitch_torque);
    ctx->motor.yaw->send_torque(ctx->data.out_yaw_torque);
}

void screw_gimbal_t::_communicate_chassis()
{
    auto &board_drv = board_drv_t::get_instance(board_drv_t::role_t::GIMBAL, can_hub_t::can1);
    if (!board_drv.check_online())
    {
        return;
    }

    const auto &rx_data = board_drv.get_c2g_rx_data();
    _ctx.data.chassis_q[0] = static_cast<float>(rx_data.chassis_q[0]) / 32767.0f;
    _ctx.data.chassis_q[1] = static_cast<float>(rx_data.chassis_q[1]) / 32767.0f;
    _ctx.data.chassis_q[2] = static_cast<float>(rx_data.chassis_q[2]) / 32767.0f;
    _ctx.data.chassis_q[3] = static_cast<float>(rx_data.chassis_q[3]) / 32767.0f;
}

void screw_gimbal_t::_calculate_relative_angles()
{
    const float cw = _ctx.data.chassis_q[0], cx = _ctx.data.chassis_q[1],
                cy = _ctx.data.chassis_q[2], cz = _ctx.data.chassis_q[3];

    const float gw = _ctx.data.gimbal_q[0], gx = _ctx.data.gimbal_q[1],
                gy = _ctx.data.gimbal_q[2], gz = _ctx.data.gimbal_q[3];

    const float chassis_yaw_imu = std::atan2(2.0f * (cw * cz + cx * cy),
                                             1.0f - 2.0f * (cy * cy + cz * cz));
    const float gimbal_yaw_imu  = std::atan2(2.0f * (gw * gz + gx * gy),
                                             1.0f - 2.0f * (gy * gy + gz * gz));

    _ctx.data.chassis_yaw_imu   = chassis_yaw_imu;

    float sinp = 2.0f * (cw * cy - cx * cz);
    sinp = std::clamp(sinp, -1.0f, 1.0f);
    _ctx.data.chassis_pitch_rad = std::asin(sinp);

    float raw_yaw_error =
        gimbal_yaw_imu - chassis_yaw_imu -
        _ctx.data.relative_yaw_motor_wrapped_rad;
    raw_yaw_error          = pyro::loop_fp32_constrain(raw_yaw_error, -PI, PI);

    const float half_err   = raw_yaw_error * 0.5f;
    const float comp_w     = std::cos(half_err);
    const float comp_z     = std::sin(half_err);

    const float aw         = comp_w * cw - comp_z * cz;
    const float ax         = comp_w * cx - comp_z * cy;
    const float ay         = comp_w * cy + comp_z * cx;
    const float az         = comp_w * cz + comp_z * cw;

    const float RcT_row2_0 = 2.0f * (ax * az + aw * ay);
    const float RcT_row2_1 = 2.0f * (ay * az - aw * ax);
    const float RcT_row2_2 = 1.0f - 2.0f * (ax * ax + ay * ay);

    const float Rg_col0_0  = 1.0f - 2.0f * (gy * gy + gz * gz);
    const float Rg_col0_1  = 2.0f * (gx * gy + gw * gz);
    const float Rg_col0_2  = 2.0f * (gx * gz - gw * gy);

    const float Rg_col2_0  = 2.0f * (gx * gz + gw * gy);
    const float Rg_col2_1  = 2.0f * (gy * gz - gw * gx);
    const float Rg_col2_2  = 1.0f - 2.0f * (gx * gx + gy * gy);

    const float r31        = RcT_row2_0 * Rg_col0_0 + RcT_row2_1 * Rg_col0_1 +
                      RcT_row2_2 * Rg_col0_2;
    const float r33 = RcT_row2_0 * Rg_col2_0 + RcT_row2_1 * Rg_col2_1 +
                      RcT_row2_2 * Rg_col2_2;

    _ctx.data.relative_pitch_rad = std::atan2(-r31, r33);
}

bool screw_gimbal_t::_calibrate_pitch_offset()
{
    _calib_tick++;
    if (_calib_tick < PITCH_CALIB_DELAY_TICKS)
    {
        return false;
    }
    _calib_pitch_sum += _ctx.data.relative_pitch_rad;

    if (_calib_tick >= PITCH_CALIB_MAX_TICKS + PITCH_CALIB_DELAY_TICKS)
    {
        const float avg_relative_pitch =
            _calib_pitch_sum / static_cast<float>(PITCH_CALIB_MAX_TICKS);

        const float theoretical_motor_rad =
            _pitch_rad_to_motor_rad(avg_relative_pitch);
        _ctx.data.total_pitch_motor_rad = theoretical_motor_rad;

        const float limit1 = _pitch_rad_to_motor_rad(PITCH_MAX_RELATIVE_RAD);
        const float limit2 = _pitch_rad_to_motor_rad(PITCH_MIN_RELATIVE_RAD);

        _ctx.data.pitch_motor_upper_limit = std::max(limit1, limit2);
        _ctx.data.pitch_motor_lower_limit = std::min(limit1, limit2);
        return true;
    }
    return false;
}

float screw_gimbal_t::_get_motor_to_pitch_jacobian(float pitch_rad) const
{
    const float theta_rad = SCREW_THETA_ZERO_RAD + pitch_rad;
    float current_S = std::sqrt(SCREW_L1_SQ_PLUS_L2_SQ - SCREW_TWO_L1_L2 * std::cos(theta_rad));

    if (current_S < 1.0f)
        current_S = 1.0f;

    const float dS_dpitch = (SCREW_TWO_L1_L2 * std::sin(theta_rad)) / (2.0f * current_S);
    const float dMotor_dpitch = dS_dpitch * 2.0f * PI;

    return dMotor_dpitch;
}

float screw_gimbal_t::_pitch_rad_to_motor_rad(float pitch_rad) const
{
    const float theta_rad = SCREW_THETA_ZERO_RAD + pitch_rad;
    const float target_S  = std::sqrt(SCREW_L1_SQ_PLUS_L2_SQ -
                                      SCREW_TWO_L1_L2 * std::cos(theta_rad));

    const float delta_S   = target_S - SCREW_S_LIMIT_BOTTOM;
    return delta_S * 2.0f * PI;
}

float screw_gimbal_t::_motor_rad_to_pitch_rad(float motor_rad) const
{
    const float delta_S   = motor_rad / (2.0f * PI);

    const float current_S = SCREW_S_LIMIT_BOTTOM + delta_S;

    float cos_theta =
        (SCREW_L1_SQ_PLUS_L2_SQ - current_S * current_S) / SCREW_TWO_L1_L2;
    cos_theta             = std::clamp(cos_theta, -1.0f, 1.0f);

    const float theta_rad = std::acos(cos_theta);
    return theta_rad - SCREW_THETA_ZERO_RAD;
}

float screw_gimbal_t::_calculate_pitch_compensation(bool is_autoaim) const
{
    static float equivalent_joint_friction = 0.0f;
    static bool is_init = false;

    // 新增静态变量用于记录施密特触发器的当前状态方向 (保持迟滞状态)
    // 1: 正向摩擦力, -1: 反向摩擦力, 0: 无摩擦力
    static int active_direction = 0;

    if (!is_init)
    {
        const float ref_dMotor_dpitch = _get_motor_to_pitch_jacobian(PITCH_FRICTION_REF_ANGLE_RAD);
        equivalent_joint_friction = PITCH_FRICTION_REF_TORQUE * ref_dMotor_dpitch;
        is_init = true;
    }

    float current_friction_mag = 2.0f;
    if (std::abs(_ctx.data.current_jacobian) > 0.001f)
    {
        current_friction_mag = equivalent_joint_friction / _ctx.data.current_jacobian;
    }

    // 动态速度死区与缓冲区切换
    float velocity_deadband = is_autoaim ? PITCH_DEADBAND_AUTOAIM_RADPS : PITCH_DEADBAND_NORMAL_RADPS;
    float velocity_buffer   = is_autoaim ? PITCH_BUFFER_AUTOAIM_RADPS : PITCH_BUFFER_NORMAL_RADPS;

    float target_radps = _ctx.data.target_pitch_radps;
    float abs_radps = std::abs(target_radps);

    // 施密特触发器阈值
    float threshold_high = velocity_deadband + velocity_buffer;
    float threshold_low  = velocity_deadband;

    // 施密特触发器逻辑 (Hysteresis)
    if (target_radps > threshold_high)
    {
        // 向上越过 deadband + buffer，触发正向突变
        active_direction = 1;
    }
    else if (target_radps < -threshold_high)
    {
        // 向下越过 -(deadband + buffer)，触发反向突变
        active_direction = -1;
    }
    else if (abs_radps < threshold_low)
    {
        // 绝对值回落至 deadband 以下，输出清零
        active_direction = 0;
    }
    // else: 如果速度落在 [threshold_low, threshold_high] 的缓冲区内，active_direction 保持上一次的状态不变

    float dynamic_friction_comp = active_direction * current_friction_mag;

    return dynamic_friction_comp;
}

void screw_gimbal_t::_fsm_execute()
{
    _ctx.cmd          = &_current_cmd;

    bool allow_active = (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode);

    if (_ctx.data.is_calibrating || !_ctx.data.has_initial_calibrated)
    {
        allow_active = false;
    }

    if (allow_active)
    {
        _main_fsm.change_state(&_fsm_active);
    }
    else
    {
        _main_fsm.change_state(&_fsm_passive);
    }

    _main_fsm.execute(this);
}

} // namespace pyro
