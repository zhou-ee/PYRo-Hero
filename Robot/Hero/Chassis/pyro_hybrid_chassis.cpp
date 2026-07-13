#include "pyro_hybrid_chassis.h"
#include "pyro_algo_common.h"
#include <arm_math.h> // 引入 CMSIS-DSP 库
#include "pyro_dji_motor_drv.h"
#include "pyro_power_control.h"
#include "pyro_sr04_drv.h"
#include <algorithm>
#include "pyro_referee.h"
#include "pyro_sr05_drv.h"
#include "pyro_us100_drv.h"

namespace pyro
{

namespace
{
constexpr uint16_t CAP_EXTRA_POWER_ENABLE_CV = 1900;
constexpr uint16_t CAP_EXTRA_POWER_DISABLE_CV = 1750;
constexpr float CAP_EXTRA_POWER_W = 100.0f;

constexpr uint32_t SUPERCAP_ENABLE_DELAY_TICKS = 1000;
constexpr uint32_t SUPERCAP_REFRESH_TICKS = 10;
constexpr uint32_t SUPERCAP_DISABLE_DEBOUNCE_TICKS = 30;
} // namespace

// =========================================================
// 构造与初始化
// =========================================================

hybrid_chassis_t::hybrid_chassis_t() : module_base_t("hybrid")
{
    _ctx = {};
}

status_t hybrid_chassis_t::_init()
{
    _ctx.motor  = _module_deps.motor_deps;
    _ctx.pid    = _module_deps.pid_deps;

    // 使用 config.h 中的参数初始化运动学模型xssx
    _kinematics = new hybrid_kin_t(
        TRACK_SPACING, MEC_FRONT_TRACK_WIDTH / 2 + MEC_FRONT_WHEELBASE,
        MEC_FRONT_TRACK_WIDTH / 2 + MEC_FRONT_WHEELBASE,
        MEC_REAR_TRACK_WIDTH / 2 + MEC_REAR_WHEELBASE,
        MEC_REAR_TRACK_WIDTH / 2 + MEC_REAR_WHEELBASE);
    // float x = 0.15f;
    // _kinematics = new hybrid_kin_t(TRACK_SPACING,
    //                            0.15f + x,0.15f + x,0.66f - x, 0.66f - x);

    _ctx.powermeter = new powermeter_drv_t(0x212, can_hub_t::can2);
    _ctx.powermeter->init();

    _power_control_init();

    return PYRO_OK;
}

hybrid_chassis_t::hybrid_context_t &hybrid_chassis_t::get_ctx()
{
    return _ctx;
}


void hybrid_chassis_t::_power_control_init()
{
    power_fit_params_t params;

    // 麦轮（3508）功控参数
    // 四组参数取平均后的结果
    params.k1    = 0.03912453f; // 机械功率项系数
    params.k2    = 0.06985056f; // 铜损/热损耗系数
    params.k3    = 0.00001723f; // 高频摩擦系数 (转速平方项)
    params.k4    = 0.00917522f; // 库仑摩擦系数 (摩擦损耗项)
    params.k5    = 0.75000000f; // 静态基础功耗 (强制固定)
    params.alpha = 0.00393f;    // 默认电阻温度系数

    // 注册 4 个麦轮电机
    for (int i = 0; i < 4; i++)
    {
        _ctx.power_motor_data[i] =
            power_controller_t::get_instance().register_motor(params);
    }

    // 履带 （dm4310p）功控参数
    // params.k1    = 0.10707706f; // 机械功率项系数
    // params.k2    = 0.60690610f; // 铜损/热损耗系数
    // params.k3    = 0.00001238f; // 高频摩擦系数
    // params.k4    = 0.00000000f; // 库仑摩擦系数
    // params.k5    = 0.85000000f; // 静态基础功耗 (强制固定)
    // params.alpha = 0.00393f;    // 默认电阻温度系数

    // 注册 2 个履带电机
    // for (int i = 0; i < 2; i++)
    // {
    //     _ctx.power_motor_data[i + 4] =
    //         power_controller_t::get_instance().register_motor(params);
    // }

    // 初始化缓冲能量 PID (安全缓冲参考值设为 60J,pid默认值)
    power_controller_t::get_instance().config_buffer_loop(40.0f);
}


void hybrid_chassis_t::_update_feedback()
{
    // 1. 更新所有电机反馈
    for (const auto &i : _ctx.motor.mecanum)
        i->update_feedback();
    for (const auto &i : _ctx.motor.track)
        i->update_feedback();
    for (const auto &i : _ctx.motor.leg)
        i->update_feedback();
    _ctx.motor.yaw->update_feedback();

    // 2. 读取 IMU 数据作为底盘姿态反馈
    float raw_yaw, raw_pitch, raw_roll;
    ins_drv_t::get_instance()->get_rads_n(&raw_yaw, &raw_pitch, &raw_roll);

    // 减去机械安装零点偏移
    raw_pitch -= PITCH_OFFSET_RAD;
    raw_roll -= ROLL_OFFSET_RAD;

    // --- 一阶低通滤波 (IMU) ---
    static float filtered_pitch = 0.0f;
    static float filtered_roll  = 0.0f;
    static bool is_first_run    = true;
    const float IMU_LPF_ALPHA   = 0.15f;

    if (is_first_run)
    {
        filtered_pitch = raw_pitch;
        filtered_roll  = raw_roll;
        is_first_run   = false;
    }
    else
    {
        filtered_pitch =
            IMU_LPF_ALPHA * raw_pitch + (1.0f - IMU_LPF_ALPHA) * filtered_pitch;
        filtered_roll =
            IMU_LPF_ALPHA * raw_roll + (1.0f - IMU_LPF_ALPHA) * filtered_roll;
    }

    _ctx.data.current_yaw_rad   = raw_yaw;
    _ctx.data.current_pitch_rad = filtered_pitch;
    _ctx.data.current_roll_rad  = filtered_roll;

    // =========================================================================
    // --- 新增：测距模块 变化率限幅 + 二阶低通滤波 (2nd-Order LPF) ---
    // =========================================================================
    int32_t raw_front_dist_int32 =
        static_cast<int32_t>(sr04_drv::get_instance().get_distance()) -
        FRONT_DISTANCE_OFFSET;
    int32_t raw_back_dist_int32 =
        static_cast<int32_t>(sr05_drv::get_instance().get_distance()) -
        BACK_DISTANCE_OFFSET;

    auto raw_front_dist = static_cast<float>(raw_front_dist_int32);
    auto raw_back_dist  = static_cast<float>(raw_back_dist_int32);

    // if (!_ctx.data.distance_lpf_initialized)
    // {
    //     // 赋予初始值，防止开机瞬间缓慢从 0 爬升
    //     for (int i = 0; i < 2; i++)
    //     {
    //         _ctx.data.front_lpf_state[i] = raw_front_dist;
    //         _ctx.data.back_lpf_state[i]  = raw_back_dist;
    //     }
    //     _ctx.data.distance_lpf_initialized = true;
    // }
    // else
    // {
    // 1. 变化率限幅 (Slew Rate Limiting)
    // 即使开机第一帧是毛刺导致初始化错误，后续也能以最大合法步长迅速回归真实值，避免死锁
    float front_delta   = raw_front_dist - _ctx.data.front_lpf_state[1];
    if (front_delta > CLIMB_DIST_MAX_JUMP)
        raw_front_dist = _ctx.data.front_lpf_state[1] + CLIMB_DIST_MAX_JUMP;
    else if (front_delta < -CLIMB_DIST_MAX_JUMP)
        raw_front_dist = _ctx.data.front_lpf_state[1] - CLIMB_DIST_MAX_JUMP;

    float back_delta = raw_back_dist - _ctx.data.back_lpf_state[1];
    if (back_delta > CLIMB_DIST_MAX_JUMP)
        raw_back_dist = _ctx.data.back_lpf_state[1] + CLIMB_DIST_MAX_JUMP;
    else if (back_delta < -CLIMB_DIST_MAX_JUMP)
        raw_back_dist = _ctx.data.back_lpf_state[1] - CLIMB_DIST_MAX_JUMP;

    // 2. 前测距 二阶级联滤波 (平滑高频白噪声)
    _ctx.data.front_lpf_state[0] =
        CLIMB_DIST_LPF_ALPHA * raw_front_dist +
        (1.0f - CLIMB_DIST_LPF_ALPHA) * _ctx.data.front_lpf_state[0];
    _ctx.data.front_lpf_state[1] =
        CLIMB_DIST_LPF_ALPHA * _ctx.data.front_lpf_state[0] +
        (1.0f - CLIMB_DIST_LPF_ALPHA) * _ctx.data.front_lpf_state[1];

    // 3. 后测距 二阶级联滤波
    _ctx.data.back_lpf_state[0] =
        CLIMB_DIST_LPF_ALPHA * raw_back_dist +
        (1.0f - CLIMB_DIST_LPF_ALPHA) * _ctx.data.back_lpf_state[0];
    _ctx.data.back_lpf_state[1] =
        CLIMB_DIST_LPF_ALPHA * _ctx.data.back_lpf_state[0] +
        (1.0f - CLIMB_DIST_LPF_ALPHA) * _ctx.data.back_lpf_state[1];
    // }

    _ctx.data.front_distance_mm       = static_cast<int32_t>(raw_front_dist);
    _ctx.data.back_distance_mm        = static_cast<int32_t>(raw_back_dist);

    // 取出第二阶的结果作为最终平滑值，供外部状态机判定使用
    _ctx.data.filtered_front_distance = _ctx.data.front_lpf_state[1];
    _ctx.data.filtered_back_distance  = _ctx.data.back_lpf_state[1];
    // =========================================================================


    // 3. 转换并记录电机转速与位置
    float current_angle =
        _ctx.motor.yaw->get_current_position() - YAW_OFFSET_RAD;
    current_angle = loop_fp32_constrain(current_angle, -PI, PI);
    _ctx.data.current_yaw_error =
        loop_fp32_constrain(0 - current_angle, -PI, PI);

    for (int i = 0; i < 4; i++)
    {
        _ctx.data.current_wheel_rpm[i] =
            radps_to_rpm(_ctx.motor.mecanum[i]->get_current_rotate() *
                         dji_m3508_motor_drv_t::reciprocal_reduction_ratio);
        _ctx.data.current_mecanum_torque[i] =
            _ctx.motor.mecanum[i]->get_current_torque();
        _ctx.data.current_mecanum_temp[i] =
            _ctx.motor.mecanum[i]->get_temperature();
        _ctx.data.wheel_online[i] = _ctx.motor.mecanum[i]->is_online();
    }

    auto real_vel = _kinematics->forward_solve(
        rpm_to_mps(_ctx.data.current_wheel_rpm[0], WHEEL_RADIUS),
        rpm_to_mps(-_ctx.data.current_wheel_rpm[1], WHEEL_RADIUS),
        rpm_to_mps(_ctx.data.current_wheel_rpm[2], WHEEL_RADIUS),
        rpm_to_mps(-_ctx.data.current_wheel_rpm[3], WHEEL_RADIUS));

    _ctx.data.real_vx = real_vel.vx;
    _ctx.data.real_vy = real_vel.vy;
    _ctx.data.real_wz = real_vel.wz;

    for (int i = 0; i < 2; i++)
    {
        _ctx.data.current_track_rpm[i] =
            radps_to_rpm(_ctx.motor.track[i]->get_current_rotate());
        _ctx.data.current_track_torque[i] =
            _ctx.motor.track[i]->get_current_torque();
        _ctx.data.current_track_temp[i] =
            _ctx.motor.track[i]->get_temperature();
    }

    // 左右腿对称性修正：对右腿(leg[1])的读取数据取反，抹平机械差异
    float left_leg_raw =
        -(_ctx.motor.leg[0]->get_current_position() - LEFT_LEG_OFFSET_RAD);
    _ctx.data.current_leg_rad[0]   = loop_fp32_constrain(left_leg_raw, -PI, PI);
    _ctx.data.current_leg_radps[0] = -_ctx.motor.leg[0]->get_current_rotate();

    float right_leg_raw =
        _ctx.motor.leg[1]->get_current_position() - RIGHT_LEG_OFFSET_RAD;
    _ctx.data.current_leg_rad[1] = loop_fp32_constrain(right_leg_raw, -PI, PI);
    _ctx.data.current_leg_radps[1] = _ctx.motor.leg[1]->get_current_rotate();


    _ctx.powermeter->get_data(_ctx.powermeter_feedback);

    _ctx.data.total_predicted_power =
        power_controller_t::get_instance().get_total_predicted_power();

    _ctx.data.buf_energy =
        referee_drv_t::get_instance()->get_data().power_heat.buffer_energy;

    // 4. 更新 cap_tx 数据
    _ctx.supercap_cmd.power_referee = 0;
    _ctx.supercap_cmd.power_limit_referee =
        referee_drv_t::get_instance()
            ->get_data()
            .robot_status.chassis_power_limit;
    _ctx.supercap_cmd.power_buffer_limit_referee = 60.0f;
    _ctx.supercap_cmd.power_buffer_referee =
        referee_drv_t::get_instance()->get_data().power_heat.buffer_energy;
    _ctx.supercap_cmd.kill_chassis_user = 0;
    _ctx.supercap_cmd.speed_up_user_now = 0;

    // 5. 更新 cap_rx 数据
    _ctx.cap_feedback = supercap_drv_t::get_instance()->get_feedback();

    _ctx.powermeter->get_data(_ctx.powermeter_feedback);
}

// =========================================================
// 核心解算与控制逻辑
// =========================================================

void hybrid_chassis_t::_kinematics_solve()
{
    // -------------------------------------------------------------
    // 1. 跟随 PID 计算 (算出底盘需要的自旋速度 wz)
    // -------------------------------------------------------------
    // Calculate(measurement, target) 或 (error, 0)
    // 假设 pid_t::calculate(target, current)，我们将 error 作为 P项输入
    float target_relative_yaw_rad = 0.0f;
    const auto now_tick = static_cast<uint32_t>(xTaskGetTickCount());

    if (_ctx.cmd->pseudo_gyro_en)
    {
        if (!_ctx.data.pseudo_gyro_active)
        {
            _ctx.data.pseudo_gyro_active    = true;
            _ctx.data.pseudo_gyro_phase_rad = 0.0f;
            _ctx.data.pseudo_gyro_last_tick = now_tick;
            if (_ctx.pid.follow_yaw_pid != nullptr)
            {
                _ctx.pid.follow_yaw_pid->clear();
            }
        }
        else
        {
            const uint32_t elapsed_tick =
                now_tick - _ctx.data.pseudo_gyro_last_tick;
            const float elapsed_s =
                static_cast<float>(elapsed_tick * portTICK_PERIOD_MS) *
                0.001f;

            _ctx.data.pseudo_gyro_phase_rad =
                loop_fp32_constrain(_ctx.data.pseudo_gyro_phase_rad +
                                        PSEUDO_GYRO_PHASE_RADPS * elapsed_s,
                                    -PI, PI);
            _ctx.data.pseudo_gyro_last_tick = now_tick;
        }

        target_relative_yaw_rad =
            PSEUDO_GYRO_YAW_AMPLITUDE_RAD *
            arm_sin_f32(_ctx.data.pseudo_gyro_phase_rad);
    }
    else
    {
        if (_ctx.data.pseudo_gyro_active && _ctx.pid.follow_yaw_pid != nullptr)
        {
            _ctx.pid.follow_yaw_pid->clear();
        }
        _ctx.data.pseudo_gyro_active         = false;
        _ctx.data.pseudo_gyro_phase_rad      = 0.0f;
        _ctx.data.pseudo_gyro_target_yaw_rad = 0.0f;
        _ctx.data.pseudo_gyro_last_tick      = now_tick;
    }

    _ctx.data.pseudo_gyro_target_yaw_rad = target_relative_yaw_rad;
    const float yaw_pid_measure =
        _ctx.data.current_yaw_error + target_relative_yaw_rad;
    const float follow_wz =
        _ctx.pid.follow_yaw_pid->calculate(0.0f, yaw_pid_measure);

    // 最终角速度 = 跟随产生的角速度 + 选手手动输入的角速度(小陀螺/微调)
    float final_wz      = follow_wz;

    // -------------------------------------------------------------
    // 2. 矢量旋转 (将云台坐标系速度转换到底盘坐标系)
    // const float theta       = _ctx.data.current_yaw_error;
    //
    // const float c_theta     = arm_cos_f32(theta);
    // const float s_theta     = arm_sin_f32(theta);
    //
    // // 旋转矩阵公式 (逆时针旋转 theta)
    // const float vx_chassis  = _ctx.cmd->vx * c_theta + _ctx.cmd->vy *
    // s_theta; const float vy_chassis  = -_ctx.cmd->vx * s_theta + _ctx.cmd->vy
    // * c_theta;
    //
    // const auto wheel_speeds = _kinematics->solve(vx_chassis, vy_chassis,
    //                                              final_wz,
    //                                              _ctx.cmd->track_en);

    const float theta   = _ctx.data.current_yaw_error;
    // const float theta   = 0;

    const float c_theta = arm_cos_f32(theta);
    const float s_theta = arm_sin_f32(theta);

    // 旋转矩阵公式 (逆时针旋转 theta)
    float vx_chassis    = _ctx.cmd->vx * c_theta + _ctx.cmd->vy * s_theta;
    float vy_chassis    = -_ctx.cmd->vx * s_theta + _ctx.cmd->vy * c_theta;


    if (abs(final_wz) > 2.5f)
    {
        vx_chassis *= 0.15f;
        vy_chassis *= 0.15f;
    }


    if (_ctx.cmd->crossing_en)
    {
        vx_chassis = std::clamp(vx_chassis, -0.7f, 0.7f);
    }



    int offline_count  = 0;
    auto missing_wheel = hybrid_kin_t::missing_mec_e::NONE;

    // 根据数组索引对应找出具体离线的轮子 (0:FL, 1:FR, 2:BL, 3:BR)
    if (!_ctx.data.wheel_online[0])
    {
        offline_count++;
        missing_wheel = hybrid_kin_t::missing_mec_e::FL;
    }
    if (!_ctx.data.wheel_online[1])
    {
        offline_count++;
        missing_wheel = hybrid_kin_t::missing_mec_e::FR;
    }
    if (!_ctx.data.wheel_online[2])
    {
        offline_count++;
        missing_wheel = hybrid_kin_t::missing_mec_e::BL;
    }
    if (!_ctx.data.wheel_online[3])
    {
        offline_count++;
        missing_wheel = hybrid_kin_t::missing_mec_e::BR;
    }

    // 如果有两个或以上的轮子离线，失去冗余控制能力，强制速度全为 0
    if (offline_count >= 2)
    {
        vx_chassis    = 0.0f;
        vy_chassis    = 0.0f;
        final_wz      = 0.0f;
        missing_wheel = hybrid_kin_t::missing_mec_e::NONE; // 速度全为0
    }

    // -------------------------------------------------------------
    // 4. 运动学解算 (带入缺失轮枚举)
    // -------------------------------------------------------------
    const auto wheel_speeds = _kinematics->solve(
        vx_chassis, vy_chassis, final_wz, _ctx.cmd->crossing_en, missing_wheel);

    // 麦轮转速分配 (右侧反转视底层驱动而定，此处按常规处理)
    _ctx.data.target_wheel_rpm[0] =
        mps_to_rpm(wheel_speeds.mec_fl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[1] =
        -mps_to_rpm(wheel_speeds.mec_fr, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[2] =
        mps_to_rpm(wheel_speeds.mec_bl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[3] =
        -mps_to_rpm(wheel_speeds.mec_br, WHEEL_RADIUS);

    // 履带分配 (差速模型)
    if (_ctx.cmd->crossing_en)
    {
        _ctx.data.target_track_rpm[0] =
            mps_to_rpm(wheel_speeds.track_l, TRACK_RADIUS);
        _ctx.data.target_track_rpm[1] =
            -mps_to_rpm(wheel_speeds.track_r, TRACK_RADIUS);
    }
    else
    {
        _ctx.data.target_track_rpm[0] = 0.0f;
        _ctx.data.target_track_rpm[1] = 0.0f;
    }

    _ctx.data.target_pitch_rad = NORMAL_PITCH;
}

void hybrid_chassis_t::_supercap_control()
{
    // NOLINTBEGIN
    static bool cap_enabled = false;
    static uint32_t enable_timer = 0;
    static uint32_t refresh_timer = 0;
    static uint32_t disable_timer = 0;

    bool current_status      = referee_drv_t::get_instance()
                              ->get_data()
                              .robot_status.power_management_chassis_output;

    if (current_status)
    {
        disable_timer = 0;

        if (!cap_enabled)
        {
            refresh_timer = 0;
            if (++enable_timer >= SUPERCAP_ENABLE_DELAY_TICKS)
            {
                enable_timer             = 0;
                cap_enabled              = true;
                _ctx.supercap_cmd.use_cap = 1;
                supercap_drv_t::get_instance()->send_cmd(_ctx.supercap_cmd);
            }
        }
        else if (++refresh_timer >= SUPERCAP_REFRESH_TICKS)
        {
            refresh_timer           = 0;
            _ctx.supercap_cmd.use_cap = 1;
            _ctx.supercap_cmd.power_buffer_referee =
                referee_drv_t::get_instance()
                    ->get_data()
                    .power_heat.buffer_energy;
            supercap_drv_t::get_instance()->send_cmd(
                _ctx.supercap_cmd); // NOLINT
        }
    }
    else
    {
        enable_timer  = 0;
        refresh_timer = 0;

        if (cap_enabled &&
            ++disable_timer >= SUPERCAP_DISABLE_DEBOUNCE_TICKS)
        {
            disable_timer            = 0;
            cap_enabled              = false;
            _ctx.supercap_cmd.use_cap = 0;
            supercap_drv_t::get_instance()->send_cmd(
                _ctx.supercap_cmd); // NOLINT
        }
        else if (!cap_enabled)
        {
            disable_timer = 0;
        }
    }
    // NOLINTEND
}


void hybrid_chassis_t::_power_control()
{
    // 1. 将底层反馈与 PID 输出的期望扭矩传入功率控制节点
    for (int i = 0; i < 4; i++)
    {
        if (_ctx.power_motor_data[i] != nullptr)
        {
            _ctx.power_motor_data[i]->target_cmd =
                _ctx.data.out_mecanum_torque[i];
            _ctx.power_motor_data[i]->rpm  = _ctx.data.current_wheel_rpm[i];
            _ctx.power_motor_data[i]->temp = _ctx.data.current_mecanum_temp[i];
        }
    }

    // for (int i = 0; i < 2; i++)
    // {
    //     if (_ctx.power_motor_data[i + 4] != nullptr)
    //     {
    //         _ctx.power_motor_data[i + 4]->target_cmd =
    //             _ctx.data.out_track_torque[i];
    //         _ctx.power_motor_data[i + 4]->rpm =
    //         _ctx.data.current_track_rpm[i]; _ctx.power_motor_data[i +
    //         4]->temp =
    //             _ctx.data.current_track_temp[i];
    //     }
    // }

    // 2. 调用核心求解器进行动态功率限制

    static bool cap_extra_power_enabled = false;
    if (_ctx.cap_feedback.vot_cap >= CAP_EXTRA_POWER_ENABLE_CV)
    {
        cap_extra_power_enabled = true;
    }
    else if (_ctx.cap_feedback.vot_cap <= CAP_EXTRA_POWER_DISABLE_CV)
    {
        cap_extra_power_enabled = false;
    }

    const float cap_extra_power =
        cap_extra_power_enabled ? CAP_EXTRA_POWER_W : 0.0f;

    power_controller_t::get_instance().solve(
        referee_drv_t::get_instance()
            ->get_data()
            .robot_status.chassis_power_limit,
        referee_drv_t::get_instance()->get_data().power_heat.buffer_energy,
        cap_extra_power);

    // 3. 将解算后的安全指令写回到底盘数据上下文中，等待发送
    for (int i = 0; i < 4; i++)
    {
        if (_ctx.power_motor_data[i] != nullptr)
        {
            _ctx.data.out_mecanum_torque[i] =
                _ctx.power_motor_data[i]->safe_cmd;
        }
    }

    // for (int i = 0; i < 2; i++)
    // {
    //     if (_ctx.power_motor_data[i + 4] != nullptr)
    //     {
    //         _ctx.data.out_track_torque[i] = _ctx.power_motor_data[i +
    //         4]->safe_cmd;
    //     }
    // }
}


void hybrid_chassis_t::_leg_vmc()
{
    const float pitch     = _ctx.data.current_pitch_rad;
    const float roll      = _ctx.data.current_roll_rad;

    // 预计算 DSP 三角函数
    const float cos_pitch = arm_cos_f32(pitch);
    const float sin_pitch = arm_sin_f32(pitch);

    // 1. 计算姿态维稳所需的宏观虚拟力
    const float f_pitch =
        _ctx.pid.pitch_pid->calculate(_ctx.data.target_pitch_rad, pitch);
    const float f_roll = _ctx.pid.roll_pid->calculate(0, roll);

    for (int i = 0; i < 2; i++)
    {
        const float theta     = _ctx.data.current_leg_rad[i];
        const float theta_dot = _ctx.data.current_leg_radps[i];

        // 2. 多项式求解雅可比及端点坐标
        const float j_x =
            evaluate_polynomial(theta, JX_POLY_COEF, JX_POLY_DEGREE);
        const float j_y =
            evaluate_polynomial(theta, JY_POLY_COEF, JY_POLY_DEGREE);
        const float x_b =
            evaluate_polynomial(theta, XB_POLY_COEF, XB_POLY_DEGREE);
        const float y_b =
            evaluate_polynomial(theta, YB_POLY_COEF, YB_POLY_DEGREE);

        // 3. 计算重力前馈补偿 (使用预计算的 DSP 三角函数)
        const float y_wheel   = y_b - H_HIP_OFFSET;
        const float numerator = DIST_FRONT * cos_pitch - H_COG * sin_pitch;
        const float denominator =
            (DIST_FRONT + DIST_HIP + x_b) * cos_pitch + y_wheel * sin_pitch;

        float f_gravity_ff = 0.0f;
        if (fabsf(denominator) > 1e-4f)
        {
            f_gravity_ff = (MASS * GRAVITY * numerator) / denominator;
        }

        // 4. 提取动态雅可比标量
        const float j_dynamic   = j_y * cos_pitch + j_x * sin_pitch;

        // 5. 将任务空间的力分别映射为关节空间的力矩
        const float tau_gravity = j_dynamic * (0.5f * f_gravity_ff);
        float tau_pid  = j_dynamic * (f_pitch + (i == 0 ? f_roll : -f_roll));

        // 6. 虚拟阻尼墙限位保护
        float tau_wall = 0.0f;
        if (theta > LEG_MAX_POS - LEG_POS_BUFFER_RAD)
        {
            tau_wall =
                -LEG_K_WALL * (theta - (LEG_MAX_POS - LEG_POS_BUFFER_RAD)) -
                LEG_D_WALL * theta_dot;
            tau_wall = fminf(0.0f, tau_wall);
        }
        else if (theta < LEG_MIN_POS + LEG_POS_BUFFER_RAD)
        {
            tau_wall =
                LEG_K_WALL * ((LEG_MIN_POS + LEG_POS_BUFFER_RAD) - theta) -
                LEG_D_WALL * theta_dot;
            tau_wall = fmaxf(0.0f, tau_wall);
        }

        // 7. 力矩饱和安全限制 (基于优先级的削峰逻辑)
        const float tau_priority = tau_gravity + tau_wall;
        float tau_total          = 0.0f;

        if (fabsf(tau_priority + tau_pid) > LEG_MAX_TORQUE)
        {
            if (fabsf(tau_priority) >= LEG_MAX_TORQUE)
            {
                tau_total =
                    (tau_priority > 0.0f) ? LEG_MAX_TORQUE : -LEG_MAX_TORQUE;
            }
            else
            {
                const float tau_avail = LEG_MAX_TORQUE - fabsf(tau_priority);
                tau_pid   = (tau_pid > 0.0f) ? tau_avail : -tau_avail;
                tau_total = tau_priority + tau_pid;
            }
        }
        else
        {
            tau_total = tau_priority + tau_pid;
        }

        tau_total = fminf(fmaxf(tau_total, -LEG_MAX_TORQUE), LEG_MAX_TORQUE);

        // tau_total = tau_gravity; // 重力补偿测试
        // if (fabsf(tau_total) > LEG_MAX_TORQUE)
        // {
        //     tau_total =
        //             (tau_total > 0.0f) ? LEG_MAX_TORQUE : -LEG_MAX_TORQUE;
        // }
        // 8. 输出并再次针对右腿作符号映射
        // _ctx.data.out_leg_torque[i] = (i == 0 ? 1.0f : -1.0f) * tau_gravity;
        _ctx.data.out_leg_torque[i] = (i == 0 ? -1.0f : 1.0f) * tau_total;
    }
}

void hybrid_chassis_t::_leg_length_control()
{
    const float pitch        = _ctx.data.current_pitch_rad;

    // 预计算 DSP 三角函数，用于雅可比映射与重力场旋转投影
    const float cos_pitch    = arm_cos_f32(pitch);
    const float sin_pitch    = arm_sin_f32(pitch);

    // 1. 预计算收腿的目标长度 (Task Space Target)
    // 使用专为长度控制放宽的限幅 LEG_LENGTH_MIN_POS (0.0f) 加上缓冲，
    // 确保收腿指令能引导机构钻入最深处的物理限位
    const float target_theta = LEG_LENGTH_MIN_POS + LEG_LENGTH_POS_BUFFER_RAD;
    // const float target_theta = 1.4f;
    const float target_y =
        evaluate_polynomial(target_theta, YB_POLY_COEF, YB_POLY_DEGREE);

    for (int i = 0; i < 2; i++)
    {
        const float theta     = _ctx.data.current_leg_rad[i];
        const float theta_dot = _ctx.data.current_leg_radps[i];

        // 2. 正向运动学：计算当前真实腿长 y_b 与动态雅可比
        const float j_x =
            evaluate_polynomial(theta, JX_POLY_COEF, JX_POLY_DEGREE);
        const float j_y =
            evaluate_polynomial(theta, JY_POLY_COEF, JY_POLY_DEGREE);
        const float y_b =
            evaluate_polynomial(theta, YB_POLY_COEF, YB_POLY_DEGREE);

        // 当前腿部在 Y 轴的真实收缩线速度 (m/s)
        const float y_dot = j_y * theta_dot;

        // 3. 任务空间(腿长)串级 PID 计算期望动态拉力 f_pid (N)
        _ctx.data.target_leg_radps[i] =
            _ctx.pid.leg_pos_pid[i]->calculate(target_y, y_b);
        float f_pid = _ctx.pid.leg_vel_pid[i]->calculate(
            _ctx.data.target_leg_radps[i], y_dot);

        // 4. 雅可比映射：将直线拉力转换为电机主动力矩 (仅为运动所需力)
        const float j_dynamic = j_y * cos_pitch + j_x * sin_pitch;
        float tau_pid         = j_dynamic * f_pid;

        // ==========================================================
        // 5. 【核心】：非线性自重补偿 + 旋转场降维投影
        // ==========================================================
        // 从 SolidWorks 提取的纯粹抗重力拟合多项式
        float tau_gravity_base =
            evaluate_polynomial(theta, TAU_GRAVITY_COEF, TAU_GRAVITY_DEGREE);

        // 将平地重力映射乘上 cos(pitch)，衰减掉倾斜失去的垂直分量
        // 同时引入你在 config 中预留的微调系数 K_TAU_GRAVITY
        float tau_gravity_ff = tau_gravity_base * cos_pitch * K_TAU_GRAVITY;

        // ==========================================================
        // 6. 极软虚拟墙保护与 PID 强制剥权 (基于专用的长度限幅)
        // ==========================================================
        float tau_wall       = 0.0f;

        // 使用针对悬空状态设计的极软参数 (LEG_GRA_K_WALL / LEG_GRA_D_WALL)
        // 限幅基准替换为 LEG_LENGTH_MAX_POS 和 LEG_LENGTH_MIN_POS
        if (theta > LEG_LENGTH_MAX_POS - LEG_LENGTH_POS_BUFFER_RAD)
        {
            const float spring =
                -LEG_GRA_K_WALL *
                (theta - (LEG_LENGTH_MAX_POS - LEG_LENGTH_POS_BUFFER_RAD));
            const float damp =
                (theta_dot > 0.0f) ? (-LEG_GRA_D_WALL * theta_dot) : 0.0f;
            tau_wall = fminf(0.0f, spring + damp);

            // 剥权保护：方向错误时没收 PID 控制权
            if (tau_pid > 0.0f)
                tau_pid = 0.0f;
        }
        else if (theta < LEG_LENGTH_MIN_POS + LEG_LENGTH_POS_BUFFER_RAD)
        {
            const float spring =
                LEG_GRA_K_WALL *
                ((LEG_LENGTH_MIN_POS + LEG_LENGTH_POS_BUFFER_RAD) - theta);
            const float damp =
                (theta_dot < 0.0f) ? (-LEG_GRA_D_WALL * theta_dot) : 0.0f;
            tau_wall = fmaxf(0.0f, spring + damp);

            // 剥权保护：方向错误时没收 PID 控制权
            if (tau_pid < 0.0f)
                tau_pid = 0.0f;
        }

        // 7. 物理限幅与最终输出
        float tau_total             = tau_pid + tau_wall + tau_gravity_ff;
        // float tau_total = tau_gravity_ff;
        tau_total                   = fminf(fmaxf(tau_total, -18.0f), 18.0f);

        // 针对右腿作符号反转映射
        // _ctx.data.out_leg_torque[i] = (i == 0 ? 1.0f : -1.0f) * tau_total;
        _ctx.data.out_leg_torque[i] = (i == 0 ? -1.0f : 1.0f) * tau_total;
    }
}


void hybrid_chassis_t::_mecanum_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_mecanum_torque[i] = _ctx.pid.mecanum_pid[i]->calculate(
            _ctx.data.target_wheel_rpm[i], _ctx.data.current_wheel_rpm[i]);
    }
    // _power_control();
    // _ctx.data.out_mecanum_torque[0] = 0;
    //  _ctx.data.out_mecanum_torque[1] = 0;
    //  // _ctx.data.out_mecanum_torque[2] = 0;
    //  _ctx.data.out_mecanum_torque[3] = 0;
}

void hybrid_chassis_t::_track_control()
{
    for (int i = 0; i < 2; i++)
    {
        _ctx.data.out_track_torque[i] = _ctx.pid.track_pid[i]->calculate(
            _ctx.data.target_track_rpm[i], _ctx.data.current_track_rpm[i]);
    }
    // _ctx.data.out_track_torque[0] = 0;
    // _ctx.data.out_track_torque[1] = 0;
}

void hybrid_chassis_t::_send_motor_command() const
{
    // 静态分频标志位，每次调用翻转一次，实现 1/2 频率
    static bool freq_div_flag = false;
    freq_div_flag             = !freq_div_flag;

    // 麦轮和履带：仅在 flag 为 true 时发送指令
    if (freq_div_flag)
    {
        for (int i = 0; i < 4; i++)
            _ctx.motor.mecanum[i]->send_torque(_ctx.data.out_mecanum_torque[i]);
        // _ctx.motor.mecanum[0]->send_torque(_ctx.data.out_mecanum_torque[0]);
        // _ctx.motor.mecanum[1]->send_torque(0);
        // _ctx.motor.mecanum[2]->send_torque(0);
        // _ctx.motor.mecanum[3]->send_torque(0);

        for (int i = 0; i < 2; i++)
            _ctx.motor.track[i]->send_torque(_ctx.data.out_track_torque[i]);
        // for (int i = 0; i < 4; i++)
        //     _ctx.motor.mecanum[i]->send_torque(0);
        // for (int i = 0; i < 2; i++)
        //     _ctx.motor.track[i]->send_torque(0);
    }
    // 腿部电机：保持原频率控制 (VMC 和腿长控制通常需要高频以维持稳定性)
    for (int i = 0; i < 2; i++)
        _ctx.motor.leg[i]->send_torque(_ctx.data.out_leg_torque[i]);
}
// =========================================================
// 核心运行时与状态机
// =========================================================

void hybrid_chassis_t::_calibrate_leg_offsets()
{
    if (!_ctx.motor.leg[0] || !_ctx.motor.leg[1])
    {
        return;
    }

    LEFT_LEG_OFFSET_RAD  = _ctx.motor.leg[0]->get_current_position();
    RIGHT_LEG_OFFSET_RAD = _ctx.motor.leg[1]->get_current_position();
    _ctx.data.current_leg_rad[0] = 0.0f;
    _ctx.data.current_leg_rad[1] = 0.0f;
    _ctx.data.target_leg_rad[0]  = 0.0f;
    _ctx.data.target_leg_rad[1]  = 0.0f;
}

void hybrid_chassis_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (_ctx.cmd->leg_calibration != _last_leg_calibration_flag)
    {
        _last_leg_calibration_flag = _ctx.cmd->leg_calibration;
        _calibrate_leg_offsets();
    }

    _supercap_control();

    if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);
    else if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}


} // namespace pyro
