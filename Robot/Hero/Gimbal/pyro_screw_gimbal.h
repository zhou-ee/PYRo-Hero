#ifndef __PYRO_SCREW_GIMBAL_H__
#define __PYRO_SCREW_GIMBAL_H__

#include "pyro_algo_pid.h"
#include "pyro_algo_leso.h" // 【新增】引入 LESO 观测器
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct screw_gimbal_cmd_t final : public cmd_base_t
{
    enum class sling_preaim_source_t : uint8_t
    {
        NONE = 0,
        FIXED_DELTA,
        CHASSIS_COORD
    };

    float pitch_delta_angle;
    float yaw_delta_angle;

    bool trigger_calibration;
    bool sling_mode;
    bool sling_pitch_flag;
    uint32_t sling_preaim_seq;
    sling_preaim_source_t sling_preaim_source;
    bool autoaim_mode;
    bool track_en;
    float target_pitch;
    float target_yaw;

    screw_gimbal_cmd_t()
        : pitch_delta_angle(0.0f), yaw_delta_angle(0.0f),
          trigger_calibration(false), sling_mode(false),
          sling_pitch_flag(false), sling_preaim_seq(0),
          sling_preaim_source(sling_preaim_source_t::NONE),
          autoaim_mode(false), track_en(false), target_pitch(0.0f),
          target_yaw(0.0f)
    {
    }
};

struct screw_gimbal_deps_t
{
    struct motor_deps_t
    {
        motor_base_t *pitch{nullptr};
        motor_base_t *yaw{nullptr};
    };

    struct pid_deps_t
    {
        pid_t *pitch_pos{nullptr};
        pid_t *pitch_spd{nullptr};

        // 自瞄专用 PID 参数
        pid_t *pitch_auto_pos{nullptr};
        pid_t *pitch_auto_spd{nullptr};

        pid_t *yaw_pos{nullptr};
        pid_t *yaw_spd{nullptr};
        pid_t *yaw_relative_pos{nullptr};
        pid_t *yaw_relative_spd{nullptr};

        // 【新增】吊射模式下用于 Yaw 轴机械角前馈补偿的 LESO
        leso_t<3> *yaw_pos_leso{nullptr};
        leso_t<2> *yaw_spd_leso{nullptr};

        leso_t<3> *yaw_pos_imu_leso{nullptr};
        leso_t<2> *yaw_spd_imu_leso{nullptr};
    };

    motor_deps_t motor_deps{};
    pid_deps_t pid_deps{};
};

// =========================================================
// 2. 云台类
// =========================================================
class screw_gimbal_t final
    : public module_base_t<screw_gimbal_t, screw_gimbal_cmd_t,
                           screw_gimbal_deps_t>
{
    friend class module_base_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct gimbal_context_t;

  public:
    [[nodiscard]] gimbal_context_t& get_ctx();

  private:
    screw_gimbal_t();
    ~screw_gimbal_t() override = default;

    // --- 基类接口实现 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 私有辅助方法 ---
    void _gimbal_control();
    void _gimbal_autoaim_control();
    void _gimbal_sling_control();
    static void _send_motor_command(gimbal_context_t *ctx);
    void _communicate_chassis();
    void _calculate_relative_angles();
    void _handle_dynamic_calibration();
    void _apply_yaw_relative_limit();

    // --- 核心运动学 (纯数学模型，需外部传入任意角度解算) ---
    bool _calibrate_pitch_offset();
    float _pitch_rad_to_motor_rad(float pitch_rad) const;
    float _motor_rad_to_pitch_rad(float motor_rad) const;
    float _get_motor_to_pitch_jacobian(float pitch_rad) const;

    // --- 业务控制逻辑 (无参化，完全依赖 _ctx.data 内部状态) ---
    float _calculate_pitch_compensation(bool is_autoaim) const;

    // --- 成员变量 ---

    uint32_t _calib_tick{0};
    float _calib_pitch_sum{0.0f};

    // 动态校准计时与均值缓存
    uint32_t _dynamic_calib_timer{0};
    float _dynamic_calib_sum{0.0f};

    // 运行时数据
    struct data_ctx_t
    {
        bool is_calibrating{false};
        bool has_initial_calibrated{false};
        bool allow_dynamic_calib{true}; // 是否允许动态校准

        float pitch_motor_upper_limit{0};
        float pitch_motor_lower_limit{0};

        // --- Pitch 电机增量套圈处理变量 ---
        float last_pitch_rotor_rad{0};
        float total_pitch_motor_rad{0};

        // --- 反馈 (含 Offset) ---
        float current_pitch_motor_rad{0};
        float current_pitch_motor_radps{0};
        float current_pitch_motor_world{0};

        // 当前姿态下的传动比缓存 (dMotor / dPitch)
        float current_jacobian{0};

        // 姿态反馈 (基于 IMU)
        float pitch_imu_rad{0};
        float pitch_imu_radps{0};
        float yaw_imu_rad{0};
        float yaw_imu_radps{0};
        float roll_imu_rad{0};
        float roll_imu_radps{0};

        // 加速度反馈 (基于 IMU)
        float z_accel_imu{0};
        float x_accel_imu{0};
        float y_accel_imu{0};

        // 四元数与相对角反馈
        float current_chassis_pitch_rad{0};
        float chassis_yaw_imu{0};
        float chassis_pitch_rad{0};
        float chassis_q[4]{};
        float gimbal_q[4]{};
        float relative_pitch_rad{0};
        float relative_roll_rad{0};
        float relative_yaw_motor_rad{0};
        float relative_yaw_motor_wrapped_rad{0};
        float relative_yaw_motor_radps{0};

        // 目标与误差
        float target_pitch_rad{0};
        float target_pitch_radps{0};
        float target_yaw_rad{0};
        float target_yaw_radps{0};
        float yaw_error_rad{0};

        float target_relative_yaw_rad{0};
        float relative_yaw_error_rad{0};

        // 输出
        float out_pitch_torque{0};
        float out_yaw_torque{0};

        // 调试用
        float pos_imu_leso_z1;
        float spd_imu_leso_z0;
        float pos_leso_z0;
        float pos_leso_z1;
        float pos_leso_out;
        float spd_leso_z0;
    };

    // 总 Context
    struct gimbal_context_t
    {
        screw_gimbal_deps_t::motor_deps_t motor;
        screw_gimbal_deps_t::pid_deps_t pid;
        data_ctx_t data;
        screw_gimbal_cmd_t *cmd{};
    };

    gimbal_context_t _ctx;

    // =====================================================
    // 状态定义 (HFSM)
    // =====================================================
    using owner = screw_gimbal_t;

    struct fsm_passive_t : public fsm_t<owner>
    {
        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

        struct calibration_state_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct idle_state_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

      private:
        calibration_state_t calibration_state;
        idle_state_t idle_state;
        bool _last_calib_flag{false};
    };

    struct fsm_active_t : public fsm_t<owner>
    {
        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

        struct sling_state_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct normal_state_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct autoaim_state_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

      private:
        sling_state_t sling_state;
        autoaim_state_t autoaim_state;
        normal_state_t normal_state;
    };

    fsm_passive_t _fsm_passive;
    fsm_active_t _fsm_active;
    fsm_t<owner> _main_fsm;

    // =====================================================
    // 静态配置 (编译器常量)
    // =====================================================
    static constexpr float PITCH_OFFSET_RAD = 0.0f;
};

} // namespace pyro

#endif
