#ifndef __PYRO_HYBRID_CHASSIS_H__
#define __PYRO_HYBRID_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_kin_hybrid.h"
#include "pyro_motor_base.h"
#include "pyro_ins.h" // 新增 IMU 依赖
#include "hybrid_config.h"
#include "pyro_power_control.h"
#include "pyro_powermeter.h"
#include "pyro_supercap_drv.h"

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct hybrid_cmd_t : cmd_base_t
{
    float vx;          // 云台坐标系下的 X 轴速度 m/s (推前)
    float vy;          // 云台坐标系下的 Y 轴速度 m/s (推左)
    float wz;          // z轴角速度 rad/s (通常跟随模式下该值为0，除非做小陀螺)
    float delta_pitch; // 腿部目标位置相对于当前的增量 rad
    float delta_yaw;
    bool leg_calibration;
    bool pseudo_gyro_en;
    bool crossing_en; // 是否启用履带 (true: 履带 + 麦轮混合驱动, false: 仅麦轮)
    bool leg_retract; // 是否进入腿部收回状态 (仅在 crossing_en=true 时有效)

    hybrid_cmd_t()
        : vx(0), vy(0), wz(0), delta_pitch(0), delta_yaw(0),
          leg_calibration(false), pseudo_gyro_en(false), crossing_en(false),
          leg_retract(false)
    {
    }
};

struct hybrid_deps_t
{
    // 电机句柄
    struct motor_deps_t
    {
        motor_base_t *mecanum[4]{nullptr};
        motor_base_t *track[2]{nullptr};
        motor_base_t *leg[2]{nullptr};
        motor_base_t *yaw{nullptr};
    };

    // 算法对象
    struct pid_deps_t
    {
        pid_t *mecanum_pid[4]{nullptr};
        pid_t *follow_yaw_pid{nullptr};
        pid_t *track_pid[2]{nullptr};
        pid_t *pitch_pid{nullptr};
        pid_t *roll_pid{nullptr};

        pid_t *leg_pos_pid[2]{nullptr}; // 腿部位置控制 PID
        pid_t *leg_vel_pid[2]{nullptr}; // 腿部速度控制 PID
    };

    motor_deps_t motor_deps{};
    pid_deps_t pid_deps{};
};

// =========================================================
// 2. 混合底盘类
// =========================================================
class hybrid_chassis_t final
    : public module_base_t<hybrid_chassis_t, hybrid_cmd_t, hybrid_deps_t>
{
    friend class module_base_t<hybrid_chassis_t, hybrid_cmd_t, hybrid_deps_t>;

    friend class jcom_drv_t;

    struct motor_deps_t;
    struct pid_deps_t;
    struct data_ctx_t;
    struct hybrid_context_t;

  public:
    hybrid_chassis_t(const hybrid_chassis_t &)            = delete;
    hybrid_chassis_t &operator=(const hybrid_chassis_t &) = delete;

    hybrid_context_t& get_ctx();

  private:
    hybrid_chassis_t();
    ~hybrid_chassis_t() override = default;

    // --- 基类接口 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 派生方法 ---
    void _power_control_init();
    void _kinematics_solve();
    void _supercap_control();
    void _power_control();
    void _mecanum_control();
    void _track_control();
    void _leg_vmc();
    void _leg_length_control();
    void _calibrate_leg_offsets();
    void _send_motor_command() const;
    hybrid_kin_t *_kinematics{nullptr};

    // 运行时数据
    struct data_ctx_t
    {
        bool wheel_online[4]{};
        float current_wheel_rpm[4]{};
        float current_track_rpm[2]{};
        float current_leg_rad[2]{};
        float current_leg_radps[2]{};

        // IMU 姿态反馈
        float current_pitch_rad{0};
        float current_roll_rad{0};
        float current_yaw_rad{0};
        float target_pitch_rad{0};
        float target_yaw_rad{0};

        // 逆解算速度
        float real_vx{0};
        float real_vy{0};
        float real_wz{0};

        // 测距模块原始反馈
        int32_t front_distance_mm{0};
        int32_t back_distance_mm{0};

        // --- 新增：测距 2阶 LPF 状态变量 ---
        float filtered_front_distance{0.0f};
        float filtered_back_distance{0.0f};
        float front_lpf_state[2]{0.0f, 0.0f}; // 数组大小改为 2
        float back_lpf_state[2]{0.0f, 0.0f};
        bool  distance_lpf_initialized{false};

        // YAW 电机差值反馈（用于底盘跟随云台）
        float current_yaw_error{0};
        float pseudo_gyro_phase_rad{0};
        float pseudo_gyro_target_yaw_rad{0};
        uint32_t pseudo_gyro_last_tick{0};
        bool pseudo_gyro_active{false};
        float target_wheel_rpm[4]{};
        float target_track_rpm[2]{};
        float target_leg_rad[2]{};
        float target_leg_radps[2]{};

        // 输出
        float out_mecanum_torque[4]{};
        float out_track_torque[2]{};
        float out_leg_torque[2]{};

        // 电机功率相关反馈
        float current_mecanum_torque[4]{};
        float current_mecanum_temp[4]{};
        float current_track_torque[2]{};
        float current_track_temp[2]{};
        float total_predicted_power{};
        float buf_energy{};
    };

    struct hybrid_context_t
    {
        hybrid_deps_t::motor_deps_t motor;
        hybrid_deps_t::pid_deps_t pid;
        data_ctx_t data;
        powermeter_drv_t *powermeter{nullptr};
        powermeter_data powermeter_feedback{};
        supercap_drv_t::chassis_cmd_t supercap_cmd{};
        supercap_drv_t::cap_feedback_t cap_feedback{};
        power_node_t *power_motor_data[6]{};
        hybrid_cmd_t *cmd{};
    };

    // 总 Context
    hybrid_context_t _ctx;
    bool _last_leg_calibration_flag{false};

    // =====================================================
    // 状态定义 (HFSM)
    // =====================================================
    using owner = hybrid_chassis_t;

    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct fsm_active_t final : public fsm_t<owner>
    {
        struct cruising_state_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct climbing_fsm_t final : public fsm_t<owner>
        {
            struct track_climbing_state_t final : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;
            };

            struct leg_retraction_state_t final : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;
            };

            void on_enter(owner *owner) override;
            void on_execute(owner *owner) override;
            void on_exit(owner *owner) override;

          private:
            track_climbing_state_t track_climbing_state;
            leg_retraction_state_t leg_retraction_state;

            // 自动收腿控制标志位
            bool _auto_retract_flag{false};
            uint32_t _retract_hold_tick{0};
        };

        // FSM Hooks
        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

      private:
        cruising_state_t cruising_state;
        climbing_fsm_t climbing_fsm;
    };

    // 状态实例
    state_passive_t _state_passive;
    fsm_active_t _state_active;
    fsm_t<owner> _main_fsm;
};

} // namespace pyro

#endif
