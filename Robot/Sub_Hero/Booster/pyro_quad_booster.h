#ifndef __PYRO_QUAD_BOOSTER_H__
#define __PYRO_QUAD_BOOSTER_H__

#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_module_base.h"
#include <cmath>

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct quad_booster_cmd_t final : public cmd_base_t
{
    bool fric_on;  // 摩擦轮开启
    bool anti_jam; // 防卡弹：一级摩擦轮恒力矩反转，拨弹盘失能
    bool force_deploy;
    uint8_t reset_count;
    uint8_t shoot_data_reset_count;

    uint8_t fire_count; // 拨弹计数器，替代 fire_enable

    float trig_target_spd; // 新增：拨弹盘目标速度

    quad_booster_cmd_t()
        : fric_on(false), anti_jam(false), force_deploy(false), reset_count(0),
          shoot_data_reset_count(0), fire_count(0), trig_target_spd(0.0f)
    {
    }
};

struct quad_deps_t
{
    struct motor_deps_t
    {
        motor_base_t *fric_wheels[4]{nullptr};
        motor_base_t *trigger_wheel{nullptr};
    };

    struct pid_deps_t
    {
        pid_t *fric_pid[4]{nullptr};
        pid_t *trigger_pos_pid{nullptr};
        pid_t *trigger_spd_pid{nullptr};
        pid_t *ball_speed_pid{nullptr};
    };

    motor_deps_t motor_deps;
    pid_deps_t pid_deps;
};

// =========================================================
// 2. 四轮发射机构类
// =========================================================
class quad_booster_t final
    : public module_base_t<quad_booster_t, quad_booster_cmd_t, quad_deps_t>
{
    friend class module_base_t<quad_booster_t, quad_booster_cmd_t, quad_deps_t>;
    friend class jcom_drv_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct booster_ctx_t;

  public:
    quad_booster_t(const quad_booster_t &)            = delete;
    quad_booster_t &operator=(const quad_booster_t &) = delete;
    [[nodiscard]] booster_ctx_t &get_ctx();

  private:
    quad_booster_t();
    ~quad_booster_t() override = default;

    // --- 接口实现 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 内部辅助 ---
    void _speed_control();
    void _fric_control();
    void _anti_jam_control();
    void _trigger_position_control();
    void _trigger_speed_control();
    void _send_fric_command() const;
    void _send_raw_fric_command() const;
    void _send_trigger_command() const;
    void _launch_delay_calculate();
    void _reset_active_shoot_data();
    [[nodiscard]] bool _use_deploy_data() const;

    // 角度归一化辅助函数
    static float _normalize_angle(float angle);
    static bool _is_trigger_located(float trigger_rad);
    static float _get_next_trigger_preset(float trigger_rad,
                                          float min_advance_rad);

    // --- 成员变量 ---
    struct motor_ctx_t
    {
        motor_base_t *fric_wheels[4]{nullptr};
        motor_base_t *trigger_wheel{nullptr};
    };

    struct pid_ctx_t
    {
        pid_t *fric_pid[4]{nullptr};
        pid_t *trigger_pos_pid{nullptr};
        pid_t *trigger_spd_pid{nullptr};
        pid_t *ball_speed_pid{nullptr};
    };

    struct data_ctx_t
    {
        uint8_t internal_fire_count{0}; // 内部拨弹计数器追踪
        bool deploy_mode{false};
        bool trigger_located{false};
        bool anti_jam_active{false};
        bool ready_state_flag{false};

        bool fric_err{};
        uint8_t internal_reset_count{0};
        uint8_t internal_shoot_data_reset_count{0};

        float launch_delay_timer[3]{}; // 发射延时计时器
        float signal_timer{0};         // 信号持续时间计时器
        float avg_launch_delay{0};     // 平均发射延时
        uint32_t fresh_timer{0};       // 发弹延迟计算的刷新计时器
        uint8_t fire_count{0};

        // 反馈
        float abs_current_fric_mps[4]{}; // 绝对值，用于发弹延迟计算
        float current_fric_mps[4]{};
        float current_trig_radps{0};
        float current_trig_torque{0};
        float current_trig_rad{0}; // -PI ~ PI (归一化后的输出)

        // 目标
        float target_fric_mps[4]{};
        float target_trig_rad{0};
        float target_trig_radps{0};

        float target_shoot_speed{0};

        float current_fric_torque[4]{};
        // 输出
        float out_fric_torque[4]{};
        float out_trig_torque{0};
    };

    struct shoot_data_t
    {
        const float target_speed{};
        float fric1_mps{};
        float fric2_mps{};
        const float initial_fric1_mps = fric1_mps;
        const float initial_fric2_mps = fric2_mps;
        float ball_speed[3]{};
        float avg_ball_speed = target_speed;
        float real_ball_speed[8]{};
        float avg_real_ball_speed = target_speed;

        void reset()
        {
            fric1_mps = initial_fric1_mps;
            fric2_mps = initial_fric2_mps;
            for (float &speed : ball_speed)
            {
                speed = 0.0f;
            }
            for (float &speed : real_ball_speed)
            {
                speed = 0.0f;
            }
            avg_ball_speed      = target_speed;
            avg_real_ball_speed = target_speed;
        }
    };

    struct booster_ctx_t
    {
        quad_deps_t::motor_deps_t motor;
        quad_deps_t::pid_deps_t pid;
        data_ctx_t data;
        shoot_data_t shoot_normal_data{11.7f, 11.3f, 7.8f};
        shoot_data_t shoot_deploy_data{16.3f, 14.9f, 10.1f};
        quad_booster_cmd_t *cmd{};
    };

    booster_ctx_t _ctx;

    // =====================================================
    // 状态机定义
    // ========================================
    //
    //
    //
    //
    //
    // =============
    using owner = quad_booster_t;

    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;

      private:
        bool _trigger_stopped{false}; // 用于确保拨弹盘完全停止后发0
    };

    struct fsm_active_t final : public fsm_t<owner>
    {
        struct state_homing_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;

          private:
            float _homing_turnback_start_time{0.0f};
        };
        struct state_interim_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;

          private:
            float _ready_wait_start_time{0.0f};
            float _fric_ready_start_time{0.0f};
        };
        struct state_ready_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;

          private:
            float _fric_unready_start_time{0.0f};
        };
        struct state_busy_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_stall_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_anti_jam_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

      private:
        state_homing_t _homing_state;
        state_interim_t _interim_state;
        state_ready_t _ready_state;
        state_busy_t _busy_state;
        state_stall_t _stall_state;
        state_anti_jam_t _anti_jam_state;
    };
    struct state_temp_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };
    state_passive_t _state_passive;
    fsm_active_t _state_active;
    state_temp_t _state_temp;
    fsm_t<owner> _main_fsm;
};

} // namespace pyro
#endif
