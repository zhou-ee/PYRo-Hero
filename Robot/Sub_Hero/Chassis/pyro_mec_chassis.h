#ifndef __PYRO_MEC_CHASSIS_H__
#define __PYRO_MEC_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_kin_mec.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "pyro_power_control.h"
#include "pyro_powermeter.h"
#include "pyro_supercap_drv.h"

namespace pyro
{

struct mec_cmd_t final : public cmd_base_t
{
    float vx;
    float vy;
    float wz;

    mec_cmd_t() : vx(0.0f), vy(0.0f), wz(0.0f)
    {
    }
};

struct mec_deps_t
{
    struct motor_deps_t
    {
        motor_base_t *wheels[4]{nullptr}; // FL, FR, BL, BR
        motor_base_t *yaw{nullptr};
    };

    struct pid_deps_t
    {
        pid_t *wheel_pid[4]{nullptr};
        pid_t *follow_yaw_pid{nullptr};
    };

    motor_deps_t motor_deps{};
    pid_deps_t pid_deps{};
};

class mec_chassis_t final
    : public module_base_t<mec_chassis_t, mec_cmd_t, mec_deps_t>
{
    friend class module_base_t<mec_chassis_t, mec_cmd_t, mec_deps_t>;
    friend class jcom_drv_t;

    struct data_ctx_t;
    struct mec_context_t;

  public:
    mec_chassis_t(const mec_chassis_t &)            = delete;
    mec_chassis_t &operator=(const mec_chassis_t &) = delete;

    [[nodiscard]] mec_context_t &get_ctx();

  private:
    mec_chassis_t();
    ~mec_chassis_t() override = default;

    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    void _power_control_init();
    void _supercap_control();
    void _power_control();
    void _kinematics_solve();
    void _mecanum_control();
    void _send_motor_command() const;

    mecanum_kin_t *_kinematics{nullptr};

    struct data_ctx_t
    {
        float current_yaw_error{0.0f};

        bool wheel_online[4]{};
        float current_wheel_rpm[4]{};
        float current_wheel_torque[4]{};
        float current_wheel_temp[4]{};
        float target_wheel_rpm[4]{};
        float out_wheel_torque[4]{};

        float real_vx{0.0f};
        float real_vy{0.0f};
        float real_wz{0.0f};

        float total_predicted_power{0.0f};
        float buffer_energy{0.0f};
    };

    struct mec_context_t
    {
        mec_deps_t::motor_deps_t motor;
        mec_deps_t::pid_deps_t pid;
        data_ctx_t data;

        mec_cmd_t *cmd{nullptr};
        powermeter_drv_t *powermeter{nullptr};
        powermeter_data powermeter_feedback{};
        supercap_drv_t::chassis_cmd_t supercap_cmd{};
        supercap_drv_t::cap_feedback_t cap_feedback{};
        power_node_t *power_motor_data[4]{};
    };

    mec_context_t _ctx;

    using owner = mec_chassis_t;

    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct state_active_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    state_passive_t _state_passive;
    state_active_t _state_active;
    fsm_t<owner> _main_fsm;
};

} // namespace pyro

#endif // __PYRO_MEC_CHASSIS_H__
