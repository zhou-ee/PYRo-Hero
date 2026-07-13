#ifndef __CONFIG_H__
#define __CONFIG_H__
#include <cstdint>
#include "BMI088_driver.h"

// =========================================================
// 丝杠机构运动学参数 (由 SolidWorks 测算)
// L1 = 80.0mm, L2 = 106.8mm
// =========================================================
constexpr float SCREW_L1_SQ_PLUS_L2_SQ   = 17806.24f;  // 80^2 + 106.8^2
constexpr float SCREW_TWO_L1_L2          = 17088.0f;   // 2 * 80 * 106.8
constexpr float SCREW_THETA_ZERO_RAD     = 1.2353835f; // 水平时的推杆内部夹角
constexpr float SCREW_S_LIMIT_BOTTOM     = 128.0424f;  // 丝杠最底端初始长度 (mm)
constexpr float SCREW_MAX_TORQUE         = 20.0f;      // 电机最大扭矩

// =========================================================
// 校准相关参数 (Tick 数)
// =========================================================
constexpr uint32_t PITCH_CALIB_DELAY_TICKS = 1000; // 上电校准前等待稳定时间
constexpr uint32_t PITCH_CALIB_MAX_TICKS   = 1000; // 采集均值的持续时间
constexpr uint32_t DYNAMIC_CALIB_WINDOW_TICKS = 500; // 动态校准的观察窗口期

// =========================================================
// 动态校准触发阈值 (防零漂判定条件)
// =========================================================
inline float CALIB_PITCH_STILL_RADPS = 0.03f; // Pitch IMU 角速度极小判定
inline float CALIB_ROLL_LEVEL_RAD    = 0.05f; // Roll 水平判定
inline float CALIB_PITCH_ERROR_RAD   = 0.02f; // 当前与目标误差极小判定
inline float CALIB_MOTOR_STILL_RADPS = 0.03f; // Pitch 电机角速度极小判定

// =========================================================
// 摩擦力前馈与死区参数
// =========================================================
constexpr float PITCH_FRICTION_REF_ANGLE_RAD  = -0.1f; // 标定参考角
constexpr float PITCH_FRICTION_REF_TORQUE     = 3.0f;  // 标定参考力矩
constexpr float PITCH_DEADBAND_NORMAL_RADPS   = 0.005f; // 常规模式速度死区
constexpr float PITCH_BUFFER_NORMAL_RADPS   = 0.003f; // 常规模式速度缓冲区
constexpr float PITCH_DEADBAND_AUTOAIM_RADPS  = 0.05f; // 自瞄模式速度死区 (抗IMU噪声)
constexpr float PITCH_BUFFER_AUTOAIM_RADPS  = 0.04f; // 自瞄模式速度缓冲区

// 【弃用】以下使用施密特触发器的硬编码摩擦力导致了欠阻尼震荡
// constexpr float YAW_SLING_FRICTION_TORQUE     = 0.12f;
// constexpr float YAW_SLING_DEADBAND_RADPS      = 0.004f;
// constexpr float YAW_SLING_BUFFER_RADPS        = 0.004f;


constexpr float YAW_SLING_TORQUE_LIMIT        = 3.0f;  // Yaw 输出限幅

// =========================================================
// 限位与偏置
// =========================================================
constexpr float PITCH_MIN_RELATIVE_RAD   = -0.72f; // Pitch 轴最小角度 (rad)
constexpr float PITCH_MAX_RELATIVE_RAD   = -0.1f;  // Pitch 轴最大角度 (rad)
constexpr float PITCH_MIN_IMU_RAD        = -0.85f;  // IMU 读数最小角度 (rad)
constexpr float PITCH_MAX_IMU_RAD        = -0.2f;  // IMU 读数最大角度 (rad)

constexpr float GIMBAL_CROSSING_PITCH_RAD = -0.15f;

constexpr float YAW_OFFSET_RAD           = -0.892776966f;
constexpr float YAW_MIN_RELATIVE_RAD     = -1.5f;  // Yaw 轴相对最小角度(待调整)
constexpr float YAW_MAX_RELATIVE_RAD     = 1.5f;   // Yaw 轴相对最大角度(待调整)

#endif
