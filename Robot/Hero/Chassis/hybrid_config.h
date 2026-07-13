#ifndef __CONFIG_H__
#define __CONFIG_H__
#include "cstdint"
#include "BMI088_driver.h"

constexpr float TRACK_SPACING             = 0.456f; // 履带中心距 (m)
constexpr float MEC_WHEELBASE             = 0.45f;  // 麦轮轴距
constexpr float MEC_FRONT_WHEELBASE       = 0.225f;
constexpr float MEC_REAR_WHEELBASE        = MEC_WHEELBASE - MEC_FRONT_WHEELBASE;
constexpr float MEC_FRONT_TRACK_WIDTH     = 0.3f;   // 麦轮前轮距
constexpr float MEC_REAR_TRACK_WIDTH      = 0.48f;  // 麦轮后轮距
constexpr float WHEEL_RADIUS              = 0.076f; // 轮子半径 (m)
constexpr float TRACK_RADIUS              = 0.025f; // 履带驱动半径 (m)
constexpr float LEG_MIN_POS               = 0.15f;  // 腿部最小位置 (rad)
constexpr float LEG_MAX_POS               = 1.64f;  // 腿部最大位置
constexpr float LEG_LENGTH_MIN_POS        = -0.25f; // 腿部最小伸展位置(rad)
constexpr float LEG_LENGTH_MAX_POS        = 1.64f;  // 腿部最大伸展位置(rad)
constexpr float LEG_POS_BUFFER_RAD        = 0.1f;   // 腿部位置缓冲 (rad)
constexpr float LEG_LENGTH_POS_BUFFER_RAD = 0.1f;   // 腿部长度位置缓冲 (rad)
constexpr float LEG_MAX_TORQUE            = 25.0f;  // 腿部最大输出扭矩 (N*m)
constexpr float LEG_K_WALL                = 300.0f; // 虚拟墙弹性系数 (N*m/rad)
constexpr float LEG_D_WALL                = 20.0f; // 虚拟墙阻尼系数 (N*m*s/rad)
constexpr float LEG_GRA_K_WALL            = 50.0f; // 自重补偿虚拟墙 (N*m/rad)
constexpr float LEG_GRA_D_WALL            = 10.0f;
constexpr float YAW_OFFSET_RAD            = -0.892776966f;
constexpr float PSEUDO_GYRO_YAW_AMPLITUDE_RAD = 1.4f;
constexpr float PSEUDO_GYRO_YAW_SPEED_RADPS   = 4.0f;
constexpr float PSEUDO_GYRO_PHASE_RADPS =
    PSEUDO_GYRO_YAW_SPEED_RADPS / PSEUDO_GYRO_YAW_AMPLITUDE_RAD;
// IMU 机械安装偏移补偿 (需将底盘置于绝对水平面上读取原始数据填入)
constexpr float PITCH_OFFSET_RAD =
    -0.0675839931; // 示例值，正负号取决于你的 IMU 坐标系方向
constexpr float NORMAL_PITCH =
    0.1f; // 机器人正常站立时的俯仰角 (rad)，用于自重补偿的基准点
constexpr float ROLL_OFFSET_RAD                  = -0.0f;
constexpr uint32_t JX_POLY_DEGREE                = 7; // JX 多项式拟合阶数
constexpr uint32_t JY_POLY_DEGREE                = 5; // JY 多项式拟合阶数
constexpr uint32_t XB_POLY_DEGREE                = 6; // XB 多项式拟合阶数
constexpr uint32_t YB_POLY_DEGREE                = 4; // YB 多项式拟合阶数
constexpr uint32_t TAU_GRAVITY_DEGREE            = 4; // 自重补偿多项式阶数
constexpr float JX_POLY_COEF[JX_POLY_DEGREE + 1] = {
    -0.0742, 0.3986, -0.8740, 0.9990,
    -0.6561, 0.2485, -0.0279, -0.0072}; // JX多项式拟合系数，单位m/rad
constexpr float JY_POLY_COEF[JY_POLY_DEGREE + 1] = {
    -0.0194, 0.1161,  -0.2662,
    0.3140,  -0.2409, 0.2048}; // JY多项式拟合系数，单位m/rad
constexpr float XB_POLY_COEF[XB_POLY_DEGREE + 1] = {
    -0.0086, 0.0379,  -0.0700, 0.0618,
    -0.0154, -0.0062, -0.0011}; // XB多项式拟合系数，单位m
constexpr float YB_POLY_COEF[YB_POLY_DEGREE + 1] = {
    -0.0056, 0.0311, -0.0793, 0.1957, 0.0745}; // YB多项式拟合系数，单位m
constexpr float TAU_GRAVITY_COEF[TAU_GRAVITY_DEGREE + 1] = {
    -1.4132, 4.2032, -5.0430, 3.7907, -3.3777};
constexpr float K_TAU_GRAVITY = 1.0f;    // 自重补偿系数
constexpr float MASS          = 25.5f;   // 机器人质量 (kg)
constexpr float GRAVITY       = gNORM;   // 重力加速度 (m/s^2)
constexpr float DIST_FRONT    = 0.2395f; // 质心到前轴距离 (m)
constexpr float DIST_HIP      = 0.1805f; // 质心到髋关节的水平距离
constexpr float H_COG         = 0.18f;   // 重心垂直高度
constexpr float H_HIP_OFFSET  = 0.074f;  // 髋关节与前轮轴心的垂直落差
inline float LEFT_LEG_OFFSET_RAD =
    0.984015703f; // 左腿位置偏移 (rad)，正值表示向前偏移
inline float RIGHT_LEG_OFFSET_RAD =
    -1.1565913f; // 右腿位置偏移 (rad)，正值表示向前偏移 // 变了0.1

// ================= 新增：自动越障收腿配置参数 =================
constexpr float CLIMB_DIST_LPF_ALPHA =
    0.03f; // 二阶滤波响应较快，Alpha 取 0.1f 是一个很好的平衡点
constexpr float CLIMB_DIST_MAX_JUMP =
    35.0f; // 变化率限幅：单帧测距允许的最大跳变绝对值 (mm)
constexpr uint16_t FRONT_DISTANCE_OFFSET = 38;  // 前测距模块平地偏置
constexpr uint16_t BACK_DISTANCE_OFFSET  = 116; // 后测距模块平地偏置
constexpr uint16_t CLIMB_DIST_THRES_HIGH = 175; // 导轮悬空距离上限判定 (mm)
constexpr uint16_t CLIMB_DIST_THRES_LOW  = 25;  // 导轮搭上台阶距离下限判定 (mm)
constexpr float CLIMB_VEL_X_FRONT_THRES =
    0.1f; // 触发自动收腿的底盘最小真实前推速度 (m/s)
constexpr float CLIMB_VEL_X_BACK_THRES         = 0.15f; // 倒车保护判定速度
constexpr uint32_t CLIMB_RETRACT_TIMEOUT_TICKS = 3000;  // 超时放弃收腿

#endif
