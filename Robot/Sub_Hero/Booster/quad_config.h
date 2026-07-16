#ifndef __QUAD_CONFIG_H__
#define __QUAD_CONFIG_H__

constexpr float FRIC1_RADIUS = 0.04f; // 摩擦轮半径 (m)
constexpr float FRIC2_RADIUS = 0.03f; // 摩擦轮半径 (m)
constexpr float FRIC1_ANTI_JAM_REVERSE_TORQUE = 2.0f;

// constexpr float TRIGGER_OFFSET = 0.334f;

constexpr float TRIGGER_OFFSET = -1.0f; //越小越紧
constexpr float TRIGGER_PRESET_MIN_ADVANCE_RAD = 0.15f;
constexpr float TRIGGER_PRESET_DEFORM_THRESHOLD_RAD = 0.15f;
constexpr float TRIGGER_LOCATED_THRESHOLD_RAD = 0.15f;

constexpr float TRIGGER_FEED_DIR = 1.0f;

constexpr float kDrag    = 0.0113307845f; // 二次空气阻力系数，需随弹丸和场地标定
constexpr float kGravity = 9.85534603f;   // 重力加速度，单位 m/s^2

constexpr float kDenominatorMin = 1.0e-6f; // 积分分母下限，避免接近奇点时除零
constexpr float kJacobianStep   = 1.0e-4f; // 有限差分步长，过小易受浮点噪声影响
constexpr float kSingularEpsilon =
    1.0e-6f; // 雅可比行列式阈值，小于该值视为不可逆
constexpr float kSolveTolerance = 1.0e-2f; // 残差收敛阈值，单位约等于米
constexpr int kIntegralSteps    = 60;      // Simpson 积分分段数，需为偶数
constexpr int kMaxIterations    = 30; // Newton 最大迭代次数，防止异常输入卡死

constexpr float kSlopeGapMin    = 1.0e-5f;
constexpr float kSlopeLimit     = 50.0f;
constexpr int kLineSearchSteps  = 8;

#endif

