#ifndef __QUAD_CONFIG_H__
#define __QUAD_CONFIG_H__

constexpr float FRIC1_RADIUS = 0.04f; // 摩擦轮半径 (m)
constexpr float FRIC2_RADIUS = 0.03f; // 摩擦轮半径 (m)
constexpr float FRIC1_ANTI_JAM_REVERSE_TORQUE = 4.0f;

// constexpr float TRIGGER_OFFSET = 0.334f;
constexpr float TRIGGER_OFFSET = 0.1f;
//越小越紧
constexpr float TRIGGER_PRESET_MIN_ADVANCE_RAD = 0.15f;
constexpr float TRIGGER_PRESET_DEFORM_THRESHOLD_RAD = 0.15f;
constexpr float TRIGGER_LOCATED_THRESHOLD_RAD = 0.1f;
constexpr float TRIGGER_FEED_DIR = -1.0f;

#endif

