#ifndef DUAL_YAW_AIM_H
#define DUAL_YAW_AIM_H

#include <stdint.h>

/* 修改：静止双Yaw分配使用ID2编码器角度，逆时针为正。 */
#define DUAL_YAW_SMALL_CENTER_DEG 159.39f
#define DUAL_YAW_SMALL_MIN_DEG    125.0f
#define DUAL_YAW_SMALL_MAX_DEG    212.0f

typedef enum
{
    DUAL_YAW_AIM_OFF = 0,
    DUAL_YAW_AIM_TRACK = 1,
    DUAL_YAW_AIM_BRAKE = 2,
    DUAL_YAW_AIM_HOLD = 3
} dual_yaw_aim_state_t;

typedef struct
{
    dual_yaw_aim_state_t state;
    uint8_t initialized;
    uint8_t has_target_frame;
    uint32_t applied_frame;
    float camera_target_deg;
    float error_deg;
    float total_rpm;
    float big_ref_rpm;
    float small_ref_rpm;
    float filtered_big_rpm;
    /* 修改：限位时允许提高大Yaw分担，独立保存正常低通状态。 */
    float big_lowpass_rpm;
} dual_yaw_aim_t;

void Dual_Yaw_Aim_Reset(dual_yaw_aim_t *state);

/* 修改：角度为deg、速度为rpm；视觉误差须由调用方统一为逆时针正。
 * fresh表示目标仍新鲜，frame_count仅在收到完整有效视觉帧时递增。
 * 调用方在退出自瞄、反馈失效或模式切换时Reset；本模块不直接驱动电机。
 */
void Dual_Yaw_Aim_Update(dual_yaw_aim_t *state,
                         uint8_t fresh,
                         uint32_t frame_count,
                         float camera_yaw_deg,
                         float small_angle_deg,
                         float big_actual_rpm,
                         float signed_yaw_error_deg,
                         float angle_kp_rpm_per_deg,
                         float max_small_rpm,
                         float dt_s);

/* 修改：左中手动跟随，目标是相机世界航向；松杆后仍持续让ID2回中。
 * 底盘、ID1实际转速均为逆时针正，ID2补足相机合成速度。
 */
void Dual_Yaw_Follow_Update(dual_yaw_aim_t *state,
                            float camera_target_deg,
                            float camera_yaw_deg,
                            float small_angle_deg,
                            float big_actual_rpm,
                            float chassis_actual_rpm,
                            float angle_kp_rpm_per_deg,
                            float max_small_rpm,
                            float dt_s);

/* 修改：左中视觉保持时继续回中，只重新分配速度，不改变相机目标及状态。
 * 刹停阶段不回中，由调用方在HOLD后使用；此入口不参与旋转中视觉。
 */
void Dual_Yaw_Aim_RecenterHold(dual_yaw_aim_t *state,
                               float small_angle_deg,
                               float angle_kp_rpm_per_deg,
                               float max_small_rpm);

#endif
