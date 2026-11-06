#include "dual_yaw_aim.h"

#include <math.h>
#include <string.h>

/* 修改：ID2先响应相机误差，ID1缓慢接管持续转动并使ID2回到中位。 */
#define DUAL_YAW_ERROR_LIMIT_DEG       45.0f
#define DUAL_YAW_TOTAL_FILTER_TAU_S     0.01f
#define DUAL_YAW_BIG_FILTER_TAU_S       0.25f
#define DUAL_YAW_FEEDBACK_FILTER_TAU_S  0.01f
#define DUAL_YAW_CENTER_KP_RPM_PER_DEG  0.15f
#define DUAL_YAW_EDGE_KP_RPM_PER_DEG    0.40f
#define DUAL_YAW_EDGE_MARGIN_DEG        8.0f
#define DUAL_YAW_BIG_MAX_RPM           25.0f
#define DUAL_YAW_HOLD_MAX_RPM          10.0f
#define DUAL_YAW_TOTAL_BRAKE_RPM_S    800.0f
#define DUAL_YAW_BIG_BRAKE_RPM_S      150.0f

static float Dual_Yaw_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float Dual_Yaw_WrapDeg(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    else if (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float Dual_Yaw_Filter(float previous, float input, float dt_s, float tau_s)
{
    return previous + dt_s / (tau_s + dt_s) * (input - previous);
}

static float Dual_Yaw_ApproachZero(float value, float step)
{
    if (value > step)
    {
        return value - step;
    }
    if (value < -step)
    {
        return value + step;
    }
    return 0.0f;
}

static float Dual_Yaw_LimitSmall(float target_rpm,
                                  float small_angle_deg,
                                  float angle_kp_rpm_per_deg,
                                  float max_small_rpm)
{
    float positive_limit;
    float negative_limit;

    /* 修改：距机械边界越近，允许朝外的速度越低；越界后仍允许向内恢复。 */
    positive_limit = Dual_Yaw_Clamp(
        (DUAL_YAW_SMALL_MAX_DEG - small_angle_deg) * angle_kp_rpm_per_deg,
        0.0f,
        max_small_rpm);
    negative_limit = Dual_Yaw_Clamp(
        (small_angle_deg - DUAL_YAW_SMALL_MIN_DEG) * angle_kp_rpm_per_deg,
        0.0f,
        max_small_rpm);
    return Dual_Yaw_Clamp(target_rpm, -negative_limit, positive_limit);
}

void Dual_Yaw_Aim_Reset(dual_yaw_aim_t *state)
{
    memset(state, 0, sizeof(*state));
    state->state = DUAL_YAW_AIM_OFF;
}

void Dual_Yaw_Aim_Update(dual_yaw_aim_t *state,
                         uint8_t fresh,
                         uint32_t frame_count,
                         float camera_yaw_deg,
                         float small_angle_deg,
                         float big_actual_rpm,
                         float signed_yaw_error_deg,
                         float angle_kp_rpm_per_deg,
                         float max_small_rpm,
                         float dt_s)
{
    float target_total_rpm;
    float raw_small_rpm;
    float center_kp;
    float needed_big_rpm;
    uint8_t new_target_frame;

    if (dt_s <= 0.0f)
    {
        dt_s = 0.002f;
    }

    if (state->initialized == 0U)
    {
        /* 修改：首次进入且无目标时，保持当前相机朝向，禁止追逐默认零角。 */
        state->initialized = 1U;
        state->camera_target_deg = Dual_Yaw_WrapDeg(camera_yaw_deg);
        state->filtered_big_rpm = big_actual_rpm;
        state->state = DUAL_YAW_AIM_HOLD;
    }

    state->filtered_big_rpm = Dual_Yaw_Filter(state->filtered_big_rpm,
                                              big_actual_rpm,
                                              dt_s,
                                              DUAL_YAW_FEEDBACK_FILTER_TAU_S);
    new_target_frame = (uint8_t)(fresh != 0U &&
                                 (state->has_target_frame == 0U ||
                                  frame_count != state->applied_frame));
    if (new_target_frame != 0U)
    {
        /* 修改：每帧只锚定一次世界目标；两帧之间继续使用IMU反馈闭环。 */
        state->has_target_frame = 1U;
        state->applied_frame = frame_count;
        state->camera_target_deg = Dual_Yaw_WrapDeg(
            camera_yaw_deg + Dual_Yaw_Clamp(signed_yaw_error_deg,
                                            -DUAL_YAW_ERROR_LIMIT_DEG,
                                            DUAL_YAW_ERROR_LIMIT_DEG));
        state->state = DUAL_YAW_AIM_TRACK;
    }
    else if (fresh == 0U && state->state == DUAL_YAW_AIM_TRACK)
    {
        state->state = DUAL_YAW_AIM_BRAKE;
    }

    state->error_deg = Dual_Yaw_WrapDeg(state->camera_target_deg - camera_yaw_deg);

    if (state->state == DUAL_YAW_AIM_TRACK)
    {
        target_total_rpm = Dual_Yaw_Clamp(angle_kp_rpm_per_deg * state->error_deg,
                                          -max_small_rpm,
                                          max_small_rpm);
        state->total_rpm = Dual_Yaw_Filter(state->total_rpm,
                                            target_total_rpm,
                                            dt_s,
                                            DUAL_YAW_TOTAL_FILTER_TAU_S);
        state->big_lowpass_rpm = Dual_Yaw_Filter(state->big_lowpass_rpm,
                                                  state->total_rpm,
                                                  dt_s,
                                                  DUAL_YAW_BIG_FILTER_TAU_S);
        center_kp = DUAL_YAW_CENTER_KP_RPM_PER_DEG;
        if (small_angle_deg < DUAL_YAW_SMALL_MIN_DEG + DUAL_YAW_EDGE_MARGIN_DEG ||
            small_angle_deg > DUAL_YAW_SMALL_MAX_DEG - DUAL_YAW_EDGE_MARGIN_DEG)
        {
            center_kp = DUAL_YAW_EDGE_KP_RPM_PER_DEG;
        }
        state->big_ref_rpm = Dual_Yaw_Clamp(
            state->big_lowpass_rpm +
                center_kp * (small_angle_deg - DUAL_YAW_SMALL_CENTER_DEG),
            -DUAL_YAW_BIG_MAX_RPM,
            DUAL_YAW_BIG_MAX_RPM);
    }
    else if (state->state == DUAL_YAW_AIM_BRAKE)
    {
        /* 修改：丢目标先卸下追踪速度，等ID1实际停稳后再锁住此时相机朝向。 */
        state->total_rpm = Dual_Yaw_ApproachZero(state->total_rpm,
                                                 DUAL_YAW_TOTAL_BRAKE_RPM_S * dt_s);
        state->big_ref_rpm = Dual_Yaw_ApproachZero(state->big_ref_rpm,
                                                   DUAL_YAW_BIG_BRAKE_RPM_S * dt_s);
        state->big_lowpass_rpm = state->big_ref_rpm;
        if (fabsf(state->total_rpm) < 0.01f &&
            fabsf(state->big_ref_rpm) < 0.01f &&
            fabsf(big_actual_rpm) < 1.0f)
        {
            state->camera_target_deg = Dual_Yaw_WrapDeg(camera_yaw_deg);
            state->error_deg = 0.0f;
            state->state = DUAL_YAW_AIM_HOLD;
        }
    }

    if (state->state == DUAL_YAW_AIM_HOLD)
    {
        target_total_rpm = Dual_Yaw_Clamp(angle_kp_rpm_per_deg * state->error_deg,
                                          -DUAL_YAW_HOLD_MAX_RPM,
                                          DUAL_YAW_HOLD_MAX_RPM);
        state->total_rpm = Dual_Yaw_Filter(state->total_rpm,
                                            target_total_rpm,
                                            dt_s,
                                            DUAL_YAW_TOTAL_FILTER_TAU_S);
        state->big_ref_rpm = 0.0f;
        state->big_lowpass_rpm = 0.0f;
    }

    /* 修改：扣除ID1实际余转，而非尚未实现的指令速度，保持相机合成速度。 */
    raw_small_rpm = state->total_rpm - state->filtered_big_rpm;
    state->small_ref_rpm = Dual_Yaw_LimitSmall(raw_small_rpm,
                                               small_angle_deg,
                                               angle_kp_rpm_per_deg,
                                               max_small_rpm);
    if (state->state == DUAL_YAW_AIM_TRACK)
    {
        /* 修改：ID2受到速度或机械限位约束时，允许ID1立即增加对应方向分担。 */
        needed_big_rpm = state->total_rpm - state->small_ref_rpm;
        if (raw_small_rpm > state->small_ref_rpm && needed_big_rpm > state->big_ref_rpm)
        {
            state->big_ref_rpm = needed_big_rpm;
        }
        else if (raw_small_rpm < state->small_ref_rpm && needed_big_rpm < state->big_ref_rpm)
        {
            state->big_ref_rpm = needed_big_rpm;
        }
        state->big_ref_rpm = Dual_Yaw_Clamp(state->big_ref_rpm,
                                            -DUAL_YAW_BIG_MAX_RPM,
                                            DUAL_YAW_BIG_MAX_RPM);
    }
}
