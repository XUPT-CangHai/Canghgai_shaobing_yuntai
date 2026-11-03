#include "pitch_test.h"

#include <stdio.h>

#include "can.h"
#include "dji_motor.h"
#include "imu_system.h"
#include "pid.h"
#include "remote_control.h"
#include "vision_aim.h"
#include "usart.h"

/* ID2 Yaw锛氳縼绉绘棫宸ョ▼鐨勨€滄墜鍔ㄨ窡闅?+ 鏉炬潌浣嶇疆淇濇寔鈥濅覆绾ф帶鍒躲€?*/
#define YAW_MOTOR_INDEX         1U       /* 6020 ID2锛屽弽棣堝抚 0x206銆?*/
#define YAW_RC_DEADZONE         10
#define YAW_RC_DIRECTION       -1.0f
#define YAW_RC_MAX_SPEED_DEG_S  55.0f
#define YAW_ANGLE_KP             1.35f  /* 璋冭瘯锛氭彁楂樿搴﹁窡闅忕伒鏁忓害銆?*/
#define YAW_ANGLE_MAX_RPM        160.0f
#define YAW_SPEED_KP             180.0f
#define YAW_SPEED_KI             0.35f
#define YAW_SPEED_MAX_I          300.0f
#define YAW_CURRENT_LIMIT        16000.0f
#define YAW_ANGLE_MIN_DEG        125.0f
#define YAW_ANGLE_MAX_DEG        212.0f
#define AIM_YAW_ERROR_MAX_DEG    45.0f
#define AIM_PITCH_ERROR_MAX_DEG  20.0f

#define PITCH_MOTOR_INDEX       2U       /* 6020 ID3锛屽弽棣堝抚 0x207銆?*/
#define PITCH_MOTOR_TIMEOUT_MS  100U
#define PITCH_REMOTE_TIMEOUT_MS 200U
#define PITCH_RC_DEADZONE       40
#define PITCH_CONTROL_PERIOD_S   0.002f /* main姣?ms璋冪敤涓€娆℃帶鍒躲€?*/
#define PITCH_RC_MAX_SPEED_DEG_S 55.0f /* 婊℃潌鐩爣绉诲姩閫熷害锛涙寜瀹炴祴鎵嬫劅浠?5鎻愰珮鍒?0搴?s銆?*/
#define PITCH_RC_STEP            (PITCH_RC_MAX_SPEED_DEG_S * PITCH_CONTROL_PERIOD_S / 660.0f)
#define YAW_RC_STEP              (YAW_RC_MAX_SPEED_DEG_S * PITCH_CONTROL_PERIOD_S / 660.0f)
#define PITCH_RC_DIRECTION      -1.0f
#define PITCH_MOTOR_DIRECTION    1.0f
/* CAN2 ID3缂栫爜鍣ㄧ粷瀵硅锛氱洰鏍囪蒋闄愪綅26~80搴︼紝澶辫兘鍚庝粛涓洪浂杈撳嚭銆?*/
#define PITCH_ANGLE_MIN_DEG     26.0f
#define PITCH_ANGLE_MAX_DEG     80.0f

/* 浣嶇疆PI锛歅浠?50灏忔鎻愰珮鍒?000锛屽皬绉垎琛ュ伩璐熻浇闈欏樊锛屼笉浣跨敤閫熷害鍐呯幆銆?*/
#define PITCH_ANGLE_KP          1050.0f
#define PITCH_ANGLE_KI          65.0f /* 鎸夌绉垎锛?ms涓€娆＄浉褰撲簬姣忔绱姞0.12鍊嶈搴﹁宸€?*/
#define PITCH_INTEGRAL_LIMIT    1500.0f /* 绉垎鍗曠嫭闄愬箙锛岄伩鍏嶅彈闃绘椂涓嶆柇绱Н銆?*/
#define PITCH_CURRENT_LIMIT     16000.0f

static dji_6020_feedback_t g_pitch_feedback;
static float g_pitch_target_angle;
static uint8_t g_pitch_target_initialized;
static uint8_t g_pitch_motor_online;
static uint8_t g_pitch_remote_online;
static uint8_t g_pitch_remote_valid;
/* 璋冨弬蹇収锛氫繚瀛樻湰杞湡姝ｅ弬涓庢帶鍒剁殑杈撳叆銆佽宸拰杈撳嚭銆?*/
static int16_t g_pitch_debug_channel;
static int16_t g_pitch_debug_current;
static float g_pitch_debug_error;
static float g_pitch_integral;
static uint32_t g_pitch_can_tx_fail;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float max_iout;
    float max_out;
    float error[3];
    float iout;
    float out;
} yaw_legacy_pid_t;

enum
{
    YAW_MODE_MANUAL = 0U,
    YAW_MODE_HOLD = 1U,
    YAW_MODE_DISABLED = 2U,
    YAW_MODE_AIM = 3U
};

static uint8_t g_yaw_target_initialized;
static uint8_t g_yaw_mode = YAW_MODE_DISABLED;
static float g_yaw_target_angle;
static yaw_legacy_pid_t g_yaw_angle_pid;
static yaw_legacy_pid_t g_yaw_speed_pid;

/* 渚涗覆鍙ｈ皟璇曡鍙栫殑 Yaw 鐘舵€侊紝鍗曚綅锛氳搴︿负0.1搴︺€?*/
static int16_t g_yaw_debug_channel;
static int16_t g_yaw_debug_angle10;
static int16_t g_yaw_debug_target10;
static int16_t g_yaw_debug_error10;
static int16_t g_yaw_debug_target_rpm;
static int16_t g_yaw_debug_speed_rpm;
static int16_t g_yaw_debug_current;
static uint32_t g_aim_yaw_applied_frame_count;
static uint32_t g_aim_pitch_applied_frame_count;
static float g_aim_yaw_target_angle;
static uint8_t g_aim_active;

static float Pitch_Test_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static void Yaw_LegacyPid_Init(yaw_legacy_pid_t *pid,
                               float kp,
                               float ki,
                               float kd,
                               float max_iout,
                               float max_out)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_iout = max_iout;
    pid->max_out = max_out;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->iout = 0.0f;
    pid->out = 0.0f;
}

static void Yaw_LegacyPid_Clear(yaw_legacy_pid_t *pid)
{
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->iout = 0.0f;
    pid->out = 0.0f;
}

static float Yaw_LegacyPid_Position(yaw_legacy_pid_t *pid, float error)
{
    float derivative;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = error;
    pid->iout += pid->ki * pid->error[0];
    pid->iout = Pitch_Test_Clamp(pid->iout, pid->max_iout);
    derivative = pid->error[0] - pid->error[1];
    pid->out = pid->kp * pid->error[0] + pid->iout + pid->kd * derivative;
    return Pitch_Test_Clamp(pid->out, pid->max_out);
}

static float Yaw_LegacyPid_Delta(yaw_legacy_pid_t *pid,
                                 float target,
                                 float feedback)
{
    float error;
    float delta;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    error = target - feedback;
    pid->error[0] = error;
    delta = pid->kp * (pid->error[0] - pid->error[1]) +
            pid->ki * pid->error[0] +
            pid->kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    pid->out = Pitch_Test_Clamp(pid->out + delta, pid->max_out);
    return pid->out;
}

static float Pitch_Test_GetAngle(const dji_6020_feedback_t *feedback)
{
    return ((float)feedback->ecd * 360.0f) / 8192.0f;
}

static float Pitch_Test_ClampAngle(float angle)
{
    if (angle < PITCH_ANGLE_MIN_DEG)
    {
        return PITCH_ANGLE_MIN_DEG;
    }
    if (angle > PITCH_ANGLE_MAX_DEG)
    {
        return PITCH_ANGLE_MAX_DEG;
    }
    return angle;
}

static float Yaw_ClampAngle(float angle)
{
    if (angle < YAW_ANGLE_MIN_DEG)
    {
        return YAW_ANGLE_MIN_DEG;
    }
    if (angle > YAW_ANGLE_MAX_DEG)
    {
        return YAW_ANGLE_MAX_DEG;
    }
    return angle;
}

static float Yaw_ShortestError(float target, float current)
{
    float error = target - current;

    while (error > 180.0f)
    {
        error -= 360.0f;
    }
    while (error < -180.0f)
    {
        error += 360.0f;
    }
    return error;
}

static uint8_t Pitch_Test_AimIsActive(const vision_aim_command_t *aim,
                                      uint32_t now_ms)
{
    return (uint8_t)((aim->target_valid != 0U) &&
                     ((uint32_t)(now_ms - aim->last_rx_tick_ms) <=
                      VISION_AIM_TIMEOUT_MS) && (aim->aim_enabled != 0U));
}

static int16_t Yaw_Update(const remote_control_t *remote,
                          const vision_aim_command_t *aim,
                          uint8_t aim_active,
                          uint32_t now_ms)
{
    dji_6020_feedback_t feedback = {0};
    float current_angle;
    float angle_error;
    float target_rpm;
    int16_t rc_input;

    /* Yaw 浣跨敤 ID2锛涜嚜鐬勪笉渚濊禆閬ユ帶鍣紝寮€鍚絾鏃犳湁鏁堢洰鏍囨椂杈撳嚭闆剁數娴併€?*/
    if ((DJI_6020_IsOnline(YAW_MOTOR_INDEX, now_ms,
                            PITCH_MOTOR_TIMEOUT_MS) == 0U) ||
        ((aim->aim_enabled != 0U) && (aim_active == 0U)) ||
        ((aim->aim_enabled == 0U) &&
         ((g_pitch_remote_online == 0U) ||
          (g_pitch_remote_valid == 0U) ||
           (remote->switch_position[RC_S_LEFT] != RC_SW_MID))))
    {
        g_yaw_target_initialized = 0U;
        g_aim_yaw_applied_frame_count = 0U;
        g_yaw_mode = YAW_MODE_DISABLED;
        Yaw_LegacyPid_Clear(&g_yaw_angle_pid);
        Yaw_LegacyPid_Clear(&g_yaw_speed_pid);
        g_yaw_debug_channel = 0;
        g_yaw_debug_angle10 = 0;
        g_yaw_debug_target10 = 0;
        g_yaw_debug_error10 = 0;
        g_yaw_debug_target_rpm = 0;
        g_yaw_debug_speed_rpm = 0;
        g_yaw_debug_current = 0;
        return 0;
    }

    DJI_6020_GetFeedback(YAW_MOTOR_INDEX, &feedback);
    current_angle = Pitch_Test_GetAngle(&feedback);
    if (g_yaw_target_initialized == 0U)
    {
        /* 閲嶆柊浣胯兘鏃朵粠褰撳墠浣嶇疆寮€濮嬶紝閬垮厤杩借刀涓婃鍋滅敤鍓嶇殑鐩爣銆?*/
        g_yaw_target_angle = Yaw_ClampAngle(current_angle);
        g_yaw_mode = YAW_MODE_MANUAL;
        g_yaw_target_initialized = 1U;
    }

    if (aim_active != 0U)
    {
        if (g_aim_yaw_applied_frame_count != aim->new_frame_count)
        {
            g_aim_yaw_target_angle = current_angle +
                ((float)aim->yaw_error_cdeg / 100.0f);
            g_aim_yaw_applied_frame_count = aim->new_frame_count;
        }
        while (g_aim_yaw_target_angle >= 360.0f)
        {
            g_aim_yaw_target_angle -= 360.0f;
        }
        while (g_aim_yaw_target_angle < 0.0f)
        {
            g_aim_yaw_target_angle += 360.0f;
        }
        g_aim_yaw_target_angle = Yaw_ClampAngle(g_aim_yaw_target_angle);
        angle_error = Pitch_Test_Clamp(
            Yaw_ShortestError(g_aim_yaw_target_angle, current_angle),
            AIM_YAW_ERROR_MAX_DEG);
        g_yaw_mode = YAW_MODE_AIM;
    }
    else
    {
        rc_input = remote->channel[0];
        if ((rc_input >= -YAW_RC_DEADZONE) && (rc_input <= YAW_RC_DEADZONE))
        {
            rc_input = 0;
        }
        if (rc_input != 0)
        {
            /* 鏃у伐绋嬫墜鍔ㄦā寮忥細鎽囨潌鐩存帴鐢熸垚瑙掑害璇樊锛岀瓑鏁堜负閫熷害鎺у埗銆?*/
            g_yaw_mode = YAW_MODE_MANUAL;
            g_yaw_target_angle += (float)rc_input * YAW_RC_STEP *
                                  YAW_RC_DIRECTION;
            g_yaw_target_angle = Yaw_ClampAngle(g_yaw_target_angle);
            angle_error = Yaw_ShortestError(g_yaw_target_angle, current_angle);
        }
        else
        {
            if (g_yaw_mode != YAW_MODE_HOLD)
            {
                /* 鏃у伐绋嬩綅缃繚鎸佹ā寮忥細杩涘叆鏃堕攣瀹氬綋鍓嶈搴︺€?*/
                g_yaw_mode = YAW_MODE_HOLD;
                Yaw_LegacyPid_Clear(&g_yaw_angle_pid);
                Yaw_LegacyPid_Clear(&g_yaw_speed_pid);
            }
            angle_error = Yaw_ShortestError(g_yaw_target_angle, current_angle);
        }

        /* Only block motion farther outside the Yaw limits. */
        if (((current_angle <= YAW_ANGLE_MIN_DEG) && (angle_error < 0.0f)) ||
            ((current_angle >= YAW_ANGLE_MAX_DEG) && (angle_error > 0.0f)))
        {
            angle_error = 0.0f;
            Yaw_LegacyPid_Clear(&g_yaw_angle_pid);
            Yaw_LegacyPid_Clear(&g_yaw_speed_pid);
        }
    }

    target_rpm = Yaw_LegacyPid_Position(&g_yaw_angle_pid, angle_error);
    g_yaw_debug_channel = remote->channel[0];
    g_yaw_debug_angle10 = (int16_t)(current_angle * 10.0f);
    g_yaw_debug_target10 = (int16_t)((current_angle + angle_error) * 10.0f);
    g_yaw_debug_error10 = (int16_t)(angle_error * 10.0f);
    g_yaw_debug_target_rpm = (int16_t)target_rpm;
    g_yaw_debug_speed_rpm = feedback.speed_rpm;
    g_yaw_debug_current = (int16_t)Yaw_LegacyPid_Delta(
        &g_yaw_speed_pid, target_rpm, (float)feedback.speed_rpm);
    return g_yaw_debug_current;
}

void Pitch_Test_Init(void)
{
    /* main鍏堣皟鐢⊿hoot_Test_Init锛屽叡鐢ㄥ凡缁忓惎鍔ㄧ殑CAN2鍜岄仴鎺у櫒鎺ユ敹銆?*/
    g_pitch_target_initialized = 0U;
    g_pitch_motor_online = 0U;
    g_pitch_remote_online = 0U;
    g_pitch_remote_valid = 0U;
    g_pitch_debug_channel = 0;
    g_pitch_debug_current = 0;
    g_pitch_debug_error = 0.0f;
    g_pitch_integral = 0.0f;
    g_pitch_can_tx_fail = 0U;
    g_yaw_target_initialized = 0U;
    g_yaw_mode = YAW_MODE_DISABLED;
    g_aim_yaw_applied_frame_count = 0U;
    g_aim_pitch_applied_frame_count = 0U;
    g_aim_yaw_target_angle = 0.0f;
    g_aim_active = 0U;
    Yaw_LegacyPid_Init(&g_yaw_angle_pid, YAW_ANGLE_KP, 0.0f, 0.0f,
                       0.0f, YAW_ANGLE_MAX_RPM);
    Yaw_LegacyPid_Init(&g_yaw_speed_pid, YAW_SPEED_KP, YAW_SPEED_KI, 0.0f,
                       YAW_SPEED_MAX_I, YAW_CURRENT_LIMIT);

    /* 鏈ā鍧楀彧鎺у埗CAN2鐨?020锛孋AN1鍙戝皠鐢眘hoot_test缁熶竴鎺у埗銆?*/
    (void)DJI_6020_SendCurrent(&hcan2, 0, 0, 0, 0);
}

void Pitch_Test_Update(void)
{
    remote_control_t remote = {0};
    vision_aim_command_t aim = {0};
    uint32_t now_ms = HAL_GetTick();
    float current_angle;
    float angle_error;
    float current_request;
    int16_t pitch_current = 0;
    int16_t yaw_current;
    int16_t rc_input;
    uint8_t aim_active;

    Remote_Control_Get(&remote);
    Vision_Aim_Service();
    Vision_Aim_Get(&aim);
    g_pitch_remote_online = Remote_Control_IsOnline(now_ms,
                                                    PITCH_REMOTE_TIMEOUT_MS);
    g_pitch_remote_valid = Remote_Control_DataValid(&remote);

    DJI_6020_GetFeedback(PITCH_MOTOR_INDEX, &g_pitch_feedback);
    g_pitch_motor_online = DJI_6020_IsOnline(PITCH_MOTOR_INDEX,
                                             now_ms,
                                             PITCH_MOTOR_TIMEOUT_MS);
    current_angle = Pitch_Test_GetAngle(&g_pitch_feedback);
    g_pitch_debug_channel = remote.channel[1];
    g_pitch_debug_error = 0.0f;
    aim_active = Pitch_Test_AimIsActive(&aim, now_ms);
    if (aim_active != g_aim_active)
    {
        /* 鑷瀯涓庨仴鎺у垏鎹㈡椂娓呮棫浣嶇疆鐜姸鎬侊紝閬垮厤鍒囨崲鐬棿杩芥棫鐩爣銆?*/
        g_aim_active = aim_active;
        g_aim_yaw_applied_frame_count = 0U;
        g_aim_pitch_applied_frame_count = 0U;
        g_pitch_target_initialized = 0U;
        g_yaw_target_initialized = 0U;
        g_pitch_integral = 0.0f;
        Yaw_LegacyPid_Clear(&g_yaw_angle_pid);
        Yaw_LegacyPid_Clear(&g_yaw_speed_pid);
    }

    /* 鑷瀯鐢?USART6 鐙珛浣胯兘锛涘彧鏈夋墜鍔ㄦā寮忚姹傞仴鎺у湪绾夸笖宸﹀紑鍏崇疆涓€?*/
    if ((g_pitch_motor_online == 0U) ||
        ((aim.aim_enabled != 0U) && (aim_active == 0U)) ||
        ((aim.aim_enabled == 0U) &&
         ((g_pitch_remote_online == 0U) ||
          (g_pitch_remote_valid == 0U) ||
          (remote.switch_position[RC_S_LEFT] != RC_SW_MID))))
    {
        g_pitch_target_initialized = 0U;
        g_aim_pitch_applied_frame_count = 0U;
        /* 澶辫兘娓呯Н鍒嗭紝閲嶆柊鍚敤鏃朵笉甯﹀叆涓婃琛ュ伩銆?*/
        g_pitch_integral = 0.0f;
        /* 澶辫兘鏃舵樉绀哄綋鍓嶄綅缃紱涓嬫浣胯兘閲嶆柊閽充綅锛屼笉杩借刀鏃х洰鏍囥€?*/
        g_pitch_target_angle = current_angle;
    }
    else if (aim_active != 0U)
    {
        g_pitch_target_initialized = 1U;
        if (g_aim_pitch_applied_frame_count != aim.new_frame_count)
        {
            g_pitch_target_angle = current_angle +
                ((float)aim.pitch_error_cdeg / 100.0f);
            g_aim_pitch_applied_frame_count = aim.new_frame_count;
        }
        g_pitch_target_angle = Pitch_Test_ClampAngle(g_pitch_target_angle);
        angle_error = Pitch_Test_Clamp(
            g_pitch_target_angle - current_angle, AIM_PITCH_ERROR_MAX_DEG);
        g_pitch_debug_error = angle_error;
        current_request = angle_error * PITCH_ANGLE_KP;
        pitch_current = (int16_t)Pitch_Test_Clamp(
            current_request * PITCH_MOTOR_DIRECTION, PITCH_CURRENT_LIMIT);
    }
    else
    {
        if (g_pitch_target_initialized == 0U)
        {
            g_pitch_target_angle = Pitch_Test_ClampAngle(current_angle);
            g_pitch_integral = 0.0f;
            g_pitch_target_initialized = 1U;
        }

        /* 鍙虫憞鏉嗗瀭鐩撮€氶亾涓?channel[1]锛岀洰鏍囪搴﹂檺鍒跺湪26~80搴︺€?*/
        rc_input = remote.channel[1];
        if ((rc_input < PITCH_RC_DEADZONE) &&
            (rc_input > -PITCH_RC_DEADZONE))
        {
            rc_input = 0;
        }
        g_pitch_target_angle += (float)rc_input * PITCH_RC_STEP *
                                PITCH_RC_DIRECTION;
        g_pitch_target_angle = Pitch_Test_ClampAngle(g_pitch_target_angle);

        angle_error = g_pitch_target_angle - current_angle;
        g_pitch_debug_error = angle_error;
        current_request = angle_error * PITCH_ANGLE_KP + g_pitch_integral;
        /* 杈撳嚭楗卞拰鏃朵粎鍏佽鍙嶅悜绉垎锛岄伩鍏嶇户缁悜楗卞拰鏂瑰悜绉疮銆?*/
        if (((current_request < PITCH_CURRENT_LIMIT) &&
             (current_request > -PITCH_CURRENT_LIMIT)) ||
            ((current_request >= PITCH_CURRENT_LIMIT) && (angle_error < 0.0f)) ||
            ((current_request <= -PITCH_CURRENT_LIMIT) && (angle_error > 0.0f)))
        {
            g_pitch_integral = Pitch_Test_Clamp(
                g_pitch_integral + angle_error * PITCH_ANGLE_KI * PITCH_CONTROL_PERIOD_S,
                PITCH_INTEGRAL_LIMIT);
        }
        pitch_current = (int16_t)Pitch_Test_Clamp(
            (angle_error * PITCH_ANGLE_KP + g_pitch_integral) * PITCH_MOTOR_DIRECTION,
            PITCH_CURRENT_LIMIT);
    }

    yaw_current = Yaw_Update(&remote, &aim, aim_active, now_ms);
    g_pitch_debug_current = pitch_current;
    /* CAN2缁熶竴鍙戦€侊細ID1鍥哄畾闆剁數娴併€両D2 Yaw銆両D3 Pitch銆?*/
    if (DJI_6020_SendCurrent(&hcan2, 0, yaw_current, pitch_current, 0) != HAL_OK)
    {
        g_pitch_can_tx_fail++;
    }
}

void Pitch_Test_DebugPrint(void)
{
    const imu_system_t *imu = IMU_System_GetData();
    vision_aim_command_t aim = {0};
    dji_6020_feedback_t yaw_feedback = {0};
    dji_6020_feedback_t pitch_feedback = {0};
    static char message[512];
    uint32_t now_ms = HAL_GetTick();
    uint32_t frame_age_ms;
    float yaw_angle;
    float pitch_angle;
    float yaw_target;
    int yaw_limit;
    int pitch_limit;
    int length;

    /* 自瞄专用调试：上一条 DMA 未发完时，不改写发送缓冲。 */
    if (huart6.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    Vision_Aim_Get(&aim);
    DJI_6020_GetFeedback(YAW_MOTOR_INDEX, &yaw_feedback);
    DJI_6020_GetFeedback(PITCH_MOTOR_INDEX, &pitch_feedback);
    frame_age_ms = (aim.valid_frame_count != 0U) ?
                   (uint32_t)(now_ms - aim.last_rx_tick_ms) : UINT32_MAX;
    yaw_angle = Pitch_Test_GetAngle(&yaw_feedback);
    pitch_angle = Pitch_Test_GetAngle(&pitch_feedback);
    yaw_target = (g_aim_active != 0U) ? g_aim_yaw_target_angle : g_yaw_target_angle;
    yaw_limit = (yaw_angle <= YAW_ANGLE_MIN_DEG) ? -1 :
                ((yaw_angle >= YAW_ANGLE_MAX_DEG) ? 1 : 0);
    pitch_limit = (pitch_angle <= PITCH_ANGLE_MIN_DEG) ? -1 :
                  ((pitch_angle >= PITCH_ANGLE_MAX_DEG) ? 1 : 0);

    /* a/t/yerr/perr 单位为度；cur 为命令/反馈电流，age 单位为毫秒。 */
    length = snprintf(message, sizeof(message),
                      "AIM:enabled=%u active=%u valid=%u fresh=%u "
                      "seq=%u yerr=%.2f perr=%.2f bytes=%lu frames=%lu new=%lu bad=%lu age=%lu "
                      "Y:on=%u a=%.1f t=%.1f cur=%d/%d lim=%d "
                      "P:on=%u a=%.1f t=%.1f cur=%d/%d lim=%d txfail=%lu "
                      "IMU:yaw=%.2f roll=%.2f\r\n",
                      (unsigned int)aim.aim_enabled,
                      (unsigned int)g_aim_active,
                      (unsigned int)aim.target_valid,
                      (unsigned int)((aim.valid_frame_count != 0U) &&
                                     (frame_age_ms <= VISION_AIM_TIMEOUT_MS)),
                      (unsigned int)aim.seq,
                      (double)aim.yaw_error_cdeg / 100.0,
                      (double)aim.pitch_error_cdeg / 100.0,
                      (unsigned long)aim.rx_byte_count,
                      (unsigned long)aim.valid_frame_count,
                      (unsigned long)aim.new_frame_count,
                      (unsigned long)aim.checksum_error_count,
                      (unsigned long)frame_age_ms,
                      (unsigned int)DJI_6020_IsOnline(YAW_MOTOR_INDEX, now_ms,
                                                     PITCH_MOTOR_TIMEOUT_MS),
                      (double)yaw_angle,
                      (double)yaw_target,
                      (int)g_yaw_debug_current,
                      (int)yaw_feedback.given_current,
                      yaw_limit,
                      (unsigned int)DJI_6020_IsOnline(PITCH_MOTOR_INDEX, now_ms,
                                                     PITCH_MOTOR_TIMEOUT_MS),
                      (double)pitch_angle,
                      (double)g_pitch_target_angle,
                      (int)g_pitch_debug_current,
                      (int)pitch_feedback.given_current,
                      pitch_limit,
                      (unsigned long)g_pitch_can_tx_fail,
                      /* EKF 内部为弧度，串口统一输出角度（度）。 */
                      (double)(imu->ekf.yaw * 57.29578f),
                      (double)(imu->ekf.roll * 57.29578f));
    if ((length > 0) && (length < (int)sizeof(message)))
    {
        (void)HAL_UART_Transmit_DMA(&huart6, (uint8_t *)message,
                                    (uint16_t)length);
    }
}

void Pitch_Test_GetYawDebug(uint8_t *mode,
                            int16_t *channel,
                            int16_t *angle10,
                            int16_t *target10,
                            int16_t *error10,
                            int16_t *target_rpm,
                            int16_t *speed_rpm,
                            int16_t *current)
{
    if (mode != 0)
    {
        *mode = g_yaw_mode;
    }
    if (channel != 0)
    {
        *channel = g_yaw_debug_channel;
    }
    if (angle10 != 0)
    {
        *angle10 = g_yaw_debug_angle10;
    }
    if (target10 != 0)
    {
        *target10 = g_yaw_debug_target10;
    }
    if (error10 != 0)
    {
        *error10 = g_yaw_debug_error10;
    }
    if (target_rpm != 0)
    {
        *target_rpm = g_yaw_debug_target_rpm;
    }
    if (speed_rpm != 0)
    {
        *speed_rpm = g_yaw_debug_speed_rpm;
    }
    if (current != 0)
    {
        *current = g_yaw_debug_current;
    }
}

int8_t Pitch_Test_GetYawLimit(void)
{
    dji_6020_feedback_t feedback = {0};
    float angle;

    DJI_6020_GetFeedback(YAW_MOTOR_INDEX, &feedback);
    angle = Pitch_Test_GetAngle(&feedback);

    if (angle <= YAW_ANGLE_MIN_DEG)
    {
        return -1;
    }
    if (angle >= YAW_ANGLE_MAX_DEG)
    {
        return 1;
    }
    return 0;
}
