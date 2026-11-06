#include "pitch_test.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can.h"
#include "dji_motor.h"
#include "imu_system.h"
#include "pid.h"
#include "remote_control.h"
#include "vision_aim.h"
#include "usart.h"

/* ID2 Yaw锛氳縼绉绘棫宸ョ▼鐨勨€滄墜鍔ㄨ窡闅?+ 鏉炬潌浣嶇疆淇濇寔鈥濅覆绾ф帶鍒躲€?*/
#define BIG_YAW_MOTOR_INDEX     0U       /* 6020 ID1，大 Yaw，反馈帧 0x205。 */
#define YAW_MOTOR_INDEX         1U       /* 6020 ID2，小 Yaw，反馈帧 0x206。 */
#define BIG_YAW_STATUS_TIMEOUT_MS 120U
#define BIG_YAW_SPEED_MAX_RPM   300.0f
/* 修改：正反转实测保留300/0.7，400/1.0出现反馈尖峰和输出饱和。 */
#define BIG_YAW_SPEED_KP         300.0f
#define BIG_YAW_SPEED_KI         0.70f
#define BIG_YAW_SPEED_MAX_I      300.0f
#define BIG_YAW_CURRENT_LIMIT    16000.0f
/* 修改：ID1专用串口快照为50ms，观察转速误差和云台朝向。 */
#define BIG_YAW_DEBUG_PERIOD_MS  50U
/* A5逆时针为正，ID1目标取反；补偿物理方向已经现场确认。 */
#define BIG_YAW_COMPENSATION_SIGN (-1.0f)
/* 修改：朝向外环输出单位rpm；实测后上电启用，运行仍由底盘A5/拨杆门控。 */
#define BIG_YAW_HOLD_ANGLE_KP    0.05f  /* rpm/deg。 */
#define BIG_YAW_HOLD_ANGLE_KI    0.003f /* rpm/(deg*s)。 */
#define BIG_YAW_HOLD_TRIM_MAX_RPM 3.0f
#define BIG_YAW_HOLD_MAX_RPM     10.0f
#define YAW_RC_DEADZONE         10
#define YAW_RC_DIRECTION       -1.0f
#define YAW_RC_MAX_SPEED_DEG_S  55.0f
#define YAW_ANGLE_KP             1.35f  /* 璋冭瘯锛氭彁楂樿搴﹁窡闅忕伒鏁忓害銆?*/
#define YAW_ANGLE_MAX_RPM        160.0f
#define YAW_SPEED_KP             180.0f
#define YAW_SPEED_KI             0.35f
#define YAW_SPEED_MAX_I          300.0f
#define YAW_CURRENT_LIMIT        16000.0f
/* 修改：2026-11-06现场确认ID2小Yaw机械中位ecd=3627，绝对编码器角159.39度。 */
#define YAW_ANGLE_MIN_DEG        125.0f
#define YAW_ANGLE_MAX_DEG        212.0f
#define AIM_YAW_ERROR_MAX_DEG    45.0f
#define AIM_PITCH_ERROR_MAX_DEG  20.0f

/* 视觉/串口共用的 Yaw 参数，遥控参数仍使用上面的原值。 */
#define YAW_AIM_ANGLE_KP         2.8f
#define YAW_AIM_SPEED_KP         180.0f
#define YAW_AIM_SPEED_KI         0.35f /* 每 2ms 累加一次。 */
#define YAW_AIM_SPEED_MAX_I      2000.0f

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
/* 修改：主人此前确认的Pitch水平中位ecd=1191，绝对编码器角52.34度。 */
#define PITCH_ANGLE_MIN_DEG     26.0f
#define PITCH_ANGLE_MAX_DEG     80.0f

/* 浣嶇疆PI锛歅浠?50灏忔鎻愰珮鍒?000锛屽皬绉垎琛ュ伩璐熻浇闈欏樊锛屼笉浣跨敤閫熷害鍐呯幆銆?*/
#define PITCH_ANGLE_KP          1050.0f
#define PITCH_ANGLE_KI          65.0f /* 鎸夌绉垎锛?ms涓€娆＄浉褰撲簬姣忔绱姞0.12鍊嶈搴﹁宸€?*/
#define PITCH_INTEGRAL_LIMIT    500.0f /* 加入前馈后，积分只修正剩余静差。 */ /* 绉垎鍗曠嫭闄愬箙锛岄伩鍏嶅彈闃绘椂涓嶆柇绱Н銆?*/
#define PITCH_CURRENT_LIMIT     16000.0f
#define PITCH_SERIAL_LINE_MAX   64U

/* Pitch 重力/摩擦补偿首版，表值单位为 6020 发送电流。 */
#define PITCH_GRAVITY_FF_SCALE   0.30f
#define PITCH_FRICTION_FF        180.0f
#define PITCH_FRICTION_SPEED_RPM 30
#define PITCH_STATIC_FRICTION_ERROR_DEG 0.5f
#define PITCH_CASCADE_ANGLE_KP  10.0f
#define PITCH_CASCADE_MAX_RPM    300.0f
#define PITCH_CASCADE_SPEED_KP  80.0f
#define PITCH_CASCADE_SPEED_KI  8.0f
#define PITCH_CASCADE_SPEED_MAX_I 2000.0f
#define PITCH_SPEED_FEEDBACK_DIRECTION (-1.0f)

static const float g_pitch_gravity_angle[] =
    {30.0f, 35.0f, 40.0f, 45.0f, 50.0f, 55.0f, 60.0f, 65.0f, 70.0f};
static const float g_pitch_gravity_current[] =
    {2643.0f, 2288.0f, 2368.0f, 2717.0f, 2562.0f,
     2730.0f, 2446.0f, 3003.0f, 2681.0f};
static float g_pitch_gravity_ff;
static float g_pitch_friction_ff;

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
static int16_t g_pitch_debug_target_rpm;
static uint32_t g_pitch_can_tx_fail;
static uint8_t g_pitch_serial_mode;
static float g_pitch_serial_target_angle;
static uint8_t g_pitch_serial_status_request;
static volatile uint8_t g_pitch_serial_rx_length;
static volatile uint8_t g_pitch_serial_line_ready;
static volatile char g_pitch_serial_rx_line[PITCH_SERIAL_LINE_MAX];
static volatile char g_pitch_serial_pending_line[PITCH_SERIAL_LINE_MAX];

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

static yaw_legacy_pid_t g_pitch_angle_pid;
static yaw_legacy_pid_t g_pitch_speed_pid;
static void Yaw_LegacyPid_Clear(yaw_legacy_pid_t *pid);

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
static yaw_legacy_pid_t g_big_yaw_speed_pid;
static uint8_t g_big_yaw_enabled;
/* 修改：保留小数目标速度，避免低速跟随误差被整数快照隐藏。 */
static float g_big_yaw_debug_target_rpm;
static int16_t g_big_yaw_debug_speed_rpm;
static int16_t g_big_yaw_debug_current;
static uint8_t g_big_yaw_debug_enabled;
/* 修改：记录补偿运行中因通信/电机超时撤力的次数，右中正常停止不计数。 */
static uint32_t g_big_yaw_drop_count;
static uint8_t g_big_yaw_hold_enabled;
static uint8_t g_big_yaw_hold_initialized;
static uint8_t g_big_yaw_hold_valid;
static float g_big_yaw_small_yaw_sign;
static float g_big_yaw_hold_kp;
static float g_big_yaw_hold_ki;
static float g_big_yaw_hold_heading;
static float g_big_yaw_hold_target;
static float g_big_yaw_hold_error;
static float g_big_yaw_hold_trim;
static float g_big_yaw_hold_correction;

/* 视觉与串口阶跃共用此组 PID；遥控器保留原来的参数。 */
static yaw_legacy_pid_t g_yaw_aim_speed_pid;
static float g_yaw_aim_angle_kp = YAW_AIM_ANGLE_KP;
static float g_yaw_aim_max_rpm = YAW_ANGLE_MAX_RPM;
static uint8_t g_yaw_serial_mode;
static uint8_t g_yaw_debug_enabled;
static float g_yaw_serial_target_angle;
static uint32_t g_yaw_command_tick_ms;

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

static float Pitch_Test_GravityFeedforward(float angle)
{
    uint32_t i;
    const uint32_t count =
        (uint32_t)(sizeof(g_pitch_gravity_angle) /
                   sizeof(g_pitch_gravity_angle[0]));

    if (angle <= g_pitch_gravity_angle[0])
    {
        return g_pitch_gravity_current[0] * PITCH_GRAVITY_FF_SCALE;
    }
    if (angle >= g_pitch_gravity_angle[count - 1U])
    {
        return g_pitch_gravity_current[count - 1U] *
               PITCH_GRAVITY_FF_SCALE;
    }
    for (i = 1U; i < count; i++)
    {
        if (angle <= g_pitch_gravity_angle[i])
        {
            float a0 = g_pitch_gravity_angle[i - 1U];
            float a1 = g_pitch_gravity_angle[i];
            float c0 = g_pitch_gravity_current[i - 1U];
            float c1 = g_pitch_gravity_current[i];
            float ratio = (angle - a0) / (a1 - a0);
            return (c0 + (c1 - c0) * ratio) * PITCH_GRAVITY_FF_SCALE;
        }
    }
    return 0.0f;
}

static float Pitch_Test_FrictionFeedforward(int16_t speed_rpm,
                                            float angle_error)
{
    if (speed_rpm > PITCH_FRICTION_SPEED_RPM)
    {
        return PITCH_FRICTION_FF;
    }
    if (speed_rpm < -PITCH_FRICTION_SPEED_RPM)
    {
        return -PITCH_FRICTION_FF;
    }
    if (angle_error > PITCH_STATIC_FRICTION_ERROR_DEG)
    {
        return PITCH_FRICTION_FF;
    }
    if (angle_error < -PITCH_STATIC_FRICTION_ERROR_DEG)
    {
        return -PITCH_FRICTION_FF;
    }
    return 0.0f;
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

/* Pitch 速度内环使用位置式 PI，避免增量式 PID 在反馈换向时反冲。 */
static float Pitch_SpeedPid_Position(yaw_legacy_pid_t *pid,
                                     float target,
                                     float feedback)
{
    float error = target - feedback;

    pid->iout += pid->ki * error * PITCH_CONTROL_PERIOD_S;
    pid->iout = Pitch_Test_Clamp(pid->iout, pid->max_iout);
    pid->out = pid->kp * error + pid->iout;
    return Pitch_Test_Clamp(pid->out, pid->max_out);
}

/* 独立保存积分，输出饱和时只允许消除饱和方向的积分。 */
static float Yaw_AimSpeedPid(yaw_legacy_pid_t *pid, float target, float feedback)
{
    float error = target - feedback;
    float request = pid->kp * error + pid->iout;

    if (((request < pid->max_out) && (request > -pid->max_out)) ||
        ((request >= pid->max_out) && (error < 0.0f)) ||
        ((request <= -pid->max_out) && (error > 0.0f)))
    {
        pid->iout = Pitch_Test_Clamp(pid->iout + pid->ki * error, pid->max_iout);
    }
    pid->out = Pitch_Test_Clamp(pid->kp * error + pid->iout, pid->max_out);
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

/* 修改：只清朝向外环，保留正在运行的速度PI，避免切换外环时撤力。 */
static void Big_Yaw_ClearHold(void)
{
    g_big_yaw_hold_initialized = 0U;
    g_big_yaw_hold_error = 0.0f;
    g_big_yaw_hold_trim = 0.0f;
    g_big_yaw_hold_correction = 0.0f;
}

static void Pitch_Test_SerialSend(const char *text)
{
    static char tx_buffer[160];
    size_t length;

    if ((text == NULL) || (huart6.gState != HAL_UART_STATE_READY))
    {
        return;
    }
    length = strlen(text);
    if ((length == 0U) || (length >= sizeof(tx_buffer)))
    {
        return;
    }
    memcpy(tx_buffer, text, length + 1U);
    (void)HAL_UART_Transmit_DMA(&huart6, (uint8_t *)tx_buffer,
                                (uint16_t)length);
}

static uint8_t Pitch_Test_SerialParseFloat(const char *text, float *value)
{
    char *end;
    float parsed;

    if ((text == NULL) || (value == NULL))
    {
        return 0U;
    }
    parsed = strtof(text, &end);
    while (isspace((unsigned char)*end) != 0)
    {
        end++;
    }
    if ((end == text) || (*end != '\0') || !isfinite(parsed))
    {
        return 0U;
    }
    *value = parsed;
    return 1U;
}

static void Pitch_Test_SerialProcess(float current_angle)
{
    char line[PITCH_SERIAL_LINE_MAX];
    char *argument;
    float target_angle;
    uint32_t primask;
    uint8_t i;

    if (g_pitch_serial_line_ready == 0U)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(line, (const void *)g_pitch_serial_pending_line, sizeof(line));
    g_pitch_serial_line_ready = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    for (i = 0U; i < (uint8_t)(sizeof(line) - 1U); i++)
    {
        if (line[i] == '\0')
        {
            break;
        }
        line[i] = (char)toupper((unsigned char)line[i]);
    }
    line[sizeof(line) - 1U] = '\0';

    /* 修改：BY命令仅采集/调整ID1速度PID，启停仍由底盘拨杆和A5决定。 */
    if (strcmp(line, "BY GET") == 0)
    {
        g_big_yaw_debug_enabled = 1U;
        g_pitch_serial_status_request = 1U;
        return;
    }
    if (strcmp(line, "BY LOG 0") == 0)
    {
        g_big_yaw_debug_enabled = 0U;
        Pitch_Test_SerialSend("ACK BY LOG 0\r\n");
        return;
    }
    /* 修改：先确认ID2转角符号，再启用朝向外环；BY命令仍不独立启动ID1。 */
    if ((strcmp(line, "BY HOLD 0") == 0) ||
        (strcmp(line, "BY HOLD 1") == 0))
    {
        if ((line[8] == '1') && (g_big_yaw_hold_valid == 0U))
        {
            Pitch_Test_SerialSend("ERR BY HOLD sensor\r\n");
            return;
        }
        g_big_yaw_hold_enabled = (uint8_t)(line[8] == '1');
        Big_Yaw_ClearHold();
        g_big_yaw_debug_enabled = 1U;
        Pitch_Test_SerialSend("ACK BY HOLD\r\n");
        return;
    }
    if (strncmp(line, "BY SIGN ", 8U) == 0)
    {
        float sign;

        if ((g_big_yaw_hold_enabled != 0U) ||
            !Pitch_Test_SerialParseFloat(&line[8], &sign) ||
            ((sign != 1.0f) && (sign != -1.0f)))
        {
            Pitch_Test_SerialSend("ERR BY SIGN hold_off_then_1_or_-1\r\n");
            return;
        }
        g_big_yaw_small_yaw_sign = sign;
        Big_Yaw_ClearHold();
        g_big_yaw_debug_enabled = 1U;
        Pitch_Test_SerialSend("ACK BY SIGN\r\n");
        return;
    }
    if ((strncmp(line, "BY HKP ", 7U) == 0) ||
        (strncmp(line, "BY HKI ", 7U) == 0))
    {
        float gain;
        uint8_t is_integral = (uint8_t)(line[5] == 'I');

        if (!Pitch_Test_SerialParseFloat(&line[7], &gain) ||
            (gain < 0.0f) || (gain > (is_integral ? 0.05f : 0.5f)))
        {
            Pitch_Test_SerialSend("ERR BY HOLD gain_range\r\n");
            return;
        }
        if (is_integral != 0U)
        {
            g_big_yaw_hold_ki = gain;
        }
        else
        {
            g_big_yaw_hold_kp = gain;
        }
        /* 调参后重新锚定当前朝向；现场调参应先右中停稳。 */
        Big_Yaw_ClearHold();
        g_big_yaw_debug_enabled = 1U;
        Pitch_Test_SerialSend("ACK BY HOLD GAIN\r\n");
        return;
    }
    if (strncmp(line, "BY TUNE ", 8U) == 0)
    {
        float values[2];
        char *token = strtok(&line[8], " ");
        for (i = 0U; i < 2U; i++)
        {
            if (!Pitch_Test_SerialParseFloat(token, &values[i]))
            {
                Pitch_Test_SerialSend("ERR BY TUNE speed_kp speed_ki\r\n");
                return;
            }
            token = strtok(NULL, " ");
        }
        if ((token != NULL) || (values[0] <= 0.0f) || (values[0] > 1000.0f) ||
            (values[1] < 0.0f) || (values[1] > 20.0f))
        {
            Pitch_Test_SerialSend("ERR BY TUNE range\r\n");
            return;
        }
        g_big_yaw_speed_pid.kp = values[0];
        g_big_yaw_speed_pid.ki = values[1];
        Yaw_LegacyPid_Clear(&g_big_yaw_speed_pid);
        g_big_yaw_debug_enabled = 1U;
        Pitch_Test_SerialSend("ACK BY TUNE\r\n");
        return;
    }

    /* Yaw 调试命令只设置目标，运动仍由主循环的视觉控制路径执行。 */
    if (strcmp(line, "YAW STOP") == 0)
    {
        g_yaw_serial_mode = 0U;
        g_yaw_target_initialized = 0U;
        Yaw_LegacyPid_Clear(&g_yaw_aim_speed_pid);
        Pitch_Test_SerialSend("ACK YAW STOP\r\n");
        return;
    }
    if (strcmp(line, "YAW GET") == 0)
    {
        g_yaw_debug_enabled = 1U;
        g_pitch_serial_status_request = 1U;
        return;
    }
    if ((strncmp(line, "YAW SET ", 8U) == 0) ||
        (strcmp(line, "YAW HOLD") == 0))
    {
        dji_6020_feedback_t feedback = {0};
        if (strcmp(line, "YAW HOLD") == 0)
        {
            DJI_6020_GetFeedback(YAW_MOTOR_INDEX, &feedback);
            target_angle = Yaw_ClampAngle(Pitch_Test_GetAngle(&feedback));
        }
        else if (!Pitch_Test_SerialParseFloat(&line[8], &target_angle))
        {
            Pitch_Test_SerialSend("ERR YAW SET angle\r\n");
            return;
        }
        if ((target_angle < YAW_ANGLE_MIN_DEG) ||
            (target_angle > YAW_ANGLE_MAX_DEG))
        {
            Pitch_Test_SerialSend("ERR YAW SET range=125..212\r\n");
            return;
        }
        if (g_yaw_serial_mode == 0U)
        {
            Yaw_LegacyPid_Clear(&g_yaw_aim_speed_pid);
        }
        g_yaw_serial_target_angle = target_angle;
        g_yaw_serial_mode = 1U;
        g_yaw_debug_enabled = 1U;
        g_yaw_command_tick_ms = HAL_GetTick();
        Pitch_Test_SerialSend("ACK YAW SET\r\n");
        return;
    }
    if (strncmp(line, "YAW TUNE ", 9U) == 0)
    {
        float values[4];
        char *token = strtok(&line[9], " ");
        for (i = 0U; i < 4U; i++)
        {
            if (!Pitch_Test_SerialParseFloat(token, &values[i]))
            {
                Pitch_Test_SerialSend("ERR YAW TUNE angle_kp speed_kp speed_ki max_rpm\r\n");
                return;
            }
            token = strtok(NULL, " ");
        }
        if ((token != NULL) || (values[0] <= 0.0f) || (values[0] > 20.0f) ||
            (values[1] <= 0.0f) || (values[1] > 400.0f) ||
            (values[2] < 0.0f) || (values[2] > 2.0f) ||
            (values[3] <= 0.0f) || (values[3] > 250.0f))
        {
            Pitch_Test_SerialSend("ERR YAW TUNE range\r\n");
            return;
        }
        g_yaw_aim_angle_kp = values[0];
        g_yaw_aim_speed_pid.kp = values[1];
        g_yaw_aim_speed_pid.ki = values[2];
        g_yaw_aim_max_rpm = values[3];
        Yaw_LegacyPid_Clear(&g_yaw_aim_speed_pid);
        Pitch_Test_SerialSend("ACK YAW TUNE\r\n");
        return;
    }

    if (strcmp(line, "PIT STOP") == 0)
    {
        g_pitch_serial_mode = 0U;
        g_pitch_serial_status_request = 0U;
        g_pitch_target_initialized = 0U;
        g_pitch_integral = 0.0f;
        Yaw_LegacyPid_Clear(&g_pitch_angle_pid);
        Yaw_LegacyPid_Clear(&g_pitch_speed_pid);
        Pitch_Test_SerialSend("ACK PIT STOP\r\n");
        return;
    }
    if (strcmp(line, "PIT HOLD") == 0)
    {
        g_pitch_serial_mode = 1U;
        g_pitch_serial_target_angle = Pitch_Test_ClampAngle(current_angle);
        g_pitch_target_initialized = 1U;
        g_pitch_integral = 0.0f;
        Yaw_LegacyPid_Clear(&g_pitch_angle_pid);
        Yaw_LegacyPid_Clear(&g_pitch_speed_pid);
        Pitch_Test_SerialSend("ACK PIT HOLD\r\n");
        return;
    }
    if ((strcmp(line, "PIT GET") == 0) ||
        (strcmp(line, "PIT STATUS") == 0))
    {
        g_pitch_serial_status_request = 1U;
        return;
    }
    if (strcmp(line, "PIT HELP") == 0)
    {
        Pitch_Test_SerialSend(
            "CMD PIT SET <deg>|HOLD|GET|STATUS|STOP|HELP deg=26..80\r\n");
        return;
    }
    if (strncmp(line, "PIT SET ", 8U) == 0)
    {
        argument = &line[8];
        if (!Pitch_Test_SerialParseFloat(argument, &target_angle))
        {
            Pitch_Test_SerialSend("ERR PIT SET angle\r\n");
            return;
        }
        if ((target_angle < PITCH_ANGLE_MIN_DEG) ||
            (target_angle > PITCH_ANGLE_MAX_DEG))
        {
            Pitch_Test_SerialSend("ERR PIT SET range=26..80\r\n");
            return;
        }
        g_pitch_serial_mode = 1U;
        g_pitch_serial_target_angle = target_angle;
        g_pitch_target_initialized = 1U;
        g_pitch_integral = 0.0f;
        Yaw_LegacyPid_Clear(&g_pitch_angle_pid);
        Yaw_LegacyPid_Clear(&g_pitch_speed_pid);
        Pitch_Test_SerialSend("ACK PIT SET\r\n");
        return;
    }

    Pitch_Test_SerialSend("ERR PIT CMD\r\n");
}

void Pitch_Test_SerialRxByte(uint8_t byte)
{
    uint8_t length;

    if ((byte == '\r') || (byte == '\n'))
    {
        length = g_pitch_serial_rx_length;
        if ((length != 0U) && (g_pitch_serial_line_ready == 0U))
        {
            memcpy((void *)g_pitch_serial_pending_line,
                   (const void *)g_pitch_serial_rx_line, length);
            g_pitch_serial_pending_line[length] = '\0';
            g_pitch_serial_line_ready = 1U;
        }
        g_pitch_serial_rx_length = 0U;
        return;
    }
    if ((byte < 0x20U) || (byte >= 0x7FU))
    {
        g_pitch_serial_rx_length = 0U;
        return;
    }
    if (g_pitch_serial_line_ready != 0U)
    {
        return;
    }
    length = g_pitch_serial_rx_length;
    if (length < (uint8_t)(PITCH_SERIAL_LINE_MAX - 1U))
    {
        g_pitch_serial_rx_line[length] = (char)byte;
        g_pitch_serial_rx_length = (uint8_t)(length + 1U);
    }
    else
    {
        g_pitch_serial_rx_length = 0U;
    }
}

void Pitch_Test_SerialSendReady(void)
{
    Pitch_Test_SerialSend("SYS:READY PIT_PROTO=1\r\n");
}

static uint8_t Pitch_Test_AimIsActive(const vision_aim_command_t *aim,
                                      uint32_t now_ms)
{
    return (uint8_t)((aim->target_valid != 0U) &&
                     ((uint32_t)(now_ms - aim->last_rx_tick_ms) <=
                      VISION_AIM_TIMEOUT_MS) && (aim->aim_enabled != 0U));
}

/*
 * 大 Yaw（6020 ID1）只负责小陀螺反向补偿，ID2 小 Yaw 不参与此链路。
 * A5中的底盘转速由四轮反馈反算，单位rpm，正值为逆时针，与两套IMU同号。
 * ID1按输出轴端逆时针为正配置：底盘逆时针时，ID1目标为负以反向补偿。
 */
static int16_t Big_Yaw_Update(void)
{
    const imu_system_t *imu = IMU_System_GetData();
    remote_chassis_status_t chassis = {0};
    dji_6020_feedback_t feedback = {0};
    dji_6020_feedback_t small_yaw_feedback = {0};
    uint32_t now_ms;
    float target_rpm;

    /* 修改：先取同一轮反馈快照，再取时间；防止新帧时间比入口旧时间晚1ms，
     * 无符号相减误判超时并清空积分。有效性判断和控制共用这两份快照。 */
    Remote_Control_GetChassisStatus(&chassis);
    DJI_6020_GetFeedback(BIG_YAW_MOTOR_INDEX, &feedback);
    DJI_6020_GetFeedback(YAW_MOTOR_INDEX, &small_yaw_feedback);
    now_ms = HAL_GetTick();
    /* 修改：IMU随ID2转，扣除ID2相对角才能得到大Yaw的惯性朝向。
     * 全部角度差都取最短角，跨过IMU的±180度或编码器零点不产生大误差。 */
    g_big_yaw_hold_valid = (uint8_t)((imu->initialized != 0U) &&
        isfinite(imu->ekf.yaw) &&
        (small_yaw_feedback.update_tick_ms != 0U) &&
        ((uint32_t)(now_ms - small_yaw_feedback.update_tick_ms) <=
         PITCH_MOTOR_TIMEOUT_MS));
    if (g_big_yaw_hold_valid != 0U)
    {
        g_big_yaw_hold_heading = Yaw_ShortestError(
            imu->ekf.yaw * 57.29578f,
            g_big_yaw_small_yaw_sign * Pitch_Test_GetAngle(&small_yaw_feedback));
    }
    if ((chassis.update_tick_ms == 0U) ||
        ((uint32_t)(now_ms - chassis.update_tick_ms) >
         BIG_YAW_STATUS_TIMEOUT_MS) ||
        (chassis.small_gyro_enable == 0U) ||
        (feedback.update_tick_ms == 0U) ||
        ((uint32_t)(now_ms - feedback.update_tick_ms) >
         PITCH_MOTOR_TIMEOUT_MS))
    {
        if ((g_big_yaw_enabled != 0U) &&
            (chassis.small_gyro_enable != 0U))
        {
            g_big_yaw_drop_count++;
        }
        g_big_yaw_enabled = 0U;
        g_big_yaw_debug_target_rpm = 0;
        g_big_yaw_debug_speed_rpm = 0;
        g_big_yaw_debug_current = 0;
        Yaw_LegacyPid_Clear(&g_big_yaw_speed_pid);
        Big_Yaw_ClearHold();
        return 0;
    }

    target_rpm = BIG_YAW_COMPENSATION_SIGN * chassis.chassis_z_speed_rpm;
    if ((g_big_yaw_hold_enabled != 0U) && (g_big_yaw_hold_valid != 0U))
    {
        float correction_request;

        if (g_big_yaw_hold_initialized == 0U)
        {
            g_big_yaw_hold_target = g_big_yaw_hold_heading;
            g_big_yaw_hold_initialized = 1U;
        }
        g_big_yaw_hold_error = Yaw_ShortestError(
            g_big_yaw_hold_target, g_big_yaw_hold_heading);
        correction_request = g_big_yaw_hold_kp * g_big_yaw_hold_error +
                             g_big_yaw_hold_trim;
        /* 修改：小积分补恒速静差，修正饱和时只允许积分向退出饱和的方向变化。 */
        if (((correction_request < BIG_YAW_HOLD_MAX_RPM) &&
             (correction_request > -BIG_YAW_HOLD_MAX_RPM)) ||
            ((correction_request >= BIG_YAW_HOLD_MAX_RPM) &&
             (g_big_yaw_hold_error < 0.0f)) ||
            ((correction_request <= -BIG_YAW_HOLD_MAX_RPM) &&
             (g_big_yaw_hold_error > 0.0f)))
        {
            g_big_yaw_hold_trim = Pitch_Test_Clamp(
                g_big_yaw_hold_trim + g_big_yaw_hold_ki *
                g_big_yaw_hold_error * PITCH_CONTROL_PERIOD_S,
                BIG_YAW_HOLD_TRIM_MAX_RPM);
        }
        g_big_yaw_hold_correction = Pitch_Test_Clamp(
            g_big_yaw_hold_kp * g_big_yaw_hold_error + g_big_yaw_hold_trim,
            BIG_YAW_HOLD_MAX_RPM);
        /* ID1反馈正转对应惯性yaw增大；正角度误差增加ID1正向目标rpm。 */
        target_rpm += g_big_yaw_hold_correction;
    }
    else
    {
        /* ID2/IMU不可用时撤销外环，保留原反向速度补偿；恢复后重新锚定。 */
        Big_Yaw_ClearHold();
    }
    target_rpm = Pitch_Test_Clamp(target_rpm, BIG_YAW_SPEED_MAX_RPM);
    g_big_yaw_enabled = 1U;
    g_big_yaw_debug_target_rpm = target_rpm;
    g_big_yaw_debug_speed_rpm = feedback.speed_rpm;
    g_big_yaw_debug_current = (int16_t)Yaw_LegacyPid_Delta(
        &g_big_yaw_speed_pid, target_rpm, (float)feedback.speed_rpm);
    return g_big_yaw_debug_current;
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
        ((g_yaw_serial_mode == 0U) &&
         (((aim->aim_enabled != 0U) && (aim_active == 0U)) ||
          ((aim->aim_enabled == 0U) &&
           ((g_pitch_remote_online == 0U) ||
            (g_pitch_remote_valid == 0U) ||
            ((remote->switch_position[RC_S_LEFT] != RC_SW_MID) &&
             (remote->switch_position[RC_S_LEFT] != RC_SW_UP)))))))
    {
        g_yaw_target_initialized = 0U;
        g_aim_yaw_applied_frame_count = 0U;
        g_yaw_mode = YAW_MODE_DISABLED;
        Yaw_LegacyPid_Clear(&g_yaw_angle_pid);
        Yaw_LegacyPid_Clear(&g_yaw_speed_pid);
        Yaw_LegacyPid_Clear(&g_yaw_aim_speed_pid);
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

    if (g_yaw_serial_mode != 0U)
    {
        angle_error = Pitch_Test_Clamp(
            Yaw_ShortestError(g_yaw_serial_target_angle, current_angle),
            AIM_YAW_ERROR_MAX_DEG);
        g_yaw_mode = 4U;
    }
    else if (aim_active != 0U)
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

    if ((g_yaw_serial_mode != 0U) || (aim_active != 0U))
    {
        /* 两条目标来源共用同一个视觉 Yaw 控制器。 */
        target_rpm = Pitch_Test_Clamp(angle_error * g_yaw_aim_angle_kp,
                                      g_yaw_aim_max_rpm);
        g_yaw_debug_current = (int16_t)Yaw_AimSpeedPid(
            &g_yaw_aim_speed_pid, target_rpm, (float)feedback.speed_rpm);
    }
    else
    {
        target_rpm = Yaw_LegacyPid_Position(&g_yaw_angle_pid, angle_error);
        g_yaw_debug_current = (int16_t)Yaw_LegacyPid_Delta(
            &g_yaw_speed_pid, target_rpm, (float)feedback.speed_rpm);
    }
    g_yaw_debug_channel = remote->channel[0];
    g_yaw_debug_angle10 = (int16_t)(current_angle * 10.0f);
    g_yaw_debug_target10 = (int16_t)((current_angle + angle_error) * 10.0f);
    g_yaw_debug_error10 = (int16_t)(angle_error * 10.0f);
    g_yaw_debug_target_rpm = (int16_t)target_rpm;
    g_yaw_debug_speed_rpm = feedback.speed_rpm;
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
    g_pitch_debug_target_rpm = 0;
    g_pitch_integral = 0.0f;
    g_pitch_gravity_ff = 0.0f;
    g_pitch_friction_ff = 0.0f;
    g_pitch_can_tx_fail = 0U;
    g_pitch_serial_mode = 0U;
    g_pitch_serial_target_angle = 0.0f;
    g_pitch_serial_status_request = 0U;
    g_pitch_serial_rx_length = 0U;
    g_pitch_serial_line_ready = 0U;
    memset((void *)g_pitch_serial_rx_line, 0, sizeof(g_pitch_serial_rx_line));
    memset((void *)g_pitch_serial_pending_line, 0,
           sizeof(g_pitch_serial_pending_line));
    g_yaw_target_initialized = 0U;
    g_yaw_mode = YAW_MODE_DISABLED;
    g_big_yaw_enabled = 0U;
    g_big_yaw_debug_target_rpm = 0;
    g_big_yaw_debug_speed_rpm = 0;
    g_big_yaw_debug_current = 0;
    g_big_yaw_debug_enabled = 0U;
    g_big_yaw_drop_count = 0U;
    /* 修改：正反转验证后默认启用；进入小陀螺时锁定当时的大Yaw朝向。 */
    g_big_yaw_hold_enabled = 1U;
    g_big_yaw_hold_valid = 0U;
    /* 修改：ID2小角度往返实测，编码器角与IMU航向同向，扣除系数为+1。 */
    g_big_yaw_small_yaw_sign = 1.0f;
    g_big_yaw_hold_kp = BIG_YAW_HOLD_ANGLE_KP;
    g_big_yaw_hold_ki = BIG_YAW_HOLD_ANGLE_KI;
    g_big_yaw_hold_heading = 0.0f;
    g_big_yaw_hold_target = 0.0f;
    Big_Yaw_ClearHold();
    g_aim_yaw_applied_frame_count = 0U;
    g_aim_pitch_applied_frame_count = 0U;
    g_aim_yaw_target_angle = 0.0f;
    g_aim_active = 0U;
    Yaw_LegacyPid_Init(&g_yaw_angle_pid, YAW_ANGLE_KP, 0.0f, 0.0f,
                       0.0f, YAW_ANGLE_MAX_RPM);
    Yaw_LegacyPid_Init(&g_yaw_speed_pid, YAW_SPEED_KP, YAW_SPEED_KI, 0.0f,
                       YAW_SPEED_MAX_I, YAW_CURRENT_LIMIT);
    Yaw_LegacyPid_Init(&g_big_yaw_speed_pid, BIG_YAW_SPEED_KP,
                       BIG_YAW_SPEED_KI, 0.0f, BIG_YAW_SPEED_MAX_I,
                       BIG_YAW_CURRENT_LIMIT);
    Yaw_LegacyPid_Init(&g_yaw_aim_speed_pid, YAW_AIM_SPEED_KP, YAW_AIM_SPEED_KI,
                       0.0f, YAW_AIM_SPEED_MAX_I, YAW_CURRENT_LIMIT);
    g_yaw_serial_mode = 0U;
    g_yaw_debug_enabled = 0U;
    Yaw_LegacyPid_Init(&g_pitch_angle_pid, PITCH_CASCADE_ANGLE_KP, 0.0f,
                       0.0f, 0.0f, PITCH_CASCADE_MAX_RPM);
    Yaw_LegacyPid_Init(&g_pitch_speed_pid, PITCH_CASCADE_SPEED_KP,
                       PITCH_CASCADE_SPEED_KI, 0.0f,
                       PITCH_CASCADE_SPEED_MAX_I, PITCH_CURRENT_LIMIT);

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
    int16_t big_yaw_current;
    int16_t yaw_current;
    int16_t rc_input;
    uint8_t aim_active;
    uint8_t pitch_control_enabled = 0U;

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
    Pitch_Test_SerialProcess(current_angle);
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
        Yaw_LegacyPid_Clear(&g_pitch_angle_pid);
        Yaw_LegacyPid_Clear(&g_pitch_speed_pid);
        Yaw_LegacyPid_Clear(&g_yaw_angle_pid);
        Yaw_LegacyPid_Clear(&g_yaw_speed_pid);
        Yaw_LegacyPid_Clear(&g_yaw_aim_speed_pid);
    }

    /* 自瞄由USART6独立使能；手动/小陀螺模式要求左开关中档或上档。 */
    if ((g_pitch_motor_online == 0U) ||
        ((g_pitch_serial_mode == 0U) &&
         (((aim.aim_enabled != 0U) && (aim_active == 0U)) ||
          ((aim.aim_enabled == 0U) &&
           ((g_pitch_remote_online == 0U) ||
            (g_pitch_remote_valid == 0U) ||
            ((remote.switch_position[RC_S_LEFT] != RC_SW_MID) &&
             (remote.switch_position[RC_S_LEFT] != RC_SW_UP)))))))
    {
        g_pitch_target_initialized = 0U;
        g_aim_pitch_applied_frame_count = 0U;
        g_pitch_debug_target_rpm = 0;
        /* 澶辫兘娓呯Н鍒嗭紝閲嶆柊鍚敤鏃朵笉甯﹀叆涓婃琛ュ伩銆?*/
        g_pitch_integral = 0.0f;
        Yaw_LegacyPid_Clear(&g_pitch_angle_pid);
        Yaw_LegacyPid_Clear(&g_pitch_speed_pid);
        /* 澶辫兘鏃舵樉绀哄綋鍓嶄綅缃紱涓嬫浣胯兘閲嶆柊閽充綅锛屼笉杩借刀鏃х洰鏍囥€?*/
        g_pitch_target_angle = current_angle;
    }
    else if (g_pitch_serial_mode != 0U)
    {
        float target_rpm;

        pitch_control_enabled = 1U;
        g_pitch_target_initialized = 1U;
        g_pitch_target_angle = Pitch_Test_ClampAngle(g_pitch_serial_target_angle);
        angle_error = g_pitch_target_angle - current_angle;
        g_pitch_debug_error = angle_error;
        target_rpm = Yaw_LegacyPid_Position(&g_pitch_angle_pid, angle_error);
        g_pitch_debug_target_rpm = (int16_t)target_rpm;
        current_request = Pitch_SpeedPid_Position(
            &g_pitch_speed_pid, target_rpm,
            (float)g_pitch_feedback.speed_rpm *
                PITCH_SPEED_FEEDBACK_DIRECTION);
        pitch_current = (int16_t)Pitch_Test_Clamp(
            current_request * PITCH_MOTOR_DIRECTION,
            PITCH_CURRENT_LIMIT);
    }
    else if (aim_active != 0U)
    {
        float target_rpm;

        pitch_control_enabled = 1U;
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
        target_rpm = Yaw_LegacyPid_Position(&g_pitch_angle_pid, angle_error);
        g_pitch_debug_target_rpm = (int16_t)target_rpm;
        current_request = Pitch_SpeedPid_Position(
            &g_pitch_speed_pid, target_rpm,
            (float)g_pitch_feedback.speed_rpm *
                PITCH_SPEED_FEEDBACK_DIRECTION);
        pitch_current = (int16_t)Pitch_Test_Clamp(
            current_request * PITCH_MOTOR_DIRECTION, PITCH_CURRENT_LIMIT);
    }
    else
    {
        g_pitch_debug_target_rpm = 0;
        pitch_control_enabled = 1U;
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

    if (pitch_control_enabled != 0U)
    {
        g_pitch_gravity_ff = Pitch_Test_GravityFeedforward(current_angle);
        g_pitch_friction_ff = Pitch_Test_FrictionFeedforward(
            (int16_t)((float)g_pitch_feedback.speed_rpm *
                      PITCH_SPEED_FEEDBACK_DIRECTION),
            g_pitch_debug_error);
        pitch_current = (int16_t)Pitch_Test_Clamp(
            (float)pitch_current + g_pitch_gravity_ff + g_pitch_friction_ff,
            PITCH_CURRENT_LIMIT);
    }
    else
    {
        g_pitch_gravity_ff = 0.0f;
        g_pitch_friction_ff = 0.0f;
    }

    big_yaw_current = Big_Yaw_Update();
    yaw_current = Yaw_Update(&remote, &aim, aim_active, now_ms);
    g_pitch_debug_current = pitch_current;
    /* 修改：CAN2统一发送ID1大Yaw补偿、ID2小Yaw和ID3 Pitch输出。 */
    if (DJI_6020_SendCurrent(&hcan2, big_yaw_current, yaw_current,
                             pitch_current, 0) != HAL_OK)
    {
        g_pitch_can_tx_fail++;
    }
    if (g_pitch_serial_status_request != 0U)
    {
        g_pitch_serial_status_request = 0U;
        Pitch_Test_DebugPrint();
    }
}

uint32_t Pitch_Test_DebugPeriodMs(void)
{
    /* 修改：小陀螺自动启用快照；BY GET可继续记录停车和换向过程。 */
    if ((g_big_yaw_enabled != 0U) || (g_big_yaw_debug_enabled != 0U))
    {
        return BIG_YAW_DEBUG_PERIOD_MS;
    }
    return (g_yaw_debug_enabled != 0U) ? 20U : PITCH_DEBUG_PERIOD_MS;
}

void Pitch_Test_DebugPrint(void)
{
    const imu_system_t *imu = IMU_System_GetData();
    vision_aim_command_t aim = {0};
    remote_chassis_status_t chassis_status = {0};
    dji_6020_feedback_t big_yaw_feedback = {0};
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

    /* 恢复完整自瞄输出：上一条 DMA 未发完时，不改写发送缓冲。 */
    if (huart6.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    if ((g_big_yaw_enabled != 0U) || (g_big_yaw_debug_enabled != 0U))
    {
        uint32_t status_age_ms;

        Remote_Control_GetChassisStatus(&chassis_status);
        DJI_6020_GetFeedback(BIG_YAW_MOTOR_INDEX, &big_yaw_feedback);
        DJI_6020_GetFeedback(YAW_MOTOR_INDEX, &yaw_feedback);
        /* 修改：快照后的时间也用于遥测，避免on/age被跨毫秒更新误报。 */
        now_ms = HAL_GetTick();
        status_age_ms = (chassis_status.update_tick_ms != 0U) ?
                        (uint32_t)(now_ms - chassis_status.update_tick_ms) :
                        UINT32_MAX;
        /* 修改：ms/age为毫秒，chz/trpm/rpm为rpm，yaw为度，gz为度/秒。
         * err = 目标转速 - ID1反馈；gz和yaw用来判断是否真正保持朝向。 */
        length = snprintf(message, sizeof(message),
            "BY:ms=%lu on=%u en=%u age=%lu chz=%.2f trpm=%.2f "
            "rpm=%d err=%.2f cmd=%d fbI=%d temp=%u "
            "yaw=%.2f gz=%.2f kp=%.1f ki=%.3f drop=%lu "
            "becd=%u secd=%u sa=%.2f hold=%u hv=%u "
            "hd=%.2f ht=%.2f he=%.2f cor=%.2f trim=%.2f "
            "akp=%.3f aki=%.4f sd=%.0f\r\n",
            (unsigned long)now_ms,
            (unsigned int)((big_yaw_feedback.update_tick_ms != 0U) &&
                ((uint32_t)(now_ms - big_yaw_feedback.update_tick_ms) <=
                 PITCH_MOTOR_TIMEOUT_MS)),
            (unsigned int)g_big_yaw_enabled,
            (unsigned long)status_age_ms,
            (double)chassis_status.chassis_z_speed_rpm,
            (double)g_big_yaw_debug_target_rpm,
            (int)big_yaw_feedback.speed_rpm,
            (double)(g_big_yaw_debug_target_rpm -
                     (float)big_yaw_feedback.speed_rpm),
            (int)g_big_yaw_debug_current,
            (int)big_yaw_feedback.given_current,
            (unsigned int)big_yaw_feedback.temperature,
            (double)(imu->ekf.yaw * 57.29578f),
            (double)(imu->gyro[2] * 57.29578f),
            (double)g_big_yaw_speed_pid.kp,
            (double)g_big_yaw_speed_pid.ki,
            (unsigned long)g_big_yaw_drop_count,
            (unsigned int)big_yaw_feedback.ecd,
            (unsigned int)yaw_feedback.ecd,
            (double)Pitch_Test_GetAngle(&yaw_feedback),
            (unsigned int)g_big_yaw_hold_enabled,
            (unsigned int)g_big_yaw_hold_valid,
            (double)g_big_yaw_hold_heading,
            (double)g_big_yaw_hold_target,
            (double)g_big_yaw_hold_error,
            (double)g_big_yaw_hold_correction,
            (double)g_big_yaw_hold_trim,
            (double)g_big_yaw_hold_kp,
            (double)g_big_yaw_hold_ki,
            (double)g_big_yaw_small_yaw_sign);
        if ((length > 0) && (length < (int)sizeof(message)))
        {
            (void)HAL_UART_Transmit_DMA(&huart6, (uint8_t *)message,
                                       (uint16_t)length);
        }
        return;
    }

    if (g_yaw_debug_enabled != 0U)
    {
        dji_6020_feedback_t yaw = {0};
        DJI_6020_GetFeedback(YAW_MOTOR_INDEX, &yaw);
        /* 板上时间戳避免 Windows 串口缓冲影响收敛时间测量。 */
        length = snprintf(message, sizeof(message),
            "YAW:ms=%lu setms=%lu on=%u serial=%u a=%.2f t=%.2f "
            "e=%.2f rpm=%d trpm=%d cmd=%d fbI=%d temp=%u\r\n",
            (unsigned long)now_ms, (unsigned long)g_yaw_command_tick_ms,
            (unsigned int)DJI_6020_IsOnline(YAW_MOTOR_INDEX, now_ms,
                                           PITCH_MOTOR_TIMEOUT_MS),
            (unsigned int)g_yaw_serial_mode,
            (double)Pitch_Test_GetAngle(&yaw),
            (double)((g_yaw_serial_mode != 0U) ? g_yaw_serial_target_angle :
                     ((float)g_yaw_debug_target10 / 10.0f)),
            (double)g_yaw_debug_error10 / 10.0f,
            (int)yaw.speed_rpm, (int)g_yaw_debug_target_rpm,
            (int)g_yaw_debug_current, (int)yaw.given_current,
            (unsigned int)yaw.temperature);
        if ((length > 0) && (length < (int)sizeof(message)))
        {
            (void)HAL_UART_Transmit_DMA(&huart6, (uint8_t *)message,
                                       (uint16_t)length);
        }
        return;
    }

    Vision_Aim_Get(&aim);
    Remote_Control_GetChassisStatus(&chassis_status);
    DJI_6020_GetFeedback(BIG_YAW_MOTOR_INDEX, &big_yaw_feedback);
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
                      "BY:on=%u en=%u chz=%.1f trpm=%d rpm=%d cmd=%d "
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
                      (unsigned int)DJI_6020_IsOnline(BIG_YAW_MOTOR_INDEX,
                                                     now_ms,
                                                     PITCH_MOTOR_TIMEOUT_MS),
                      (unsigned int)g_big_yaw_enabled,
                      (double)chassis_status.chassis_z_speed_rpm,
                      (int)g_big_yaw_debug_target_rpm,
                      (int)big_yaw_feedback.speed_rpm,
                      (int)g_big_yaw_debug_current,
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
