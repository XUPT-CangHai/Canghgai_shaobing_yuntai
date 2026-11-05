#include "pitch_test.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can.h"
#include "dji_motor.h"
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
           (remote->switch_position[RC_S_LEFT] != RC_SW_MID))))))
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
    g_aim_yaw_applied_frame_count = 0U;
    g_aim_pitch_applied_frame_count = 0U;
    g_aim_yaw_target_angle = 0.0f;
    g_aim_active = 0U;
    Yaw_LegacyPid_Init(&g_yaw_angle_pid, YAW_ANGLE_KP, 0.0f, 0.0f,
                       0.0f, YAW_ANGLE_MAX_RPM);
    Yaw_LegacyPid_Init(&g_yaw_speed_pid, YAW_SPEED_KP, YAW_SPEED_KI, 0.0f,
                       YAW_SPEED_MAX_I, YAW_CURRENT_LIMIT);
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

    /* 鑷瀯鐢?USART6 鐙珛浣胯兘锛涘彧鏈夋墜鍔ㄦā寮忚姹傞仴鎺у湪绾夸笖宸﹀紑鍏崇疆涓€?*/
    if ((g_pitch_motor_online == 0U) ||
        ((g_pitch_serial_mode == 0U) &&
         (((aim.aim_enabled != 0U) && (aim_active == 0U)) ||
          ((aim.aim_enabled == 0U) &&
           ((g_pitch_remote_online == 0U) ||
            (g_pitch_remote_valid == 0U) ||
            (remote.switch_position[RC_S_LEFT] != RC_SW_MID))))))
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

    yaw_current = Yaw_Update(&remote, &aim, aim_active, now_ms);
    g_pitch_debug_current = pitch_current;
    /* CAN2缁熶竴鍙戦€侊細ID1鍥哄畾闆剁數娴併€両D2 Yaw銆両D3 Pitch銆?*/
    if (DJI_6020_SendCurrent(&hcan2, 0, yaw_current, pitch_current, 0) != HAL_OK)
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
    return (g_yaw_debug_enabled != 0U) ? 20U : PITCH_DEBUG_PERIOD_MS;
}

void Pitch_Test_DebugPrint(void)
{
    remote_control_t remote = {0};
    vision_aim_command_t aim = {0};
    dji_6020_feedback_t pitch_feedback = {0};
    static char message[512];
    uint32_t now_ms = HAL_GetTick();
    uint32_t frame_age_ms;
    float pitch_angle;
    int pitch_limit;
    int length;

    /* 上一条 DMA 未发完时，不改写正在发送的 Pitch 快照。 */
    if (huart6.gState != HAL_UART_STATE_READY)
    {
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

    Remote_Control_Get(&remote);
    Vision_Aim_Get(&aim);
    DJI_6020_GetFeedback(PITCH_MOTOR_INDEX, &pitch_feedback);
    frame_age_ms = (aim.valid_frame_count != 0U) ?
                   (uint32_t)(now_ms - aim.last_rx_tick_ms) : UINT32_MAX;
    pitch_angle = Pitch_Test_GetAngle(&pitch_feedback);
    pitch_limit = (pitch_angle <= PITCH_ANGLE_MIN_DEG) ? -1 :
                  ((pitch_angle >= PITCH_ANGLE_MAX_DEG) ? 1 : 0);

    /* USART6 每秒输出一次完整 Pitch 快照，用于重力补偿测量。 */
    length = snprintf(message, sizeof(message),
                      "PIT:on=%u serial=%u rc=%u/%u sw=%u ch=%d "
                      "init=%u ecd=%u last=%u a=%.2f t=%.2f e=%.2f "
                      "rpm=%d prpm=%d cmd=%d fbI=%d temp=%u int=%.1f "
                      "gff=%.0f fff=%.0f lim=%d txfail=%lu "
                      "AIM:en=%u active=%u valid=%u fresh=%u perr=%.2f seq=%u age=%lu\r\n",
                      (unsigned int)DJI_6020_IsOnline(PITCH_MOTOR_INDEX, now_ms,
                                                     PITCH_MOTOR_TIMEOUT_MS),
                      (unsigned int)g_pitch_serial_mode,
                      (unsigned int)g_pitch_remote_online,
                      (unsigned int)g_pitch_remote_valid,
                      (unsigned int)remote.switch_position[RC_S_LEFT],
                      (int)g_pitch_debug_channel,
                      (unsigned int)g_pitch_target_initialized,
                      (unsigned int)pitch_feedback.ecd,
                      (unsigned int)pitch_feedback.last_ecd,
                      (double)pitch_angle,
                      (double)g_pitch_target_angle,
                      (double)g_pitch_debug_error,
                      (int)pitch_feedback.speed_rpm,
                      (int)g_pitch_debug_target_rpm,
                      (int)g_pitch_debug_current,
                      (int)pitch_feedback.given_current,
                      (unsigned int)pitch_feedback.temperature,
                      (double)g_pitch_integral,
                      (double)g_pitch_gravity_ff,
                      (double)g_pitch_friction_ff,
                      pitch_limit,
                      (unsigned long)g_pitch_can_tx_fail,
                      (unsigned int)aim.aim_enabled,
                      (unsigned int)g_aim_active,
                      (unsigned int)aim.target_valid,
                      (unsigned int)((aim.valid_frame_count != 0U) &&
                                     (frame_age_ms <= VISION_AIM_TIMEOUT_MS)),
                      (double)aim.pitch_error_cdeg / 100.0,
                      (unsigned int)aim.seq,
                      (unsigned long)frame_age_ms);
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
