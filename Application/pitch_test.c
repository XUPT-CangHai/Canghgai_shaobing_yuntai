#include "pitch_test.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can.h"
#include "dji_motor.h"
#include "dual_yaw_aim.h"
#include "imu_system.h"
#include "pid.h"
#include "remote_control.h"
#include "shoot_test.h"
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
#define YAW_ANGLE_MIN_DEG        DUAL_YAW_SMALL_MIN_DEG
#define YAW_ANGLE_MAX_DEG        DUAL_YAW_SMALL_MAX_DEG
#define AIM_YAW_ERROR_MAX_DEG    45.0f
#define AIM_PITCH_ERROR_MAX_DEG  20.0f
/* 修改：画面右正与IMU/两轴编码器的逆时针正相反，只在控制入口取反一次。
 * 新增速度/加速度暂只接收和遥测，前馈系数为0，不改变已经调好的PID。 */
#define AIM_YAW_DIRECTION        (-1.0f)
#define AIM_PITCH_DIRECTION      1.0f
#define AIM_BRAKE_RPM_PER_S      800.0f
/* 修改：固化枪口正对底盘车头时的ID1+ID2正前零位。
 * 2026-11-07用户纠正大Yaw正前位置前后反了：原ecd=698加半圈4096，改为ecd=4794。
 * ID1大Yaw正前绝对编码器角为210.67383度；此处修正参考，不命令电机转半圈。
 * ID2小Yaw保持此前机械中位ecd=3627/159.39度，采样偏斜值ecd=3654不采用。
 * 正前合成零位：wrap180((4794+3627)*360/8192)=10.06348度。
 * 上电直接使用此零位；FOLLOW ZERO仍可临时重标，重启恢复下方固化值。 */
#define FOLLOW_HEADING_ZERO_DEG  10.06348f
#define FOLLOW_HEADING_CALIBRATED 1U

/* 视觉/串口共用的 Yaw 参数，遥控参数仍使用上面的原值。 */
#define YAW_AIM_ANGLE_KP         2.8f
#define YAW_AIM_SPEED_KP         180.0f
#define YAW_AIM_SPEED_KI         0.35f /* 每 2ms 累加一次。 */
#define YAW_AIM_SPEED_MAX_I      2000.0f

#define PITCH_MOTOR_INDEX       2U       /* 6020 ID3锛屽弽棣堝抚 0x207銆?*/
#define PITCH_MOTOR_TIMEOUT_MS  100U
#define PITCH_REMOTE_TIMEOUT_MS 200U
#define PITCH_RC_DEADZONE       40
#define PITCH_CONTROL_PERIOD_S   0.002f /* 修改：沿用main的2ms周期，保持速度环积分强度。 */
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
#define PITCH_CASCADE_SPEED_KI  16.0f /* 修改：由8提高到16，加快自瞄静差的积分修正。 */
#define PITCH_CASCADE_SPEED_MAX_I 2000.0f
/* 修改：实测编码器角增大时原始rpm为正，速度内环和补偿均使用此同向反馈。 */
#define PITCH_SPEED_FEEDBACK_DIRECTION 1.0f

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
/* 修改：发射命令应答等待UART空闲再发，独立DMA缓冲避免被下一条命令覆盖。 */
static char g_shoot_serial_reply[64];
static char g_shoot_serial_tx[64];
static uint8_t g_shoot_serial_status_request;
static uint32_t g_shoot_serial_status_tick_ms;
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
static uint32_t g_aim_pitch_applied_frame_count;
static uint8_t g_aim_active;
/* 修改：静止视觉独占两轴Yaw；有效图像与模式使能分开，丢目标仍减速保持。 */
static dual_yaw_aim_t g_dual_yaw_aim;
static uint8_t g_static_aim_enabled;
static uint8_t g_dual_yaw_active;
static uint8_t g_pitch_aim_state;
static float g_pitch_aim_target_rpm;
/* 修改：左中手动采用相机惯性航向目标，松杆保持航向并继续小Yaw回中。 */
static uint8_t g_manual_yaw_follow;
static float g_follow_camera_target_deg;
static float g_follow_chassis_rpm;
static float g_follow_heading_zero_deg;
static uint8_t g_follow_heading_calibrated;

/* 修改：PDBG只保存本轮2ms控制真正使用的输入/输出，打印时不重新读取电机和视觉。
 * a/t/ve/e单位0.01度，vv单位0.01度/秒，rt单位0.01rpm，r已换成控制同号rpm。
 * p/i/po/g/f/u/fb为电流指令量；src=0停用/1手动/2视觉/3串口，st沿用0/1/2/3。
 * ms/dt/age/fa为板端毫秒；age/fa无历史反馈时为UINT32_MAX。
 * 手动和停用时串级p/i/po/rt清零，避免把上一段视觉PI状态误当当前输出。 */
typedef struct
{
    uint32_t ms, age, nf, fail, dt, fa;
    int32_t a, t, ve, vv, e, rt, r, u, fb, p, i, po, g, f, lim;
    uint8_t src, en, v, fr, st, seq, on;
} pitch_telemetry_t;

static pitch_telemetry_t g_pitch_telemetry;
static uint8_t g_pitch_telemetry_valid;
static uint32_t g_pitch_telemetry_previous_ms;
static uint32_t g_pitch_telemetry_fast_tick_ms;
static uint32_t g_pitch_telemetry_slow_tick_ms;
static uint32_t g_pitch_telemetry_sequence;
static uint32_t g_pitch_telemetry_drop;
#define PITCH_TELEMETRY_PERIOD_MS 10U

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

static void Pitch_Test_QueueShootReply(const char *text)
{
    /* 修改：停止等后续命令可覆盖尚未发出的旧应答，实际动作以FIRE状态为准。 */
    (void)snprintf(g_shoot_serial_reply, sizeof(g_shoot_serial_reply), "%s", text);
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

    /* 修改：串口按钮固定时长，不接受外部时长；重复请求不延长正在运行的动作。 */
    if ((strcmp(line, "SHOOT FIRE") == 0) || (strcmp(line, "SHOOT FEED") == 0))
    {
        uint8_t mode = (strcmp(line, "SHOOT FIRE") == 0) ? 2U : 1U;
        if (Shoot_Test_SerialStart(mode) != 0U)
        {
            Pitch_Test_QueueShootReply((mode == 2U) ?
                "ACK SHOOT FIRE ms=1000\r\n" : "ACK SHOOT FEED ms=2000\r\n");
        }
        else
        {
            Pitch_Test_QueueShootReply("ERR SHOOT BUSY\r\n");
        }
        return;
    }
    if (strcmp(line, "SHOOT STOP") == 0)
    {
        Shoot_Test_SerialStop();
        Pitch_Test_QueueShootReply("ACK SHOOT STOP\r\n");
        return;
    }
    /* 修改：完整发射快照排队等待UART，DMA忙时不丢SHOOT GET。 */
    if (strcmp(line, "SHOOT GET") == 0)
    {
        g_shoot_serial_status_request = 1U;
        return;
    }

    /* 修改：串口正前标定只记录编码器，不发起运动；左中右下停稳后对准车头。 */
    if (strcmp(line, "FOLLOW ZERO") == 0)
    {
        dji_6020_feedback_t big = {0};
        dji_6020_feedback_t small = {0};
        remote_control_t remote = {0};
        remote_chassis_status_t chassis = {0};
        uint32_t now_ms;
        DJI_6020_GetFeedback(BIG_YAW_MOTOR_INDEX, &big);
        DJI_6020_GetFeedback(YAW_MOTOR_INDEX, &small);
        Remote_Control_Get(&remote);
        Remote_Control_GetChassisStatus(&chassis);
        now_ms = HAL_GetTick();
        if ((remote.update_tick_ms == 0U) ||
            ((uint32_t)(now_ms - remote.update_tick_ms) > PITCH_REMOTE_TIMEOUT_MS) ||
            (remote.switch_position[RC_S_LEFT] != RC_SW_MID) ||
            (remote.switch_position[RC_S_RIGHT] != RC_SW_DOWN) ||
            (big.update_tick_ms == 0U) || (small.update_tick_ms == 0U) ||
            ((uint32_t)(now_ms - big.update_tick_ms) > PITCH_MOTOR_TIMEOUT_MS) ||
            ((uint32_t)(now_ms - small.update_tick_ms) > PITCH_MOTOR_TIMEOUT_MS) ||
            (abs(big.speed_rpm) > 2) || (abs(small.speed_rpm) > 2) ||
            (fabsf(chassis.chassis_z_speed_rpm) > 1.0f))
        {
            Pitch_Test_SerialSend("ERR FOLLOW ZERO stop_left_mid_right_down\r\n");
            return;
        }
        g_follow_heading_zero_deg = Yaw_ShortestError(
            Pitch_Test_GetAngle(&big) + Pitch_Test_GetAngle(&small), 0.0f);
        g_follow_heading_calibrated = 1U;
        {
            char ack[120];
            (void)snprintf(ack, sizeof(ack),
                "ACK FOLLOW ZERO z=%.5f becd=%u secd=%u\r\n",
                (double)g_follow_heading_zero_deg,
                (unsigned int)big.ecd, (unsigned int)small.ecd);
            Pitch_Test_SerialSend(ack);
        }
        return;
    }
    if (strcmp(line, "FOLLOW GET") == 0)
    {
        g_pitch_serial_status_request = 1U;
        return;
    }

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
                     ((uint32_t)(now_ms - aim->last_new_target_tick_ms) <=
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

/* 修改：用本轮反馈快照判断在线，避免中断更新后与旧时间相减误判。 */
static uint8_t Pitch_Test_MotorOnline(const dji_6020_feedback_t *feedback,
                                      uint32_t now_ms)
{
    return (uint8_t)((feedback->update_tick_ms != 0U) &&
        ((uint32_t)(now_ms - feedback->update_tick_ms) <= PITCH_MOTOR_TIMEOUT_MS));
}

static uint8_t Static_Aim_Allowed(const remote_chassis_status_t *chassis,
                                   uint32_t now_ms)
{
    /* 修改：自瞄由串口命令使能，不要求遥控器在线或指定拨杆位置。
     * 已接入底盘A5时仍要求状态新鲜且停转；无A5可独立台架自瞄。 */
    if ((chassis->update_tick_ms != 0U) &&
        (((uint32_t)(now_ms - chassis->update_tick_ms) > BIG_YAW_STATUS_TIMEOUT_MS) ||
         (chassis->small_gyro_enable != 0U) ||
         (fabsf(chassis->chassis_z_speed_rpm) > 1.0f)))
    {
        return 0U;
    }
    return 1U;
}

static void Dual_Yaw_ControlReset(void)
{
    Dual_Yaw_Aim_Reset(&g_dual_yaw_aim);
    g_dual_yaw_active = 0U;
    g_big_yaw_enabled = 0U;
    g_big_yaw_debug_target_rpm = 0.0f;
    g_big_yaw_debug_current = 0;
    g_follow_chassis_rpm = 0.0f;
    g_yaw_debug_target_rpm = 0;
    g_yaw_debug_current = 0;
    g_yaw_target_initialized = 0U;
    g_yaw_mode = YAW_MODE_DISABLED;
    Big_Yaw_ClearHold();
    Yaw_LegacyPid_Clear(&g_big_yaw_speed_pid);
    Yaw_LegacyPid_Clear(&g_yaw_angle_pid);
    Yaw_LegacyPid_Clear(&g_yaw_speed_pid);
    Yaw_LegacyPid_Clear(&g_yaw_aim_speed_pid);
}

static void Static_DualYaw_Update(const vision_aim_command_t *aim,
                                   uint8_t fresh,
                                   uint8_t recenter_hold,
                                   const dji_6020_feedback_t *big,
                                   const dji_6020_feedback_t *small,
                                   uint32_t now_ms,
                                   int16_t *big_current,
                                   int16_t *small_current)
{
    const imu_system_t *imu = IMU_System_GetData();
    float small_angle = Pitch_Test_GetAngle(small);
    float small_request;

    *big_current = 0;
    *small_current = 0;
    if ((imu->initialized == 0U) || !isfinite(imu->ekf.yaw) ||
        (Pitch_Test_MotorOnline(big, now_ms) == 0U) ||
        (Pitch_Test_MotorOnline(small, now_ms) == 0U))
    {
        /* 修改：任一Yaw反馈失效，整组退出并清两轴PID，禁止退回旧单轴视觉。 */
        Dual_Yaw_ControlReset();
        return;
    }
    g_dual_yaw_active = 1U;
    g_big_yaw_enabled = 0U; /* 独立于原小陀螺补偿模式。 */
    Dual_Yaw_Aim_Update(&g_dual_yaw_aim, fresh, aim->new_frame_count,
        imu->ekf.yaw * 57.29578f, small_angle, (float)big->speed_rpm,
        AIM_YAW_DIRECTION * (float)aim->yaw_error_cdeg / 100.0f,
        g_yaw_aim_angle_kp, g_yaw_aim_max_rpm, PITCH_CONTROL_PERIOD_S);
    if (recenter_hold != 0U)
    {
        /* 修改：左中视觉丢目标并停稳后，保持枪口航向的同时仍允许ID2回中。 */
        Dual_Yaw_Aim_RecenterHold(&g_dual_yaw_aim, small_angle,
                                  g_yaw_aim_angle_kp, g_yaw_aim_max_rpm);
    }
    *big_current = (int16_t)Yaw_LegacyPid_Delta(&g_big_yaw_speed_pid,
        g_dual_yaw_aim.big_ref_rpm, (float)big->speed_rpm);
    small_request = Yaw_AimSpeedPid(&g_yaw_aim_speed_pid,
        g_dual_yaw_aim.small_ref_rpm, (float)small->speed_rpm);
    /* 修改：软限位不仅限目标速度，也清除向外残余积分，向内恢复不受阻。 */
    if (((small_angle <= YAW_ANGLE_MIN_DEG) && (small_request < 0.0f)) ||
        ((small_angle >= YAW_ANGLE_MAX_DEG) && (small_request > 0.0f)))
    {
        small_request = 0.0f;
        Yaw_LegacyPid_Clear(&g_yaw_aim_speed_pid);
    }
    *small_current = (int16_t)small_request;
    g_big_yaw_debug_target_rpm = g_dual_yaw_aim.big_ref_rpm;
    g_big_yaw_debug_speed_rpm = big->speed_rpm;
    g_big_yaw_debug_current = *big_current;
    g_yaw_mode = YAW_MODE_AIM;
    g_yaw_debug_channel = 0;
    g_yaw_debug_angle10 = (int16_t)(small_angle * 10.0f);
    /* ID2调试目标仍是编码器角；相机惯性目标在AIM里的ct单独输出。 */
    g_yaw_debug_target10 = (int16_t)(small_angle * 10.0f);
    g_yaw_debug_error10 = (int16_t)(g_dual_yaw_aim.error_deg * 10.0f);
    g_yaw_debug_target_rpm = (int16_t)g_dual_yaw_aim.small_ref_rpm;
    g_yaw_debug_speed_rpm = small->speed_rpm;
    g_yaw_debug_current = *small_current;
}

/* 修改：左中右摇杆控制最终枪口航向，ID1随动接管，ID2持续往159.39度回中。
 * 底盘拨轮旋转也按实际转速补偿，不能只在小陀螺标志有效时补偿。 */
static void Manual_DualYaw_Update(const remote_control_t *remote,
                                   const remote_chassis_status_t *chassis,
                                   const dji_6020_feedback_t *big,
                                   const dji_6020_feedback_t *small,
                                   uint32_t now_ms,
                                   int16_t *big_current,
                                   int16_t *small_current)
{
    const imu_system_t *imu = IMU_System_GetData();
    float camera_yaw_deg = imu->ekf.yaw * 57.29578f;
    float small_angle = Pitch_Test_GetAngle(small);
    float small_request;
    int16_t rc_input = remote->channel[0];
    *big_current = 0;
    *small_current = 0;
    if ((imu->initialized == 0U) || !isfinite(camera_yaw_deg) ||
        (Pitch_Test_MotorOnline(big, now_ms) == 0U) ||
        (Pitch_Test_MotorOnline(small, now_ms) == 0U) ||
        ((chassis->update_tick_ms != 0U) &&
         ((uint32_t)(now_ms - chassis->update_tick_ms) > BIG_YAW_STATUS_TIMEOUT_MS)))
    {
        Dual_Yaw_ControlReset();
        return;
    }
    if (g_dual_yaw_aim.initialized == 0U)
    {
        g_follow_camera_target_deg = camera_yaw_deg;
    }
    if ((rc_input >= -YAW_RC_DEADZONE) && (rc_input <= YAW_RC_DEADZONE))
    {
        rc_input = 0;
    }
    /* 修改：沿用右摇杆55度/s；松杆不重置航向目标，ID1仍继续回中。 */
    g_follow_camera_target_deg = Yaw_ShortestError(
        g_follow_camera_target_deg + (float)rc_input * YAW_RC_STEP * YAW_RC_DIRECTION,
        0.0f);
    g_follow_camera_target_deg = Yaw_ShortestError(camera_yaw_deg +
        Pitch_Test_Clamp(Yaw_ShortestError(g_follow_camera_target_deg, camera_yaw_deg),
                          AIM_YAW_ERROR_MAX_DEG), 0.0f);
    g_follow_chassis_rpm = (chassis->update_tick_ms != 0U) ?
                           chassis->chassis_z_speed_rpm : 0.0f;
    Dual_Yaw_Follow_Update(&g_dual_yaw_aim, g_follow_camera_target_deg,
        camera_yaw_deg, small_angle, (float)big->speed_rpm, g_follow_chassis_rpm,
        YAW_ANGLE_KP, YAW_ANGLE_MAX_RPM, PITCH_CONTROL_PERIOD_S);
    g_dual_yaw_active = 1U;
    g_big_yaw_enabled = 0U;
    *big_current = (int16_t)Yaw_LegacyPid_Delta(&g_big_yaw_speed_pid,
        g_dual_yaw_aim.big_ref_rpm, (float)big->speed_rpm);
    small_request = Yaw_LegacyPid_Delta(&g_yaw_speed_pid,
        g_dual_yaw_aim.small_ref_rpm, (float)small->speed_rpm);
    if (((small_angle <= YAW_ANGLE_MIN_DEG) && (small_request < 0.0f)) ||
        ((small_angle >= YAW_ANGLE_MAX_DEG) && (small_request > 0.0f)))
    {
        small_request = 0.0f;
        Yaw_LegacyPid_Clear(&g_yaw_speed_pid);
    }
    *small_current = (int16_t)small_request;
    g_big_yaw_debug_target_rpm = g_dual_yaw_aim.big_ref_rpm;
    g_big_yaw_debug_speed_rpm = big->speed_rpm;
    g_big_yaw_debug_current = *big_current;
    g_yaw_mode = (rc_input != 0) ? YAW_MODE_MANUAL : YAW_MODE_HOLD;
    g_yaw_debug_channel = rc_input;
    g_yaw_debug_angle10 = (int16_t)(small_angle * 10.0f);
    g_yaw_debug_target10 = (int16_t)(DUAL_YAW_SMALL_CENTER_DEG * 10.0f);
    g_yaw_debug_error10 = (int16_t)(g_dual_yaw_aim.error_deg * 10.0f);
    g_yaw_debug_target_rpm = (int16_t)g_dual_yaw_aim.small_ref_rpm;
    g_yaw_debug_speed_rpm = small->speed_rpm;
    g_yaw_debug_current = *small_current;
}

static float Aim_ApproachZero(float value)
{
    float step = AIM_BRAKE_RPM_PER_S * PITCH_CONTROL_PERIOD_S;
    if (value > step) return value - step;
    if (value < -step) return value + step;
    return 0.0f;
}

static float Static_Pitch_TargetRpm(const vision_aim_command_t *aim,
                                     uint8_t fresh,
                                     float current_angle)
{
    if (g_pitch_aim_state == DUAL_YAW_AIM_OFF)
    {
        g_pitch_aim_state = DUAL_YAW_AIM_HOLD;
        g_pitch_target_angle = Pitch_Test_ClampAngle(current_angle);
        g_pitch_aim_target_rpm = 0.0f;
        g_aim_pitch_applied_frame_count = aim->new_frame_count - 1U;
    }
    if ((fresh != 0U) &&
        (g_aim_pitch_applied_frame_count != aim->new_frame_count))
    {
        /* 修改：新图像只锚定一次目标，重复seq不累计角度。 */
        g_pitch_target_angle = Pitch_Test_ClampAngle(current_angle +
            Pitch_Test_Clamp(AIM_PITCH_DIRECTION *
                (float)aim->pitch_error_cdeg / 100.0f, AIM_PITCH_ERROR_MAX_DEG));
        g_aim_pitch_applied_frame_count = aim->new_frame_count;
        g_pitch_aim_state = DUAL_YAW_AIM_TRACK;
    }
    else if ((fresh == 0U) && (g_pitch_aim_state == DUAL_YAW_AIM_TRACK))
    {
        g_pitch_aim_state = DUAL_YAW_AIM_BRAKE;
    }
    g_pitch_debug_error = Pitch_Test_Clamp(g_pitch_target_angle - current_angle,
                                            AIM_PITCH_ERROR_MAX_DEG);
    if (g_pitch_aim_state == DUAL_YAW_AIM_BRAKE)
    {
        g_pitch_aim_target_rpm = Aim_ApproachZero(g_pitch_aim_target_rpm);
        g_pitch_target_angle = Pitch_Test_ClampAngle(current_angle);
        g_pitch_debug_error = 0.0f;
        if ((g_pitch_aim_target_rpm == 0.0f) &&
            (abs(g_pitch_feedback.speed_rpm) < 2))
        {
            g_pitch_aim_state = DUAL_YAW_AIM_HOLD;
        }
    }
    else
    {
        g_pitch_aim_target_rpm = Yaw_LegacyPid_Position(&g_pitch_angle_pid,
                                                       g_pitch_debug_error);
        if (g_pitch_aim_state == DUAL_YAW_AIM_HOLD)
        {
            g_pitch_aim_target_rpm = Pitch_Test_Clamp(g_pitch_aim_target_rpm, 10.0f);
        }
    }
    return g_pitch_aim_target_rpm;
}

static int16_t Yaw_Update(const remote_control_t *remote,
                          const dji_6020_feedback_t *snapshot,
                          uint8_t aim_enabled,
                          uint32_t now_ms)
{
    dji_6020_feedback_t feedback = {0};
    float current_angle;
    float angle_error;
    float target_rpm;
    int16_t rc_input;

    /* 修改：此入口只保留手动/串口ID2控制；静止视觉由双Yaw入口独占。 */
    if ((Pitch_Test_MotorOnline(snapshot, now_ms) == 0U) ||
        ((g_yaw_serial_mode == 0U) &&
         ((aim_enabled != 0U) ||
          ((aim_enabled == 0U) &&
           ((g_pitch_remote_online == 0U) ||
            (g_pitch_remote_valid == 0U) ||
            ((remote->switch_position[RC_S_LEFT] != RC_SW_MID) &&
             (remote->switch_position[RC_S_LEFT] != RC_SW_UP)))))))
    {
        g_yaw_target_initialized = 0U;
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

    feedback = *snapshot;
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

    if (g_yaw_serial_mode != 0U)
    {
        /* 修改：串口阶跃沿用已调好的ID2视觉速度环。 */
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
    /* 修改：仅初始化诊断缓存和发送节拍，不改变任何控制目标。 */
    memset(&g_pitch_telemetry, 0, sizeof(g_pitch_telemetry));
    g_pitch_telemetry_valid = 0U;
    g_pitch_telemetry_previous_ms = HAL_GetTick();
    g_pitch_telemetry_fast_tick_ms = g_pitch_telemetry_previous_ms;
    g_pitch_telemetry_slow_tick_ms = g_pitch_telemetry_previous_ms;
    g_pitch_telemetry_sequence = 0U;
    g_pitch_telemetry_drop = 0U;
    g_pitch_serial_mode = 0U;
    g_pitch_serial_target_angle = 0.0f;
    g_pitch_serial_status_request = 0U;
    g_shoot_serial_reply[0] = '\0';
    g_shoot_serial_tx[0] = '\0';
    g_shoot_serial_status_request = 0U;
    g_shoot_serial_status_tick_ms = HAL_GetTick();
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
    g_aim_pitch_applied_frame_count = 0U;
    g_aim_active = 0U;
    g_static_aim_enabled = 0U;
    g_dual_yaw_active = 0U;
    g_pitch_aim_state = DUAL_YAW_AIM_OFF;
    g_pitch_aim_target_rpm = 0.0f;
    g_manual_yaw_follow = 0U;
    g_follow_camera_target_deg = 0.0f;
    g_follow_chassis_rpm = 0.0f;
    g_follow_heading_zero_deg = FOLLOW_HEADING_ZERO_DEG;
    g_follow_heading_calibrated = FOLLOW_HEADING_CALIBRATED;
    Dual_Yaw_Aim_Reset(&g_dual_yaw_aim);
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
    remote_chassis_status_t chassis = {0};
    vision_aim_command_t aim = {0};
    dji_6020_feedback_t big = {0};
    dji_6020_feedback_t small = {0};
    uint32_t now_ms;
    float current_angle;
    float angle_error;
    float current_request;
    float target_rpm = 0.0f;
    float telemetry_speed_p = 0.0f;
    float telemetry_speed_i = 0.0f;
    float telemetry_speed_output = 0.0f;
    int16_t pitch_current = 0;
    int16_t big_yaw_current = 0;
    int16_t yaw_current = 0;
    int16_t rc_input;
    uint8_t static_aim;
    uint8_t manual_follow;
    uint8_t pitch_control_enabled = 0U;

    /* 修改：先复制命令和所有反馈，再取本轮时间；控制不混用下一帧反馈。 */
    Vision_Aim_Service();
    Remote_Control_Get(&remote);
    Remote_Control_GetChassisStatus(&chassis);
    Vision_Aim_Get(&aim);
    DJI_6020_GetFeedback(PITCH_MOTOR_INDEX, &g_pitch_feedback);
    DJI_6020_GetFeedback(BIG_YAW_MOTOR_INDEX, &big);
    DJI_6020_GetFeedback(YAW_MOTOR_INDEX, &small);
    now_ms = HAL_GetTick();
    g_pitch_remote_online = (uint8_t)((remote.update_tick_ms != 0U) &&
        ((uint32_t)(now_ms - remote.update_tick_ms) <= PITCH_REMOTE_TIMEOUT_MS));
    g_pitch_remote_valid = Remote_Control_DataValid(&remote);
    g_pitch_motor_online = Pitch_Test_MotorOnline(&g_pitch_feedback, now_ms);
    current_angle = Pitch_Test_GetAngle(&g_pitch_feedback);
    Pitch_Test_SerialProcess(current_angle);
    g_pitch_debug_channel = remote.channel[1];
    g_pitch_debug_error = 0.0f;
    static_aim = (uint8_t)((aim.aim_enabled != 0U) &&
        (g_yaw_serial_mode == 0U) && (g_pitch_serial_mode == 0U) &&
        (Static_Aim_Allowed(&chassis, now_ms) != 0U));
    manual_follow = (uint8_t)((aim.aim_enabled == 0U) &&
        (g_yaw_serial_mode == 0U) && (g_pitch_serial_mode == 0U) &&
        (g_pitch_remote_online != 0U) && (g_pitch_remote_valid != 0U) &&
        (remote.switch_position[RC_S_LEFT] == RC_SW_MID));
    if ((static_aim != g_static_aim_enabled) || (manual_follow != g_manual_yaw_follow))
    {
        /* 修改：Yaw控制来源切换只清Yaw，不能连带丢掉Pitch的保持目标。 */
        Dual_Yaw_ControlReset();
        g_manual_yaw_follow = manual_follow;
    }
    if (static_aim != g_static_aim_enabled)
    {
        /* 修改：Pitch仅在视觉来源切换时清环，丢目标仍按BRAKE/HOLD处理。 */
        g_static_aim_enabled = static_aim;
        g_pitch_aim_state = DUAL_YAW_AIM_OFF;
        g_pitch_aim_target_rpm = 0.0f;
        g_pitch_target_initialized = 0U;
        g_pitch_integral = 0.0f;
        Yaw_LegacyPid_Clear(&g_pitch_angle_pid);
        Yaw_LegacyPid_Clear(&g_pitch_speed_pid);
    }
    g_aim_active = (uint8_t)((static_aim != 0U) &&
        (Pitch_Test_AimIsActive(&aim, now_ms) != 0U));

    /* 修改：串口测试优先；静止视觉使能后无目标也会进入保持。
     * 任一串口轴被接管时整组暂停视觉，避免ID1回中与测试互相打架。 */
    if ((g_pitch_motor_online == 0U) ||
        ((g_pitch_serial_mode == 0U) && (static_aim == 0U) &&
         ((aim.aim_enabled != 0U) || (g_pitch_remote_online == 0U) ||
          (g_pitch_remote_valid == 0U) ||
          ((remote.switch_position[RC_S_LEFT] != RC_SW_MID) &&
           (remote.switch_position[RC_S_LEFT] != RC_SW_UP)))))
    {
        g_pitch_target_initialized = 0U;
        g_pitch_aim_state = DUAL_YAW_AIM_OFF;
        g_pitch_aim_target_rpm = 0.0f;
        g_pitch_debug_target_rpm = 0;
        g_pitch_integral = 0.0f;
        Yaw_LegacyPid_Clear(&g_pitch_angle_pid);
        Yaw_LegacyPid_Clear(&g_pitch_speed_pid);
        g_pitch_target_angle = current_angle;
    }
    else if ((g_pitch_serial_mode != 0U) || (static_aim != 0U))
    {
        pitch_control_enabled = 1U;
        g_pitch_target_initialized = 1U;
        if (g_pitch_serial_mode != 0U)
        {
            g_pitch_target_angle = Pitch_Test_ClampAngle(g_pitch_serial_target_angle);
            g_pitch_debug_error = g_pitch_target_angle - current_angle;
            target_rpm = Yaw_LegacyPid_Position(&g_pitch_angle_pid,
                                                 g_pitch_debug_error);
        }
        else
        {
            target_rpm = Static_Pitch_TargetRpm(&aim, g_aim_active, current_angle);
        }
        if (((current_angle <= PITCH_ANGLE_MIN_DEG) && (target_rpm < 0.0f)) ||
            ((current_angle >= PITCH_ANGLE_MAX_DEG) && (target_rpm > 0.0f)))
        {
            target_rpm = 0.0f;
            g_pitch_aim_target_rpm = 0.0f;
            Yaw_LegacyPid_Clear(&g_pitch_speed_pid);
        }
        g_pitch_debug_target_rpm = (int16_t)target_rpm;
        current_request = Pitch_SpeedPid_Position(&g_pitch_speed_pid, target_rpm,
            (float)g_pitch_feedback.speed_rpm * PITCH_SPEED_FEEDBACK_DIRECTION);
        /* 修改：只复制此次实际使用的PI分量；后续限位清积分也不混入下一轮数据。 */
        telemetry_speed_p = g_pitch_speed_pid.kp * (target_rpm -
            (float)g_pitch_feedback.speed_rpm * PITCH_SPEED_FEEDBACK_DIRECTION);
        telemetry_speed_i = g_pitch_speed_pid.iout;
        telemetry_speed_output = current_request;
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
        rc_input = remote.channel[1];
        if ((rc_input < PITCH_RC_DEADZONE) && (rc_input > -PITCH_RC_DEADZONE))
        {
            rc_input = 0;
        }
        g_pitch_target_angle += (float)rc_input * PITCH_RC_STEP * PITCH_RC_DIRECTION;
        g_pitch_target_angle = Pitch_Test_ClampAngle(g_pitch_target_angle);
        angle_error = g_pitch_target_angle - current_angle;
        g_pitch_debug_error = angle_error;
        current_request = angle_error * PITCH_ANGLE_KP + g_pitch_integral;
        if (((current_request < PITCH_CURRENT_LIMIT) &&
             (current_request > -PITCH_CURRENT_LIMIT)) ||
            ((current_request >= PITCH_CURRENT_LIMIT) && (angle_error < 0.0f)) ||
            ((current_request <= -PITCH_CURRENT_LIMIT) && (angle_error > 0.0f)))
        {
            g_pitch_integral = Pitch_Test_Clamp(g_pitch_integral +
                angle_error * PITCH_ANGLE_KI * PITCH_CONTROL_PERIOD_S,
                PITCH_INTEGRAL_LIMIT);
        }
        pitch_current = (int16_t)Pitch_Test_Clamp(
            (angle_error * PITCH_ANGLE_KP + g_pitch_integral) * PITCH_MOTOR_DIRECTION,
            PITCH_CURRENT_LIMIT);
    }
    if (pitch_control_enabled != 0U)
    {
        /* 修改：沿用已采集的重力/摩擦补偿表和原来的比例，不重新拟合。 */
        g_pitch_gravity_ff = Pitch_Test_GravityFeedforward(current_angle);
        g_pitch_friction_ff = Pitch_Test_FrictionFeedforward(
            (int16_t)((float)g_pitch_feedback.speed_rpm * PITCH_SPEED_FEEDBACK_DIRECTION),
            g_pitch_debug_error);
        pitch_current = (int16_t)Pitch_Test_Clamp(
            (float)pitch_current + g_pitch_gravity_ff + g_pitch_friction_ff,
            PITCH_CURRENT_LIMIT);
        /* 修改：到达软限位只禁止继续向外，允许向内脱离限位。 */
        if (((current_angle <= PITCH_ANGLE_MIN_DEG) && (pitch_current < 0)) ||
            ((current_angle >= PITCH_ANGLE_MAX_DEG) && (pitch_current > 0)))
        {
            pitch_current = 0;
            g_pitch_integral = 0.0f;
            Yaw_LegacyPid_Clear(&g_pitch_speed_pid);
        }
    }
    else
    {
        g_pitch_gravity_ff = 0.0f;
        g_pitch_friction_ff = 0.0f;
    }

    if (static_aim != 0U)
    {
        /* 修改：视觉HOLD回中同样由自瞄状态决定，不再要求左中拨杆。 */
        Static_DualYaw_Update(&aim, g_aim_active, 1U,
                              &big, &small, now_ms,
                              &big_yaw_current, &yaw_current);
    }
    else if (manual_follow != 0U)
    {
        Manual_DualYaw_Update(&remote, &chassis, &big, &small, now_ms,
                              &big_yaw_current, &yaw_current);
    }
    else
    {
        /* 修改：原小陀螺补偿保留；旧ID2入口只允许遥控/串口，不接视觉。 */
        big_yaw_current = Big_Yaw_Update();
        yaw_current = Yaw_Update(&remote, &small, aim.aim_enabled, now_ms);
    }
    g_pitch_debug_current = pitch_current;
    /* 修改：同一CAN帧统一发送ID1/ID2/ID3，两个Yaw来源互斥。 */
    if (DJI_6020_SendCurrent(&hcan2, big_yaw_current, yaw_current,
                             pitch_current, 0) != HAL_OK)
    {
        g_pitch_can_tx_fail++;
    }
    /* 修改：在控制输出确定后冻结本周期诊断，不在串口发送函数内拼接新旧反馈。 */
    g_pitch_telemetry.ms = now_ms;
    g_pitch_telemetry.src = (pitch_control_enabled == 0U) ? 0U :
        ((g_pitch_serial_mode != 0U) ? 3U : ((static_aim != 0U) ? 2U : 1U));
    g_pitch_telemetry.en = aim.aim_enabled;
    g_pitch_telemetry.v = aim.target_valid;
    g_pitch_telemetry.fr = g_aim_active;
    g_pitch_telemetry.st = g_pitch_aim_state;
    g_pitch_telemetry.seq = aim.seq;
    g_pitch_telemetry.nf = aim.new_frame_count;
    g_pitch_telemetry.age = (aim.new_frame_count != 0U) ?
        (uint32_t)(now_ms - aim.last_new_target_tick_ms) : UINT32_MAX;
    g_pitch_telemetry.a = (int32_t)lroundf(current_angle * 100.0f);
    g_pitch_telemetry.t = (int32_t)lroundf(g_pitch_target_angle * 100.0f);
    g_pitch_telemetry.ve = aim.pitch_error_cdeg;
    g_pitch_telemetry.vv = aim.pitch_velocity_cdeg_s;
    g_pitch_telemetry.e = (int32_t)lroundf(g_pitch_debug_error * 100.0f);
    g_pitch_telemetry.rt = (int32_t)lroundf(target_rpm * 100.0f);
    g_pitch_telemetry.r = (int32_t)lroundf(
        (float)g_pitch_feedback.speed_rpm * PITCH_SPEED_FEEDBACK_DIRECTION);
    g_pitch_telemetry.u = pitch_current;
    g_pitch_telemetry.fb = g_pitch_feedback.given_current;
    g_pitch_telemetry.p = (int32_t)lroundf(telemetry_speed_p);
    g_pitch_telemetry.i = (int32_t)lroundf(telemetry_speed_i);
    g_pitch_telemetry.po = (int32_t)lroundf(telemetry_speed_output);
    g_pitch_telemetry.g = (int32_t)lroundf(g_pitch_gravity_ff);
    g_pitch_telemetry.f = (int32_t)lroundf(g_pitch_friction_ff);
    g_pitch_telemetry.lim = (current_angle <= PITCH_ANGLE_MIN_DEG) ? -1 :
        ((current_angle >= PITCH_ANGLE_MAX_DEG) ? 1 : 0);
    g_pitch_telemetry.on = g_pitch_motor_online;
    g_pitch_telemetry.fail = g_pitch_can_tx_fail;
    g_pitch_telemetry.dt = (uint32_t)(now_ms - g_pitch_telemetry_previous_ms);
    g_pitch_telemetry.fa = (g_pitch_feedback.update_tick_ms != 0U) ?
        (uint32_t)(now_ms - g_pitch_feedback.update_tick_ms) : UINT32_MAX;
    g_pitch_telemetry_previous_ms = now_ms;
    g_pitch_telemetry_valid = 1U;
    if (g_pitch_serial_status_request != 0U)
    {
        g_pitch_serial_status_request = 0U;
        Pitch_Test_DebugPrint();
    }
}

/* 修改：独立诊断调度，概要到期优先发；DMA忙只丢PDBG并记drop，控制环不等待。 */
void Pitch_Test_TelemetryService(void)
{
    static char message[512];
    static char shoot_message[96];
    const pitch_telemetry_t *s = &g_pitch_telemetry;
    uint32_t now_ms = HAL_GetTick();
    int length;

    /* 修改：发射应答和按需快照优先于周期遥测，所有发送均不等待DMA。 */
    if (g_shoot_serial_reply[0] != '\0')
    {
        if (huart6.gState == HAL_UART_STATE_READY)
        {
            size_t reply_length = strlen(g_shoot_serial_reply);
            memcpy(g_shoot_serial_tx, g_shoot_serial_reply, reply_length + 1U);
            if (HAL_UART_Transmit_DMA(&huart6, (uint8_t *)g_shoot_serial_tx,
                                      (uint16_t)reply_length) == HAL_OK)
            {
                g_shoot_serial_reply[0] = '\0';
            }
        }
        if (g_shoot_serial_reply[0] != '\0')
        {
            return;
        }
    }
    if ((g_shoot_serial_status_request != 0U) &&
        (huart6.gState == HAL_UART_STATE_READY))
    {
        Shoot_Test_DebugPrint();
        if (huart6.gState != HAL_UART_STATE_READY)
        {
            g_shoot_serial_status_request = 0U;
        }
    }
    /* 修改：独立FIRE状态每100ms输出，任何Yaw/自瞄模式下都能操作发射按钮。
     * mode=0无串口动作、1拨弹2秒、2三电机1秒；left单位ms，seq只在成功启动时递增。 */
    if (((uint32_t)(now_ms - g_shoot_serial_status_tick_ms) >= 100U) &&
        (huart6.gState == HAL_UART_STATE_READY))
    {
        uint8_t shoot_mode;
        uint8_t shoot_locked;
        uint32_t shoot_left_ms;
        uint32_t shoot_sequence;
        Shoot_Test_GetSerialStatus(&shoot_mode, &shoot_left_ms,
                                   &shoot_locked, &shoot_sequence);
        length = snprintf(shoot_message, sizeof(shoot_message),
            "FIRE:ms=%lu mode=%u left=%lu lock=%u seq=%lu\r\n",
            (unsigned long)now_ms, (unsigned int)shoot_mode,
            (unsigned long)shoot_left_ms, (unsigned int)shoot_locked,
            (unsigned long)shoot_sequence);
        if ((length > 0) && (length < (int)sizeof(shoot_message)) &&
            (HAL_UART_Transmit_DMA(&huart6, (uint8_t *)shoot_message,
                                  (uint16_t)length) == HAL_OK))
        {
            g_shoot_serial_status_tick_ms = now_ms;
        }
    }
    if (g_pitch_telemetry_valid == 0U)
    {
        return;
    }
    if (((uint32_t)(now_ms - g_pitch_telemetry_slow_tick_ms) >=
         Pitch_Test_DebugPeriodMs()) && (huart6.gState == HAL_UART_STATE_READY))
    {
        g_pitch_telemetry_slow_tick_ms = now_ms;
        Pitch_Test_DebugPrint();
    }
    if ((uint32_t)(now_ms - g_pitch_telemetry_fast_tick_ms) <
        PITCH_TELEMETRY_PERIOD_MS)
    {
        return;
    }
    g_pitch_telemetry_fast_tick_ms = now_ms;
    g_pitch_telemetry_sequence++;
    if (huart6.gState != HAL_UART_STATE_READY)
    {
        g_pitch_telemetry_drop++;
        return;
    }

    /* 修改：整数键值行供网页直接解析。概要/命令与此缓冲均只在UART空闲时写入。 */
    length = snprintf(message, sizeof(message),
        "PDBG:ms=%lu n=%lu src=%u en=%u v=%u fr=%u st=%u seq=%u nf=%lu age=%lu "
        "a=%ld t=%ld ve=%ld vv=%ld e=%ld rt=%ld r=%ld u=%ld fb=%ld "
        "p=%ld i=%ld po=%ld g=%ld f=%ld lim=%ld on=%u fail=%lu drop=%lu dt=%lu fa=%lu\r\n",
        (unsigned long)s->ms, (unsigned long)g_pitch_telemetry_sequence,
        (unsigned int)s->src, (unsigned int)s->en, (unsigned int)s->v,
        (unsigned int)s->fr, (unsigned int)s->st, (unsigned int)s->seq,
        (unsigned long)s->nf, (unsigned long)s->age,
        (long)s->a, (long)s->t, (long)s->ve, (long)s->vv, (long)s->e,
        (long)s->rt, (long)s->r, (long)s->u, (long)s->fb,
        (long)s->p, (long)s->i, (long)s->po, (long)s->g, (long)s->f,
        (long)s->lim, (unsigned int)s->on, (unsigned long)s->fail,
        (unsigned long)g_pitch_telemetry_drop, (unsigned long)s->dt,
        (unsigned long)s->fa);
    if ((length <= 0) || (length >= (int)sizeof(message)) ||
        (HAL_UART_Transmit_DMA(&huart6, (uint8_t *)message,
                               (uint16_t)length) != HAL_OK))
    {
        g_pitch_telemetry_drop++;
    }
}

uint32_t Pitch_Test_DebugPeriodMs(void)
{
    /* 修改：完整自瞄概要100ms；旧BY/YAW调参模式仍保留各自输出。 */
    if ((g_static_aim_enabled != 0U) || (g_manual_yaw_follow != 0U))
        return PITCH_DEBUG_PERIOD_MS;
    /* 小陀螺自动启用快照；BY GET可继续记录停车和换向过程。 */
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
    static char message[768];
    uint32_t now_ms = HAL_GetTick();
    uint32_t frame_age_ms;
    float yaw_angle;
    float pitch_angle;
    int yaw_limit;
    int pitch_limit;
    int length;

    /* 恢复完整自瞄输出：上一条 DMA 未发完时，不改写发送缓冲。 */
    if (huart6.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    if ((g_static_aim_enabled == 0U) && (g_manual_yaw_follow == 0U) &&
        ((g_big_yaw_enabled != 0U) || (g_big_yaw_debug_enabled != 0U)))
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

    if ((g_static_aim_enabled == 0U) && (g_manual_yaw_follow == 0U) &&
        (g_yaw_debug_enabled != 0U))
    {
        dji_6020_feedback_t yaw = {0};
        DJI_6020_GetFeedback(YAW_MOTOR_INDEX, &yaw);
        now_ms = HAL_GetTick();
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
    now_ms = HAL_GetTick();
    frame_age_ms = (aim.new_frame_count != 0U) ?
        (uint32_t)(now_ms - aim.last_new_target_tick_ms) : UINT32_MAX;
    yaw_angle = Pitch_Test_GetAngle(&yaw_feedback);
    pitch_angle = Pitch_Test_GetAngle(&pitch_feedback);
    yaw_limit = (yaw_angle <= YAW_ANGLE_MIN_DEG) ? -1 :
                ((yaw_angle >= YAW_ANGLE_MAX_DEG) ? 1 : 0);
    pitch_limit = (pitch_angle <= PITCH_ANGLE_MIN_DEG) ? -1 :
                  ((pitch_angle >= PITCH_ANGLE_MAX_DEG) ? 1 : 0);

    /* 修改：原始协议保持右正/上正；DYA目标/速度为控制坐标的逆时针正。
     * state：0停用、1跟踪、2刹停、3保持。age为新图像年龄，rxage为通信年龄。
     * yv/ya/pv完整接收并显示，本版前馈系数为0；ct是相机航向，不是ID2编码器角。
     * P:rpm保留原始反馈，sd是控制速度方向系数；PDBG:r已转换到控制方向。 */
    length = snprintf(message, sizeof(message),
        "AIM:ms=%lu en=%u active=%u valid=%u fresh=%u seq=%u cap=%lu "
        "ye=%.2f yv=%.2f ya=%.2f pe=%.2f pv=%.2f age=%lu rxage=%lu "
        "bytes=%lu frames=%lu new=%lu bad=%lu "
        "DYA:en=%u on=%u state=%u cy=%.2f ct=%.2f ce=%.2f total=%.2f "
        "FOLLOW:on=%u cal=%u zero=%.5f base=%.2f "
        "B:on=%u trpm=%.2f rpm=%d cmd=%d "
        "S:on=%u a=%.2f trpm=%.2f rpm=%d cmd=%d lim=%d "
        "P:on=%u state=%u a=%.2f t=%.2f e=%.2f trpm=%d rpm=%d sd=%d cmd=%d fbI=%d lim=%d "
        "txfail=%lu IMU:roll=%.2f\r\n",
        (unsigned long)now_ms,
        (unsigned int)aim.aim_enabled, (unsigned int)g_aim_active,
        (unsigned int)aim.target_valid,
        (unsigned int)Pitch_Test_AimIsActive(&aim, now_ms),
        (unsigned int)aim.seq, (unsigned long)aim.capture_time_ms,
        (double)aim.yaw_error_cdeg / 100.0,
        (double)aim.yaw_velocity_cdeg_s / 100.0,
        (double)aim.yaw_acceleration_cdeg_s2 / 100.0,
        (double)aim.pitch_error_cdeg / 100.0,
        (double)aim.pitch_velocity_cdeg_s / 100.0,
        (unsigned long)frame_age_ms,
        (unsigned long)((aim.valid_frame_count != 0U) ?
            (uint32_t)(now_ms - aim.last_rx_tick_ms) : UINT32_MAX),
        (unsigned long)aim.rx_byte_count,
        (unsigned long)aim.valid_frame_count,
        (unsigned long)aim.new_frame_count,
        (unsigned long)aim.checksum_error_count,
        (unsigned int)g_static_aim_enabled, (unsigned int)g_dual_yaw_active,
        (unsigned int)g_dual_yaw_aim.state,
        (double)(imu->ekf.yaw * 57.29578f),
        (double)g_dual_yaw_aim.camera_target_deg,
        (double)g_dual_yaw_aim.error_deg, (double)g_dual_yaw_aim.total_rpm,
        (unsigned int)g_manual_yaw_follow,
        (unsigned int)g_follow_heading_calibrated,
        (double)g_follow_heading_zero_deg, (double)g_follow_chassis_rpm,
        (unsigned int)Pitch_Test_MotorOnline(&big_yaw_feedback, now_ms),
        (double)g_big_yaw_debug_target_rpm,
        (int)big_yaw_feedback.speed_rpm, (int)g_big_yaw_debug_current,
        (unsigned int)Pitch_Test_MotorOnline(&yaw_feedback, now_ms),
        (double)yaw_angle, (double)((g_dual_yaw_active != 0U) ?
            g_dual_yaw_aim.small_ref_rpm : (float)g_yaw_debug_target_rpm),
        (int)yaw_feedback.speed_rpm, (int)g_yaw_debug_current, yaw_limit,
        (unsigned int)Pitch_Test_MotorOnline(&pitch_feedback, now_ms),
        (unsigned int)g_pitch_aim_state,
        (double)pitch_angle, (double)g_pitch_target_angle,
        (double)g_pitch_debug_error, (int)g_pitch_debug_target_rpm,
        (int)pitch_feedback.speed_rpm, (int)PITCH_SPEED_FEEDBACK_DIRECTION,
        (int)g_pitch_debug_current, (int)pitch_feedback.given_current, pitch_limit,
        (unsigned long)g_pitch_can_tx_fail, (double)(imu->ekf.roll * 57.29578f));
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

/* 修改：使用两个绝对编码器合成相对朝向，不直接相减独立初始化的两套IMU。 */
uint8_t Pitch_Test_GetRelativeHeading(int16_t *angle_cdeg,
                                      uint16_t *big_ecd,
                                      uint16_t *small_ecd)
{
    dji_6020_feedback_t big = {0};
    dji_6020_feedback_t small = {0};
    uint32_t now_ms;
    DJI_6020_GetFeedback(BIG_YAW_MOTOR_INDEX, &big);
    DJI_6020_GetFeedback(YAW_MOTOR_INDEX, &small);
    now_ms = HAL_GetTick();
    *big_ecd = big.ecd;
    *small_ecd = small.ecd;
    *angle_cdeg = (int16_t)lroundf(Yaw_ShortestError(
        Pitch_Test_GetAngle(&big) + Pitch_Test_GetAngle(&small),
        g_follow_heading_zero_deg) * 100.0f);
    return (uint8_t)((g_follow_heading_calibrated != 0U) &&
        (Pitch_Test_MotorOnline(&big, now_ms) != 0U) &&
        (Pitch_Test_MotorOnline(&small, now_ms) != 0U));
}
