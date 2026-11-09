#include "shoot_test.h"

#include <stdio.h>

#include "bsp_can.h"
#include "can.h"
#include "dji_motor.h"
#include "pid.h"
#include "pitch_test.h"
#include "remote_control.h"
#include "usart.h"

#define SHOOT_MOTOR_COUNT       3U
#define SHOOT_MOTOR_TIMEOUT_MS  100U
#define SHOOT_REMOTE_TIMEOUT_MS 200U
#define SHOOT_SERIAL_FIRE_MS   1000U
#define SHOOT_SERIAL_FEED_MS   2000U

/* CAN1 0x200: ID1不用，ID2左摩擦轮，ID3右摩擦轮，ID4拨弹轮。 */
/* 实车确认：ID4 拨弹轮需要反向旋转。 */
/* 修改：摩擦轮保持6750rpm；串口定时拨弹仍沿原方向使用1200rpm。 */
#define TRIGGER_TARGET_RPM     (-1200)
#define TRIGGER_REMOTE_MAX_RPM  3000
#define TRIGGER_DIAL_RANGE      660
#define TRIGGER_DIAL_DEADZONE   10
#define LEFT_FRIC_TARGET_RPM   (-6750)
#define RIGHT_FRIC_TARGET_RPM   6750

/* 控制数组顺序固定为：拨弹轮ID4、左摩擦轮ID2、右摩擦轮ID3。 */
static const uint8_t g_feedback_index[SHOOT_MOTOR_COUNT] = {3U, 1U, 2U};

typedef enum
{
    SHOOT_MODE_STOP = 0,
    SHOOT_MODE_TRIGGER,
    SHOOT_MODE_FIRE
} shoot_mode_t;

static pid_t g_speed_pid[SHOOT_MOTOR_COUNT];
static dji_motor_feedback_t g_feedback[SHOOT_MOTOR_COUNT];
static int16_t g_target_rpm[SHOOT_MOTOR_COUNT];
static int16_t g_output_current[SHOOT_MOTOR_COUNT];
static shoot_mode_t g_mode;
static uint8_t g_online_mask;
static uint8_t g_remote_online;
static uint8_t g_remote_valid;
static uint8_t g_output_enabled;
static uint32_t g_can_tx_fail_count;
/* 修改：串口动作由本机计时，断开串口或遥控器都不会续期。 */
static uint8_t g_serial_mode;
static uint8_t g_serial_locked;
static uint32_t g_serial_start_ms;
static uint32_t g_serial_duration_ms;
static uint32_t g_serial_seq;

/* 修改：遥控拨轮回中停拨弹，两侧比例调速；正值沿原负转速送弹，负值反转。
 * 有效遥控允许到±700，先按额定±660钳制，确保目标不超过±3000rpm。 */
static int16_t Shoot_Test_DialTarget(int16_t input)
{
    if ((input >= -TRIGGER_DIAL_DEADZONE) && (input <= TRIGGER_DIAL_DEADZONE))
    {
        return 0;
    }
    if (input > TRIGGER_DIAL_RANGE)
    {
        input = TRIGGER_DIAL_RANGE;
    }
    else if (input < -TRIGGER_DIAL_RANGE)
    {
        input = -TRIGGER_DIAL_RANGE;
    }
    return (int16_t)(-(int32_t)input * TRIGGER_REMOTE_MAX_RPM / TRIGGER_DIAL_RANGE);
}

static void Shoot_Test_ClearControl(void)
{
    uint8_t i;

    for (i = 0U; i < SHOOT_MOTOR_COUNT; i++)
    {
        g_target_rpm[i] = 0;
        g_output_current[i] = 0;
        PID_Clear(&g_speed_pid[i]);
    }
}

static const char *Shoot_Test_ModeName(shoot_mode_t mode)
{
    if (mode == SHOOT_MODE_TRIGGER)
    {
        return "TRIGGER";
    }
    if (mode == SHOOT_MODE_FIRE)
    {
        return "FIRE";
    }
    return "STOP";
}

uint8_t Shoot_Test_SerialStart(uint8_t mode)
{
    if ((g_serial_mode != 0U) ||
        ((mode != SHOOT_MODE_TRIGGER) && (mode != SHOOT_MODE_FIRE)))
    {
        return 0U;
    }
    /* 修改：串口接管优先于任何遥控档位，清除之前的遥控积分。 */
    Shoot_Test_ClearControl();
    g_serial_start_ms = HAL_GetTick();
    g_serial_duration_ms = (mode == SHOOT_MODE_TRIGGER) ?
        SHOOT_SERIAL_FEED_MS : SHOOT_SERIAL_FIRE_MS;
    g_serial_mode = mode;
    g_serial_locked = 1U;
    g_serial_seq++;
    return 1U;
}

void Shoot_Test_SerialStop(void)
{
    /* 修改：停止后锁住遥控接管，必须收到有效的遥控STOP才重新放行。
     * 清零值由同轮Shoot_Test_Update统一发送CAN，不等待串口应答。 */
    g_serial_mode = 0U;
    g_serial_locked = 1U;
    g_mode = SHOOT_MODE_STOP;
    g_output_enabled = 0U;
    Shoot_Test_ClearControl();
}

void Shoot_Test_GetSerialStatus(uint8_t *mode, uint32_t *remain_ms,
                                uint8_t *locked, uint32_t *seq)
{
    uint32_t elapsed_ms = (uint32_t)(HAL_GetTick() - g_serial_start_ms);

    if (mode != 0)
    {
        *mode = g_serial_mode;
    }
    if (remain_ms != 0)
    {
        *remain_ms = ((g_serial_mode != 0U) && (elapsed_ms < g_serial_duration_ms)) ?
            g_serial_duration_ms - elapsed_ms : 0U;
    }
    if (locked != 0)
    {
        *locked = g_serial_locked;
    }
    if (seq != 0)
    {
        *seq = g_serial_seq;
    }
}

static const char *Shoot_Test_RemoteSourceName(uint8_t source)
{
    if (source == RC_SOURCE_DIRECT)
    {
        return "DIRECT";
    }
    if (source == RC_SOURCE_FORWARD)
    {
        return "FORWARD";
    }
    return "NONE";
}

void Shoot_Test_Init(void)
{
    if (BSP_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* 3508走CAN1；启动CAN2，供main中的Pitch_Test控制6020。 */
    if (BSP_CAN_Init(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }

    Remote_Control_Init();

    /* 参数沿用旧工程：拨弹轮限流8000，两个摩擦轮限流16000。 */
    PID_Init(&g_speed_pid[0], 5.0f, 0.0f, 0.0f, 1000.0f, 8000.0f);
    PID_Init(&g_speed_pid[1], 2.5f, 0.1f, 0.0f, 1000.0f, 16000.0f);
    PID_Init(&g_speed_pid[2], 2.5f, 0.1f, 0.0f, 1000.0f, 16000.0f);

    /* 修改：上电不自动执行串口动作，默认仍使用原遥控逻辑。 */
    g_serial_mode = 0U;
    g_serial_locked = 0U;
    g_serial_start_ms = 0U;
    g_serial_duration_ms = 0U;
    g_serial_seq = 0U;
    g_mode = SHOOT_MODE_STOP;
    g_output_enabled = 0U;
    Shoot_Test_ClearControl();
    (void)DJI_Motor_SendChassisCurrent(&hcan1, 0, 0, 0, 0);
    (void)DJI_6020_SendCurrent(&hcan2, 0, 0, 0, 0);
}

void Shoot_Test_Update(void)
{
    remote_control_t remote = {0};
    uint32_t now_ms;
    uint8_t i;
    shoot_mode_t remote_mode = SHOOT_MODE_STOP;

    Remote_Control_Get(&remote);
    /* 修改：先取遥控和反馈快照，再读时间；避免跨毫秒新反馈被误判离线。 */
    for (i = 0U; i < SHOOT_MOTOR_COUNT; i++)
    {
        DJI_Motor_GetFeedback(g_feedback_index[i], &g_feedback[i]);
    }
    now_ms = HAL_GetTick();
    g_remote_online = (uint8_t)((remote.update_tick_ms != 0U) &&
        ((uint32_t)(now_ms - remote.update_tick_ms) <= SHOOT_REMOTE_TIMEOUT_MS));
    g_remote_valid = Remote_Control_DataValid(&remote);

    g_online_mask = 0U;
    for (i = 0U; i < SHOOT_MOTOR_COUNT; i++)
    {
        if ((g_feedback[i].update_tick_ms != 0U) &&
            ((uint32_t)(now_ms - g_feedback[i].update_tick_ms) <= SHOOT_MOTOR_TIMEOUT_MS))
        {
            g_online_mask |= (uint8_t)(1U << g_feedback_index[i]);
        }
    }

    /* 修改：左中右中只开拨弹，左中右上持续开摩擦轮；两档拨弹均由拨轮调速。 */
    if ((g_remote_online != 0U) && (g_remote_valid != 0U) &&
        (remote.switch_position[RC_S_LEFT] == RC_SW_MID))
    {
        if (remote.switch_position[RC_S_RIGHT] == RC_SW_MID)
        {
            remote_mode = SHOOT_MODE_TRIGGER;
        }
        else if (remote.switch_position[RC_S_RIGHT] == RC_SW_UP)
        {
            remote_mode = SHOOT_MODE_FIRE;
        }
    }

    /* 修改：先处理到期，再选择唯一控制来源；重复命令不触碰开始时刻。 */
    if ((g_serial_mode != 0U) &&
        ((uint32_t)(now_ms - g_serial_start_ms) >= g_serial_duration_ms))
    {
        Shoot_Test_SerialStop();
    }
    if (g_serial_mode != 0U)
    {
        g_mode = (shoot_mode_t)g_serial_mode;
    }
    else if (g_serial_locked != 0U)
    {
        /* 修改：到期/STOP时不直接恢复仍在拨弹或发射档的遥控。
         * 遥控离线不算回STOP；新的串口动作仍可以独立启动。 */
        g_mode = SHOOT_MODE_STOP;
        if ((g_remote_online != 0U) && (g_remote_valid != 0U) &&
            (remote_mode == SHOOT_MODE_STOP))
        {
            g_serial_locked = 0U;
        }
    }
    else
    {
        g_mode = remote_mode;
    }

    /* 三台3508仍分别判断在线；串口动作不需要遥控器参与。 */
    g_output_enabled = (uint8_t)(g_mode != SHOOT_MODE_STOP);
    if (g_output_enabled == 0U)
    {
        Shoot_Test_ClearControl();
    }
    else
    {
        /* 修改：串口动作独立于拨轮；遥控FIRE在拨轮回中时仅停拨弹，摩擦轮仍转。 */
        int16_t trigger_target = (g_serial_mode != 0U) ? TRIGGER_TARGET_RPM :
                                 Shoot_Test_DialTarget(remote.channel[4]);
        g_target_rpm[0] = ((g_mode == SHOOT_MODE_TRIGGER) ||
                           (g_mode == SHOOT_MODE_FIRE)) ?
                          trigger_target : 0;
        g_target_rpm[1] = (g_mode == SHOOT_MODE_FIRE) ?
                          LEFT_FRIC_TARGET_RPM : 0;
        g_target_rpm[2] = (g_mode == SHOOT_MODE_FIRE) ?
                          RIGHT_FRIC_TARGET_RPM : 0;

        for (i = 0U; i < SHOOT_MOTOR_COUNT; i++)
        {
            if ((g_target_rpm[i] == 0) ||
                ((g_online_mask & (uint8_t)(1U << g_feedback_index[i])) == 0U))
            {
                g_output_current[i] = 0;
                PID_Clear(&g_speed_pid[i]);
            }
            else
            {
                g_output_current[i] = (int16_t)PID_Update(
                    &g_speed_pid[i],
                    (float)g_target_rpm[i],
                    (float)g_feedback[i].speed_rpm);
            }
        }
    }

    if (DJI_Motor_SendChassisCurrent(&hcan1,
                                     0,
                                     g_output_current[1],
                                     g_output_current[2],
                                     g_output_current[0]) != HAL_OK)
    {
        g_can_tx_fail_count++;
    }
}

void Shoot_Test_DebugPrint(void)
{
    remote_control_t remote = {0};
    dji_6020_feedback_t pitch_feedback = {0};
    dji_6020_feedback_t yaw_feedback = {0};
    uint32_t pitch_angle10;
    uint32_t yaw_angle10_display;
    uint8_t remote_source;
    uint8_t serial_mode;
    uint8_t serial_locked;
    uint32_t serial_left_ms;
    uint32_t serial_seq;
    uint8_t yaw_mode;
    int16_t yaw_channel;
    int16_t yaw_angle10_debug;
    int16_t yaw_target10;
    int16_t yaw_error10;
    int16_t yaw_target_rpm;
    int16_t yaw_speed_rpm;
    int16_t yaw_current;
    static char message[512];
    int length;

    /* 修改：SHOOT GET按需查看状态，上一条DMA未发完时不覆盖发送缓冲。 */
    if (huart6.gState != HAL_UART_STATE_READY)
    {
        return;
    }
    Remote_Control_Get(&remote);
    remote_source = Remote_Control_GetSource();
    Shoot_Test_GetSerialStatus(&serial_mode, &serial_left_ms, &serial_locked, &serial_seq);
    /* 两个6020角度合并到同一条DMA输出，避免多次发送相互占用。 */
    DJI_6020_GetFeedback(2U, &pitch_feedback);
    /* 当前 Yaw 使用 6020 ID2，反馈数组下标为1；ID1不控制。 */
    DJI_6020_GetFeedback(1U, &yaw_feedback);
    Pitch_Test_GetYawDebug(&yaw_mode,
                           &yaw_channel,
                           &yaw_angle10_debug,
                           &yaw_target10,
                           &yaw_error10,
                           &yaw_target_rpm,
                           &yaw_speed_rpm,
                           &yaw_current);
    pitch_angle10 = ((uint32_t)pitch_feedback.ecd * 3600U + 4096U) / 8192U;
    yaw_angle10_display = ((uint32_t)yaw_feedback.ecd * 3600U + 4096U) / 8192U;
    length = snprintf(message, sizeof(message),
                      "SHOOT:src=%s mode=%s serial=%u left=%lu lock=%u seq=%lu "
                      "en=%u rc=%u/%u swL=%u swR=%u on=0x%02X "
                      "tgt[4,2,3]=%d,%d,%d rpm[4,2,3]=%d,%d,%d "
                      "cmd[4,2,3]=%d,%d,%d fbI[4,2,3]=%d,%d,%d "
                      "temp[4,2,3]=%u,%u,%u txfail=%lu "
                      "CAN:free=%lu state=%u err=0x%08lX "
                      "PIT:angle=%lu.%lu "
                      "YAW:mode=%u ch=%d a10=%d t10=%d e10=%d rpmT=%d rpm=%d cmd=%d "
                      "angle=%lu.%lu\r\n",
                       Shoot_Test_RemoteSourceName(remote_source),
                       Shoot_Test_ModeName(g_mode),
                      (unsigned int)serial_mode,
                      (unsigned long)serial_left_ms,
                      (unsigned int)serial_locked,
                      (unsigned long)serial_seq,
                      (unsigned int)g_output_enabled,
                      (unsigned int)g_remote_online,
                      (unsigned int)g_remote_valid,
                      (unsigned int)remote.switch_position[RC_S_LEFT],
                      (unsigned int)remote.switch_position[RC_S_RIGHT],
                      (unsigned int)g_online_mask,
                      (int)g_target_rpm[0],
                      (int)g_target_rpm[1],
                      (int)g_target_rpm[2],
                      (int)g_feedback[0].speed_rpm,
                      (int)g_feedback[1].speed_rpm,
                      (int)g_feedback[2].speed_rpm,
                      (int)g_output_current[0],
                      (int)g_output_current[1],
                      (int)g_output_current[2],
                      (int)g_feedback[0].given_current,
                      (int)g_feedback[1].given_current,
                      (int)g_feedback[2].given_current,
                      (unsigned int)g_feedback[0].temperature,
                      (unsigned int)g_feedback[1].temperature,
                      (unsigned int)g_feedback[2].temperature,
                      (unsigned long)g_can_tx_fail_count,
                      (unsigned long)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1),
                      (unsigned int)HAL_CAN_GetState(&hcan1),
                      (unsigned long)HAL_CAN_GetError(&hcan1),
                       (unsigned long)(pitch_angle10 / 10U),
                       (unsigned long)(pitch_angle10 % 10U),
                       (unsigned int)yaw_mode,
                       (int)yaw_channel,
                       (int)yaw_angle10_debug,
                      (int)yaw_target10,
                      (int)yaw_error10,
                      (int)yaw_target_rpm,
                      (int)yaw_speed_rpm,
                      (int)yaw_current,
                       (unsigned long)(yaw_angle10_display / 10U),
                       (unsigned long)(yaw_angle10_display % 10U));
    if ((length > 0) && (length < (int)sizeof(message)))
    {
        (void)HAL_UART_Transmit_DMA(&huart6, (uint8_t *)message,
                                    (uint16_t)length);
    }
}
