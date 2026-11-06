#include "vision_aim.h"

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "pitch_test.h"
#include <string.h>

static volatile vision_aim_command_t g_vision_aim;
static uint8_t g_frame[VISION_AIM_FRAME_SIZE];
static uint8_t g_frame_length;
static uint8_t g_have_sequence;
#define VISION_AIM_COMMAND_MAX 10U
static uint8_t g_command[VISION_AIM_COMMAND_MAX];
static uint8_t g_command_length;
static uint8_t g_debug_rx_byte;

static void Vision_Aim_CommandReset(void)
{
    g_command_length = 0U;
}

static void Vision_Aim_Enable(void)
{
    /* 调试串口只切换自瞄使能；开启后必须等一帧新的 USB 目标。 */
    g_vision_aim.target_valid = 0U;
    g_have_sequence = 0U;
    g_vision_aim.aim_enabled = 1U;
    g_vision_aim.enable_count++;
    /* 开启后若一直没看到目标，也从开启时刻开始计 10 秒。 */
    g_vision_aim.last_target_tick_ms = HAL_GetTick();
    Vision_Aim_CommandReset();
}

static void Vision_Aim_Disable(void)
{
    g_vision_aim.aim_enabled = 0U;
    g_vision_aim.target_valid = 0U;
    g_vision_aim.disable_count++;
    Vision_Aim_CommandReset();
}

static void Vision_Aim_ParseCommandByte(uint8_t byte)
{
    static const uint8_t enable_command[] = "zimiaokai";
    static const uint8_t disable_command[] = "zimioaguan";
    uint8_t command_length;

    if ((byte == '\r') || (byte == '\n') ||
        (byte < (uint8_t)'a') || (byte > (uint8_t)'z'))
    {
        Vision_Aim_CommandReset();
        return;
    }

    if (g_command_length >= VISION_AIM_COMMAND_MAX)
    {
        memmove(g_command, &g_command[1], VISION_AIM_COMMAND_MAX - 1U);
        g_command_length = VISION_AIM_COMMAND_MAX - 1U;
    }
    g_command[g_command_length++] = byte;

    command_length = (uint8_t)(sizeof(enable_command) - 1U);
    if ((g_command_length >= command_length) &&
        (memcmp(&g_command[g_command_length - command_length],
                enable_command, command_length) == 0))
    {
        Vision_Aim_Enable();
        return;
    }

    command_length = (uint8_t)(sizeof(disable_command) - 1U);
    if ((g_command_length >= command_length) &&
        (memcmp(&g_command[g_command_length - command_length],
                disable_command, command_length) == 0))
    {
        Vision_Aim_Disable();
    }
}

static uint16_t Vision_Aim_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t Vision_Aim_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

/* 修改：固定22字节帧改用CRC16/MODBUS，CRC低字节位于frame[20]。 */
static uint16_t Vision_Aim_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U) :
                               (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

void Vision_Aim_Init(void)
{
    g_vision_aim = (vision_aim_command_t){0};
    g_frame_length = 0U;
    g_have_sequence = 0U;
    g_command_length = 0U;
}

void Vision_Aim_DebugUartStart(void)
{
    /* USART6：115200、8N1，接收 ASCII 开关命令；TX 仍供调试 DMA 输出。 */
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    (void)HAL_UART_Receive_IT(&huart6, &g_debug_rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6)
    {
        uint8_t byte = g_debug_rx_byte;

        (void)HAL_UART_Receive_IT(&huart6, &g_debug_rx_byte, 1U);
        /* USART6 同时承载 Pitch 测试协议；这里只收字节，控制在主循环执行。 */
        Pitch_Test_SerialRxByte(byte);
        /* 收齐 zimiaokai / zimioaguan 立即生效，有无回车换行均可。 */
        Vision_Aim_ParseCommandByte(byte);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6)
    {
        /* 调试串口出现溢出或帧错误后重启 RX，不中止正在进行的调试 TX。 */
        (void)HAL_UART_AbortReceive(huart);
        Vision_Aim_CommandReset();
        Vision_Aim_DebugUartStart();
    }
}

void Vision_Aim_Invalidate(void)
{
    /* USB 断开等同于丢目标，由 Service 统一执行 10 秒自动关闭。 */
    g_vision_aim.target_valid = 0U;
    g_frame_length = 0U;
    g_have_sequence = 0U;
}

void Vision_Aim_Service(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t now_ms;

    /* 与 USART6 命令、USB 目标更新互斥，避免刚收到新目标就被超时关闭。 */
    __disable_irq();
    now_ms = HAL_GetTick();
    /* 修改：旧seq保活不能维持旧目标；只有新图像可恢复target_valid。 */
    if ((g_vision_aim.target_valid != 0U) &&
        ((uint32_t)(now_ms - g_vision_aim.last_new_target_tick_ms) >
         VISION_AIM_TIMEOUT_MS))
    {
        g_vision_aim.target_valid = 0U;
    }
    if ((g_vision_aim.aim_enabled != 0U) &&
        ((uint32_t)(now_ms - g_vision_aim.last_target_tick_ms) >=
         VISION_AIM_LOST_CLOSE_MS))
    {
        Vision_Aim_Disable();
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Vision_Aim_Receive(const uint8_t *data, uint32_t length)
{
    uint32_t offset;
    uint32_t now_ms;
    uint8_t sequence;

    if ((data == 0) || (length == 0U))
    {
        return;
    }

    g_vision_aim.rx_byte_count += length;

    for (offset = 0U; offset < length; offset++)
    {
        /* USB 只接收视觉二进制帧，不解析调试串口命令。 */
        g_frame[g_frame_length++] = data[offset];
        if (g_frame_length < VISION_AIM_FRAME_SIZE)
        {
            continue;
        }

        if ((g_frame[0] == 0xA5U) && (g_frame[1] == 0x5AU) &&
            (Vision_Aim_Crc16(g_frame, VISION_AIM_FRAME_SIZE - 2U) ==
             Vision_Aim_ReadU16(&g_frame[VISION_AIM_FRAME_SIZE - 2U])) &&
            ((g_frame[3] == 0U) || (g_frame[3] == 1U)))
        {
            now_ms = HAL_GetTick();
            sequence = g_frame[2];
            g_vision_aim.last_rx_tick_ms = now_ms;
            g_vision_aim.valid_frame_count++;
            /* 修改：丢目标优先于seq判重，同seq的valid=1不重新激活旧目标。 */
            if (g_frame[3] == 0U)
            {
                g_vision_aim.target_valid = 0U;
            }
            if ((g_have_sequence == 0U) || (sequence != g_vision_aim.seq))
            {
                /* 修改：测量值和图像时间仅随新seq更新，不被重复保活包覆盖。 */
                g_vision_aim.seq = sequence;
                g_vision_aim.target_valid = g_frame[3];
                g_vision_aim.capture_time_ms = Vision_Aim_ReadU32(&g_frame[4]);
                g_vision_aim.yaw_error_cdeg =
                    (int16_t)Vision_Aim_ReadU16(&g_frame[8]);
                g_vision_aim.yaw_velocity_cdeg_s =
                    (int16_t)Vision_Aim_ReadU16(&g_frame[10]);
                g_vision_aim.yaw_acceleration_cdeg_s2 =
                    (int32_t)Vision_Aim_ReadU32(&g_frame[12]);
                g_vision_aim.pitch_error_cdeg =
                    (int16_t)Vision_Aim_ReadU16(&g_frame[16]);
                g_vision_aim.pitch_velocity_cdeg_s =
                    (int16_t)Vision_Aim_ReadU16(&g_frame[18]);
                if (g_vision_aim.target_valid != 0U)
                {
                    g_vision_aim.last_new_target_tick_ms = now_ms;
                    g_vision_aim.last_target_tick_ms = now_ms;
                }
                g_vision_aim.new_frame_count++;
            }
            g_have_sequence = 1U;
            g_frame_length = 0U;
            continue;
        }

        if ((g_frame[0] == 0xA5U) && (g_frame[1] == 0x5AU))
        {
            g_vision_aim.checksum_error_count++;
        }

        /* 坏帧后滑动一个字节，支持拆包、粘包和重新同步。 */
        memmove(g_frame, &g_frame[1], VISION_AIM_FRAME_SIZE - 1U);
        g_frame_length = VISION_AIM_FRAME_SIZE - 1U;
    }
}

void Vision_Aim_Get(vision_aim_command_t *command)
{
    uint32_t primask;

    if (command == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *command = g_vision_aim;
    if (primask == 0U)
    {
        __enable_irq();
    }
}
