#ifndef VISION_AIM_H
#define VISION_AIM_H

#include <stdint.h>

#define VISION_AIM_FRAME_SIZE 13U
#define VISION_AIM_TIMEOUT_MS 200U
#define VISION_AIM_LOST_CLOSE_MS 10000U /* 连续 10 秒无目标才关闭自瞄。 */

/*
 * USB CDC 视觉自瞄帧（固定 13 字节，小端）：
 * A5 5A | valid | seq | capture_time_ms(4) |
 * yaw_error(2) | pitch_error(2) | XOR(1)
 *
 * yaw_error / pitch_error 单位为 0.01°：向右、向上为正。
 * checksum 是前 12 字节逐字节异或。valid=0 表示当前没有目标，
 * 但该帧仍然算作一帧有效协议帧，用于及时退出自瞄。
 */
typedef struct
{
    uint8_t target_valid;
    uint8_t seq;
    uint32_t capture_time_ms;
    int16_t yaw_error_cdeg;
    int16_t pitch_error_cdeg;
    uint32_t last_rx_tick_ms;
    uint32_t last_target_tick_ms;
    uint32_t valid_frame_count;
    uint32_t new_frame_count;
    uint32_t checksum_error_count;
    uint32_t rx_byte_count;
    uint8_t aim_enabled;
    uint32_t enable_count;
    uint32_t disable_count;
} vision_aim_command_t;

void Vision_Aim_Init(void);
/* 初始化 USART6 后调用：zimiaokai 开启，zimioaguan 关闭，无需遥控器。 */
void Vision_Aim_DebugUartStart(void);
/* 可在 USB CDC 回调中调用；内部只解析和发布快照，不控制电机。 */
void Vision_Aim_Receive(const uint8_t *data, uint32_t length);
void Vision_Aim_Get(vision_aim_command_t *command);
/* 由主循环调用，10 秒没有有效目标时自动关闭自瞄。 */
void Vision_Aim_Service(void);
void Vision_Aim_Invalidate(void);

#endif
