#ifndef VISION_AIM_H
#define VISION_AIM_H

#include <stdint.h>

#define VISION_AIM_FRAME_SIZE 22U
#define VISION_AIM_TIMEOUT_MS 100U /* 修改：按新图像时间判断新鲜度，保活不续期。 */
#define VISION_AIM_LOST_CLOSE_MS 10000U /* 连续 10 秒无目标才关闭自瞄。 */

/*
 * 修改：USB CDC视觉帧固定22字节、小端，不混收旧13字节帧：
 * A5 5A | seq | valid | capture_time_ms(4) |
 * yaw_error(2) | yaw_velocity(2) | yaw_acceleration(4) |
 * pitch_error(2) | pitch_velocity(2) | CRC16/MODBUS(2)
 *
 * 误差单位0.01度，速度0.01度/秒，加速度0.01度/秒平方，向右/向上为正。
 * CRC覆盖前20字节，初值0xFFFF、多项式0xA001，低字节先发。
 * capture_time_ms为视觉端时间，仅记录；超时使用本机接收新图像的时间。
 * 同seq只保活；valid=0即使同seq也立即丢目标，再有效必须收到新seq。
 */
typedef struct
{
    uint8_t target_valid;
    uint8_t seq;
    uint32_t capture_time_ms;
    int16_t yaw_error_cdeg;
    int16_t yaw_velocity_cdeg_s;
    int32_t yaw_acceleration_cdeg_s2;
    int16_t pitch_error_cdeg;
    int16_t pitch_velocity_cdeg_s;
    uint32_t last_rx_tick_ms;
    uint32_t last_new_target_tick_ms; /* 修改：仅新seq且valid=1时更新。 */
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
