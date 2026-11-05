#ifndef PITCH_TEST_H
#define PITCH_TEST_H

#include <stdint.h>

/* USART6 每 100ms 输出一次完整 Pitch 调试快照，便于快速采样。 */
#define PITCH_TUNING_DEBUG 1
#define PITCH_DEBUG_PERIOD_MS 100U

void Pitch_Test_Init(void);
void Pitch_Test_Update(void);
void Pitch_Test_DebugPrint(void);
/* Yaw 调参时使用 20ms 板上时间戳快照，其余仍为 100ms。 */
uint32_t Pitch_Test_DebugPeriodMs(void);
/* USART6 串口测试协议：逐字节接收，命令在 Pitch_Test_Update 中执行。 */
void Pitch_Test_SerialRxByte(uint8_t byte);
void Pitch_Test_SerialSendReady(void);
void Pitch_Test_GetYawDebug(uint8_t *mode,
                            int16_t *channel,
                            int16_t *angle10,
                            int16_t *target10,
                            int16_t *error10,
                            int16_t *target_rpm,
                            int16_t *speed_rpm,
                            int16_t *current);
uint8_t Pitch_Test_IsAimActive(void);
int8_t Pitch_Test_GetYawLimit(void);

#endif
