#ifndef PITCH_TEST_H
#define PITCH_TEST_H

#include <stdint.h>

/* USART6 每秒输出一次完整 Pitch 调试快照。 */
#define PITCH_TUNING_DEBUG 1
#define PITCH_DEBUG_PERIOD_MS 1000U

void Pitch_Test_Init(void);
void Pitch_Test_Update(void);
void Pitch_Test_DebugPrint(void);
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
