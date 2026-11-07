#ifndef SHOOT_TEST_H
#define SHOOT_TEST_H

#include <stdint.h>

void Shoot_Test_Init(void);
void Shoot_Test_Update(void);
void Shoot_Test_DebugPrint(void);

/* 修改：mode=1仅拨弹2秒，mode=2三电机发射1秒；运行中返回0且不延长。 */
uint8_t Shoot_Test_SerialStart(uint8_t mode);
void Shoot_Test_SerialStop(void);
/* 修改：mode=0无串口动作；locked表示串口接管/等待遥控回STOP，seq每次启动递增。 */
void Shoot_Test_GetSerialStatus(uint8_t *mode, uint32_t *remain_ms,
                                uint8_t *locked, uint32_t *seq);

#endif
