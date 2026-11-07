#ifndef PITCH_TEST_H
#define PITCH_TEST_H

#include <stdint.h>

/* 修改：460800调试串口，PDBG名义10ms、完整AIM概要100ms；BY/YAW调参节拍保留。 */
#define PITCH_TUNING_DEBUG 1
#define PITCH_DEBUG_PERIOD_MS 100U

void Pitch_Test_Init(void);
void Pitch_Test_Update(void);
void Pitch_Test_DebugPrint(void);
/* 修改：主循环非阻塞发送同控制周期Pitch快照，DMA忙时丢遥测并累计drop。 */
void Pitch_Test_TelemetryService(void);
/* 修改：ID1串口命令BY GET、BY LOG 0、BY TUNE <speed_kp> <speed_ki>。
 * BY GET开启50ms快照，BY LOG 0取消强制快照，TUNE参数断电恢复默认值。
 * BY HOLD 0/1关闭/启用朝向外环；实测后上电默认启用，ID2转角系数为+1。
 * BY SIGN <1|-1>仅在HOLD 0时设置ID2编码器方向；BY HKP/HKI <gain>调外环。
 * HKP单位rpm/deg，HKI单位rpm/(deg*s)，调参后重新锚定，应先右中停稳。
 * 快照增加becd/secd两轴编码器、sa小Yaw角、hd/ht/he大Yaw朝向/目标/误差。
 * cor/trim为外环修正rpm和积分rpm，hv为IMU及ID2反馈有效；参数不写Flash。
 * BY命令不启动电机，ID1启停仍由底盘A5/拨杆控制。 */
uint32_t Pitch_Test_DebugPeriodMs(void);
/* USART6 串口测试协议：逐字节接收，命令在 Pitch_Test_Update 中执行。 */
void Pitch_Test_SerialRxByte(uint8_t byte);
void Pitch_Test_SerialSendReady(void);
/* 修改：板间回传最终枪口相对底盘角，逆时针正，单位0.01度。
 * FOLLOW ZERO在左中右下停稳、枪口对准车头时标定；断电后恢复编译时零位。 */
uint8_t Pitch_Test_GetRelativeHeading(int16_t *angle_cdeg,
                                      uint16_t *big_ecd,
                                      uint16_t *small_ecd);
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
