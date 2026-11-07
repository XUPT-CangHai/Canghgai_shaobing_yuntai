#ifndef GIMBAL_HEADING_LINK_H
#define GIMBAL_HEADING_LINK_H

/* 修改：云台每10ms通过USART1回传最终相机相对底盘朝向，逆时针为正。
 * 固定12字节：A7 5A | seq | valid | angle_cdeg(i16) |
 * big_ecd(u16) | small_ecd(u16) | CRC16/MODBUS(前10字节)。
 * 多字节均为小端；未标定正前零位或反馈失效时valid=0。
 * 主循环调用即可，DMA忙时跳过，不修改正在发送的缓冲。
 */
void Gimbal_Heading_Link_Update(void);

#endif
