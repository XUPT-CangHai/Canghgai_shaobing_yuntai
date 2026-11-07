#include "gimbal_heading_link.h"

#include <stdint.h>

#include "pitch_test.h"
#include "usart.h"

#define GIMBAL_HEADING_FRAME_SIZE 12U
#define GIMBAL_HEADING_PERIOD_MS  10U

static uint16_t Gimbal_Heading_Crc16(const uint8_t *data, uint16_t length)
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

static void Gimbal_Heading_PutU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

void Gimbal_Heading_Link_Update(void)
{
    static uint8_t frame[GIMBAL_HEADING_FRAME_SIZE];
    static uint8_t sequence;
    static uint32_t last_tx_tick_ms;
    int16_t angle_cdeg = 0;
    uint16_t big_ecd = 0U;
    uint16_t small_ecd = 0U;
    uint32_t now_ms = HAL_GetTick();

    if (((uint32_t)(now_ms - last_tx_tick_ms) < GIMBAL_HEADING_PERIOD_MS) ||
        (huart1.gState != HAL_UART_STATE_READY))
    {
        return;
    }

    /* 修改：与原底盘状态接收分方向使用USART1；只在TX空闲后写DMA缓冲。 */
    frame[0] = 0xA7U;
    frame[1] = 0x5AU;
    frame[2] = sequence;
    frame[3] = Pitch_Test_GetRelativeHeading(&angle_cdeg, &big_ecd, &small_ecd);
    Gimbal_Heading_PutU16(&frame[4], (uint16_t)angle_cdeg);
    Gimbal_Heading_PutU16(&frame[6], big_ecd);
    Gimbal_Heading_PutU16(&frame[8], small_ecd);
    Gimbal_Heading_PutU16(&frame[10], Gimbal_Heading_Crc16(frame, 10U));
    if (HAL_UART_Transmit_DMA(&huart1, frame, sizeof(frame)) == HAL_OK)
    {
        sequence++;
        last_tx_tick_ms = now_ms;
    }
}
