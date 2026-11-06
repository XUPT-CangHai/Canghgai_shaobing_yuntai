/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "imu_system.h"
#include <math.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "shoot_test.h"
#include "pitch_test.h"
#include "vision_aim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VISION_IMU_SEND_PERIOD_MS 5U /* USB 虚拟串口每 5ms 上报，目标频率 200Hz。 */
#define IMU_STATUS_LED_PWM 50000U
#define IMU_DEBUG_PERIOD_MS 100U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
static uint32_t control_tick_ms;

static uint32_t g_imu_tick_ms;
static uint32_t imu_debug_warmup_tick_ms;
static uint32_t vision_imu_tick_ms;
static uint32_t aim_debug_tick_ms;
static uint32_t boot_tick_ms;
static uint16_t vision_frame_seq;
typedef enum
{
  IMU_LED_WARMING = 0U,
  IMU_LED_STABLE,
  IMU_LED_CALIBRATING,
  IMU_LED_READY
} imu_led_state_t;
static imu_led_state_t imu_led_state;
static uint8_t imu_led_phase;
static uint32_t imu_led_tick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t Vision_Crc8(const uint8_t *data, uint16_t length)
{
  uint8_t crc = 0U;
  for (uint16_t i = 0U; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; bit++)
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
  }
  return crc;
}

static void Vision_PutU16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void Vision_PutU32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static int16_t Vision_Scaled100(float value)
{
  float scaled = value * 100.0f;
  if (scaled > 32767.0f) return 32767;
  if (scaled < -32768.0f) return -32768;
  return (int16_t)lroundf(scaled);
}

/* TIM5 CH1/CH2/CH3 分别连接蓝/绿/红灯，使用 PWM 设置 RGB 亮度。 */
static void IMU_StatusLed_Output(uint16_t red, uint16_t green, uint16_t blue)
{
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, blue);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, green);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, red);
}

static void IMU_StatusLed_Set(imu_led_state_t state)
{
  imu_led_state = state;
  imu_led_phase = 1U;
  imu_led_tick = HAL_GetTick();
  if (state == IMU_LED_WARMING) IMU_StatusLed_Output(IMU_STATUS_LED_PWM, 0U, 0U);
  else if (state == IMU_LED_STABLE) IMU_StatusLed_Output(IMU_STATUS_LED_PWM, IMU_STATUS_LED_PWM, 0U);
  else if (state == IMU_LED_CALIBRATING) IMU_StatusLed_Output(0U, 0U, IMU_STATUS_LED_PWM);
  else IMU_StatusLed_Output(0U, IMU_STATUS_LED_PWM, 0U);
}

static void IMU_StatusLed_Update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t period = (imu_led_state == IMU_LED_CALIBRATING) ? 250U : 500U;
  if (imu_led_state == IMU_LED_READY)
  {
    IMU_StatusLed_Output(0U, IMU_STATUS_LED_PWM, 0U);
    return;
  }
  if ((uint32_t)(now - imu_led_tick) >= period)
  {
    imu_led_tick = now;
    imu_led_phase ^= 1U;
    if (imu_led_phase == 0U) IMU_StatusLed_Output(0U, 0U, 0U);
    else if (imu_led_state == IMU_LED_WARMING) IMU_StatusLed_Output(IMU_STATUS_LED_PWM, 0U, 0U);
    else if (imu_led_state == IMU_LED_STABLE) IMU_StatusLed_Output(IMU_STATUS_LED_PWM, IMU_STATUS_LED_PWM, 0U);
    else IMU_StatusLed_Output(0U, 0U, IMU_STATUS_LED_PWM);
  }
}

/* AA 55 + length(0x15) + 19-byte payload + CRC = 23 bytes. */
static void Vision_SendImuFrame(void)
{
  static uint8_t frame[23];
  const imu_system_t *imu = IMU_System_GetData();
  USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (!imu->initialized) return;
  /* USB 未连接或上一帧尚未发完时跳过，避免改写正在发送的缓冲。 */
  if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
      (cdc == NULL) || (cdc->TxState != 0U)) return;
  frame[0] = 0xAAU; frame[1] = 0x55U; frame[2] = 0x15U;
  Vision_PutU32(&frame[3], HAL_GetTick() - boot_tick_ms);
  Vision_PutU16(&frame[7], vision_frame_seq++);
  /* BMI088_read has removed the XYZ base offset; EKF removes residual X/Y bias. */
  Vision_PutU16(&frame[9], (uint16_t)Vision_Scaled100((imu->gyro[0] - imu->ekf.gyro_bias[0]) * 57.29578f));
  Vision_PutU16(&frame[11], (uint16_t)Vision_Scaled100((imu->gyro[1] - imu->ekf.gyro_bias[1]) * 57.29578f));
  Vision_PutU16(&frame[13], (uint16_t)Vision_Scaled100((imu->gyro[2] - imu->ekf.gyro_bias[2]) * 57.29578f));
  Vision_PutU16(&frame[15], (uint16_t)Vision_Scaled100(imu->ekf.yaw * 57.29578f));
  Vision_PutU16(&frame[17], (uint16_t)Vision_Scaled100(imu->ekf.pitch * 57.29578f));
  Vision_PutU16(&frame[19], (uint16_t)Vision_Scaled100(imu->ekf.roll * 57.29578f));
  frame[21] = (uint8_t)(1U | 2U | 4U | (imu->ekf.accel_valid ? 8U : 0U));
  frame[22] = Vision_Crc8(frame, 22U);
  (void)CDC_Transmit_FS(frame, sizeof(frame));
}

static void IMU_Temperature_SetPWM(uint16_t pwm)
{
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm);
}

#if 0 /* 旧 IMU 调试串口输出已停用，USART6 改为 Pitch 快照专用。 */
static void IMU_DebugPrint(void)
{
  /* 已停用：USART6 只保留每秒一次的 Pitch 快照输出。 */
  static char message[256];
  const imu_system_t *imu = IMU_System_GetData();
  uint32_t now_ms = HAL_GetTick();
  int length;

  if (huart6.gState != HAL_UART_STATE_READY) return;
  length = snprintf(message, sizeof(message),
                    "ms=%lu A=%.3f,%.3f,%.3f AF=%.3f,%.3f,%.3f "
                    "G=%.3f,%.3f,%.3f YPR=%.2f,%.2f,%.2f T=%.2f "
                    "DT=%lu STABLE=%u DVALID=%u CHI=%.4f DB=%.4f,%.4f,%.4f\r\n",
                    (unsigned long)(now_ms - boot_tick_ms),
                    (double)imu->accel[0], (double)imu->accel[1], (double)imu->accel[2],
                    (double)imu->accel_filtered[0],
                    (double)imu->accel_filtered[1],
                    (double)imu->accel_filtered[2],
                    (double)imu->gyro[0], (double)imu->gyro[1], (double)imu->gyro[2],
                    (double)(imu->ekf.yaw * 57.29578f),
                    (double)(imu->ekf.pitch * 57.29578f),
                    (double)(imu->ekf.roll * 57.29578f),
                    (double)imu->temperature,
                    (unsigned long)imu->dt_us,
                    (unsigned int)imu->ekf.stable,
                    (unsigned int)imu->ekf.accel_valid,
                    (double)imu->ekf.chi_square,
                    (double)imu->ekf.gyro_bias[0],
                    (double)imu->ekf.gyro_bias[1],
                    (double)imu->ekf.gyro_bias[2]);
  if ((length > 0) && (length < (int)sizeof(message)))
  {
    (void)HAL_UART_Transmit_DMA(&huart6, (uint8_t *)message, (uint16_t)length);
  }
}
#endif

static void IMU_DebugPrintIfDue(void)
{
  /* 保留启动回调，但不再发送旧的 IMU 调试行。 */
  uint32_t now_ms = HAL_GetTick();
  if ((uint32_t)(now_ms - imu_debug_warmup_tick_ms) < IMU_DEBUG_PERIOD_MS) return;
  imu_debug_warmup_tick_ms = now_ms;
  /* IMU_DebugPrint(); */
}

static uint32_t IMU_GetTickMs(void)
{
  return HAL_GetTick();
}

static uint8_t IMU_HeaterStart(void)
{
  return (uint8_t)(HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) == HAL_OK);
}

static void IMU_StageChanged(imu_system_stage_t stage)
{
  if (stage == IMU_SYSTEM_WARMING) IMU_StatusLed_Set(IMU_LED_WARMING);
  else if (stage == IMU_SYSTEM_TEMPERATURE_STABLE) IMU_StatusLed_Set(IMU_LED_STABLE);
  else if (stage == IMU_SYSTEM_CALIBRATING) IMU_StatusLed_Set(IMU_LED_CALIBRATING);
  else IMU_StatusLed_Set(IMU_LED_READY);
}

static void IMU_StartupProgress(void)
{
  IMU_DebugPrintIfDue();
  IMU_StatusLed_Update();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  boot_tick_ms = HAL_GetTick();
  imu_debug_warmup_tick_ms = boot_tick_ms;
  imu_system_config_t imu_config = {
      .get_tick_ms = IMU_GetTickMs,
      .heater_start = IMU_HeaterStart,
      .heater_set_pwm = IMU_Temperature_SetPWM,
      .stage_changed = IMU_StageChanged,
      .progress = IMU_StartupProgress};
  if (!IMU_System_Init(&imu_config)) Error_Handler();
  MX_TIM14_Init();
  /* 视觉流解析器必须先初始化，再启动 USB CDC 接收�?*/
  Vision_Aim_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* 云台调试串口接收自瞄开关；USB CDC 仅接收视觉角度帧�?*/
  Vision_Aim_DebugUartStart();
  Shoot_Test_Init();
  /* CAN1发射与CAN2 Yaw/Pitch共用遥控器，外设由Shoot_Test_Init统一启动�?*/
  Pitch_Test_Init();
  /* 加热、稳定和陀螺校准结束后通知上位机可以开始 Pitch 串口测试。 */
  Pitch_Test_SerialSendReady();
  control_tick_ms = HAL_GetTick();
  g_imu_tick_ms = control_tick_ms;

  vision_imu_tick_ms = control_tick_ms;
  aim_debug_tick_ms = control_tick_ms;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ((uint32_t)(HAL_GetTick() - control_tick_ms) >= 2U)
    {
      control_tick_ms = HAL_GetTick();
      if ((uint32_t)(HAL_GetTick() - g_imu_tick_ms) >= 2U)
      {
        uint32_t imu_now_ms = HAL_GetTick();
        uint32_t imu_elapsed_ms = imu_now_ms - g_imu_tick_ms;
        float imu_dt = (float)imu_elapsed_ms * 0.001f;
        g_imu_tick_ms = imu_now_ms;
        IMU_System_Update(imu_dt);
      }
      Pitch_Test_Update();
      /* Shooter output remains disabled during IMU and gimbal observation. */
      /* 每2ms更新CAN2：ID1大Yaw小陀螺补偿、ID2小Yaw、ID3 Pitch。 */

    }

    /* 独立按 5ms 调度 USB，避免受 2ms 控制周期影响变成每 6ms 一帧。 */
    if ((uint32_t)(HAL_GetTick() - vision_imu_tick_ms) >= VISION_IMU_SEND_PERIOD_MS)
    {
      vision_imu_tick_ms = HAL_GetTick();
      Vision_SendImuFrame();
    }

    /* 修改：USART6按模式输出，ID1小陀螺50ms，ID2调参20ms，默认自瞄1s。 */
    if ((uint32_t)(HAL_GetTick() - aim_debug_tick_ms) >= Pitch_Test_DebugPeriodMs())
    {
      aim_debug_tick_ms = HAL_GetTick();
      Pitch_Test_DebugPrint();
      /* IMU_DebugPrint(); */
    }
    IMU_StatusLed_Update();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number. */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
