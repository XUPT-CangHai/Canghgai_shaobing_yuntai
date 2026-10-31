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
#include "BMI088driver.h"
#include "quaternion_ekf.h"
#include <stdio.h>
#include <math.h>

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t control_tick_ms;

static uint32_t heartbeat_tick_ms;
quaternion_ekf_t g_ekf;
float g_imu_gyro_bias[3];
float g_imu_accel[3], g_imu_gyro[3], g_imu_temp;
static uint32_t g_imu_tick_ms;
static uint32_t raw_debug_tick_ms;
static uint32_t boot_tick_ms;
static uint16_t vision_frame_seq;
/* Reference temperature controller: TIM10 CH1, compare counts (ARR=4999). */
static uint8_t imu_calibration_done, imu_calibration_active;
static const float RefTemp = 40.0f;
static const float HeatKp = 1000.0f, HeatKi = 20.0f;
static const float HeatIntegralLimit = 300.0f, HeatMaxOut = 2000.0f;
static float heat_integral;
static uint16_t heat_pwm;
static uint8_t heat_ready;
static uint32_t heat_start_tick;
static uint32_t heat_pi_tick;
static uint8_t heat_calibration_paused;
static uint8_t heat_pi_started;

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

/* AA 55 + length(0x15) + 19-byte payload + CRC = 23 bytes. */
static void Vision_SendImuFrame(void)
{
  static uint8_t frame[23];
  if (!imu_calibration_done) return;
  frame[0] = 0xAAU; frame[1] = 0x55U; frame[2] = 0x15U;
  Vision_PutU32(&frame[3], HAL_GetTick() - boot_tick_ms);
  Vision_PutU16(&frame[7], vision_frame_seq++);
  Vision_PutU16(&frame[9], (uint16_t)Vision_Scaled100((g_imu_gyro[0] - g_imu_gyro_bias[0]) * 57.29578f));
  Vision_PutU16(&frame[11], (uint16_t)Vision_Scaled100((g_imu_gyro[1] - g_imu_gyro_bias[1]) * 57.29578f));
  Vision_PutU16(&frame[13], (uint16_t)Vision_Scaled100((g_imu_gyro[2] - g_imu_gyro_bias[2]) * 57.29578f));
  Vision_PutU16(&frame[15], (uint16_t)Vision_Scaled100(g_ekf.yaw * 57.29578f));
  Vision_PutU16(&frame[17], (uint16_t)Vision_Scaled100(g_ekf.pitch * 57.29578f));
  Vision_PutU16(&frame[19], (uint16_t)Vision_Scaled100(g_ekf.roll * 57.29578f));
  frame[21] = (uint8_t)(1U | 2U | 4U | (g_ekf.accel_valid ? 8U : 0U));
  frame[22] = Vision_Crc8(frame, 22U);
  (void)CDC_Transmit_FS(frame, sizeof(frame));
}

static void IMU_Temperature_Ctrl(float temperature, float dt)
{
  uint32_t now=HAL_GetTick();
  if (!isfinite(temperature)) return;
  if (!heat_pi_started && temperature >= RefTemp-0.5f) { heat_pi_started=1U; heat_pi_tick=now; }
  if (!heat_pi_started) { heat_pwm=4500U; __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,heat_pwm); return; }
  heat_integral=fmaxf(-HeatIntegralLimit,fminf(HeatIntegralLimit,heat_integral+HeatKi*(RefTemp-temperature)*dt));
  heat_pwm=(uint16_t)fmaxf(0.0f,fminf(HeatMaxOut,HeatKp*(RefTemp-temperature)+heat_integral));
  __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,heat_pwm);
}
static void IMU_Temperature_Start(void){heat_integral=0;heat_pwm=0;heat_pi_started=0;heat_ready=0;heat_calibration_paused=0;heat_start_tick=HAL_GetTick();if(HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1)!=HAL_OK)Error_Handler();}
static void BMI088_Update(void){BMI088_read(g_imu_gyro,g_imu_accel,&g_imu_temp);}
static void IMU_Temperature_Wait(void){uint32_t last=HAL_GetTick(),start=last;for(;;){uint32_t now=HAL_GetTick();if(now-last<2)continue;float dt=(now-last)*.001f;last=now;BMI088_Update();IMU_Temperature_Ctrl(g_imu_temp,dt);if((heat_pi_started&&now-heat_pi_tick>=1000U)||(now-start>=100000U)){heat_calibration_paused=1;heat_pwm=0;__HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,0);break;}}heat_ready=1;}
static void BMI088_CalibrateGyro(void)
{
  float g[3], a[3], t, sum[3] = {0};
  uint32_t print_tick = HAL_GetTick();
  imu_calibration_active = 1U;
  for (uint16_t i = 0U; i < 200U; i++)
  {
    BMI088_read(g, a, &t);
    g_imu_temp = t;
    sum[0] += g[0]; sum[1] += g[1]; sum[2] += g[2];
    if ((uint32_t)(HAL_GetTick() - print_tick) >= 100U)
    {
      print_tick = HAL_GetTick();
      BMI088_PrintRaw();
    }
    HAL_Delay(2U);
  }
  for (int i = 0; i < 3; i++) g_imu_gyro_bias[i] = sum[i] / 200.0f;
  imu_calibration_active = 0U;
}

static void BMI088_PrintRaw(void)
{
  Vision_SendImuFrame();
  if (huart6.gState == HAL_UART_STATE_READY)
  {
    static char line[320];
    int n = snprintf(line, sizeof(line),
      "ms=%lu A=%.3f,%.3f,%.3f G=%.3f,%.3f,%.3f "
      "B=%.3f,%.3f,%.3f C=%.3f,%.3f,%.3f "
      "YPR=%.2f,%.2f,%.2f T=%.2f EKF=%.4f valid=%u PI=%u PWM=%u\r\n",
      (unsigned long)(HAL_GetTick() - boot_tick_ms),
      (double)g_imu_accel[0], (double)g_imu_accel[1], (double)g_imu_accel[2],
      (double)(g_imu_gyro[0] * 57.29578f), (double)(g_imu_gyro[1] * 57.29578f), (double)(g_imu_gyro[2] * 57.29578f),
      (double)(g_imu_gyro_bias[0] * 57.29578f), (double)(g_imu_gyro_bias[1] * 57.29578f), (double)(g_imu_gyro_bias[2] * 57.29578f),
      (double)((g_imu_gyro[0] - g_imu_gyro_bias[0]) * 57.29578f),
      (double)((g_imu_gyro[1] - g_imu_gyro_bias[1]) * 57.29578f),
      (double)((g_imu_gyro[2] - g_imu_gyro_bias[2]) * 57.29578f),
      (double)(g_ekf.yaw * 57.29578f), (double)(g_ekf.pitch * 57.29578f),
      (double)(g_ekf.roll * 57.29578f), (double)g_imu_temp,
      (double)g_ekf.innovation, (unsigned)g_ekf.accel_valid,
      (unsigned)heat_pi_started, (unsigned)heat_pwm);
    if (n > 0 && n < (int)sizeof(line))
      (void)HAL_UART_Transmit_DMA(&huart6, (uint8_t *)line, (uint16_t)n);
  }
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
  /* Match reference startup order: PWM first, then BMI088 and estimator init. */
  IMU_Temperature_Start();
  if (BMI088_init() != BMI088_NO_ERROR) { Error_Handler(); }
  IMU_Temperature_Wait();
  BMI088_CalibrateGyro();
  heat_calibration_paused = 0U;
  imu_calibration_done = 1U;
  QuaternionEKF_Init(&g_ekf);
  g_ekf.gyro_bias[0] = g_imu_gyro_bias[0];
  g_ekf.gyro_bias[1] = g_imu_gyro_bias[1];
  g_ekf.gyro_bias[2] = g_imu_gyro_bias[2];
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
  control_tick_ms = HAL_GetTick();
  g_imu_tick_ms = control_tick_ms;

  heartbeat_tick_ms = control_tick_ms;
  raw_debug_tick_ms = control_tick_ms;
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
        float imu_dt = (float)(HAL_GetTick() - g_imu_tick_ms) * 0.001f;
        g_imu_tick_ms = HAL_GetTick();
        BMI088_Update();
        IMU_Temperature_Ctrl(g_imu_temp, imu_dt);

        QuaternionEKF_Update(&g_ekf, g_imu_gyro, g_imu_accel, imu_dt);
      }
      if ((uint32_t)(HAL_GetTick() - raw_debug_tick_ms) >= 100U)
      {
        raw_debug_tick_ms = HAL_GetTick();
        BMI088_PrintRaw();
      }
      /* Other periodic control outputs disabled during IMU observation. */
      /* �?ms更新CAN2�?020 ID2 Yaw和ID3 Pitch位置控制，ID1不控制�?*/

    }

/* IMU inspection: other periodic UART telemetry disabled. */
    if ((uint32_t)(HAL_GetTick() - heartbeat_tick_ms) >= 500U)
    {
      heartbeat_tick_ms = HAL_GetTick();
      HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    }
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
