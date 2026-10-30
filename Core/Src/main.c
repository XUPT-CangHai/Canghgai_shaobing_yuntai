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
#include "BMI088driver.h"
#include "quaternion_ekf.h"

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
static uint32_t debug_tick_ms;
static uint32_t heartbeat_tick_ms;
quaternion_ekf_t g_ekf;
float g_imu_gyro_bias[3];
float g_imu_accel[3], g_imu_gyro[3], g_imu_temp;
static uint32_t g_imu_tick_ms;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void BMI088_CalibrateGyro(void)
{
  float gyro[3], accel[3], temp;
  float sum[3] = {0.0f, 0.0f, 0.0f};
  for (uint16_t i = 0U; i < 200U; i++)
  {
    BMI088_read(gyro, accel, &temp);
    sum[0] += gyro[0]; sum[1] += gyro[1]; sum[2] += gyro[2];
    HAL_Delay(2U);
  }
  g_imu_gyro_bias[0] = sum[0] / 200.0f;
  g_imu_gyro_bias[1] = sum[1] / 200.0f;
  g_imu_gyro_bias[2] = sum[2] / 200.0f;
}

static void BMI088_Update(void)
{
  BMI088_read(g_imu_gyro, g_imu_accel, &g_imu_temp);
  QuaternionEKF_Update(&g_ekf, g_imu_gyro, g_imu_accel, 0.002f);
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
  if (BMI088_init() != BMI088_NO_ERROR) { Error_Handler(); }
  BMI088_CalibrateGyro();
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
  debug_tick_ms = control_tick_ms;
  heartbeat_tick_ms = control_tick_ms;
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
        g_imu_tick_ms += 2U;
        BMI088_Update();
      }
      Shoot_Test_Update();
      /* �?ms更新CAN2�?020 ID2 Yaw和ID3 Pitch位置控制，ID1不控制�?*/
      Pitch_Test_Update();
    }

#if PITCH_TUNING_DEBUG
    /* Pitch调参使用20Hz快照，采样期间只输出CAN2 ID3，不与整车DMA抢占�?*/
    if ((uint32_t)(HAL_GetTick() - debug_tick_ms) >= PITCH_DEBUG_PERIOD_MS)
    {
      debug_tick_ms = HAL_GetTick();
      Pitch_Test_DebugPrint();
    }
#else
    if ((uint32_t)(HAL_GetTick() - debug_tick_ms) >= 1000U)
    {
      debug_tick_ms += 1000U;
      Shoot_Test_DebugPrint();
    }
#endif
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
