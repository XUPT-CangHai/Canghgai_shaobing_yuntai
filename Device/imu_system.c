#include "imu_system.h"

#include <math.h>
#include <string.h>

#include "BMI088driver.h"
#include "imu_temperature_control.h"

#define IMU_TEMPERATURE_TARGET_C 39.5f
#define IMU_TEMPERATURE_PI_START_PERCENT 90.0f /* 达到目标温度的此百分比时切入 PI。 */
#define IMU_TEMPERATURE_STABLE_ERROR 0.25f
#define IMU_TEMPERATURE_STABLE_MS 2000U
#define IMU_TEMPERATURE_COLD_START_THRESHOLD_C 25.0f
#define IMU_TEMPERATURE_COLD_START_FEED_FORWARD_PWM 1100.0f
#define IMU_TEMPERATURE_NORMAL_START_FEED_FORWARD_PWM 900.0f
#define IMU_GYRO_CALIBRATION_SAMPLES 6000U
#define IMU_ACCEL_INIT_SAMPLES 100U
#define IMU_STATIC_GYRO_MAX_RAD_S 0.05f
#define IMU_STATIC_ACCEL_TOLERANCE 0.5f
#define IMU_ACCEL_FILTER_ALPHA 0.386f
#define IMU_SAMPLE_PERIOD_MS 2U

static imu_system_t g_imu;
static imu_system_config_t g_config;
static imu_temperature_controller_t g_temperature_controller;
static imu_system_stage_t g_stage;
static uint8_t g_stage_initialized;
static uint8_t g_accel_filter_initialized;

static void IMU_System_SetStage(imu_system_stage_t stage)
{
  if (g_stage_initialized && g_stage == stage) return;
  g_stage = stage;
  g_stage_initialized = 1U;
  if (g_config.stage_changed != NULL) g_config.stage_changed(stage);
}

static void IMU_System_Progress(void)
{
  if (g_config.progress != NULL) g_config.progress();
}

static void IMU_System_ReadSensor(void)
{
  float raw_accel[3];
  BMI088_read(g_imu.gyro, raw_accel, &g_imu.temperature);
  for (uint8_t i = 0U; i < 3U; i++)
  {
    g_imu.accel[i] = raw_accel[i];
    if (!g_accel_filter_initialized)
      g_imu.accel_filtered[i] = raw_accel[i];
    else
      g_imu.accel_filtered[i] +=
          IMU_ACCEL_FILTER_ALPHA * (raw_accel[i] - g_imu.accel_filtered[i]);
  }
  g_accel_filter_initialized = 1U;
}

static void IMU_System_UpdateTemperature(float dt)
{
  IMU_TemperatureController_Update(&g_temperature_controller,
                                  g_imu.temperature, dt);
}

static uint8_t IMU_System_TemperatureInRange(void)
{
  return (uint8_t)(
      IMU_TemperatureController_PIStarted(&g_temperature_controller) &&
      fabsf(g_imu.temperature - g_temperature_controller.target_temp) <=
          IMU_TEMPERATURE_STABLE_ERROR);
}

static void IMU_System_WaitForTemperature(void)
{
  uint32_t last_tick = g_config.get_tick_ms();
  uint32_t stable_start_tick = 0U;

  for (;;)
  {
    uint32_t now = g_config.get_tick_ms();
    uint32_t elapsed_ms = now - last_tick;
    if (elapsed_ms < IMU_SAMPLE_PERIOD_MS) continue;

    last_tick = now;
    g_imu.dt_us = elapsed_ms * 1000U;
    IMU_System_ReadSensor();
    IMU_System_UpdateTemperature((float)elapsed_ms * 0.001f);
    IMU_System_Progress();

    if (IMU_System_TemperatureInRange())
    {
      IMU_System_SetStage(IMU_SYSTEM_TEMPERATURE_STABLE);
      if (stable_start_tick == 0U) stable_start_tick = now;
      if ((uint32_t)(now - stable_start_tick) >= IMU_TEMPERATURE_STABLE_MS) return;
    }
    else
    {
      IMU_System_SetStage(IMU_SYSTEM_WARMING);
      stable_start_tick = 0U;
    }
  }
}

static void IMU_System_CalibrateGyro(void)
{
  const float zero_offset[3] = {0.0f, 0.0f, 0.0f};
  float sum[3] = {0.0f};
  uint16_t valid_samples = 0U;
  uint32_t last_tick = g_config.get_tick_ms();

  BMI088_set_gyro_offset(zero_offset);
  IMU_System_SetStage(IMU_SYSTEM_CALIBRATING);
  while (valid_samples < IMU_GYRO_CALIBRATION_SAMPLES)
  {
    uint32_t now = g_config.get_tick_ms();
    uint32_t elapsed_ms = now - last_tick;
    if (elapsed_ms < IMU_SAMPLE_PERIOD_MS) continue;

    last_tick = now;
    g_imu.dt_us = elapsed_ms * 1000U;
    IMU_System_ReadSensor();
    IMU_System_UpdateTemperature((float)elapsed_ms * 0.001f);
    IMU_System_Progress();

    if (!IMU_System_TemperatureInRange())
    {
      valid_samples = 0U;
      memset(sum, 0, sizeof(sum));
      IMU_System_WaitForTemperature();
      IMU_System_SetStage(IMU_SYSTEM_CALIBRATING);
      last_tick = g_config.get_tick_ms();
      continue;
    }

    float gyro_norm = sqrtf(g_imu.gyro[0] * g_imu.gyro[0] +
                            g_imu.gyro[1] * g_imu.gyro[1] +
                            g_imu.gyro[2] * g_imu.gyro[2]);
    float accel_norm = sqrtf(g_imu.accel[0] * g_imu.accel[0] +
                             g_imu.accel[1] * g_imu.accel[1] +
                             g_imu.accel[2] * g_imu.accel[2]);
    if (gyro_norm < IMU_STATIC_GYRO_MAX_RAD_S &&
        fabsf(accel_norm - 9.80665f) < IMU_STATIC_ACCEL_TOLERANCE)
    {
      for (uint8_t i = 0U; i < 3U; i++) sum[i] += g_imu.gyro[i];
      valid_samples++;
    }
    else
    {
      valid_samples = 0U;
      memset(sum, 0, sizeof(sum));
    }
  }

  g_imu.gyro_bias[0] = sum[0] / (float)IMU_GYRO_CALIBRATION_SAMPLES;
  g_imu.gyro_bias[1] = sum[1] / (float)IMU_GYRO_CALIBRATION_SAMPLES;
  g_imu.gyro_bias[2] = sum[2] / (float)IMU_GYRO_CALIBRATION_SAMPLES;
  BMI088_set_gyro_offset(g_imu.gyro_bias);
}

static void IMU_System_CollectInitialQuaternion(float q[4])
{
  float accel_sum[3] = {0.0f, 0.0f, 0.0f};
  uint16_t samples = 0U;
  uint32_t last_tick = g_config.get_tick_ms();

  while (samples < IMU_ACCEL_INIT_SAMPLES)
  {
    uint32_t now = g_config.get_tick_ms();
    uint32_t elapsed_ms = now - last_tick;
    if (elapsed_ms < IMU_SAMPLE_PERIOD_MS) continue;

    last_tick = now;
    g_imu.dt_us = elapsed_ms * 1000U;
    IMU_System_ReadSensor();
    IMU_System_UpdateTemperature((float)elapsed_ms * 0.001f);
    IMU_System_Progress();
    for (uint8_t i = 0U; i < 3U; i++) accel_sum[i] += g_imu.accel[i];
    samples++;
  }

  for (uint8_t i = 0U; i < 3U; i++) accel_sum[i] /= (float)IMU_ACCEL_INIT_SAMPLES;
  {
    float norm = sqrtf(accel_sum[0] * accel_sum[0] +
                       accel_sum[1] * accel_sum[1] +
                       accel_sum[2] * accel_sum[2]);
    float axis_x;
    float axis_y;
    float axis_norm;
    float half_angle;

    q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    if (norm < 1.0f) return;
    accel_sum[0] /= norm;
    accel_sum[1] /= norm;
    accel_sum[2] /= norm;
    axis_x = accel_sum[1];
    axis_y = -accel_sum[0];
    axis_norm = sqrtf(axis_x * axis_x + axis_y * axis_y);
    half_angle = 0.5f * acosf(fmaxf(-1.0f, fminf(1.0f, accel_sum[2])));
    if (axis_norm < 1.0e-6f) return;
    q[0] = cosf(half_angle);
    q[1] = axis_x * sinf(half_angle) / axis_norm;
    q[2] = axis_y * sinf(half_angle) / axis_norm;
  }
}

uint8_t IMU_System_Init(const imu_system_config_t *config)
{
  if (config == NULL || config->get_tick_ms == NULL ||
      config->heater_start == NULL || config->heater_set_pwm == NULL)
    return 0U;

  memset(&g_imu, 0, sizeof(g_imu));
  g_config = *config;
  g_stage_initialized = 0U;
  g_accel_filter_initialized = 0U;

  IMU_TemperatureController_Init(&g_temperature_controller,
                                g_config.heater_set_pwm);
  g_temperature_controller.target_temp = IMU_TEMPERATURE_TARGET_C;
  g_temperature_controller.pi_start_temp =
      g_temperature_controller.target_temp *
      (IMU_TEMPERATURE_PI_START_PERCENT / 100.0f);
  IMU_System_SetStage(IMU_SYSTEM_WARMING);
  if (BMI088_init() != BMI088_NO_ERROR) return 0U;

  /* 修改：加热前读取启动温度，冷启动和常温启动使用不同前馈。 */
  IMU_System_ReadSensor();
  g_temperature_controller.feed_forward_pwm =
      (g_imu.temperature < IMU_TEMPERATURE_COLD_START_THRESHOLD_C) ?
          IMU_TEMPERATURE_COLD_START_FEED_FORWARD_PWM :
          IMU_TEMPERATURE_NORMAL_START_FEED_FORWARD_PWM;

  if (!g_config.heater_start()) return 0U;

  IMU_System_WaitForTemperature();
  IMU_System_CalibrateGyro();
  {
    float init_q[4];
    IMU_System_CollectInitialQuaternion(init_q);
    IMU_DynamicEKF_Init(&g_imu.ekf, init_q, NULL,
                      10.0f, 0.001f, 1000000.0f, 1.0e-8f);
  }
  IMU_DynamicEKF_SetZBiasFrozen(&g_imu.ekf, 1U);

  g_imu.initialized = 1U;
  IMU_System_SetStage(IMU_SYSTEM_READY);
  IMU_System_Progress();
  return 1U;
}

void IMU_System_Update(float dt)
{
  if (!g_imu.initialized) return;
  g_imu.dt_us = (uint32_t)lroundf(dt * 1000000.0f);
  IMU_System_ReadSensor();
  IMU_System_UpdateTemperature(dt);
  IMU_DynamicEKF_Update(&g_imu.ekf, g_imu.gyro,
                        g_imu.accel_filtered, dt);
}

const imu_system_t *IMU_System_GetData(void)
{
  return &g_imu;
}
