#ifndef IMU_SYSTEM_H
#define IMU_SYSTEM_H

#include <stdint.h>

#include "imu_dynamic_ekf.h"

typedef enum
{
  IMU_SYSTEM_WARMING = 0U,
  IMU_SYSTEM_TEMPERATURE_STABLE,
  IMU_SYSTEM_CALIBRATING,
  IMU_SYSTEM_READY
} imu_system_stage_t;

typedef struct
{
  uint32_t (*get_tick_ms)(void);
  uint8_t (*heater_start)(void);
  void (*heater_set_pwm)(uint16_t pwm);
  void (*stage_changed)(imu_system_stage_t stage);
  void (*progress)(void);
} imu_system_config_t;

typedef struct
{
  float accel[3];
  float accel_filtered[3];
  float gyro[3];
  float gyro_bias[3];
  float temperature;
  uint32_t dt_us;
  uint8_t initialized;
  imu_dynamic_ekf_t ekf;
} imu_system_t;

uint8_t IMU_System_Init(const imu_system_config_t *config);
void IMU_System_Update(float dt);
const imu_system_t *IMU_System_GetData(void);

#endif
