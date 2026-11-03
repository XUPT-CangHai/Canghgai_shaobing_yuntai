#ifndef IMU_TEMPERATURE_CONTROL_H
#define IMU_TEMPERATURE_CONTROL_H

#include <stdint.h>

typedef void (*imu_temperature_pwm_setter_t)(uint16_t pwm);

typedef struct
{
  float target_temp;
  float pi_start_temp;
  float kp;
  float ki;
  float integral_limit;
  float feed_forward_pwm;
  uint16_t fast_pwm;
  uint16_t pi_max_pwm;
  float integral;
  uint16_t pwm;
  uint8_t pi_started;
  imu_temperature_pwm_setter_t set_pwm;
} imu_temperature_controller_t;

void IMU_TemperatureController_Init(imu_temperature_controller_t *controller,
                                    imu_temperature_pwm_setter_t setter);
void IMU_TemperatureController_Update(imu_temperature_controller_t *controller,
                                      float temperature, float dt);
void IMU_TemperatureController_Stop(imu_temperature_controller_t *controller);
uint8_t IMU_TemperatureController_PIStarted(const imu_temperature_controller_t *controller);
uint16_t IMU_TemperatureController_PWM(const imu_temperature_controller_t *controller);

#endif
