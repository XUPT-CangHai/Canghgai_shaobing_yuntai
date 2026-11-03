#include "imu_temperature_control.h"

#include <math.h>

void IMU_TemperatureController_Init(imu_temperature_controller_t *c,
                                    imu_temperature_pwm_setter_t setter)
{
  if (c == 0) return;
  *c = (imu_temperature_controller_t){
      .target_temp = 40.0f,
      .pi_start_temp = 39.5f,
      .kp = 1000.0f,
      .ki = 20.0f,
      .integral_limit = 1000.0f,
      .fast_pwm = 4500U,
      .pi_max_pwm = 2000U,
      .set_pwm = setter};
}

void IMU_TemperatureController_Update(imu_temperature_controller_t *c,
                                      float temperature, float dt)
{
  float output;
  if (c == 0 || !isfinite(temperature)) return;
  if (dt < 0.0f) dt = 0.0f;
  if (!c->pi_started && temperature >= c->pi_start_temp)
    c->pi_started = 1U;
  if (!c->pi_started)
  {
    c->pwm = c->fast_pwm;
  }
  else
  {
    c->integral += c->ki * (c->target_temp - temperature) * dt;
    c->integral = fmaxf(-c->integral_limit, fminf(c->integral_limit, c->integral));
    /* 前馈独立提供保温功率，积分只补偿剩余温差。 */
    output = c->feed_forward_pwm +
             c->kp * (c->target_temp - temperature) + c->integral;
    output = fmaxf(0.0f, fminf((float)c->pi_max_pwm, output));
    c->pwm = (uint16_t)output;
  }
  if (c->set_pwm != 0) c->set_pwm(c->pwm);
}

void IMU_TemperatureController_Stop(imu_temperature_controller_t *c)
{
  if (c == 0) return;
  c->pwm = 0U;
  if (c->set_pwm != 0) c->set_pwm(0U);
}

uint8_t IMU_TemperatureController_PIStarted(const imu_temperature_controller_t *c)
{
  return (c != 0) ? c->pi_started : 0U;
}

uint16_t IMU_TemperatureController_PWM(const imu_temperature_controller_t *c)
{
  return (c != 0) ? c->pwm : 0U;
}
