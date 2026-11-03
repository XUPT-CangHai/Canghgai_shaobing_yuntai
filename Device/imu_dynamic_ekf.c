#include "imu_dynamic_ekf.h"

#include <math.h>
#include <string.h>

#define IMU_EKF_G 9.80665f
#define IMU_EKF_MIN_DT 0.00001f
#define IMU_EKF_MAX_DT 0.1f
#define IMU_EKF_MAX_BIAS_STEP 0.01f
#define IMU_EKF_STATE_COUNT 6U
#define IMU_EKF_HALF_PI 1.57079632679f

static float Square(float value)
{
  return value * value;
}

static float Clamp(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

static void NormalizeQuaternion(float q[4])
{
  float norm = sqrtf(Square(q[0]) + Square(q[1]) +
                     Square(q[2]) + Square(q[3]));
  if (norm < 1.0e-9f)
  {
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    return;
  }
  for (uint8_t i = 0U; i < 4U; i++) q[i] /= norm;
}

static void PredictGravity(const float q[4], float gravity[3])
{
  gravity[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
  gravity[1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
  gravity[2] = Square(q[0]) - Square(q[1]) - Square(q[2]) + Square(q[3]);
}

static void SetMeasurementJacobian(const float q[4], float h[18])
{
  float q0 = 2.0f * q[0];
  float q1 = 2.0f * q[1];
  float q2 = 2.0f * q[2];
  float q3 = 2.0f * q[3];

  memset(h, 0, 18U * sizeof(float));
  h[0] = -q2; h[1] = q3;  h[2] = -q0; h[3] = q1;
  h[6] = q1;  h[7] = q0;  h[8] = q3;  h[9] = q2;
  h[12] = q0; h[13] = -q1; h[14] = -q2; h[15] = q3;
}

static uint8_t Invert3x3(const float a[9], float inverse[9])
{
  float c00 = a[4] * a[8] - a[5] * a[7];
  float c01 = a[2] * a[7] - a[1] * a[8];
  float c02 = a[1] * a[5] - a[2] * a[4];
  float c10 = a[5] * a[6] - a[3] * a[8];
  float c11 = a[0] * a[8] - a[2] * a[6];
  float c12 = a[2] * a[3] - a[0] * a[5];
  float c20 = a[3] * a[7] - a[4] * a[6];
  float c21 = a[1] * a[6] - a[0] * a[7];
  float c22 = a[0] * a[4] - a[1] * a[3];
  float determinant = a[0] * c00 + a[1] * c10 + a[2] * c20;

  if (fabsf(determinant) < 1.0e-12f) return 0U;
  determinant = 1.0f / determinant;
  inverse[0] = c00 * determinant;
  inverse[1] = c01 * determinant;
  inverse[2] = c02 * determinant;
  inverse[3] = c10 * determinant;
  inverse[4] = c11 * determinant;
  inverse[5] = c12 * determinant;
  inverse[6] = c20 * determinant;
  inverse[7] = c21 * determinant;
  inverse[8] = c22 * determinant;
  return 1U;
}

static void InitializeCovariance(float p[36])
{
  for (uint8_t row = 0U; row < IMU_EKF_STATE_COUNT; row++)
    for (uint8_t column = 0U; column < IMU_EKF_STATE_COUNT; column++)
      p[row * IMU_EKF_STATE_COUNT + column] =
          (row == column) ? ((row < 4U) ? 100000.0f : 100.0f) : 0.1f;
}

void IMU_DynamicEKF_Init(imu_dynamic_ekf_t *ekf,
                         const float init_q[4], const float init_bias[3],
                         float process_noise_q, float process_noise_bias,
                         float measurement_noise, float chi_square_threshold)
{
  if (ekf == NULL) return;

  memset(ekf, 0, sizeof(*ekf));
  ekf->q[0] = 1.0f;
  if (init_q != NULL) memcpy(ekf->q, init_q, 4U * sizeof(float));
  NormalizeQuaternion(ekf->q);
  if (init_bias != NULL)
  {
    ekf->gyro_bias[0] = init_bias[0];
    ekf->gyro_bias[1] = init_bias[1];
  }
  ekf->gyro_bias[2] = 0.0f;
  ekf->process_noise_q = (process_noise_q > 0.0f) ? process_noise_q : 10.0f;
  ekf->process_noise_bias =
      (process_noise_bias > 0.0f) ? process_noise_bias : 0.001f;
  ekf->measurement_noise =
      (measurement_noise > 0.0f) ? measurement_noise : 1000000.0f;
  ekf->chi_square_threshold =
      (chi_square_threshold > 0.0f) ? chi_square_threshold : 1.0e-8f;
  ekf->lambda = 1.0f;
  ekf->adaptive_gain_scale = 1.0f;
  ekf->z_bias_frozen = 1U;
  InitializeCovariance(ekf->_p);
}

void IMU_DynamicEKF_SetZBiasFrozen(imu_dynamic_ekf_t *ekf, uint8_t frozen)
{
  if (ekf == NULL) return;
  ekf->z_bias_frozen = (frozen != 0U) ? 1U : 0U;
  ekf->gyro_bias[2] = 0.0f;
}

void IMU_DynamicEKF_Update(imu_dynamic_ekf_t *ekf,
                           const float gyro[3], const float accel[3], float dt)
{
  float x[IMU_EKF_STATE_COUNT];
  float xminus[IMU_EKF_STATE_COUNT];
  float f[IMU_EKF_STATE_COUNT * IMU_EKF_STATE_COUNT];
  float p[IMU_EKF_STATE_COUNT * IMU_EKF_STATE_COUNT];
  float pminus[IMU_EKF_STATE_COUNT * IMU_EKF_STATE_COUNT];
  float h[18];
  float s[9];
  float sinv[9];
  float gain[18];
  float innovation[3];
  float gravity[3];
  float correction[IMU_EKF_STATE_COUNT];
  float corrected_gyro[3];
  float accel_norm;
  float gyro_norm;
  float chi_square = 0.0f;
  uint8_t measurement_update = 0U;

  if (ekf == NULL || gyro == NULL || accel == NULL) return;
  dt = Clamp(dt, IMU_EKF_MIN_DT, IMU_EKF_MAX_DT);

  memcpy(x, ekf->q, 4U * sizeof(float));
  x[4] = ekf->gyro_bias[0];
  x[5] = ekf->gyro_bias[1];
  memcpy(p, ekf->_p, sizeof(p));

  corrected_gyro[0] = gyro[0] - x[4];
  corrected_gyro[1] = gyro[1] - x[5];
  corrected_gyro[2] = gyro[2];
  gyro_norm = sqrtf(Square(corrected_gyro[0]) +
                    Square(corrected_gyro[1]) +
                    Square(corrected_gyro[2]));

  xminus[0] = x[0] + 0.5f *
      (-x[1] * corrected_gyro[0] - x[2] * corrected_gyro[1] -
       x[3] * corrected_gyro[2]) * dt;
  xminus[1] = x[1] + 0.5f *
      (x[0] * corrected_gyro[0] + x[2] * corrected_gyro[2] -
       x[3] * corrected_gyro[1]) * dt;
  xminus[2] = x[2] + 0.5f *
      (x[0] * corrected_gyro[1] - x[1] * corrected_gyro[2] +
       x[3] * corrected_gyro[0]) * dt;
  xminus[3] = x[3] + 0.5f *
      (x[0] * corrected_gyro[2] + x[1] * corrected_gyro[1] -
       x[2] * corrected_gyro[0]) * dt;
  xminus[4] = x[4];
  xminus[5] = x[5];
  NormalizeQuaternion(xminus);

  memset(f, 0, sizeof(f));
  for (uint8_t i = 0U; i < IMU_EKF_STATE_COUNT; i++)
    f[i * IMU_EKF_STATE_COUNT + i] = 1.0f;
  f[1] = -0.5f * corrected_gyro[0] * dt;
  f[2] = -0.5f * corrected_gyro[1] * dt;
  f[3] = -0.5f * corrected_gyro[2] * dt;
  f[6] = 0.5f * corrected_gyro[0] * dt;
  f[8] = 0.5f * corrected_gyro[2] * dt;
  f[9] = -0.5f * corrected_gyro[1] * dt;
  f[12] = 0.5f * corrected_gyro[1] * dt;
  f[13] = -0.5f * corrected_gyro[2] * dt;
  f[15] = 0.5f * corrected_gyro[0] * dt;
  f[18] = 0.5f * corrected_gyro[2] * dt;
  f[19] = 0.5f * corrected_gyro[1] * dt;
  f[20] = -0.5f * corrected_gyro[0] * dt;
  f[4] = 0.5f * xminus[1] * dt;
  f[5] = 0.5f * xminus[2] * dt;
  f[10] = -0.5f * xminus[0] * dt;
  f[11] = 0.5f * xminus[3] * dt;
  f[16] = -0.5f * xminus[3] * dt;
  f[17] = -0.5f * xminus[0] * dt;
  f[22] = 0.5f * xminus[2] * dt;
  f[23] = -0.5f * xminus[1] * dt;

  p[28] = fminf(p[28] / ekf->lambda, 10000.0f);
  p[35] = fminf(p[35] / ekf->lambda, 10000.0f);
  memset(pminus, 0, sizeof(pminus));
  for (uint8_t row = 0U; row < IMU_EKF_STATE_COUNT; row++)
    for (uint8_t column = 0U; column < IMU_EKF_STATE_COUNT; column++)
      for (uint8_t left = 0U; left < IMU_EKF_STATE_COUNT; left++)
        for (uint8_t right = 0U; right < IMU_EKF_STATE_COUNT; right++)
          pminus[row * IMU_EKF_STATE_COUNT + column] +=
              f[row * IMU_EKF_STATE_COUNT + left] *
              p[left * IMU_EKF_STATE_COUNT + right] *
              f[column * IMU_EKF_STATE_COUNT + right];
  for (uint8_t i = 0U; i < 4U; i++)
    pminus[i * IMU_EKF_STATE_COUNT + i] += ekf->process_noise_q * dt;
  pminus[28] += ekf->process_noise_bias * dt;
  pminus[35] += ekf->process_noise_bias * dt;

  accel_norm = sqrtf(Square(accel[0]) + Square(accel[1]) + Square(accel[2]));
  ekf->accel_norm = accel_norm;
  ekf->gyro_norm = gyro_norm;
  ekf->stable = (uint8_t)(gyro_norm < 0.3f &&
      accel_norm > IMU_EKF_G - 0.5f && accel_norm < IMU_EKF_G + 0.5f);
  ekf->accel_valid = 0U;
  ekf->chi_square = 0.0f;

  if (accel_norm > 1.0f)
  {
    for (uint8_t i = 0U; i < 3U; i++) innovation[i] = accel[i] / accel_norm;
    PredictGravity(xminus, gravity);
    for (uint8_t i = 0U; i < 3U; i++)
    {
      innovation[i] -= gravity[i];
      ekf->innovation[i] = innovation[i];
    }
    SetMeasurementJacobian(xminus, h);

    for (uint8_t row = 0U; row < 3U; row++)
      for (uint8_t column = 0U; column < 3U; column++)
      {
        s[row * 3U + column] = (row == column) ? ekf->measurement_noise : 0.0f;
        for (uint8_t left = 0U; left < IMU_EKF_STATE_COUNT; left++)
          for (uint8_t right = 0U; right < IMU_EKF_STATE_COUNT; right++)
            s[row * 3U + column] +=
                h[row * IMU_EKF_STATE_COUNT + left] *
                pminus[left * IMU_EKF_STATE_COUNT + right] *
                h[column * IMU_EKF_STATE_COUNT + right];
      }

    if (Invert3x3(s, sinv))
    {
      uint8_t reject_measurement = 0U;
      for (uint8_t row = 0U; row < 3U; row++)
        for (uint8_t column = 0U; column < 3U; column++)
          chi_square += innovation[row] * sinv[row * 3U + column] *
                        innovation[column];
      ekf->chi_square = chi_square;
      if (chi_square < 0.5f * ekf->chi_square_threshold) ekf->converged = 1U;

      if (chi_square > ekf->chi_square_threshold && ekf->converged)
      {
        if (ekf->stable) ekf->error_count++;
        else ekf->error_count = 0U;
        if (ekf->error_count > 50U) ekf->converged = 0U;
        else reject_measurement = 1U;
      }
      else
      {
        if (chi_square > 0.1f * ekf->chi_square_threshold && ekf->converged)
          ekf->adaptive_gain_scale =
              (ekf->chi_square_threshold - chi_square) /
              (0.9f * ekf->chi_square_threshold);
        else
          ekf->adaptive_gain_scale = 1.0f;
        ekf->error_count = 0U;
      }

      if (!reject_measurement)
      {
        float orientation_x = acosf(Clamp(fabsf(gravity[0]), 0.0f, 1.0f));
        float orientation_y = acosf(Clamp(fabsf(gravity[1]), 0.0f, 1.0f));

        for (uint8_t row = 0U; row < IMU_EKF_STATE_COUNT; row++)
          for (uint8_t column = 0U; column < 3U; column++)
          {
            gain[row * 3U + column] = 0.0f;
            for (uint8_t left = 0U; left < IMU_EKF_STATE_COUNT; left++)
              for (uint8_t right = 0U; right < 3U; right++)
                gain[row * 3U + column] +=
                    pminus[row * IMU_EKF_STATE_COUNT + left] *
                    h[right * IMU_EKF_STATE_COUNT + left] *
                    sinv[right * 3U + column];
            gain[row * 3U + column] *= ekf->adaptive_gain_scale;
            if (row == 4U) gain[row * 3U + column] *= orientation_x / IMU_EKF_HALF_PI;
            if (row == 5U) gain[row * 3U + column] *= orientation_y / IMU_EKF_HALF_PI;
          }

        for (uint8_t row = 0U; row < IMU_EKF_STATE_COUNT; row++)
        {
          correction[row] = 0.0f;
          for (uint8_t column = 0U; column < 3U; column++)
            correction[row] += gain[row * 3U + column] * innovation[column];
        }
        if (ekf->converged)
        {
          correction[4] = Clamp(correction[4], -IMU_EKF_MAX_BIAS_STEP * dt,
                                IMU_EKF_MAX_BIAS_STEP * dt);
          correction[5] = Clamp(correction[5], -IMU_EKF_MAX_BIAS_STEP * dt,
                                IMU_EKF_MAX_BIAS_STEP * dt);
        }
        correction[3] = 0.0f;
        for (uint8_t i = 0U; i < IMU_EKF_STATE_COUNT; i++)
          xminus[i] += correction[i];

        for (uint8_t row = 0U; row < IMU_EKF_STATE_COUNT; row++)
          for (uint8_t column = 0U; column < IMU_EKF_STATE_COUNT; column++)
          {
            p[row * IMU_EKF_STATE_COUNT + column] =
                pminus[row * IMU_EKF_STATE_COUNT + column];
            for (uint8_t measurement = 0U; measurement < 3U; measurement++)
              for (uint8_t state = 0U; state < IMU_EKF_STATE_COUNT; state++)
                p[row * IMU_EKF_STATE_COUNT + column] -=
                    gain[row * 3U + measurement] *
                    h[measurement * IMU_EKF_STATE_COUNT + state] *
                    pminus[state * IMU_EKF_STATE_COUNT + column];
          }
        measurement_update = 1U;
      }
    }
  }

  if (!measurement_update) memcpy(p, pminus, sizeof(p));
  memcpy(ekf->q, xminus, 4U * sizeof(float));
  ekf->gyro_bias[0] = xminus[4];
  ekf->gyro_bias[1] = xminus[5];
  ekf->gyro_bias[2] = 0.0f;
  ekf->accel_valid = measurement_update;

  ekf->yaw = atan2f(2.0f * (ekf->q[0] * ekf->q[3] +
                            ekf->q[1] * ekf->q[2]),
                     2.0f * (Square(ekf->q[0]) + Square(ekf->q[1])) - 1.0f);
  ekf->pitch = atan2f(2.0f * (ekf->q[0] * ekf->q[1] +
                              ekf->q[2] * ekf->q[3]),
                       2.0f * (Square(ekf->q[0]) + Square(ekf->q[3])) - 1.0f);
  ekf->roll = asinf(Clamp(-2.0f * (ekf->q[1] * ekf->q[3] -
                                    ekf->q[0] * ekf->q[2]), -1.0f, 1.0f));
  ekf->update_count++;
  memcpy(ekf->_p, p, sizeof(p));
}
