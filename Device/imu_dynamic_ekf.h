#ifndef IMU_DYNAMIC_EKF_H
#define IMU_DYNAMIC_EKF_H

#include <stdint.h>

/* Six states: q0..q3 and residual gyro bias X/Y. */
typedef struct
{
  float q[4];
  float gyro_bias[3];
  float yaw;
  float pitch;
  float roll;
  float innovation[3];
  float chi_square;
  float accel_norm;
  float gyro_norm;
  float process_noise_q;
  float process_noise_bias;
  float measurement_noise;
  float chi_square_threshold;
  float lambda;
  float adaptive_gain_scale;
  uint32_t update_count;
  uint32_t error_count;
  uint8_t accel_valid;
  uint8_t converged;
  uint8_t stable;
  uint8_t z_bias_frozen;
  float _p[36];
} imu_dynamic_ekf_t;

/* init_q may be NULL (identity is used); init_bias may be NULL (zero is used). */
void IMU_DynamicEKF_Init(imu_dynamic_ekf_t *ekf,
                         const float init_q[4],
                         const float init_bias[3],
                         float process_noise_q,
                         float process_noise_bias,
                         float measurement_noise,
                         float chi_square_threshold);

/* Kept for the existing API. Z bias is not an EKF state and always remains zero. */
void IMU_DynamicEKF_SetZBiasFrozen(imu_dynamic_ekf_t *ekf, uint8_t frozen);

/* gyro: rad/s, accel: m/s^2, dt: seconds. */
void IMU_DynamicEKF_Update(imu_dynamic_ekf_t *ekf,
                           const float gyro[3],
                           const float accel[3],
                           float dt);

#endif
