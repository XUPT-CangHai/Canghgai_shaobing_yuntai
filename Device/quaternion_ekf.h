#ifndef QUATERNION_EKF_H
#define QUATERNION_EKF_H
#include <stdint.h>
typedef struct { float q[4]; float gyro_bias[3]; float yaw,pitch,roll; uint8_t accel_valid; float innovation; } quaternion_ekf_t;
void QuaternionEKF_Init(quaternion_ekf_t *ekf);
void QuaternionEKF_Update(quaternion_ekf_t *ekf,const float gyro[3],const float accel[3],float dt);
#endif
