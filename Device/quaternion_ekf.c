#include "quaternion_ekf.h"
#include <math.h>
#define EKF_G 9.80665f
#define EKF_CHI2 0.25f
static float invsqrtf(float x){return 1.0f/sqrtf(x);}
static void norm4(float *q){float n=invsqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]); for(int i=0;i<4;i++)q[i]*=n;}
void QuaternionEKF_Init(quaternion_ekf_t *e){*e=(quaternion_ekf_t){0};e->q[0]=1.0f;}
void QuaternionEKF_Update(quaternion_ekf_t *e,const float g[3],const float a[3],float dt){
 float q0=e->q[0],q1=e->q[1],q2=e->q[2],q3=e->q[3];
 float wx=g[0]-e->gyro_bias[0],wy=g[1]-e->gyro_bias[1],wz=g[2]-e->gyro_bias[2];
 e->q[0]+=0.5f*(-q1*wx-q2*wy-q3*wz)*dt; e->q[1]+=0.5f*(q0*wx+q2*wz-q3*wy)*dt; e->q[2]+=0.5f*(q0*wy-q1*wz+q3*wx)*dt; e->q[3]+=0.5f*(q0*wz+q1*wy-q2*wx)*dt; norm4(e->q);
 float an=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]); float r=an-EKF_G; e->innovation=r*r; e->accel_valid=(uint8_t)(e->innovation<EKF_CHI2 && an>1.0f);
 if(e->accel_valid){float ax=a[0]/an,ay=a[1]/an,az=a[2]/an; float vx=2*(q1*q3-q0*q2),vy=2*(q0*q1+q2*q3),vz=q0*q0-q1*q1-q2*q2+q3*q3; float ex=ay*vz-az*vy,ey=az*vx-ax*vz,ez=ax*vy-ay*vx; const float k=0.08f; e->q[1]+=k*ex*dt; e->q[2]+=k*ey*dt; e->q[3]+=k*ez*dt; norm4(e->q);}
 q0=e->q[0];q1=e->q[1];q2=e->q[2];q3=e->q[3]; e->roll=atan2f(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2)); e->pitch=asinf(fmaxf(-1.0f,fminf(1.0f,2*(q0*q2-q3*q1)))); e->yaw=atan2f(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));}
