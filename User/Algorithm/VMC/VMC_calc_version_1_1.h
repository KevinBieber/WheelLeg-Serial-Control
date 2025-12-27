#ifndef VMC_CALC_VERSION_1_1_H
#define VMC_CALC_VERSION_1_1_H

#include "arm_math.h"

typedef struct{
    float phi;
    float phi_velocity;
    float phi_vel_last;
    float phi_accel;
    float leg_length;
    float leg_length_velocity;
}leg_position_t;

typedef struct{
    float l1;
    float theta1;
    float theta2;
    float theta1_velocity;
    float theta2_velocity;
    float jacobbi[4];
    leg_position_t leg_pos;
}vmc_leg_t;

void VMCInit(void);
void VMCDataPrepare(vmc_leg_t* vmc_legx, float* theta);
void legPosCalc(vmc_leg_t* vmc_legx, float* theta_velocity, float dt);
void VMCVirtual2RealCalc(vmc_leg_t* vmc_legx, float* real_torque, float* virtual_force);//将虚拟力转换为电机力矩
void VMCLegForceCalc(vmc_leg_t* vmc_legx, float* motor_torque, float* leg_force);//将电机力矩转换为腿部力
#endif
