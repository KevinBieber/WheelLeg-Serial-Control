#ifndef LQR_H
#define LQR_H

#include "arm_math.h"

void initLQR(float* x_status_left, float* u_control_left, float* x_status_right, float* u_control_right);
void LQRCalculate(float leg_length,uint8_t leg_side);//leg_side:0-left leg,1-right leg

#endif