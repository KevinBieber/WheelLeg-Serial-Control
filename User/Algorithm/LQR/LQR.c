#include "LQR.h"

inline float K_data[12];

arm_matrix_instance_f32 K_matrix;
arm_matrix_instance_f32 X_matrix_left;
arm_matrix_instance_f32 X_matrix_right;
arm_matrix_instance_f32 U_matrix_left;
arm_matrix_instance_f32 U_matrix_right;

void initLQR(float* x_status_left, float* u_control_left, float* x_status_right, float* u_control_right){
    arm_mat_init_f32(&K_matrix, 2, 6, K_data);
    arm_mat_init_f32(&X_matrix_left, 6, 1, x_status_left);
    arm_mat_init_f32(&U_matrix_left, 2, 1, u_control_left);
    arm_mat_init_f32(&X_matrix_right, 6, 1, x_status_right);
    arm_mat_init_f32(&U_matrix_right, 2, 1, u_control_right);
}

void LQRCalculate(float leg_length,uint8_t leg_side){//leg_side:0-left leg,1-right leg
    float L_1 = leg_length;
    float L_2 = leg_length * leg_length;
    float L_3 = leg_length * leg_length * leg_length;

    K_data[1] = -1.2152 * L_3 + 5.6587 * L_2 + -2.0705 * L_1 + 3.6191;
    K_data[2] = -0.1409 * L_3 + 1.9166 * L_2 + -0.2697 * L_1 + 0.7642;
    K_data[3] = 0.1648 * L_3 + -0.7359 * L_2 + 1.1233 * L_1 + 0.3836;
    K_data[4] = 0.5098 * L_3 + -2.0674 * L_2 + 3.1236 * L_1 + 0.5566;
    K_data[5] = 1.2774 * L_3 + -2.6755 * L_2 + -4.6717 * L_1 + 12.9793;
    K_data[6] = 0.0617 * L_3 + -0.0571 * L_2 + -0.5325 * L_1 + 1.5821;
    K_data[7] = 3.3781 * L_3 + -20.5734 * L_2 + 41.7678 * L_1 + -45.6411;
    K_data[8] = -0.4175 * L_3 + 0.2510 * L_2 + 1.7101 * L_1 + -9.6199;
    K_data[9] = 0.3125 * L_3 + -2.5879 * L_2 + 7.6388 * L_1 + -8.6095;
    K_data[10] = -0.6761 * L_3 + 0.2446 * L_2 + 8.0646 * L_1 + -14.8759;
    K_data[11] = 13.1726 * L_3 + -68.1842 * L_2 + 126.9402 * L_1 + 37.8255;
    K_data[12] = 1.0660 * L_3 + -5.5060 * L_2 + 10.8483 * L_1 + 1.1334;

    if(leg_side == 0){
        arm_mat_mult_f32(&K_matrix, &X_matrix_left, &U_matrix_left);
    }
    else if(leg_side == 1){
        arm_mat_mult_f32(&K_matrix, &X_matrix_right, &U_matrix_right);
    }
}
