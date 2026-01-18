#include "LQR.h"

float K_data[12];

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

K_data[0] = 145.2013f * L_3 + -103.1212f * L_2 + 31.7539f * L_1 + 1.0995f;
K_data[1] = 6.1566f * L_3 + -5.0574f * L_2 + 2.8430f * L_1 + 0.2856f;
K_data[2] = 9.3986f * L_3 + -7.0654f * L_2 + 1.9421f * L_1 + 0.2955f;
K_data[3] = 10.4928f * L_3 + -8.7971f * L_2 + 2.7075f * L_1 + 0.6766f;
K_data[4] = -8.0486f * L_3 + 50.8712f * L_2 + -30.7259f * L_1 + 6.5019f;
K_data[5] = -5.3554f * L_3 + 5.1999f * L_2 + -2.0326f * L_1 + 0.3941f;
K_data[6] = -197.9657f * L_3 + 81.2551f * L_2 + 0.0877f * L_1 + -4.2081f;
K_data[7] = -9.3239f * L_3 + -0.8952f * L_2 + 3.2107f * L_1 + -1.0195f;
K_data[8] = -2.6168f * L_3 + -5.5550f * L_2 + 4.2261f * L_1 + -0.9406f;
K_data[9] = 19.0864f * L_3 + -24.9618f * L_2 + 11.1116f * L_1 + -2.0481f;
K_data[10] = 377.0854f * L_3 + -272.9152f * L_2 + 72.5907f * L_1 + 8.5444f;
K_data[11] = 18.0383f * L_3 + -12.9474f * L_2 + 3.4603f * L_1 + 0.3673f;
	
    if(leg_side == 0){
        arm_mat_mult_f32(&K_matrix, &X_matrix_left, &U_matrix_left);
    }
    else if(leg_side == 1){
        arm_mat_mult_f32(&K_matrix, &X_matrix_right, &U_matrix_right);
    }
}
