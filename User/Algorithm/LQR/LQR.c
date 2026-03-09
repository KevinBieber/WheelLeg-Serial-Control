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

float phiTargetCalc(float leg_length){
    float L_1 = leg_length;
    float L_2 = leg_length * leg_length;
    float L_3 = leg_length * leg_length * leg_length;
	float phi_target = 209.0047f * L_3 + -99.7388f * L_2 + 13.8301f * L_1 + -0.3304f;
	return phi_target;
}

void LQRCalculate(float leg_length,uint8_t leg_side){//leg_side:0-left leg,1-right leg
    float L_1 = leg_length;
    float L_2 = leg_length * leg_length;
    float L_3 = leg_length * leg_length * leg_length;

K_data[0] = -631.0761f * L_3 + 285.2972f * L_2 + -30.0604f * L_1 + 4.0503f;
K_data[1] = 20.3849f * L_3 + -12.0844f * L_2 + 3.9481f * L_1 + 0.2329f;
K_data[2] = 16.6077f * L_3 + -10.6351f * L_2 + 2.5092f * L_1 + 0.2674f;
K_data[3] = 34.9595f * L_3 + -21.1920f * L_2 + 4.7289f * L_1 + 0.5742f;
K_data[4] = -261.8351f * L_3 + 175.3150f * L_2 + -49.9900f * L_1 + 7.3890f;
K_data[5] = -13.3726f * L_3 + 9.1519f * L_2 + -2.6463f * L_1 + 0.4221f;
K_data[6] = 410.1481f * L_3 + -220.5179f * L_2 + 47.5906f * L_1 + -6.4529f;
K_data[7] = 27.2424f * L_3 + -19.0052f * L_2 + 6.0616f * L_1 + -1.1553f;
K_data[8] = 25.3229f * L_3 + -19.6293f * L_2 + 6.5039f * L_1 + -1.0551f;
K_data[9] = 63.1864f * L_3 + -46.9804f * L_2 + 14.6329f * L_1 + -2.2218f;
K_data[10] = 585.5012f * L_3 + -373.0637f * L_2 + 87.8715f * L_1 + 7.8268f;
K_data[11] = 30.6295f * L_3 + -19.0308f * L_2 + 4.3894f * L_1 + 0.3245f;
	
    if(leg_side == 0){
        arm_mat_mult_f32(&K_matrix, &X_matrix_left, &U_matrix_left);
    }
    else if(leg_side == 1){
        arm_mat_mult_f32(&K_matrix, &X_matrix_right, &U_matrix_right);
    }
}
