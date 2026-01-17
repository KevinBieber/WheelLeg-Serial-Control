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

	K_data[0] = -554.5265f * L_3 + 320.8439f * L_2 + -62.6160f * L_1 + 3.3924f;
	K_data[1] = -65.0368f * L_3 + 36.0827f * L_2 + -6.4284f * L_1 + 0.2287f;
	K_data[2] = -147.1182f * L_3 + 85.3302f * L_2 + -16.8926f * L_1 + 0.6576f;
	K_data[3] = -248.5385f * L_3 + 144.2411f * L_2 + -28.4060f * L_1 + 1.0473f;
	K_data[4] = 83.5067f * L_3 + 162.3151f * L_2 + -91.1427f * L_1 + 12.7293f;
	K_data[5] = -11.2203f * L_3 + 13.9289f * L_2 + -4.9376f * L_1 + 0.6017f;
	K_data[6] = 1986.6381f * L_3 + -566.3321f * L_2 + -47.8981f * L_1 + 20.1167f;
	K_data[7] = -136.1618f * L_3 + 125.7071f * L_2 + -37.1597f * L_1 + 3.6803f;
	K_data[8] = 73.9794f * L_3 + 91.2915f * L_2 + -55.7171f * L_1 + 7.9697f;
	K_data[9] = -462.8655f * L_3 + 472.8848f * L_2 + -153.0546f * L_1 + 17.2691f;
	K_data[10] = 17778.1266f * L_3 + -10322.9485f * L_2 + 2044.7504f * L_1 + -76.5950f;
	K_data[11] = 527.9996f * L_3 + -299.1789f * L_2 + 57.1534f * L_1 + -1.4201f;
	
    if(leg_side == 0){
        arm_mat_mult_f32(&K_matrix, &X_matrix_left, &U_matrix_left);
    }
    else if(leg_side == 1){
        arm_mat_mult_f32(&K_matrix, &X_matrix_right, &U_matrix_right);
    }
}
