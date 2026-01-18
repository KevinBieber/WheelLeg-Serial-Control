#include "VMC_calc_version_1_1.h"

vmc_leg_t vmc_leg_left;
vmc_leg_t vmc_leg_right;

void VMCInit(){
    vmc_leg_left.l1 = 0.070f;
    vmc_leg_right.l1 = 0.070f;
}

void VMCDataPrepare(vmc_leg_t* vmc_legx, float* theta){
    vmc_legx->theta1 = theta[0];
    vmc_legx->theta2 = theta[1];
    vmc_legx->jacobbi[0] = 0.5f;
    vmc_legx->jacobbi[1] = 0.5f;
    vmc_legx->jacobbi[2] = 2 * vmc_legx->l1 * arm_sin_f32((theta[1] - theta[0]) / 2.0f);
    vmc_legx->jacobbi[3] = -2 * vmc_legx->l1 * arm_cos_f32((theta[1] - theta[0]) / 2.0f);
}

void legPosCalc(vmc_leg_t* vmc_legx, float* theta_velocity, float dt){
    float theta[2] = {0};
    theta[0] = vmc_legx->theta1;
    theta[1] = vmc_legx->theta2;
    vmc_legx->theta1_velocity = theta_velocity[0];
    vmc_legx->theta2_velocity = theta_velocity[1];
    vmc_legx->leg_pos.phi = (theta[0] + theta[1]) / 2.0f;
    vmc_legx->leg_pos.leg_length = 4 * vmc_legx->l1 * arm_cos_f32((theta[0] - theta[1]) / 2.0f);
    vmc_legx->leg_pos.phi_velocity = (theta_velocity[0] + theta_velocity[1]) / 2.0f;
    vmc_legx->leg_pos.leg_length_velocity = -2 * vmc_legx->l1 * arm_sin_f32((theta[1] - theta[0]) / 2.0f) * (theta_velocity[1] - theta_velocity[0]);
    vmc_legx->leg_pos.phi_accel = (vmc_legx->leg_pos.phi_velocity - vmc_legx->leg_pos.phi_vel_last) / dt;
    vmc_legx->leg_pos.phi_vel_last = vmc_legx->leg_pos.phi_velocity;
	
	while(vmc_legx->leg_pos.phi > PI || vmc_legx->leg_pos.phi < -PI){
		if(vmc_legx->leg_pos.phi > PI){
			vmc_legx->leg_pos.phi -= 2*PI;
		}
		else if(vmc_legx->leg_pos.phi < -PI){
			vmc_legx->leg_pos.phi += 2*PI;
		}
	}
}

void VMCVirtual2RealCalc(vmc_leg_t* vmc_legx, float* real_torque, float* virtual_force){
    float trans_data[4];
    arm_matrix_instance_f32 jacobbi_matrix;
    arm_matrix_instance_f32 jacobbi_trans_matrix;
    arm_matrix_instance_f32 real_torque_matrix;
    arm_matrix_instance_f32 virtual_force_matrix;

    arm_mat_init_f32(&jacobbi_matrix, 2, 2, vmc_legx->jacobbi);
    arm_mat_init_f32(&jacobbi_trans_matrix, 2, 2, trans_data);
    arm_mat_init_f32(&real_torque_matrix, 2, 1, real_torque);
    arm_mat_init_f32(&virtual_force_matrix, 2, 1, virtual_force);
    arm_mat_trans_f32(&jacobbi_matrix, &jacobbi_trans_matrix);

    arm_mat_mult_f32(&jacobbi_trans_matrix, &virtual_force_matrix, &real_torque_matrix);
}

void VMCLegForceCalc(vmc_leg_t* vmc_legx, float* motor_torque, float* leg_force){
    float trans_data[4];
    float trans_inv_data[4];
    arm_matrix_instance_f32 jacobbi_matrix;
    arm_matrix_instance_f32 jacobbi_trans_matrix;
    arm_matrix_instance_f32 jacobbi_trans_inv_matrix;
    arm_matrix_instance_f32 motor_torque_matrix;
    arm_matrix_instance_f32 leg_force_matrix;

    arm_mat_init_f32(&jacobbi_matrix, 2, 2, vmc_legx->jacobbi);
    arm_mat_init_f32(&jacobbi_trans_matrix, 2, 2, trans_data);
    arm_mat_init_f32(&jacobbi_trans_inv_matrix, 2, 2, trans_inv_data);
    arm_mat_init_f32(&motor_torque_matrix, 2, 1, motor_torque);
    arm_mat_init_f32(&leg_force_matrix, 2, 1, leg_force);

    arm_mat_trans_f32(&jacobbi_matrix, &jacobbi_trans_matrix);
    arm_mat_inverse_f32(&jacobbi_trans_matrix, &jacobbi_trans_inv_matrix);

    arm_mat_mult_f32(&jacobbi_trans_inv_matrix, &motor_torque_matrix, &leg_force_matrix);
}
