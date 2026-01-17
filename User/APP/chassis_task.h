#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "VMC_calc_version_1_1.h"
#include "INS_task.h"
#include "LQR.h"
#include "remote_control.h"

typedef enum{
    CHASSIS_STATE_INIT=0,//初始化
    CHASSIS_STATE_REST,//失能
    CHASSIS_STATE_STANDUP,//起身
    CHASSIS_STATE_COMMON,//正常跑跳
    CHASSIS_STATE_FAULT//状态不对，倒了或者死了
}chassis_status_t;

typedef enum{
    CHASSIS_RUN = 0,
    CHASSIS_JUMP,
}common_mode_t;

typedef enum{
    JUMP_STATE_PREPARE=0,//准备跳跃，压腿
    JUMP_STATE_TAKEOFF,//起跳
    JUMP_STATE_FLIGHT,//飞行
    JUMP_STATE_LANDING,//落地
    JUMP_STATE_RECOVER,//恢复站立
    JUMP_STATE_NONE//非跳跃状态
}jump_status_t;

typedef enum{
    STANDUP_STATE_START=0,
    STANDUP_STATE_MID,
    STANDUP_STATE_END
}standup_status_t;

typedef struct{
    float v_x_target;
    float w_target;

    chassis_status_t chassis_status;
    common_mode_t common_mode;
    jump_status_t jump_status;
    standup_status_t standup_status;
}ctrl_data_t;

typedef struct{
    Joint_Motor_t leg_motor[2];//前面的是0号关节，后面的是1号关节
    Wheel_Motor_t wheel_motor;
    vmc_leg_t* vmc_leg_x;
    float leg_motor_torque[2];//发送给电机的力矩
    float wheel_motor_torque;//发送给轮毂电机的力矩
    float vmc_force[2];//虚拟力（用来算电机力）
    float lqr_force[2];
    float leg_force[2];//腿部实际力(用电机力算腿部力)
}chassis_leg_t;

typedef struct{
    float roll;
    float roll_gyro;
    float pitch;
    float pitch_gyro;
    float yaw;
    float yaw_gyro;
    float leg_length_left_target;
    float leg_length_right_target;
    float leg_force_ref;//腿支持力补偿
	float leg_phi_left_target;
	float leg_phi_right_target;
	
    float v_x_max;
    float w_max;
	float wheel_r;//轮半径
}chassis_t;

void initChassisTask(void);
void loadManualControl(void);
void updateChassisControl(void);
void chassis_task(void);
#endif
