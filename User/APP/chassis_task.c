#include "chassis_task.h"
#include "cmsis_os.h"
#include "usb_receive_task.h"

extern rc_data_t rc_data;


extern INS_t INS;
extern vmc_leg_t vmc_leg_left;
extern vmc_leg_t vmc_leg_right;

float asd = 0.0f;

chassis_leg_t leg_left;
chassis_leg_t leg_right;
chassis_t bipe_chassis;
float x_left[6] = {0.0f};
float x_left_target[6] = {0.0f};
float x_right[6] = {0.0f};
float x_right_target[6] = {0.0f};
float u_left[2] = {0.0f};
float u_right[2] = {0.0f};
ctrl_data_t ctrl_data;

PidTypeDef LegR_Pid;//���ȵ��ȳ�pd
PidTypeDef LegL_Pid;//���ȵ��ȳ�pd
PidTypeDef Tp_Pid;//�����油��pd
PidTypeDef Turn_Pid;//ת��pd
PidTypeDef Roll_Pid;//����ǲ���pd
PidTypeDef PhiL_Pid;//���Ȱڽǲ���pd
PidTypeDef PhiR_Pid;//���Ȱڽǲ���pd
PidTypeDef PhiVelL_Pid;//���Ȱڽ��ٶȲ���pd
PidTypeDef PhiVelR_Pid;//���Ȱڽ��ٶȲ���pd

void mylimit_float(float* in, float maxlimit, float minlimit){
    if(*in > maxlimit){
        *in = maxlimit;
    }
    else if(*in < minlimit){
        *in = minlimit;
    }
}

void initChassisTask(void){
	while(INS.ins_flag==0)
	{//�ȴ����ٶ�����
	  osDelay(1);
	}
    float LegR_Pid_params[3] = {500.0f, 0.0f, 1.0f};//pid����
    float LegL_Pid_params[3] = {800.0f, 0.0f, 1.0f};
    float Tp_Pid_params[3] = {8.0f, 0.0f, 0.5f};
    float Turn_Pid_params[3] = {0.5f, 0.01f, 0.0f};
    float Roll_Pid_params[3] = {0.15f, 0.0f, 0.0f};
    float PhiL_Pid_params[3] = {15.0f, 0.1f, 0.1f};
    float PhiR_Pid_params[3] = {15.0f, 0.1f, 0.1f};
    float PhiVelL_Pid_params[3] = {10.0f, 0.0f, 1.0f};
    float PhiVelR_Pid_params[3] = {10.0f, 0.0f, 1.0f};
    PID_init(&LegR_Pid, PID_DELTA, LegR_Pid_params, 50.0f, 3.0f);
    PID_init(&LegL_Pid, PID_DELTA, LegL_Pid_params, 50.0f, 3.0f);
    PID_init(&Tp_Pid, PID_DELTA, Tp_Pid_params, 15.0f, 3.0f);
    PID_init(&Turn_Pid, PID_DELTA, Turn_Pid_params, 1.5f, 0.5f);
    PID_init(&Roll_Pid, PID_DELTA, Roll_Pid_params, 0.5f, 0.1f);
    PID_init(&PhiL_Pid, PID_DELTA, PhiL_Pid_params, 5.0f, 1.0f);
    PID_init(&PhiR_Pid, PID_DELTA, PhiR_Pid_params, 5.0f, 1.0f);
    PID_init(&PhiVelL_Pid, PID_DELTA, PhiVelL_Pid_params, 5.0f, 1.0f);
    PID_init(&PhiVelR_Pid, PID_DELTA, PhiVelR_Pid_params, 5.0f, 1.0f);
	
    bipe_chassis.leg_force_ref = 20.0f;
	bipe_chassis.v_x_max = 1.8f;
	bipe_chassis.w_max = 2.5f;
	bipe_chassis.wheel_r = 0.05f;
	

    leg_left.vmc_leg_x = &vmc_leg_left;
    leg_right.vmc_leg_x = &vmc_leg_right;
    VMCInit();
    
    initLQR(x_left, u_left, x_right, u_right);

	joint_motor_init(&leg_left.leg_motor[0],0x03,MIT_MODE);
	joint_motor_init(&leg_left.leg_motor[1],0x04,MIT_MODE);
	wheel_motor_init(&leg_left.wheel_motor,0x01,MIT_MODE);
    joint_motor_init(&leg_right.leg_motor[0],0x05,MIT_MODE);
    joint_motor_init(&leg_right.leg_motor[1],0x06,MIT_MODE);
    wheel_motor_init(&leg_right.wheel_motor,0x02,MIT_MODE);

	for(int j=0;j<5;j++)
	{
	    enable_motor_mode(&hfdcan1,leg_left.leg_motor[0].para.id,leg_left.leg_motor[0].mode);
	    osDelay(1);
	}
	for(int j=0;j<5;j++)
	{
	    enable_motor_mode(&hfdcan1,leg_left.leg_motor[1].para.id,leg_left.leg_motor[1].mode);
	    osDelay(1);
	}
	for(int j=0;j<5;j++)
	{
        enable_motor_mode(&hfdcan1,leg_left.wheel_motor.para.id,leg_left.wheel_motor.mode);//�����챵��
	    osDelay(1);
	}
	for(int j=0;j<5;j++)
	{
	    enable_motor_mode(&hfdcan1,leg_right.leg_motor[0].para.id,leg_right.leg_motor[0].mode);
	    osDelay(1);
	}
	for(int j=0;j<5;j++)
	{
	    enable_motor_mode(&hfdcan1,leg_right.leg_motor[1].para.id,leg_right.leg_motor[1].mode);
	    osDelay(1);
	}
	for(int j=0;j<5;j++)
	{
        enable_motor_mode(&hfdcan1,leg_right.wheel_motor.para.id,leg_right.wheel_motor.mode);//�ұ���챵��
	    osDelay(1);
	}

    ctrl_data.chassis_status = CHASSIS_STATE_INIT;
    ctrl_data.jump_status = JUMP_STATE_NONE;
    ctrl_data.standup_status = STANDUP_STATE_END;
}

void loadManualControl(void){
    /* SB: up=2 RC only; mid=1 both; down=0 PC only.
     * SA（遥控）与 PC control_mode 均可触发失能/起身（边沿）：
     *   control_mode: 0=跑动 1=失能(REST) 2=跳跃 3=起身(STANDUP) */
    #define PC_CMD_TIMEOUT_MS  500U

    if(rc_data.state == REMOTE_ON){
        static uint8_t last_SA = 0;
        static uint8_t last_SB = 0;
        static uint8_t last_SD = 0;
        static uint8_t last_pc_mode = 0xFF;

        if(rc_data.swich_SA == 3) rc_data.swich_SA = last_SA;
        if(rc_data.swich_SB == 3) rc_data.swich_SB = last_SB;
        if(rc_data.swich_SD == 3) rc_data.swich_SD = last_SD;

        if(rc_data.swich_SA != last_SA){
            if(rc_data.swich_SA == 0){
                ctrl_data.chassis_status = CHASSIS_STATE_REST;
            }
            else if(rc_data.swich_SA == 2){
                ctrl_data.chassis_status = CHASSIS_STATE_STANDUP;
                ctrl_data.standup_status = STANDUP_STATE_START;
            }
            last_SA = rc_data.swich_SA;
        }
        last_SB = rc_data.swich_SB;

        uint8_t allow_rc = (rc_data.swich_SB == 2) || (rc_data.swich_SB == 1);
        uint8_t allow_pc = (rc_data.swich_SB == 0) || (rc_data.swich_SB == 1);
        uint8_t pc_fresh = robotCmdIsFresh(PC_CMD_TIMEOUT_MS);

        /* PC 失能/起身/跳跃/回跑：边沿触发；失能与起身不要求已在 COMMON */
        if(allow_pc && pc_fresh){
            if(robot_cmd.control_mode != last_pc_mode){
                if(robot_cmd.control_mode == 1){
                    ctrl_data.chassis_status = CHASSIS_STATE_REST;
                }
                else if(robot_cmd.control_mode == 3){
                    ctrl_data.chassis_status = CHASSIS_STATE_STANDUP;
                    ctrl_data.standup_status = STANDUP_STATE_START;
                }
                else if(robot_cmd.control_mode == 2){
                    if(ctrl_data.chassis_status == CHASSIS_STATE_COMMON){
                        ctrl_data.common_mode = CHASSIS_JUMP;
                        ctrl_data.jump_status = JUMP_STATE_PREPARE;
                    }
                }
                else if(robot_cmd.control_mode == 0){
                    if(ctrl_data.chassis_status == CHASSIS_STATE_COMMON){
                        if(ctrl_data.common_mode == CHASSIS_JUMP &&
                           (ctrl_data.jump_status == JUMP_STATE_NONE ||
                            ctrl_data.jump_status == JUMP_STATE_LANDING)){
                            ctrl_data.common_mode = CHASSIS_RUN;
                            ctrl_data.jump_status = JUMP_STATE_NONE;
                        }
                    }
                }
                last_pc_mode = robot_cmd.control_mode;
            }
        }else{
            last_pc_mode = 0xFF;
        }

        if(ctrl_data.chassis_status == CHASSIS_STATE_COMMON){
            float vx_rc = 0.0f;
            float w_rc = 0.0f;
            float leg_rc = 0.15f;
            float vx_pc = 0.0f;
            float w_pc = 0.0f;
            float leg_pc = 0.15f;
            uint8_t use_pc_leg = 0;

            if(allow_rc){
                vx_rc = rc_data.left_y * bipe_chassis.v_x_max;
                w_rc = -rc_data.left_x * bipe_chassis.w_max;
                if(ctrl_data.common_mode == CHASSIS_RUN){
                    leg_rc = 0.15f + 0.06f * rc_data.right_y;
                    bipe_chassis.leg_phi_left_target = PI / 3.0f * rc_data.left_x;
                    bipe_chassis.leg_phi_right_target = PI / 3.0f * rc_data.left_x;
                }
            }

            if(allow_pc && pc_fresh){
                vx_pc = robot_cmd.vel_x;
                w_pc = robot_cmd.vel_w;
                leg_pc = robot_cmd.leg_length;
                use_pc_leg = 1;
            }

            if(allow_rc && allow_pc){
                ctrl_data.v_x_target = vx_rc + vx_pc;
                ctrl_data.w_target = w_rc + w_pc;
            }else if(allow_rc){
                ctrl_data.v_x_target = vx_rc;
                ctrl_data.w_target = w_rc;
            }else{
                ctrl_data.v_x_target = vx_pc;
                ctrl_data.w_target = w_pc;
            }
            mylimit_float(&ctrl_data.v_x_target, bipe_chassis.v_x_max, -bipe_chassis.v_x_max);
            mylimit_float(&ctrl_data.w_target, bipe_chassis.w_max, -bipe_chassis.w_max);

            if(ctrl_data.common_mode == CHASSIS_RUN){
                float leg = leg_rc;
                if(use_pc_leg){
                    leg = leg_pc;
                }else if(allow_rc){
                    leg = leg_rc;
                }
                bipe_chassis.leg_length_left_target = leg;
                bipe_chassis.leg_length_right_target = leg;
                mylimit_float(&bipe_chassis.leg_length_left_target, 0.20f, 0.10f);
                mylimit_float(&bipe_chassis.leg_length_right_target, 0.20f, 0.10f);
            }

            if(allow_rc){
                if(rc_data.swich_SD != last_SD){
                    if(rc_data.swich_SD == 0){
                        ctrl_data.common_mode = CHASSIS_RUN;
                    }
                    else if(rc_data.swich_SD == 2){
                        ctrl_data.common_mode = CHASSIS_JUMP;
                        ctrl_data.jump_status = JUMP_STATE_PREPARE;
                    }
                    last_SD = rc_data.swich_SD;
                }
            }
        }
    }
    else{
        ctrl_data.chassis_status = CHASSIS_STATE_FAULT;
    }
}

void updateChassisControl(void){
    static uint32_t last_time = 0;
    static uint32_t current_time = 0;
    float dt;

    float leg_motor_left[2] = {0.0f};
    float leg_motor_right[2] = {0.0f};
    float leg_motor_left_velocity[2] = {0.0f};
    float leg_motor_right_velocity[2] = {0.0f};

    leg_motor_left[0] = leg_left.leg_motor[0].para.pos;//�ǶȽ���
    leg_motor_left[1] = leg_left.leg_motor[1].para.pos;
    leg_motor_right[0] = leg_right.leg_motor[0].para.pos;//�ǶȽ���
    leg_motor_right[1] = leg_right.leg_motor[1].para.pos;
    leg_motor_left_velocity[0] = leg_left.leg_motor[0].para.vel;
    leg_motor_left_velocity[1] = leg_left.leg_motor[1].para.vel;
    leg_motor_right_velocity[0] = leg_right.leg_motor[0].para.vel;
    leg_motor_right_velocity[1] = leg_right.leg_motor[1].para.vel;

    VMCDataPrepare(leg_left.vmc_leg_x, leg_motor_left);
    VMCDataPrepare(leg_right.vmc_leg_x, leg_motor_right);
    current_time = xTaskGetTickCount();
    dt = (current_time - last_time) / 1000.0f;
    legPosCalc(leg_left.vmc_leg_x, leg_motor_left_velocity, dt);//�����Ȳ���Ϣ
    legPosCalc(leg_right.vmc_leg_x, leg_motor_right_velocity, dt);
	

    x_left[0] = leg_left.vmc_leg_x->leg_pos.phi - phiTargetCalc(leg_left.vmc_leg_x->leg_pos.leg_length);
    x_left[1] = leg_left.vmc_leg_x->leg_pos.phi_velocity;
	x_left[3] = leg_left.wheel_motor.para.vel * bipe_chassis.wheel_r - ctrl_data.v_x_target;
    x_left[4] = INS.Pitch;
    x_left[5] = INS.Gyro[1];
	
//    x_left[0] = 0.0f;
//    x_left[1] = 0.0f;
//	  x_left[2] = 0.0f;
//	  x_left[3] = 0.0f;
//    x_left[4] = 0.0f;
//    x_left[5] = 0.0f;

    x_right[0] = leg_right.vmc_leg_x->leg_pos.phi - phiTargetCalc(leg_right.vmc_leg_x->leg_pos.leg_length);
    x_right[1] = leg_right.vmc_leg_x->leg_pos.phi_velocity;
    x_right[3] = leg_right.wheel_motor.para.vel * bipe_chassis.wheel_r - ctrl_data.v_x_target;
    x_right[4] = INS.Pitch;
    x_right[5] = INS.Gyro[1];
	
	current_time = xTaskGetTickCount();
    dt = (current_time - last_time) / 1000.0f;
    x_right[2] += (x_right[3] + x_left[3]) / 2.0f * dt;
	x_left[2] = x_right[2];
	
	mylimit_float(&x_right[2],2.0f,-2.0f);
	mylimit_float(&x_left[2],2.0f,-2.0f);
	
//    x_right[0] = 0.0f;
//    x_right[1] = 0.0f;
//	  x_right[2] = 0.0f;
//	  x_right[3] = 0.0f;
//    x_right[4] = 0.0f;
//    x_right[5] = 0.0f;


    //��ؼ��
    float real_leg_torque_left[2] = {0.0f, 0.0f};
    float real_leg_torque_right[2] = {0.0f, 0.0f};
    float real_leg_force_left[2] = {0.0f, 0.0f};
    float real_leg_force_right[2] = {0.0f, 0.0f};
    real_leg_torque_left[0] = leg_left.leg_motor[0].para.tor;
    real_leg_torque_left[1] = leg_left.leg_motor[1].para.tor;
    real_leg_torque_right[0] = leg_right.leg_motor[0].para.tor;
    real_leg_torque_right[1] = leg_right.leg_motor[1].para.tor;
    VMCLegForceCalc(leg_left.vmc_leg_x, real_leg_torque_left, real_leg_force_left);
    VMCLegForceCalc(leg_right.vmc_leg_x, real_leg_torque_right, real_leg_force_right);
    leg_left.leg_force[0] = real_leg_force_left[0];
    leg_left.leg_force[1] = real_leg_force_left[1];
    leg_right.leg_force[0] = real_leg_force_right[0];
    leg_right.leg_force[1] = real_leg_force_right[1];

    if(ctrl_data.chassis_status == CHASSIS_STATE_REST){
        leg_left.leg_motor_torque[0] = 0.0f;
        leg_left.leg_motor_torque[1] = 0.0f;
        leg_left.wheel_motor_torque = 0.0f;
        leg_right.leg_motor_torque[0] = 0.0f;
        leg_right.leg_motor_torque[1] = 0.0f;
        leg_right.wheel_motor_torque = 0.0f;
    }
    else if(ctrl_data.chassis_status == CHASSIS_STATE_STANDUP){//����
        if(ctrl_data.standup_status == STANDUP_STATE_START){
            //��ʱ��дת��  
            ctrl_data.standup_status = STANDUP_STATE_MID;
        }
        else if(ctrl_data.standup_status == STANDUP_STATE_MID){
            bipe_chassis.leg_length_left_target = 0.10f;
            bipe_chassis.leg_length_right_target = 0.10f;
//			bipe_chassis.leg_length_left_target = 0.15f + 0.06f * rc_data.right_y;
//			bipe_chassis.leg_length_right_target = 0.15f + 0.06f * rc_data.right_y;
//			mylimit_float(&bipe_chassis.leg_length_left_target,0.20f,0.10f);
//			mylimit_float(&bipe_chassis.leg_length_right_target,0.20f,0.10f);
			
            PID_Calc(&LegL_Pid, leg_left.vmc_leg_x->leg_pos.leg_length, bipe_chassis.leg_length_left_target);
            PID_Calc(&LegR_Pid, leg_right.vmc_leg_x->leg_pos.leg_length, bipe_chassis.leg_length_right_target);
            leg_left.vmc_force[1] = LegL_Pid.out;
            leg_right.vmc_force[1] = LegR_Pid.out;

            PID_Calc(&PhiL_Pid, leg_left.vmc_leg_x->leg_pos.phi, 0.0f);//���������õ��ǵ��������Ч��������Ҫ�ĳ�˫��
            PID_Calc(&PhiR_Pid, leg_right.vmc_leg_x->leg_pos.phi, 0.0f);
            leg_left.vmc_force[0] = PhiL_Pid.out;
            leg_right.vmc_force[0] = PhiR_Pid.out;

            VMCVirtual2RealCalc(leg_left.vmc_leg_x, leg_left.leg_motor_torque, leg_left.vmc_force);
            VMCVirtual2RealCalc(leg_right.vmc_leg_x, leg_right.leg_motor_torque, leg_right.vmc_force);

            if(leg_left.vmc_leg_x->leg_pos.leg_length < 0.13f && leg_right.vmc_leg_x->leg_pos.leg_length < 0.13f &&
                fabs(leg_left.vmc_leg_x->leg_pos.phi) < 0.1f && fabs(leg_right.vmc_leg_x->leg_pos.phi) < 0.1f
            ){
                ctrl_data.standup_status = STANDUP_STATE_END;
                PID_clear(&LegL_Pid);
                PID_clear(&LegR_Pid);
                PID_clear(&PhiL_Pid);
                PID_clear(&PhiR_Pid);
            }
        }
        else if(ctrl_data.standup_status == STANDUP_STATE_END){
            //վ����������������״̬
            ctrl_data.chassis_status = CHASSIS_STATE_COMMON;
            ctrl_data.common_mode = CHASSIS_RUN;
        }
    }
    else if(ctrl_data.chassis_status == CHASSIS_STATE_COMMON){//������������
        if(ctrl_data.common_mode == CHASSIS_RUN){//����ǰ��
            //��������
            leg_left.wheel_motor_torque = 0.0f;
            leg_left.leg_motor_torque[0] = 0.0f;
            leg_left.leg_motor_torque[1] = 0.0f;
            leg_left.vmc_force[0] = 0.0f;
            leg_left.vmc_force[1] = 0.0f;
            leg_right.wheel_motor_torque = 0.0f;
            leg_right.leg_motor_torque[0] = 0.0f;
            leg_right.leg_motor_torque[1] = 0.0f;
            leg_right.vmc_force[0] = 0.0f;
            leg_right.vmc_force[1] = 0.0f;
			//���԰ڽǻ���
//            PID_Calc(&PhiL_Pid, leg_left.vmc_leg_x->leg_pos.phi, bipe_chassis.leg_phi_left_target);
//            PID_Calc(&PhiR_Pid, leg_right.vmc_leg_x->leg_pos.phi, bipe_chassis.leg_phi_right_target);
//            leg_left.vmc_force[0] = PhiL_Pid.out;
//            leg_right.vmc_force[0] = PhiR_Pid.out;
            //���滷
            PID_Calc(&Tp_Pid, leg_left.vmc_leg_x->leg_pos.phi - leg_right.vmc_leg_x->leg_pos.phi, 0.0f);
            leg_left.vmc_force[0] += Tp_Pid.out;
            leg_right.vmc_force[0] -= Tp_Pid.out;
            //roll��
            PID_Calc(&Roll_Pid, INS.Roll, 0.0f);
            bipe_chassis.leg_length_left_target += Roll_Pid.out;
            bipe_chassis.leg_length_right_target -= Roll_Pid.out;
			
			mylimit_float(&bipe_chassis.leg_length_left_target,0.20f,0.10f);
			mylimit_float(&bipe_chassis.leg_length_right_target,0.20f,0.10f);
            //�ȳ���
            PID_Calc(&LegL_Pid, leg_left.vmc_leg_x->leg_pos.leg_length, bipe_chassis.leg_length_left_target);
            PID_Calc(&LegR_Pid, leg_right.vmc_leg_x->leg_pos.leg_length, bipe_chassis.leg_length_right_target);
            leg_left.vmc_force[1] = LegL_Pid.out + bipe_chassis.leg_force_ref;
            leg_right.vmc_force[1] = LegR_Pid.out + bipe_chassis.leg_force_ref;
            //ת��
			PID_Calc(&Turn_Pid, INS.Gyro[2], ctrl_data.w_target);
            leg_left.wheel_motor_torque -= Turn_Pid.out;
            leg_right.wheel_motor_torque += Turn_Pid.out;
            //LQR
            LQRCalculate(leg_left.vmc_leg_x->leg_pos.leg_length, 0);
            leg_left.wheel_motor_torque += u_left[0];
            leg_left.vmc_force[0] += u_left[1];
            LQRCalculate(leg_right.vmc_leg_x->leg_pos.leg_length, 1);
            leg_right.wheel_motor_torque += u_right[0];
            leg_right.vmc_force[0] += u_right[1];
            //������ת�����
            VMCVirtual2RealCalc(leg_left.vmc_leg_x, leg_left.leg_motor_torque, leg_left.vmc_force);
            VMCVirtual2RealCalc(leg_right.vmc_leg_x, leg_right.leg_motor_torque, leg_right.vmc_force);

            if(leg_left.leg_force[1] < 4.0f && leg_right.leg_force[1] < 4.0f){
                //˫�����
                ctrl_data.common_mode = CHASSIS_JUMP;
                ctrl_data.jump_status = JUMP_STATE_LANDING;
            }
        }
        else if(ctrl_data.common_mode == CHASSIS_JUMP){//��Ծ
            if(ctrl_data.jump_status == JUMP_STATE_PREPARE){//����
                bipe_chassis.leg_length_left_target = 0.10f;
                bipe_chassis.leg_length_right_target = 0.10f;
                PID_Calc(&LegL_Pid, leg_left.vmc_leg_x->leg_pos.leg_length, bipe_chassis.leg_length_left_target);
                PID_Calc(&LegR_Pid, leg_right.vmc_leg_x->leg_pos.leg_length, bipe_chassis.leg_length_right_target);
                leg_left.vmc_force[1] = LegL_Pid.out + bipe_chassis.leg_force_ref;
                leg_right.vmc_force[1] = LegR_Pid.out + bipe_chassis.leg_force_ref;

                PID_Calc(&PhiL_Pid, leg_left.vmc_leg_x->leg_pos.phi, 0.0f);//���������õ��ǵ��������Ч��������Ҫ�ĳ�˫��
                PID_Calc(&PhiR_Pid, leg_right.vmc_leg_x->leg_pos.phi, 0.0f);
                leg_left.vmc_force[0] = PhiL_Pid.out;
                leg_right.vmc_force[0] = PhiR_Pid.out;

                //������ת�����
                VMCVirtual2RealCalc(leg_left.vmc_leg_x, leg_left.leg_motor_torque, leg_left.vmc_force);
                VMCVirtual2RealCalc(leg_right.vmc_leg_x, leg_right.leg_motor_torque, leg_right.vmc_force);

                if(leg_left.vmc_leg_x->leg_pos.leg_length < 0.13f && leg_right.vmc_leg_x->leg_pos.leg_length < 0.13f &&
                    fabs(leg_left.vmc_leg_x->leg_pos.phi) < 0.1f && fabs(leg_right.vmc_leg_x->leg_pos.phi) < 0.1f
                ){
                    ctrl_data.jump_status = JUMP_STATE_TAKEOFF;
                    PID_clear(&LegL_Pid);
                    PID_clear(&LegR_Pid);
                    PID_clear(&PhiL_Pid);
                    PID_clear(&PhiR_Pid);
                }
            }
            else if(ctrl_data.jump_status == JUMP_STATE_TAKEOFF)//����
            {
                leg_left.vmc_force[1] = 100.0f;
                leg_right.vmc_force[1] = 100.0f;

                PID_Calc(&PhiL_Pid, leg_left.vmc_leg_x->leg_pos.phi, 0.0f);//���������õ��ǵ��������Ч��������Ҫ�ĳ�˫��
                PID_Calc(&PhiR_Pid, leg_right.vmc_leg_x->leg_pos.phi, 0.0f);
                leg_left.vmc_force[0] = PhiL_Pid.out;
                leg_right.vmc_force[0] = PhiR_Pid.out;

                //������ת�����
                VMCVirtual2RealCalc(leg_left.vmc_leg_x, leg_left.leg_motor_torque, leg_left.vmc_force);
                VMCVirtual2RealCalc(leg_right.vmc_leg_x, leg_right.leg_motor_torque, leg_right.vmc_force);

                if(leg_left.vmc_leg_x->leg_pos.leg_length > 0.20f && leg_right.vmc_leg_x->leg_pos.leg_length > 0.20f){
                    ctrl_data.jump_status = JUMP_STATE_FLIGHT;
                    PID_clear(&PhiL_Pid);
                    PID_clear(&PhiR_Pid);
                }
            }
            else if(ctrl_data.jump_status == JUMP_STATE_FLIGHT)//��ߴ�����
            {
                bipe_chassis.leg_length_left_target = 0.10f;
                bipe_chassis.leg_length_right_target = 0.10f;
                PID_Calc(&LegL_Pid, leg_left.vmc_leg_x->leg_pos.leg_length, bipe_chassis.leg_length_left_target);
                PID_Calc(&LegR_Pid, leg_right.vmc_leg_x->leg_pos.leg_length, bipe_chassis.leg_length_right_target);
                leg_left.vmc_force[1] = LegL_Pid.out - bipe_chassis.leg_force_ref;//���ȱ为��
                leg_right.vmc_force[1] = LegR_Pid.out - bipe_chassis.leg_force_ref;

                PID_Calc(&PhiL_Pid, leg_left.vmc_leg_x->leg_pos.phi, 0.0f);//���������õ��ǵ��������Ч��������Ҫ�ĳ�˫��
                PID_Calc(&PhiR_Pid, leg_right.vmc_leg_x->leg_pos.phi, 0.0f);
                leg_left.vmc_force[0] = PhiL_Pid.out;
                leg_right.vmc_force[0] = PhiR_Pid.out;

                //������ת�����
                VMCVirtual2RealCalc(leg_left.vmc_leg_x, leg_left.leg_motor_torque, leg_left.vmc_force);
                VMCVirtual2RealCalc(leg_right.vmc_leg_x, leg_right.leg_motor_torque, leg_right.vmc_force);

                if(leg_left.vmc_leg_x->leg_pos.leg_length < 0.13f && leg_right.vmc_leg_x->leg_pos.leg_length < 0.13f &&
                    fabs(leg_left.vmc_leg_x->leg_pos.phi) < 0.1f && fabs(leg_right.vmc_leg_x->leg_pos.phi) < 0.1f
                ){
                    ctrl_data.jump_status = JUMP_STATE_LANDING;
                    PID_clear(&LegL_Pid);
                    PID_clear(&LegR_Pid);
                    PID_clear(&PhiL_Pid);
                    PID_clear(&PhiR_Pid);
                }
            }
            else if(ctrl_data.jump_status == JUMP_STATE_LANDING){//�������
                bipe_chassis.leg_length_left_target = 0.20f;
                bipe_chassis.leg_length_right_target = 0.20f;
                PID_Calc(&LegL_Pid, leg_left.vmc_leg_x->leg_pos.leg_length, bipe_chassis.leg_length_left_target);
                PID_Calc(&LegR_Pid, leg_right.vmc_leg_x->leg_pos.leg_length, bipe_chassis.leg_length_right_target);
                leg_left.vmc_force[1] = LegL_Pid.out + bipe_chassis.leg_force_ref;
                leg_right.vmc_force[1] = LegR_Pid.out + bipe_chassis.leg_force_ref;

                PID_Calc(&PhiL_Pid, leg_left.vmc_leg_x->leg_pos.phi, 0.0f);//���������õ��ǵ��������Ч��������Ҫ�ĳ�˫��
                PID_Calc(&PhiR_Pid, leg_right.vmc_leg_x->leg_pos.phi, 0.0f);
                leg_left.vmc_force[0] = PhiL_Pid.out;
                leg_right.vmc_force[0] = PhiR_Pid.out;

                //������ת�����
                VMCVirtual2RealCalc(leg_left.vmc_leg_x, leg_left.leg_motor_torque, leg_left.vmc_force);
                VMCVirtual2RealCalc(leg_right.vmc_leg_x, leg_right.leg_motor_torque, leg_right.vmc_force);

                if(leg_left.leg_force[1] > 15.0f && leg_right.leg_force[1] > 15.0f){
                    //˫���ŵ�
                    ctrl_data.common_mode = CHASSIS_RUN;
                    ctrl_data.jump_status = JUMP_STATE_NONE;
                }
            }
            leg_left.wheel_motor_torque = 0.0f;
            leg_right.wheel_motor_torque = 0.0f;
        }
    }
    else{
        //����״̬��ȫ��ʧ��
        leg_left.leg_motor_torque[0] = 0.0f;
        leg_left.leg_motor_torque[1] = 0.0f;
        leg_left.wheel_motor_torque = 0.0f;
        leg_right.leg_motor_torque[0] = 0.0f;
        leg_right.leg_motor_torque[1] = 0.0f;
        leg_right.wheel_motor_torque = 0.0f;
    }

    leg_right.leg_motor_torque[0] = -leg_right.leg_motor_torque[0];
    leg_right.leg_motor_torque[1] = -leg_right.leg_motor_torque[1];
    leg_right.wheel_motor_torque = -leg_right.wheel_motor_torque;
	
	mylimit_float(&leg_left.leg_motor_torque[0],10.0f,-10.0f);
	mylimit_float(&leg_left.leg_motor_torque[1],10.0f,-10.0f);
	mylimit_float(&leg_left.wheel_motor_torque,2.5f,-2.5f);
	mylimit_float(&leg_right.leg_motor_torque[0],10.0f,-10.0f);
	mylimit_float(&leg_right.leg_motor_torque[1],10.0f,-10.0f);
	mylimit_float(&leg_right.wheel_motor_torque,2.5f,-2.5f);

	mit_ctrl(&hfdcan1,0x03, 0.0f, 0.0f,0.0f, 0.0f,leg_left.leg_motor_torque[0]);
	osDelay(1);
	mit_ctrl(&hfdcan1,0x04, 0.0f, 0.0f,0.0f, 0.0f,leg_left.leg_motor_torque[1]);
	osDelay(1);
	mit_ctrl2(&hfdcan1,0x01, 0.0f, 0.0f,0.0f, 0.0f,leg_left.wheel_motor_torque);
	osDelay(1);
	mit_ctrl(&hfdcan1,0x05, 0.0f, 0.0f,0.0f, 0.0f,leg_right.leg_motor_torque[0]);
	osDelay(1);
	mit_ctrl(&hfdcan1,0x06, 0.0f, 0.0f,0.0f, 0.0f,leg_right.leg_motor_torque[1]);
	osDelay(1);
	mit_ctrl2(&hfdcan1,0x02, 0.0f, 0.0f,0.0f, 0.0f,leg_right.wheel_motor_torque);
	osDelay(1);

    last_time = current_time;
}

void chassis_task(void){
	initChassisTask();
	while(1){
		loadManualControl();
		updateChassisControl();
	}
}
