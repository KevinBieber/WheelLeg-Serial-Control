/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制右半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can1总线上
	*						 从底盘上往下看，右上角的DM4310发送id为6、接收id为3，
	*						 右下角的DM4310发送id为8、接收id为4，
	*						 右边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "chassisR_task.h"
#include "fdcan.h"
#include "cmsis_os.h"

float LQR_K_R[12]={

  -3.3259  , -0.2272,   -1.2959,   -1.2122 ,   3.3133,    0.3249,
    2.5219 ,   0.2142 ,   1.8118 ,   1.6073 ,   8.2857 ,   0.4180

};

//三次多项式拟合系数
float Poly_Coefficient[12][4]={{-213.6885, 153.3306, -50.978, -0.13318},
{-1.1412, 1.2471, -3.633, 0.056666},
{-82.3054, 49.8361, -10.6676, -0.73082},
{-70.3514, 43.3124, -10.1995, -0.64679},
{-246.3632, 173.9108, -47.6573, 6.1294},
{-13.1949, 10.2265, -3.1718, 0.52012},
{114.4332, -51.7589, 2.8343, 2.599},
{14.4172, -8.5621, 1.6232, 0.13359},
{-154.0047, 107.3901, -28.8305, 3.5029},
{-128.9122, 90.0203, -24.2995, 3.035},
{577.6103, -351.5575, 75.9638, 4.2419},
{46.4618, -29.0229, 6.5446, 0.061617}
};

vmc_leg_t right;

extern INS_t INS;
extern vmc_leg_t left;
																
chassis_t chassis_move;
float jump_time;
extern float jump_time2;

PidTypeDef LegR_Pid;//右腿的腿长pd	

PidTypeDef Tp_Pid;//防劈叉补偿pd
PidTypeDef Turn_Pid;//转向pd
PidTypeDef Roll_Pid;//横滚角补偿pd

uint32_t CHASSR_TIME=1;	

void ChassisR_task(void)
{
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}

  ChassisR_init(&chassis_move,&right,&LegR_Pid);//初始化右边两个关节电机和右边轮毂电机的id和控制模式、初始化腿部
  Pensation_init(&Roll_Pid,&Tp_Pid,&Turn_Pid);//补偿pid初始化
	
	while(1)
	{	
		chassisR_feedback_update(&chassis_move,&right,&INS);//更新数据
		
	  chassisR_control_loop(&chassis_move,&right,&INS,LQR_K_R,&LegR_Pid);//控制计算
   
		if(chassis_move.start_flag==1)	
		{
			mit_ctrl(&hfdcan1,0x08, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[1]);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1,0x06, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[0]);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			mit_ctrl2(&hfdcan1,0x01, 0.0f, 0.0f,0.0f, 0.0f,chassis_move.wheel_motor[0].wheel_T);//右边轮毂电机
			osDelay(CHASSR_TIME);
		}
		else if(chassis_move.start_flag==0)	
		{
			mit_ctrl(&hfdcan1,0x08, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1,0x06, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			mit_ctrl2(&hfdcan1,0x01, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//右边轮毂电机	
			osDelay(CHASSR_TIME);
		}
	
	}
}

void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legr)
{
  const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};

	joint_motor_init(&chassis->joint_motor[0],6,MIT_MODE);//发送id为6
	joint_motor_init(&chassis->joint_motor[1],8,MIT_MODE);//发送id为8
	
	wheel_motor_init(&chassis->wheel_motor[0],1,MIT_MODE);//发送id为1
	
	VMC_init(vmc);//给杆长赋值
	
	PID_init(legr, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid

	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);
	  osDelay(1);
	}

	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan1,chassis->wheel_motor[0].para.id,chassis->wheel_motor[0].mode);//右边轮毂电机
	  osDelay(1);
	}
}

void Pensation_init(PidTypeDef *roll,PidTypeDef *Tp,PidTypeDef *turn)
{//补偿pid初始化：横滚角补偿、防劈叉补偿、偏航角补偿
  const static float roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI,ROLL_PID_KD};
	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	
	PID_init(roll, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);

}

void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
  vmc->phi1=pi/2.0f+chassis->joint_motor[0].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[1].para.pos;
		
	chassis->myPithR=ins->Pitch;
	chassis->myPithGyroR=ins->Gyro[1];
	
	chassis->total_yaw=ins->YawTotalAngle;
	chassis->roll=ins->Roll;
	chassis->theta_err=0.0f-(vmc->theta+left.theta);
	
	if(ins->Pitch<(3.1415926f/6.0f)&&ins->Pitch>(-3.1415926f/6.0f))
	{//根据pitch角度判断倒地自起是否完成
		chassis->recover_flag=0;
	}
}

uint8_t right_flag=0;
extern uint8_t left_flag;
//float mg=21.0f;
float mg=18.0f;
void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,PidTypeDef *leg)
{
	VMC_calc_1_right(vmcr,ins,((float)CHASSR_TIME)*3.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是3*0.001秒
	
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0 );	
	}
		
	//chassis->turn_T=PID_Calc(&Turn_Pid, chassis->total_yaw, chassis->turn_set);//yaw轴pid计算
  chassis->turn_T=Turn_Pid.Kp*(chassis->turn_set-chassis->total_yaw)-Turn_Pid.Kd*ins->Gyro[2];//这样计算更稳一点

	chassis->roll_f0=Roll_Pid.Kp*(chassis->roll_set-chassis->roll)-Roll_Pid.Kd*ins->Gyro[0];
  
	mySaturate(&chassis->roll_f0,-Roll_Pid.max_out,Roll_Pid.max_out);
	
	chassis->leg_tp=PID_Calc(&Tp_Pid, chassis->theta_err,0.0f);//防劈叉pid计算
	
	chassis->wheel_motor[0].wheel_T=(LQR_K[0]*(vmcr->theta-0.0f)
																	+LQR_K[1]*(vmcr->d_theta-0.0f)
																	+LQR_K[2]*(chassis->x_filter-chassis->x_set)
																	+LQR_K[3]*(chassis->v_filter-0.4f*chassis->v_set)
																	+LQR_K[4]*(chassis->myPithR-0.04f-chassis->phi_set)
																	+LQR_K[5]*(chassis->myPithGyroR-0.0f));
	
	//右边髋关节输出力矩				
	 vmcr->Tp=(LQR_K[6]*(vmcr->theta-0.0f)
					+LQR_K[7]*(vmcr->d_theta-0.0f)
					+LQR_K[8]*(chassis->x_filter-chassis->x_set)
				  +LQR_K[9]*(chassis->v_filter-0.4f*chassis->v_set)
					+LQR_K[10]*(chassis->myPithR-0.04f-chassis->phi_set)
					+LQR_K[11]*(chassis->myPithGyroR-0.0f));

	vmcr->Tp=vmcr->Tp+chassis->leg_tp;//髋关节输出力矩
	
	chassis->wheel_motor[0].wheel_T=chassis->wheel_motor[0].wheel_T-chassis->turn_T;	//轮毂电机输出力矩

	mySaturate(&chassis->wheel_motor[0].wheel_T,-2.0f,2.0f);	
	 
	if(chassis->jump_flag==1||chassis->jump_flag==2||chassis->jump_flag==3)
	{
    if(chassis->jump_flag==1)
		{//压缩阶段
		 vmcr->F0=mg/arm_cos_f32(vmcr->theta)+PID_Calc(leg,vmcr->L0,0.08f);//前馈+pd

		 if(vmcr->L0<0.10f)
		 {
		  jump_time++;
		 }
		 if(jump_time>=10&&jump_time2>=10)
		 {  
			 jump_time=0;
			 jump_time2=0;
			 chassis->jump_flag=2;//压缩完毕进入上升加速阶段
			 chassis->jump_flag2=2;//压缩完毕进入上升加速阶段
		 }			 
		}
		else if(chassis->jump_flag==2)
		{//上升加速阶段			
			 vmcr->F0=mg/arm_cos_f32(vmcr->theta)+PID_Calc(leg,vmcr->L0,0.40f);//前馈+pd
			
			 if(vmcr->L0>0.18f)
			 {
				jump_time++;
			 }
			 if(jump_time>=2&&jump_time2>=2)
			 {  
				 jump_time=0;
				 jump_time2=0;
				 chassis->jump_flag=3;//上升完毕进入缩腿阶段
				 chassis->jump_flag2=3;
			 }	 
		}
		else if(chassis->jump_flag==3)
		{//缩腿阶段
			vmcr->F0=PID_Calc(leg,vmcr->L0,0.10f);//pd
			chassis->theta_set=0.0f;
		  if(vmcr->L0<0.15f)
		  {
			 jump_time++;
		  }
		  if(jump_time>=3&&jump_time2>=3)
		  { 
			 jump_time=0;
			 jump_time2=0;
			 chassis->leg_set=0.10f;
			 chassis->last_leg_set=0.10f;
			 chassis->jump_flag=0;//缩腿完毕
		   chassis->jump_flag2=0;			
		  }
		}
	}	
	else
	{
		vmcr->F0=mg/arm_cos_f32(vmcr->theta)+PID_Calc(leg,vmcr->L0,chassis->leg_set);//前馈+pd
	}
		
   right_flag=ground_detectionR(vmcr,ins);//右腿离地检测
	 
	 if(chassis->recover_flag==0)		
	 {//倒地自起不需要检测是否离地	 
		if((right_flag==1&&left_flag==1&&vmcr->leg_flag==0&&chassis->jump_flag!=1&&chassis->jump_flag2!=1&&chassis->jump_flag!=2&&chassis->jump_flag2!=2)
			||chassis->jump_flag==3)
		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
			//排除跳跃的压缩阶段、上升阶段、跳跃的缩腿阶段
				chassis->wheel_motor[0].wheel_T=0.0f;
				vmcr->Tp=LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f);

				chassis->x_filter=0.0f;
				chassis->x_set=chassis->x_filter;
				vmcr->Tp=vmcr->Tp+chassis->leg_tp;			 
		}
		else
		{//没有离地
			vmcr->leg_flag=0;//置为0
							
			if(chassis->jump_flag==0)
			{//不跳跃的时候需要roll轴补偿						
			 vmcr->F0=vmcr->F0+chassis->roll_f0;//roll轴补偿取反然后加上去    			
			}
		}
	 }
	 else if(chassis->recover_flag==1)
	 {
		 vmcr->Tp=0.0f;
		 vmcr->F0=0.0f;
	 }

	mySaturate(&vmcr->F0,-100.0f,100.0f);//限幅 

	VMC_calc_2(vmcr);//计算期望的关节输出力矩

	if(chassis->jump_flag==1||chassis->jump_flag==2||chassis->jump_flag==3)
	{//跳跃的时候需要更大扭矩
		mySaturate(&vmcr->torque_set[1],-6.0f,6.0f);	
		mySaturate(&vmcr->torque_set[0],-6.0f,6.0f);	
	}	
	else
	{//不跳跃的时候最大为额定扭矩
    mySaturate(&vmcr->torque_set[1],-3.0f,3.0f);	
		mySaturate(&vmcr->torque_set[0],-3.0f,3.0f);	
	}	
}

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}





