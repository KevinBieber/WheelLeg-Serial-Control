#include "vofa_task.h"
#include <stdio.h>
#include "cmsis_os.h"
#include "arm_math.h"
#include "chassis_task.h"

extern PidTypeDef LegR_Pid;
extern chassis_leg_t leg_left;
extern chassis_t bipe_chassis;
extern rc_data_t rc_data;

#define MAX_BUFFER_SIZE 128
uint8_t send_flag = 0;
uint8_t send_buf[MAX_BUFFER_SIZE];
uint16_t cnt = 0;

float w = 6.28f;

void vofaTaskInit(void){
	//todo
}

void sendDataAppend(float send_data){
	uint8_t data[4];
	memcpy(data, (uint8_t*)&send_data, 4);
	memcpy(send_buf + cnt, data, 4);
	cnt += 4;
}

void vofaSend(void){
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x80;
	send_buf[cnt++] = 0x7F;
	
	CDC_Transmit_HS(send_buf, cnt);
	cnt = 0;
}

void vofa_task(void){
	vofaTaskInit();
	while(1){
		//²âÊÔÓÃ
//		static float t = 0.0f;
//		t += 0.002f;
//		if(t > 50)t = 0;
//		sendDataAppend(arm_sin_f32(w * t));
		
		sendDataAppend(rc_data.right_x);
		sendDataAppend(rc_data.right_y);
		sendDataAppend(rc_data.left_x);
		sendDataAppend(rc_data.left_y);
		vofaSend();
		
		osDelay(2);
	}
}
