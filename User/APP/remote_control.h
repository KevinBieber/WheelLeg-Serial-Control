#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "usart.h"
#include "main.h"
#define UART1_DMA_RX_BUFFER_SIZE  64
#define UART1_FRAME_BUFFER_SIZE  64

typedef enum{
    REMOTE_OFF=0,
    REMOTE_ON=1,
}remote_state_t;

typedef struct{
    uint8_t* data;
    uint8_t length;
    uint32_t last_time;
}frame_data_t;

typedef struct {
    uint8_t sync_byte;
    uint8_t length;
    uint8_t type;
    uint8_t payload[22];
    uint16_t ch[16];
    uint8_t crc;
}crsf_data_t;

typedef struct{
    remote_state_t state;
    uint8_t swich_SA;
    uint8_t swich_SB;
    uint8_t swich_SC;
    uint8_t swich_SD;
    float left_x;
    float left_y;
    float right_x;
    float right_y;
}rc_data_t;

uint8_t crsf_crc8(const uint8_t *data, uint8_t len);
void crsf_decode_channels(const uint8_t *payload, uint16_t *ch);
uint8_t remoteControlParse(crsf_data_t* crsf_data, rc_data_t* rc_data, uint8_t* buffer, uint8_t buffer_length);
void dmaToFrameBuffer(void);
void initRemoteControl(void);
void remoteControlTask(void);

#endif
