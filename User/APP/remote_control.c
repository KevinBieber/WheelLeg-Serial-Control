#include "remote_control.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>

extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t uart1_dma_rx_buffer[UART1_DMA_RX_BUFFER_SIZE];
uint8_t uart1_frame_buffer[UART1_FRAME_BUFFER_SIZE];

frame_data_t frame_data;
crsf_data_t crsf_data;
rc_data_t rc_data;

uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc = crc << 1;
        }
    }
    return crc;
}

void crsf_decode_channels(const uint8_t *payload, uint16_t *ch)
{
    uint64_t bitBuffer = 0;
    uint8_t bitsInBuffer = 0;
    int byteIndex = 0;

    for (int i = 0; i < 16; i++) {
        while (bitsInBuffer < 11) {
            // 从payload中读取字节
            bitBuffer |= ((uint64_t)payload[byteIndex++] << bitsInBuffer);
            bitsInBuffer += 8;
        }
        ch[i] = bitBuffer & 0x07FF; // 取低11位作为通道值
        bitBuffer >>= 11;
        bitsInBuffer -= 11;
    }
}

uint8_t getSwichBit(uint16_t ch_data){
	if(ch_data == 191)return 0x00;
	else if(ch_data == 997)return 0x01;
	else if(ch_data == 1792)return 0x02;
	return 0x03;
}

void my_limit(float* a, float limit){
	if(*a > limit)*a = limit;
	else if(*a < -limit)*a = -limit;
}

uint8_t remoteControlParse(crsf_data_t* crsf_data, rc_data_t* rc_data, uint8_t* buffer, uint8_t buffer_length){
    if(buffer[0] != 0xC8){
        return 0x01; // Invalid sync byte or insufficient length
    }

    uint8_t length = buffer[1];

    if(length + 2 != buffer_length){
        return 0x02; // Insufficient buffer length
    }

    uint8_t crc_calculated = crsf_crc8(&buffer[2], buffer_length - 3);
    if(crc_calculated != buffer[buffer_length - 1]){
        return 0x03; // CRC mismatch
    }

    crsf_data->sync_byte = buffer[0];
    crsf_data->length = buffer[1];
    crsf_data->type = buffer[2];
    memcpy(crsf_data->payload, &buffer[3], 22);
    crsf_decode_channels(crsf_data->payload, crsf_data->ch);
    crsf_data->crc = buffer[buffer_length - 1];
	
    rc_data->swich_SA = getSwichBit(crsf_data->ch[4]);
    rc_data->swich_SB = getSwichBit(crsf_data->ch[5]);
    rc_data->swich_SC = getSwichBit(crsf_data->ch[6]);
    rc_data->swich_SD = getSwichBit(crsf_data->ch[7]);
    rc_data->left_x = ((float)(crsf_data->ch[3] - 172) / 1639.0f - 0.5f) * 2.0f;
    rc_data->left_y = ((float)(crsf_data->ch[1] - 172) / 1639.0f - 0.5f) * 2.0f;
    rc_data->right_x = ((float)(crsf_data->ch[0] - 172) / 1639.0f - 0.5f) * 2.0f;
    rc_data->right_y = ((float)(crsf_data->ch[2] - 172) / 1639.0f - 0.5f) * 2.0f;
	
	if(fabs(rc_data->left_x) < 0.005)rc_data->left_x = 0.0f;
	if(fabs(rc_data->left_y) < 0.005)rc_data->left_y = 0.0f;
	if(fabs(rc_data->right_x) < 0.005)rc_data->right_x = 0.0f;
	if(fabs(rc_data->right_y) < 0.005)rc_data->right_y = 0.0f;
	
	my_limit(&rc_data->left_x, 1.0f);
	my_limit(&rc_data->left_y, 1.0f);
	my_limit(&rc_data->right_x, 1.0f);
	my_limit(&rc_data->right_y, 1.0f);

    return 0x00; // Success
}

void dmaToFrameBuffer(void){
    static uint8_t last_index = 0;
    uint8_t data_size = 0;
    uint8_t current_index = UART1_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    if (current_index > last_index) {
        data_size = current_index - last_index;
        memcpy(frame_data.data, uart1_dma_rx_buffer + last_index, current_index - last_index);
    }
    else {
        memcpy(frame_data.data, uart1_dma_rx_buffer + last_index, UART1_DMA_RX_BUFFER_SIZE - last_index);
        data_size += UART1_DMA_RX_BUFFER_SIZE - last_index;
        memcpy(frame_data.data + data_size, uart1_dma_rx_buffer, current_index);
        data_size += current_index;
    }
    last_index = current_index;
    frame_data.length = data_size;

    frame_data.last_time = xTaskGetTickCount();
}

void initRemoteControl(void){
    frame_data.data = uart1_frame_buffer;
    frame_data.length = 0;

    HAL_UART_Receive_DMA(&huart1, uart1_dma_rx_buffer, UART1_DMA_RX_BUFFER_SIZE);
    //使能 IDLE 空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

void remoteControlTask(void){
    initRemoteControl();

    while(1){
        uint8_t result = remoteControlParse(&crsf_data, &rc_data, uart1_frame_buffer, frame_data.length);
        if(result == 0x00){
            remoteControlParse(&crsf_data, &rc_data, uart1_frame_buffer, frame_data.length);
        }
        
		uint32_t time = xTaskGetTickCount();
        if(xTaskGetTickCount() - frame_data.last_time > 1000){
            rc_data.state = REMOTE_OFF;//遥控掉线
        }
        else{
            rc_data.state = REMOTE_ON;//遥控在线
        }

        osDelay(10);
    }
}
