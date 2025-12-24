#include "remote_control.h"

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
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
        }
    }
    return crc;
}

void crsf_decode_channels(const uint8_t *payload, uint16_t *ch)
{
    uint32_t bitbuf = 0;
    uint8_t bitcnt = 0;
    uint8_t idx = 0;

    for (int i = 0; i < 22; i++) {
        bitbuf |= ((uint32_t)payload[i]) << bitcnt;
        bitcnt += 8;

        while (bitcnt >= 11 && idx < 16) {
            ch[idx++] = bitbuf & 0x7FF;
            bitbuf >>= 11;
            bitcnt -= 11;
        }
    }
}

uint8_t remoteControlParse(crsf_data_t* crsf_data, rc_data_t* rc_data, uint8_t* buffer, uint8_t buffer_length){
    if(buffer[0] != 0xC8){
        return 0x01; // Invalid sync byte or insufficient length
    }

    uint8_t length = buffer[1];

    if(length + 4 != buffer_length){
        return 0x02; // Insufficient buffer length
    }

    uint8_t crc_calculated = crsf_crc8(buffer, buffer_length - 1);
    if(crc_calculated != buffer[buffer_length - 1]){
        return 0x03; // CRC mismatch
    }

    crsf_data->sync_byte = buffer[0];
    crsf_data->length = buffer[1];
    crsf_data->type = buffer[2];
    memcpy(crsf_data->payload, &buffer[3], 22);
    crsf_decode_channels(crsf_data->payload, crsf_data->ch);
    crsf_data->crc = buffer[buffer_length - 1];

    rc_data->swich_SA = (crsf_data->ch[4] - 1024) / 512;
    rc_data->swich_SB = (crsf_data->ch[5] - 1024) / 512;
    rc_data->swich_SC = (crsf_data->ch[6] - 1024) / 512;
    rc_data->swich_SD = (crsf_data->ch[7] - 1024) / 512;
    rc_data->left_x = (float)(crsf_data->ch[0] - 1024) / 1024.0f;
    rc_data->left_y = (float)(crsf_data->ch[1] - 1024) / 1024.0f;
    rc_data->right_x = (float)(crsf_data->ch[2] - 1024) / 1024.0f;
    rc_data->right_y = (float)(crsf_data->ch[3] - 1024) / 1024.0f;

    return 0x00; // Success
}

void dmaToFrameBuffer(uint8_t* dma_buffer, uint8_t* frame_buffer, uint8_t* frame_length){
    static uint8_t last_index = 0;
    uint8_t data_size = 0;
    uint8_t current_index = UART1_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    if (current_index > last_index) {
        data_size = current_index - last_index;
        memcpy(frame_buffer, dma_buffer + last_index, current_index - last_index);
    }
    else {
        memcpy(frame_buffer, dma_buffer + last_index, UART1_DMA_RX_BUFFER_SIZE - last_index);
        data_size += UART1_DMA_RX_BUFFER_SIZE - last_index;
        memcpy(frame_buffer + data_size, dma_buffer, current_index);
        data_size += current_index;
    }
    last_index = current_index;
    *frame_length = data_size;

    frame_data.last_time = xTaskGetTickCount();
}

void initRemoteControl(){
    frame_data.data = uart1_frame_buffer;
    frame_data.length = 0;

    HAL_UART_Receive_DMA(&huart1, uart1_dma_rx_buffer, UART1_DMA_RX_BUFFER_SIZE);
    //使能 IDLE 空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

void remoteControlTask(){
    initRemoteControl();

    while(1){
        uint8_t result = remoteControlParse(&crsf_data, &rc_data, uart1_frame_buffer, frame_data.length);
        if(result == 0x00){
            remoteControlParse(&crsf_data, &rc_data, uart1_frame_buffer, frame_data.length);
        }
        
        if(xTaskGetTickCount() - frame_data.last_time > 1000){
            rc_data.state = REMOTE_OFF;//遥控掉线
        }
        else{
            rc_data.state = REMOTE_ON;//遥控在线
        }

        os_delay(10);
    }
}