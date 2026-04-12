#ifndef USB_RECEIVE_TASK_H
#define USB_RECEIVE_TASK_H

#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"

// 必须强制字节对齐，否则 float 解析会出错
typedef struct __attribute__((packed)) {
    float vel_x;
    float vel_y;
    float vel_w;
    float leg_length;
    uint8_t control_mode;
} RobotCommand_t;

typedef struct {
    uint8_t msg_type;
    uint8_t app_id;
    uint16_t length;
    uint8_t data[20]; // 根据实际需求调整大小
} UsbPacket_t;

void usbReceiveTaskInit(void);
uint8_t usbReceive(void);
void usb_receive_task(void);
uint8_t checkCrc16(uint8_t *data, uint16_t length, uint16_t expected_crc);
void getRobotCommand(void);

#endif
