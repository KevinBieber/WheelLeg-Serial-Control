#include "usb_receive_task.h"
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"

#define USB_RECEIVE_BUF_SIZE 256 // usb接收缓冲区
#define USB_DATA_BUF_SIZE 128 // 反转义后的数据缓冲区

extern USBD_HandleTypeDef hUsbDeviceHS;

const uint16_t crc16_table[256] = { // crc16_table 用于快速计算 CRC16 校验码
            0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
            0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
            0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
            0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
            0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
            0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
            0x1600, 0xD6C1, 0xD781, 0x1740, 0xD501, 0x15C0, 0x1480, 0xD441,
            0xD201, 0x22C0, 0x2380, 0xD341, 0x2100, 0xD1C1, 0xD081, 0x2040,
            0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
            0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
            0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
            0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
            0x2E00, 0xEEC1, 0xEF81, 0x2F40, 0xED01, 0x2DC0, 0x2C80, 0xEC41,
            0xE601, 0x26C0, 0x2780, 0xE741, 0x2500, 0xE5C1, 0xE481, 0x2440,
            0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
            0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
            0x3800, 0xF8C1, 0xF981, 0x3940, 0xFB01, 0x3BC0, 0x3A80, 0xFA41,
            0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
            0x7600, 0xB6C1, 0xB781, 0x7740, 0xB501, 0x75C0, 0x7480, 0xB441,
            0xB201, 0x72C0, 0x7380, 0xB341, 0x7100, 0xB1C1, 0xB081, 0x7040,
            0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
            0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
            0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
            0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
            0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
            0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
            0x4600, 0x86C1, 0x8781, 0x4740, 0x8501, 0x45C0, 0x4480, 0x8441,
            0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
            0xC001, 0x00C0, 0x0180, 0xC141, 0x0300, 0xC3C1, 0xC281, 0x0240,
            0x0600, 0xC6C1, 0xC781, 0x0740, 0xC501, 0x05C0, 0x0480, 0xC441,
            0x0C00, 0xCCC1, 0xCD81, 0x0D40, 0xCF01, 0x0FC0, 0x0E80, 0xCE41,
            0xCA01, 0x0AC0, 0x0B80, 0xCB41, 0x0900, 0xC9C1, 0xC881, 0x0840
};

RobotCommand_t robot_cmd; // 存放解析后的指令
UsbPacket_t usb_packet; // 存放解析后的USB包信息

uint8_t usb_receive_buf[USB_RECEIVE_BUF_SIZE]; // usb接收缓冲区
uint8_t usb_buf[USB_DATA_BUF_SIZE]; // 反转义后的数据缓冲区

void usbReceiveTaskInit(void){
    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, usb_receive_buf);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);
}

/**
 * @brief 极简反转义：直接将 src 还原到 dest
 * @return 还原后的实际长度
 */
uint16_t unescapeSimple(uint8_t* src, uint8_t* dest) {
    uint16_t j = 0;
    for (uint16_t i = 0; i < USB_RECEIVE_BUF_SIZE; i++) {
		if(src[i] == 0xFE && i + 1 < USB_RECEIVE_BUF_SIZE) { // 发现转义符
            i++; // 跳过 0xFE
            if(src[i] == 0x7D) {
                dest[j++] = 0xFD; // 还原为 0xFD
            }
            else if(src[i] == 0x78) {
                dest[j++] = 0xF8; // 还原为 0xF8
            }
            else if(src[i] == 0x7E) {
                dest[j++] = 0xFE; // 还原为 0xFE
			}
		}
		else {
			dest[j++] = src[i]; // 普通数据直接复制
		}
		
		if(src[i] == 0xF8) {
			break;
		}
    }
    return j; // 返回还原后的长度
}

/**
 * @brief 校验接收到的数据包
 * @param data: 待校验的数据缓冲区 (通常从 msg_type 开始)
 * @param length: 待校验的数据长度 (不包含 CRC 字节本身)
 * @param expected_crc: 从串口包里解析出来的那个 2 字节 CRC 值
 * @return uint8_t: 1 表示校验通过，0 表示失败
 */
uint8_t checkCrc16(uint8_t *data, uint16_t length, uint16_t expected_crc) {
    uint16_t crc_accum = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        uint8_t index = (crc_accum ^ data[i]) & 0xFF;
        crc_accum = (crc_accum >> 8) ^ crc16_table[index];
    }

    return (crc_accum == expected_crc);
}

uint8_t usbReceive(void){
    uint16_t buf_length = unescapeSimple(usb_receive_buf, usb_buf); // 帧长度
    if(buf_length - 8 != (usb_buf[3] << 8 | usb_buf[4])) {
        return 0x01; // 长度不匹配，丢弃
    }
    if(usb_buf[2] != 0x02) {
        return 0x02; // app不匹配，丢弃
    }
    usb_packet.length = buf_length - 8;
    usb_packet.msg_type = usb_buf[1];
    usb_packet.app_id = usb_buf[2];
    uint16_t crc = (usb_buf[buf_length - 1 - 2] << 8) | usb_buf[buf_length - 1 - 1];
    if(!checkCrc16(usb_buf + 1,buf_length - 4, crc)) {
        return 0x03; // CRC校验失败，丢弃
    }
	
	//解析
	usb_packet.msg_type = usb_buf[1];
	usb_packet.app_id = usb_buf[2];
	usb_packet.length = usb_buf[3] << 8 | usb_buf[4];
	memcpy(usb_packet.data, usb_buf + 5, usb_packet.length);
	
	if(usb_packet.msg_type == 0x01) { // 上位机发送控制指令
		getRobotCommand();
	}
	
	return 0x00;
}

/**
 * @brief 将大端序字节数组转换为 float (针对 STM32 小端环境)
 */
float bytesToFloat(uint8_t* b) {
    uint32_t temp = ((uint32_t)b[0] << 24) | 
                    ((uint32_t)b[1] << 16) | 
                    ((uint32_t)b[2] << 8)  | 
                    ((uint32_t)b[3]);
    float f;
    memcpy(&f, &temp, 4);
    return f;
}

void getRobotCommand(void) {
    // 协议规定 data 部分是 5 个大端序 float
    // 分别对应: vel_x, vel_y, vel_w, leg_length, control_mode
    
    robot_cmd.vel_x        = bytesToFloat(&usb_packet.data[0]);
    robot_cmd.vel_y        = bytesToFloat(&usb_packet.data[4]);
    robot_cmd.vel_w        = bytesToFloat(&usb_packet.data[8]);
    robot_cmd.leg_length   = bytesToFloat(&usb_packet.data[12]);
    robot_cmd.control_mode = usb_packet.data[16];
}

void usb_receive_task(void){
	usbReceiveTaskInit();
	while(1) {
		usbReceive();
			
		USBD_CDC_SetRxBuffer(&hUsbDeviceHS, usb_receive_buf);
		USBD_CDC_ReceivePacket(&hUsbDeviceHS);
		
		osDelay(1);
	}
}
