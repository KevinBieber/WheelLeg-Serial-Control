#pragma once
#include "stm32h7xx_hal.h"   // 换成你的芯片头文件

void DWT_Init(void);
uint32_t micros(void);