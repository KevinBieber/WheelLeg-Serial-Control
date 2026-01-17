#ifndef VOFA_TASK_H
#define VOFA_TASK_H

#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"

void vofaTaskInit(void);
void sendDataAppend(float send_data);
void vofaSend(void);
void vofa_task(void);

#endif
