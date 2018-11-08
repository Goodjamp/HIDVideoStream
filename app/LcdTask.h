#ifndef __LCD_TASK_H__
#define __LCD_TASK_H__

#include "stdint.h"

void lcdTaskCreate(void);
void videoCommandProcessing(uint8_t frameBuff[]);

#endif
