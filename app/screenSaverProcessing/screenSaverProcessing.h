#include "stdint.h"

#ifndef SCREENSAVERPROCESSING_H_
#define SCREENSAVERPROCESSING_H_

void calculateScreenSaverFrame(uint32_t cntMs, uint8_t *buff, uint32_t buffSize);
void initScreenSaver(void);

#endif
