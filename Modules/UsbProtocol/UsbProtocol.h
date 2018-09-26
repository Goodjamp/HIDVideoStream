#ifndef __USB_PROTOCOL_H__
#define __USB_PROTOCOL_H__

#include <stdint.h>

void HID_ProcessReport(const uint8_t outReport[], uint8_t inReport[]);

#endif
