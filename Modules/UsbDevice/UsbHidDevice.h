#ifndef __USB_HID_DEVICE_H__
#define __USB_HID_DEVICE_H__

#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"

void usbHidDeviceSetCallbacks(void (*reportSentCb)(void), void (*reportReceivedCb)(const uint8_t reportData[], size_t reportSize));
void usbHidDeviceTaskFunction(void *pvParameters);
bool usbHidDeviceSendReport(const uint8_t reportData[]);

#endif /* __USB_HID_DEVICE_H__ */
