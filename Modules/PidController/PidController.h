#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include <stdint.h>

typedef struct PidController {
	int32_t kP;
	int32_t kI;
	int32_t kD;
	int32_t errorIntegralValue;
	int32_t lastError;
	int32_t positiveSaturationValue;
	int32_t negativeSaturationValue;
    int32_t positiveIntegralClampingValue;
    int32_t negativeIntegralClampingValue;
    int32_t dampingValue;
} PidController;

void pidControllerInit(PidController *pidController, int32_t kP, int32_t kI, int32_t kD, int32_t positiveSaturationValue, int32_t negativeSaturationValue, int32_t dampingValue);
int32_t pidControllerUpdate(PidController *pidController, int32_t error);
void pidControllerReset(PidController *pidController);

#endif


