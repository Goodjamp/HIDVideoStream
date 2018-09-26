#include "PidController.h"

void pidControllerInit(PidController *pidController, int32_t kP, int32_t kI, int32_t kD, int32_t positiveSaturationValue, int32_t negativeSaturationValue, int32_t dampingValue)
{
	if (!pidController)
		return;

	pidControllerReset(pidController);

	pidController->kP = kP;
	pidController->kI = kI;
	pidController->kD = kD;
	pidController->positiveSaturationValue = positiveSaturationValue;
	pidController->negativeSaturationValue = negativeSaturationValue;
    pidController->positiveIntegralClampingValue = positiveSaturationValue * kI / kP;
    pidController->negativeIntegralClampingValue = negativeSaturationValue * kI / kP;
    pidController->dampingValue = dampingValue;
}

static int32_t pidControllerClampIntegralValue(PidController *pidController, int32_t value)
{
    if ((value * pidController->kP / pidController->kI) > pidController->positiveSaturationValue)
        value = pidController->positiveIntegralClampingValue;

    if ((value * pidController->kP / pidController->kI) < pidController->negativeSaturationValue)
       value = pidController->negativeIntegralClampingValue;

    return value;
}

int32_t pidControllerUpdate(PidController *pidController, int32_t error)
{
    int32_t cI;
    int32_t cD;

    if (!pidController)
		return 0;

    error /= pidController->dampingValue;

    pidController->errorIntegralValue = pidController->errorIntegralValue + ((error + pidController->lastError) >> 1);
	pidController->errorIntegralValue = pidControllerClampIntegralValue(pidController, pidController->errorIntegralValue);
	cI = pidController->kI ? pidController->errorIntegralValue / pidController->kI : 0;
    cD = (error - pidController->lastError) * pidController->kD;

    pidController->lastError = error;

    return (error + cI + cD) * pidController->kP;
}

void pidControllerReset(PidController *pidController)
{
	if (!pidController)
		return;

	pidController->errorIntegralValue = 0;
	pidController->lastError = 0;
}
