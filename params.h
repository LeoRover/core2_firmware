#ifndef LEO_FIRMWARE_PARAMS_H_
#define LEO_FIRMWARE_PARAMS_H_

#include "hFramework.h"

static const char *FIRMWARE_VERSION = "1.1.0-ERC";

// The pin which will be used to drive the informative LED on the power switch
// By default it is set to pin1 on hExt port
static hFramework::hGPIO &LED = hExt.pin1;

#endif  // LEO_FIRMWARE_PARAMS_H_
