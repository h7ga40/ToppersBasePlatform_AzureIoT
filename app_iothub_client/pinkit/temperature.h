// https://github.com/ms-iotkithol-jp/IoTKitHoLV3/blob/master/PinKitIoTHubApp/PinKitIoTHubApp/PinKit/Temperature.cs

#include <stdint.h>
#include <stdbool.h>

#ifndef _TEMPERATURE_H_
#define _TEMPERATURE_H_

bool Temperature_Init();
bool Temperature_Start();
double Temperature_TakeMeasurement();

#endif // _TEMPERATURE_H_
