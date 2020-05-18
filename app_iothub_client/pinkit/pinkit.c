/*
 *  PinKitセンサータスク
 */

#include "pinkit.h"
#include <math.h>
#include "device.h"
#include "syssvc/syslog.h"

pinkit_t pinkit;
uint16_t led_state = 0;

void pinkit_task(intptr_t exinf)
{
	int i2c_error = 0;

	Accelerometer_Init(100, 1000);
	Temperature_Init();
	BoardFullColorLED_Init();
	LightSensor_Init();

	for (;;) {
		if (i2c_error < 3) {
			if (Accelerometer_TakeMeasurements(&pinkit.accel))
				i2c_error = 0;
			else 
				i2c_error++;
		}
		else {
			i2c_error++;
			if (i2c_error == 4) {
				syslog(LOG_NOTICE, "Accelerometer error.");
			}
			else if (i2c_error >= 10000)
				i2c_error = 0;
		}

		pinkit.temperature = Temperature_TakeMeasurement();
		pinkit.humidity = LightSensor_TakeMeasurement();

		BoardFullColorLED_SetRgb(fabs(pinkit.accel.X) > 0.5, fabs(pinkit.accel.Y) > 0.5, fabs(pinkit.accel.Z) > 0.5);
		if (pinkit.ledOn)
			led_state |= LED04;
		else
			led_state &= ~LED04;
		led_out(led_state);

		Temperature_Start();
		LightSensor_Start();

		dly_tsk(10);
	}
}

