// https://github.com/ms-iotkithol-jp/IoTKitHoLV3/blob/master/PinKitIoTHubApp/PinKitIoTHubApp/PinKit/Accelerometer.cs

#include <kernel.h>
#include <t_syslog.h>
#include <target_syssvc.h>
#include "pinkit.h"
#include "kernel_cfg.h"

#define ADXL345_ADDR (0x1d << 1)
// ADXL345 registers
#define R_DEVID (0x00)
#define R_THRESH_TAP (0x1D)
#define R_OFSX (0x1E)
#define R_OFSY (0x1F)
#define R_OFSZ (0x20)
#define R_DUR (0x21)
#define R_Latent (0x22)
#define R_Window (0x23)
#define R_THRESH_INACT (0x25)
#define R_TIME_INACT (0x26)
#define R_ACT_INACT_CTL (0x27)
#define R_THRESH_FF (0x28)
#define R_TIME_FF (0x29)
#define R_TAP_AXES (0x2A)
#define R_ACT_TAP_STATUS (0x2B)
#define R_BW_RATE (0x2C)
#define R_POWER_CTL (0x2D)
#define R_INT_ENABLE (0x2E)
#define R_INT_MAP (0x2F)
#define R_INT_SOURCE (0x30)
#define R_DATA_FORMAT (0x31)
#define R_DATAX0 (0x32)
#define R_DATAX1 (0x33)
#define R_DATAY0 (0x34)
#define R_DATAY1 (0x35)
#define R_DATAZ0 (0x36)
#define R_DATAZ1 (0x37)
#define R_FIFO_CTL (0x38)
#define R_FIFO_STATUS (0x39)

I2C_Handle_t *hi2c;
#define GPIO_I2C_CS 13
static int timeout = 1000;
char xyz[6];

int RegRead(uint8_t reg)
{
	ER ret;
	int result = -1;
	uint8_t rdata[1];

	digitalWrite(GPIO_I2C_CS, 0);

	ret = i2c_memread(hi2c, ADXL345_ADDR, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, rdata, 1, timeout);
	if (ret != E_OK)
		goto exit;

	digitalWrite(GPIO_I2C_CS, 1);

	result = rdata[0];
exit:
#ifdef DEBUG_ADXL345
	printf("R[0x%2X]=0x%02X\n", reg, rdata[0]);
#endif
	return result;
}

int RegReads(uint8_t reg, char *data, int len)
{
	ER ret;
	int result = -1;

	digitalWrite(GPIO_I2C_CS, 0);

	ret = i2c_memread(hi2c, ADXL345_ADDR, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, timeout);
	if (ret != E_OK)
		goto exit;

	result = 0;
exit:
	digitalWrite(GPIO_I2C_CS, 1);

#ifdef DEBUG_ADXL345
	for (int i = 0; i < len; i++)
	{
		printf("R[0x%2X]=0x%02X\n", (reg + i), data[i]);
	}
#endif
	return result;
}

int RegWrite(uint8_t reg, uint8_t val)
{
	ER ret;
	int result = -1;
	uint8_t wdata[1];

	wdata[0] = val;

	digitalWrite(GPIO_I2C_CS, 0);

	ret = i2c_memwrite(hi2c, ADXL345_ADDR, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, wdata, 1, timeout);
	if (ret != E_OK)
		goto exit;

	digitalWrite(GPIO_I2C_CS, 1);

	result = 0;
exit:
	return result;
}

int RegWriteMask(uint8_t reg, uint8_t val, uint8_t mask)
{
	ER ret;
	int result = -1;
	uint8_t wdata[1];

	uint8_t tmp = RegRead(reg);

	wdata[0] = (uint8_t)(((int)tmp & ~(int)mask) | ((int)val & (int)mask));

	digitalWrite(GPIO_I2C_CS, 0);

	ret = i2c_memwrite(hi2c, ADXL345_ADDR, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, wdata, 1, timeout);
	if (ret != E_OK)
		goto exit;

	digitalWrite(GPIO_I2C_CS, 1);

	result = 0;
exit:
	return result;
}

uint8_t DeviceID()
{
	return RegRead(R_DEVID);
}

void ToSleep()
{
	RegWriteMask(R_POWER_CTL, 0x04, 0x04);
}

void ToWakeup()
{
	RegWriteMask(R_POWER_CTL, 0x00, 0x04);
}

// Normal Power
// Code     Output  BandWidth
// 0b1111   3200Hz  (1600Hz)
// 0b1110   1600Hz  (800Hz)
// 0b1101   800Hz   (40Hz)
// 0b1100   400Hz   (200Hz)
// 0b1011   200Hz   (100Hz)
// 0b1010   100Hz   (50Hz)
// 0b1001   50Hz    (25H)
// 0b1000   25Hz    (12.5Hz)
// 0b0111   12.5Hz  (6.25Hz)
// 0b0110   6.25Hz  (3.125Hz)
// Low Power
// 0b1100   400Hz   (200Hz)
// 0b1011   200Hz   (100Hz)
// 0b1010   100Hz   (50Hz)
// 0b1001   50Hz    (25H)
// 0b1000   25Hz    (12.5Hz)
// 0b0111   12.5Hz  (6.25Hz)
void SetBW_RATE(uint8_t n)
{
	RegWrite(R_BW_RATE, n);
}

// D7: SELF_TEST
// D6: SPI
// D5: INT_INVERT
// D4: 0
// D3: FULL_RES
// D2: Justfy
// D1-D0: Range
// 0 - 0: +-2g
// 0 - 1: +-4g
// 1 - 0: +-8g
// 1 - 1: +-16g
void SetDataFormat(uint8_t n)
{
	RegWrite(R_DATA_FORMAT, n);
}

void SetFullResolution()
{
	RegWriteMask(R_DATA_FORMAT, 0x08, 0x08);
}

void Measure()
{
	RegWriteMask(R_POWER_CTL, 0x08, 0x08);
}

static int ReadXYZ(int16_t *x, int16_t *y, int16_t *z)
{
	int result;
	result = RegReads(R_DATAX0, xyz, 6);
	*x = (int16_t)(((uint16_t)xyz[1] << 8) + (uint16_t)xyz[0]);
	*y = (int16_t)(((uint16_t)xyz[3] << 8) + (uint16_t)xyz[2]);
	*z = (int16_t)(((uint16_t)xyz[5] << 8) + (uint16_t)xyz[4]);
	return result;
}

static bool measuring = false;

/*
 *  GPIO設定関数
 */
static void
pinMode(uint8_t no, uint32_t mode)
{
	Arduino_PortControlBlock *pgcb = getGpioTable(DIGITAL_PIN, no);
	GPIO_Init_t GPIO_Init_Data;

	if(pgcb == NULL)
		return;
	pinClock(no);
	GPIO_Init_Data.mode  = mode;
	GPIO_Init_Data.pull  = GPIO_PULLUP;
	GPIO_Init_Data.otype = GPIO_OTYPE_PP;
	GPIO_Init_Data.speed = GPIO_SPEED_FAST;
	gpio_setup(pgcb->giobase, &GPIO_Init_Data, pgcb->giopin);
}

bool Accelerometer_Init(int clockRate, int tmout)
{
	I2C_Init_t i2c_initd;

	timeout = tmout;

	i2c_initd.Timing          = 0x00D00E28;
	i2c_initd.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	i2c_initd.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2c_initd.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2c_initd.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
	i2c_initd.OwnAddress1     = 0;
	i2c_initd.OwnAddress2     = 0;
	i2c_initd.semid           = I2CTRS_SEM;
	i2c_initd.semlock         = I2CLOC_SEM;

	if ((hi2c = i2c_init(I2C_PORTID, &i2c_initd)) == NULL) {
		/* Initialization Error */
		syslog_0(LOG_ERROR, "## I2C ERROR(1) ##");
		return false;
	}

	pinMode(GPIO_I2C_CS, GPIO_MODE_OUTPUT);

	ToWakeup();
	dly_tsk(10);			// 10[ms]

	SetDataFormat(0x01);    // +-2g 10bit
	SetBW_RATE(0x19);       // sampling rate 50Hz

	return true;
}

bool Accelerometer_TakeMeasurements(SensorReading *result)
{
	if (result == NULL)
		return false;

	if (!measuring)
	{
		Measure();
	}

	int16_t x, y, z;
	int ret = ReadXYZ(&x, &y, &z);

	result->X = ((double)x) / 127.0;
	result->Y = ((double)y) / 127.0;
	result->Z = ((double)z) / 127.0;

	return ret == 0;
}
