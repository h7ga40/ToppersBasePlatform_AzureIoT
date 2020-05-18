/*
 *  PinKitセンサータスク
 */

#ifndef _PINKIT_H_
#define _PINKIT_H_

#include <kernel.h>
#include <t_syslog.h>
#include <target_syssvc.h>
#include "device.h"
#include "pinmode.h"
#include "i2c.h"
#include "adc.h"
#include "accelerometer.h"
#include "temperature.h"
#include "lightsensor.h"
#include "boardfullcolorled.h"

#define PINKIT_PRIORITY			5		/* PinKitタスクの優先度 */

#define PINKIT_STACK_SIZE		1024	/* PinKitタスクのスタック領域のサイズ */

#define I2C_PORTID I2C1_PORTID
#define INHNO_I2CEV   IRQ_VECTOR_I2C1_EV	/* 割込みハンドラ番号 */
#define INTNO_I2CEV   IRQ_VECTOR_I2C1_EV	/* 割込み番号 */
#define INTPRI_I2CEV  -5			/* 割込み優先度 */
#define INTATR_I2CEV  0				/* 割込み属性 */

#define INHNO_I2CER   IRQ_VECTOR_I2C1_ER	/* 割込みハンドラ番号 */
#define INTNO_I2CER   IRQ_VECTOR_I2C1_ER	/* 割込み番号 */
#define INTPRI_I2CER  -5			/* 割込み優先度 */
#define INTATR_I2CER  0				/* 割込み属性 */

#define INHNO_ADC     IRQ_VECTOR_ADC	/* 割込みハンドラ番号 */
#define INTNO_ADC     IRQ_VECTOR_ADC	/* 割込み番号 */
#define INTPRI_ADC    -5          /* 割込み優先度 */
#define INTATR_ADC    0           /* 割込み属性 */

#define INHNO_DMAADC  IRQ_VECTOR_DMA2_STREAM0	/* 割込みハンドラ番号 */
#define INTNO_DMAADC  IRQ_VECTOR_DMA2_STREAM0	/* 割込み番号 */
#define INTPRI_DMAADC -4          /* 割込み優先度 */
#define INTATR_DMAADC TA_EDGE     /* 割込み属性 */

typedef struct pinkit_t {
	SensorReading accel;
	double temperature;
	double humidity;
	int ledOn;
} pinkit_t;

extern pinkit_t pinkit;

extern void pinkit_task(intptr_t exinf);

extern void device_info_init(intptr_t exinf);

extern void i2c_ev_isr(intptr_t exinf);
extern void i2c_er_isr(intptr_t exinf);

#endif // _PINKIT_H_
