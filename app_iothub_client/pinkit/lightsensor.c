// https://github.com/ms-iotkithol-jp/IoTKitHoLV3/blob/master/PinKitIoTHubApp/PinKitIoTHubApp/PinKit/LightSensor.cs

#include <math.h>
#include <kernel.h>
#include <t_syslog.h>
#include <target_syssvc.h>
#include "pinkit.h"
#include "kernel_cfg.h"

ADC_Handle_t *aiLightSensor;

volatile uint16_t LightSensorValue;

/*
 *  ADC転送終了コールバック関数
 */
static void HAL_ADC_ConvCpltCallback2(ADC_Handle_t* hadc)
{
	LightSensorValue = adc_getvalue(aiLightSensor);
	adc_end_int(hadc);
}

bool LightSensor_Init()
{
	ADC_Init_t     aInit;
	ADC_ChannelConf_t sConfig;
	uint32_t event = 0;
	int i;
	Arduino_PortControlBlock *pcb;

	/*
	 *  ADC1初期化
	 */
	aInit.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	aInit.Resolution = ADC_RESOLUTION_12B;
	aInit.ScanConvMode = ADC_SCANMODE_DISABLE;
//	aInit.ContinuousConvMode = ADC_CONTINUOUS_ENABLE;
	aInit.ContinuousConvMode = ADC_CONTINUOUS_DISABLE;
	aInit.DiscontinuousConvMode = ADC_DISCONTINUOUS_DISABLE;
	aInit.NumDiscConversion = 0;
	aInit.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	aInit.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	aInit.DataAlign = ADC_DATAALIGN_RIGHT;
	aInit.NumConversion = 1;
	aInit.DMAContinuousRequests = ADC_DMACONTINUOUS_ENABLE;
	aInit.EOCSelection = ADC_EOC_SEQ_DISABLE;

	if((aiLightSensor = adc_init(ADC1_PORTID, &aInit)) == NULL){
		/* Initialization Error */
		syslog_0(LOG_ERROR, "## adc_init ERROR ##");
		return false;
	}
	aiLightSensor->xfercallback = HAL_ADC_ConvCpltCallback2;

	/*
	 *  ADCチャネル設定
	 */
	pcb = getGpioTable(ANALOG_PIN, 0);
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfig.GpioBase = pcb->giobase;
	sConfig.GpioPin  = pcb->giopin;
	if(adc_setupchannel(aiLightSensor, &sConfig) != E_OK){
		/* Channel Configuration Error */
		syslog_0(LOG_ERROR, "## adc_setupchannel ERROR ##");
		return false;
	}

	/*
	 *  ADC DMAスタート
	 */
	if(adc_start_int(aiLightSensor) != E_OK){
		/* Start Conversation Error */
		syslog_0(LOG_ERROR, "## adc_start_int ERROR ##");
		return false;
	}

	return true;
}

bool LightSensor_Start()
{
	if (aiLightSensor->status >= ADC_STATUS_EOC)
		return false;

	/*
	 *  ADC DMAスタート
	 */
	if(adc_start_int(aiLightSensor) != E_OK){
		/* Start Conversation Error */
		syslog_0(LOG_NOTICE, "## adc_start_dma ERROR ##");
		return false;
	}

	return true;
}

double LightSensor_TakeMeasurement()
{
	if (aiLightSensor == NULL)
		return 0.0;

	int raw = LightSensorValue << (16 - 12);

	return (double)raw / 65536.0;
}
