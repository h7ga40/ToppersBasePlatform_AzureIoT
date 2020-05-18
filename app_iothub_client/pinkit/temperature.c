// https://github.com/ms-iotkithol-jp/IoTKitHoLV3/blob/master/PinKitIoTHubApp/PinKitIoTHubApp/PinKit/Temperature.cs

#include <math.h>
#include <kernel.h>
#include <t_syslog.h>
#include <target_syssvc.h>
#include "pinkit.h"
#include "kernel_cfg.h"

ADC_Handle_t *aiTemperature;

// Adjust VR1 - Default 5000Ω
double VR1;

// B constant - Default 3435Ω
double Bc;

volatile uint16_t TemperatureValue;

/*
 *  ADC転送終了コールバック関数
 */
static void HAL_ADC_ConvCpltCallback2(ADC_Handle_t* hadc)
{
	TemperatureValue = adc_getvalue(aiTemperature);
	adc_end_int(hadc);
}

bool Temperature_Init()
{
	ADC_Init_t     aInit;
	ADC_ChannelConf_t sConfig;
	uint32_t event = 0;
	int i;
	Arduino_PortControlBlock *pcb;

	VR1 = 5000.0;
	Bc = 3435.0;

	/*
	 *  ADC5初期化
	 */
	aInit.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	aInit.Resolution = ADC_RESOLUTION_12B;
	aInit.ScanConvMode = ADC_SCANMODE_DISABLE;
	aInit.ContinuousConvMode = ADC_CONTINUOUS_ENABLE;
	aInit.DiscontinuousConvMode = ADC_DISCONTINUOUS_DISABLE;
	aInit.NumDiscConversion = 0;
	aInit.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	aInit.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	aInit.DataAlign = ADC_DATAALIGN_RIGHT;
	aInit.NumConversion = 1;
	aInit.DMAContinuousRequests = ADC_DMACONTINUOUS_DISABLE;
	aInit.EOCSelection = ADC_EOC_SEQ_DISABLE;

	if((aiTemperature = adc_init(ADC3_PORTID, &aInit)) == NULL){
		/* Initialization Error */
		syslog_0(LOG_ERROR, "## adc_init ERROR ##");
		return false;
	}
	aiTemperature->xfercallback = HAL_ADC_ConvCpltCallback2;

	/*
	 *  ADCチャネル設定
	 */
	pcb = getGpioTable(ANALOG_PIN, 5);
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfig.GpioBase = pcb->giobase;
	sConfig.GpioPin  = pcb->giopin;
	if(adc_setupchannel(aiTemperature, &sConfig) != E_OK){
		/* Channel Configuration Error */
		syslog_0(LOG_ERROR, "## adc_setupchannel ERROR ##");
		return false;
	}

	/*
	 *  ADC DMAスタート
	 */
	if(adc_start_int(aiTemperature) != E_OK){
		/* Start Conversation Error */
		syslog_0(LOG_ERROR, "## adc_start_int ERROR ##");
		return false;
	}

	return true;
}

bool Temperature_Start()
{
	if (aiTemperature->status >= ADC_STATUS_EOC)
		return false;

	/*
	 *  ADC DMAスタート
	 */
	if(adc_start_int(aiTemperature) != E_OK){
		/* Start Conversation Error */
		syslog_0(LOG_NOTICE, "## adc_start_int ERROR ##");
		return false;
	}

	return true;
}

double Temperature_TakeMeasurement()
{
	if (aiTemperature == NULL)
		return -273.15;

	int raw = TemperatureValue << (16 - 12);

	double tk = 273.0;
	double t25 = tk + 25.0;
	double r25 = 10000.0;
	double t = 1.0 / (log(VR1 * raw / (65536 - raw) / r25) / Bc + 1.0 / t25) - tk;
	return t;
}
