// https://github.com/ms-iotkithol-jp/IoTKitHoLV3/blob/master/PinKitIoTHubApp/PinKitIoTHubApp/PinKit/BoardFullColorLED.cs

#include <kernel.h>
#include "pinkit.h"
#include "kernel_cfg.h"

extern uint16_t led_state;

// コンストラクター
void BoardFullColorLED_Init()
{
	// 各 LED の InputPort インスタンス
	led_state = 0;
}

/// <summary>
/// 指定の色で LED を点灯、消灯する
/// </summary>
/// <param name="redOn">true ならば赤を点灯</param>
/// <param name="greenOn">true ならば緑を点灯</param>
/// <param name="blueOn">true ならば青を点灯</param>
void BoardFullColorLED_SetRgb(bool redOn, bool greenOn, bool blueOn)
{
	if (redOn)
		led_state |= LED01;
	else
		led_state &= ~LED01;
	if (greenOn)
		led_state |= LED02;
	else
		led_state &= ~LED02;
	if (blueOn)
		led_state |= LED03;
	else
		led_state &= ~LED03;
	led_out(led_state);
}

/// <summary>
/// 色名指定で LED を点灯する
/// </summary>
/// <param name="color"></param>
void BoardFullColorLED_SetColor(Colors color)
{
	int redFlag = (int)color & (int)Colors_Red;
	int greenFlag = (int)color & (int)Colors_Green;
	int blueFlag = (int)color & (int)Colors_Blue;
	if (redFlag != 0)
		led_state |= LED01;
	else
		led_state &= ~LED01;
	if (greenFlag != 0)
		led_state |= LED02;
	else
		led_state &= ~LED02;
	if (blueFlag != 0)
		led_state |= LED03;
	else
		led_state &= ~LED03;
	led_out(led_state);
}
