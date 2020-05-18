/*
 *  TOPPERS BASE PLATFORM MIDDLEWARE
 * 
 *  Copyright (C) 2017-2019 by TOPPERS PROJECT
 *                             Educational Working Group.
 *
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id$
 */

/*
 *  Written by Limor Fried/Ladyada for Adafruit Industries.
 *  MIT license, all text above must be included in any redistribution
 *　http://opensource.org/licenses/mit-license.php 
 */

/*
 *  ADAFRUIT ILI9341 2.8"LCD制御プログラムのヘッダファイル
 */

#ifndef _ADAFRUIT_ILI9341_H_
#define _ADAFRUIT_ILI9341_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "device.h"
#include "pinmode.h"
#include "spi.h"

/*
 *  LCDカラーモード定義
 */
#define CM_ARGB8888         0x00000000			/* ARGB8888 color mode */
#define CM_RGB888           0x00000001			/* RGB888 color mode */
#define CM_RGB565           0x00000002			/* RGB565 color mode */



#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0A
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR   0x30
#define ILI9341_MADCTL  0x36
#define ILI9341_PIXFMT  0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1

// Color definitions
#define	ILI9341_BLACK   0x0000
#define	ILI9341_BLUE    0x001F
#define	ILI9341_RED     0xF800
#define	ILI9341_GREEN   0x07E0
#define ILI9341_CYAN    0x07FF
#define ILI9341_MAGENTA 0xF81F
#define ILI9341_YELLOW  0xFFE0
#define ILI9341_WHITE   0xFFFF


#ifndef TOPPERS_MACRO_ONLY

/*
 *  カラーの属性を定義
 */
typedef uint16_t   color_t;

/*
 *  LCDハンドラ構造体定義
 */
typedef struct
{
	SPI_Handle_t            *hspi;		/* spiハンドラ */
	uint16_t                _width;		/* 幅ピクセル数 */
	uint16_t				_height;	/* 高さピクセル数 */
	uint16_t                colstart;
	uint16_t                rowstart;
	uint8_t                 rotation;	/* LCD回転設定 */
	uint8_t                 cs_pin;
	uint8_t                 dc_pin;
	uint8_t                 rst_pin;
	ID                      spi_lock;
}LCD_Handler_t;

/*
 *  描画構造体
 */
typedef struct
{
	LCD_Handler_t           *hlcd;
	color_t                 TextColor;
	color_t                 BackColor;
	void                    *pFont;
}LCD_DrawProp_t;

typedef struct
{
	int16_t X;
	int16_t Y;
}Point, * pPoint;

/*
 *  関数のプロトタイプ宣言
 */
extern ER lcd_writecommand(LCD_Handler_t *hlcd, uint8_t c);
extern ER lcd_writedata(LCD_Handler_t *hlcd, uint8_t c);
extern ER lcd_writedata2(LCD_Handler_t *hlcd, uint16_t c, int cnt);
extern ER lcd_writedata3(LCD_Handler_t *hlcd, uint8_t *pbmp, uint16_t width, uint16_t height);

extern void lcd_init(LCD_Handler_t *hlcd, uint8_t rotation);
extern void lcd_setAddrWindow(LCD_Handler_t *hlcd, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
extern void lcd_fillRect(LCD_Handler_t *hlcd, int16_t x, int16_t y, int16_t w, int16_t h, color_t color);
extern void lcd_pushColor(LCD_Handler_t *hlcd, color_t color);
extern void lcd_drawPixel(LCD_Handler_t *hlcd, int16_t x, int16_t y, color_t color);
extern void lcd_drawFastVLine(LCD_Handler_t *hlcd, int16_t x, int16_t y, int16_t h, color_t color);
extern void lcd_drawFastHLine(LCD_Handler_t *hlcd, int16_t x, int16_t y, int16_t w, color_t color);
extern void lcd_drawImageHLine(LCD_Handler_t *hlcd, int16_t x, int16_t y, uint16_t w, color_t *pcolor);
extern void lcd_drawBitmap(LCD_Handler_t *hlcd, uint16_t x0, uint16_t y0, uint8_t *pbmp);
extern void lcd_setRotation(LCD_Handler_t *hlcd, uint8_t m);
extern void lcd_invertDisplay(LCD_Handler_t *hlcd, bool_t i);

extern void lcd_fillScreen(LCD_DrawProp_t *pDrawProp);
extern void lcd_drawRect(LCD_DrawProp_t *pDrawProp, int16_t x, int16_t y, int16_t w, int16_t h);
extern void lcd_DrawCircle(LCD_DrawProp_t *pDrawProp, uint16_t x0, uint16_t y0, uint16_t Radius);
extern void lcd_drawLine(LCD_DrawProp_t *pDrawProp, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
extern void lcd_drawPolygon(LCD_DrawProp_t *pDrawProp, pPoint Points, uint16_t PointCount);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif

#endif	/* _ADAFRUIT_ILI9341_H_ */

