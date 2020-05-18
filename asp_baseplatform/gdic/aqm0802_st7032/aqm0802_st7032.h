/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2010 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2015-2017 by TOPPERS PROJECT Educational Working Group.
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
 *  AQM0802 ST7032 LCD制御プログラムのヘッダファイル
 */

#ifndef _AQM0802_ST7032_H_
#define _AQM0802_ST7032_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "device.h"
#include "pinmode.h"
#include "i2c.h"

/*
 *  ST7032 RS R/W
 */
#define ST7032_CMD          0x00
#define ST7032_DAT          0x40
#define ST7032_CONT         0x80

#define LCD_INTERVAL1       70
#define LCD_INTERVAL2       27

#ifndef TOPPERS_MACRO_ONLY

/*
 *  LCDハンドラ構造体定義
 */
typedef struct
{
	I2C_Handle_t            *hi2c;
	uint16_t                saddr;		/* 幅ピクセル数 */
	uint8_t                 max_col;	/* 高さピクセル数 */
	uint8_t                 max_raw;
}CLCD_Handler_t;

/*
 *  関数のプロトタイプ宣言
 */
extern ER aqm0802_init(CLCD_Handler_t *hlcd);
extern ER aqm0802_set_command(CLCD_Handler_t *hlcd, uint8_t c);
extern ER aqm0802_set_data(CLCD_Handler_t *hlcd, uint8_t *p, uint8_t len);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif

#endif	/* _AQM0802_ST7032_H_ */

