/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2010 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2015-2016 by TOPPERS PROJECT Educational Working Group.
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
 *  $Id: lcdshield.h 2055 2020-03-02 03:39:28Z coas-nagasima $
 */

/*
 *  LCDシールド制御プログラムのヘッダファイル
 */

/*
 *  ターゲット依存の定義
 */
#include "target_test.h"
#include "usb_otg.h"
#include "tusbh_msc.h"
#include "glcd_disp.h"

/*
 *  各タスクの優先度の定義
 */

#define MAIN_PRIORITY	9		/* メインタスクの優先度 */
								/* HIGH_PRIORITYより高くすること */

#define HIGH_PRIORITY	6		/* 並行実行されるタスクの優先度 */
#define MID_PRIORITY	10
#define LOW_PRIORITY	11

/*
 *  ターゲットに依存する可能性のある定数の定義
 */

#ifndef TASK_PORTID
#define	TASK_PORTID		1			/* 文字入力するシリアルポートID */
#endif /* TASK_PORTID */

#ifndef STACK_SIZE
#define	STACK_SIZE		4096		/* タスクのスタックサイズ */
#endif /* STACK_SIZE */

//#define SPISDCARD_PORTID    0

#define PORT_HIGH      1
#define PORT_LOW       0 

#define GPIO_OTYPE     GPIO_OTYPE_PP
#ifndef GPIO_SELECT_PDOWN
#define GPIO_PULLMODE  GPIO_PULLUP
#else
#define GPIO_PULLMODE  GPIO_PULLDOWN
#endif

#define BUFFERSIZE      140

#define INHNO_SPI   IRQ_VECTOR_SPI1	/* 割込みハンドラ番号 */
#define INTNO_SPI   IRQ_VECTOR_SPI1	/* 割込み番号 */
#define INTPRI_SPI  -4          /* 割込み優先度 */
#define INTATR_SPI  TA_EDGE     /* 割込み属性 */

#define INHNO_DMARX   IRQ_VECTOR_DMA2_STREAM2	/* 割込みハンドラ番号 */
#define INTNO_DMARX   IRQ_VECTOR_DMA2_STREAM2	/* 割込み番号 */
#define INTPRI_DMARX  -4          /* 割込み優先度 */
#define INTATR_DMARX  TA_EDGE     /* 割込み属性 */

#define INHNO_DMATX   IRQ_VECTOR_DMA2_STREAM3	/* 割込みハンドラ番号 */
#define INTNO_DMATX   IRQ_VECTOR_DMA2_STREAM3	/* 割込み番号 */
#define INTPRI_DMATX  -4          /* 割込み優先度 */
#define INTATR_DMATX  TA_EDGE     /* 割込み属性 */


#define INHNO_USBFS   IRQ_VECTOR_OTG_FS	/* 割込みハンドラ番号 */
#define INTNO_USBFS   IRQ_VECTOR_OTG_FS	/* 割込み番号 */
#define INTPRI_USBFS  -5			/* 割込み優先度 */
#define INTATR_USBFS  0				/* 割込み属性 */

#define INHNO_FSWKUP  IRQ_VECTOR_OTG_FS_WKUP	/* 割込みハンドラ番号 */
#define INTNO_FSWKUP  IRQ_VECTOR_OTG_FS_WKUP	/* 割込み番号 */
#define INTPRI_FSWKUP -10			/* 割込み優先度 */
#define INTATR_FSWKUP 0				/* 割込み属性 */


#define INHNO_ADC     IRQ_VECTOR_ADC	/* 割込みハンドラ番号 */
#define INTNO_ADC     IRQ_VECTOR_ADC	/* 割込み番号 */
#define INTPRI_ADC    -5          /* 割込み優先度 */
#define INTATR_ADC    0           /* 割込み属性 */

#define INHNO_DMAADC  IRQ_VECTOR_DMA2_STREAM0	/* 割込みハンドラ番号 */
#define INTNO_DMAADC  IRQ_VECTOR_DMA2_STREAM0	/* 割込み番号 */
#define INTPRI_DMAADC -4          /* 割込み優先度 */
#define INTATR_DMAADC TA_EDGE     /* 割込み属性 */

/* Definition for ADCx's Channel */
#define ADCx_CHANNEL                    ADC_CHANNEL_9

/* Definition for ADCx Channel Pin */
#define ADCx_CHANNEL_GPIO_PORT          TADR_GPIOF_BASE
#define ADCx_CHANNEL_PINNO              3

#define NUM_USBH_EVT1   64
#define NUM_USBH_EVT2   16

#define NUM_JSPOSITION 6

#define JS_OFF      0x01
#define JS_ON       0x02
#define JS_UP       0x04
#define JS_DOWN     0x08
#define JS_LEFT     0x10
#define JS_RIGHT    0x20
#define JS_MASK     0x3F

#define CS_PORT     10
#define RST_PORT    9
#define RS_PORT     8
#define CS2_PORT    4

#define cs_set(sw)  digitalWrite(CS_PORT, sw)
#define rs_set(sw)  digitalWrite(RS_PORT, sw)
#define rst_set(sw) digitalWrite(RST_PORT, sw)
#define cs2_set(sw) digitalWrite(CS2_PORT, sw)

#ifndef TOPPERS_MACRO_ONLY

/*
 *  ヒープ領域の設定
 */
extern uint32_t heap_param[2];

/*
 *  関数のプロトタイプ宣言
 */
extern void	main_task(intptr_t exinf);
extern void stick_task(intptr_t exinf);

extern void heap_init(intptr_t exinf);
extern void sw_int(void);
extern void device_info_init(intptr_t exinf);


#endif /* TOPPERS_MACRO_ONLY */
