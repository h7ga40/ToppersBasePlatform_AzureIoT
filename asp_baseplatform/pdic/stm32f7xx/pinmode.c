/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
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
 *  $Id$
 */

/*
 *  ARDUNO-PINドライバ
 */

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <string.h>
#include <target_syssvc.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "device.h"
#include "pinmode.h"

#define sil_orw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) | (b))
#define sil_andw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) & ~(b))
#define sil_modw_mem(a, b, c)	sil_wrw_mem((a), (sil_rew_mem(a) & (~b)) | (c))


static const Arduino_PortControlBlock Digital_Port[] = {
#if defined(TOPPERS_STM32F7_DISCOVERY)
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOCEN, TADR_GPIOC_BASE, 7},	/* D0 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOCEN, TADR_GPIOC_BASE, 6},	/* D1 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOGEN, TADR_GPIOG_BASE, 6},	/* D2 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOBEN, TADR_GPIOB_BASE, 4},	/* D3 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOGEN, TADR_GPIOG_BASE, 7},	/* D4 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOAEN, TADR_GPIOA_BASE, 8},	/* D5 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOHEN, TADR_GPIOH_BASE, 6},	/* D6 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOIEN, TADR_GPIOI_BASE, 3},	/* D7 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOIEN, TADR_GPIOI_BASE, 2},	/* D8 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOAEN, TADR_GPIOA_BASE, 15},	/* D9 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOIEN, TADR_GPIOI_BASE, 0},	/* D10 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOBEN, TADR_GPIOB_BASE, 15},	/* D11 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOBEN, TADR_GPIOB_BASE, 14},	/* D12 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOIEN, TADR_GPIOI_BASE, 1}		/* D13 */
#else	/* TOPPERS_STM32F746_NUCLEO144 TOPPERS_STM32F767_NUCLEO144 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOGEN, TADR_GPIOG_BASE, 9},	/* D0 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOGEN, TADR_GPIOG_BASE, 14},	/* D1 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOFEN, TADR_GPIOF_BASE, 15},	/* D2 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOEEN, TADR_GPIOE_BASE, 13},	/* D3 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOFEN, TADR_GPIOF_BASE, 14},	/* D4 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOEEN, TADR_GPIOE_BASE, 11},	/* D5 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOEEN, TADR_GPIOE_BASE, 9},	/* D6 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOFEN, TADR_GPIOF_BASE, 13},	/* D7 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOFEN, TADR_GPIOF_BASE, 12},	/* D8 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIODEN, TADR_GPIOD_BASE, 15},	/* D9 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIODEN, TADR_GPIOD_BASE, 14},	/* D10 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOAEN, TADR_GPIOA_BASE, 7},	/* D11 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOAEN, TADR_GPIOA_BASE, 6},	/* D12 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOAEN, TADR_GPIOA_BASE, 5}		/* D13 */
#endif
};

#define NUM_DIGITAL_PORT (sizeof(Digital_Port)/sizeof(Arduino_PortControlBlock))

static const Arduino_PortControlBlock Analog_Port[] = {
#if defined(TOPPERS_STM32F7_DISCOVERY)
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOAEN, TADR_GPIOA_BASE, 0},	/* A0 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOFEN, TADR_GPIOF_BASE, 10},	/* A1 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOFEN, TADR_GPIOF_BASE, 9},	/* A2 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOFEN, TADR_GPIOF_BASE, 8},	/* A3 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOFEN, TADR_GPIOF_BASE, 7},	/* A4 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOFEN, TADR_GPIOF_BASE, 6}		/* A5 */
#else	/* TOPPERS_STM32F746_NUCLEO144 TOPPERS_STM32F767_NUCLEO144 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOAEN, TADR_GPIOA_BASE, 3},	/* A0 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOCEN, TADR_GPIOC_BASE, 0},	/* A1 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOCEN, TADR_GPIOC_BASE, 3},	/* A2 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOFEN, TADR_GPIOF_BASE, 3},	/* A3 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOFEN, TADR_GPIOF_BASE, 5},	/* A4 */
  { (TADR_RCC_BASE+TOFF_RCC_AHB1ENR),  RCC_AHB1ENR_GPIOFEN, TADR_GPIOF_BASE, 10}	/* A5 */
#endif
};

#define NUM_ANALOG_PORT (sizeof(Analog_Port)/sizeof(Arduino_PortControlBlock))

void
pinClock(uint8_t no)
{
	const Arduino_PortControlBlock *ppcb = &Digital_Port[no];

	if(no >= NUM_DIGITAL_PORT)
		return;
	sil_orw_mem((uint32_t *)ppcb->gioclockbase, ppcb->gioclockbit);
}

void
digitalWrite(uint8_t no, int sw)
{
	const Arduino_PortControlBlock *ppcb = &Digital_Port[no];

	if(no >= NUM_DIGITAL_PORT)
		return;
	if(sw == 0)
		sil_wrw_mem((uint32_t *)(ppcb->giobase+TOFF_GPIO_BSRR), (1<<(ppcb->giopin+16)));
	else
		sil_wrw_mem((uint32_t *)(ppcb->giobase+TOFF_GPIO_BSRR), (1<<(ppcb->giopin)));
}

int
digitalRead(uint8_t no)
{
	const Arduino_PortControlBlock *ppcb = &Digital_Port[no];
 	int sw;

	if(no >= NUM_DIGITAL_PORT)
		return 0;
	sw = sil_rew_mem((uint32_t *)(ppcb->giobase+TOFF_GPIO_IDR));
	sw = (sw>>(ppcb->giopin)) & 1;
	return sw;
}

Arduino_PortControlBlock*
getGpioTable(uint8_t mode, uint8_t no)
{
	if(mode == ANALOG_PIN){
		if(no >= NUM_ANALOG_PORT)
			return NULL;
		return (Arduino_PortControlBlock*)&Analog_Port[no];
	}
	else{
		if(no >= NUM_DIGITAL_PORT)
			return NULL;
		return (Arduino_PortControlBlock*)&Digital_Port[no];
	}
}


