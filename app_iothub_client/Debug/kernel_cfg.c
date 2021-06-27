/* kernel_cfg.c */
#include "kernel/kernel_int.h"
#include "kernel_cfg.h"

#if TKERNEL_PRID != 0x07u
#error The kernel does not match this configuration file.
#endif

/*
 *  Include Directives (#include)
 */

#include "target_timer.h"
#include "syssvc/syslog.h"
#include "syssvc/banner.h"
#include "target_syssvc.h"
#include "target_serial.h"
#include "syssvc/serial.h"
#include "syssvc/logtask.h"
#include "device.h"
#include "device.h"
#include "sdmmc.h"
#include "storagedevice.h"
#include "ff.h"
#include "sddiskio.h"
#include "monitor.h"
#include "lwip.h"
#include "pinkit.h"
#include "itron.h"
#include "device.h"
#include "emac.h"
#include "main.h"

/*
 *  Task Management Functions
 */

#define TNUM_STSKID	14

const ID _kernel_tmax_tskid = (TMIN_TSKID + TNUM_TSKID - 1);
const ID _kernel_tmax_stskid = (TMIN_TSKID + TNUM_STSKID - 1);

static STK_T _kernel_stack_LOGTASK[COUNT_STK_T(LOGTASK_STACK_SIZE)];
static STK_T _kernel_stack_SDM_TASK[COUNT_STK_T(1024)];
static STK_T _kernel_stack_MONTASK[COUNT_STK_T(MONITOR_STACK_SIZE)];
static STK_T _kernel_stack_LWIP_TASK_1[COUNT_STK_T(LWIP_STACK_SIZE)];
static STK_T _kernel_stack_LWIP_TASK_2[COUNT_STK_T(LWIP_STACK_SIZE)];
static STK_T _kernel_stack_LWIP_TASK_3[COUNT_STK_T(LWIP_STACK_SIZE)];
static STK_T _kernel_stack_LWIP_TASK_4[COUNT_STK_T(LWIP_STACK_SIZE)];
static STK_T _kernel_stack_LWIP_TASK_5[COUNT_STK_T(LWIP_STACK_SIZE)];
static STK_T _kernel_stack_LWIP_TASK_6[COUNT_STK_T(LWIP_STACK_SIZE)];
static STK_T _kernel_stack_LWIP_TASK_7[COUNT_STK_T(LWIP_STACK_SIZE)];
static STK_T _kernel_stack_LWIP_TASK_8[COUNT_STK_T(LWIP_STACK_SIZE)];
static STK_T _kernel_stack_LWIP_TASK_N[COUNT_STK_T(LWIP_STACK_SIZE)];
static STK_T _kernel_stack_PINKIT_TASK[COUNT_STK_T(PINKIT_STACK_SIZE)];
static STK_T _kernel_stack_MAIN_TASK1[COUNT_STK_T(STACK_SIZE)];

const TINIB _kernel_tinib_table[TNUM_STSKID] = {
	{ (TA_ACT), (intptr_t)(LOGTASK_PORTID), ((TASK)(logtask_main)), INT_PRIORITY(LOGTASK_PRIORITY), ROUND_STK_T(LOGTASK_STACK_SIZE), _kernel_stack_LOGTASK, (TA_NULL), (NULL) },
	{ (TA_ACT), (intptr_t)(1), ((TASK)(SDMSence_task)), INT_PRIORITY(15), ROUND_STK_T(1024), _kernel_stack_SDM_TASK, (TA_NULL), (NULL) },
	{ (TA_ACT), (intptr_t)(MONITOR_PORTID), ((TASK)(monitor)), INT_PRIORITY(MONITOR_PRIORITY), ROUND_STK_T(MONITOR_STACK_SIZE), _kernel_stack_MONTASK, (TA_NULL), (NULL) },
	{ (TA_NULL), (intptr_t)(0), ((TASK)(lwip_task)), INT_PRIORITY(LWIP_DEFAULT_PRIORITY), ROUND_STK_T(LWIP_STACK_SIZE), _kernel_stack_LWIP_TASK_1, (TA_NULL), (NULL) },
	{ (TA_NULL), (intptr_t)(1), ((TASK)(lwip_task)), INT_PRIORITY(LWIP_DEFAULT_PRIORITY), ROUND_STK_T(LWIP_STACK_SIZE), _kernel_stack_LWIP_TASK_2, (TA_NULL), (NULL) },
	{ (TA_NULL), (intptr_t)(2), ((TASK)(lwip_task)), INT_PRIORITY(LWIP_DEFAULT_PRIORITY), ROUND_STK_T(LWIP_STACK_SIZE), _kernel_stack_LWIP_TASK_3, (TA_NULL), (NULL) },
	{ (TA_NULL), (intptr_t)(3), ((TASK)(lwip_task)), INT_PRIORITY(LWIP_DEFAULT_PRIORITY), ROUND_STK_T(LWIP_STACK_SIZE), _kernel_stack_LWIP_TASK_4, (TA_NULL), (NULL) },
	{ (TA_NULL), (intptr_t)(4), ((TASK)(lwip_task)), INT_PRIORITY(LWIP_DEFAULT_PRIORITY), ROUND_STK_T(LWIP_STACK_SIZE), _kernel_stack_LWIP_TASK_5, (TA_NULL), (NULL) },
	{ (TA_NULL), (intptr_t)(5), ((TASK)(lwip_task)), INT_PRIORITY(LWIP_DEFAULT_PRIORITY), ROUND_STK_T(LWIP_STACK_SIZE), _kernel_stack_LWIP_TASK_6, (TA_NULL), (NULL) },
	{ (TA_NULL), (intptr_t)(6), ((TASK)(lwip_task)), INT_PRIORITY(LWIP_DEFAULT_PRIORITY), ROUND_STK_T(LWIP_STACK_SIZE), _kernel_stack_LWIP_TASK_7, (TA_NULL), (NULL) },
	{ (TA_NULL), (intptr_t)(7), ((TASK)(lwip_task)), INT_PRIORITY(LWIP_DEFAULT_PRIORITY), ROUND_STK_T(LWIP_STACK_SIZE), _kernel_stack_LWIP_TASK_8, (TA_NULL), (NULL) },
	{ (TA_NULL), (intptr_t)(8), ((TASK)(lwip_task)), INT_PRIORITY(LWIP_DEFAULT_PRIORITY), ROUND_STK_T(LWIP_STACK_SIZE), _kernel_stack_LWIP_TASK_N, (TA_NULL), (NULL) },
	{ (TA_ACT), (intptr_t)(0), ((TASK)(pinkit_task)), INT_PRIORITY(PINKIT_PRIORITY), ROUND_STK_T(PINKIT_STACK_SIZE), _kernel_stack_PINKIT_TASK, (TA_NULL), (NULL) },
	{ (TA_ACT), (intptr_t)(1), ((TASK)(main_task)), INT_PRIORITY(MAIN_PRIORITY), ROUND_STK_T(STACK_SIZE), _kernel_stack_MAIN_TASK1, (TA_NULL), (NULL) }
};

TOPPERS_EMPTY_LABEL(TINIB, _kernel_atinib_table);

TCB _kernel_tcb_table[TNUM_TSKID];

const ID _kernel_torder_table[TNUM_STSKID] = {
	LOGTASK, SDM_TASK, MONTASK, LWIP_TASK_1, LWIP_TASK_2, LWIP_TASK_3, LWIP_TASK_4, LWIP_TASK_5, LWIP_TASK_6, LWIP_TASK_7, LWIP_TASK_8, LWIP_TASK_N, PINKIT_TASK, MAIN_TASK1
};

/*
 *  Semaphore Functions
 */

#define TNUM_SSEMID	62

const ID _kernel_tmax_semid = (TMIN_SEMID + TNUM_SEMID - 1);
const ID _kernel_tmax_ssemid = (TMIN_SEMID + TNUM_SSEMID - 1);

const SEMINIB _kernel_seminib_table[TNUM_SSEMID] = {
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_NULL), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (0), (1) },
	{ (TA_TPRI), (1), (1) },
	{ (TA_TPRI), (1), (1) }
};

TOPPERS_EMPTY_LABEL(SEMINIB, _kernel_aseminib_table);

SEMCB _kernel_semcb_table[TNUM_SEMID];

/*
 *  Eventflag Functions
 */

#define TNUM_SFLGID	1

const ID _kernel_tmax_flgid = (TMIN_FLGID + TNUM_FLGID - 1);
const ID _kernel_tmax_sflgid = (TMIN_FLGID + TNUM_SFLGID - 1);

const FLGINIB _kernel_flginib_table[TNUM_SFLGID] = {
	{ ((TA_TFIFO|TA_WSGL)), (0) }
};

TOPPERS_EMPTY_LABEL(FLGINIB, _kernel_aflginib_table);

FLGCB _kernel_flgcb_table[TNUM_FLGID];

/*
 *  Dataqueue Functions
 */

#define TNUM_SDTQID	0

const ID _kernel_tmax_dtqid = (TMIN_DTQID + TNUM_DTQID - 1);
const ID _kernel_tmax_sdtqid = (TMIN_DTQID + TNUM_SDTQID - 1);

TOPPERS_EMPTY_LABEL(const DTQINIB, _kernel_dtqinib_table);

TOPPERS_EMPTY_LABEL(DTQINIB, _kernel_adtqinib_table);

TOPPERS_EMPTY_LABEL(DTQCB, _kernel_dtqcb_table);

/*
 *  Priority Dataqueue Functions
 */

#define TNUM_SPDQID	0

const ID _kernel_tmax_pdqid = (TMIN_PDQID + TNUM_PDQID - 1);
const ID _kernel_tmax_spdqid = (TMIN_PDQID + TNUM_SPDQID - 1);

TOPPERS_EMPTY_LABEL(const PDQINIB, _kernel_pdqinib_table);

TOPPERS_EMPTY_LABEL(PDQINIB, _kernel_apdqinib_table);

TOPPERS_EMPTY_LABEL(PDQCB, _kernel_pdqcb_table);

/*
 *  Mailbox Functions
 */

#define TNUM_SMBXID	0

const ID _kernel_tmax_mbxid = (TMIN_MBXID + TNUM_MBXID - 1);
const ID _kernel_tmax_smbxid = (TMIN_MBXID + TNUM_SMBXID - 1);

TOPPERS_EMPTY_LABEL(const MBXINIB, _kernel_mbxinib_table);

TOPPERS_EMPTY_LABEL(MBXINIB, _kernel_ambxinib_table);

TOPPERS_EMPTY_LABEL(MBXCB, _kernel_mbxcb_table);

/*
 *  Mutex Functions
 */

#define TNUM_SMTXID	0

const ID _kernel_tmax_mtxid = (TMIN_MTXID + TNUM_MTXID - 1);
const ID _kernel_tmax_smtxid = (TMIN_MTXID + TNUM_SMTXID - 1);

TOPPERS_EMPTY_LABEL(const MTXINIB, _kernel_mtxinib_table);

TOPPERS_EMPTY_LABEL(MTXINIB, _kernel_amtxinib_table);

TOPPERS_EMPTY_LABEL(MTXCB, _kernel_mtxcb_table);

/*
 *  Fixed-sized Memorypool Functions
 */

#define TNUM_SMPFID	0

const ID _kernel_tmax_mpfid = (TMIN_MPFID + TNUM_MPFID - 1);
const ID _kernel_tmax_smpfid = (TMIN_MPFID + TNUM_SMPFID - 1);

TOPPERS_EMPTY_LABEL(const MPFINIB, _kernel_mpfinib_table);

TOPPERS_EMPTY_LABEL(MPFINIB, _kernel_ampfinib_table);

TOPPERS_EMPTY_LABEL(MPFCB, _kernel_mpfcb_table);

/*
 *  Cyclic Handler Functions
 */

#define TNUM_SCYCID	0

const ID _kernel_tmax_cycid = (TMIN_CYCID + TNUM_CYCID - 1);
const ID _kernel_tmax_scycid = (TMIN_CYCID + TNUM_SCYCID - 1);

TOPPERS_EMPTY_LABEL(const CYCINIB, _kernel_cycinib_table);

TOPPERS_EMPTY_LABEL(CYCINIB, _kernel_acycinib_table);

TOPPERS_EMPTY_LABEL(CYCCB, _kernel_cyccb_table);

/*
 *  Alarm Handler Functions
 */

#define TNUM_SALMID	0

const ID _kernel_tmax_almid = (TMIN_ALMID + TNUM_ALMID - 1);
const ID _kernel_tmax_salmid = (TMIN_ALMID + TNUM_SALMID - 1);

TOPPERS_EMPTY_LABEL(const ALMINIB, _kernel_alminib_table);

TOPPERS_EMPTY_LABEL(ALMINIB, _kernel_aalminib_table);

TOPPERS_EMPTY_LABEL(ALMCB, _kernel_almcb_table);

/*
 *  Interrupt Management Functions
 */

const uint_t _kernel_tnum_isr_queue = 8;

const ISR_ENTRY _kernel_isr_queue_list[8] = {
	{ 47, &(_kernel_isr_queue_table[0]) },
	{ 48, &(_kernel_isr_queue_table[1]) },
	{ 55, &(_kernel_isr_queue_table[2]) },
	{ 72, &(_kernel_isr_queue_table[3]) },
	{ 75, &(_kernel_isr_queue_table[4]) },
	{ 77, &(_kernel_isr_queue_table[5]) },
	{ 85, &(_kernel_isr_queue_table[6]) },
	{ 87, &(_kernel_isr_queue_table[7]) }
};

QUEUE _kernel_isr_queue_table[8];

void
_kernel_inthdr_47(void)
{
	i_begin_int(47);
	_kernel_call_isr(&(_kernel_isr_queue_table[0]));
	i_end_int(47);
}

void
_kernel_inthdr_48(void)
{
	i_begin_int(48);
	_kernel_call_isr(&(_kernel_isr_queue_table[1]));
	i_end_int(48);
}

void
_kernel_inthdr_55(void)
{
	i_begin_int(55);
	_kernel_call_isr(&(_kernel_isr_queue_table[2]));
	i_end_int(55);
}

void
_kernel_inthdr_72(void)
{
	i_begin_int(72);
	_kernel_call_isr(&(_kernel_isr_queue_table[3]));
	i_end_int(72);
}

void
_kernel_inthdr_75(void)
{
	i_begin_int(75);
	_kernel_call_isr(&(_kernel_isr_queue_table[4]));
	i_end_int(75);
}

void
_kernel_inthdr_77(void)
{
	i_begin_int(77);
	_kernel_call_isr(&(_kernel_isr_queue_table[5]));
	i_end_int(77);
}

void
_kernel_inthdr_85(void)
{
	i_begin_int(85);
	_kernel_call_isr(&(_kernel_isr_queue_table[6]));
	i_end_int(85);
}

void
_kernel_inthdr_87(void)
{
	i_begin_int(87);
	_kernel_call_isr(&(_kernel_isr_queue_table[7]));
	i_end_int(87);
}

#define TNUM_SISR	8
#define TNUM_ISR	8

const ID _kernel_tmax_isrid = (TMIN_ISRID + TNUM_ISRID - 1);
const uint_t _kernel_tnum_sisr = TNUM_SISR;

const ISRINIB _kernel_sisrinib_table[TNUM_SISR] = {
	{ (TA_NULL), (intptr_t)(1), (INTNO_SIO1), (&(_kernel_isr_queue_table[2])), (sio_usart_isr), (1) },
	{ (TA_NULL), (intptr_t)(2), (INTNO_SIO2), (&(_kernel_isr_queue_table[7])), (sio_usart_isr), (1) },
	{ (TA_NULL), (intptr_t)(DMARX_SID), (INTNO_DMARX), (&(_kernel_isr_queue_table[4])), (stream_dma_isr), (1) },
	{ (TA_NULL), (intptr_t)(DMATX_SID), (INTNO_DMATX), (&(_kernel_isr_queue_table[6])), (stream_dma_isr), (1) },
	{ (TA_NULL), (intptr_t)(I2C_PORTID), (INTNO_I2CEV), (&(_kernel_isr_queue_table[0])), (i2c_ev_isr), (1) },
	{ (TA_NULL), (intptr_t)(I2C_PORTID), (INTNO_I2CER), (&(_kernel_isr_queue_table[1])), (i2c_er_isr), (1) },
	{ (TA_NULL), (intptr_t)(DMA2STM0_SID), (INTNO_DMAADC), (&(_kernel_isr_queue_table[3])), (stream_dma_isr), (1) },
	{ (TA_NULL), (intptr_t)(EMAC_PORTID), (INTNO_EMAC), (&(_kernel_isr_queue_table[5])), (emac_hps_isr), (1) }
};

TOPPERS_EMPTY_LABEL(ISRINIB, _kernel_aisrinib_table);

ISRCB _kernel_isrcb_table[TNUM_ISR];

#define TNUM_INHNO	16
const uint_t _kernel_tnum_inhno = TNUM_INHNO;

INTHDR_ENTRY(INHNO_TIMER, 15, target_timer_handler)
INTHDR_ENTRY(INHNO_DMA2D, 106, dma2d_handler)
INTHDR_ENTRY(INHNO_ALARM, 57, rtc_handler)
INTHDR_ENTRY(INHNO_RTCWUP, 19, wakeup_handler)
INTHDR_ENTRY(INHNO_EXTI9, 39, exti9_handler)
INTHDR_ENTRY(INHNO_EXTI15, 56, exti15_handler)
INTHDR_ENTRY(INHNO_SDMMC, 65, sdmmc_handler)
INTHDR_ENTRY(INHNO_ADC, 34, adc_int_handler)
INTHDR_ENTRY(47, 47, _kernel_inthdr_47)
INTHDR_ENTRY(48, 48, _kernel_inthdr_48)
INTHDR_ENTRY(55, 55, _kernel_inthdr_55)
INTHDR_ENTRY(72, 72, _kernel_inthdr_72)
INTHDR_ENTRY(75, 75, _kernel_inthdr_75)
INTHDR_ENTRY(77, 77, _kernel_inthdr_77)
INTHDR_ENTRY(85, 85, _kernel_inthdr_85)
INTHDR_ENTRY(87, 87, _kernel_inthdr_87)

const INHINIB _kernel_inhinib_table[TNUM_INHNO] = {
	{ (INHNO_TIMER), (TA_NULL), (FP)(INT_ENTRY(INHNO_TIMER, target_timer_handler)) },
	{ (INHNO_DMA2D), (TA_NULL), (FP)(INT_ENTRY(INHNO_DMA2D, dma2d_handler)) },
	{ (INHNO_ALARM), (TA_NULL), (FP)(INT_ENTRY(INHNO_ALARM, rtc_handler)) },
	{ (INHNO_RTCWUP), (TA_NULL), (FP)(INT_ENTRY(INHNO_RTCWUP, wakeup_handler)) },
	{ (INHNO_EXTI9), (TA_NULL), (FP)(INT_ENTRY(INHNO_EXTI9, exti9_handler)) },
	{ (INHNO_EXTI15), (TA_NULL), (FP)(INT_ENTRY(INHNO_EXTI15, exti15_handler)) },
	{ (INHNO_SDMMC), (TA_NULL), (FP)(INT_ENTRY(INHNO_SDMMC, sdmmc_handler)) },
	{ (INHNO_ADC), (TA_NULL), (FP)(INT_ENTRY(INHNO_ADC, adc_int_handler)) },
	{ (47), (TA_NULL), (FP)(INT_ENTRY(47, _kernel_inthdr_47)) },
	{ (48), (TA_NULL), (FP)(INT_ENTRY(48, _kernel_inthdr_48)) },
	{ (55), (TA_NULL), (FP)(INT_ENTRY(55, _kernel_inthdr_55)) },
	{ (72), (TA_NULL), (FP)(INT_ENTRY(72, _kernel_inthdr_72)) },
	{ (75), (TA_NULL), (FP)(INT_ENTRY(75, _kernel_inthdr_75)) },
	{ (77), (TA_NULL), (FP)(INT_ENTRY(77, _kernel_inthdr_77)) },
	{ (85), (TA_NULL), (FP)(INT_ENTRY(85, _kernel_inthdr_85)) },
	{ (87), (TA_NULL), (FP)(INT_ENTRY(87, _kernel_inthdr_87)) }
};

#define TNUM_INTNO	16
const uint_t _kernel_tnum_intno = TNUM_INTNO;

const INTINIB _kernel_intinib_table[TNUM_INTNO] = {
	{ (INTNO_TIMER), (TA_ENAINT|INTATR_TIMER), (INTPRI_TIMER) },
	{ (INTNO_SIO1), (TA_ENAINT|INTATR_SIO), (INTPRI_SIO) },
	{ (INTNO_SIO2), (TA_ENAINT|INTATR_SIO), (INTPRI_SIO) },
	{ (INTNO_DMA2D), (TA_ENAINT | INTATR_DMA2D), (INTPRI_DMA2D) },
	{ (INTNO_ALARM), (TA_ENAINT | INTATR_ALARM), (INTPRI_ALARM) },
	{ (INTNO_RTCWUP), (TA_ENAINT | INTATR_RTCWUP), (INTPRI_RTCWUP) },
	{ (INTNO_EXTI9), (TA_ENAINT | INTATR_EXTI9), (INTPRI_EXTI9) },
	{ (INTNO_EXTI15), (TA_ENAINT | INTATR_EXTI15), (INTPRI_EXTI15) },
	{ (INTNO_SDMMC), (TA_ENAINT | INTATR_SDMMC), (INTPRI_SDMMC) },
	{ (INTNO_DMARX), (TA_ENAINT | INTATR_DMARX), (INTPRI_DMARX) },
	{ (INTNO_DMATX), (TA_ENAINT | INTATR_DMATX), (INTPRI_DMATX) },
	{ (INTNO_I2CEV), (TA_ENAINT | INTATR_I2CEV), (INTPRI_I2CEV) },
	{ (INTNO_I2CER), (TA_ENAINT | INTATR_I2CER), (INTPRI_I2CER) },
	{ (INTNO_DMAADC), (TA_ENAINT | INTATR_DMAADC), (INTPRI_DMAADC) },
	{ (INTNO_ADC), (TA_ENAINT | INTATR_ADC), (INTPRI_ADC) },
	{ (INTNO_EMAC), (TA_NULL | INTATR_EMAC), (INTPRI_EMAC) }
};

/*
 *  CPU Exception Management Functions
 */

#define TNUM_EXCNO	0
const uint_t _kernel_tnum_excno = TNUM_EXCNO;


TOPPERS_EMPTY_LABEL(const EXCINIB, _kernel_excinib_table);

/*
 *  Stack Area for Non-task Context
 */

#ifdef DEFAULT_ISTK

#define TOPPERS_ISTKSZ		DEFAULT_ISTKSZ
#define TOPPERS_ISTK		DEFAULT_ISTK

#else /* DEAULT_ISTK */

static STK_T _kernel_istack[COUNT_STK_T(DEFAULT_ISTKSZ)];

#define TOPPERS_ISTKSZ		ROUND_STK_T(DEFAULT_ISTKSZ)
#define TOPPERS_ISTK		_kernel_istack

#endif /* DEAULT_ISTK */

const SIZE		_kernel_istksz = TOPPERS_ISTKSZ;
STK_T *const	_kernel_istk = TOPPERS_ISTK;

#ifdef TOPPERS_ISTKPT
STK_T *const	_kernel_istkpt = TOPPERS_ISTKPT(TOPPERS_ISTK, TOPPERS_ISTKSZ);
#endif /* TOPPERS_ISTKPT */

/*
 *  Memory Area Allocated by Kernel
 */

#define TOPPERS_KMMSZ		0
#define TOPPERS_KMM			NULL

const SIZE		_kernel_kmmsz = TOPPERS_KMMSZ;
MB_T *const		_kernel_kmm = TOPPERS_KMM;

/*
 *  Time Event Management
 */

TMEVTN   _kernel_tmevt_heap[TNUM_TSKID + TNUM_CYCID + TNUM_ALMID];

/*
 *  Module Initialization Function
 */

void
_kernel_initialize_object(void)
{
	_kernel_initialize_task();
	_kernel_initialize_semaphore();
	_kernel_initialize_eventflag();
	_kernel_initialize_interrupt();
	_kernel_initialize_isr();
	_kernel_initialize_exception();
}

/*
 *  Initialization Routine
 */

void
_kernel_call_inirtn(void)
{
	((INIRTN)(target_timer_initialize))((intptr_t)(0));
	((INIRTN)(syslog_initialize))((intptr_t)(0));
	((INIRTN)(print_banner))((intptr_t)(0));
	((INIRTN)(sio_initialize))((intptr_t)(0));
	((INIRTN)(serial_initialize))((intptr_t)(0));
	((INIRTN)(led_init))((intptr_t)(0));
	((INIRTN)(switch_push_init))((intptr_t)(0));
	((INIRTN)(rtc_init))((intptr_t)(0));
	((INIRTN)(rtc_info_init))((intptr_t)(0));
	((INIRTN)(sdmmc_init))((intptr_t)(0));
	((INIRTN)(sdev_init))((intptr_t)(0));
	((INIRTN)(stdfile_init))((intptr_t)(0));
	((INIRTN)(volume_info_init))((intptr_t)(0));
	((INIRTN)(sd_init))((intptr_t)(0));
	((INIRTN)(setup_sw_func))((intptr_t)(sw_int));
	((INIRTN)(device_info_init))((intptr_t)(0));
}

/*
 *  Termination Routine
 */

void
_kernel_call_terrtn(void)
{
	((TERRTN)(logtask_terminate))((intptr_t)(0));
	((TERRTN)(target_timer_terminate))((intptr_t)(0));
}

/*
 *  Target-dependent Definitions (ARM-M)
 */



__attribute__ ((section(".vector"))) 
const FP _kernel_vector_table[] =      
{                                    
	(FP)(TOPPERS_ISTKPT(TOPPERS_ISTK, TOPPERS_ISTKSZ)), /* 0 The initial stack pointer */
	(FP)_kernel__start,                    /* 1 The reset handler */
 	(FP)(_kernel_core_exc_entry), /* 2 */
 	(FP)(_kernel_core_exc_entry), /* 3 */
 	(FP)(_kernel_core_exc_entry), /* 4 */
 	(FP)(_kernel_core_exc_entry), /* 5 */
 	(FP)(_kernel_core_exc_entry), /* 6 */
 	(FP)(_kernel_core_exc_entry), /* 7 */
 	(FP)(_kernel_core_exc_entry), /* 8 */
 	(FP)(_kernel_core_exc_entry), /* 9 */
 	(FP)(_kernel_core_exc_entry), /* 10 */
 	(FP)(_kernel_svc_handler),        /* 11 SVCall handler */
 	(FP)(_kernel_core_exc_entry), /* 12 */
 	(FP)(_kernel_core_exc_entry), /* 13 */
 	(FP)(_kernel_core_exc_entry), /* 14 */
 	(FP)(_kernel_core_int_entry), /* 15 */
 	(FP)(_kernel_core_int_entry), /* 16 */
 	(FP)(_kernel_core_int_entry), /* 17 */
 	(FP)(_kernel_core_int_entry), /* 18 */
 	(FP)(_kernel_core_int_entry), /* 19 */
 	(FP)(_kernel_core_int_entry), /* 20 */
 	(FP)(_kernel_core_int_entry), /* 21 */
 	(FP)(_kernel_core_int_entry), /* 22 */
 	(FP)(_kernel_core_int_entry), /* 23 */
 	(FP)(_kernel_core_int_entry), /* 24 */
 	(FP)(_kernel_core_int_entry), /* 25 */
 	(FP)(_kernel_core_int_entry), /* 26 */
 	(FP)(_kernel_core_int_entry), /* 27 */
 	(FP)(_kernel_core_int_entry), /* 28 */
 	(FP)(_kernel_core_int_entry), /* 29 */
 	(FP)(_kernel_core_int_entry), /* 30 */
 	(FP)(_kernel_core_int_entry), /* 31 */
 	(FP)(_kernel_core_int_entry), /* 32 */
 	(FP)(_kernel_core_int_entry), /* 33 */
 	(FP)(_kernel_core_int_entry), /* 34 */
 	(FP)(_kernel_core_int_entry), /* 35 */
 	(FP)(_kernel_core_int_entry), /* 36 */
 	(FP)(_kernel_core_int_entry), /* 37 */
 	(FP)(_kernel_core_int_entry), /* 38 */
 	(FP)(_kernel_core_int_entry), /* 39 */
 	(FP)(_kernel_core_int_entry), /* 40 */
 	(FP)(_kernel_core_int_entry), /* 41 */
 	(FP)(_kernel_core_int_entry), /* 42 */
 	(FP)(_kernel_core_int_entry), /* 43 */
 	(FP)(_kernel_core_int_entry), /* 44 */
 	(FP)(_kernel_core_int_entry), /* 45 */
 	(FP)(_kernel_core_int_entry), /* 46 */
 	(FP)(_kernel_core_int_entry), /* 47 */
 	(FP)(_kernel_core_int_entry), /* 48 */
 	(FP)(_kernel_core_int_entry), /* 49 */
 	(FP)(_kernel_core_int_entry), /* 50 */
 	(FP)(_kernel_core_int_entry), /* 51 */
 	(FP)(_kernel_core_int_entry), /* 52 */
 	(FP)(_kernel_core_int_entry), /* 53 */
 	(FP)(_kernel_core_int_entry), /* 54 */
 	(FP)(_kernel_core_int_entry), /* 55 */
 	(FP)(_kernel_core_int_entry), /* 56 */
 	(FP)(_kernel_core_int_entry), /* 57 */
 	(FP)(_kernel_core_int_entry), /* 58 */
 	(FP)(_kernel_core_int_entry), /* 59 */
 	(FP)(_kernel_core_int_entry), /* 60 */
 	(FP)(_kernel_core_int_entry), /* 61 */
 	(FP)(_kernel_core_int_entry), /* 62 */
 	(FP)(_kernel_core_int_entry), /* 63 */
 	(FP)(_kernel_core_int_entry), /* 64 */
 	(FP)(_kernel_core_int_entry), /* 65 */
 	(FP)(_kernel_core_int_entry), /* 66 */
 	(FP)(_kernel_core_int_entry), /* 67 */
 	(FP)(_kernel_core_int_entry), /* 68 */
 	(FP)(_kernel_core_int_entry), /* 69 */
 	(FP)(_kernel_core_int_entry), /* 70 */
 	(FP)(_kernel_core_int_entry), /* 71 */
 	(FP)(_kernel_core_int_entry), /* 72 */
 	(FP)(_kernel_core_int_entry), /* 73 */
 	(FP)(_kernel_core_int_entry), /* 74 */
 	(FP)(_kernel_core_int_entry), /* 75 */
 	(FP)(_kernel_core_int_entry), /* 76 */
 	(FP)(_kernel_core_int_entry), /* 77 */
 	(FP)(_kernel_core_int_entry), /* 78 */
 	(FP)(_kernel_core_int_entry), /* 79 */
 	(FP)(_kernel_core_int_entry), /* 80 */
 	(FP)(_kernel_core_int_entry), /* 81 */
 	(FP)(_kernel_core_int_entry), /* 82 */
 	(FP)(_kernel_core_int_entry), /* 83 */
 	(FP)(_kernel_core_int_entry), /* 84 */
 	(FP)(_kernel_core_int_entry), /* 85 */
 	(FP)(_kernel_core_int_entry), /* 86 */
 	(FP)(_kernel_core_int_entry), /* 87 */
 	(FP)(_kernel_core_int_entry), /* 88 */
 	(FP)(_kernel_core_int_entry), /* 89 */
 	(FP)(_kernel_core_int_entry), /* 90 */
 	(FP)(_kernel_core_int_entry), /* 91 */
 	(FP)(_kernel_core_int_entry), /* 92 */
 	(FP)(_kernel_core_int_entry), /* 93 */
 	(FP)(_kernel_core_int_entry), /* 94 */
 	(FP)(_kernel_core_int_entry), /* 95 */
 	(FP)(_kernel_core_int_entry), /* 96 */
 	(FP)(_kernel_core_int_entry), /* 97 */
 	(FP)(_kernel_core_int_entry), /* 98 */
 	(FP)(_kernel_core_int_entry), /* 99 */
 	(FP)(_kernel_core_int_entry), /* 100 */
 	(FP)(_kernel_core_int_entry), /* 101 */
 	(FP)(_kernel_core_int_entry), /* 102 */
 	(FP)(_kernel_core_int_entry), /* 103 */
 	(FP)(_kernel_core_int_entry), /* 104 */
 	(FP)(_kernel_core_int_entry), /* 105 */
 	(FP)(_kernel_core_int_entry), /* 106 */
 	(FP)(_kernel_core_int_entry), /* 107 */
 	(FP)(_kernel_core_int_entry), /* 108 */
 	(FP)(_kernel_core_int_entry), /* 109 */
 	(FP)(_kernel_core_int_entry), /* 110 */
 	(FP)(_kernel_core_int_entry), /* 111 */
 	(FP)(_kernel_core_int_entry), /* 112 */
 	(FP)(_kernel_core_int_entry), /* 113 */
 	(FP)(_kernel_core_int_entry), /* 114 */
 	(FP)(_kernel_core_int_entry), /* 115 */
 	(FP)(_kernel_core_int_entry), /* 116 */
 	(FP)(_kernel_core_int_entry), /* 117 */
 	(FP)(_kernel_core_int_entry), /* 118 */
 	(FP)(_kernel_core_int_entry), /* 119 */
 	(FP)(_kernel_core_int_entry), /* 120 */
 	(FP)(_kernel_core_int_entry), /* 121 */
 	(FP)(_kernel_core_int_entry), /* 122 */
 	(FP)(_kernel_core_int_entry), /* 123 */
 	(FP)(_kernel_core_int_entry), /* 124 */
 	(FP)(_kernel_core_int_entry), /* 125 */

};


const FP _kernel_exc_tbl[] = 
{
	(FP)(_kernel_default_exc_handler), /* 0 */
	(FP)(_kernel_default_exc_handler), /* 1 */
	(FP)(_kernel_default_exc_handler), /* 2 */
	(FP)(_kernel_default_exc_handler), /* 3 */
	(FP)(_kernel_default_exc_handler), /* 4 */
	(FP)(_kernel_default_exc_handler), /* 5 */
	(FP)(_kernel_default_exc_handler), /* 6 */
	(FP)(_kernel_default_exc_handler), /* 7 */
	(FP)(_kernel_default_exc_handler), /* 8 */
	(FP)(_kernel_default_exc_handler), /* 9 */
	(FP)(_kernel_default_exc_handler), /* 10 */
	(FP)(_kernel_default_exc_handler), /* 11 */
	(FP)(_kernel_default_exc_handler), /* 12 */
	(FP)(_kernel_default_exc_handler), /* 13 */
	(FP)(_kernel_default_exc_handler), /* 14 */
	(FP)(target_timer_handler), /* 15 */
	(FP)(_kernel_default_int_handler), /* 16 */
	(FP)(_kernel_default_int_handler), /* 17 */
	(FP)(_kernel_default_int_handler), /* 18 */
	(FP)(wakeup_handler), /* 19 */
	(FP)(_kernel_default_int_handler), /* 20 */
	(FP)(_kernel_default_int_handler), /* 21 */
	(FP)(_kernel_default_int_handler), /* 22 */
	(FP)(_kernel_default_int_handler), /* 23 */
	(FP)(_kernel_default_int_handler), /* 24 */
	(FP)(_kernel_default_int_handler), /* 25 */
	(FP)(_kernel_default_int_handler), /* 26 */
	(FP)(_kernel_default_int_handler), /* 27 */
	(FP)(_kernel_default_int_handler), /* 28 */
	(FP)(_kernel_default_int_handler), /* 29 */
	(FP)(_kernel_default_int_handler), /* 30 */
	(FP)(_kernel_default_int_handler), /* 31 */
	(FP)(_kernel_default_int_handler), /* 32 */
	(FP)(_kernel_default_int_handler), /* 33 */
	(FP)(adc_int_handler), /* 34 */
	(FP)(_kernel_default_int_handler), /* 35 */
	(FP)(_kernel_default_int_handler), /* 36 */
	(FP)(_kernel_default_int_handler), /* 37 */
	(FP)(_kernel_default_int_handler), /* 38 */
	(FP)(exti9_handler), /* 39 */
	(FP)(_kernel_default_int_handler), /* 40 */
	(FP)(_kernel_default_int_handler), /* 41 */
	(FP)(_kernel_default_int_handler), /* 42 */
	(FP)(_kernel_default_int_handler), /* 43 */
	(FP)(_kernel_default_int_handler), /* 44 */
	(FP)(_kernel_default_int_handler), /* 45 */
	(FP)(_kernel_default_int_handler), /* 46 */
	(FP)(_kernel_inthdr_47), /* 47 */
	(FP)(_kernel_inthdr_48), /* 48 */
	(FP)(_kernel_default_int_handler), /* 49 */
	(FP)(_kernel_default_int_handler), /* 50 */
	(FP)(_kernel_default_int_handler), /* 51 */
	(FP)(_kernel_default_int_handler), /* 52 */
	(FP)(_kernel_default_int_handler), /* 53 */
	(FP)(_kernel_default_int_handler), /* 54 */
	(FP)(_kernel_inthdr_55), /* 55 */
	(FP)(exti15_handler), /* 56 */
	(FP)(rtc_handler), /* 57 */
	(FP)(_kernel_default_int_handler), /* 58 */
	(FP)(_kernel_default_int_handler), /* 59 */
	(FP)(_kernel_default_int_handler), /* 60 */
	(FP)(_kernel_default_int_handler), /* 61 */
	(FP)(_kernel_default_int_handler), /* 62 */
	(FP)(_kernel_default_int_handler), /* 63 */
	(FP)(_kernel_default_int_handler), /* 64 */
	(FP)(sdmmc_handler), /* 65 */
	(FP)(_kernel_default_int_handler), /* 66 */
	(FP)(_kernel_default_int_handler), /* 67 */
	(FP)(_kernel_default_int_handler), /* 68 */
	(FP)(_kernel_default_int_handler), /* 69 */
	(FP)(_kernel_default_int_handler), /* 70 */
	(FP)(_kernel_default_int_handler), /* 71 */
	(FP)(_kernel_inthdr_72), /* 72 */
	(FP)(_kernel_default_int_handler), /* 73 */
	(FP)(_kernel_default_int_handler), /* 74 */
	(FP)(_kernel_inthdr_75), /* 75 */
	(FP)(_kernel_default_int_handler), /* 76 */
	(FP)(_kernel_inthdr_77), /* 77 */
	(FP)(_kernel_default_int_handler), /* 78 */
	(FP)(_kernel_default_int_handler), /* 79 */
	(FP)(_kernel_default_int_handler), /* 80 */
	(FP)(_kernel_default_int_handler), /* 81 */
	(FP)(_kernel_default_int_handler), /* 82 */
	(FP)(_kernel_default_int_handler), /* 83 */
	(FP)(_kernel_default_int_handler), /* 84 */
	(FP)(_kernel_inthdr_85), /* 85 */
	(FP)(_kernel_default_int_handler), /* 86 */
	(FP)(_kernel_inthdr_87), /* 87 */
	(FP)(_kernel_default_int_handler), /* 88 */
	(FP)(_kernel_default_int_handler), /* 89 */
	(FP)(_kernel_default_int_handler), /* 90 */
	(FP)(_kernel_default_int_handler), /* 91 */
	(FP)(_kernel_default_int_handler), /* 92 */
	(FP)(_kernel_default_int_handler), /* 93 */
	(FP)(_kernel_default_int_handler), /* 94 */
	(FP)(_kernel_default_int_handler), /* 95 */
	(FP)(_kernel_default_int_handler), /* 96 */
	(FP)(_kernel_default_int_handler), /* 97 */
	(FP)(_kernel_default_int_handler), /* 98 */
	(FP)(_kernel_default_int_handler), /* 99 */
	(FP)(_kernel_default_int_handler), /* 100 */
	(FP)(_kernel_default_int_handler), /* 101 */
	(FP)(_kernel_default_int_handler), /* 102 */
	(FP)(_kernel_default_int_handler), /* 103 */
	(FP)(_kernel_default_int_handler), /* 104 */
	(FP)(_kernel_default_int_handler), /* 105 */
	(FP)(dma2d_handler), /* 106 */
	(FP)(_kernel_default_int_handler), /* 107 */
	(FP)(_kernel_default_int_handler), /* 108 */
	(FP)(_kernel_default_int_handler), /* 109 */
	(FP)(_kernel_default_int_handler), /* 110 */
	(FP)(_kernel_default_int_handler), /* 111 */
	(FP)(_kernel_default_int_handler), /* 112 */
	(FP)(_kernel_default_int_handler), /* 113 */
	(FP)(_kernel_default_int_handler), /* 114 */
	(FP)(_kernel_default_int_handler), /* 115 */
	(FP)(_kernel_default_int_handler), /* 116 */
	(FP)(_kernel_default_int_handler), /* 117 */
	(FP)(_kernel_default_int_handler), /* 118 */
	(FP)(_kernel_default_int_handler), /* 119 */
	(FP)(_kernel_default_int_handler), /* 120 */
	(FP)(_kernel_default_int_handler), /* 121 */
	(FP)(_kernel_default_int_handler), /* 122 */
	(FP)(_kernel_default_int_handler), /* 123 */
	(FP)(_kernel_default_int_handler), /* 124 */
	(FP)(_kernel_default_int_handler), /* 125 */

};

const uint32_t _kernel_bitpat_cfgint[8] = {
	UINT32_C(0x00088000), 
	UINT32_C(0x03818084), 
	UINT32_C(0x00a02902), 
	UINT32_C(0x00000400), 
	UINT32_C(0x00000000), 
	UINT32_C(0x00000000), 
	UINT32_C(0x00000000), 
	UINT32_C(0x00000000), 

};

const uint32_t _kernel_int_iipm_tbl[] = {
	UINT32_C(0x00000000), /* 0x000 */
	UINT32_C(0x00000000), /* 0x001 */
	UINT32_C(0x00000000), /* 0x002 */
	UINT32_C(0x00000000), /* 0x003 */
	UINT32_C(0x00000000), /* 0x004 */
	UINT32_C(0x00000000), /* 0x005 */
	UINT32_C(0x00000000), /* 0x006 */
	UINT32_C(0x00000000), /* 0x007 */
	UINT32_C(0x00000000), /* 0x008 */
	UINT32_C(0x00000000), /* 0x009 */
	UINT32_C(0x00000000), /* 0x00a */
	UINT32_C(0x00000000), /* 0x00b */
	UINT32_C(0x00000000), /* 0x00c */
	UINT32_C(0x00000000), /* 0x00d */
	UINT32_C(0x00000000), /* 0x00e */
	UINT32_C(0x000000a0), /* 0x00f */
	UINT32_C(0x00000000), /* 0x010 */
	UINT32_C(0x00000000), /* 0x011 */
	UINT32_C(0x00000000), /* 0x012 */
	UINT32_C(0x00000080), /* 0x013 */
	UINT32_C(0x00000000), /* 0x014 */
	UINT32_C(0x00000000), /* 0x015 */
	UINT32_C(0x00000000), /* 0x016 */
	UINT32_C(0x00000000), /* 0x017 */
	UINT32_C(0x00000000), /* 0x018 */
	UINT32_C(0x00000000), /* 0x019 */
	UINT32_C(0x00000000), /* 0x01a */
	UINT32_C(0x00000000), /* 0x01b */
	UINT32_C(0x00000000), /* 0x01c */
	UINT32_C(0x00000000), /* 0x01d */
	UINT32_C(0x00000000), /* 0x01e */
	UINT32_C(0x00000000), /* 0x01f */
	UINT32_C(0x00000000), /* 0x020 */
	UINT32_C(0x00000000), /* 0x021 */
	UINT32_C(0x000000b0), /* 0x022 */
	UINT32_C(0x00000000), /* 0x023 */
	UINT32_C(0x00000000), /* 0x024 */
	UINT32_C(0x00000000), /* 0x025 */
	UINT32_C(0x00000000), /* 0x026 */
	UINT32_C(0x000000c0), /* 0x027 */
	UINT32_C(0x00000000), /* 0x028 */
	UINT32_C(0x00000000), /* 0x029 */
	UINT32_C(0x00000000), /* 0x02a */
	UINT32_C(0x00000000), /* 0x02b */
	UINT32_C(0x00000000), /* 0x02c */
	UINT32_C(0x00000000), /* 0x02d */
	UINT32_C(0x00000000), /* 0x02e */
	UINT32_C(0x000000b0), /* 0x02f */
	UINT32_C(0x000000b0), /* 0x030 */
	UINT32_C(0x00000000), /* 0x031 */
	UINT32_C(0x00000000), /* 0x032 */
	UINT32_C(0x00000000), /* 0x033 */
	UINT32_C(0x00000000), /* 0x034 */
	UINT32_C(0x00000000), /* 0x035 */
	UINT32_C(0x00000000), /* 0x036 */
	UINT32_C(0x000000d0), /* 0x037 */
	UINT32_C(0x000000c0), /* 0x038 */
	UINT32_C(0x00000050), /* 0x039 */
	UINT32_C(0x00000000), /* 0x03a */
	UINT32_C(0x00000000), /* 0x03b */
	UINT32_C(0x00000000), /* 0x03c */
	UINT32_C(0x00000000), /* 0x03d */
	UINT32_C(0x00000000), /* 0x03e */
	UINT32_C(0x00000000), /* 0x03f */
	UINT32_C(0x00000000), /* 0x040 */
	UINT32_C(0x000000b0), /* 0x041 */
	UINT32_C(0x00000000), /* 0x042 */
	UINT32_C(0x00000000), /* 0x043 */
	UINT32_C(0x00000000), /* 0x044 */
	UINT32_C(0x00000000), /* 0x045 */
	UINT32_C(0x00000000), /* 0x046 */
	UINT32_C(0x00000000), /* 0x047 */
	UINT32_C(0x000000c0), /* 0x048 */
	UINT32_C(0x00000000), /* 0x049 */
	UINT32_C(0x00000000), /* 0x04a */
	UINT32_C(0x000000a0), /* 0x04b */
	UINT32_C(0x00000000), /* 0x04c */
	UINT32_C(0x000000b0), /* 0x04d */
	UINT32_C(0x00000000), /* 0x04e */
	UINT32_C(0x00000000), /* 0x04f */
	UINT32_C(0x00000000), /* 0x050 */
	UINT32_C(0x00000000), /* 0x051 */
	UINT32_C(0x00000000), /* 0x052 */
	UINT32_C(0x00000000), /* 0x053 */
	UINT32_C(0x00000000), /* 0x054 */
	UINT32_C(0x000000a0), /* 0x055 */
	UINT32_C(0x00000000), /* 0x056 */
	UINT32_C(0x000000d0), /* 0x057 */
	UINT32_C(0x00000000), /* 0x058 */
	UINT32_C(0x00000000), /* 0x059 */
	UINT32_C(0x00000000), /* 0x05a */
	UINT32_C(0x00000000), /* 0x05b */
	UINT32_C(0x00000000), /* 0x05c */
	UINT32_C(0x00000000), /* 0x05d */
	UINT32_C(0x00000000), /* 0x05e */
	UINT32_C(0x00000000), /* 0x05f */
	UINT32_C(0x00000000), /* 0x060 */
	UINT32_C(0x00000000), /* 0x061 */
	UINT32_C(0x00000000), /* 0x062 */
	UINT32_C(0x00000000), /* 0x063 */
	UINT32_C(0x00000000), /* 0x064 */
	UINT32_C(0x00000000), /* 0x065 */
	UINT32_C(0x00000000), /* 0x066 */
	UINT32_C(0x00000000), /* 0x067 */
	UINT32_C(0x00000000), /* 0x068 */
	UINT32_C(0x00000000), /* 0x069 */
	UINT32_C(0x00000090), /* 0x06a */
	UINT32_C(0x00000000), /* 0x06b */
	UINT32_C(0x00000000), /* 0x06c */
	UINT32_C(0x00000000), /* 0x06d */
	UINT32_C(0x00000000), /* 0x06e */
	UINT32_C(0x00000000), /* 0x06f */
	UINT32_C(0x00000000), /* 0x070 */
	UINT32_C(0x00000000), /* 0x071 */
	UINT32_C(0x00000000), /* 0x072 */
	UINT32_C(0x00000000), /* 0x073 */
	UINT32_C(0x00000000), /* 0x074 */
	UINT32_C(0x00000000), /* 0x075 */
	UINT32_C(0x00000000), /* 0x076 */
	UINT32_C(0x00000000), /* 0x077 */
	UINT32_C(0x00000000), /* 0x078 */
	UINT32_C(0x00000000), /* 0x079 */
	UINT32_C(0x00000000), /* 0x07a */
	UINT32_C(0x00000000), /* 0x07b */
	UINT32_C(0x00000000), /* 0x07c */
	UINT32_C(0x00000000), /* 0x07d */

};

