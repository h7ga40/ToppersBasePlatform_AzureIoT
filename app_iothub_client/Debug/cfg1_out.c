/* cfg1_out.c */
#define TOPPERS_CFG1_OUT  1
#include "kernel/kernel_int.h"
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


#ifdef INT64_MAX
  typedef int64_t signed_t;
  typedef uint64_t unsigned_t;
#else
  typedef int32_t signed_t;
  typedef uint32_t unsigned_t;
#endif

#include "target_cfg1_out.h"

const uint32_t TOPPERS_cfg_magic_number = 0x12345678;
const uint32_t TOPPERS_cfg_sizeof_signed_t = sizeof(signed_t);
const uint32_t TOPPERS_cfg_sizeof_pointer = sizeof(const volatile void*);
const unsigned_t TOPPERS_cfg_CHAR_BIT = ((unsigned char)~0u == 0xff ? 8 : 16);
const unsigned_t TOPPERS_cfg_CHAR_MAX = ((char)-1 < 0 ? (char)((unsigned char)~0u >> 1) : (unsigned char)~0u);
const unsigned_t TOPPERS_cfg_CHAR_MIN = (unsigned_t)((char)-1 < 0 ? -((unsigned char)~0u >> 1) - 1 : 0);
const unsigned_t TOPPERS_cfg_SCHAR_MAX = (signed char)((unsigned char)~0u >> 1);
const unsigned_t TOPPERS_cfg_SHRT_MAX = (short)((unsigned short)~0u >> 1);
const unsigned_t TOPPERS_cfg_INT_MAX = (int)(~0u >> 1);
const unsigned_t TOPPERS_cfg_LONG_MAX = (long)(~0ul >> 1);

const unsigned_t TOPPERS_cfg_SIL_ENDIAN_BIG = 
#if defined(SIL_ENDIAN_BIG)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_SIL_ENDIAN_LITTLE = 
#if defined(SIL_ENDIAN_LITTLE)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TA_NULL = ( unsigned_t )TA_NULL;
const unsigned_t TOPPERS_cfg_TA_ACT = ( unsigned_t )TA_ACT;
const unsigned_t TOPPERS_cfg_TA_TPRI = ( unsigned_t )TA_TPRI;
const unsigned_t TOPPERS_cfg_TA_MPRI = ( unsigned_t )TA_MPRI;
const unsigned_t TOPPERS_cfg_TA_WMUL = ( unsigned_t )TA_WMUL;
const unsigned_t TOPPERS_cfg_TA_CLR = ( unsigned_t )TA_CLR;
const unsigned_t TOPPERS_cfg_TA_CEILING = ( unsigned_t )TA_CEILING;
const unsigned_t TOPPERS_cfg_TA_STA = ( unsigned_t )TA_STA;
const unsigned_t TOPPERS_cfg_TA_NONKERNEL = ( unsigned_t )TA_NONKERNEL;
const unsigned_t TOPPERS_cfg_TA_ENAINT = ( unsigned_t )TA_ENAINT;
const unsigned_t TOPPERS_cfg_TA_EDGE = ( unsigned_t )TA_EDGE;
const signed_t TOPPERS_cfg_TMIN_TPRI = ( signed_t )TMIN_TPRI;
const signed_t TOPPERS_cfg_TMAX_TPRI = ( signed_t )TMAX_TPRI;
const signed_t TOPPERS_cfg_TMIN_DPRI = ( signed_t )TMIN_DPRI;
const signed_t TOPPERS_cfg_TMAX_DPRI = ( signed_t )TMAX_DPRI;
const signed_t TOPPERS_cfg_TMIN_MPRI = ( signed_t )TMIN_MPRI;
const signed_t TOPPERS_cfg_TMAX_MPRI = ( signed_t )TMAX_MPRI;
const signed_t TOPPERS_cfg_TMIN_ISRPRI = ( signed_t )TMIN_ISRPRI;
const signed_t TOPPERS_cfg_TMAX_ISRPRI = ( signed_t )TMAX_ISRPRI;
const unsigned_t TOPPERS_cfg_TBIT_TEXPTN = ( unsigned_t )TBIT_TEXPTN;
const unsigned_t TOPPERS_cfg_TBIT_FLGPTN = ( unsigned_t )TBIT_FLGPTN;
const unsigned_t TOPPERS_cfg_TMAX_MAXSEM = ( unsigned_t )TMAX_MAXSEM;
const unsigned_t TOPPERS_cfg_TMAX_RELTIM = ( unsigned_t )TMAX_RELTIM;
const signed_t TOPPERS_cfg_TMIN_INTPRI = ( signed_t )TMIN_INTPRI;
const unsigned_t TOPPERS_cfg_OMIT_INITIALIZE_INTERRUPT = 
#if defined(OMIT_INITIALIZE_INTERRUPT)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_OMIT_INITIALIZE_EXCEPTION = 
#if defined(OMIT_INITIALIZE_EXCEPTION)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_USE_TSKINICTXB = 
#if defined(USE_TSKINICTXB)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TARGET_TSKATR = 
#if defined(TARGET_TSKATR)
(TARGET_TSKATR);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TARGET_INTATR = 
#if defined(TARGET_INTATR)
(TARGET_INTATR);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TARGET_INHATR = 
#if defined(TARGET_INHATR)
(TARGET_INHATR);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TARGET_ISRATR = 
#if defined(TARGET_ISRATR)
(TARGET_ISRATR);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TARGET_EXCATR = 
#if defined(TARGET_EXCATR)
(TARGET_EXCATR);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TARGET_MIN_STKSZ = 
#if defined(TARGET_MIN_STKSZ)
(TARGET_MIN_STKSZ);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TARGET_MIN_ISTKSZ = 
#if defined(TARGET_MIN_ISTKSZ)
(TARGET_MIN_ISTKSZ);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_STKSZ_ALIGN = 
#if defined(CHECK_STKSZ_ALIGN)
(CHECK_STKSZ_ALIGN);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_FUNC_ALIGN = 
#if defined(CHECK_FUNC_ALIGN)
(CHECK_FUNC_ALIGN);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_FUNC_NONNULL = 
#if defined(CHECK_FUNC_NONNULL)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_STACK_ALIGN = 
#if defined(CHECK_STACK_ALIGN)
(CHECK_STACK_ALIGN);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_STACK_NONNULL = 
#if defined(CHECK_STACK_NONNULL)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_MPF_ALIGN = 
#if defined(CHECK_MPF_ALIGN)
(CHECK_MPF_ALIGN);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_MPF_NONNULL = 
#if defined(CHECK_MPF_NONNULL)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_MB_ALIGN = 
#if defined(CHECK_MB_ALIGN)
(CHECK_MB_ALIGN);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_sizeof_ID = ( unsigned_t )sizeof(ID);
const unsigned_t TOPPERS_cfg_sizeof_uint_t = ( unsigned_t )sizeof(uint_t);
const unsigned_t TOPPERS_cfg_sizeof_SIZE = ( unsigned_t )sizeof(SIZE);
const unsigned_t TOPPERS_cfg_sizeof_ATR = ( unsigned_t )sizeof(ATR);
const unsigned_t TOPPERS_cfg_sizeof_PRI = ( unsigned_t )sizeof(PRI);
const unsigned_t TOPPERS_cfg_sizeof_void_ptr = ( unsigned_t )sizeof(void*);
const unsigned_t TOPPERS_cfg_sizeof_VP = ( unsigned_t )sizeof(void*);
const unsigned_t TOPPERS_cfg_sizeof_intptr_t = ( unsigned_t )sizeof(intptr_t);
const unsigned_t TOPPERS_cfg_sizeof_FP = ( unsigned_t )sizeof(FP);
const unsigned_t TOPPERS_cfg_sizeof_INHNO = ( unsigned_t )sizeof(INHNO);
const unsigned_t TOPPERS_cfg_sizeof_INTNO = ( unsigned_t )sizeof(INTNO);
const unsigned_t TOPPERS_cfg_sizeof_EXCNO = ( unsigned_t )sizeof(EXCNO);
const unsigned_t TOPPERS_cfg_sizeof_TINIB = ( unsigned_t )sizeof(TINIB);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_tskatr = ( unsigned_t )offsetof(TINIB,tskatr);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_exinf = ( unsigned_t )offsetof(TINIB,exinf);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_task = ( unsigned_t )offsetof(TINIB,task);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_ipriority = ( unsigned_t )offsetof(TINIB,ipriority);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_stksz = 
#if !defined(USE_TSKINICTXB)
(offsetof(TINIB,stksz));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_TINIB_stk = 
#if !defined(USE_TSKINICTXB)
(offsetof(TINIB,stk));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_TINIB_texatr = ( unsigned_t )offsetof(TINIB,texatr);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_texrtn = ( unsigned_t )offsetof(TINIB,texrtn);
const unsigned_t TOPPERS_cfg_sizeof_SEMINIB = ( unsigned_t )sizeof(SEMINIB);
const unsigned_t TOPPERS_cfg_offsetof_SEMINIB_sematr = ( unsigned_t )offsetof(SEMINIB,sematr);
const unsigned_t TOPPERS_cfg_offsetof_SEMINIB_isemcnt = ( unsigned_t )offsetof(SEMINIB,isemcnt);
const unsigned_t TOPPERS_cfg_offsetof_SEMINIB_maxsem = ( unsigned_t )offsetof(SEMINIB,maxsem);
const unsigned_t TOPPERS_cfg_sizeof_FLGPTN = ( unsigned_t )sizeof(FLGPTN);
const unsigned_t TOPPERS_cfg_sizeof_FLGINIB = ( unsigned_t )sizeof(FLGINIB);
const unsigned_t TOPPERS_cfg_offsetof_FLGINIB_flgatr = ( unsigned_t )offsetof(FLGINIB,flgatr);
const unsigned_t TOPPERS_cfg_offsetof_FLGINIB_iflgptn = ( unsigned_t )offsetof(FLGINIB,iflgptn);
const unsigned_t TOPPERS_cfg_sizeof_DTQINIB = ( unsigned_t )sizeof(DTQINIB);
const unsigned_t TOPPERS_cfg_offsetof_DTQINIB_dtqatr = ( unsigned_t )offsetof(DTQINIB,dtqatr);
const unsigned_t TOPPERS_cfg_offsetof_DTQINIB_dtqcnt = ( unsigned_t )offsetof(DTQINIB,dtqcnt);
const unsigned_t TOPPERS_cfg_offsetof_DTQINIB_p_dtqmb = ( unsigned_t )offsetof(DTQINIB,p_dtqmb);
const unsigned_t TOPPERS_cfg_sizeof_PDQINIB = ( unsigned_t )sizeof(PDQINIB);
const unsigned_t TOPPERS_cfg_offsetof_PDQINIB_pdqatr = ( unsigned_t )offsetof(PDQINIB,pdqatr);
const unsigned_t TOPPERS_cfg_offsetof_PDQINIB_pdqcnt = ( unsigned_t )offsetof(PDQINIB,pdqcnt);
const unsigned_t TOPPERS_cfg_offsetof_PDQINIB_maxdpri = ( unsigned_t )offsetof(PDQINIB,maxdpri);
const unsigned_t TOPPERS_cfg_offsetof_PDQINIB_p_pdqmb = ( unsigned_t )offsetof(PDQINIB,p_pdqmb);
const unsigned_t TOPPERS_cfg_sizeof_MBXINIB = ( unsigned_t )sizeof(MBXINIB);
const unsigned_t TOPPERS_cfg_offsetof_MBXINIB_mbxatr = ( unsigned_t )offsetof(MBXINIB,mbxatr);
const unsigned_t TOPPERS_cfg_offsetof_MBXINIB_maxmpri = ( unsigned_t )offsetof(MBXINIB,maxmpri);
const unsigned_t TOPPERS_cfg_sizeof_MTXINIB = ( unsigned_t )sizeof(MTXINIB);
const unsigned_t TOPPERS_cfg_offsetof_MTXINIB_mtxatr = ( unsigned_t )offsetof(MTXINIB,mtxatr);
const unsigned_t TOPPERS_cfg_offsetof_MTXINIB_ceilpri = ( unsigned_t )offsetof(MTXINIB,ceilpri);
const unsigned_t TOPPERS_cfg_sizeof_MPFINIB = ( unsigned_t )sizeof(MPFINIB);
const unsigned_t TOPPERS_cfg_offsetof_MPFINIB_mpfatr = ( unsigned_t )offsetof(MPFINIB,mpfatr);
const unsigned_t TOPPERS_cfg_offsetof_MPFINIB_blkcnt = ( unsigned_t )offsetof(MPFINIB,blkcnt);
const unsigned_t TOPPERS_cfg_offsetof_MPFINIB_blksz = ( unsigned_t )offsetof(MPFINIB,blksz);
const unsigned_t TOPPERS_cfg_offsetof_MPFINIB_mpf = ( unsigned_t )offsetof(MPFINIB,mpf);
const unsigned_t TOPPERS_cfg_offsetof_MPFINIB_p_mpfmb = ( unsigned_t )offsetof(MPFINIB,p_mpfmb);
const unsigned_t TOPPERS_cfg_sizeof_CYCINIB = ( unsigned_t )sizeof(CYCINIB);
const unsigned_t TOPPERS_cfg_offsetof_CYCINIB_cycatr = ( unsigned_t )offsetof(CYCINIB,cycatr);
const unsigned_t TOPPERS_cfg_offsetof_CYCINIB_exinf = ( unsigned_t )offsetof(CYCINIB,exinf);
const unsigned_t TOPPERS_cfg_offsetof_CYCINIB_cychdr = ( unsigned_t )offsetof(CYCINIB,cychdr);
const unsigned_t TOPPERS_cfg_offsetof_CYCINIB_cyctim = ( unsigned_t )offsetof(CYCINIB,cyctim);
const unsigned_t TOPPERS_cfg_offsetof_CYCINIB_cycphs = ( unsigned_t )offsetof(CYCINIB,cycphs);
const unsigned_t TOPPERS_cfg_sizeof_ALMINIB = ( unsigned_t )sizeof(ALMINIB);
const unsigned_t TOPPERS_cfg_offsetof_ALMINIB_almatr = ( unsigned_t )offsetof(ALMINIB,almatr);
const unsigned_t TOPPERS_cfg_offsetof_ALMINIB_exinf = ( unsigned_t )offsetof(ALMINIB,exinf);
const unsigned_t TOPPERS_cfg_offsetof_ALMINIB_almhdr = ( unsigned_t )offsetof(ALMINIB,almhdr);
const unsigned_t TOPPERS_cfg_sizeof_ISRINIB = ( unsigned_t )sizeof(ISRINIB);
const unsigned_t TOPPERS_cfg_offsetof_ISRINIB_isratr = ( unsigned_t )offsetof(ISRINIB,isratr);
const unsigned_t TOPPERS_cfg_offsetof_ISRINIB_exinf = ( unsigned_t )offsetof(ISRINIB,exinf);
const unsigned_t TOPPERS_cfg_offsetof_ISRINIB_intno = ( unsigned_t )offsetof(ISRINIB,intno);
const unsigned_t TOPPERS_cfg_offsetof_ISRINIB_p_isr_queue = ( unsigned_t )offsetof(ISRINIB,p_isr_queue);
const unsigned_t TOPPERS_cfg_offsetof_ISRINIB_isr = ( unsigned_t )offsetof(ISRINIB,isr);
const unsigned_t TOPPERS_cfg_offsetof_ISRINIB_isrpri = ( unsigned_t )offsetof(ISRINIB,isrpri);
const unsigned_t TOPPERS_cfg_sizeof_INHINIB = 
#if !defined(OMIT_INITIALIZE_INTERRUPT)
(sizeof(INHINIB));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offset_INHINIB_inhno = 
#if !defined(OMIT_INITIALIZE_INTERRUPT)
(offsetof(INHINIB,inhno));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offset_INHINIB_inhatr = 
#if !defined(OMIT_INITIALIZE_INTERRUPT)
(offsetof(INHINIB,inhatr));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offset_INHINIB_int_entry = 
#if !defined(OMIT_INITIALIZE_INTERRUPT)
(offsetof(INHINIB,int_entry));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_sizeof_INTINIB = 
#if !defined(OMIT_INITIALIZE_INTERRUPT)
(sizeof(INTINIB));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offset_INTINIB_intno = 
#if !defined(OMIT_INITIALIZE_INTERRUPT)
(offsetof(INTINIB,intno));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offset_INTINIB_intatr = 
#if !defined(OMIT_INITIALIZE_INTERRUPT)
(offsetof(INTINIB,intatr));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offset_INTINIB_intpri = 
#if !defined(OMIT_INITIALIZE_INTERRUPT)
(offsetof(INTINIB,intpri));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_sizeof_EXCINIB = 
#if !defined(OMIT_INITIALIZE_EXCEPTION)
(sizeof(EXCINIB));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offset_EXCINIB_excno = 
#if !defined(OMIT_INITIALIZE_EXCEPTION)
(offsetof(EXCINIB,excno));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offset_EXCINIB_excatr = 
#if !defined(OMIT_INITIALIZE_EXCEPTION)
(offsetof(EXCINIB,excatr));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offset_EXCINIB_exc_entry = 
#if !defined(OMIT_INITIALIZE_EXCEPTION)
(offsetof(EXCINIB,exc_entry));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TMAX_INTNO = ( unsigned_t )TMAX_INTNO;
const unsigned_t TOPPERS_cfg_TBITW_IPRI = ( unsigned_t )TBITW_IPRI;
const unsigned_t TOPPERS_cfg___TARGET_ARCH_THUMB = ( unsigned_t )__TARGET_ARCH_THUMB;
const unsigned_t TOPPERS_cfg_sizeof_TCB = ( unsigned_t )sizeof(TCB);
const unsigned_t TOPPERS_cfg_offsetof_TCB_p_tinib = ( unsigned_t )offsetof(TCB,p_tinib);
const unsigned_t TOPPERS_cfg_offsetof_TCB_texptn = ( unsigned_t )offsetof(TCB,texptn);
const unsigned_t TOPPERS_cfg_offsetof_TCB_sp = ( unsigned_t )offsetof(TCB,tskctxb.sp);
const unsigned_t TOPPERS_cfg_offsetof_TCB_pc = ( unsigned_t )offsetof(TCB,tskctxb.pc);


/* #include "target_timer.h" */

#line 8 "../../asp_baseplatform/arch/arm_m_gcc/common/core_timer.cfg"
const unsigned_t TOPPERS_cfg_static_api_0 = 0;
const unsigned_t TOPPERS_cfg_valueof_iniatr_0 = ( unsigned_t )( TA_NULL ); 
#line 9 "../../asp_baseplatform/arch/arm_m_gcc/common/core_timer.cfg"
const unsigned_t TOPPERS_cfg_static_api_1 = 1;
const unsigned_t TOPPERS_cfg_valueof_teratr_1 = ( unsigned_t )( TA_NULL ); 
#line 10 "../../asp_baseplatform/arch/arm_m_gcc/common/core_timer.cfg"
const unsigned_t TOPPERS_cfg_static_api_2 = 2;
const unsigned_t TOPPERS_cfg_valueof_inhno_2 = ( unsigned_t )( INHNO_TIMER ); const unsigned_t TOPPERS_cfg_valueof_inhatr_2 = ( unsigned_t )( TA_NULL ); 
#line 11 "../../asp_baseplatform/arch/arm_m_gcc/common/core_timer.cfg"
const unsigned_t TOPPERS_cfg_static_api_3 = 3;
const unsigned_t TOPPERS_cfg_valueof_intno_3 = ( unsigned_t )( INTNO_TIMER ); const unsigned_t TOPPERS_cfg_valueof_intatr_3 = ( unsigned_t )( TA_ENAINT|INTATR_TIMER ); const signed_t TOPPERS_cfg_valueof_intpri_3 = ( signed_t )( INTPRI_TIMER ); /* #include "syssvc/syslog.h" */

#line 10 "../../asp_baseplatform/syssvc/syslog.cfg"
const unsigned_t TOPPERS_cfg_static_api_4 = 4;
const unsigned_t TOPPERS_cfg_valueof_iniatr_4 = ( unsigned_t )( TA_NULL ); /* #include "syssvc/banner.h" */

#line 10 "../../asp_baseplatform/syssvc/banner.cfg"
const unsigned_t TOPPERS_cfg_static_api_5 = 5;
const unsigned_t TOPPERS_cfg_valueof_iniatr_5 = ( unsigned_t )( TA_NULL ); /* #include "target_syssvc.h" */
/* #include "target_serial.h" */

#line 9 "../../asp_baseplatform/target/stm32f767nucleo144_gcc/target_serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_6 = 6;
const unsigned_t TOPPERS_cfg_valueof_iniatr_6 = ( unsigned_t )( TA_NULL ); 
#line 10 "../../asp_baseplatform/target/stm32f767nucleo144_gcc/target_serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_7 = 7;
const unsigned_t TOPPERS_cfg_valueof_isratr_7 = ( unsigned_t )( TA_NULL ); const unsigned_t TOPPERS_cfg_valueof_intno_7 = ( unsigned_t )( INTNO_SIO1 ); const signed_t TOPPERS_cfg_valueof_isrpri_7 = ( signed_t )( 1 ); 
#line 11 "../../asp_baseplatform/target/stm32f767nucleo144_gcc/target_serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_8 = 8;
const unsigned_t TOPPERS_cfg_valueof_intno_8 = ( unsigned_t )( INTNO_SIO1 ); const unsigned_t TOPPERS_cfg_valueof_intatr_8 = ( unsigned_t )( TA_ENAINT|INTATR_SIO ); const signed_t TOPPERS_cfg_valueof_intpri_8 = ( signed_t )( INTPRI_SIO ); 
#if TNUM_SIOP >= 2

#line 13 "../../asp_baseplatform/target/stm32f767nucleo144_gcc/target_serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_9 = 9;
const unsigned_t TOPPERS_cfg_valueof_isratr_9 = ( unsigned_t )( TA_NULL ); const unsigned_t TOPPERS_cfg_valueof_intno_9 = ( unsigned_t )( INTNO_SIO2 ); const signed_t TOPPERS_cfg_valueof_isrpri_9 = ( signed_t )( 1 ); 
#line 14 "../../asp_baseplatform/target/stm32f767nucleo144_gcc/target_serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_10 = 10;
const unsigned_t TOPPERS_cfg_valueof_intno_10 = ( unsigned_t )( INTNO_SIO2 ); const unsigned_t TOPPERS_cfg_valueof_intatr_10 = ( unsigned_t )( TA_ENAINT|INTATR_SIO ); const signed_t TOPPERS_cfg_valueof_intpri_10 = ( signed_t )( INTPRI_SIO ); 
#endif
/* #include "syssvc/serial.h" */

#line 13 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_11 = 11;
const unsigned_t TOPPERS_cfg_valueof_iniatr_11 = ( unsigned_t )( TA_NULL ); 
#line 15 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_12 = 12;
#define SERIAL_RCV_SEM1	(<>)

#line 15 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_12 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_12 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_12 = ( unsigned_t )( 1 ); 
#line 16 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_13 = 13;
#define SERIAL_SND_SEM1	(<>)

#line 16 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_13 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_13 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_13 = ( unsigned_t )( 1 ); 
#if TNUM_PORT >= 2

#line 18 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_14 = 14;
#define SERIAL_RCV_SEM2	(<>)

#line 18 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_14 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_14 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_14 = ( unsigned_t )( 1 ); 
#line 19 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_15 = 15;
#define SERIAL_SND_SEM2	(<>)

#line 19 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_15 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_15 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_15 = ( unsigned_t )( 1 ); 
#endif 

#if TNUM_PORT >= 3

#line 22 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_16 = 16;
#define SERIAL_RCV_SEM3	(<>)

#line 22 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_16 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_16 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_16 = ( unsigned_t )( 1 ); 
#line 23 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_17 = 17;
#define SERIAL_SND_SEM3	(<>)

#line 23 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_17 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_17 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_17 = ( unsigned_t )( 1 ); 
#endif 

#if TNUM_PORT >= 4

#line 26 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_18 = 18;
#define SERIAL_RCV_SEM4	(<>)

#line 26 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_18 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_18 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_18 = ( unsigned_t )( 1 ); 
#line 27 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_static_api_19 = 19;
#define SERIAL_SND_SEM4	(<>)

#line 27 "../../asp_baseplatform/syssvc/serial.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_19 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_19 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_19 = ( unsigned_t )( 1 ); 
#endif 
/* #include "syssvc/logtask.h" */

#line 10 "../../asp_baseplatform/syssvc/logtask.cfg"
const unsigned_t TOPPERS_cfg_static_api_20 = 20;
#define LOGTASK	(<>)

#line 10 "../../asp_baseplatform/syssvc/logtask.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_20 = ( unsigned_t )( TA_ACT ); const signed_t TOPPERS_cfg_valueof_itskpri_20 = ( signed_t )( LOGTASK_PRIORITY ); const unsigned_t TOPPERS_cfg_valueof_stksz_20 = ( unsigned_t )( LOGTASK_STACK_SIZE ); 
#line 12 "../../asp_baseplatform/syssvc/logtask.cfg"
const unsigned_t TOPPERS_cfg_static_api_21 = 21;
const unsigned_t TOPPERS_cfg_valueof_teratr_21 = ( unsigned_t )( TA_NULL ); /* #include "device.h" */

#line 11 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_22 = 22;
const unsigned_t TOPPERS_cfg_valueof_iniatr_22 = ( unsigned_t )( TA_NULL ); 
#line 12 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_23 = 23;
const unsigned_t TOPPERS_cfg_valueof_iniatr_23 = ( unsigned_t )( TA_NULL ); 
#if defined(TOPPERS_STM32F7_DISCOVERY) || defined(TOPPERS_STM32F769_DISCOVERY)

#line 14 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_24 = 24;
const unsigned_t TOPPERS_cfg_valueof_iniatr_24 = ( unsigned_t )( TA_NULL ); 
#endif

#line 16 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_25 = 25;
const unsigned_t TOPPERS_cfg_valueof_iniatr_25 = ( unsigned_t )( TA_NULL ); 
#line 17 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_26 = 26;
const unsigned_t TOPPERS_cfg_valueof_iniatr_26 = ( unsigned_t )( TA_NULL ); 
#line 19 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_27 = 27;
#define DMA2DSEM	(<>)

#line 19 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_27 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_27 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_27 = ( unsigned_t )( 1 ); 
#line 20 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_28 = 28;
#define DMA2DTRNSEM	(<>)

#line 20 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_28 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_28 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_28 = ( unsigned_t )( 1 ); 
#line 21 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_29 = 29;
#define RTCSEM	(<>)

#line 21 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_29 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_29 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_29 = ( unsigned_t )( 1 ); 
#line 23 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_30 = 30;
const unsigned_t TOPPERS_cfg_valueof_inhno_30 = ( unsigned_t )( INHNO_DMA2D ); const unsigned_t TOPPERS_cfg_valueof_inhatr_30 = ( unsigned_t )( TA_NULL ); 
#line 24 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_31 = 31;
const unsigned_t TOPPERS_cfg_valueof_intno_31 = ( unsigned_t )( INTNO_DMA2D ); const unsigned_t TOPPERS_cfg_valueof_intatr_31 = ( unsigned_t )( TA_ENAINT | INTATR_DMA2D ); const signed_t TOPPERS_cfg_valueof_intpri_31 = ( signed_t )( INTPRI_DMA2D ); 
#line 26 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_32 = 32;
const unsigned_t TOPPERS_cfg_valueof_inhno_32 = ( unsigned_t )( INHNO_ALARM ); const unsigned_t TOPPERS_cfg_valueof_inhatr_32 = ( unsigned_t )( TA_NULL ); 
#line 27 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_33 = 33;
const unsigned_t TOPPERS_cfg_valueof_intno_33 = ( unsigned_t )( INTNO_ALARM ); const unsigned_t TOPPERS_cfg_valueof_intatr_33 = ( unsigned_t )( TA_ENAINT | INTATR_ALARM ); const signed_t TOPPERS_cfg_valueof_intpri_33 = ( signed_t )( INTPRI_ALARM ); 
#line 29 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_34 = 34;
const unsigned_t TOPPERS_cfg_valueof_inhno_34 = ( unsigned_t )( INHNO_RTCWUP ); const unsigned_t TOPPERS_cfg_valueof_inhatr_34 = ( unsigned_t )( TA_NULL ); 
#line 30 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_35 = 35;
const unsigned_t TOPPERS_cfg_valueof_intno_35 = ( unsigned_t )( INTNO_RTCWUP ); const unsigned_t TOPPERS_cfg_valueof_intatr_35 = ( unsigned_t )( TA_ENAINT | INTATR_RTCWUP ); const signed_t TOPPERS_cfg_valueof_intpri_35 = ( signed_t )( INTPRI_RTCWUP ); 
#line 32 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_36 = 36;
const unsigned_t TOPPERS_cfg_valueof_inhno_36 = ( unsigned_t )( INHNO_EXTI9 ); const unsigned_t TOPPERS_cfg_valueof_inhatr_36 = ( unsigned_t )( TA_NULL ); 
#line 33 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_37 = 37;
const unsigned_t TOPPERS_cfg_valueof_intno_37 = ( unsigned_t )( INTNO_EXTI9 ); const unsigned_t TOPPERS_cfg_valueof_intatr_37 = ( unsigned_t )( TA_ENAINT | INTATR_EXTI9 ); const signed_t TOPPERS_cfg_valueof_intpri_37 = ( signed_t )( INTPRI_EXTI9 ); 
#line 35 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_38 = 38;
const unsigned_t TOPPERS_cfg_valueof_inhno_38 = ( unsigned_t )( INHNO_EXTI15 ); const unsigned_t TOPPERS_cfg_valueof_inhatr_38 = ( unsigned_t )( TA_NULL ); 
#line 36 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_39 = 39;
const unsigned_t TOPPERS_cfg_valueof_intno_39 = ( unsigned_t )( INTNO_EXTI15 ); const unsigned_t TOPPERS_cfg_valueof_intatr_39 = ( unsigned_t )( TA_ENAINT | INTATR_EXTI15 ); const signed_t TOPPERS_cfg_valueof_intpri_39 = ( signed_t )( INTPRI_EXTI15 ); 
#ifdef INHNO_SW1

#line 39 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_40 = 40;
const unsigned_t TOPPERS_cfg_valueof_inhno_40 = ( unsigned_t )( INHNO_SW1 ); const unsigned_t TOPPERS_cfg_valueof_inhatr_40 = ( unsigned_t )( TA_NULL ); 
#line 40 "../../asp_baseplatform/pdic/stm32f7xx/device.cfg"
const unsigned_t TOPPERS_cfg_static_api_41 = 41;
const unsigned_t TOPPERS_cfg_valueof_intno_41 = ( unsigned_t )( INTNO_SW1 ); const unsigned_t TOPPERS_cfg_valueof_intatr_41 = ( unsigned_t )( TA_ENAINT | INTATR_SW1 ); const signed_t TOPPERS_cfg_valueof_intpri_41 = ( signed_t )( INTPRI_SW1 ); 
#endif
/* #include "device.h" */
/* #include "sdmmc.h" */

#line 12 "../../asp_baseplatform/pdic/stm32f7xx/sdmmc.cfg"
const unsigned_t TOPPERS_cfg_static_api_42 = 42;
const unsigned_t TOPPERS_cfg_valueof_iniatr_42 = ( unsigned_t )( TA_NULL ); 
#line 13 "../../asp_baseplatform/pdic/stm32f7xx/sdmmc.cfg"
const unsigned_t TOPPERS_cfg_static_api_43 = 43;
#define SDMMC_SEM	(<>)

#line 13 "../../asp_baseplatform/pdic/stm32f7xx/sdmmc.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_43 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_43 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_43 = ( unsigned_t )( 1 ); 
#line 15 "../../asp_baseplatform/pdic/stm32f7xx/sdmmc.cfg"
const unsigned_t TOPPERS_cfg_static_api_44 = 44;
const unsigned_t TOPPERS_cfg_valueof_inhno_44 = ( unsigned_t )( INHNO_SDMMC ); const unsigned_t TOPPERS_cfg_valueof_inhatr_44 = ( unsigned_t )( TA_NULL ); 
#line 16 "../../asp_baseplatform/pdic/stm32f7xx/sdmmc.cfg"
const unsigned_t TOPPERS_cfg_static_api_45 = 45;
const unsigned_t TOPPERS_cfg_valueof_intno_45 = ( unsigned_t )( INTNO_SDMMC ); const unsigned_t TOPPERS_cfg_valueof_intatr_45 = ( unsigned_t )( TA_ENAINT | INTATR_SDMMC ); const signed_t TOPPERS_cfg_valueof_intpri_45 = ( signed_t )( INTPRI_SDMMC ); 
#line 17 "../../asp_baseplatform/pdic/stm32f7xx/sdmmc.cfg"
const unsigned_t TOPPERS_cfg_static_api_46 = 46;
const unsigned_t TOPPERS_cfg_valueof_isratr_46 = ( unsigned_t )( TA_NULL ); const unsigned_t TOPPERS_cfg_valueof_intno_46 = ( unsigned_t )( INTNO_DMARX ); const signed_t TOPPERS_cfg_valueof_isrpri_46 = ( signed_t )( 1 ); 
#line 18 "../../asp_baseplatform/pdic/stm32f7xx/sdmmc.cfg"
const unsigned_t TOPPERS_cfg_static_api_47 = 47;
const unsigned_t TOPPERS_cfg_valueof_intno_47 = ( unsigned_t )( INTNO_DMARX ); const unsigned_t TOPPERS_cfg_valueof_intatr_47 = ( unsigned_t )( TA_ENAINT | INTATR_DMARX ); const signed_t TOPPERS_cfg_valueof_intpri_47 = ( signed_t )( INTPRI_DMARX ); 
#line 19 "../../asp_baseplatform/pdic/stm32f7xx/sdmmc.cfg"
const unsigned_t TOPPERS_cfg_static_api_48 = 48;
const unsigned_t TOPPERS_cfg_valueof_isratr_48 = ( unsigned_t )( TA_NULL ); const unsigned_t TOPPERS_cfg_valueof_intno_48 = ( unsigned_t )( INTNO_DMATX ); const signed_t TOPPERS_cfg_valueof_isrpri_48 = ( signed_t )( 1 ); 
#line 20 "../../asp_baseplatform/pdic/stm32f7xx/sdmmc.cfg"
const unsigned_t TOPPERS_cfg_static_api_49 = 49;
const unsigned_t TOPPERS_cfg_valueof_intno_49 = ( unsigned_t )( INTNO_DMATX ); const unsigned_t TOPPERS_cfg_valueof_intatr_49 = ( unsigned_t )( TA_ENAINT | INTATR_DMATX ); const signed_t TOPPERS_cfg_valueof_intpri_49 = ( signed_t )( INTPRI_DMATX ); /* #include "storagedevice.h" */

#line 10 "../../asp_baseplatform/files/storagedevice.cfg"
const unsigned_t TOPPERS_cfg_static_api_50 = 50;
const unsigned_t TOPPERS_cfg_valueof_iniatr_50 = ( unsigned_t )( TA_NULL ); 
#line 11 "../../asp_baseplatform/files/storagedevice.cfg"
const unsigned_t TOPPERS_cfg_static_api_51 = 51;
const unsigned_t TOPPERS_cfg_valueof_iniatr_51 = ( unsigned_t )( TA_NULL ); 
#line 12 "../../asp_baseplatform/files/storagedevice.cfg"
const unsigned_t TOPPERS_cfg_static_api_52 = 52;
const unsigned_t TOPPERS_cfg_valueof_iniatr_52 = ( unsigned_t )( TA_NULL ); 
#line 13 "../../asp_baseplatform/files/storagedevice.cfg"
const unsigned_t TOPPERS_cfg_static_api_53 = 53;
#define SEM_STDFILE	(<>)

#line 13 "../../asp_baseplatform/files/storagedevice.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_53 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_53 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_53 = ( unsigned_t )( 1 ); 
#ifndef SDEV_SENSE_ONETIME

#line 16 "../../asp_baseplatform/files/storagedevice.cfg"
const unsigned_t TOPPERS_cfg_static_api_54 = 54;
#define SDM_TASK	(<>)

#line 16 "../../asp_baseplatform/files/storagedevice.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_54 = ( unsigned_t )( TA_ACT ); const signed_t TOPPERS_cfg_valueof_itskpri_54 = ( signed_t )( 15 ); const unsigned_t TOPPERS_cfg_valueof_stksz_54 = ( unsigned_t )( 1024 ); 
#endif
/* #include "ff.h" */
/* #include "sddiskio.h" */

#line 12 "../../asp_baseplatform/files/ff/fatfs.cfg"
const unsigned_t TOPPERS_cfg_static_api_55 = 55;
const unsigned_t TOPPERS_cfg_valueof_iniatr_55 = ( unsigned_t )( TA_NULL ); 
#line 14 "../../asp_baseplatform/files/ff/fatfs.cfg"
const unsigned_t TOPPERS_cfg_static_api_56 = 56;
#define SEMID_FATFS1	(<>)

#line 14 "../../asp_baseplatform/files/ff/fatfs.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_56 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_56 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_56 = ( unsigned_t )( 1 ); 
#line 15 "../../asp_baseplatform/files/ff/fatfs.cfg"
const unsigned_t TOPPERS_cfg_static_api_57 = 57;
#define SEMID_FATFS2	(<>)

#line 15 "../../asp_baseplatform/files/ff/fatfs.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_57 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_57 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_57 = ( unsigned_t )( 1 ); 
#line 16 "../../asp_baseplatform/files/ff/fatfs.cfg"
const unsigned_t TOPPERS_cfg_static_api_58 = 58;
#define SEMID_FATFS3	(<>)

#line 16 "../../asp_baseplatform/files/ff/fatfs.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_58 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_58 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_58 = ( unsigned_t )( 1 ); 
#line 17 "../../asp_baseplatform/files/ff/fatfs.cfg"
const unsigned_t TOPPERS_cfg_static_api_59 = 59;
#define SEMID_FATFS4	(<>)

#line 17 "../../asp_baseplatform/files/ff/fatfs.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_59 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_59 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_59 = ( unsigned_t )( 1 ); /* #include "monitor.h" */

#line 10 "../../asp_baseplatform/monitor/monitor.cfg"
const unsigned_t TOPPERS_cfg_static_api_60 = 60;
#define MONTASK	(<>)

#line 10 "../../asp_baseplatform/monitor/monitor.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_60 = ( unsigned_t )( TA_ACT ); const signed_t TOPPERS_cfg_valueof_itskpri_60 = ( signed_t )( MONITOR_PRIORITY ); const unsigned_t TOPPERS_cfg_valueof_stksz_60 = ( unsigned_t )( MONITOR_STACK_SIZE ); 
#line 4 "../../asp_baseplatform/syssvc/tlsf.cfg"
const unsigned_t TOPPERS_cfg_static_api_61 = 61;
#define TLSF_SEM	(<>)

#line 4 "../../asp_baseplatform/syssvc/tlsf.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_61 = ( unsigned_t )( TA_NULL ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_61 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_61 = ( unsigned_t )( 1 ); /* #include "lwip.h" */

#line 12 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_62 = 62;
#define LWIP_TASK_1	(<>)

#line 12 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_62 = ( unsigned_t )( TA_NULL ); const signed_t TOPPERS_cfg_valueof_itskpri_62 = ( signed_t )( LWIP_DEFAULT_PRIORITY ); const unsigned_t TOPPERS_cfg_valueof_stksz_62 = ( unsigned_t )( LWIP_STACK_SIZE ); 
#line 13 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_63 = 63;
#define LWIP_TASK_2	(<>)

#line 13 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_63 = ( unsigned_t )( TA_NULL ); const signed_t TOPPERS_cfg_valueof_itskpri_63 = ( signed_t )( LWIP_DEFAULT_PRIORITY ); const unsigned_t TOPPERS_cfg_valueof_stksz_63 = ( unsigned_t )( LWIP_STACK_SIZE ); 
#line 14 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_64 = 64;
#define LWIP_TASK_3	(<>)

#line 14 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_64 = ( unsigned_t )( TA_NULL ); const signed_t TOPPERS_cfg_valueof_itskpri_64 = ( signed_t )( LWIP_DEFAULT_PRIORITY ); const unsigned_t TOPPERS_cfg_valueof_stksz_64 = ( unsigned_t )( LWIP_STACK_SIZE ); 
#line 15 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_65 = 65;
#define LWIP_TASK_4	(<>)

#line 15 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_65 = ( unsigned_t )( TA_NULL ); const signed_t TOPPERS_cfg_valueof_itskpri_65 = ( signed_t )( LWIP_DEFAULT_PRIORITY ); const unsigned_t TOPPERS_cfg_valueof_stksz_65 = ( unsigned_t )( LWIP_STACK_SIZE ); 
#line 16 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_66 = 66;
#define LWIP_TASK_5	(<>)

#line 16 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_66 = ( unsigned_t )( TA_NULL ); const signed_t TOPPERS_cfg_valueof_itskpri_66 = ( signed_t )( LWIP_DEFAULT_PRIORITY ); const unsigned_t TOPPERS_cfg_valueof_stksz_66 = ( unsigned_t )( LWIP_STACK_SIZE ); 
#line 17 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_67 = 67;
#define LWIP_TASK_6	(<>)

#line 17 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_67 = ( unsigned_t )( TA_NULL ); const signed_t TOPPERS_cfg_valueof_itskpri_67 = ( signed_t )( LWIP_DEFAULT_PRIORITY ); const unsigned_t TOPPERS_cfg_valueof_stksz_67 = ( unsigned_t )( LWIP_STACK_SIZE ); 
#line 18 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_68 = 68;
#define LWIP_TASK_7	(<>)

#line 18 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_68 = ( unsigned_t )( TA_NULL ); const signed_t TOPPERS_cfg_valueof_itskpri_68 = ( signed_t )( LWIP_DEFAULT_PRIORITY ); const unsigned_t TOPPERS_cfg_valueof_stksz_68 = ( unsigned_t )( LWIP_STACK_SIZE ); 
#line 19 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_69 = 69;
#define LWIP_TASK_8	(<>)

#line 19 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_69 = ( unsigned_t )( TA_NULL ); const signed_t TOPPERS_cfg_valueof_itskpri_69 = ( signed_t )( LWIP_DEFAULT_PRIORITY ); const unsigned_t TOPPERS_cfg_valueof_stksz_69 = ( unsigned_t )( LWIP_STACK_SIZE ); 
#line 20 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_70 = 70;
#define LWIP_TASK_N	(<>)

#line 20 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_70 = ( unsigned_t )( TA_NULL ); const signed_t TOPPERS_cfg_valueof_itskpri_70 = ( signed_t )( LWIP_DEFAULT_PRIORITY ); const unsigned_t TOPPERS_cfg_valueof_stksz_70 = ( unsigned_t )( LWIP_STACK_SIZE ); 
#line 22 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_71 = 71;
#define LWIP_PROTECT	(<>)

#line 22 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_71 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_71 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_71 = ( unsigned_t )( 1 ); 
#line 23 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_72 = 72;
#define LWIP_THREAD	(<>)

#line 23 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_72 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_72 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_72 = ( unsigned_t )( 1 ); 
#line 24 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_73 = 73;
#define LWIP_SNPRINTF	(<>)

#line 24 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_73 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_73 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_73 = ( unsigned_t )( 1 ); 
#line 25 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_74 = 74;
#define LWIP_LOCK_1	(<>)

#line 25 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_74 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_74 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_74 = ( unsigned_t )( 1 ); 
#line 26 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_75 = 75;
#define LWIP_LOCK_2	(<>)

#line 26 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_75 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_75 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_75 = ( unsigned_t )( 1 ); 
#line 27 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_76 = 76;
#define LWIP_LOCK_3	(<>)

#line 27 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_76 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_76 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_76 = ( unsigned_t )( 1 ); 
#line 28 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_77 = 77;
#define LWIP_LOCK_4	(<>)

#line 28 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_77 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_77 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_77 = ( unsigned_t )( 1 ); 
#line 29 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_78 = 78;
#define LWIP_LOCK_5	(<>)

#line 29 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_78 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_78 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_78 = ( unsigned_t )( 1 ); 
#line 30 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_79 = 79;
#define LWIP_LOCK_6	(<>)

#line 30 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_79 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_79 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_79 = ( unsigned_t )( 1 ); 
#line 31 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_80 = 80;
#define LWIP_LOCK_7	(<>)

#line 31 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_80 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_80 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_80 = ( unsigned_t )( 1 ); 
#line 32 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_81 = 81;
#define LWIP_LOCK_8	(<>)

#line 32 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_81 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_81 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_81 = ( unsigned_t )( 1 ); 
#line 33 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_82 = 82;
#define LWIP_LOCK_9	(<>)

#line 33 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_82 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_82 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_82 = ( unsigned_t )( 1 ); 
#line 34 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_83 = 83;
#define LWIP_LOCK_10	(<>)

#line 34 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_83 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_83 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_83 = ( unsigned_t )( 1 ); 
#line 35 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_84 = 84;
#define LWIP_LOCK_11	(<>)

#line 35 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_84 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_84 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_84 = ( unsigned_t )( 1 ); 
#line 36 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_85 = 85;
#define LWIP_LOCK_12	(<>)

#line 36 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_85 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_85 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_85 = ( unsigned_t )( 1 ); 
#line 37 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_86 = 86;
#define LWIP_LOCK_13	(<>)

#line 37 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_86 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_86 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_86 = ( unsigned_t )( 1 ); 
#line 38 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_87 = 87;
#define LWIP_LOCK_N	(<>)

#line 38 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_87 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_87 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_87 = ( unsigned_t )( 1 ); 
#line 40 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_88 = 88;
#define LWIP0_SEM_1	(<>)

#line 40 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_88 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_88 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_88 = ( unsigned_t )( 1 ); 
#line 41 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_89 = 89;
#define LWIP0_SEM_2	(<>)

#line 41 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_89 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_89 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_89 = ( unsigned_t )( 1 ); 
#line 42 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_90 = 90;
#define LWIP0_SEM_3	(<>)

#line 42 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_90 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_90 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_90 = ( unsigned_t )( 1 ); 
#line 43 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_91 = 91;
#define LWIP0_SEM_4	(<>)

#line 43 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_91 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_91 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_91 = ( unsigned_t )( 1 ); 
#line 44 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_92 = 92;
#define LWIP0_SEM_5	(<>)

#line 44 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_92 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_92 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_92 = ( unsigned_t )( 1 ); 
#line 45 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_93 = 93;
#define LWIP0_SEM_6	(<>)

#line 45 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_93 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_93 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_93 = ( unsigned_t )( 1 ); 
#line 46 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_94 = 94;
#define LWIP0_SEM_7	(<>)

#line 46 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_94 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_94 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_94 = ( unsigned_t )( 1 ); 
#line 47 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_95 = 95;
#define LWIP0_SEM_8	(<>)

#line 47 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_95 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_95 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_95 = ( unsigned_t )( 1 ); 
#line 48 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_96 = 96;
#define LWIP0_SEM_9	(<>)

#line 48 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_96 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_96 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_96 = ( unsigned_t )( 1 ); 
#line 49 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_97 = 97;
#define LWIP0_SEM_10	(<>)

#line 49 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_97 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_97 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_97 = ( unsigned_t )( 1 ); 
#line 50 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_98 = 98;
#define LWIP0_SEM_11	(<>)

#line 50 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_98 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_98 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_98 = ( unsigned_t )( 1 ); 
#line 51 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_99 = 99;
#define LWIP0_SEM_12	(<>)

#line 51 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_99 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_99 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_99 = ( unsigned_t )( 1 ); 
#line 52 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_100 = 100;
#define LWIP0_SEM_13	(<>)

#line 52 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_100 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_100 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_100 = ( unsigned_t )( 1 ); 
#line 53 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_101 = 101;
#define LWIP0_SEM_14	(<>)

#line 53 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_101 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_101 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_101 = ( unsigned_t )( 1 ); 
#line 54 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_102 = 102;
#define LWIP0_SEM_15	(<>)

#line 54 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_102 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_102 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_102 = ( unsigned_t )( 1 ); 
#line 55 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_103 = 103;
#define LWIP0_SEM_16	(<>)

#line 55 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_103 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_103 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_103 = ( unsigned_t )( 1 ); 
#line 56 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_104 = 104;
#define LWIP0_SEM_17	(<>)

#line 56 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_104 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_104 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_104 = ( unsigned_t )( 1 ); 
#line 57 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_105 = 105;
#define LWIP0_SEM_18	(<>)

#line 57 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_105 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_105 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_105 = ( unsigned_t )( 1 ); 
#line 58 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_106 = 106;
#define LWIP0_SEM_19	(<>)

#line 58 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_106 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_106 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_106 = ( unsigned_t )( 1 ); 
#line 59 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_107 = 107;
#define LWIP0_SEM_20	(<>)

#line 59 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_107 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_107 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_107 = ( unsigned_t )( 1 ); 
#line 60 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_108 = 108;
#define LWIP0_SEM_21	(<>)

#line 60 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_108 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_108 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_108 = ( unsigned_t )( 1 ); 
#line 61 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_109 = 109;
#define LWIP0_SEM_22	(<>)

#line 61 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_109 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_109 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_109 = ( unsigned_t )( 1 ); 
#line 62 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_110 = 110;
#define LWIP0_SEM_23	(<>)

#line 62 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_110 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_110 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_110 = ( unsigned_t )( 1 ); 
#line 63 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_111 = 111;
#define LWIP0_SEM_24	(<>)

#line 63 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_111 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_111 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_111 = ( unsigned_t )( 1 ); 
#line 64 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_112 = 112;
#define LWIP0_SEM_25	(<>)

#line 64 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_112 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_112 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_112 = ( unsigned_t )( 1 ); 
#line 65 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_113 = 113;
#define LWIP0_SEM_26	(<>)

#line 65 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_113 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_113 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_113 = ( unsigned_t )( 1 ); 
#line 66 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_114 = 114;
#define LWIP0_SEM_27	(<>)

#line 66 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_114 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_114 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_114 = ( unsigned_t )( 1 ); 
#line 67 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_static_api_115 = 115;
#define LWIP0_SEM_N	(<>)

#line 67 "../../asp_baseplatform/lwip/lwip.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_115 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_115 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_115 = ( unsigned_t )( 1 ); /* #include "pinkit.h" */

#line 6 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_static_api_116 = 116;
#define I2CTRS_SEM	(<>)

#line 6 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_116 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_116 = ( unsigned_t )( 0 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_116 = ( unsigned_t )( 1 ); 
#line 7 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_static_api_117 = 117;
#define I2CLOC_SEM	(<>)

#line 7 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_117 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_117 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_117 = ( unsigned_t )( 1 ); 
#line 9 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_static_api_118 = 118;
#define PINKIT_TASK	(<>)

#line 9 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_118 = ( unsigned_t )( TA_ACT ); const signed_t TOPPERS_cfg_valueof_itskpri_118 = ( signed_t )( PINKIT_PRIORITY ); const unsigned_t TOPPERS_cfg_valueof_stksz_118 = ( unsigned_t )( PINKIT_STACK_SIZE ); 
#line 11 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_static_api_119 = 119;
const unsigned_t TOPPERS_cfg_valueof_isratr_119 = ( unsigned_t )( TA_NULL ); const unsigned_t TOPPERS_cfg_valueof_intno_119 = ( unsigned_t )( INTNO_I2CEV ); const signed_t TOPPERS_cfg_valueof_isrpri_119 = ( signed_t )( 1 ); 
#line 12 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_static_api_120 = 120;
const unsigned_t TOPPERS_cfg_valueof_intno_120 = ( unsigned_t )( INTNO_I2CEV ); const unsigned_t TOPPERS_cfg_valueof_intatr_120 = ( unsigned_t )( TA_ENAINT | INTATR_I2CEV ); const signed_t TOPPERS_cfg_valueof_intpri_120 = ( signed_t )( INTPRI_I2CEV ); 
#line 13 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_static_api_121 = 121;
const unsigned_t TOPPERS_cfg_valueof_isratr_121 = ( unsigned_t )( TA_NULL ); const unsigned_t TOPPERS_cfg_valueof_intno_121 = ( unsigned_t )( INTNO_I2CER ); const signed_t TOPPERS_cfg_valueof_isrpri_121 = ( signed_t )( 1 ); 
#line 14 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_static_api_122 = 122;
const unsigned_t TOPPERS_cfg_valueof_intno_122 = ( unsigned_t )( INTNO_I2CER ); const unsigned_t TOPPERS_cfg_valueof_intatr_122 = ( unsigned_t )( TA_ENAINT | INTATR_I2CER ); const signed_t TOPPERS_cfg_valueof_intpri_122 = ( signed_t )( INTPRI_I2CER ); 
#line 16 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_static_api_123 = 123;
const unsigned_t TOPPERS_cfg_valueof_isratr_123 = ( unsigned_t )( TA_NULL ); const unsigned_t TOPPERS_cfg_valueof_intno_123 = ( unsigned_t )( INTNO_DMAADC ); const signed_t TOPPERS_cfg_valueof_isrpri_123 = ( signed_t )( 1 ); 
#line 17 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_static_api_124 = 124;
const unsigned_t TOPPERS_cfg_valueof_intno_124 = ( unsigned_t )( INTNO_DMAADC ); const unsigned_t TOPPERS_cfg_valueof_intatr_124 = ( unsigned_t )( TA_ENAINT | INTATR_DMAADC ); const signed_t TOPPERS_cfg_valueof_intpri_124 = ( signed_t )( INTPRI_DMAADC ); 
#line 18 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_static_api_125 = 125;
const unsigned_t TOPPERS_cfg_valueof_inhno_125 = ( unsigned_t )( INHNO_ADC ); const unsigned_t TOPPERS_cfg_valueof_inhatr_125 = ( unsigned_t )( TA_NULL ); 
#line 19 "../pinkit/pinkit.cfg"
const unsigned_t TOPPERS_cfg_static_api_126 = 126;
const unsigned_t TOPPERS_cfg_valueof_intno_126 = ( unsigned_t )( INTNO_ADC ); const unsigned_t TOPPERS_cfg_valueof_intatr_126 = ( unsigned_t )( TA_ENAINT | INTATR_ADC ); const signed_t TOPPERS_cfg_valueof_intpri_126 = ( signed_t )( INTPRI_ADC ); /* #include "itron.h" */
/* #include "device.h" */
/* #include "emac.h" */
/* #include "main.h" */

#line 28 "../src/main.cfg"
const unsigned_t TOPPERS_cfg_static_api_127 = 127;
const unsigned_t TOPPERS_cfg_valueof_iniatr_127 = ( unsigned_t )( TA_NULL ); 
#line 29 "../src/main.cfg"
const unsigned_t TOPPERS_cfg_static_api_128 = 128;
const unsigned_t TOPPERS_cfg_valueof_iniatr_128 = ( unsigned_t )( TA_NULL ); 
#line 31 "../src/main.cfg"
const unsigned_t TOPPERS_cfg_static_api_129 = 129;
#define MAIN_TASK1	(<>)

#line 31 "../src/main.cfg"
const unsigned_t TOPPERS_cfg_valueof_tskatr_129 = ( unsigned_t )( TA_ACT ); const signed_t TOPPERS_cfg_valueof_itskpri_129 = ( signed_t )( MAIN_PRIORITY ); const unsigned_t TOPPERS_cfg_valueof_stksz_129 = ( unsigned_t )( STACK_SIZE ); 
#line 32 "../src/main.cfg"
const unsigned_t TOPPERS_cfg_static_api_130 = 130;
#define FLG_GMAC	(<>)

#line 32 "../src/main.cfg"
const unsigned_t TOPPERS_cfg_valueof_flgatr_130 = ( unsigned_t )( (TA_TFIFO|TA_WSGL) ); const unsigned_t TOPPERS_cfg_valueof_iflgptn_130 = ( unsigned_t )( 0 ); 
#line 33 "../src/main.cfg"
const unsigned_t TOPPERS_cfg_static_api_131 = 131;
#define SEM_GMAC_MDIO	(<>)

#line 33 "../src/main.cfg"
const unsigned_t TOPPERS_cfg_valueof_sematr_131 = ( unsigned_t )( TA_TPRI ); const unsigned_t TOPPERS_cfg_valueof_isemcnt_131 = ( unsigned_t )( 1 ); const unsigned_t TOPPERS_cfg_valueof_maxsem_131 = ( unsigned_t )( 1 ); 
#line 35 "../src/main.cfg"
const unsigned_t TOPPERS_cfg_static_api_132 = 132;
const unsigned_t TOPPERS_cfg_valueof_isratr_132 = ( unsigned_t )( TA_NULL ); const unsigned_t TOPPERS_cfg_valueof_intno_132 = ( unsigned_t )( INTNO_EMAC ); const signed_t TOPPERS_cfg_valueof_isrpri_132 = ( signed_t )( 1 ); 
#line 36 "../src/main.cfg"
const unsigned_t TOPPERS_cfg_static_api_133 = 133;
const unsigned_t TOPPERS_cfg_valueof_intno_133 = ( unsigned_t )( INTNO_EMAC ); const unsigned_t TOPPERS_cfg_valueof_intatr_133 = ( unsigned_t )( TA_NULL | INTATR_EMAC ); const signed_t TOPPERS_cfg_valueof_intpri_133 = ( signed_t )( INTPRI_EMAC ); 
