/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2011 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2015-2019 by TOPPERS PROJECT Educational Working Group.
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
 *  @(#) $Id$
 */
/*
 * 
 *  STM32F7XX:EMAC(LWIP用)ドライバ関数群
 *
 */
#include "kernel_impl.h"
#include <t_syslog.h>
#include <t_stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sil.h>
#include <target_syssvc.h>
#include <string.h>
#include "device.h"
#include "phyreg.h"
#include "emac.h"

/*
 *  SIL関数のマクロ定義
 */
#define sil_orw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) | (b))
#define sil_andw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) & ~(b))
#define sil_modw_mem(a, b, c)	sil_wrw_mem((a), (sil_rew_mem(a) & (~b)) | (c))

#define MIN(a, b) ((a) > (b) ? (b) : (a))

/*
 *  EMACポートIDから管理ブロックを取り出すためのマクロ
 */
#define INDEX_EMAC(emacid)      ((uint_t)((emacid) - 1))

/*
 *  AUTO NEGOTIATION判定
 */
#define is_autonego(h)          (((h)->linkvalue & EMAC_LINK_AUTONEGO) != 0)

/* MDIO の Ready 待ち時間(msec) */
#define EMAC_MDIO_READY_WAIT_MSEC   50

#define MII_CLKRANGE_150_250M	(0x10)


/*
 *  MIIモード定義
 */
#define EMAC_CTRL_PHYSEL_GMII_MII 0x0
#define EMAC_CTRL_PHYSEL_RGMII    0x1

/*
 *  DESCRIPTOR定義
 */
/*  TX DESCRIPTERステータスビット定義 */
#define DESC_TXSTS_OWNBYDMA     (1 << 31)		/* OWN bit: descriptor is owned by DMA engine */
#define DESC_TXSTS_TXINT        (1 << 30)		/* Interrupt on Completion */
#define DESC_TXSTS_TXLAST       (1 << 29)		/* Last Segment */
#define DESC_TXSTS_TXFIRST      (1 << 28)		/* First Segment */
#define DESC_TXSTS_TXCRCDIS     (1 << 27)		/* Disable CRC */

#define DESC_TXSTS_TXPADDIS     (1 << 26)		/* Disable Padding */
#define DESC_TXSTS_DMATTSE      (1 << 25)		/* Transmit Time Stamp Enable */
#define DESC_TXSTS_TXCHECKINSCTRL (3 << 22)
#define DESC_TXSTS_TXRINGEND    (1 << 21)		/* Transmit End of Ring */
#define DESC_TXSTS_TXCHAIN      (1 << 20)		/* Second Address Chained */
#define DESC_TXSTS_MSK          (0x1FFFF << 0)
#define DESC_TXSTS_IHE          (1 << 16)		/* IP Header Error */
#define DESC_TXSTS_ES           (1 << 15)		/* Error summary: OR of the following bits: UE || ED || EC || LCO || NC || LCA || FF || JT */
#define DESC_TXSTS_JT           (1 << 14)		/* Jabber Timeout */
#define DESC_TXSTS_FF           (1 << 13)		/* Frame Flushed: DMA/MTL flushed the frame due to SW flush */
#define DESC_TXSTS_PCE          (1 << 12)		/* Payload Checksum Error */
#define DESC_TXSTS_LCA          (1 << 11)		/* Loss of Carrier: carrier lost during transmission */
#define DESC_TXSTS_NC           (1 << 10)		/* No Carrier: no carrier signal from the transceiver */
#define DESC_TXSTS_LCO          (1 << 9)		/* Late Collision: transmission aborted due to collision */
#define DESC_TXSTS_EC           (1 << 8)		/* Excessive Collision: transmission aborted after 16 collisions */
#define DESC_TXSTS_VF           (1 << 7)		/* VLAN Frame */
#define DESC_TXSTS_CC           (15 << 3)		/* Collision Count */
#define DESC_TXSTS_ED           (1 << 2)		/* Excessive Deferral */
#define DESC_TXSTS_UF           (1 << 1)		/* Underflow Error: late data arrival from the memory */
#define DESC_TXSTS_DB           (1 << 0)		/* Deferred Bit */

/*  RX DESCRIPTERステータスビット定義 */
#define DESC_RXSTS_OWNBYDMA     (1 << 31)		/* OWN bit: descriptor is owned by DMA engine  */
#define DESC_RXSTS_DAFILTERFAIL (1 << 30)		/* DA Filter Fail for the rx frame  */
#define DESC_RXSTS_FRMLENMSK    (0x3FFF << 16)
#define DESC_RXSTS_FRMLENSHFT   (16)

#define DESC_RXSTS_ERROR        (1 << 15)		/* Error summary: OR of the following bits */
#define DESC_RXSTS_DESCERROR    (1 << 14)		/* Descriptor error: no more descriptors for receive frame */
#define DESC_RXSTS_SAFILTERFAIL (1 << 13)		/* SA Filter Fail for the received frame */
#define DESC_RXSTS_FRAMELE      (1 << 12)		/* Frame size not matching with length field */
#define DESC_RXSTS_OFERROR      (1 << 11)		/* Overflow Error: Frame was damaged due to buffer overflow */
#define DESC_RXSTS_RXVLANTAG    (1 << 10)		/* VLAN Tag: received frame is a VLAN frame */
#define DESC_RXSTS_RXFIRST      (1 << 9)		/* First descriptor of the frame */
#define DESC_RXSTS_RXLAST       (1 << 8)		/* Last descriptor of the frame */
#define DESC_RXSTS_IPV4HCE      (1 << 7)		/* IPC Checksum Error: Rx Ipv4 header checksum error */
#define DESC_RXSTS_RXCOLLISION  (1 << 6)		/* Late collision occurred during reception */
#define DESC_RXSTS_RXFRAMEETHER (1 << 5)		/* Frame type - Ethernet, otherwise 802.3 */
#define DESC_RXSTS_RXWATCHDOG   (1 << 4)		/* Receive Watchdog Timeout: watchdog timer expired during reception */
#define DESC_RXSTS_RXMIIERROR   (1 << 3)		/* Receive error: error reported by MII interface  */
#define DESC_RXSTS_RXDRIBBLING  (1 << 2)		/* Dribble bit error: frame contains non int multiple of 8 bits  */
#define DESC_RXSTS_RXCRC        (1 << 1)		/* CRC error */
#define DESC_RXSTS_MAMPCE       (1 << 0))		/* Rx MAC Address/Payload Checksum Error: Rx MAC address matched/ Rx Payload Checksum Error */

/*  TX DESCRIPTOR 制御ビット定義 */
#define DESC_TXCTRL_SIZE1SHFT   (0)
#define DESC_TXCTRL_SIZE1MASK   (0x1FFF << DESC_TXCTRL_SIZE1SHFT)
#define DESC_TXCTRL_SIZE2SHFT   (16)
#define DESC_TXCTRL_SIZE2MASK   (0x1FFF << DESC_TXCTRL_SIZE2SHFT)

/*  RX DESCRIPTOR 制御ビット定義 */
#define DESC_RXCTRL_RXINTDIS    (1 << 31)
#define DESC_RXCTRL_RXRINGEND   (1 << 15)
#define DESC_RXCTRL_RXCHAIN     (1 << 14)

#define DESC_RXCTRL_SIZE1SHFT   (0)
#define DESC_RXCTRL_SIZE1MASK   (0x1FFF << DESC_RXCTRL_SIZE1SHFT)
#define DESC_RXCTRL_SIZE2SHFT   (16)
#define DESC_RXCTRL_SIZE2MASK   (0x1FFF << DESC_RXCTRL_SIZE2SHFT)


/*
 *  EMAC制御設定
 */
#define ETH_MACCR_MASK          (ETH_MACCR_WD | ETH_MACCR_JD | ETH_MACCR_IFG | ETH_MACCR_CSD | \
								 ETH_MACCR_FES | ETH_MACCR_ROD | ETH_MACCR_LM | ETH_MACCR_DM | \
								 ETH_MACCR_IPCO | ETH_MACCR_RD | ETH_MACCR_APCS | ETH_MACCR_DC | \
								 ETH_MACCR_BL)
#define ETH_MACCR_INIT          (ETH_MACCR_IFG_96Bit | ETH_MACCR_FES | ETH_MACCR_DM | \
								 ETH_MACCR_IPCO | ETH_MACCR_RD | ETH_MACCR_BL_10)
#define ETH_MACFFR_INIT         (ETH_MACFFR_PCF_BlockAll)
#define ETH_MACFCR_MASK         (ETH_MACFCR_PT | ETH_MACFCR_ZQPD | ETH_MACFCR_PLT | ETH_MACFCR_UPFD | \
								 ETH_MACFCR_RFCE | ETH_MACFCR_TFCE)
#define ETH_MACFCR_INIT         ((0<<16) | ETH_MACFCR_ZQPD | ETH_MACFCR_PLT_Minus4)
#define ETH_MACVLANTR_INIT      ((0<<16) | 0x0000)

#define ETH_DMAOMR_MASK         (ETH_DMAOMR_DTCEFD | ETH_DMAOMR_RSF | ETH_DMAOMR_TSF | ETH_DMAOMR_DFRF | \
								 ETH_DMAOMR_ST | ETH_DMAOMR_TTC | ETH_DMAOMR_FEF | ETH_DMAOMR_FUGF | \
								 ETH_DMAOMR_RTC | ETH_DMAOMR_OSF)
#define ETH_DMAOMR_INIT         (ETH_DMAOMR_RSF | /* ETH_DMAOMR_TSF | */ ETH_DMAOMR_TTC_64Bytes | \
								 ETH_DMAOMR_RTC_64Bytes | ETH_DMAOMR_OSF)
#define ETH_DMABMR_INIT         (ETH_DMABMR_AAB | ETH_DMABMR_FB | ETH_DMABMR_RDP_32Beat | ETH_DMABMR_PBL_32Beat | \
								 ETH_DMABMR_EDE | (0<<2) | ETH_DMABMR_RTPR_1_1)

/*
 *  EMAC-DMA割り込み設定
 */
#define DMA_INTER_RXSTAT_ERROR  (ETH_DMAIER_ROIE | ETH_DMAIER_RBUIE | ETH_DMAIER_RPSIE | ETH_DMAIER_RWTIE)
#define DMA_INTR_NORMAL	        (ETH_DMAIER_NISE | ETH_DMAIER_RIE | ETH_DMAIER_TIE)
#define DMA_INTR_ABNORMAL       (ETH_DMAIER_AISE | ETH_DMAIER_TUIE | ETH_DMAIER_ETIE | \
								 ETH_DMAIER_FBEIE | ETH_DMAIER_TPSIE | ETH_DMAIER_TJTIE | \
								 DMA_INTER_RXSTAT_ERROR)
#define DMA_INTR_DEFAULT_MASK	(DMA_INTR_NORMAL | DMA_INTR_ABNORMAL | ETH_DMAIER_ERIE)

/*
 *  PHY FEATURES 定義
 */
#define ADVERTISE_MASK1         (ADVERTISE_10HALF   | ADVERTISE_10FULL    | \
							     ADVERTISE_100HALF  | ADVERTISE_100FULL   | \
								 ADVERTISE_100BASE4 | ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM)


/*
 *  PHY-GPIO定義
 */
#define GPIO_ETH_AF              0x0B
#define RMII_REF_CLK_PORT        TADR_GPIOA_BASE
#define RMII_REF_CLK_BIN         1
#define RMII_MDIO_PORT           TADR_GPIOA_BASE
#define RMII_MDIO_PIN            2
#define RMII_MDC_PORT            TADR_GPIOC_BASE
#define RMII_MDC_PIN             1
#define RMII_MII_CRS_DV_PORT     TADR_GPIOA_BASE
#define RMII_MII_CRS_DV_PIN      7
#define RMII_MII_RXD0_PORT       TADR_GPIOC_BASE
#define RMII_MII_RXD0_PIN        4
#define RMII_MII_RXD1_PORT       TADR_GPIOC_BASE
#define RMII_MII_RXD1_PIN        5
#define RMII_MII_RXER_PORT       TADR_GPIOG_BASE
#define RMII_MII_RXER_PIN        2
#define RMII_MII_TX_EN_PORT      TADR_GPIOG_BASE
#define RMII_MII_TX_EN_PIN       11
#define RMII_MII_TXD0_PORT       TADR_GPIOG_BASE
#define RMII_MII_TXD0_PIN        13
#ifndef RMII_MII_TXD1_PORT
#define RMII_MII_TXD1_PORT       TADR_GPIOB_BASE
#define RMII_MII_TXD1_PIN        13
#endif

/*
 *  EMACハードウェア設定構造体
 */
typedef struct _I2C_PortControlBlock{
	uint32_t              base;
	uint8_t               intno;
	uint8_t               physel_shift;
	uint8_t               phy_sel;
	uint8_t               phy_addr;
} EMAC_PortControlBlock;

/*
 *  EMACポート設定テーブル
 */
static const EMAC_PortControlBlock emac_pcb[NUM_MACPORT] = {
  {	TADR_ETH_BASE, IRQ_VECTOR_ETH, 23, EMAC_CTRL_PHYSEL_RGMII, 0}
};

/*
 *  EMAC ハンドラ
 */
static EMAC_Handle_t EMacHandle[NUM_MACPORT];

static uint32_t
emac_ether_clockdiv(void)
{
#if SysFreHCLK >= 20000000 && SysFreHCLK < 35000000
	/* CSR Clock Range between 20-35 MHz */
    retrun (uint32_t)ETH_MACMIIAR_CR_Div16;
#elif SysFreHCLK >= 35000000 && SysFreHCLK < 60000000
    /* CSR Clock Range between 35-60 MHz */
	return (uint32_t)ETH_MACMIIAR_CR_Div26;
#elif SysFreHCLK >= 60000000 && SysFreHCLK < 100000000
	/* CSR Clock Range between 60-100 MHz */
	return (uint32_t)ETH_MACMIIAR_CR_Div42;
#elif SysFreHCLK >= 100000000 && SysFreHCLK < 150000000
	/* CSR Clock Range between 100-150 MHz */
	return (uint32_t)ETH_MACMIIAR_CR_Div62;
#else
	/* CSR Clock Range between 150-216 MHz */
	return (uint32_t)ETH_MACMIIAR_CR_Div102;
#endif
}

/*
 *  MDIOのREADY待ち
 *  parameter1 base  EMACレジスタのベースアドレス
 *  return     E_OKで正常終了
 */
static ER
emac_mdio_wait_ready(uint32_t base)
{
	int     i;

	for (i = 0; i < (EMAC_MDIO_READY_WAIT_MSEC*10); i++) {
		if((sil_rew_mem((uint32_t *)(base+TOFF_ETH_MACMIIAR)) & ETH_MACMIIAR_MB) == 0)
			return E_OK;
		sil_dly_nse(100*1000);
	}
	syslog_0(LOG_ERROR, "EMAC:Error mdio timeout!");
	return E_TMOUT;
}

/*
 *  PHYレジスタへのデータ書き込み
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  parameter2 reg   レジスタ番号
 *  parameter3 val   書き込み値
 *  return     E_OKで正常終了
 */
static ER
emac_mdio_write(EMAC_Handle_t *hmac, int reg, int val)
{
	uint32_t miiaddr;
	ER       ercd = E_TMOUT;

	if(hmac->Init.semid != 0)
		wai_sem(hmac->Init.semid);

	miiaddr  = sil_rew_mem((uint32_t *)(hmac->base+TOFF_ETH_MACMIIAR)) & ETH_MACMIIAR_CR;
	miiaddr |= ((hmac->phy_addr << 11) & ETH_MACMIIAR_PA) | \
		  ((reg << 6) & ETH_MACMIIAR_MR) | ETH_MACMIIAR_MW;

	/*
	 *  READY待ち
	 */
	if(emac_mdio_wait_ready(hmac->base) == E_OK){
		/*
		 *  データ設定後、PHYアドレスセット
		 */
		sil_wrw_mem((uint32_t *)(hmac->base+TOFF_ETH_MACMIIDR), val);
		sil_wrw_mem((uint32_t *)(hmac->base+TOFF_ETH_MACMIIAR), (miiaddr | ETH_MACMIIAR_MB));

		/*
		 *  READY待ち
		 */
		if (emac_mdio_wait_ready(hmac->base) == E_OK)
			ercd = E_OK;
	}

	if(hmac->Init.semid != 0)
		sig_sem(hmac->Init.semid);
	return ercd;
}

/*
 *  PHYレジスタからのデータ読み込み
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  parameter2 reg   レジスタ番号
 *  return     0以上でレジスタ値
 */
static int
emac_mdio_read(EMAC_Handle_t *hmac, int reg)
{
	uint32_t miiaddr;
	int     val = -1;

	if(hmac->Init.semid != 0)
		wai_sem(hmac->Init.semid);

	miiaddr  = sil_rew_mem((uint32_t *)(hmac->base+TOFF_ETH_MACMIIAR)) & ETH_MACMIIAR_CR;
	miiaddr |= ((hmac->phy_addr << 11) & ETH_MACMIIAR_PA) | ((reg << 6) & ETH_MACMIIAR_MR);

	/*
	 *  READY待ち
	 */
	if(emac_mdio_wait_ready(hmac->base) == E_OK){
		/*
		 *  PHYアドレスセット
		 */
		sil_wrw_mem((uint32_t *)(hmac->base+TOFF_ETH_MACMIIAR), (miiaddr | ETH_MACMIIAR_MB));

		/*
		 *  READY待ちの後、データ取得
		 */
		if(emac_mdio_wait_ready(hmac->base) == E_OK)
			val = sil_rew_mem((uint32_t *)(hmac->base+TOFF_ETH_MACMIIDR)) & 0x0000ffffUL;
	}

	if(hmac->Init.semid != 0)
		sig_sem(hmac->Init.semid);
	return val;
}

/*
 *  PHYの初期設定
 *  parameter1 hmac   EMACハンドラへのポインタ
 *  parameter2 phyadr PHYアドレス(32以上ならば自動センス)
 *  return     E_OKで正常終了
 */
static ER
emac_phy_init(EMAC_Handle_t *hmac, uint8_t phy_addr)
{
	int  val;

	if(phy_addr < 0x20){
		hmac->phy_addr = phy_addr;
		val = emac_mdio_read(hmac, MII_PHYSID1);
		if((val > 0) && (val != 0xffff))
			return E_OK;
		else{
			syslog_1(LOG_ERROR, "EMAC:emac_phy_init Error unkown phy address(%d)", phy_addr);
			return E_ID;
		}
	}

	/*
	 *  自動センスモード
	 */
	for(phy_addr = 0; phy_addr < 0x20; phy_addr++){
		hmac->phy_addr = phy_addr;
		val = emac_mdio_read(hmac, MII_PHYSID1);
		if((val > 0) && (val != 0xffff))
			return E_OK;
	}
	syslog_0(LOG_ERROR, "EMAC:emac_phy_init Error unkown phy address");
	return E_ID;
}

/*
 *  PHYをリセットする
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  return     E_OKで正常終了
 */
static ER
emac_phy_reset(EMAC_Handle_t *hmac)
{
	int i, val;

	val = emac_mdio_read(hmac, MII_BMCR);
	val &= ~(BMCR_ISOLATE | BMCR_ANENABLE);
	val |= BMCR_RESET;
	emac_mdio_write(hmac, MII_BMCR, val);

	for(i = 0 ; i < 1000; i++){
		val = emac_mdio_read(hmac, MII_BMCR);
		if (val < 0)
			break;
		if ((val & BMCR_RESET) == 0) {
			if (val & BMCR_ISOLATE) {
				val &= (~BMCR_ISOLATE);
				emac_mdio_write(hmac, MII_BMCR, val);
			}
			return E_OK;
		}
		dly_tsk(1);
	}
	syslog_0(LOG_ERROR, "EMAC:emac_phy_reset Error spin reset timeout");
	return E_TMOUT;
}

/*
 *  PHYの状態を取り出す
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  return     PHYの状態値
 */
static uint32_t
emac_phy_features(EMAC_Handle_t *hmac)
{
	uint32_t   bmsr;
	uint32_t   mode = ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM;

	bmsr = emac_mdio_read(hmac, MII_BMSR);
	if((bmsr & BMSR_10HALF) != 0)
		mode |= ADVERTISE_10HALF;
	if((bmsr & BMSR_10FULL) != 0)
		mode |= ADVERTISE_10FULL;
	if((bmsr & BMSR_100HALF) != 0)
		mode |= ADVERTISE_100HALF;
	if((bmsr & BMSR_100FULL) != 0)
		mode |= ADVERTISE_100FULL;
	return mode;
}

/*
 *  PHY処理開始要求
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  parameter2 link  PHY設定値
 *  return     E_OKで正常終了
 */
static ER
emac_phy_start(EMAC_Handle_t *hmac, uint32_t link)
{
	ER       ercd;
	uint32_t id;
	uint32_t mode, val;
	int      speed = 0, duplex = 0;

	/*
	 *  PHYをリセットする
	 */
	if((ercd = emac_phy_reset(hmac)) != E_OK)
		return ercd;

	/*
	 *  PHY情報を取得する
	 */
	hmac->phy_features = emac_phy_features(hmac);
	syslog_1(LOG_INFO, "EMAC:emac_phy_start phy featurs[%08x]", hmac->phy_features);

	/*
	 *  PHYのチップIDの確認
	 */
	id  = (emac_mdio_read(hmac, MII_PHYSID1) << 16);
	id |= (emac_mdio_read(hmac, MII_PHYSID2) & 0xffff);

	syslog_2(LOG_INFO, "EMAC:emac_phy_start PHY-ID[OUI&Model:0x%x, Rev:0x%x]", id & ~0xf, id & 0xf);

	/*
	 *  オプション設定があれば、実行
	 */
	if(hmac->phy_opt_init)
		hmac->phy_opt_init(hmac);

	/*
	 *  LINKパラメータを設定
	 */
	link &= ~EMAC_LINK_CONNECT;
	hmac->linkvalue = link;

	switch(link){
	case EMAC_LINK_10BASE_FULL:
		duplex = BMCR_FULLDPLX;
	case EMAC_LINK_10BASE_HALF:
		speed = BMCR_SPEED10;
		break;
	case EMAC_LINK_100BASE_FULL:
		duplex = BMCR_FULLDPLX;
	case EMAC_LINK_100BASE_HALF:
		speed = BMCR_SPEED100;
		break;
	case EMAC_LINK_AUTONEGO:
	default:
		/* Setup Auto-negotiation */
		hmac->linkvalue |= EMAC_LINK_AUTONEGO;
		mode = hmac->phy_features;
		val = emac_mdio_read(hmac, MII_ADVERTISE);
		val &= ~ADVERTISE_MASK1;
		val |= mode & ADVERTISE_MASK1;
		emac_mdio_write(hmac, MII_ADVERTISE, val);
		break;
	}

	val = emac_mdio_read(hmac, MII_BMCR);
	val &= ~(BMCR_FULLDPLX | BMCR_SPEED100 | BMCR_ANENABLE);
	val |= is_autonego(hmac) ? (BMCR_ANENABLE | BMCR_ANRESTART) : (speed | duplex);
	emac_mdio_write(hmac, MII_BMCR, val);
	return E_OK;
}

/*
 *  AUTO-NEGOTIATION設定時のLINK状態の取得
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  return     LINK状態
 */
static uint32_t
emac_phy_link_autonego(EMAC_Handle_t *hmac)
{
	uint32_t  link_detect;
	uint32_t  lpa;

	lpa = emac_mdio_read(hmac, MII_LPA) & emac_mdio_read(hmac, MII_ADVERTISE);
	if(lpa & LPA_100FULL)
		link_detect = EMAC_LINK_100BASE_FULL;
	else if(lpa & LPA_100HALF)
		link_detect = EMAC_LINK_100BASE_HALF;
	else if(lpa & LPA_10FULL)
		link_detect = EMAC_LINK_10BASE_FULL;
	else
		link_detect = EMAC_LINK_10BASE_HALF;
	return link_detect;
}

/*
 *  マニュアル設定時のLINK状態の取得
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  return     LINK状態
 */
static uint32_t
emac_phy_link_manual(EMAC_Handle_t *hmac)
{
	uint32_t    link_detect;
	int     bmcr;

	bmcr = emac_mdio_read(hmac, MII_BMCR);

	if(bmcr & BMCR_SPEED100)
		link_detect = EMAC_SPEED_100BASE;
	else
		link_detect = EMAC_SPEED_10BASE;
	link_detect |= (bmcr & BMCR_FULLDPLX) ? EMAC_DUPLEX_FULL : EMAC_DUPLEX_HALF;
	return link_detect;
}

/*
 *  DESCRIPTER領域の初期化
 *  parameter1 hmac  EMACハンドラへのポインタ
 */
static void
emac_bd_set(EMAC_Handle_t *hmac)
{
	uint32_t   *desc, *next;
	uint8_t    *pdata = hmac->Init.databuffer;
	int        i;

	/*
	 *  DESCRIPTOR/BUFFER領域ののキャッシュをクリア
	 */
	desc  = (uint32_t *)hmac->Init.quebuffer;
	next  = (uint32_t *)(desc + DESC_TXRX_SIZE);

	hmac->rx_top_desc = desc;
	for(i = 0 ; i < hmac->Init.rxquecount ; i++) {
		desc[DESC_TXRX_BUFFER] = (uint32_t)pdata;
		desc[DESC_TXRX_NEXT]   = (uint32_t)next;
		desc[DESC_TXRX_CONTROL] = 
			(hmac->Init.paketbufsize & DESC_RXCTRL_SIZE1MASK) | \
				      DESC_RXCTRL_RXCHAIN;
		desc[DESC_TXRX_STATUS] = DESC_RXSTS_OWNBYDMA;

		pdata += hmac->Init.paketbufsize;
		desc  += DESC_TXRX_SIZE;
		next  += DESC_TXRX_SIZE;
	}
	*(desc - DESC_TXRX_SIZE + DESC_TXRX_NEXT)
			= (uint32_t)(hmac->rx_top_desc);

	hmac->tx_top_desc = desc;
	for(i = 0 ; i < hmac->Init.txquecount ; i++){
		desc[DESC_TXRX_BUFFER] = (uint32_t)pdata;
		desc[DESC_TXRX_NEXT]   = (uint32_t)next;
		desc[DESC_TXRX_STATUS] &= ~(DESC_TXSTS_TXINT | DESC_TXSTS_TXLAST |
				DESC_TXSTS_TXFIRST | DESC_TXSTS_TXCRCDIS | \
				DESC_TXSTS_TXCHECKINSCTRL | \
				DESC_TXSTS_TXRINGEND | DESC_TXSTS_TXPADDIS);
		desc[DESC_TXRX_STATUS] |= DESC_TXSTS_TXCHAIN | DESC_TXSTS_TXCHECKINSCTRL;
		desc[DESC_TXRX_CONTROL] = 0;
		desc[DESC_TXRX_STATUS] &= ~(DESC_TXSTS_MSK | DESC_TXSTS_OWNBYDMA);

		pdata += hmac->Init.paketbufsize;
		desc  += DESC_TXRX_SIZE;
		next  += DESC_TXRX_SIZE;
	}
	*(desc - DESC_TXRX_SIZE + DESC_TXRX_NEXT)
			= (uint32_t)(hmac->tx_top_desc);
}


/*
 *  EMACデバイスの初期化
 *  parameter1 portid EMACのポートID
 *  parameter2 ini    初期化構造体へのポインタ
 *  return            ハンドラへのポインタ
 */
EMAC_Handle_t *
emac_init(ID portid, EMAC_Init_t *ini)
{
	GPIO_Init_t GPIO_Init_Data;
	const EMAC_PortControlBlock *pcb;
	EMAC_Handle_t *hmac;
	uint32_t base, no, tmp;
	uint32_t timeout = 3000;

	if(portid == 0 || portid > NUM_MACPORT)
		return NULL;

	no = INDEX_EMAC(portid);
	pcb = &emac_pcb[no];
	hmac = &EMacHandle[no];

	hmac->base     = base = pcb->base;
	hmac->intno    = pcb->intno;
	memcpy(&hmac->Init, ini, sizeof(EMAC_Init_t));

	/*
	 *  PHY GPIO クロック設定
	 */
	sil_orw_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR), RCC_AHB1ENR_GPIOAEN);
	tmp = sil_rew_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR));
	sil_orw_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR), RCC_AHB1ENR_GPIOCEN);
	tmp = sil_rew_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR));
	sil_orw_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR), RCC_AHB1ENR_GPIOGEN);
	tmp = sil_rew_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR));
#ifdef RMII_MII_TXD1_CLOCK
	sil_orw_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR), RMII_MII_TXD1_CLOCK);
	tmp = sil_rew_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR));
#endif

	/*
	 *  PHY GPIOピン設定
	 */
	GPIO_Init_Data.mode      = GPIO_MODE_AF;
	GPIO_Init_Data.pull      = GPIO_OTYPE_PP;
	GPIO_Init_Data.otype     = GPIO_NOPULL;
	GPIO_Init_Data.speed     = GPIO_SPEED_HIGH;
	GPIO_Init_Data.alternate = GPIO_ETH_AF;
	gpio_setup(RMII_REF_CLK_PORT, &GPIO_Init_Data, RMII_REF_CLK_BIN);
	gpio_setup(RMII_MDIO_PORT, &GPIO_Init_Data, RMII_MDIO_PIN);
	gpio_setup(RMII_MDC_PORT, &GPIO_Init_Data, RMII_MDC_PIN);
	gpio_setup(RMII_MII_CRS_DV_PORT, &GPIO_Init_Data, RMII_MII_CRS_DV_PIN);
	gpio_setup(RMII_MII_RXD0_PORT, &GPIO_Init_Data, RMII_MII_RXD0_PIN);
	gpio_setup(RMII_MII_RXD1_PORT, &GPIO_Init_Data, RMII_MII_RXD1_PIN);
	gpio_setup(RMII_MII_RXER_PORT, &GPIO_Init_Data, RMII_MII_RXER_PIN);
	gpio_setup(RMII_MII_TX_EN_PORT, &GPIO_Init_Data, RMII_MII_TX_EN_PIN);
	gpio_setup(RMII_MII_TXD0_PORT, &GPIO_Init_Data, RMII_MII_TXD0_PIN);
	gpio_setup(RMII_MII_TXD1_PORT, &GPIO_Init_Data, RMII_MII_TXD1_PIN);

	/*
	 *  EMACクロック設定
	 */
	sil_orw_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR), RCC_AHB1ENR_ETHMACEN);
	tmp = sil_rew_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR));
	sil_orw_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR), RCC_AHB1ENR_ETHMACTXEN);
	tmp = sil_rew_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR));
	sil_orw_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR), RCC_AHB1ENR_ETHMACRXEN);
	tmp = sil_rew_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR));

	dis_int(hmac->intno);
	sil_orw_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_APB2ENR), RCC_APB2ENR_SYSCFGEN);
	sil_andw_mem((uint32_t *)(TADR_SYSCFG_BASE+TOFF_SYSCFG_PMC), 1<<(pcb->physel_shift));
	sil_orw_mem((uint32_t *)(TADR_SYSCFG_BASE+TOFF_SYSCFG_PMC), (pcb->phy_sel)<<(pcb->physel_shift));

	/*
	 *  MACをリセット
	 */
	sil_orw_mem((uint32_t *)(base+TOFF_ETH_DMABMR), ETH_DMABMR_SR);
	while(timeout > 0){
		if((sil_rew_mem((uint32_t *)(base+TOFF_ETH_DMABMR)) & ETH_DMABMR_SR) == 0)
			break;
		dly_tsk(1);
		timeout--;
	}

	/* Get the ETHERNET MACMIIAR value */
	tmp= sil_rew_mem((uint32_t *)(base+TOFF_ETH_MACMIIAR));
	/* Clear CSR Clock Range CR[2:0] bits */
	tmp &= ~ETH_MACMIIAR_CR;
	tmp |= emac_ether_clockdiv();
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACMIIAR), tmp);

	/*
	 *  DESCRIPTORの初期化
	 */
	emac_bd_set(hmac);

	if(emac_phy_init(hmac, pcb->phy_addr) != E_OK)
		return NULL;
	hmac->macid = portid;
	return hmac;
}

/*
 *  EMAC終了処理
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  return     E_OKで正常終了
 */
ER
emac_deinit(EMAC_Handle_t *hmac)
{
	if(hmac == NULL || hmac->macid == 0 || hmac->macid > NUM_MACPORT)
		return E_OBJ;
	dis_int(hmac->intno);

	/*
	 *  EMACクロック停止
	 */
	sil_andw_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR), RCC_AHB1ENR_ETHMACEN);
	sil_andw_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR), RCC_AHB1ENR_ETHMACTXEN);
	sil_andw_mem((uint32_t *)(TADR_RCC_BASE+TOFF_RCC_AHB1ENR), RCC_AHB1ENR_ETHMACRXEN);

	hmac->macid = 0;
	return E_OK;
}

/*
 *  EMAC送信要求
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  parameter2 num   送信パケット数
 */
static void
emac_send_req(EMAC_Handle_t *hmac, int num)
{
	volatile uint32_t *desc, *top;
	uint32_t base = hmac->base;

	if(num > 0){
		top = desc = hmac->tx_cur_desc;
		desc = (uint32_t *)(desc[DESC_TXRX_NEXT]);
		dis_int(hmac->intno);
		if(hmac->tx_wait == 0)
			sil_andw_mem((uint32_t *)(hmac->base+TOFF_ETH_DMAOMR), ETH_DMAOMR_ST);
		while (--num) {
			/*
			 *  連続DESCRIPTORの送信DMA KICK
			 */
			desc[DESC_TXRX_STATUS] |= DESC_TXSTS_OWNBYDMA;
			hmac->tx_wait++;
			desc = (uint32_t *)(desc[DESC_TXRX_NEXT]);
		}
		/*
		 *  先頭のDESCRIPTORの送信DMA KICK
		 */
		top[DESC_TXRX_STATUS] |= DESC_TXSTS_OWNBYDMA;
		hmac->tx_wait++;
		hmac->tx_cur_desc = desc;
		ena_int(hmac->intno);
	}
	else
		sil_andw_mem((uint32_t *)(hmac->base+TOFF_ETH_DMAOMR), ETH_DMAOMR_ST);
	if((sil_rew_mem((uint32_t *)(base+TOFF_ETH_DMASR)) & ETH_DMASR_TBUS) != 0){
		sil_wrw_mem((uint32_t *)(base+TOFF_ETH_DMASR), ETH_DMASR_TBUS);
		sil_wrw_mem((uint32_t *)(base+TOFF_ETH_DMATPDR), 0);
	}
	sil_orw_mem((uint32_t *)(base+TOFF_ETH_DMAOMR), ETH_DMAOMR_ST);
}

/*
 *  EMAC送信処理
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  parameter2 pktp  送信パケット構造体(LWIP型)
 *  return     E_OKで正常終了
 */
ER
emac_send(EMAC_Handle_t *hmac, struct emac_pkt *pktp)
{
	int      num, len, tlen, retry, i;
	int      start = 1;
	int      actlen = 0;
	ER       ercd = E_OK;
	volatile uint32_t *desc;
	uint32_t info_word = 0;
	volatile uint8_t  *pdata;
	volatile uint8_t  *s, *d;

	if(hmac == NULL || pktp == NULL)
		return E_PAR;

	desc = hmac->tx_cur_desc;
	s = pktp->payload;
	len = pktp->len;

	for(num = 0 ; num < hmac->Init.txquecount && pktp != NULL ; num++){
		if(pktp->len == 0)
			break;

		/*
		 *  送信DESCRIPTORの開放待ち
		 */
		if((desc[DESC_TXRX_STATUS] & DESC_TXSTS_OWNBYDMA) != 0){
			for (retry = 0; retry < 3; retry++) {
				/* 送信ディスクリプタがフルのとき、開放されるのを待つ */
				dly_tsk(10);
				/* 再度チェックを行う */
				if((desc[DESC_TXRX_STATUS] & DESC_TXSTS_OWNBYDMA) == 0)
					break;
				/* 念のため、送信の開始を再要求する */
				emac_send_req(hmac, 0);
			}
			if((desc[DESC_TXRX_STATUS] & DESC_TXSTS_OWNBYDMA) != 0){
				syslog_1(LOG_ERROR, "EMAC:emac_send retry(%d) no descriptor", retry+1);
				ercd = E_TMOUT;
				break;
			}
		}

		/*
		 *  送信データをバッファに書き込む
		 */
		pdata = d = (volatile uint8_t *)(desc[DESC_TXRX_BUFFER]);
		while(actlen < hmac->Init.paketbufsize){
			tlen = MIN(len, (hmac->Init.paketbufsize - actlen));
			for(i = 0 ; i < tlen ; i++)
				*d++ = *s++;
			actlen += tlen;
			len    -= tlen;
			if(len == 0){
				if((pktp = pktp->next) == NULL)
					break;
				if(pktp->len == 0)
					break;
				s = pktp->payload;
				len = pktp->len;
			}
		}
		syslog_4(LOG_DEBUG, "EMAC:emac_send(%d) desc[%08x] buf[%08x] actlen[%04x]", num, desc, pdata, actlen);

		/*
		 *  送信サイズと送信モードをセット
		 */
		desc[DESC_TXRX_CONTROL] = (actlen << DESC_TXCTRL_SIZE1SHFT) & DESC_TXCTRL_SIZE1MASK;

		info_word  = desc[DESC_TXRX_STATUS];
		info_word &= ~(DESC_TXSTS_MSK | DESC_TXSTS_TXFIRST | DESC_TXSTS_TXLAST);
		info_word |= DESC_TXSTS_TXINT;

		if (start) {
			start = 0;
			info_word |= DESC_TXSTS_TXFIRST;
		}
		/*
		 *  次のデータ確認と終了マーク設定
		 */
		if(pktp == NULL || pktp->len == 0)
			info_word |= DESC_TXSTS_TXLAST;
		desc[DESC_TXRX_STATUS] = info_word;

		/*
		 *  次のDESCRIPTORへ更新する
		 */
		desc = (uint32_t *)(desc[DESC_TXRX_NEXT]);
	}

	/* パケットの終端までセットしていないときはエラー応答 */
	if((info_word & DESC_TXSTS_TXLAST) == 0){
		syslog_0(LOG_ERROR, "EMAC:emac_send last error !");
		return -1;
	}

	/* 送信を要求する */
	if(num > 0)
		emac_send_req(hmac, num);
	return ercd;
}

/*
 *  次の受信DESCRIPTORを取得する
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  return     次のDESCRIPTORへのポインタ
 */
volatile static uint32_t *
emac_recv_desc_next(EMAC_Handle_t *hmac)
{
	volatile uint32_t *desc;

	desc = hmac->rx_cur_desc;
	desc[DESC_TXRX_CONTROL] = 
		(hmac->Init.paketbufsize & DESC_RXCTRL_SIZE1MASK) | \
			      DESC_RXCTRL_RXCHAIN;
	desc[DESC_TXRX_STATUS]  = DESC_RXSTS_OWNBYDMA;
	desc = (uint32_t *)(desc[DESC_TXRX_NEXT]);
	hmac->rx_cur_desc = desc;
	return desc;
}

/*
 *  受信の先頭パケットのDESCRIPTORを探す
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  return     NULLでなければ、受信の先頭DESCRIPTORへのポインタ
 */
volatile static uint32_t *
emac_recv_sop_find(EMAC_Handle_t *hmac)
{
	int      num;
	volatile uint32_t *desc = hmac->rx_cur_desc;
	uint32_t info_word, length;

	for(num = 0 ; num < hmac->Init.rxquecount ; num++) {
		info_word = desc[DESC_TXRX_STATUS];

		/*
		 *  受信データなしのときは、終了する
		 */
		if ((info_word & DESC_RXSTS_OWNBYDMA) != 0)
			break;

		/*
		 * 受信データあり場合、Start of Packet のチェックを行う
		 */
		length = (info_word & DESC_RXSTS_FRMLENMSK) >> DESC_RXSTS_FRMLENSHFT;
		if (info_word & DESC_RXSTS_RXFIRST) {
			if(length >= ETHER_HDR_SIZE)
				return desc;
		}
		else
			hmac->ifShortPkts++;

		/*
		 * 次のDESCRIPTORを取得する
		 */
		desc = emac_recv_desc_next(hmac);
	}
	return NULL;
}

/*
 *  受信データ長の取り出し
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  return     0でなければ、受信パケット長
 */
int
emac_recv_length(EMAC_Handle_t *hmac)
{
	int      num, length = 0;
	volatile uint32_t *desc;
	uint32_t info_word;

	/*
	 *  受信パケットの確認し、ない場合は終了
	 */
	if((desc = emac_recv_sop_find(hmac)) == NULL)
		return 0;

	for(num = 0 ; num < hmac->Init.rxquecount ; num++){
		info_word = desc[DESC_TXRX_STATUS];

		/*  何も受信していないときは、終了する */
		if ((info_word & DESC_RXSTS_OWNBYDMA) != 0)
			break;

		/*  受信データ長を取得する */
		length += (desc[DESC_TXRX_STATUS] & DESC_RXSTS_FRMLENMSK) >> DESC_RXSTS_FRMLENSHFT;

		/*
		 *  End of Packetならば、終了
		 */
		if(info_word & DESC_RXSTS_RXLAST)
			break;

		/*
		 *  次のDESCRIPTORを取得する
		 */
		desc = (uint32_t *)(desc[DESC_TXRX_NEXT]);
	}
	return length;
}

/*
 *  受信パケットの取得
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  parameter2 pktp  受信データ格納パケット構造体(LWIP型)
 *  return     E_OKならば、受信パケット長
 */
ER
emac_recv(EMAC_Handle_t *hmac, struct emac_pkt *pktp)
{
	int      num, actlen, i;
	volatile uint32_t *desc;
	volatile uint8_t  *b, *pdata;
	uint32_t info_word, len, tlen;

	if(pktp != NULL){
		b = pktp->payload;
		len = pktp->len;
	}
	else{
		b = NULL;
		len = 0;
	}

	sil_wrw_mem((uint32_t *)(hmac->base+TOFF_ETH_DMASR), ETH_DMASR_RS | DMA_INTER_RXSTAT_ERROR);

	/*
	 *  受信パケットの確認し、ない場合はエラー終了
	 */
	if((desc = emac_recv_sop_find(hmac)) == NULL)
		return E_NOMEM;

	for(num = 0 ; num < hmac->Init.rxquecount ; num++){
		info_word = desc[DESC_TXRX_STATUS];

		/*
		 *  受信データがない場合、終了する
		 */
		if((info_word & DESC_RXSTS_OWNBYDMA) != 0)
			break;

		/*
		 *  パケット格納領域がある場合は受信パケットを取得する
		 */
		if(b != NULL){
			/* 受信データを取得する */
			pdata = (uint8_t *)(desc[DESC_TXRX_BUFFER]);
			actlen = (desc[DESC_TXRX_STATUS] & DESC_RXSTS_FRMLENMSK) >> DESC_RXSTS_FRMLENSHFT;
			syslog_5(LOG_DEBUG, "EMAC:emac_recv num(%d) desc[%08x] b[%08x] pdata[%08x] actlen[%08x] !", num, desc, b, pdata, actlen);
			syslog_4(LOG_DEBUG, "  12[%02x] 13[%02x] 14[%02x] 15[%02x]", pdata[12], pdata[13], pdata[14], pdata[15]);
			while(actlen > 0){
				tlen = MIN(len, actlen);
				for(i = 0 ; i < tlen ; i++)
					*b++ = *pdata++;
				actlen -= tlen;
				len    -= tlen;
				if(len == 0 /*&& actlen > 0*/){
					if((pktp = pktp->next) == NULL){
						b = NULL;
						break;
					}
					b = pktp->payload;
					if((len = pktp->len) == 0){
						b = NULL;
						break;
					}
				}
			}
		}
		/*
		 *  次のDESCRIPTORの取得と更新
		 */
		desc = emac_recv_desc_next(hmac);

		/*
		 *  End of Packetの場合、正常終了
		 */
		if((info_word & DESC_RXSTS_RXLAST) != 0)
			return E_OK;
	}
	return E_NOMEM;
}

/*
 *  DESCRIPTORのリセット
 *  parameter1 hmac  EMACハンドラへのポインタ
 */
static void
emac_desc_reset(EMAC_Handle_t *hmac)
{
	volatile uint32_t *desc;
	int      i;

	/*
	 *  送信ディスクリプタの初期化
	 */
	desc = hmac->tx_top_desc;
	hmac->tx_cur_desc = desc;
	for(i = 0 ; i < hmac->Init.txquecount ; i++) {
		desc[DESC_TXRX_STATUS] &= ~(DESC_TXSTS_TXINT | DESC_TXSTS_TXLAST |
				DESC_TXSTS_TXFIRST | DESC_TXSTS_TXCRCDIS | \
				DESC_TXSTS_TXCHECKINSCTRL | \
				DESC_TXSTS_TXRINGEND | DESC_TXSTS_TXPADDIS);
		desc[DESC_TXRX_STATUS] |= DESC_TXSTS_TXCHAIN | DESC_TXSTS_TXCHECKINSCTRL;
		desc[DESC_TXRX_CONTROL] = 0;
		desc[DESC_TXRX_STATUS] &= ~(DESC_TXSTS_MSK | DESC_TXSTS_OWNBYDMA);
		desc += DESC_TXRX_SIZE;
	}

	/*
	 *  受信ディスクリプタの初期化
	 */
	desc = hmac->rx_top_desc;
	hmac->rx_cur_desc = desc;
	for (i = 0 ; i < hmac->Init.rxquecount ; i++) {
		desc[DESC_TXRX_CONTROL] = 
			(hmac->Init.paketbufsize & DESC_RXCTRL_SIZE1MASK) | DESC_RXCTRL_RXCHAIN;
		desc[DESC_TXRX_STATUS] = DESC_RXSTS_OWNBYDMA;
		desc += DESC_TXRX_SIZE;
	}

	sil_wrw_mem((uint32_t *)(hmac->base+TOFF_ETH_DMATDLAR), (uint32_t)(hmac->tx_top_desc));
	sil_wrw_mem((uint32_t *)(hmac->base+TOFF_ETH_DMARDLAR), (uint32_t)(hmac->rx_top_desc));
}

/*
 *  ETHERNET通信を開始する
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  parameter2 link  通信モード
 *  return     E_OKで正常終了
 */
ER
emac_start(EMAC_Handle_t *hmac, uint32_t link)
{
	uint32_t   base, tmp;

	if(hmac == NULL)
		return E_PAR;
	base = hmac->base;

	/*
	 *  PHYスタート
	 */
	emac_phy_start(hmac, link);

	/*
	 *  送受信を有効設定
	 */
	sil_orw_mem((uint32_t *)(base+TOFF_ETH_DMAOMR), ETH_DMAOMR_SR);

	/*
	 *  割り込みマスク設定
	 */
	sil_orw_mem((uint32_t *)(base+TOFF_ETH_DMAIER), DMA_INTR_DEFAULT_MASK);

	/*
	 *  割り込み許可
	 */
	ena_int(hmac->intno);

	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_DMASR), 0x0001FFFF);
	sil_orw_mem((uint32_t *)(base+TOFF_ETH_MACCR), (ETH_MACCR_TE | ETH_MACCR_RE));
	tmp = sil_rew_mem((uint32_t *)(base+TOFF_ETH_MACCR));
	dly_tsk(1);
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACCR), tmp);
	return E_OK;
}

/*
 *  ETHERNET通信を停止する
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  parameter2 disable 送受信の無効化設定(1で無効化)
 *  return     E_OKで正常終了
 */
ER
emac_stop(EMAC_Handle_t *hmac, int disable)
{
	uint32_t base, tmp;
	ER       ercd = E_OK;

	if(hmac == NULL)
		return E_PAR;
	base = hmac->base;

	/*
	 * 割り込み停止
	 */
	dis_int(hmac->intno);
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_DMAIER), 0);

	if(disable == 1) {
		/*
		 *  送受信を無効に設定
		 */
		sil_andw_mem((uint32_t *)(base+TOFF_ETH_MACCR), ETH_MACCR_RE);
		tmp = sil_rew_mem((uint32_t *)(base+TOFF_ETH_MACCR));
		dly_tsk(1);
		sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACCR), tmp);

		dly_tsk(200);

		/*
		 *  受信済みパケットを破棄
		 */
		while(ercd == E_OK){
			ercd = emac_recv(hmac, NULL);
			dly_tsk(1);
			syslog_1(LOG_NOTICE, "EMAC:emac_stop flush result(%d) !", ercd);
		}
		sil_andw_mem((uint32_t *)(base+TOFF_ETH_DMAOMR), ETH_DMAOMR_SR);
		sil_andw_mem((uint32_t *)(base+TOFF_ETH_MACCR), ETH_MACCR_TE);
		tmp = sil_rew_mem((uint32_t *)(base+TOFF_ETH_MACCR));
		dly_tsk(1);
		sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACCR), tmp);
		sil_andw_mem((uint32_t *)(base+TOFF_ETH_DMAOMR), ETH_DMAOMR_ST);
	}
	return E_OK;
}

/*
 *  LINK状態を取得する
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  return     LINK状態(0で未接続)
 */
uint32_t
emac_link_detect(EMAC_Handle_t *hmac)
{
	uint32_t link_detect = 0;
	int      bmsr;

	/* 最初は空読み */
	emac_mdio_read(hmac, MII_BMSR);
	bmsr = emac_mdio_read(hmac, MII_BMSR);
	syslog_2(LOG_INFO, "EMAC:phy_link_detect[%08x] phydev->auto_nego(%d)", bmsr, is_autonego(hmac));

	if((bmsr >= 0) && (bmsr & BMSR_LSTATUS)){
		link_detect = is_autonego(hmac)
				? emac_phy_link_autonego(hmac)
				: emac_phy_link_manual(hmac);
	}
	return link_detect;
}

/*
 *  EMACの制御設定を行う
 *  parameter1 hmac  EMACハンドラへのポインタ
 *  parameter2 link  EMAC設定モード
 *  return     E_OKで正常終了
 */
ER
emac_link_mode_set(EMAC_Handle_t *hmac, uint32_t link)
{
	uint32_t base, conf;

	if(hmac == NULL || link == 0)
		return E_PAR;
	base = hmac->base;
	conf = sil_rew_mem((uint32_t *)(base+TOFF_ETH_MACCR));

	if((link & EMAC_SPEED_MASK) != EMAC_SPEED_10BASE)
		conf |= ETH_MACCR_FES;
	else
		conf &= ~ETH_MACCR_FES;

	if((link & EMAC_DUPLEX_MASK) == EMAC_DUPLEX_FULL)
		conf |= ETH_MACCR_DM;
	else
		conf &= ~ETH_MACCR_DM;
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACCR), conf);
	return E_OK;
}

/*
 *  EMACをリセットする
 *  parameter1 hmac   EMACハンドラへのポインタ
 *  parameter2 macadr MACアドレスデータへのポインタ
 *  return     E_OKで正常終了
 */
ER
emac_reset(EMAC_Handle_t *hmac, uint8_t *mac_addr)
{
	uint32_t  timeout = 3000;
	uint32_t  base, tmp;
	uint32_t  macid_hi, macid_lo;
	uint8_t   *b = (uint8_t *)&mac_addr[0];

	if(hmac == NULL || mac_addr == NULL)
		return E_PAR;
	base = hmac->base;

	/*
	 *  MACをリセット
	 */
	sil_orw_mem((uint32_t *)(base+TOFF_ETH_DMABMR), ETH_DMABMR_SR);
	while(timeout > 0){
		if((sil_rew_mem((uint32_t *)(base+TOFF_ETH_DMABMR)) & ETH_DMABMR_SR) == 0)
			break;
		dly_tsk(1);
		timeout--;
	}

	/* Get the ETHERNET MACMIIAR value */
	tmp= sil_rew_mem((uint32_t *)(base+TOFF_ETH_MACMIIAR));
	/* Clear CSR Clock Range CR[2:0] bits */
	tmp &= ~ETH_MACMIIAR_CR;
	tmp |= emac_ether_clockdiv();
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACMIIAR), tmp);

	/*
	 *  送受信の停止
	 */
	emac_stop(hmac, 0);

	/*
	 *  MACアドレスの登録
	 */
	macid_lo = b[0] + (b[1] << 8) + (b[2] << 16) + (b[3] << 24);
	macid_hi = b[4] + (b[5] << 8);
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACA0HR), macid_hi);
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACA0LR), macid_lo);

	/*
	 *  MACの設定
	 */
	sil_modw_mem((uint32_t *)(base+TOFF_ETH_MACCR), ETH_MACCR_MASK, ETH_MACCR_INIT);
	tmp = sil_rew_mem((uint32_t *)(base+TOFF_ETH_MACCR));
	dly_tsk(1);
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACCR), tmp);

	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACFFR), ETH_MACFFR_INIT);
	tmp = sil_rew_mem((uint32_t *)(base+TOFF_ETH_MACFFR));
	dly_tsk(1);
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACFFR), tmp);

	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACHTHR), 0x00000000);
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACHTLR), 0x00000000);

	sil_modw_mem((uint32_t *)(base+TOFF_ETH_MACFCR), ETH_MACFCR_MASK, ETH_MACFCR_INIT);
	tmp = sil_rew_mem((uint32_t *)(base+TOFF_ETH_MACFCR));
	dly_tsk(1);
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACFCR), tmp);

	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACVLANTR), ETH_MACVLANTR_INIT);

	sil_modw_mem((uint32_t *)(base+TOFF_ETH_DMAOMR), ETH_DMAOMR_MASK, ETH_DMAOMR_INIT);
	tmp = sil_rew_mem((uint32_t *)(base+TOFF_ETH_DMAOMR));
	dly_tsk(1);
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_DMAOMR), tmp);

	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_DMABMR), ETH_DMABMR_INIT);
	tmp = sil_rew_mem((uint32_t *)(base+TOFF_ETH_DMABMR));
	dly_tsk(1);
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_DMABMR), tmp);

	/*
	 *  送受信DESCRIPTERリセット
	 */
	emac_desc_reset(hmac);

	sil_orw_mem((uint32_t *)(base+TOFF_ETH_MACCR), (ETH_MACCR_TE | ETH_MACCR_RE));
	tmp = sil_rew_mem((uint32_t *)(base+TOFF_ETH_MACCR));
	dly_tsk(1);
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_MACCR), tmp);

	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_DMASR), 0x0001FFFF);
	sil_orw_mem((uint32_t *)(base+TOFF_ETH_DMAOMR), ETH_DMAOMR_FTF);
	tmp = sil_rew_mem((uint32_t *)(base+TOFF_ETH_DMAOMR));
	dly_tsk(1);
	sil_wrw_mem((uint32_t *)(base+TOFF_ETH_DMAOMR), tmp);
	sil_orw_mem((uint32_t *)(base+TOFF_ETH_DMAOMR), ETH_DMAOMR_SR);
	return E_OK;
}


/*
 *  EMAC DMA用割り込み処理
 */
static int
emac_dma_interrupt(EMAC_Handle_t *hmac, uint32_t status)
{
	int  event = 0;

	if((status & 0x1FFFF) == 0)
		return 0;
	event = 0;

	/*
	 *  ABNORMAL割り込み
	 */
	if((status & ETH_DMASR_AIS) != 0) {
		if((status & ETH_DMASR_TUS) != 0)
			hmac->tx_undeflow_irq++;
		if((status & ETH_DMASR_TJTS) != 0)
			hmac->tx_jabber_irq++;
		if((status & ETH_DMASR_ROS) != 0)
			hmac->rx_overflow_irq++;
		if((status & ETH_DMASR_RBUS) != 0)
			hmac->rx_buf_unav_irq++;
		if((status & ETH_DMASR_RPSS) != 0)
			hmac->rx_process_stopped_irq++;
		if((status & ETH_DMASR_RWTS) != 0)
			hmac->rx_watchdog_irq++;
		if((status & ETH_DMASR_ETS) != 0)
			hmac->tx_early_irq++;
		if((status & ETH_DMASR_TPSS) != 0)
			hmac->tx_process_stopped_irq++;
		if((status & ETH_DMASR_FBES) != 0){
			hmac->fatal_bus_error_irq++;
			event |= 0x0004;
		}
	}

	/*
	 *  TX/RX NORMAL割り込み
	 */
	if((status & ETH_DMASR_NIS) != 0)
		hmac->normal_irq_n++;
	if((status & ETH_DMASR_RS) != 0) {
		uint32_t value = sil_rew_mem((uint32_t *)(hmac->base+TOFF_ETH_DMAIER));
		/* to schedule NAPI on real RIE event. */
		if((value & ETH_DMAIER_RIE) != 0){
			hmac->rx_normal_irq_n++;
			if(hmac->emacevent != NULL)
				hmac->emacevent(hmac, 0, RECV_EVENT_DETECT);
		}
	}
	if((status & ETH_DMASR_TS) != 0){
		/*
		 * 送信割り込みを通知する
		 */
		if(hmac->tx_wait != 0){
			if(hmac->emacevent != NULL)
				hmac->emacevent(hmac, 0, SEND_EVENT_DETECT);
			if(hmac->tx_wait > 0)
				hmac->tx_wait--;
		}
	}
	if((status & ETH_DMASR_ERS) != 0)
		hmac->rx_early_irq++;
	return event;
}

/*
 *  EMAC割り込みサービスルーチン
 */
void
emac_hps_isr(intptr_t exinf)
{
	EMAC_Handle_t *hmac = &EMacHandle[INDEX_EMAC((uint32_t)exinf)];
	uint32_t base, dstatus;
	uint32_t event;

	base = hmac->base;
	dstatus = sil_rew_mem((uint32_t *)(base+TOFF_ETH_DMASR));
	syslog_1(LOG_DEBUG, "emac_hps_isr SR[%08x]", dstatus);
	if(dstatus != 0){
		event = emac_dma_interrupt(hmac, dstatus);
		sil_wrw_mem((uint32_t *)(base+TOFF_ETH_DMASR), (dstatus & 0x0001FFFB));
		/*
		 *  DMAエラーがあった場合、通知
		 */
		if(event != 0){
			syslog_2(LOG_ERROR, "EMAC:emac_isr id(%d) DMA error[%08x] !", (int)exinf, event);
			if(hmac->emacevent != NULL)
				hmac->emacevent(hmac, 0, RESET_EVNT_DETECT);
		}
	}
}

