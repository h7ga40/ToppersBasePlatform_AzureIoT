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
 *  STMicroelectronics EMACデバイスドライバの外部宣言
 *
 */

#ifndef _EMAC_H_
#define _EMAC_H_

#ifdef __cplusplus
 extern "C" {
#endif

/*
 *  MACポート定義
 */
#define MAC1_PORTID             1
#define NUM_MACPORT             1
#define MACDEF_PORTID           MAC1_PORTID

/*
 *  MAC EVENT定義
 */
#define PHY_EVENT_DETECT        (1<<0)
#define SEND_EVENT_DETECT       (1<<1)
#define RECV_EVENT_DETECT       (1<<2)
#define RESET_EVNT_DETECT       (1<<7)

/*
 *  ETHERNETアドレス長
 */
#define	ADLEN_ETHER	6
#define ETHER_HDR_SIZE	((ADLEN_ETHER*2)+2)

/*
 *  LINK設定定義
 */
#define EMAC_LINK_CONNECT       0x00080000
#define EMAC_LINK_AUTONEGO      0x00010000
#define EMAC_DUPLEX_HALF        0x00000000			/* HALF DUPLEX */
#define EMAC_DUPLEX_FULL        0x00000100			/* FULL DUPLEX */
#define EMAC_DUPLEX_MASK        EMAC_DUPLEX_FULL	/* DUPLEX MASK */

#define EMAC_SPEED_MASK         0x000000FF			/* LINK SPEED MASK */
#define EMAC_SPEED_10BASE       0x00000003			/* LINK SPEED 10BASE */
#define EMAC_SPEED_100BASE      0x00000004			/* LINK SPEED 100BASE */
#define EMAC_SPEED_1000BASE     0x00000005			/* LINK SPEED 1000BASE */

#define EMAC_LINK_10BASE2       0x00000001			/* 10Base2 */
#define EMAC_LINK_10BASE5       0x00000002			/* 10Base5 */
#define EMAC_LINK_10BASE_HALF   (EMAC_SPEED_10BASE   | EMAC_DUPLEX_HALF)
#define EMAC_LINK_10BASE_FULL   (EMAC_SPEED_10BASE   | EMAC_DUPLEX_FULL)
#define EMAC_LINK_100BASE_HALF  (EMAC_SPEED_100BASE  | EMAC_DUPLEX_HALF)
#define EMAC_LINK_100BASE_FULL  (EMAC_SPEED_100BASE  | EMAC_DUPLEX_FULL)
#define EMAC_LINK_1000BASE_HALF (EMAC_SPEED_1000BASE | EMAC_DUPLEX_HALF)
#define EMAC_LINK_1000BASE_FULL (EMAC_SPEED_1000BASE | EMAC_DUPLEX_FULL)

#define LINK_SPEED_OF_YOUR_NETIF_IN_BPS  100000000

/*
 *  DESCRIPTOR定義
 */
#define NUM_RX_DESC     8		/* 受信キューの個数 */
#define NUM_TX_DESC     8		/* 送信キューの個数 */
#define EMAC_DATA_ATTRIBUTE     (section(".ethDataSection"))

/*
 *  DESCRIPTOR COMMON定義
 */
#define DESC_TXRX_STATUS        0
#define DESC_TXRX_CONTROL       1
#define DESC_TXRX_BUFFER        2
#define DESC_TXRX_NEXT          3
#define DESC_TXRX_EXTSTATUS     4
#define DESC_TXRX_TIMESTAMPLOW  6
#define DESC_TXRX_TIMESTAMPHIGH 7
#define DESC_TXRX_SIZE          8
#define EMAC_QUE_SIZE           (DESC_TXRX_SIZE * 4)

/*
 *  EMACコンフュギュレーション構造体定義
 */
typedef struct
{
	uint32_t              paketbufsize;		/* EMAC パケットバッファサイズ */
	uint32_t              rxquecount;		/* EMAC 受信ディスクリプタサイズ */
	uint32_t              txquecount;		/* EMAC 送信ディスクリプタサイズ */
	void                  *quebuffer;		/* EMAC ディスクリプタ先頭アドレス */
	void                  *databuffer;		/* EMAC データ領域先頭アドレス */
	ID                    semid;			/* EMAC 通信用セマフォ値 */
}EMAC_Init_t;

/*
 *  EMACハンドラ定義
 */
typedef struct EMAC_Handle EMAC_Handle_t;
struct EMAC_Handle {
	uint32_t     base;
	EMAC_Init_t  Init;
	uint8_t      tx_wait;
	uint8_t      intno;
	uint8_t      macid;
	uint8_t      phy_addr;
	uint32_t     phy_features;
	uint32_t     linkvalue;

	volatile uint32_t *tx_top_desc;
	volatile uint32_t *tx_cur_desc;
	volatile uint32_t *rx_top_desc;
	volatile uint32_t *rx_cur_desc;
	void         (*emacevent)(EMAC_Handle_t *hmac, int, uint32_t);
	void         (*phy_opt_init)(EMAC_Handle_t *hmac);

	uint32_t     ifShortPkts;
	uint32_t     mmc_tx_irq_n;
	uint32_t     mmc_rx_irq_n;
	uint32_t     mmc_rx_csum_offload_irq_n;
	uint32_t     irq_receive_pmt_irq_n;
	uint32_t     irq_tx_path_in_lpi_mode_n;
	uint32_t     irq_tx_path_exit_lpi_mode_n;
	uint32_t     irq_rx_path_in_lpi_mode_n;
	uint32_t     irq_rx_path_exit_lpi_mode_n;

	uint32_t     normal_irq_n;
	uint32_t     fatal_bus_error_irq;
	uint32_t     tx_undeflow_irq;
	uint32_t     tx_jabber_irq;
	uint32_t     tx_early_irq;
	uint32_t     tx_process_stopped_irq;
	uint32_t     rx_normal_irq_n;
	uint32_t     rx_early_irq;
	uint32_t     rx_overflow_irq;
	uint32_t     rx_buf_unav_irq;
	uint32_t     rx_process_stopped_irq;
	uint32_t     rx_watchdog_irq;
};

/*
 *  LWIP用パケットバッファ構造体
 */
struct emac_pkt {
	struct emac_pkt *next;					/* next pbuf chain */
	void         *payload;					/* pointer to the data buffer */
	uint16_t     tot_len;					/* total length of this buffer and all next buffers in chain */
	uint16_t     len;						/* length of this buffer */
};


extern EMAC_Handle_t *emac_init(ID portid, EMAC_Init_t *ini);
extern ER   emac_deinit(EMAC_Handle_t *hmac);
extern ER   emac_send(EMAC_Handle_t *hmac, struct emac_pkt *pktp);
extern int  emac_recv_length(EMAC_Handle_t *hmac);
extern ER   emac_recv(EMAC_Handle_t *hmac, struct emac_pkt *pktp);
extern ER   emac_start(EMAC_Handle_t *hmac, uint32_t link_mode);
extern ER   emac_stop(EMAC_Handle_t *hmac, int disable);
extern uint32_t emac_link_detect(EMAC_Handle_t *hmac);
extern ER   emac_link_mode_set(EMAC_Handle_t *hmac, uint32_t link_detect);
extern ER   emac_reset(EMAC_Handle_t *hmac, uint8_t *mac_addr);
extern void emac_hps_isr(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif /* _EMAC_H_ */

