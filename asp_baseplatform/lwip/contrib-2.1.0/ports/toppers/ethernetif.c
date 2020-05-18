/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2003-2015 by Ryosuke Takeuchi
 *                     GJ Business Division RICOH COMPANY,LTD. JAPAN
 *  Copyright (C) 2017-2019 by TOPPERS PROJECT Educational Working Group.
 * 
 *  上記著作権者は，Free Software Foundation によって公表されている 
 *  GNU General Public License の Version 2 に記述されている条件か，以
 *  下の(1)～(4)の条件を満たす場合に限り，本ソフトウェア（本ソフトウェ
 *  アを改変したものを含む．以下同じ）を使用・複製・改変・再配布（以下，
 *  利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを再利用可能なバイナリコード（リロケータブルオブ
 *      ジェクトファイルやライブラリなど）の形で利用する場合には，利用
 *      に伴うドキュメント（利用者マニュアルなど）に，上記の著作権表示，
 *      この利用条件および下記の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを再利用不可能なバイナリコードの形または機器に組
 *      み込んだ形で利用する場合には，次のいずれかの条件を満たすこと．
 *    (a) 利用に伴うドキュメント（利用者マニュアルなど）に，上記の著作
 *        権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 利用の形態を，別に定める方法によって，上記著作権者に報告する
 *        こと．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者を免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者は，
 *  本ソフトウェアに関して，その適用可能性も含めて，いかなる保証も行わ
 *  ない．また，本ソフトウェアの利用により直接的または間接的に生じたい
 *  かなる損害に関しても，その責任を負わない．
 * 
 *  @(#) $Id$
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

#include "kernel_impl.h"
#include <t_syslog.h>
#include <t_stdlib.h>
#include <sil.h>
#include <string.h>
#include <target_syssvc.h>
#include "kernel_cfg.h"

#include "lwip/opt.h"

#include "lwip/def.h"
#include "lwip/ip.h"
#include "lwip/mem.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "lwip/ethip6.h"

#include "emac.h"

/*
 *  DESCRIPTOR定義
 */
#ifndef EMAC_PKTBUFSIZE
#define EMAC_PKTBUFSIZE 1536
#endif
#ifndef NUM_RX_DESC
#define NUM_RX_DESC     32		/* 受信キューの個数 */
#endif
#ifndef NUM_TX_DESC
#define NUM_TX_DESC     24		/* 送信キューの個数 */
#endif
#define NUM_ALL_DESC    (NUM_RX_DESC + NUM_TX_DESC)

static uint32_t event_count, event_value;
#ifndef EMAC_DATA_ATTRIBUTE
static uint8_t netqueue[(NUM_ALL_DESC * EMAC_QUE_SIZE)] __attribute__ ((aligned (32)));
static uint8_t netbuffer[(NUM_ALL_DESC * EMAC_PKTBUFSIZE)] __attribute__ ((aligned (32));
#else
static uint8_t netqueue[(NUM_ALL_DESC * EMAC_QUE_SIZE)] __attribute__ (EMAC_DATA_ATTRIBUTE) __attribute__ ((aligned (32)));
static uint8_t netbuffer[(NUM_ALL_DESC * EMAC_PKTBUFSIZE)] __attribute__ (EMAC_DATA_ATTRIBUTE) __attribute__ ((aligned (32)));
#endif

/* Define those to better describe your network interface. */
#define IFNAME0 'e'
#define IFNAME1 'n'


/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
struct ethernetif {
  struct netif    *netif;
  EMAC_Handle_t   *emacdev;
  struct eth_addr *ethaddr;
  uint8_t         enable;
  /* Add whatever per-interface state that is needed here. */
};

/* Forward declarations. */
static void ethernetif_input(struct netif *netif);
static void ethernet_thread(void *arg);

/**
 *  callback from emac
 */
static void
emac_event(EMAC_Handle_t *hemac, int mode, uint32_t event)
{
  event_value |= event;
  if((++event_count % 200) == 0){
	syslog_3(LOG_NOTICE, "emac_event(%08x, %08x) count(%d)", hemac, event_value, event_count);
    event_value = 0;
  }
  if(mode == 0)
    iset_flg(FLG_GMAC, event);
  else
    set_flg(FLG_GMAC, event);
}

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void
low_level_init(struct netif *netif)
{
  EMAC_Init_t Init;
  struct ethernetif *ethernetif = netif->state;
  EMAC_Handle_t *emacdev;

  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set MAC hardware address */
  netif->hwaddr[0] = 0x02;
  netif->hwaddr[1] = 0x12;
  netif->hwaddr[2] = 0x34;
  netif->hwaddr[3] = 0x56;
  netif->hwaddr[4] = 0x78;
  netif->hwaddr[5] = 0xab;

  /* maximum transfer unit */
  netif->mtu = 1500;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;

#if LWIP_IPV6 && LWIP_IPV6_MLD
  /*
   * For hardware/netifs that implement MAC filtering.
   * All-nodes link-local is handled by default, so we must let the hardware know
   * to allow multicast packets in.
   * Should set mld_mac_filter previously. */
  if (netif->mld_mac_filter != NULL) {
    ip6_addr_t ip6_allnodes_ll;
    ip6_addr_set_allnodes_linklocal(&ip6_allnodes_ll);
    netif->mld_mac_filter(netif, &ip6_allnodes_ll, NETIF_ADD_MAC_FILTER);
  }
#endif /* LWIP_IPV6 && LWIP_IPV6_MLD */

  /* Do whatever else is needed to initialize interface. */

//  netif_set_link_up(netif);

  ethernetif->netif = netif;
  Init.paketbufsize = EMAC_PKTBUFSIZE;
  Init.rxquecount   = NUM_RX_DESC;
  Init.txquecount   = NUM_TX_DESC;
  Init.quebuffer    = netqueue;
  Init.databuffer   = netbuffer;
  Init.semid        = SEM_GMAC_MDIO;
  emacdev = emac_init(MACDEF_PORTID, &Init);
  ethernetif->emacdev = emacdev;
  emacdev->emacevent = emac_event;

  /* EMAC のリセット */
  emac_reset(emacdev, (uint8_t *)ethernetif->ethaddr);

  /* 送受信を開始する */
#ifdef LWIP_AUTONEGO
  emac_start(emacdev, EMAC_LINK_AUTONEGO);
#else	/* LWIP_AUTONEGO */
  emac_start(emacdev, EMAC_LINK_100BASE_FULL);
#endif	/* LWIP_AUTONEGO */
  ethernetif->enable = 1;
  sys_thread_new("ethernet_thread", ethernet_thread, netif, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
static err_t
low_level_output(struct netif *netif, struct pbuf *p)
{
  struct ethernetif *ethernetif = netif->state;
  err_t  err = ERR_OK;

#if ETH_PAD_SIZE
  pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

  if(emac_send(ethernetif->emacdev, (struct emac_pkt *)p) != 0)
    err = ERR_MEM;

  MIB2_STATS_NETIF_ADD(netif, ifoutoctets, p->tot_len);
  if (((u8_t*)p->payload)[0] & 1) {
    /* broadcast or multicast packet*/
    MIB2_STATS_NETIF_INC(netif, ifoutnucastpkts);
  } else {
    /* unicast packet */
    MIB2_STATS_NETIF_INC(netif, ifoutucastpkts);
  }
  /* increase ifoutdiscards or ifouterrors on error */

#if ETH_PAD_SIZE
  pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

  LINK_STATS_INC(link.xmit);

  return err;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *
low_level_input(struct netif *netif)
{
  struct ethernetif *ethernetif = (struct ethernetif *)netif->state;
  struct pbuf *p;
  int    len;
  ER     ercd;

  /* Obtain the size of the packet and put it into the "len"
     variable. */
  len = emac_recv_length(ethernetif->emacdev);
  if(len <= 0)
    return NULL;

#if ETH_PAD_SIZE
  len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif

  /* We allocate a pbuf chain of pbufs from the pool. */
  p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

  if (p != NULL) {

#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

    ercd = emac_recv(ethernetif->emacdev, (struct emac_pkt *)p);
    if(ercd != E_OK)
      syslog_1(LOG_ERROR, "low_level_input emac_recv error(%d) !", ercd);

    MIB2_STATS_NETIF_ADD(netif, ifinoctets, p->tot_len);
    if (((u8_t*)p->payload)[0] & 1) {
      /* broadcast or multicast packet*/
      MIB2_STATS_NETIF_INC(netif, ifinnucastpkts);
    } else {
      /* unicast packet*/
      MIB2_STATS_NETIF_INC(netif, ifinucastpkts);
    }
#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

    LINK_STATS_INC(link.recv);
  } else {
    emac_recv(ethernetif->emacdev, NULL);
    LINK_STATS_INC(link.memerr);
    LINK_STATS_INC(link.drop);
    MIB2_STATS_NETIF_INC(netif, ifindiscards);
  }

  return p;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
static void
ethernetif_input(struct netif *netif)
{
  struct pbuf *p;

  /* move received packet into a new pbuf */
  p = low_level_input(netif);
  /* if no packet could be read, silently ignore this */
  if (p != NULL) {
    /* pass all packets to ethernet_input, which decides what packets it supports */
    if (netif->input(p, netif) != ERR_OK) {
      LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
      pbuf_free(p);
      p = NULL;
    }
  }
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t
ethernetif_init(struct netif *netif)
{
  struct ethernetif *ethernetif;

  LWIP_ASSERT("netif != NULL", (netif != NULL));

  ethernetif = mem_malloc(sizeof(struct ethernetif));
  if (ethernetif == NULL) {
    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_init: out of memory\n"));
    return ERR_MEM;
  }

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  netif->state = ethernetif;

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */
  netif->linkoutput = low_level_output;
  netif->mtu = 1500;

  ethernetif->ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);

  low_level_init(netif);

  return ERR_OK;
}

/**
 * ethernet polling thread
 */
static void
ethernet_thread(void *arg)
{
  struct netif *netif;
  struct ethernetif *ethernetif;
  FLGPTN flgptn;
  ER     ercd;
  uint32_t rlink;
  int    len;

  netif = (struct netif *)arg;
  ethernetif = (struct ethernetif *)netif->state;
  dly_tsk(1000);

  while(1) {
    ercd = twai_flg(FLG_GMAC, 0x00ff, TWF_ORW, &flgptn, 100);
    if(ercd == E_OK){
      clr_flg(FLG_GMAC, ~flgptn);
      if((flgptn & RESET_EVNT_DETECT) != 0){
        emac_reset(ethernetif->emacdev, (uint8_t *)ethernetif->ethaddr);
        continue;
      }
    }

    rlink = emac_link_detect(ethernetif->emacdev);
    if(rlink == 0 && (netif->flags & NETIF_FLAG_LINK_UP)){
      syslog_2(LOG_NOTICE, "ethernetif:unlink[%08x][%08x] !", rlink, netif->flags);
      netif_set_link_down(netif);
    }
    else if(rlink != 0 && !(netif->flags & NETIF_FLAG_LINK_UP)){
      /* Interface mode の設定 */
      emac_link_mode_set(ethernetif->emacdev, rlink);
      syslog_2(LOG_NOTICE, "ethernetif:link[%08x][%08x] !", rlink, netif->flags);
      netif_set_link_up(netif);
    }

    len = emac_recv_length(ethernetif->emacdev);

    /* Wait for a packet to arrive. */
    if(len > 0) {
      /* Handle incoming packet. */
      ethernetif_input(netif);
    }
  }
}

