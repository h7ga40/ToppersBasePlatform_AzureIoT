/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
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
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
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
 *  サンプルプログラムの本体
 */

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <sil.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#ifdef USE_STORAGEDEVICE
#include "storagedevice.h"
#endif	/* USE_STORAGEDEVICE */
#include "device.h"
#include "sdmmc.h"
#include "kernel_cfg.h"
#include "main.h"
#include "client.h"

#include "lwip/opt.h"

#include "lwip/init.h"

#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"

#include "lwip/ip_addr.h"

#include "lwip/dns.h"
#include "lwip/dhcp.h"

#include "lwip/stats.h"

#include "lwip/tcp.h"
#include "lwip/inet_chksum.h"

#include "lwip/tcpip.h"
#include "lwip/sockets.h"


#if 0
#include "netif/pcapif.h"
#endif

#ifndef LWIP_HAVE_SLIPIF
#define LWIP_HAVE_SLIPIF 0
#endif
#if LWIP_HAVE_SLIPIF
#include "netif/slipif.h"
#define SLIP_PTY_TEST 1
#endif

#if PPP_SUPPORT
#include "netif/ppp/pppos.h"
#include "lwip/sio.h"
#define PPP_PTY_TEST 1
#include <termios.h>
#endif

#include "lwip/ip_addr.h"

#include "lwip/apps/httpd.h"
#include "apps/udpecho/udpecho.h"
#include "apps/tcpecho/tcpecho.h"
#include "apps/shell/shell.h"
#include "apps/chargen/chargen.h"
#include "apps/netio/netio.h"
#include "apps/ping/ping.h"
#include "lwip/apps/netbiosns.h"
#include "lwip/apps/mdns.h"
#include "lwip/apps/sntp.h"
#include "lwip/apps/snmp.h"
#include "lwip/apps/snmp_mib2.h"
#include "examples/snmp/snmp_private_mib/private_mib.h"
#include "lwip/apps/tftp_server.h"

#if LWIP_RAW
#include "lwip/icmp.h"
#include "lwip/raw.h"
#endif

err_t ethernetif_init(struct netif *netif);

/*
 *  サービスコールのエラーのログ出力
 */
Inline void
svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}

#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

#define HEAP_AREA_SIZE 16*1024
/*static*/ uint32_t heap_area[HEAP_AREA_SIZE] __attribute__((aligned(4096)));

void *heap_param[2] = {
	(void *)heap_area,
	(void *)(HEAP_AREA_SIZE*sizeof(uint32_t))
};


#ifdef _STORAGEDEVICE_H_
static void storagedev_notice(void *psdev, bool_t sw)
{
	syslog_2(LOG_NOTICE, "SD-CARD[%08x] on/off(%d) !", psdev, sw);
}

static uint32_t get_fat_time(void)
{
	int    reqtime = 0;

	reqtime = 0x00210000;
	return reqtime;
}
#endif	/* _STORAGEDEVICE_H_ */

#if 1	/* ROI DEBUG */
void sw_int(void)
{
	syslog_0(LOG_NOTICE, "## sw_int() ##");
}
#endif	/* ROI DEBUG */

uint32_t LWIP_RAND(void)
{
	return (uint32_t)rand();
}

#if LWIP_SNMP
static const struct snmp_mib *mibs[] = {
	&mib2,
	&mib_private
};
#endif /* LWIP_SNMP */

#if LWIP_IPV4
/* (manual) host IP configuration */
static ip_addr_t ipaddr, netmask, gw;
#endif /* LWIP_IPV4 */

struct netif netif;

/* ping out destination cmd option */
static ip_addr_t ping_addr;

static bool_t tcpip_init_flag = false;

/* nonstatic debug cmd option, exported in lwipopts.h */
unsigned char debug_flags;

static void init_netifs(void);

#if 0
static void
tcp_debug_timeout(void *data)
{
  LWIP_UNUSED_ARG(data);
#if TCP_DEBUG
  tcp_debug_print_pcbs();
#endif /* TCP_DEBUG */
  sys_timeout(5000, tcp_debug_timeout, NULL);
}
#endif

/*
 * gmtime_r関数
 */
struct tm *gmtime_r(const time_t *pt, struct tm *ptm);

void
sntp_set_system_time(u32_t sec)
{
	struct tm current_time_val;
	time_t current_time = (time_t)sec;

	gmtime_r(&current_time, &current_time_val);
	rtc_set_time((struct tm2 *)&current_time_val);

	current_time = 0;
	memset(&current_time_val, 0, sizeof(current_time_val));

	current_time = time(NULL);
	//gmtime_r(&current_time, &current_time_val);
	//printf("%04d/%02d/%02d %02d:%02d:%02d\n",
	//	current_time_val.tm_year+1970, current_time_val.tm_mon, current_time_val.tm_mday,
	//	current_time_val.tm_hour, current_time_val.tm_min, current_time_val.tm_sec);
	printf("%s\n", ctime(&current_time));
}

#if LWIP_MDNS_RESPONDER
static void
srv_txt(struct mdns_service *service, void *txt_userdata)
{
	err_t res;
	LWIP_UNUSED_ARG(txt_userdata);

	res = mdns_resp_add_service_txtitem(service, "path=/", 6);
	LWIP_ERROR("mdns add service txt failed\n", (res == ERR_OK), return);
}
#endif

#if LWIP_UDP

static void*
tftp_open(const char* fname, const char* mode, u8_t is_write)
{
#ifdef _STORAGEDEVICE_H_
	LWIP_UNUSED_ARG(mode);

	if(is_write)
		return (void*)fopen(fname, "wb");
	else
		return (void*)fopen(fname, "rb");
#else	/* _STORAGEDEVICE_H_ */
	return NULL;
#endif	/* _STORAGEDEVICE_H_ */
}

static void 
tftp_close(void* handle)
{
#ifdef _STORAGEDEVICE_H_
	fclose((FILE*)handle);
#endif	/* _STORAGEDEVICE_H_ */
}

static int
tftp_read(void* handle, void* buf, int bytes)
{
#ifdef _STORAGEDEVICE_H_
	int rlen = fread(buf, 1, bytes, (FILE*)handle);
	if(rlen <= 0)
		return -1;
	else
		return rlen;
#else	/* _STORAGEDEVICE_H_ */
	return -1;
#endif	/* _STORAGEDEVICE_H_ */
}

static int
tftp_write(void* handle, struct pbuf* p)
{
#ifdef _STORAGEDEVICE_H_
	while(p != NULL){
		if(fwrite(p->payload, 1, p->len, (FILE*)handle) != (size_t)p->len)
			return -1;
		p = p->next;
	}
	return 0;
#else	/* _STORAGEDEVICE_H_ */
	return -1;
#endif	/* _STORAGEDEVICE_H_ */
}

static const struct tftp_context tftp = {
	tftp_open,
	tftp_close,
	tftp_read,
	tftp_write
};

#endif /* LWIP_UDP */

static void
tcpip_init_done(void *arg)
{
	sys_sem_t *sem;
	sem = (sys_sem_t *)arg;

	init_netifs();

#if LWIP_IPV4
	netbiosns_set_name("toppershost");
	netbiosns_init();
#endif /* LWIP_IPV4 */

	sntp_setoperatingmode(SNTP_OPMODE_POLL);
#if LWIP_DHCP
	sntp_servermode_dhcp(1); /* get SNTP server via DHCP */
#else /* LWIP_DHCP */
#if LWIP_IPV4
	//sntp_setserver(0, netif_ip_gw4(&netif));
#endif /* LWIP_IPV4 */
#endif /* LWIP_DHCP */
	sntp_setservername(0, "ntp.nict.jp");
	sntp_init();

#if LWIP_SNMP
	lwip_privmib_init();
#if SNMP_LWIP_MIB2
#if SNMP_USE_NETCONN
	snmp_threadsync_init(&snmp_mib2_lwip_locks, snmp_mib2_lwip_synchronizer);
#endif /* SNMP_USE_NETCONN */
	snmp_mib2_set_syscontact_readonly((const u8_t*)"root", NULL);
	snmp_mib2_set_syslocation_readonly((const u8_t*)"lwIP development PC", NULL);
	snmp_mib2_set_sysdescr((const u8_t*)"toppershost", NULL);
#endif /* SNMP_LWIP_MIB2 */

	snmp_set_mibs(mibs, LWIP_ARRAYSIZE(mibs));
	snmp_init();
#endif /* LWIP_SNMP */

#if LWIP_MDNS_RESPONDER
	mdns_resp_init();
	mdns_resp_add_netif(&netif, "toppershost", 3600);
	mdns_resp_add_service(&netif, "myweb", "_http", DNSSD_PROTO_TCP, 80, 3600, srv_txt, NULL);
#endif

#if LWIP_UDP
	tftp_init(&tftp);
#endif /* LWIP_UDP */

	sys_sem_signal(sem);
}

/*-----------------------------------------------------------------------------------*/

#if LWIP_HAVE_SLIPIF
/* (manual) host IP configuration */
#if LWIP_IPV4
static ip_addr_t ipaddr_slip, netmask_slip, gw_slip;
#endif /* LWIP_IPV4 */
struct netif slipif;
#endif /* LWIP_HAVE_SLIPIF */

#if PPP_SUPPORT
sio_fd_t ppp_sio;
ppp_pcb *ppp;
struct netif pppos_netif;

static void
pppos_rx_thread(void *arg)
{
	u32_t len;
	u8_t buffer[128];
	LWIP_UNUSED_ARG(arg);

	/* Please read the "PPPoS input path" chapter in the PPP documentation. */
	while(1){
		len = sio_read(ppp_sio, buffer, sizeof(buffer));
		if (len > 0) {
			/* Pass received raw characters from PPPoS to be decoded through lwIP
			 * TCPIP thread using the TCPIP API. This is thread safe in all cases
			 * but you should avoid passing data byte after byte. */
			pppos_input_tcpip(ppp, buffer, len);
		}
	}
}

static void
ppp_link_status_cb(ppp_pcb *pcb, int err_code, void *ctx)
{
    struct netif *pppif = ppp_netif(pcb);
    LWIP_UNUSED_ARG(ctx);

    switch(err_code) {
    case PPPERR_NONE:               /* No error. */
        {
#if LWIP_DNS
        const ip_addr_t *ns;
#endif /* LWIP_DNS */
        fprintf(stderr, "ppp_link_status_cb: PPPERR_NONE\n\r");
#if LWIP_IPV4
        fprintf(stderr, "   our_ip4addr = %s\n\r", ip4addr_ntoa(netif_ip4_addr(pppif)));
        fprintf(stderr, "   his_ipaddr  = %s\n\r", ip4addr_ntoa(netif_ip4_gw(pppif)));
        fprintf(stderr, "   netmask     = %s\n\r", ip4addr_ntoa(netif_ip4_netmask(pppif)));
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
        fprintf(stderr, "   our_ip6addr = %s\n\r", ip6addr_ntoa(netif_ip6_addr(pppif, 0)));
#endif /* LWIP_IPV6 */

#if LWIP_DNS
        ns = dns_getserver(0);
        fprintf(stderr, "   dns1        = %s\n\r", ipaddr_ntoa(ns));
        ns = dns_getserver(1);
        fprintf(stderr, "   dns2        = %s\n\r", ipaddr_ntoa(ns));
#endif /* LWIP_DNS */
#if PPP_IPV6_SUPPORT
        fprintf(stderr, "   our6_ipaddr = %s\n\r", ip6addr_ntoa(netif_ip6_addr(pppif, 0)));
#endif /* PPP_IPV6_SUPPORT */
        }
        break;

    case PPPERR_PARAM:             /* Invalid parameter. */
        printf("ppp_link_status_cb: PPPERR_PARAM\n");
        break;

    case PPPERR_OPEN:              /* Unable to open PPP session. */
        printf("ppp_link_status_cb: PPPERR_OPEN\n");
        break;

    case PPPERR_DEVICE:            /* Invalid I/O device for PPP. */
        printf("ppp_link_status_cb: PPPERR_DEVICE\n");
        break;

    case PPPERR_ALLOC:             /* Unable to allocate resources. */
        printf("ppp_link_status_cb: PPPERR_ALLOC\n");
        break;

    case PPPERR_USER:              /* User interrupt. */
        printf("ppp_link_status_cb: PPPERR_USER\n");
        break;

    case PPPERR_CONNECT:           /* Connection lost. */
        printf("ppp_link_status_cb: PPPERR_CONNECT\n");
        break;

    case PPPERR_AUTHFAIL:          /* Failed authentication challenge. */
        printf("ppp_link_status_cb: PPPERR_AUTHFAIL\n");
        break;

    case PPPERR_PROTOCOL:          /* Failed to meet protocol. */
        printf("ppp_link_status_cb: PPPERR_PROTOCOL\n");
        break;

    case PPPERR_PEERDEAD:          /* Connection timeout. */
        printf("ppp_link_status_cb: PPPERR_PEERDEAD\n");
        break;

    case PPPERR_IDLETIMEOUT:       /* Idle Timeout. */
        printf("ppp_link_status_cb: PPPERR_IDLETIMEOUT\n");
        break;

    case PPPERR_CONNECTTIME:       /* PPPERR_CONNECTTIME. */
        printf("ppp_link_status_cb: PPPERR_CONNECTTIME\n");
        break;

    case PPPERR_LOOPBACK:          /* Connection timeout. */
        printf("ppp_link_status_cb: PPPERR_LOOPBACK\n");
        break;

    default:
        printf("ppp_link_status_cb: unknown errCode %d\n", err_code);
        break;
    }
}

static u32_t
ppp_output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx)
{
  LWIP_UNUSED_ARG(pcb);
  LWIP_UNUSED_ARG(ctx);
  return sio_write(ppp_sio, data, len);
}
#endif

#if LWIP_NETIF_STATUS_CALLBACK
static void
netif_status_callback(struct netif *nif)
{
	printf("NETIF: %c%c%d is %s\n", nif->name[0], nif->name[1], nif->num,
		netif_is_up(nif) ? "UP" : "DOWN");
#if LWIP_IPV4
	printf("IPV4: Host at %s ", ip4addr_ntoa(netif_ip4_addr(nif)));
	printf("mask %s ", ip4addr_ntoa(netif_ip4_netmask(nif)));
	printf("gateway %s\n", ip4addr_ntoa(netif_ip4_gw(nif)));
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
	printf("IPV6: Host at %s\n", ip6addr_ntoa(netif_ip6_addr(nif, 0)));
#endif /* LWIP_IPV6 */
#if LWIP_NETIF_HOSTNAME
	printf("FQDN: %s\n", netif_get_hostname(nif));
#endif /* LWIP_NETIF_HOSTNAME */

#if LWIP_MDNS_RESPONDER
	if(tcpip_init_flag)
		mdns_resp_netif_settings_changed(nif);
#endif
}
#endif /* LWIP_NETIF_STATUS_CALLBACK */

static void
init_netifs(void)
{
#if LWIP_HAVE_SLIPIF
#if SLIP_PTY_TEST
	u8_t siodev_slip = 3;
#else
	u8_t siodev_slip = 0;
#endif

#if LWIP_IPV4
	netif_add(&slipif, ip_2_ip4(&ipaddr_slip), ip_2_ip4(&netmask_slip), ip_2_ip4(&gw_slip),
		(void*)&siodev_slip, slipif_init, tcpip_input);
#else /* LWIP_IPV4 */
	netif_add(&slipif, (void*)&siodev_slip, slipif_init, tcpip_input);
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
	netif_create_ip6_linklocal_address(&slipif, 1);
#endif
#if LWIP_NETIF_STATUS_CALLBACK
	netif_set_status_callback(&slipif, netif_status_callback);
#endif /* LWIP_NETIF_STATUS_CALLBACK */
	netif_set_link_up(&slipif);
	netif_set_up(&slipif);
#endif /* LWIP_HAVE_SLIPIF */

#if PPP_SUPPORT
#if PPP_PTY_TEST
	ppp_sio = sio_open(2);
#else
	ppp_sio = sio_open(0);
#endif
	if(!ppp_sio){
		syslog_0(LOG_ERROR, "Error opening device: ");
		slp_tsk();
	}

	ppp = pppos_create(&pppos_netif, ppp_output_cb, ppp_link_status_cb, NULL);
	if(!ppp){
		syslog_0(LOG_ERROR, "Could not create PPP control interface");
		slp_tsk();
	}

#ifdef LWIP_PPP_CHAP_TEST
	ppp_set_auth(ppp, PPPAUTHTYPE_CHAP, "lwip", "mysecret");
#endif

	ppp_connect(ppp, 0);

#if LWIP_NETIF_STATUS_CALLBACK
	netif_set_status_callback(&pppos_netif, netif_status_callback);
#endif /* LWIP_NETIF_STATUS_CALLBACK */
#endif /* PPP_SUPPORT */

#if LWIP_IPV4
#if LWIP_DHCP
	IP_ADDR4(&gw,      0,0,0,0);
	IP_ADDR4(&ipaddr,  0,0,0,0);
	IP_ADDR4(&netmask, 0,0,0,0);
#endif /* LWIP_DHCP */
	netif_add(&netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw), NULL, ethernetif_init, tcpip_input);
#else /* LWIP_IPV4 */
	netif_add(&netif, NULL, ethernetif_init, tcpip_input);
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
	netif_create_ip6_linklocal_address(&netif, 1);
	netif.ip6_autoconfig_enabled = 1;
#endif
#if LWIP_NETIF_STATUS_CALLBACK
	netif_set_status_callback(&netif, netif_status_callback);
#endif /* LWIP_NETIF_STATUS_CALLBACK */
	netif_set_default(&netif);
	netif_set_up(&netif);

#if LWIP_DHCP
	dhcp_start(&netif);
#endif /* LWIP_DHCP */

#if 0
	/* Only used for testing purposes: */
	netif_add(&ipaddr, &netmask, &gw, NULL, pcapif_init, tcpip_input);
#endif

#if LWIP_TCP
	netio_init();
#endif
#if LWIP_TCP && LWIP_NETCONN
	tcpecho_init();
	shell_init();
	httpd_init();
#endif
#if LWIP_UDP && LWIP_NETCONN
	udpecho_init();
#endif  
#if LWIP_SOCKET
	chargen_init();
#endif
	/*  sys_timeout(5000, tcp_debug_timeout, NULL);*/
}

/*-----------------------------------------------------------------------------------*/
static void
main_thread(void *arg)
{
	sys_sem_t sem;
	LWIP_UNUSED_ARG(arg);

	if(sys_sem_new(&sem, 0) != ERR_OK) {
		LWIP_ASSERT("Failed to create semaphore", 0);
	}
	tcpip_init(tcpip_init_done, &sem);
	sys_sem_wait(&sem);
	syslog_0(LOG_NOTICE, "TCP/IP initialized.");

#if LWIP_SOCKET && defined(USE_PINGSEND)
	ping_init(&ping_addr);
#endif

	syslog_0(LOG_NOTICE, "Applications started.");

#ifdef MEM_PERF
	mem_perf_init("/tmp/memstats.client");
#endif /* MEM_PERF */
#if 0
	stats_display();
#endif
#if PPP_SUPPORT
	/* Block forever. */
	sys_thread_new("pppos_rx_thread", pppos_rx_thread, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
	sys_sem_wait(&sem);
#endif
	tcpip_init_flag = true;
}

int wolfSSL_Debugging_ON(void);

/*
 *  メインタスク
 */
void
main_task(intptr_t exinf)
{
#ifdef _STORAGEDEVICE_H_
	StorageDevice_t *psdev;
#endif	/* _STORAGEDEVICE_H_ */
	ER_UINT	ercd;

	SVC_PERROR(syslog_msk_log(LOG_UPTO(LOG_NOTICE), LOG_UPTO(LOG_EMERG)));
	syslog(LOG_NOTICE, "Sample program starts (exinf = %d).", (int_t) exinf);

	/*
	 *  シリアルポートの初期化
	 *
	 *  システムログタスクと同じシリアルポートを使う場合など，シリアル
	 *  ポートがオープン済みの場合にはここでE_OBJエラーになるが，支障は
	 *  ない．
	 */
	ercd = serial_opn_por(TASK_PORTID);
	if (ercd < 0 && MERCD(ercd) != E_OBJ) {
		syslog(LOG_ERROR, "%s (%d) reported by `serial_opn_por'.",
									itron_strerror(ercd), SERCD(ercd));
	}
	SVC_PERROR(serial_ctl_por(TASK_PORTID,
							(IOCTL_CRLF | IOCTL_FCSND | IOCTL_FCRCV)));

#ifdef _STORAGEDEVICE_H_
	/*
	 *  SD-CARDの設定
	 */
	psdev = SDMGetStorageDevice(0);
	psdev->_sdev_notice = storagedev_notice;
	SDeviceHead._get_datetime = get_fat_time;
#endif	/* _STORAGEDEVICE_H_ */

	/* startup defaults (may be overridden by one or more opts) */
#if LWIP_IPV4
	IP_ADDR4(&gw,      192,168,  0,1);
	IP_ADDR4(&netmask, 255,255,255,0);
	IP_ADDR4(&ipaddr,  192,168,  0,2);
#if LWIP_HAVE_SLIPIF
	IP_ADDR4(&gw_slip,      192,168,  2,  1);
	IP_ADDR4(&netmask_slip, 255,255,255,255);
	IP_ADDR4(&ipaddr_slip,  192,168,  2,  2);
#endif
#endif /* LWIP_IPV4 */

	IP_SET_TYPE_VAL(ping_addr, IPADDR_TYPE_V4);
	/* use debug flags defined by debug.h */
	debug_flags = LWIP_DBG_OFF;

	sys_init();
	sys_thread_new("main_thread", main_thread, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

	//wolfSSL_Debugging_ON();
	iothub_client_init();

	syslog(LOG_NOTICE, "Sample program ends.");
	slp_tsk();
	SVC_PERROR(ext_ker());
	assert(0);
}
