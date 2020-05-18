/*
 *  TOPPERS BASE PLATFORM MIDDLEWARE
 * 
 *  Copyright (C) 2017-2018 by TOPPERS PROJECT
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
 *  @(#) $Id$
 */
/*
 *  USB Host Middleware PRT CLASS部
 */

#include "tusbh_prt.h"


#define CONTROL_STATUS                  1
#define CONTROL_VENDER                  2


#define PRT_REQUEST_VENDER(d, b, c, l)  tusbhControlRequest((d), (b), PRT_VENDER_SEND_TYPE, (c), 0, 0, (l))
#define PRT_REQUEST_STATUS(d, b, c, l)  tusbhControlRequest((d), (b), PRT_REQUEST_STATUS_TYPE, (c), 0, 0, (l))


static TUSBH_ERCODE tusbhPRTInit(TUSBH_Device_t *pdevice);
static TUSBH_ERCODE tusbhPRTDeInit(TUSBH_Device_t *pdevice);
static TUSBH_ERCODE tusbhPRTProcess(TUSBH_Device_t *pdevice, uint8_t *mes);

static TUSBH_Class_t PRT_Class = {
	NULL,
	"PRT",
	PRT_CLASS,
	tusbhPRTInit,
	tusbhPRTDeInit,
	tusbhPRTProcess,
	NULL
};

/*
 *  PRTクラスセットアップ関数
 */
void tusbhLinkPRT(TUSBH_Handle_t *phost)
{
	tusbhLinkClass(phost, &PRT_Class);
}

/*
 *  PRTクラス初期設定
 *  parameter1 pdevice:  デバイスハンドラ
 *  return     TUSBH_ERCODE
 */
static TUSBH_ERCODE
tusbhPRTInit(TUSBH_Device_t *pdevice)
{
	uint8_t interface = 0; 
	PRT_Handle_t *hprt;
	TUSBH_Handle_t *phost = pdevice->pHost;
	uint8_t max_ep, i;

	interface = tusbhFindInterface(pdevice, pdevice->pClass->classCode, 0x01, 0x02);

	if(interface == NO_INTERFACE){
		syslog_1(LOG_ERROR, "[PRT] Cannot Find the interface for %s class.", pdevice->pClass->Name);
		return TUSBH_E_ERR;
	}
	tusbhSelectInterface(pdevice, interface);

	hprt = (PRT_Handle_t *)tusbmalloc(sizeof(PRT_Handle_t));
	pdevice->pData = hprt;
	memset(hprt, 0, sizeof(PRT_Handle_t));
	max_ep = ((pdevice->ItfDesc[interface].bNumEndpoints <= 2) ?
              pdevice->ItfDesc[interface].bNumEndpoints : 2);
#if 1	/* ROI DEBUG */
	syslog_3(LOG_NOTICE, "## tusbhPRTInit hmsc[%08x] max_ep(%d) buffer(%d) ##", hprt, max_ep, offsetof(PRT_Handle_t, buffer));
#endif	/* ROI DEBUG */

	for(i = 0 ; i < max_ep; i++){
		if(pdevice->EpDesc[interface][i].bEndpointAddress & 0x80){
			hprt->InEp     = (pdevice->EpDesc[interface][i].bEndpointAddress);
			hprt->InEpSize = pdevice->EpDesc[interface][i].wMaxPacketSize;
			hprt->InPipe = tusbhAllocPipe(phost, hprt->InEp);
		}
		else{
			hprt->OutEp = (pdevice->EpDesc[interface][i].bEndpointAddress);
			hprt->OutEpSize = pdevice->EpDesc[interface][i].wMaxPacketSize;
			hprt->OutPipe = tusbhAllocPipe(phost, hprt->OutEp);
		}
	}
	if(hprt->OutPipe == 0 || hprt->InPipe == 0)
		return TUSBH_E_ERR;

	hprt->state = PRT_PROCESS_INIT;
	hprt->timer = 0;
	hprt->pDev = pdevice;

	/*
	 *  NSCチャネルオープン
	 */
	tusbhOpenPipe(pdevice, hprt->OutPipe, hprt->OutEp, USB_EP_TYPE_BULK, hprt->OutEpSize);
	tusbhHDSetToggle(phost, hprt->OutPipe, 0);
	tusbhOpenPipe(pdevice, hprt->InPipe, hprt->InEp, USB_EP_TYPE_BULK, hprt->InEpSize);
	tusbhHDSetToggle(phost, hprt->InPipe, 0);

	/* タイマースタート */
	pdevice->timeid = pdevice->timecount = 100;
	return TUSBH_E_OK;
}

/*
 *  PRTクラス終了設定
 *  parameter1 pdevice:  デバイスハンドラ
 *  return     TUSBH_ERCODE
 */
static TUSBH_ERCODE
tusbhPRTDeInit(TUSBH_Device_t *pdevice)
{
	PRT_Handle_t *hprt;
	TUSBH_Handle_t *phost = pdevice->pHost;

	hprt = (PRT_Handle_t *)pdevice->pData;
	if(hprt->OutPipe != 0){
		tusbhClosePipe(pdevice, hprt->OutPipe);
		tusbFreePipe(phost, hprt->OutPipe);
		hprt->OutPipe = 0;
	}

	if(hprt->InPipe != 0){
		tusbhClosePipe(pdevice, hprt->InPipe);
		tusbFreePipe(phost, hprt->InPipe);
		hprt->InPipe = 0;
	}

	if(pdevice->pData != NULL){
		tusbfree(pdevice->pData);
		pdevice->pData = NULL;
	}
	return TUSBH_E_OK;
}

/*
 *  PRTクラスプロセス実行
 *  parameter1 pdevice:  デバイスハンドラ
 *  parameter2 mes:      通信メッセージへのポインタ
 *  return     TUSBH_ERCODE
 */
static TUSBH_ERCODE
tusbhPRTProcess(TUSBH_Device_t *pdevice, uint8_t *mes)
{
	PRT_Handle_t   *hprt  = (PRT_Handle_t *)pdevice->pData;
	TUSBH_Handle_t *phost = pdevice->pHost;
	void (*func)(TUSBH_Device_t *, uint8_t, TUSBH_ERCODE, uint32_t) = pdevice->pClass->subfunc;
	TUSBH_URBState_t URB_Status = mes[3];
	TUSBH_ERCODE ercd = TUSBH_E_BUSY;
	int32_t length;

    if(mes[0] == TUSBH_URB_EVENT){
		hprt->urb_wait = 0;
    }
	else if(mes[0] == TUSBH_TIME_EVENT){
		hprt->timer += mes[3];
		pdevice->timecount = mes[3];
	}
	else if(mes[0] == TUSBH_CLASS_EVENT && mes[2] != 0)
		hprt->state = mes[2];
	switch (hprt->state){
	case PRT_PROCESS_INIT:
		if(phost->usrcallback != NULL)
			phost->usrcallback(phost, pdevice, HOST_USER_CLASS_ACTIVE);
		hprt->state = PRT_PROCESS_IDLE;
	case PRT_PROCESS_IDLE:
		if(mes[0] != TUSBH_URB_EVENT){
			ercd = TUSBH_E_OK;
			break;
		}
		if(mes[2] == pdevice->cntl_pipe_in || mes[2] == pdevice->cntl_pipe_out){
			ercd = tusbhControlWait(pdevice, mes);
			if(ercd == TUSBH_E_OK){
				hprt->state = PRT_PROCESS_IDLE;
				hprt->controlType = 0;
			}
			else if(ercd != TUSBH_E_BUSY && ercd != TUSBH_E_BUSY_URB){
				hprt->state = PRT_PROCESS_ERROR;
				hprt->controlType = 0;
			}
			break;
		}
		else if(URB_Status == TUSBH_URB_DONE){
			if(mes[2] == hprt->InPipe){
				hprt->RxTrnPktSize = tusbhHDTrasLength(phost, hprt->InPipe);
				hprt->RxTrnLength += hprt->RxTrnPktSize;
				hprt->pRxTrnBuff += hprt->RxTrnPktSize;
				hprt->rwr_status = TUSBH_E_OK;
				if(func != NULL)
					func(pdevice, PRT_LINE_STATUS_RECEIVED, TUSBH_E_OK, hprt->RxTrnLength);
			}
			else{
				hprt->TxTrnLength += hprt->TxTrnPktSize;
				hprt->pTxTrnBuff += hprt->TxTrnPktSize;
				if(hprt->TxTrnSize > hprt->TxTrnLength)
					hprt->state = PRT_PROCESS_SEND;
				else{
					hprt->rwt_status = TUSBH_E_OK;
					if(func != NULL)
						func(pdevice, PRT_LINE_STATUS_SENDED, TUSBH_E_OK, hprt->TxTrnLength);
				}
			}
		}
		else if(URB_Status == TUSBH_URB_NOTREADY){
			if(mes[2] == hprt->InPipe)
				hprt->state = PRT_PROCESS_RECEIVE;
			else
				hprt->state = PRT_PROCESS_SEND;
		}
		else{
			hprt->state = PRT_PROCESS_ERROR;
			if(mes[2] == hprt->InPipe)
				hprt->rwr_status = TUSBH_E_ERR;
			else
				hprt->rwt_status = TUSBH_E_ERR;
			if(func != NULL)
				func(pdevice, PRT_LINE_STATUS_CHANGED, TUSBH_E_ERR, 0);
		}
		tusbSendData(phost->process_event, TUSBH_CLASS_EVENT, pdevice->idx, 0, 0);
		break;
	case PRT_PROCESS_CONTROL:
		if(hprt->controlType == CONTROL_VENDER)
			PRT_REQUEST_VENDER(pdevice, hprt->buffer, hprt->controlRequest, hprt->controlLength);
		else
			PRT_REQUEST_STATUS(pdevice, hprt->buffer, hprt->controlRequest, hprt->controlLength);
		hprt->state = PRT_PROCESS_IDLE;
		break;
	case PRT_PROCESS_SEND:
		length = hprt->TxTrnSize - hprt->TxTrnLength;
		if(length > hprt->OutEpSize)
			hprt->TxTrnPktSize = hprt->OutEpSize;
		else
			hprt->TxTrnPktSize = length;
		tusbhBulkWrite(pdevice, hprt->pTxTrnBuff, hprt->TxTrnPktSize, hprt->OutPipe);
		hprt->state = PRT_PROCESS_IDLE;
		break;
	case PRT_PROCESS_RECEIVE:
		length = hprt->RxTrnSize - hprt->RxTrnLength;
		if(length > hprt->InEpSize)
			hprt->RxTrnPktSize = hprt->InEpSize;
		else
			hprt->RxTrnPktSize = length;
		tusbhBulkRead(pdevice, hprt->pRxTrnBuff, hprt->RxTrnPktSize, hprt->InPipe);
		hprt->state = PRT_PROCESS_IDLE;
		break;
	case PRT_PROCESS_ERROR:
		tusbhClearFeature(pdevice, 0x00);
		hprt->state = PRT_PROCESS_ERROR_WAIT;
		break;
	case PRT_PROCESS_ERROR_WAIT:
		if(tusbhControlWait(pdevice, mes) == TUSBH_E_OK)
			hprt->state = PRT_PROCESS_IDLE;
		break;


	default:
		break;
	}
	return ercd;
}

/*
 *  PRT SEND EOJ
 *  parameter1 phost:   ホストハンドラ
 *  parameter2 unit:    UNIT#
 *  parameter3 cmd:     設定コマンド
 *  return     TUSBH_ERCODE
 */
TUSBH_ERCODE
tusbhPrtEOJ(TUSBH_Handle_t *phost, uint8_t unit, uint8_t cmd, uint8_t prm1, uint8_t prm2)
{
	TUSBH_Device_t *pdevice = tusbhSearchDevice(phost, PRT_Class.classCode, &unit);
	PRT_Handle_t *hprt;

	if(pdevice == NULL)
		return TUSBH_E_OBJ;
	hprt = (PRT_Handle_t *)pdevice->pData;
	if(pdevice->dstate == DEVICE_CLASS){
		hprt->buffer[0] = cmd;
		hprt->buffer[1] = prm1;
		hprt->buffer[2] = prm2;
		hprt->controlType    = CONTROL_VENDER;
		hprt->controlRequest  = PRT_VERNDER_REQUEST_EOJ;
		hprt->controlLength  = 3;
		hprt->state = PRT_PROCESS_CONTROL;
		tusbSendData(phost->process_event, TUSBH_CLASS_EVENT, pdevice->idx, 0, 0);
		while(hprt->controlType != 0){
			tusbSleep(1);
		}
	}
	return TUSBH_E_OK;
}

/*
 *  PRT GET PORTID
 *  parameter1 phost:   ホストハンドラ
 *  parameter2 unit:    UNIT#
 *  parameter3 cmd:     設定コマンド
 *  return     TUSBH_ERCODE
 */
TUSBH_ERCODE
tusbhPrtPortID(TUSBH_Handle_t *phost, uint8_t unit, uint8_t *buf)
{
	TUSBH_Device_t *pdevice = tusbhSearchDevice(phost, PRT_Class.classCode, &unit);
	PRT_Handle_t *hprt;

	if(pdevice == NULL)
		return TUSBH_E_OBJ;
	hprt = (PRT_Handle_t *)pdevice->pData;
	if(pdevice->dstate == DEVICE_CLASS){
		hprt->controlType    = CONTROL_STATUS;
		hprt->controlRequest = PRT_REQUEST_DEVICEID;
		hprt->controlLength  = 1;
		tusbSendData(phost->process_event, TUSBH_CLASS_EVENT, pdevice->idx, PRT_PROCESS_CONTROL, 0);
		while(hprt->controlType != 0){
			tusbSleep(1);
		}
		*buf = hprt->buffer[0];
	}
	return TUSBH_E_OK;
}

/*
 *  PRT GET PRINT STATUS
 *  parameter1 phost:   ホストハンドラ
 *  parameter2 unit:    UNIT#
 *  parameter3 cmd:     設定コマンド
 *  return     TUSBH_ERCODE
 */
TUSBH_ERCODE
tusbhPrtPortStatus(TUSBH_Handle_t *phost, uint8_t unit, uint8_t *buf, uint32_t length)
{
	TUSBH_Device_t *pdevice = tusbhSearchDevice(phost, PRT_Class.classCode, &unit);
	PRT_Handle_t *hprt;
	uint32_t trans_len, packet_len;

	if(length > PRT_MAX_PORTSTATUS_SIZE)
		trans_len = PRT_MAX_PORTSTATUS_SIZE;
	else
		trans_len = length;
	if(pdevice == NULL)
		return TUSBH_E_OBJ;
	hprt = (PRT_Handle_t *)pdevice->pData;
	if(pdevice->dstate == DEVICE_CLASS){
		hprt->controlType    = CONTROL_STATUS;
		hprt->controlRequest = PRT_REQUEST_PORTSTATUS;
		hprt->controlLength  = trans_len;
		tusbSendData(phost->process_event, TUSBH_CLASS_EVENT, pdevice->idx, PRT_PROCESS_CONTROL, 0);
		while(hprt->controlType != 0){
			tusbSleep(1);
		}
		packet_len = (hprt->buffer[0] << 8) | hprt->buffer[1];
		if(trans_len > (packet_len + 2))
			trans_len = packet_len + 2;
		tusbmemcpy(buf, hprt->buffer, trans_len);
	}
	return TUSBH_E_OK;
}

/*
 *  PRT DATA RECEIVE
 *  parameter1 phost:   ホストハンドラ
 *  parameter2 unit:    UNIT#
 *  parameter3 pbuf:    受信データ領域へのポインタ
 *  parameter4 length:  受信ブロック長
 *  return     TUSBH_ERCODE
 */
TUSBH_ERCODE
tusbhPrtReceive(TUSBH_Handle_t *phost, uint8_t unit, uint8_t *pbuf, uint32_t length)
{
	TUSBH_Device_t *pdevice = tusbhSearchDevice(phost, PRT_Class.classCode, &unit);
	PRT_Handle_t *hprt;

	if(pdevice != NULL){
		hprt = pdevice->pData;
		if(pdevice->dstate != DEVICE_CLASS)
			return TUSBH_E_OBJ;
		if(hprt->state == PRT_PROCESS_ERROR)
			return TUSBH_E_ERR;
		if(hprt->rwr_status == TUSBH_E_BUSY)
			return TUSBH_E_OBJ;
	}
	else
		return TUSBH_E_OBJ;

    /*
	 *  受信データ設定
	 */
	hprt->RxTrnSize   = length;
	hprt->RxTrnLength = 0;
	hprt->pRxTrnBuff  = pbuf;
	hprt->rwr_status  = TUSBH_E_BUSY;
	tusbSendData(phost->process_event, TUSBH_CLASS_EVENT, pdevice->idx, PRT_PROCESS_RECEIVE, 0);
	return TUSBH_E_OK;
}

/*
 *  PRT DATA SEND
 *  parameter1 phost:   ホストハンドラ
 *  parameter2 unit:    UNIT#
 *  parameter3 pbuf:    送信データ領域へのポインタ
 *  parameter4 length:  送信データ長
 *  return     TUSBH_ERCODE
 */
TUSBH_ERCODE
tusbhPrtSend(TUSBH_Handle_t *phost, uint8_t unit, uint8_t *pbuf, uint32_t length)
{
	TUSBH_Device_t *pdevice = tusbhSearchDevice(phost, PRT_Class.classCode, &unit);
	PRT_Handle_t *hprt;

	if(pdevice != NULL){
		hprt = pdevice->pData;
		if(pdevice->dstate != DEVICE_CLASS)
			return TUSBH_E_OBJ;
		if(hprt->state == PRT_PROCESS_ERROR)
			return TUSBH_E_ERR;
		if(hprt->rwt_status == TUSBH_E_BUSY)
			return TUSBH_E_OBJ;
	}
	else
		return TUSBH_E_OBJ;

    /*
	 *  送信データ設定
	 */
	hprt->TxTrnSize   = length;
	hprt->TxTrnLength = 0;
	hprt->pTxTrnBuff  = pbuf;
	hprt->rwt_status  = TUSBH_E_BUSY;
	tusbSendData(phost->process_event, TUSBH_CLASS_EVENT, pdevice->idx, PRT_PROCESS_SEND, 0);
	return TUSBH_E_OK;
}

/*
 *  PRTコールバック関数
 *  parameter1 phost:   ホストハンドラ
 *  parameter2 func:    関数ポインタ
 */
void
tusbhSetPrtCallBack(TUSBH_Handle_t *phost, void (*func)(TUSBH_Device_t *p, uint8_t, TUSBH_ERCODE, uint32_t))
{
	PRT_Class.subfunc = func;
}


