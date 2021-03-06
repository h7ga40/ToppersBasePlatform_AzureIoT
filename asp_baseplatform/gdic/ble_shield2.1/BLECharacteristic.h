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
The MIT License (MIT)

Copyright (c) 2014 Sandeep Mistry

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _BLE_CHARACTERISTIC_H_
#define _BLE_CHARACTERISTIC_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "BLEAttribute.h"

enum BLECharacteristicEvent {
	BLEWritten = 0,
	BLESubscribed = 1,
	BLEUnsubscribed = 2
};

typedef struct _BLECharacteristic BLECharacteristic;
typedef void (*BLECharacteristicEventHandler)(BLECentral* central, BLECharacteristic *pcharc);

struct _BLECharacteristic{
	BLEAttribute          _attr;
	unsigned char         _properties;
	unsigned char         _valueSize;
	unsigned char*        _value;
	unsigned char         _valueLength;
	bool_t                _fixedLength;

	bool_t                _notify;
	bool_t                _written;
	bool_t                _subscribed;

	void*                 _pblePeripheral;
	bool_t (*characteristicValueChanged)(nRF8001* nRF8001, BLECharacteristic *pcharc);
	bool_t (*canNotifyCharacteristic)(nRF8001* nRF8001, BLECharacteristic *pcharc);
	bool_t (*canIndicateCharacteristic)(nRF8001* nRF8001, BLECharacteristic *pcharc);

	BLECharacteristicEventHandler  _eventHandlers[3];
};

#define BLECharacteristic_getproperties(c)   (c)->_properties
#define BLECharacteristic_getvalueSize(c)    (c)->_valueSize
#define BLECharacteristic_getvalue(c)        (c)->_value
#define BLECharacteristic_getvalueLength(c)  (c)->_valueLength
#define BLECharacteristic_getfixedLength(c)  (c)->_fixedLength
#define BLECharacteristic_getsubscribed(c)   (c)->_subscribed


extern void BLEService_setup(BLEService* pserv, const char *uuid);
extern void BLECharacteristic_setup(BLECharacteristic* pcharc, const char *uuid, unsigned char properties, unsigned char *value, unsigned char len);
extern bool_t BLECharacteristic_setValue1(BLECharacteristic *pcharc, const unsigned char value[], unsigned char length);
extern bool_t BLECharacteristic_setValue2(BLECharacteristic *pcharc, const char* value);
extern bool_t BLECharacteristic_notified(BLECharacteristic *pcharc);
extern bool_t BLECharacteristic_written(BLECharacteristic *pcharc);
extern void BLECharacteristic_setValue(BLECharacteristic *pcharc, BLECentral* central, const unsigned char value[], unsigned char length);
extern void BLECharacteristic_setSubscribed(BLECharacteristic *pcharc, BLECentral* central, bool_t subscribed);
extern bool_t BLECharacteristic_canNotify(BLECharacteristic *pcharc);
extern bool_t BLECharacteristic_canIndicate(BLECharacteristic *pcharc);
extern void BLECharacteristic_setEventHandler(BLECharacteristic *pcharc, uint8_t event, void *eventHandler);

#ifdef __cplusplus
}
#endif

#endif	/* _BLE_CHARACTERISTIC_H_ */
