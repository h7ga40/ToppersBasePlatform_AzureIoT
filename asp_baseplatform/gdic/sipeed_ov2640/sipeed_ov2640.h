/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2010 by Embedded and Real-Time Systems Laboratory
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
 *  $Id$
 */
/*
 *  SIPEED OV2640 CAMARE制御プログラムのヘッダファイル
 */

#ifndef _SIPEED_OV2640_H_
#define _SIPEED_OV2640_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "kernel.h"
#include "dvp.h"

#define OV9650_ID       (0x96)
#define OV2640_ID       (0x26)
#define OV7725_ID       (0x77)
#define MT9V034_ID      (0x13)
#define LEPTON_ID       (0x54)
#define OV_CHIP_ID      (0x0A)
#define ON_CHIP_ID      (0x00)
#define GC0328_ID       (0x9d)
#define GC0328_ADDR     (0x42)

#ifndef TOPPERS_MACRO_ONLY

typedef enum {
    FRAMESIZE_INVALID = 0,
    // C/SIF Resolutions
    FRAMESIZE_QQCIF,    // 88x72
    FRAMESIZE_QCIF,     // 176x144
    FRAMESIZE_CIF,      // 352x288
    FRAMESIZE_QQSIF,    // 88x60
    FRAMESIZE_QSIF,     // 176x120
    FRAMESIZE_SIF,      // 352x240
    // VGA Resolutions
    FRAMESIZE_QQQQVGA,  // 40x30
    FRAMESIZE_QQQVGA,   // 80x60
    FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_HQQQVGA,  // 60x40
    FRAMESIZE_HQQVGA,   // 120x80
    FRAMESIZE_HQVGA,    // 240x160
    // FFT Resolutions
    FRAMESIZE_64X32,    // 64x32
    FRAMESIZE_64X64,    // 64x64
    FRAMESIZE_128X64,   // 128x64
    FRAMESIZE_128X128,  // 128x128
    // Other
    FRAMESIZE_LCD,      // 128x160
    FRAMESIZE_QQVGA2,   // 128x160
    FRAMESIZE_WVGA,     // 720x480
    FRAMESIZE_WVGA2,    // 752x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_UXGA,     // 1600x1200
    FRAMESIZE_CUSTOM,
} framesize_t;


typedef enum {
    PIXFORMAT_INVLAID = 0,
    PIXFORMAT_BAYER,     // RAW
    PIXFORMAT_RGB565,    // RGB565
    PIXFORMAT_YUV422,    // YUV422
    PIXFORMAT_GRAYSCALE, // GRAYSCALE
    PIXFORMAT_JPEG,      // JPEG/COMPRESSED
} pixformat_t;



typedef enum {
    GAINCEILING_2X,
    GAINCEILING_4X,
    GAINCEILING_8X,
    GAINCEILING_16X,
    GAINCEILING_32X,
    GAINCEILING_64X,
    GAINCEILING_128X,
} gainceiling_t;

typedef enum {
    FRAMERATE_2FPS =0x9F,
    FRAMERATE_8FPS =0x87,
    FRAMERATE_15FPS=0x83,
    FRAMERATE_30FPS=0x81,
    FRAMERATE_60FPS=0x80,
} framerate_t;

typedef enum {
    ACTIVE_LOW,
    ACTIVE_HIGH,
    ACTIVE_BINOCULAR,
} polarity_t;

typedef struct _OV2640_s {
	DVP_Handle_t    *hdvp;
	framesize_t     frameSize;
	pixformat_t     pixFormat;

	uint16_t        _width;
	uint16_t        _height;
	uint8_t         _id;
	uint32_t        *_dataBuffer;
	uint32_t        *_aiBuffer;
	uint16_t        _resetPoliraty;
	uint16_t        _pwdnPoliraty;
	uint32_t        _slaveAddr;
} OV2640_t;

extern void ov2640_getResolition(OV2640_t *hcmr, framesize_t frameSize);
extern ER ov2640_sensor_ov_detect(OV2640_t *hcmr);
extern ER ov2640_sensro_gc_detect(OV2640_t *hcmr);
extern ER ov2640_reset(OV2640_t *hcmr);
extern ER ov2640_set_pixformat(OV2640_t *hcmr);
extern ER ov2640_set_framesize(OV2640_t *hcmr);
extern ER ov2640_activate(OV2640_t *hcmr, bool_t run);
extern ER ov2640_snapshot(OV2640_t *hcmr);
extern ER ov2640_cambus_scan_gc0328(OV2640_t *hcmr);
extern int ov2640_id(OV2640_t *hcmr);

extern ER ov2640_setInvert(OV2640_t *hcmr, bool_t invert);
extern ER ov2640_set_contrast(OV2640_t *hcmr, int level);
extern ER ov2640_set_brightness(OV2640_t *hcmr, int level);
extern ER ov2640_set_saturation(OV2640_t *hcmr, int level);
extern ER ov2640_set_gainceiling(OV2640_t *hcmr, gainceiling_t gainceiling);
extern ER ov2640_set_quality(OV2640_t *hcmr, int qs);
extern ER ov2640_set_colorbar(OV2640_t *hcmr, bool_t enable);
extern ER ov2640_set_auto_exposure(OV2640_t *hcmr, bool_t enable, int exposure_us);
extern ER ov2640_get_exposure_us(OV2640_t *hcmr, int *exposure_us);
extern ER ov2640_set_auto_whitebal(OV2640_t *hcmr, bool_t enable, float r_gain_db, float g_gain_db, float b_gain_db);
extern ER ov2640_set_vflip(OV2640_t *hcmr, bool_t enable);
#ifdef USE_GAIN
extern ER ov2640_set_auto_gain(OV2640_t *hcmr, bool_t enable, float gain_db, float gain_db_ceiling);
extern ER ov2640_get_gain_db(OV2640_t *hcmr, float *gain_db);
#endif	/* USE_GAIN */

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif

#endif	/* _SIPEED_OV2640_H_ */

