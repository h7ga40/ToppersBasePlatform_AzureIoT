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
 *  $Id$
 */

/*
 *  FONTのヘッダファイル
 */

#ifndef _FONTS_H_
#define _FONTS_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <t_stddef.h>

#define NUM_FONTS   224

#ifndef TOPPERS_MACRO_ONLY

/*
 *  FONT構造体定義
 */
typedef struct _font_s {
	uint8_t     width;
	uint8_t     height;
	uint8_t     stride;
	uint8_t     iheight;
	uint8_t     image[8];
} font_t;

/*
 *  FONTヘッダ構造体定義
 */
typedef struct _font_head_s {
	const char  font_name[32];
	uint8_t     file_attribute;
	uint8_t     font_attribute;
	uint8_t     font_width;
	uint8_t     font_height;
	const font_t **font_table;
	void        *pdata;
} font_head_t;

typedef struct font_file_head_s {
	char           magic[4];
	char           name[32];
	unsigned int   fsize;
	unsigned int   csize;
	unsigned int   isize;
	unsigned int   off_cnv_table1;
	unsigned int   siz_cnv_table1;
	unsigned int   off_cnv_table2;
	unsigned int   siz_cnv_table2;
	unsigned int   off_cnv_table3;
	unsigned int   siz_cnv_table3;
	unsigned int   off_2byte_font;
	unsigned int   siz_2byte_font;
	unsigned int   off_3byte_font;
	unsigned int   siz_3byte_font;
} font_file_head_t;

extern font_head_t Embedded_Font9;
extern font_head_t Shinonome_Font12;
extern font_head_t Shinonome_Font16;


#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif

#endif

