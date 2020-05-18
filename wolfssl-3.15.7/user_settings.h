//#define DEBUG_WOLFSSL

#define WOLFSSL_LWIP
#define NO_WOLFSSL_DIR
#define NO_WRITEV

#ifndef HAVE_LIBZ
#define HAVE_LIBZ
#endif
#define HAVE_ECC
#define HAVE_AESGCM
//#define HAVE_FIPS
// google�ŕK�v
#define HAVE_TLS_EXTENSIONS
#define HAVE_SNI
#define HAVE_SUPPORTED_CURVES
// Azure��MQTToWS�ŕK�v�AOpen Weather Map�ł͊Q
//#define HAVE_SECURE_RENEGOTIATION
//#define HAVE_SERVER_RENEGOTIATION_INFO
//#define WOLFSSL_SECURE_RENEGOTIATION_ON_BY_DEFAULT

#define ECC_TIMING_RESISTANT
#define TFM_TIMING_RESISTANT

#define WOLFSSL_RIPEMD
#define WOLFSSL_SHA512
#define WOLFSSL_SHA384
#define WOLFSSL_SMALL_STACK
// curl�Ŏg�p���Ă���
#define WOLFSSL_ALLOW_TLSV10

#define WOLFSSL_CERT_EXT
//#define OPENSSL_EXTRA	GR-SAKURA��ROM������Ȃ��Ȃ�

#define WC_RSA_BLINDING

#define SINGLE_THREADED

#define CUSTOM_RAND_GENERATE_SEED custom_rand_generate_seed

#define WOLFSSL_USER_FILESYSTEM
#include <stdio.h>
#include <fcntl.h>

#define XFILE      FILE*
#define XFOPEN     fopen
#define XFSEEK     fseek
#define XFTELL     ftell
#define XREWIND    rewind
#define XFREAD     fread
#define XFWRITE    fwrite
#define XFCLOSE    fclose
#define XSEEK_END  SEEK_END
#define XBADFILE   NULL
#define XFGETS     fgets
/* WOLFSSL_USER_FILESYSTEM */

