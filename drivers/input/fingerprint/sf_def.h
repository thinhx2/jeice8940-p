#ifndef __SF_DEF_H__
#define __SF_DEF_H__

//-----------------------------------------------------------------------------
// platform lists
//-----------------------------------------------------------------------------
/*************************************************
SF_REE_MTK          联发科平台，android6以上版本
SF_REE_QUALCOMM     高通平台
SF_REE_SPREAD       展讯平台
SF_REE_HIKEY9600    华为麒麟960
SF_REE_MTK_L5_X     联发科平台，android5版本

SF_TEE_BEANPOD      豆荚TEE
SF_TEE_TRUSTKERNEL  甁砵TEE
SF_TEE_QSEE         高通TEE
SF_TEE_TRUSTONIC    trustonic TEE
SF_TEE_RONGCARD     融卡TEE
SF_TEE_TRUSTY       展讯TEE
*************************************************/

#define SF_REE_MTK                  1
#define SF_REE_QUALCOMM             2
#define SF_REE_SPREAD               3
#define SF_REE_HIKEY9600            4
#define SF_REE_MTK_L5_X             5

#define SF_TEE_BEANPOD              80
#define SF_TEE_TRUSTKERNEL          81
#define SF_TEE_QSEE                 82
#define SF_TEE_TRUSTONIC            83
#define SF_TEE_RONGCARD             84
#define SF_TEE_TRUSTY               85

//-----------------------------------------------------------------------------
// COMPATIBLE mode lists
#define SF_COMPATIBLE_NOF           0
#define SF_COMPATIBLE_REE           1
#define SF_COMPATIBLE_BEANPOD_V1    2
#define SF_COMPATIBLE_BEANPOD_V2    3
#define SF_COMPATIBLE_TRUSTKERNEL   4
#define SF_COMPATIBLE_QSEE          5
#define SF_COMPATIBLE_TRUSTY        6
#define SF_COMPATIBLE_RONGCARD      7
#define SF_COMPATIBLE_TRUSTONIC     8

//-----------------------------------------------------------------------------
// vdd power mode lists
#define PWR_MODE_NOF                0
#define PWR_MODE_GPIO               1
#define PWR_MODE_REGULATOR          2


#endif
