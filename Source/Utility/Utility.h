/*
 *  Utility.h
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */

#ifndef UTILITY_UTILITY_H_
#define UTILITY_UTILITY_H_

/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/



/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define GET_BIT_0_7(a)          ((a)&0xFF)
#define GET_BIT_8_15(a)         (((a)>>8)&0xFF)
#define GET_BIT_16_23(a)        (((a)>>16)&0xFF)
#define GET_BIT_24_31(a)        (((a)>>24)&0xFF)

#define READ32(address)         ((uint32_t)(*(uint32_t*)(address)))
#define READ16(address)         ((uint16_t)(*(uint16_t*)(address)))
#define READ8(address)          ((uint8_t)(*(uint8_t*)(address)))

#ifndef max
#define max(a, b)               (((a) > (b)) ? (a) : (b))
#define min(a, b)               (((a) < (b)) ? (a) : (b))
#endif /* max */

#ifndef HIGH_UINT16
#define HIGH_UINT16(a)          (((a) >> 8) & 0xFF)
#define LOW_UINT16(a)           ((a) & 0xFF)
#define HIGH_UINT8(a)           (((a) >> 4) & 0x0F)
#define LOW_UINT8(a)            ((a) & 0x0F)
#endif /* HI_UINT16 */

#define MERGE16U(a,b)           ((((a) << 8) & 0xFF00) | ((b) & 0x00FF))
#define MERGE32U(a,b,c,d)       ((((a) << 24) & 0xFF000000) | (((b) << 16) & 0x00FF0000) | (((c) << 8) & 0x0000FF00) | ((d) & 0x000000FF))

#define MAX_UINT32_T            0xFFFFFFFF
#define MAX_UINT16_T            0xFFFF
#define MAX_UINT8_T             0xFF

#ifndef NULL
#define NULL                    0
#endif

#ifndef TRUE
typedef enum {
	FALSE = 0,
	TRUE = 1,
} boolean;
#endif

#define UNUSED(a)              (void)(a)
#define CONCAT_2_(a,b)         a##b
#define CONCAT_3_(a,b,c)       a##b##c

#define CALLBACK_VOID(func)    do{ if(func != NULL) { func();} } while(0)
#define CALLBACK_U8(func, a)   do{ if(func != NULL) { func((a));} } while(0)


/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/

#endif /* UTILITY_UTILITY_H_ */
