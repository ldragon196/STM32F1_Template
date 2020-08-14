/*
 *  Debug.h
 *
 *  Created on: June 24, 2020
 *      Author: LongHD
 */

#ifndef UTILITY_DEBUG_DEBUG_H_
#define UTILITY_DEBUG_DEBUG_H_

/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if(DEBUG_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#ifdef DEBUG_PRINT_ENABLE

#define DEBUG_PRINT(...)                             do{printf(__VA_ARGS__);} while(0)
#define DEBUG_PRINTLN(...)                           do{printf(__VA_ARGS__); printf("\r\n");} while(0)
#define DEBUG_PRINT_HEX(a,b)                         DEBUG_PrintHex((a), (b))

#else

#define DEBUG_PRINT(...)                             do{} while(0)
#define DEBUG_PRINTLN(...)                           do{} while(0)
#define DEBUG_PRINT_HEX(...)                         do{} while(0)

#endif

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

void DEBUG_PrintHex(uint8_t *data, uint8_t size);

/******************************************************************************/
#endif

#endif /* UTILITY_DEBUG_DEBUG_H_ */