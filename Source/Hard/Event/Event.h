/*
 * Event.h
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */
/******************************************************************************/

#ifndef _SOURCE_HARD_EVENT_EVENT_H_
#define _SOURCE_HARD_EVENT_EVENT_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Config.h"

#if(EVENT_ENABLED)

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define CURRENT_TIME_MS()                              EVENT_GetCurrentTickMs()
#define TIME_ELAPSE_MS(start)                          ((CURRENT_TIME_MS() - (start)) & MAX_UINT32_T)

typedef void (*EventFunction) (void);
typedef uint8_t EventStatus;
enum{
    EVENT_UNUSED = 0x00,
    EVENT_INACTIVE = 0x01,
    EVENT_ACTIVE = 0x02,
};

typedef uint8_t EventResult;
enum{
    EVENT_SUCCESS = 0,
    EVENT_FAILURE,
    EVENT_INVALID_INIT,
};

typedef uint8_t EventControl;

typedef struct{
    EventStatus status;
    uint32_t timeStart;
    uint32_t timeExcute;
    EventFunction eventHandler;
} EventParam_t;


/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

void EVENT_Init(void);
EventResult EVENT_Creat(EventControl *eventId, EventFunction callbackFunc);
void EVENT_Task(void);

void EVENT_SetActive(EventControl eventId);
void EVENT_SetInactive(EventControl eventId);
void EVENT_SetDelayMS(EventControl eventId, uint32_t delay);

uint32_t EVENT_GetCurrentTickMs(void);

/******************************************************************************/
#endif

#endif /* _SOURCE_HARD_EVENT_EVENT_H_ */
