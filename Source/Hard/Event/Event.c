/*
 * Event.c
 *
 *  Created on: August 12, 2020
 *      Author: LongHD
 */
/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "Utility/Utility.h"
#include "Event.h"

#if(EVENT_ENABLED)

#if(EVENT_TIMER_USED==0)

static volatile uint32_t milliSecondSystemTick = 0;
#define EVENT_TICK_MS()                       milliSecondSystemTick
#define GetTimeElapse(a)                      ((EVENT_TICK_MS() - (a)) & MAX_UINT32_T)

#else

#include "Hard/Timer/Timer.h"
#if(EVENT_TIMER_USED==1)
TimerBase_t EventTimer = TIMER1_8_INSTANCE(EVENT_TIMER_USED);
#else
TimerBase_t EventTimer = TIMER2_5_INSTANCE(EVENT_TIMER_USED);
#endif

#define TIMER_PERIODS_CONFIG                  65535
#define TIMER_PRESCALER_CONFIG                35999
#define TICK_PER_MS                           2           // Timer frequence / TIMER_PERIODS_CONFIG

#define EVENT_TICK_MS()                       (TIMER_GetCounter(&EventTimer) / TICK_PER_MS)
#define GetTimeElapse(a)                      (((TIMER_GetCounter(&EventTimer) - ((a) * TICK_PER_MS)) & MAX_UINT16_T) / TICK_PER_MS)

#endif

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/


/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

EventParam_t eventControls[EVENT_MAX_CONTROL] = {0};

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/

static uint8_t EVENT_IsExpiredTime(uint8_t eventID);

/******************************************************************************/

#if(EVENT_TIMER_USED==0)
/**
 * @brief  System tick handler
 * @param  None
 * @retval None
 */

void SysTick_Handler(void){
    milliSecondSystemTick++;
}
#endif

/**
 * @brief  Initialized event control
 * @param  None
 * @retval None
 */

void EVENT_Init(void){
	#if(EVENT_TIMER_USED==0)
    // Initialized system tick for event tick
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriority(SysTick_IRQn, IRQn_LevelHighest);
	#else
	
	/* Frequence = SystemCoreClock / ( PreScaler * Periods ) */
	TimerConfig_t config = {
		.Division = TIM_CKD_DIV1,
		.Periods = TIMER_PERIODS_CONFIG,
		.Prescaler = TIMER_PRESCALER_CONFIG
	};
	// Frequence = 1s <-> 10000 tick
	
	TIMER_Init(&EventTimer, &config);
	#endif
	
    // Set event control default value
    memset((uint8_t*) eventControls, 0, sizeof(eventControls));
}

/**
 * @brief  Register event to schedule
 * @param  eventId output to store event id
 *         callbackFunc function handler when event excute
 * @retval EVENT_SUCCESS if register success
 *         error code in otherwise
 */

EventResult EVENT_Creat(EventControl *eventId, EventFunction callbackFunc){
    if((callbackFunc == NULL) || (eventId == NULL) ){
        return EVENT_INVALID_INIT;
    }

    // Check event control available
    for(uint8_t i = 0; i < EVENT_MAX_CONTROL; i++){
        if(eventControls[i].status == EVENT_UNUSED){
            eventControls[i].status = EVENT_INACTIVE;
            eventControls[i].eventHandler = callbackFunc;
            *eventId = i;
            return EVENT_SUCCESS;
        }
    }
    return EVENT_FAILURE;
}

/**
 * @brief  Active event now
 * @param  eventId
 * @retval None
 */

void EVENT_SetActive(EventControl eventId){
    if(eventId < EVENT_MAX_CONTROL){
        eventControls[eventId].timeStart = EVENT_TICK_MS();
        eventControls[eventId].timeExcute = 0;
        eventControls[eventId].status = EVENT_ACTIVE;
    }
}

/**
 * @brief  Inactive event now
 * @param  eventId
 * @retval None
 */

void EVENT_SetInactive(EventControl eventId){
    if(eventId < EVENT_MAX_CONTROL){
        eventControls[eventId].status = EVENT_INACTIVE;
    }
}

/**
 * @brief  Schedule active timer after delay ms
 * @param  eventControl
 *         delay time
 * @retval None
 */

void EVENT_SetDelayMS(EventControl eventId, uint32_t delay){
    if(eventId < EVENT_MAX_CONTROL){
        eventControls[eventId].timeStart = EVENT_TICK_MS();
        eventControls[eventId].timeExcute = delay;
        eventControls[eventId].status = EVENT_ACTIVE;
    }
}

/**
 * @brief  Get time remaining before event is excuted
 * @param  Event index
 * @retval Time remaining
 *         If event is not active, return 0xFFFFFFFF
 */

uint32_t EVENT_GetTimeRemain(EventControl eventId){
    uint32_t start, timeElapse;
	
    if(eventControls[eventId].status != EVENT_ACTIVE){
        return MAX_UINT32_T;
    }

    // Get time remaining
    start = eventControls[eventId].timeStart;
    timeElapse = GetTimeElapse(start);

    if(eventControls[eventId].timeExcute > timeElapse){
        return eventControls[eventId].timeExcute - timeElapse;
    }
    return MAX_UINT32_T;
}


/**
 * @brief  Task to schedule event
 * @param  None
 * @retval None
 */

void EVENT_Task(void){
    uint8_t i;
    for(i = 0; i < EVENT_MAX_CONTROL; i++){
        if(eventControls[i].status == EVENT_ACTIVE){
            if(EVENT_IsExpiredTime(i)){
                eventControls[i].eventHandler();
            }
        }
    }
}

/**
 * @brief  Check time expired of event
 * @param  Event id to check
 * @retval true if time is expired
 */

static uint8_t EVENT_IsExpiredTime(uint8_t eventID){
    uint32_t timeElapse, timeStart;

    // Get time elapse
    timeStart = eventControls[eventID].timeStart;
    timeElapse = GetTimeElapse(timeStart);

    if(timeElapse >= eventControls[eventID].timeExcute){
        return 1;
    }
    return 0;
}

/**
 * @brief  Get milliSecondSystemTick
 * @param  None
 * @retval milliSecondSystemTick
 */

uint32_t EVENT_GetCurrentTickMs(void){
	return EVENT_TICK_MS();
}

#endif

