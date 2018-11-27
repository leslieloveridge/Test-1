/**
 ******************************************************************************
 * @file    errorMonitor.h
 * @author  G.L.
 * @version 
 * @date    5 Jan 2018
 * @brief   Declarations of error monitor object
 *         
 ******************************************************************************
 */

/* Define to prevent recursive inclusion  */
#ifndef ERROR_MONITOR_H
#define ERROR_MONITOR_H
 
#include "poci.h"
#ifdef WIN32
#include "xTimerWinPortab.h"
#endif

#ifndef XACTIVE_H_
#include "xActive.h"
#endif


/* Type Definitions ---------------------------------------------------------*/

/**
  *@brief		ErrMonExpectedStates_tag
  *@details Expected states for various peripherals/modules
  *         during error monitoring
  **/
typedef enum ErrMonExpectedStates_tag
{
  ERRMON_EXPECT_DOOR_STATE_IGNORED = 0x00,
  ERRMON_EXPECT_DOOR_OPEN,
  ERRMON_EXPECT_DOOR_CLOSED,

  ERRMON_EXPECT_SAMPLE_STATE_IGNORED = 0x10,
  ERRMON_EXPECT_SAMPLE_DETECTED,
  ERRMON_EXPECT_SAMPLE_NOT_DETECTED,

  ERRMON_EXPECT_STRIP_STATE_IGNORED = 0x20,
  ERRMON_EXPECT_STRIP_DETECTED,
  ERRMON_EXPECT_STRIP_REMOVED,
}
eErrMonExpectedStates;

/**
* @brief Object's priority
*/  
typedef struct ErrorMonitorParams_tag
{
  uint8_t priority;
}ErrorMonitorParams_t;

/**
* @brief Event which publishes error codes
*/
typedef struct ErrMonErrorCodeEvent_tag
{
  XActive_t super;
  eErrorCode errorCode;
}ErrMonErrorCodeEvent_t;

/**
* @brief New states to change to.
*/
typedef struct ErrMonErrorSetExpStateEvent_tag
{
  XActive_t super;
  eErrMonExpectedStates newDoorState;
  eErrMonExpectedStates newStripState;
  eErrMonExpectedStates newSampleState;
  uint32_t newMaxAmbientTemp;
}ErrMonErrorSetExpStateEvent_t;

typedef struct ErrMonPreTestStatus_tag
{
  eDoorState door;
  eStripState_t strip;
  bool sampleDetected;
}ErrMonPreTestStatus_t;

/**
* @brief Object's main events and core variables
*/
typedef struct ErrorMonitor_tag
{
  XActive_t super;
  XTimer_t timer;
  uint32_t evQueueBytes[32];
  uint32_t tickIntervalMs;

  XEvent_t errorMonitorSetPreTestChecksEvent;
  XEvent_t errorMonitorStartEvent;
  XEvent_t errorMonitorStopEvent;
  XEvent_t errorMonitorStatusLevelEvent;
  XEvent_t errorMonitorStatusTiltedEvent;
  ErrMonErrorCodeEvent_t errorMonitorErrCodeEvent;
  ErrMonErrorSetExpStateEvent_t errorMonitorSetExpStateEvent;
  ErrMonPreTestStatus_t preTestStatusOf;

  eErrMonExpectedStates expectedDoorState;
  eErrMonExpectedStates expectedStripState;
  eErrMonExpectedStates expectedSampleState;

  uint32_t expectedAmbientTemp;
  float expectedMaxTiltAngle;
  bool alternateTimeEventFlag;

  eErrorCode newTiltStatus;
  eErrorCode currentTiltStatus;

  ErrorMonitorParams_t* pParams;
}ErrorMonitor_t;

/* Header Files Includes ----------------------------------------------------*/

/* Constant Definitions -----------------------------------------------------*/

/* Macros Definitions -------------------------------------------------------*/
#define ERROR_MONITOR_TIMER_TICK (1000u)          ///< In ms
#define ERROR_MONITOR_ACCELEROMETER_SAMPLES (10u) ///< Number of readings

/* Non Static Function Definitions ------------------------------------------*/
void ErrorMonitorInit(ErrorMonitor_t* pMe,
                      ErrorMonitorParams_t* pParams,
                      XActiveFramework_t* pXActiveFramework);

void ErrorMonitorSetNewState(ErrorMonitor_t* pMe, eErrMonExpectedStates state);
void ErrorMonitorSetMaxAmbientTempFromLot(ErrorMonitor_t* pMe,
                                          uint32_t ambientTempValue);

void ErrorMonitorStart(ErrorMonitor_t* pMe);
void ErrorMonitorSetPreTestChecks(ErrorMonitor_t* pMe);
void ErrorMonitorStop(ErrorMonitor_t* pMe);

#endif /* ERROR_MONITOR_H */

/********************************** End Of File ******************************/



