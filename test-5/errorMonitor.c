/**
******************************************************************************
* @file    errorMonitor.c
* @author  G.L.
* @version 
* @date    5 Jan 2018
* @brief   Error monitoring object responsible for stopping a test script
*          in case of faults or unexpected conditions.
*          
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "poci.h"

#ifndef XACTIVE_H_
#include "xActive.h"
#endif

#include "drvLIS2DH.h"
#include "drvEMC2105.h"
#include "sysErrorCodes.h"
#include "errorMonitor.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ERRORMONITOR_TILT_HYSTERESIS_THRESHOLD (2u) // in sec
#define ERRORMONITOR_TILT_EVENT_PERIODICITY    (5u) // in sec

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Externs -------------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
STATIC XState ErrorMonitorState_Initial(ErrorMonitor_t* pMe, XEvent_t const* pEv);
STATIC XState ErrorMonitorState_Idle(ErrorMonitor_t* pMe, XEvent_t const* pEv);
STATIC XState ErrorMonitorState_TestPrepare(ErrorMonitor_t* pMe, XEvent_t const* pEv);
STATIC XState ErrorMonitorState_TestRunning(ErrorMonitor_t* pMe, XEvent_t const* pEv);
STATIC void ErrorMonitorHanldeNewState(ErrorMonitor_t* pMe, XEvent_t const* pEv);

STATIC eErrorCode ErrorMonitorActOnSampleEvents(ErrorMonitor_t* pMe,
                                                       XEvent_t const* pEv);
STATIC eErrorCode ErrorMonitorActOnStripEvents(ErrorMonitor_t* pMe,
                                                      XEvent_t const* pEv);
STATIC eErrorCode ErrorMonitorActOnDoorEvents(ErrorMonitor_t* pMe,
                                                     XEvent_t const* pEv);
STATIC eErrorCode ErrorMonitorActOnTiltAngle(ErrorMonitor_t* pMe,
                                                    float* pPitch,
                                                    float* pRoll);
STATIC eErrorCode ErrorMonitorActOnAmbientTempRead(ErrorMonitor_t* pMe,
                                                          float* ambientTemp);
STATIC void ErroMonitorPublishTiltStatus(ErrorMonitor_t* pMe);


/**
* @addtogroup MeasurementSystem Measurement System
*  @{
* @brief Measurement thread.
*/

/**
* @brief        Initialisation of error monitor engine.
* @details		  If failed, an assertion is expected!
* @param[in]    pMe - pointer to object.
* @param[in]    pParams - parameters of object.
*/   
void ErrorMonitorInit(ErrorMonitor_t* pMe,
                      ErrorMonitorParams_t* pParams,
                      XActiveFramework_t* pXActiveFramework)
{
  ASSERT_NOT_NULL(pMe);
  ASSERT_NOT_NULL(pParams);

  pMe->pParams = pParams;

  XActive_ctor(&pMe->super, (XStateHandler) ErrorMonitorState_Initial);

  pMe->tickIntervalMs = ERROR_MONITOR_TIMER_TICK;

  XTimerCreate(&(pMe->timer),
               &(pMe->super),
               X_EV_TIMER,
               pMe->tickIntervalMs,
               X_TIMER_NO_START);

  XActiveStart(pXActiveFramework,
							 (XActive_t*)&(pMe->super),
							 "ErrorMonitor",
							 pParams->priority,
							 pMe->evQueueBytes,
							 sizeof(pMe->evQueueBytes),
							 NULL);

  X_EV_INIT(&pMe->errorMonitorSetPreTestChecksEvent, XMSG_ERROR_MONITOR_PRETEST_CHECKS, pMe);
  X_EV_INIT(&pMe->errorMonitorStartEvent, XMSG_ERROR_MONITOR_START, pMe);
  X_EV_INIT(&pMe->errorMonitorStopEvent, XMSG_ERROR_MONITOR_STOP, pMe);
  X_EV_INIT(&pMe->errorMonitorSetExpStateEvent, XMSG_ERROR_MONITOR_SET_EXP_STATE, pMe);
  X_EV_INIT(&pMe->errorMonitorErrCodeEvent, XMSG_ERROR_MONITOR_ERROR_CODE, pMe);
  X_EV_INIT(&pMe->errorMonitorStatusLevelEvent, XMSG_INSTRUMENT_IS_LEVEL, pMe);
  X_EV_INIT(&pMe->errorMonitorStatusTiltedEvent, XMSG_INSTRUMENT_IS_TILTED, pMe);

  X_SUBSCRIBE(pMe, XMSG_DOOR_OPENED);
  X_SUBSCRIBE(pMe, XMSG_DOOR_CLOSED);
  X_SUBSCRIBE(pMe, XMSG_STRIP_DETECTED);
  X_SUBSCRIBE(pMe, XMSG_STRIP_REMOVED);
  X_SUBSCRIBE(pMe, XMSG_SAMPLE_DETECTED);
  X_SUBSCRIBE(pMe, XMSG_SAMPLE_UNDETECTED);
  X_SUBSCRIBE(pMe, XMSG_HEATER_STRIP_TEMP_OUT_OF_RANGE);

	X_SUBSCRIBE_TO_GLOBAL_EVENTS(pMe);
}

/**
* @brief       Initial state of error monitor object.
* @param[in]   pMe - error monitor instance.
* @param[in]   pEv - Will be NULL in this state.
* @return      result - The state transition code.
*/
STATIC XState ErrorMonitorState_Initial(ErrorMonitor_t* pMe, XEvent_t const* pEv)
{
  XState result;

  XTimerStart(&(pMe->timer));
  
  result = X_TRAN(pMe, &ErrorMonitorState_Idle);
  
  return result;
}

/**
* @brief       Error monitor engine enters "Idle" state post initialisation or
*              post test completion.
* @details     NOTE: In idle mode, accelerometer is always monitored and a
*              change of status is published as an event instead of error. An
*              error will be reported in case of accelerometer read failure.
* @param[in]   pMe - Error monitor instance.
* @param[in]   pEv - Could be NULL unless a transition is instructed.
* @return      result - The state transition code.
*/
STATIC XState ErrorMonitorState_Idle(ErrorMonitor_t* pMe, XEvent_t const* pEv)
{
  XState result = X_RET_IGNORED;
  eErrorCode error = ERROR_ERRMON_NONE;
  eErrorCode drvError;
  float pitch;
  float roll;
  
	if (pEv != NULL)
	{
		switch(pEv->id)
		{
      case X_EV_ENTRY:
        pMe->expectedDoorState = ERRMON_EXPECT_DOOR_STATE_IGNORED;
        pMe->expectedSampleState = ERRMON_EXPECT_SAMPLE_STATE_IGNORED;
        pMe->expectedStripState = ERRMON_EXPECT_STRIP_STATE_IGNORED;
        pMe->expectedMaxTiltAngle = INSTRUMENT_MAX_TILT_ANGLE;
        pMe->alternateTimeEventFlag = true;
        pMe->currentTiltStatus = ERROR_ERRMON_INSTRUMENT_IS_LEVEL;
        result = X_RET_HANDLED;
        break;

      case X_EV_TIMER:
        drvError = DrvLIS2DH_GetTiltAngles(&pitch,
                                           &roll,
                                           ERROR_MONITOR_ACCELEROMETER_SAMPLES);
        if (OK_STATUS == drvError)
        {
          (void)ErrorMonitorActOnTiltAngle(pMe, &pitch, &roll);

          ErroMonitorPublishTiltStatus(pMe);
        }
        else if (ERROR_ACCELEROMETER_VIBRATION_DETECTED == drvError)
        {
          /* Ignore vibrations. This is not required based on specifications.*/
        }
        else
        {
          error = ERROR_ERRMON_ACCELEROMETER_NOT_READING;
        }
        result = X_RET_HANDLED;
        break;

      case XMSG_ERROR_MONITOR_PRETEST_CHECKS:
        result = X_TRAN(pMe, &ErrorMonitorState_TestPrepare);
        break;

      default:
        break;
		}
    
    if (ERROR_ERRMON_NONE != error)
    {
      pMe->errorMonitorErrCodeEvent.errorCode = error;
      X_PUBLISH(X_FRAMEWORK_OF(pMe), pMe->errorMonitorErrCodeEvent);
    }
	}
  
	return result;
}

/**
* @brief       Error monitor engine enters "TestPrepare" state to make last minute
*              checks on various things like strip, door and sample.
* @details     NOTE: Accelerometer is always monitored and a
*              change of status is published as an event instead of error. An
*              error will be reported in case of accelerometer read failure.
* @param[in]   pMe - Error monitor instance.
* @param[in]   pEv - Could be NULL unless a transition is instructed.
* @return      result - The state transition code.
*/
STATIC XState ErrorMonitorState_TestPrepare(ErrorMonitor_t* pMe, XEvent_t const* pEv)
{
  XState result = X_RET_IGNORED;
  eErrorCode error = ERROR_ERRMON_NONE;
  eErrorCode drvError;
  float pitch;
  float roll;

  if (pEv != NULL)
  {
    switch(pEv->id)
    {
      case X_EV_ENTRY:
        result = X_RET_IGNORED;
        break;

      case X_EV_TIMER:
        drvError = DrvLIS2DH_GetTiltAngles(&pitch,
                                           &roll,
                                           ERROR_MONITOR_ACCELEROMETER_SAMPLES);
        if (OK_STATUS == drvError)
        {
          (void)ErrorMonitorActOnTiltAngle(pMe, &pitch, &roll);

          ErroMonitorPublishTiltStatus(pMe);
        }
        else if (ERROR_ACCELEROMETER_VIBRATION_DETECTED == drvError)
        {
          /* Ignore vibrations. This is not required based on specifications.*/
        }
        else
        {
          error = ERROR_ERRMON_ACCELEROMETER_NOT_READING;
        }
        result = X_RET_HANDLED;
      break;

      case XMSG_DOOR_OPENED:
        pMe->preTestStatusOf.door = DOOR_STATE_OPEN;
        result = X_RET_HANDLED;
        break;

      case XMSG_DOOR_CLOSED:
        pMe->preTestStatusOf.door = DOOR_STATE_CLOSED;
        result = X_RET_HANDLED;
        break;

      case XMSG_STRIP_DETECTED:
        pMe->preTestStatusOf.strip = STRIP_IN;
        result = X_RET_HANDLED;
        break;

      case XMSG_STRIP_REMOVED:
        pMe->preTestStatusOf.strip = STRIP_OUT;
        result = X_RET_HANDLED;
        break;

      case XMSG_SAMPLE_DETECTED:
        pMe->preTestStatusOf.sampleDetected = true;
        result = X_RET_HANDLED;
        break;

      case XMSG_SAMPLE_UNDETECTED:
        pMe->preTestStatusOf.sampleDetected = false;
        result = X_RET_HANDLED;
        break;

      case XMSG_ERROR_MONITOR_START:
        result = X_TRAN(pMe, &ErrorMonitorState_TestRunning);
        break;

      default:
        break;
    }

    if (ERROR_ERRMON_NONE != error)
    {
      pMe->errorMonitorErrCodeEvent.errorCode = error;
      X_PUBLISH(X_FRAMEWORK_OF(pMe), pMe->errorMonitorErrCodeEvent);
    }
  }

  return result;
}

/**
* @brief			 This is the state of the error monitor engine during test.
* @details     It is event and timer driven.
* @param[in]   pMe - Error monitor instance.
* @param[in]   pEv - Could be NULL unless a transition is instructed.
* @return      result - The state transition code.
*/
STATIC XState ErrorMonitorState_TestRunning(ErrorMonitor_t* pMe, XEvent_t const* pEv)
{
	XState result = X_RET_IGNORED;
	eErrorCode error = ERROR_ERRMON_NONE;
  eErrorCode drvError;
  float pitch;
  float roll;
  float ambientTemp;
  
	switch (pEv->id)
	{
    case X_EV_ENTRY:
      /* Making sure that some checks are done again before kick off */
      if (DOOR_STATE_OPEN != pMe->preTestStatusOf.door)
      {
        error = ERROR_ERRMON_DOOR_CLOSED;
      }

      if (STRIP_IN != pMe->preTestStatusOf.strip)
      {
        error = ERROR_ERRMON_STRIP_REMOVED;
      }

      if (pMe->preTestStatusOf.sampleDetected)
      {
        error = ERROR_ERRMON_SAMPLE_DETECTED;
      }

    	result = X_RET_HANDLED;
      break;

    case X_EV_TIMER:
      if (pMe->alternateTimeEventFlag)
      {
        /* Check tilt angle. */
        drvError = DrvLIS2DH_GetTiltAngles(&pitch,
                                           &roll,
                                           ERROR_MONITOR_ACCELEROMETER_SAMPLES);
        if (OK_STATUS == drvError)
        {
          error = ErrorMonitorActOnTiltAngle(pMe, &pitch, &roll);
        }
        else if (ERROR_ACCELEROMETER_VIBRATION_DETECTED == drvError)
        {
          /* Ignore vibrations. This is not required based on specifications.*/
        }
        else
        {
          error = ERROR_ERRMON_ACCELEROMETER_NOT_READING;
        }

        pMe->alternateTimeEventFlag = false;
      }
      else
      {
        /* Check ambient temperature. */
        drvError = DrvEMC2105_GetExternalTemperature(&ambientTemp);

        if (OK_STATUS == drvError)
        {
          error = ErrorMonitorActOnAmbientTempRead(pMe, &ambientTemp);
        }
        else
        {
          error = ERROR_ERRMON_AMBIENT_TEMP_NOT_READING;
        }

        pMe->alternateTimeEventFlag = true;
      }

      result = X_RET_HANDLED;
    break;

    case XMSG_ERROR_MONITOR_SET_EXP_STATE:
      ErrorMonitorHanldeNewState(pMe, pEv);
      result = X_RET_HANDLED;
      break;

    case XMSG_DOOR_OPENED:
    case XMSG_DOOR_CLOSED:
    	error = ErrorMonitorActOnDoorEvents(pMe, pEv);
  		result = X_RET_HANDLED;
    	break;

    case XMSG_STRIP_DETECTED:
    case XMSG_STRIP_REMOVED:
    	error = ErrorMonitorActOnStripEvents(pMe, pEv);
   		result = X_RET_HANDLED;
    	break;

    case XMSG_SAMPLE_DETECTED:
    case XMSG_SAMPLE_UNDETECTED:
    	error = ErrorMonitorActOnSampleEvents(pMe, pEv);
   		result = X_RET_HANDLED;
    	break;

    case XMSG_ERROR_MONITOR_STOP:
    	result = X_TRAN(pMe, &ErrorMonitorState_Idle);
    	break;

    case XMSG_HEATER_STRIP_TEMP_OUT_OF_RANGE:
        error = ERROR_ERRMON_STRIP_TEMP_OUT_OF_RANGE;
        result = X_RET_HANDLED;
        break;

    default:
      break;
	}
  
	if (ERROR_ERRMON_NONE != error)
	{
		pMe->errorMonitorErrCodeEvent.errorCode = error;
		X_PUBLISH(X_FRAMEWORK_OF(pMe), pMe->errorMonitorErrCodeEvent);
	}

	return result;
}

/**
* @brief			 Helper call to determine unexpected state and flag error.
* @param[in]   pMe - Error monitor instance.
* @param[in]   pEv - Could be NULL unless a transition is instructed.
* @return      eErrorCode - Door errors or ERROR_ERRMON_NONE.
*/
STATIC eErrorCode ErrorMonitorActOnDoorEvents(ErrorMonitor_t* pMe,
		 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	       XEvent_t const* pEv)
{
  eErrorCode errMonError;

	if ((XMSG_DOOR_OPENED == pEv->id) &&
			(ERRMON_EXPECT_DOOR_CLOSED == pMe->expectedDoorState))
	{
	  errMonError = ERROR_ERRMON_DOOR_OPEN;
	}
	else if ((XMSG_DOOR_CLOSED == pEv->id) &&
	     	 	 (ERRMON_EXPECT_DOOR_OPEN == pMe->expectedDoorState))
	{
	  errMonError = ERROR_ERRMON_DOOR_CLOSED;
	}
	else
	{
	  errMonError = ERROR_ERRMON_NONE;
	}

	return errMonError;
}

/**
* @brief			 Helper call to determine unexpected state and flag error.
* @param[in]   pMe - Error monitor instance.
* @param[in]   pEv - Could be NULL unless a transition is instructed.
* @return      eErrorCode - Strip errors or ERROR_ERRMON_NONE.
*/
STATIC eErrorCode ErrorMonitorActOnStripEvents(ErrorMonitor_t* pMe,
	 	 	 	 	 	 	 	 															 	 	 	  XEvent_t const* pEv)
{
  eErrorCode errMonError;

	if ((XMSG_STRIP_DETECTED == pEv->id) &&
			(ERRMON_EXPECT_STRIP_REMOVED == pMe->expectedStripState))
	{
	  errMonError = ERROR_ERRMON_STRIP_DETECTED;
	}
	else if ((XMSG_STRIP_REMOVED == pEv->id) &&
	     	 	 (ERRMON_EXPECT_STRIP_DETECTED == pMe->expectedStripState))
	{
	  errMonError = ERROR_ERRMON_STRIP_REMOVED;
	}
	else
	{
	  errMonError = ERROR_ERRMON_NONE;
	}

	return errMonError;
}

/**
* @brief			 Helper call to determine unexpected state and flag error.
* @param[in]   pMe - Error monitor instance.
* @param[in]   pEv - Could be NULL unless a transition is instructed.
* @return      eErrorCode - Sample errors or ERROR_ERRMON_NONE.
*/
STATIC eErrorCode ErrorMonitorActOnSampleEvents(ErrorMonitor_t* pMe,
	 	 	 	 	 	 	 	 															 	 	 	  XEvent_t const* pEv)
{
  eErrorCode errMonError;

	if ((XMSG_SAMPLE_DETECTED == pEv->id) &&
			(ERRMON_EXPECT_SAMPLE_NOT_DETECTED == pMe->expectedSampleState))
	{
	  errMonError = ERROR_ERRMON_SAMPLE_DETECTED;
	}
	else if ((XMSG_SAMPLE_UNDETECTED == pEv->id) &&
	     	 	 (ERRMON_EXPECT_SAMPLE_DETECTED == pMe->expectedSampleState))
	{
	  errMonError = ERROR_ERRMON_SAMPLE_NOT_DETECTED;
	}
	else
	{
	  errMonError = ERROR_ERRMON_NONE;
	}

	return errMonError;
}

/**
* @brief       Helper call to act upon accelerometer reading.
* @details     Tilted error is published only when hysteresis threshold is met
* @param[in]   pMe - Error monitor instance.*
* @param[in]   pPitch - Pointer to measured value.
* @param[in]   pRoll - Pointer to measured value.
* @return      eErrorCode - Tilt error or ERROR_ERRMON_NONE.
*/
STATIC eErrorCode ErrorMonitorActOnTiltAngle(ErrorMonitor_t* pMe,
                                             float* pPitch,
                                             float* pRoll)
{
  eErrorCode errMonError = ERROR_ERRMON_NONE;
  static uint8_t hysteresisCntr = 0u;

  if ( ((float)fabs(*pPitch) > pMe->expectedMaxTiltAngle) ||
       ((float)fabs(*pRoll) > pMe->expectedMaxTiltAngle) )
  {
    hysteresisCntr++;

    if (ERRORMONITOR_TILT_HYSTERESIS_THRESHOLD <= hysteresisCntr)
    {
      /* Based on ERROR_MONITOR_TIMER_TICK, this means that instrument
       * was/is in tilted angle for at least 2sec. */
      errMonError = ERROR_ERRMON_TILT_ANGLE_OUT_OF_RANGE;
      pMe->newTiltStatus = ERROR_ERRMON_INSTRUMENT_IS_TILTED;
      hysteresisCntr = 0u;
    }
  }
  else
  {
    pMe->newTiltStatus = ERROR_ERRMON_INSTRUMENT_IS_LEVEL;
    hysteresisCntr = 0u;
  }

  return errMonError;
}

/**
* @brief       Helper call to act upon fan controller reading.
* @param[in]   pMe - Error monitor instance.*
* @param[in]   ambientTemp - Pointer to measured value.
* @return      eErrorCode - Ambient temp error or ERROR_ERRMON_NONE.
*/
STATIC eErrorCode ErrorMonitorActOnAmbientTempRead(ErrorMonitor_t* pMe,
                                                   float* ambientTemp)
{
  eErrorCode errMonError;

  if ((uint32_t)*ambientTemp >= pMe->expectedAmbientTemp)
  {
    errMonError = ERROR_ERRMON_AMBIENT_TEMP_OUT_OF_RANGE;
  }
  else
  {
    errMonError = ERROR_ERRMON_NONE;
  }

  return errMonError;
}

/**
* @brief       Helper call to publish tilt status event based on new and current
*              state.
* @param[in]   pMe - Error monitor instance.
* @return      None.
*/
STATIC void ErroMonitorPublishTiltStatus(ErrorMonitor_t* pMe)
{
  static uint8_t periodicTiltedCntr = 0u;
  
  if ((ERROR_ERRMON_INSTRUMENT_IS_LEVEL == pMe->currentTiltStatus) &&
      (ERROR_ERRMON_INSTRUMENT_IS_TILTED == pMe->newTiltStatus))
  {
    X_PUBLISH(X_FRAMEWORK_OF(pMe), pMe->errorMonitorStatusTiltedEvent);
    periodicTiltedCntr = 0u;
  }
  else if ((ERROR_ERRMON_INSTRUMENT_IS_TILTED == pMe->currentTiltStatus) &&
      (ERROR_ERRMON_INSTRUMENT_IS_TILTED == pMe->newTiltStatus))
  {
    periodicTiltedCntr++;
    
    if (ERRORMONITOR_TILT_EVENT_PERIODICITY <= periodicTiltedCntr)
    {
      X_PUBLISH(X_FRAMEWORK_OF(pMe), pMe->errorMonitorStatusTiltedEvent);
      periodicTiltedCntr = 0u;
    }
  }
  

  if ((ERROR_ERRMON_INSTRUMENT_IS_TILTED == pMe->currentTiltStatus) &&
      (ERROR_ERRMON_INSTRUMENT_IS_LEVEL == pMe->newTiltStatus))
  {
    X_PUBLISH(X_FRAMEWORK_OF(pMe), pMe->errorMonitorStatusLevelEvent);
    periodicTiltedCntr = 0u;
  }

  pMe->currentTiltStatus = pMe->newTiltStatus;
}

/**
* @brief       Helper call to handle/copy new states to the "expected" ones on
*              event driven basis.
* @details
* @param[in]   pMe - Error monitor instance.
* @param[in]   pEv - Could be NULL unless a transition is instructed.
* @return      None.
*/
STATIC void ErrorMonitorHanldeNewState(ErrorMonitor_t* pMe, XEvent_t const* pEv)
{
  const ErrMonErrorSetExpStateEvent_t* pSet = (const ErrMonErrorSetExpStateEvent_t*)pEv;

  pMe->expectedDoorState = pSet->newDoorState;
  pMe->expectedSampleState = pSet->newSampleState;
  pMe->expectedStripState = pSet->newStripState;
  pMe->expectedAmbientTemp = pSet->newMaxAmbientTemp;
}

/**
* @brief			 API call used to set the expected states of modules
* @details     This API supports Door, Strip and Sample states
* @param[in]   pMe - Error monitor instance.
* @param[in]   state - Expected states of component.
* @return      None.
*/
void ErrorMonitorSetNewState(ErrorMonitor_t* pMe,
                             eErrMonExpectedStates newExpectedstate)
{
  bool setExpectedState = false;

  switch (newExpectedstate)
  {
    case ERRMON_EXPECT_DOOR_STATE_IGNORED:
    case ERRMON_EXPECT_DOOR_OPEN:
    case ERRMON_EXPECT_DOOR_CLOSED:
      if (pMe->expectedDoorState != newExpectedstate)
      {
        pMe->errorMonitorSetExpStateEvent.newDoorState = newExpectedstate;
        setExpectedState = true;
      }
      break;

    case ERRMON_EXPECT_SAMPLE_STATE_IGNORED:
    case ERRMON_EXPECT_SAMPLE_DETECTED:
    case ERRMON_EXPECT_SAMPLE_NOT_DETECTED:
      if (pMe->expectedSampleState != newExpectedstate)
      {
        pMe->errorMonitorSetExpStateEvent.newSampleState = newExpectedstate;
        setExpectedState = true;
      }
      break;

    case ERRMON_EXPECT_STRIP_STATE_IGNORED:
    case ERRMON_EXPECT_STRIP_DETECTED:
    case ERRMON_EXPECT_STRIP_REMOVED:
      if (pMe->expectedStripState != newExpectedstate)
      {
        pMe->errorMonitorSetExpStateEvent.newStripState = newExpectedstate;
        setExpectedState = true;
      }
      break;

    default:
      break;
  }

  if (setExpectedState)
  {
    X_POST(pMe, pMe->errorMonitorSetExpStateEvent);
  }
}

/**
* @brief       API call to set the expected Ambient Temperature.
* @details     Error Monitor will be checking against this value during a test.
* @param[in]   pMe - Error monitor instance.
* @param[in]   ambientTempValue - New value set by test script (lot file).
* @return      None.
*/
void ErrorMonitorSetMaxAmbientTempFromLot(ErrorMonitor_t* pMe,
                                          uint32_t ambientTempValue)
{
  pMe->errorMonitorSetExpStateEvent.newMaxAmbientTemp = ambientTempValue;

  X_POST(pMe, pMe->errorMonitorSetExpStateEvent);
}

/**
* @brief			 API call to start error monitor engine
* @details
* @param[in]   pMe - Error monitor instance.
* @return      None.
*/
void ErrorMonitorStart(ErrorMonitor_t* pMe)
{
	X_POST(pMe, pMe->errorMonitorStartEvent);
}

void ErrorMonitorSetPreTestChecks(ErrorMonitor_t* pMe)
{
  X_POST(pMe, pMe->errorMonitorSetPreTestChecksEvent);
}


/**
* @brief			 API call to stop error monitor engine
* @details
* @param[in]   pMe - Error monitor instance.
* @return      None.
*/
void ErrorMonitorStop(ErrorMonitor_t* pMe)
{
	X_POST(pMe, pMe->errorMonitorStopEvent);
}

/**
* @}
*/
/********************************** End Of File ******************************/
