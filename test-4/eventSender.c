/**
******************************************************************************
* @file    eventSender.c
* @author DEW
* @version
* @date 23/08/2017
*    
* @brief This object listens for events and sends to the Scheduler layer API
* if applicable.
******************************************************************************
*/



#include "poci.h"
#include "xActive.h"
#include "schedulerAPI.h"
#include "consoleApi.h"
#include "drvBarCodeReader.h"
#include "opticalHctAnalysis.h"
#include "assayCalculate.h"
#include "assayCalculateTimeBased.h"
#include "fluidics.h"
#include "realTimeInr.h"
#include "opticalHct.h"
#include "schedulerAPI.h"
#include "measurementDemo.h"
#include "measurementAPI.h"
#include "dxScriptRunner.h"
#include "alManager.h"
#include "eventSender.h"



/**
* @addtogroup eventSender
*  @{
* @brief Sends selected events to the Scheduler)
*/


#ifdef WIN32
#error "Cannot build  on Win" 
#endif



STATIC XState EventSenderState_Active(EventSender_t* pEventSender,
                                      XEvent_t const* pEvent);

STATIC void EventNotifyScheduler(XEvent_t const* pEvent);
STATIC void EventSend(XEvent_t const* pEvent);
STATIC void EventLog(XEvent_t const* pEvent);

STATIC void EventSenderProcessFluidMoveComplete(EventSender_t* pEventSender,
                                                XEvent_t const * pEvent);
STATIC void EventSenderProcessBarcodeResult(EventSender_t* pEventSender,
                                            XEvent_t const * pEvent);
STATIC void EventSenderProcessBarcodeMisreadResult(EventSender_t* pEventSender,
                                                   XEvent_t const * pEvent);
STATIC void EventSenderProcessOpticalHctSelfTestResult(EventSender_t* pEventSender,
                                                       XEvent_t const * pEvent);
STATIC void EventSenderOnRealTimeInrClotResult(XEvent_t const * pEvent);

STATIC void EventSenderProcessCommandFailedEvent(EventSender_t * pEventSender,
                                                 XEvent_t const * pEvent);


/**
* @brief  Init the  sub-module.
* @details Initailise the timer, and message queue. Subscribe to events.
* @param pEventSender The instance to initialise 
* @param  pParams The initial setup
* @param pXFramework Pointer to the parent framework.
*/
void EventSenderInit(EventSender_t* pEventSender,
                     const EventSenderParams_t* pParams,
                     XActiveFramework_t* pXActiveFramework)
{
  ASSERT_NOT_NULL(pEventSender);
  ASSERT_NOT_NULL(pParams);
  ASSERT_NOT_NULL(pXActiveFramework);
  
  pEventSender->pParams = pParams;
  
  // Init and start the Active object base class.
  XActive_ctor(&pEventSender->super, (XStateHandler)&EventSenderState_Active);
  
  XActiveStart(pXActiveFramework,
               (XActive_t*)&(pEventSender->super),
               "EventSender",
               pEventSender->pParams->priority,
               pEventSender->evQueueBytes,
               sizeof(pEventSender->evQueueBytes),
               NULL);
  
  
  // List the events that should be emitted to the App Layer here:-
  X_SUBSCRIBE_TO_GLOBAL_EVENTS(pEventSender);
  X_SUBSCRIBE(pEventSender, XMSG_COMMAND_COMPLETE);
  X_SUBSCRIBE(pEventSender, XMSG_COMMAND_FAILED);
  X_SUBSCRIBE(pEventSender, XMSG_COMMAND_TIMEOUT);
  
  X_SUBSCRIBE(pEventSender, XMSG_DOOR_OPENED);   
  X_SUBSCRIBE(pEventSender, XMSG_DOOR_CLOSED);  
  
  X_SUBSCRIBE(pEventSender, XMSG_STRIP_DETECTED);                     ///< Published when a strip is inserted
  X_SUBSCRIBE(pEventSender, XMSG_STRIP_REMOVED);                    ///< Published when a strip is inserted
  
  X_SUBSCRIBE(pEventSender, XMSG_SAMPLE_DETECTED);
  X_SUBSCRIBE(pEventSender, XMSG_SAMPLE_UNDETECTED);
  
  X_SUBSCRIBE(pEventSender, XMSG_LOT_NUMBER);
  X_SUBSCRIBE(pEventSender, XMSG_TEST_STATUS_UPDATE);
  X_SUBSCRIBE(pEventSender, XMSG_TEST_COMPLETE);
  X_SUBSCRIBE(pEventSender, XMSG_BREACH_DETECTED);
  X_SUBSCRIBE(pEventSender, XMSG_SPECTRO_SCAN_DATA_COMPLETED);
  
#if 0
  //This will publish a tonne of messages. Therefore removed.
  X_SUBSCRIBE(pEventSender, XMSG_EC_FLUID_STATUS_CHANGED); 
#endif 
  
  
  X_SUBSCRIBE(pEventSender, XMSG_BARCODE_TRIGGER);
  X_SUBSCRIBE(pEventSender, XMSG_BARCODE_REVSOFT);
  X_SUBSCRIBE(pEventSender, XMSG_BARCODE_READ_RESULT);
  X_SUBSCRIBE(pEventSender, XMSG_BARCODE_MISREAD);  
  
  X_SUBSCRIBE(pEventSender, XMSG_SPECTRO_SCAN_CHANNEL_COMPLETE);
  
  X_SUBSCRIBE(pEventSender, XMSG_SCRIPT_COMPLETE);  

  X_SUBSCRIBE(pEventSender, XMSG_EMAG_STABLE);
  X_SUBSCRIBE(pEventSender, XMSG_EMAG_DISABLED);
  X_SUBSCRIBE(pEventSender, XMSG_EMAG_FAIL);
  
  X_SUBSCRIBE(pEventSender, XMSG_FMOVE_CMPLT);
  
  X_SUBSCRIBE(pEventSender, XMSG_REALTIME_INR_CLOT_RESULT);
  
  X_SUBSCRIBE(pEventSender, XMSG_EC_A1_BLDR_DOWN);
  X_SUBSCRIBE(pEventSender, XMSG_EC_A1_BLDR_UP);
  X_SUBSCRIBE(pEventSender, XMSG_EC_A3_BLDR_DOWN);
  X_SUBSCRIBE(pEventSender, XMSG_EC_A3_BLDR_UP);
  X_SUBSCRIBE(pEventSender, XMSG_EC_B2_BLDR_DOWN);
  X_SUBSCRIBE(pEventSender, XMSG_EC_B2_BLDR_UP);
  X_SUBSCRIBE(pEventSender, XMSG_EC_B4_BLDR_DOWN);
  X_SUBSCRIBE(pEventSender, XMSG_EC_B4_BLDR_UP);
  
  X_SUBSCRIBE(pEventSender, XMSG_ERROR_MONITOR_ERROR_CODE);
  X_SUBSCRIBE(pEventSender, XMSG_INSTRUMENT_IS_LEVEL);
  X_SUBSCRIBE(pEventSender, XMSG_INSTRUMENT_IS_TILTED);
  X_SUBSCRIBE(pEventSender, XMSG_HEATER_STRIP_TEMP_OUT_OF_RANGE);
  
  X_SUBSCRIBE(pEventSender, XMSG_OHCT_ST_PASS);
  X_SUBSCRIBE(pEventSender, XMSG_OHCT_ST_FAIL);
  
  X_SUBSCRIBE(pEventSender, XMSG_HTR_STABLE);
  X_SUBSCRIBE(pEventSender, XMSG_SPECTRO_SCAN_SELF_TEST_PASSED);
  X_SUBSCRIBE(pEventSender, XMSG_SPECTRO_SCAN_SELF_TEST_FAILED);
}



/**
* @brief  Active state handler
* @details state handler for listening to subscribed events, and posting them to 
* the  App layer.
* @param pEventSender The instance the message is sent to
* @param pEvent Will be NULL for initial state (passed in XActiveStart()) 
*
* @return The state transition code
*/   
STATIC XState EventSenderState_Active(EventSender_t* pEventSender, 
                                      XEvent_t const* pEvent)
{
  if (NULL == pEvent)
  {
    return  X_RET_IGNORED;
  }
  
  XState returnCode = X_RET_IGNORED;
  
  switch (pEvent->id)
  {
  case X_EV_ENTRY:
    returnCode = X_RET_HANDLED;
    break;
    
  case X_EV_EXIT:
    break;
    
  case X_EV_TIMER:
    returnCode = X_RET_IGNORED;
    break;

  case XMSG_FMOVE_CMPLT:
    EventSenderProcessFluidMoveComplete(pEventSender,
                                        pEvent);
    returnCode = X_RET_HANDLED;
    break;
    
  case XMSG_BARCODE_READ_RESULT:
    EventSenderProcessBarcodeResult(pEventSender,
                                    pEvent);
    returnCode = X_RET_HANDLED;
    break;
    
  case XMSG_BARCODE_MISREAD:
    EventSenderProcessBarcodeMisreadResult(pEventSender,
                                           pEvent);
    returnCode = X_RET_HANDLED;
    break;
    
  case XMSG_REALTIME_INR_CLOT_RESULT:
    EventSenderOnRealTimeInrClotResult(pEvent);
    returnCode = X_RET_HANDLED;
    break;    
 
  case XMSG_OHCT_ST_PASS:
  case XMSG_OHCT_ST_FAIL:
    EventSenderProcessOpticalHctSelfTestResult(pEventSender,
                                               pEvent);
    returnCode = X_RET_HANDLED;
    break;
    
  case XMSG_COMMAND_FAILED:
    EventSenderProcessCommandFailedEvent(pEventSender,
                                               pEvent);
    returnCode = X_RET_HANDLED;
    break;
    
    //
    // All subscribed messages...
    //
  default:
    //
    // Print to logging for debug
    //
    EventLog(pEvent);
    //
    // Inform Scheduler API.
    //
    EventNotifyScheduler(pEvent);
    EventSend(pEvent);
    returnCode = X_RET_HANDLED;
    
    break;
  }
  
  return returnCode;
}



/**
* @brief  Helper to send an event to the Scheduler API layer.
* @details Each relevant event calls the appropriate Scheduler API.
* @note 
* @param pEvent The event to send.
*/   
STATIC void EventNotifyScheduler(XEvent_t const* pEvent)
{
 const dxScriptRunnerScriptComplete_t * pScriptCompelteEvent = 
                                  (const dxScriptRunnerScriptComplete_t*)pEvent;
  
  eErrorCode error;
  
  switch (pEvent->id)
  {
  case XMSG_DOOR_OPENED:
    error = SchAPI_DoorOpen();
    break;
    
  case XMSG_DOOR_CLOSED:
    error = SchAPI_DoorClosed();
    break;
    
  case XMSG_STRIP_DETECTED:
    error = SchAPI_StripDetected();
    break;
    
  case  XMSG_STRIP_REMOVED:
    error = SchAPI_StripNotDetected();
    break;
    
  case XMSG_SAMPLE_DETECTED:
    error = SchAPI_SampleDetected();
    break;
    
  case XMSG_SAMPLE_UNDETECTED:
    error = SchAPI_SampleNotDetected();
    break;
    
  case XMSG_TEST_STATUS_UPDATE:
    error = SchAPI_TestStatus((uint8_t) 123 /*progress*/);
    break;
    
  case XMSG_INSTRUMENT_IS_LEVEL:
    error = SchAPI_InstrumentIsLevel();
    break;
    
  case XMSG_INSTRUMENT_IS_TILTED:
    error = SchAPI_InstrumentIsTilted();
    break;
    
  case XMSG_SCRIPT_COMPLETE:
    
    ///
    /// Perform final calculations and process the results
    /// @note. Any error in this step will be populated in the results structure
    /// because, whatever the outcome, the scheduler is to be notified
    /// that the test is complete. In case an INR has been found and an 
    /// error occurs post INR, the test rerults should still be populated 
    /// but without numerical results. Instead, we should still add information regarding the
    /// type of assay run. In disasterous errors (i.e. script didn't reach INR calculation)
    /// follow the "test terminate" approach
    ///
    AssayCalculationOnScriptCompletion(pScriptCompelteEvent->eError);
    if (OK_STATUS == pScriptCompelteEvent->eError)
    {
      error = SchAPI_TestCompleted();
    }
    else
    {
      TestResultAPI_UpdateOnTestTerminate(pScriptCompelteEvent->eError);
      SchAPI_TestTerminated(pScriptCompelteEvent->eError);
    }

    /* This is to meet SRS-3098. */
    AUDITLOG_ERROR(pScriptCompelteEvent->eError, "Test error code.");
    break;
    
  default:
    /// An event that is not of interest to Scheduler
    error = OK_STATUS;
      break;
  }
  
  ERROR_CHECK(error);
}



/**
* @brief  Helper to send an event to the Console  .
*
* @note The message box should be checked that it can receive the event first.
* @param pEvent The event to send.
*/   
STATIC void EventSend(XEvent_t const* pEvent)
{
  ASSERT_NOT_NULL(pEvent);
  ASSERT_NOT_NULL(pEvent->sender);
  
  char sourceBuffer[20u] = {"SOURCE:"};
  
  // Add the event source to the list.
  (void)strncat(sourceBuffer,
                XActiveName((XActive_t const *)pEvent->sender),
                sizeof(sourceBuffer));
   
   
   Console_PublishEvent("INS",
                        (uint32_t)pEvent->id,
                        XMsgIdLookup(pEvent->id),
                        sourceBuffer);
}



/**
* @brief  Helper function to log the event.
* @details Prints the event to the console port
*          Format =
*               EventSender:- <ObjectID> Event
*               ID = <eventId> : <eventIdString>
* @param[in] pEvent - The event to print.
**/
STATIC void EventLog(XEvent_t const* pEvent)
{
  /// Format of the output designed to fit in 60 character buffer
  //  LOG_TRACE("EventSender:- %10s Event",
  //      X_NAME(pEvent->sender));
  //  LOG_TRACE("ID = %2u : %s",
  //      pEvent->id,
  //      XMsgIdLookup(pEvent->id));
  
}



/**
* @brief   Helper call to populate the fields of barcode misread result msg.
* @param   pEvent - The event to send
*/
STATIC void EventSenderProcessBarcodeMisreadResult(EventSender_t* pEventSender,
                                                   XEvent_t const * pEvent)
{
  const BarcodeMisreadEvent_t* pBarcodeMisreadResultMsg = 
    (const BarcodeMisreadEvent_t *) pEvent;
  
  if (NULL != pEvent)
  {
    (void)SchAPI_BarcodeRead(CMD_OK, 
                             (char*)pBarcodeMisreadResultMsg->barcodeBytes);
    
    // Add the event source to the list.
    (void)snprintf(pEventSender->eventPayloadBuffer,
                   sizeof(pEventSender->eventPayloadBuffer),
                   "%s",
                   pBarcodeMisreadResultMsg->barcodeBytes);
     
     Console_PublishEvent("INS",
                          (uint32_t)pEvent->id,
                          XMsgIdLookup(pEvent->id),
                          pEventSender->eventPayloadBuffer);
  }
  else
  {
    ERROR_CHECK(ERROR_NULL_PTR);
  }
}



/**
* @brief   Helper call to populate the fields of barcode result msg.
* @details Publishes barcode num or reader's version nume.
* @param   pEvent - The event to send
*/
STATIC void EventSenderProcessBarcodeResult(EventSender_t* pEventSender,
                                            XEvent_t const* pEvent)
{
  const BarcodeReadEvent_t* pBarcodeReadResultMsg = 
    (const BarcodeReadEvent_t*) pEvent;
  
  if (NULL != pEvent)
  {
    (void)SchAPI_BarcodeRead(CMD_OK, (char*)pBarcodeReadResultMsg->barcodeBytes);
    
    // Add the event source to the list.
    (void)snprintf(pEventSender->eventPayloadBuffer,
                   sizeof(pEventSender->eventPayloadBuffer),
                   "%s",
                   pBarcodeReadResultMsg->barcodeBytes);
     
     Console_PublishEvent("INS",
                          (uint32_t)pEvent->id,
                          XMsgIdLookup(pEvent->id),
                          pEventSender->eventPayloadBuffer);
  }
  else
  {
    ERROR_CHECK(ERROR_NULL_PTR);
  }
}



/**
* @brief  Helper to the Fluid Move Complete event to the terminal.
* @details Adds the channel, and the movement time to the event. 
* @param pEvent The event to send.
*/   
STATIC void EventSenderProcessFluidMoveComplete(EventSender_t* pEventSender,
                                                XEvent_t const * pEvent)
{
  const FluidicMoveSuccessMsg_t * pMoveCompleteMsg = 
    (const FluidicMoveSuccessMsg_t *) pEvent;
  
  // Add the event source to the list.
  (void)snprintf(pEventSender->eventPayloadBuffer,
                 sizeof(pEventSender->eventPayloadBuffer),
                 "CH:%u,T:%g,PV:%.3f",
                 pMoveCompleteMsg->eChannel,
                 (float)pMoveCompleteMsg->completionTimeMs/1000.f,
                 pMoveCompleteMsg->piezoVolts);
   
   Console_PublishEvent("INS",
                        (uint32_t)pEvent->id,
                        XMsgIdLookup(pEvent->id),
                        pEventSender->eventPayloadBuffer);
}



/**                                              
* @brief  Clot Time Result Helper to publish results to the terminal.
*        Called on OK, or failure to obtain a clot time.
*
* @param pEvent The event to send.
*/
STATIC void EventSenderOnRealTimeInrClotResult(XEvent_t const * pEvent)
{
  const RealTimeInrClotResultEvent_t* pClotResultEvent =
    (const RealTimeInrClotResultEvent_t*) pEvent;
  
  char eventDataBuffer[30u];
  
  // Add the event source to the list.
  (void)snprintf(eventDataBuffer,
                 sizeof(eventDataBuffer),
                 "%3.1fs",
                 pClotResultEvent->clotTimeSeconds);
   
   Console_PublishEvent("INS",
                        (uint32_t)pEvent->id,
                        XMsgIdLookup(pEvent->id),
                        eventDataBuffer);
}



/**
* @brief Helper to print the results of an Optical HCT self test.
* @param[in] pEventSender - The event sender object
* @param[in] pEvent - The event to publish.
**/
STATIC void EventSenderProcessOpticalHctSelfTestResult(EventSender_t* pEventSender,
                                                       XEvent_t const * pEvent)
{
  const opticalHctPassFailEvent_t *pOpticalHctPassFailEvent = 
    (const opticalHctPassFailEvent_t*)pEvent;
  
  bool pass = pOpticalHctPassFailEvent->pResults->eResult;
  
  float pdVolts = pOpticalHctPassFailEvent->pResults->peakVolts;  //Calculate the signal level from
  pdVolts -= pOpticalHctPassFailEvent->pResults->darkVolts;       // peak - dark.
  
  uint32_t len = snprintf(pEventSender->eventPayloadBuffer,
                          sizeof(pEventSender->eventPayloadBuffer),
                          "LED: %u, Max Location: %u, Volts: %.3f, Result: %u",
                          pOpticalHctPassFailEvent->pResults->eLed,
                          pOpticalHctPassFailEvent->pResults->locationOfMaxima,
                          pdVolts,
                          pass);
  
  Console_PublishEventBuffer("INS",
                             pEvent->id,
                             XMsgIdLookup(pEvent->id),
                             (uint8_t *)pEventSender->eventPayloadBuffer,
                             len,
                             NULL);  // Event is infrequent. So do not need to have a callback.
}



/**
  *     @brief Processes XMSG_COMMAND_FAILED
  *     @details Prints the error code and source of the failure.
  *     @param[in] pEventSender - The event sender object
  *     @param[in] pEvent - The event to publish.
  **/
STATIC void EventSenderProcessCommandFailedEvent(EventSender_t * pEventSender,
                                                 XEvent_t const * pEvent)
{
  XMsgCmdFail_t const * pCmdFailEv = (XMsgCmdFail_t const *) pEvent;
  
   uint32_t len = snprintf(pEventSender->eventPayloadBuffer,
                          sizeof(pEventSender->eventPayloadBuffer),
                          "SOURCE: %s ERROR_CODE = %s",
                          XActiveName((XActive_t const *)pEvent->sender),
                          ErrorLookup(pCmdFailEv->eError));
  
  Console_PublishEventBuffer("INS",
                             pEvent->id,
                             XMsgIdLookup(pEvent->id),
                             (uint8_t *)pEventSender->eventPayloadBuffer,
                             len,
                             NULL);  // Event is infrequent. So do not need to have a callback.
}




/**
* @}
*/
/********************************** End Of File ******************************/
