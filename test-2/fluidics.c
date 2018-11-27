/**
******************************************************************************
* @file         fluidics.c
* @author       TMW
* @brief        Control of fluid position in a single strip channel.
* @details      Uses the electrochemistry system to determine the position of
*               bladder/fluid within the fluid eChannel.
* @note         If the electrochemistry system cannot be placed into
*               FLUID_DETECT_MODE then any movement attempts will be aborted.
******************************************************************************
*/

#include "fluidics.h"

/**
* @addtogroup Fluidics
*  @{
*/

STATIC XState FluidicState_Init(Fluidic_t* me, XEvent_t* pEv);
STATIC XState FluidicState_Idle(Fluidic_t* me, XEvent_t* pEv);
STATIC XState FluidicState_CheckForStrip(Fluidic_t* me, XEvent_t* pEv);
STATIC XState FluidicState_MoveContact(Fluidic_t* me, XEvent_t* pEv);
STATIC XState FluidicState_MoveOther(Fluidic_t* me, XEvent_t* pEv);
STATIC XState FluidicState_WaitForPiezoStop(Fluidic_t* me, XEvent_t* pEv);
STATIC XState FluidicState_MixContactControlled(Fluidic_t* me, XEvent_t* pEv);
STATIC XState FluidicState_MixPiezoControlled(Fluidic_t* me, XEvent_t* pEv);
STATIC XState FluidicState_MonitorFluidBreach(Fluidic_t* me, XEvent_t* pEv);
STATIC XState FluidicState_Err(Fluidic_t* me, XEvent_t* pEv);
STATIC XState Fluidic_defaultEvents(Fluidic_t* me, XEvent_t* pEv);
STATIC XState FluidicState_LiftUpBladder(Fluidic_t* me, XEvent_t* pEv);
STATIC XState FluidicState_WaitForContact(Fluidic_t* me, XEvent_t* pEv);


STATIC eErrorCode OnFluidMoveContact_Entry(Fluidic_t* me);
STATIC eErrorCode OnFluidMoveOther_Entry(Fluidic_t* me);
STATIC eErrorCode OnLiftUpBladders_Entry(Fluidic_t* me);
STATIC eErrorCode FluidicMixContactControlled_OnEntry(Fluidic_t* me);
STATIC eErrorCode FluidicMixPiezoControlled_OnEntry(Fluidic_t* me);
STATIC XState FluidicMoveContact_OnTick (Fluidic_t* me);
STATIC XState FluidicMixContactControlled_OnTick(Fluidic_t* me);
STATIC eErrorCode FluidOnPiezoMoveComplete(Fluidic_t* me, XEvent_t* pEv);
STATIC eErrorCode FluidOnPiezoStop(Fluidic_t* me, XEvent_t* pEv);
STATIC eErrorCode FluidOnEchemStatusChange(Fluidic_t* me, XEvent_t* pEv);
STATIC eErrorCode Fluidic_OnIdleEntry(Fluidic_t* me);
STATIC XState FluidMixMovementContinue(Fluidic_t* me);
STATIC XState FluidicState_CheckForStrip_OnTick(Fluidic_t *me);
STATIC XState FluidicStateWaitForPiezoStop_OnPiezoStop(Fluidic_t *me, 
                                                       XEvent_t* pEv);
STATIC XState FluidicState_MixWaitContinue(Fluidic_t *me, 
                                           XEvent_t* pEv);

STATIC void Fluidic_OnErrorStateEntry(Fluidic_t *me);

STATIC bool bladderControlCheckMoveValid(
                                         Fluidic_t *me,
                                         eFluidicPositions_t eTargetPos);


STATIC bool bladderControlCheckValidPosChange(
                                              eFluidicPositions_t eCurrentPos ,
                                              eFluidicPositions_t eTargetPos);

STATIC bool isFrequencyOk(Fluidic_t* me, 
                          eFluidicPositions_t eTargetPos,
                          float mixFrequency_Hz);
STATIC bool isMixPositionOk(Fluidic_t* me, eFluidicPositions_t eTargetPos);
STATIC bool isMixTimeoutOk(Fluidic_t* me, uint32_t mixTimeout_ms);


STATIC XState OnMsgBladderControlMoveToPos(Fluidic_t  *me, 
                                           const XEvent_t *pEv);

STATIC XState OnMsgBladderControlMix(Fluidic_t  *me, 
                                     const XEvent_t *pEv);
STATIC eErrorCode FluidicBeginPiezoMoveToTarget(Fluidic_t* me);

STATIC eErrorCode FluidicBeginPiezoMoveToLift(Fluidic_t* me);


STATIC void OnMsgUpdateParams(Fluidic_t *me, XEvent_t* pEv);

STATIC XState FluidicsErrorSet(Fluidic_t *me, eErrorCode err, XState retCode);

STATIC eErrorCode FluidCheckMoveParams(Fluidic_t *me,
                                       eFluidicPositions_t eTarget,
                                       float rampSpeedVoltsPerSec,
                                       uint32_t timeout_ms,
                                       eFluidOvershootCompensation_t   eOvershootCompMode,
                                       float compensationProportion);

STATIC eErrorCode FluidCheckMixParams(Fluidic_t *me,
                                      eFluidicPositions_t eTarget,
                                      float mixFrequency_Hz,
                                      uint32_t mixTimeout_ms,
                                      uint32_t numCycles,
                                      eFluidMixingType_t  eMixType,
                                      float openLoopCompensationFactor,
                                      float mixDownstrokeProportion);

STATIC eEcFluidDetectPosition_t fluidGetEchemRequirement(Fluidic_t* me,
                                                         bool isMoving);


STATIC eErrorCode FluidicStopMove(Fluidic_t* me);
STATIC eErrorCode FluidicHomeMoveBegin(Fluidic_t* me);


STATIC void FluidicOnMoveCompleteMsg(Fluidic_t* me);
STATIC void FluidicOnMixComplete(Fluidic_t* me);
STATIC void FluidicOnMoveFailMsg(Fluidic_t *me, eFluidicPositions_t pos, eErrorCode eError);
STATIC void FluidicSetCurrentAndTargetPositions(Fluidic_t* me, 
                                                eFluidicPositions_t eCurrent,
                                                eFluidicPositions_t eTarget);
STATIC void AdjustHysterisisVoltage(Fluidic_t* me, 
                                    eFluidicHysterisisChangeType_t multiplierType);

STATIC void fluidicMixingCalculaterampSpeedVoltsPerSec(Fluidic_t* me,
                                                       float startVolts,
                                                       float endVolts);
STATIC XState FluidMixOnStageComplete(Fluidic_t *me);
STATIC float FluidicMixCalculateDownStrokeMixProportion(Fluidic_t *me);
STATIC bool  FluidicStateCanAcceptCommand(Fluidic_t * me);
STATIC eErrorCode FluidicMonitorBladderDetection(Fluidic_t * me, eXEventId eventId);
STATIC XState OnMsgLiftUpBladders(Fluidic_t  *me, const XEvent_t *pEv);

STATIC eElectrochemicalChannelPos ConvertFluidPosToEchemPos(eFluidicPositions_t eFluidPos);
STATIC XState OnMsgWaitForFluidAtContact(Fluidic_t * me, const XEvent_t *pEv);

/**
* @defgroup fAPI Fluidic APIs
* @brief API calls for Fluidic objects.
* @details API calls post messages to the specified Fluidic object in order to
*          generate commands.
* @{
**/


/**
* @brief  Initialises the Fluidic channel.  
* @param me The fluidic object to initialise 
* @param pInitParams The initialisation parameters for the fluidic channel.
* @param XFwk Pointer to the parent framework.
* @details Fluidic channels are initialised by:
*           -   Enabling the four output events.
*           -   Storing the initial parameters.
*           -   Configuring the 20ms system timer.
*           -   Connecting the object to the XActive framework, and starting.
**/   
void FluidicInit(Fluidic_t* me, 
                 const FluidicInitParams_t* pInitParams,
                 XActiveFramework_t*     pXActiveFramework)
{
  ASSERT_NOT_NULL(me);
  ASSERT_NOT_NULL(pInitParams);  
  ASSERT_NOT_NULL(pInitParams->pPiezo);
  ASSERT_NOT_NULL(pInitParams->pEchem);
 // ASSERT_NOT_NULL(XFwk);
  
  (void)memset(me, 0, sizeof(Fluidic_t));
  
//  me->XFwk = XFwk;
  
  me->pPiezo  = pInitParams->pPiezo;
  me->pEchem  = pInitParams->pEchem;
  me->pParams = pInitParams ->pParams;
  
  me->super.enableDebugging = false; 
  
  me->publishCompletionEvent = true;      ///< General case is to publish completion.
  
  XActive_ctor(&me->super, (XStateHandler) &FluidicState_Init);
  
  XTimerCreate(&(me->timer), 
               &(me->super), 
               X_EV_TIMER, 
               FLUIDIC_TIMER_COUNT_MS,
               false);
  
  //Construct the published events.
  X_EV_INIT((&me->moveSuccessMsg), (XMSG_FMOVE_CMPLT), me);
  X_EV_INIT((&me->moveFailMsg), XMSG_FLUID_CHANNEL_MOVE_FAIL, me);
  X_EV_INIT(&me->mixCmpltMsg, XMSG_FLUID_MIX_COMPLETE, me);
  X_EV_INIT(&me->moveFailMsg, XMSG_FLUID_ERR, me);
  X_EV_INIT(&me->fcStartBladderDetectMsg, XMSG_FLUID_START_BLDDR_DETECT, me);
  X_EV_INIT(&me->fcStopBladderDetectMsg, XMSG_FLUID_STOP_BLDDR_DETECT, me);
  
  //Constructors for events to self.
  X_EV_INIT(&(me->errClearMsg), XMSG_FLUID_ERR_CLEAR, me);
  X_EV_INIT(&(me->stopMsg), XMSG_FLUID_CHANNEL_CANCEL, me);
  X_EV_INIT(&(me->paramsMsg), XMSG_FLUID_CHANNEL_NEW_PARAMS, me);
  X_EV_INIT(&(me->moveMsg), XMSG_FLUID_CHANNEL_MOVE_TO, me);
  X_EV_INIT(&(me->mixMsg), XMSG_FLUID_MIX, me);
  X_EV_INIT(&(me->cmdFail), XMSG_COMMAND_FAILED, me);
  X_EV_INIT(&(me->liftUpBlddrMsg), XMSG_FLUID_LIFT_UP_BLADDER, me);
  X_EV_INIT(&(me->waitForFluidContactMsg), XMSG_FLUID_WAIT_FOR_CONTACT, me);
  
  X_EV_INIT(&(me->monitorPositionMsg), XMSG_FLUID_ENABLE_BREACH_DETECT, me);
  
  X_EV_INIT(&(me->stageCompelteMsg), XMSG_FLUID_MIX_STAGE_COMPLETE, me);
  X_EV_INIT(&(me->breachDetectedMsg),
            XMSG_BREACH_DETECTED,
            me);
  
  me->stageCompelteMsg.eChannel = me->pParams->eChannel;
  
  XActiveStart(pXActiveFramework,
               (XActive_t*)&(me->super),
               pInitParams->name,
               pInitParams->prio,
               me->evQueueBytes,
               sizeof(me->evQueueBytes),
               NULL);
  
  X_SUBSCRIBE_TO_FLUIDIC_EVENTS(me);
}



/**
*     @brief API call to clear the current error of the fluidic instance.
*     @param[in]      me - The fluid controller instance.
*     @retval OK_COMMAND_ACCEPTED
**/
eErrorCode FluidicErrorClear(Fluidic_t * me)
{
  XActivePost(&me->super, (XEvent_t const*) &(me->errClearMsg));
  return OK_COMMAND_ACCEPTED;
}


/**
*     @brief API call to cancel any current actions by the fluidic controller.
*     @note If this API is used whilst mixing the fluid front will be returned to
*           the last resting position.
*           If this is not desired use the Global Halt API.
*     @param[in]      me - The fluid controller instance.
*     @retval OK_COMMAND_ACCEPTED
**/
eErrorCode FluidicStop(Fluidic_t *me)
{
  XActivePost(&me->super, (XEvent_t const*) &(me->stopMsg)); 
  return OK_COMMAND_ACCEPTED;
}



/**
*     @brief API call to alter the Fluidic controller instance's parameters.
*     @param[in]      me - The fluid controller instance.
*     @param[in]      pParams - The new parameters.
*     @retval OK_COMMAND_ACCEPTED The parameters are okay, and can be used.
*     @retval ERROR_FLUID_INVALID_PARAMS The requested parameters cannot be used.
**/
eErrorCode FluidicParamsSet(Fluidic_t* me, FluidicParams_t *pParams)
{ 
  eErrorCode error;
  
  ASSERT_NOT_NULL(me);
  ASSERT_NOT_NULL(pParams);
  
  me->paramsMsg.flAVal = pParams->positionLimits[BC_POS_FLUID_A].targetVolts;
  me->paramsMsg.flBVal = pParams->positionLimits[BC_POS_FLUID_B].targetVolts;
  me->paramsMsg.flCVal = pParams->positionLimits[BC_POS_FLUID_C].targetVolts;
  
  // Check that targets for Fluid A < Fluid B (as contact A cannot be further up
  // the channel than contact B), and that B < C (same reason)
  if((me->paramsMsg.flAVal < me->paramsMsg.flBVal) &&
     (me->paramsMsg.flBVal < me->paramsMsg.flCVal))
  {
    XActivePost(&me->super, (XEvent_t const*) &(me->paramsMsg)); 
    error = OK_COMMAND_ACCEPTED;
  }
  else
  {
    error = ERROR_FLUID_INVALID_PARAMS;
  }
  
  return error;
}




/**
*     @brief API to move the fluid control instance to a given position.
*     @note Homing move is performed instantly by the Piezo object, therefore
*           this must not be used if there is fluid in the consumable.
*     @param[in]      me - The fluidic controller instance.
*     @param[in]      eTarget - The target position.
*     @param[in]      erampSpeedVoltsPerSec - The step speed to move at.
*     @retval OK_COMMAND_ACCEPTED - The mixing can be performed.
*     @retval ERROR_FLUID_CHANNEL_INVALID_MOVE - The mixing parameters are invalid, 
*             or the move cannot be completed
**/
eErrorCode FluidicMove(Fluidic_t* me, 
                       eFluidicPositions_t eTarget, 
                       float rampSpeedVoltsPerSec,
                       uint32_t timeout_ms,
                       eFluidOvershootCompensation_t   eOvershootComp,
                       float overshootCompProportion)
{  
  eErrorCode error = ERROR_NULL_PTR;
  
  // Cannot accept a movement command when the parameters are invalid, or
  // if not in idle state.
  
  bool isReady;
  
  if (NULL != me)
  {
    // Can always accept a homing move.
    // don't need to check parameters.
    if(eTarget == BC_POS_HOME)
    {
      me->moveMsg.eTargetPos = BC_POS_HOME;
      
      XActivePost(&me->super, (XEvent_t const*) &(me->moveMsg)); 
      error = OK_COMMAND_ACCEPTED;
    }
    else
    {
      // Otherwise, do a parameter and status check before accepting command.
      error = FluidCheckMoveParams(me,
                                   eTarget,
                                   rampSpeedVoltsPerSec,
                                   timeout_ms,
                                   eOvershootComp,
                                   overshootCompProportion);
      
      isReady = FluidicStateCanAcceptCommand(me);
        
      // Have to check if move is permitted first.  
      if (isReady)
      {
        //  error = OK_STATUS, the only thing which might hold us back is
        // being busy!
        if (OK_STATUS == error)
        {
          me->moveMsg.eTargetPos = eTarget;
          me->moveMsg.rampSpeedVoltsPerSec = rampSpeedVoltsPerSec;
          me->moveMsg.timeout_ms = timeout_ms;
          me->moveMsg.eOvershootComp = eOvershootComp;
          me->moveMsg.overshootCompProportion = overshootCompProportion;
          
          
          XActivePost(&me->super, (XEvent_t const*) &(me->moveMsg)); 
          
          error = OK_COMMAND_ACCEPTED;    // Asynch command. Return command_accepted.
        }
      }
      else
      {
        error = ERROR_OBJECT_NOT_READY; // Busy. return appropriate code.
      }
    }
  }
  
  return error;
}


/**
  * @brief API to lift bladders until bladder down contacts no longer made.
  * @param[in] me - The fluid controller
  * @param[in] eTarget - Bladder Down
  * @param[in] rampSpeedVoltsPerSec - The ramp rate of the movement (velocity)
  * @param[in] timeout_ms - The move timeout
  * @param[in] eOvershootComp - Overshoot compensation mode
  * @param[in] overshootCompProportion - % of Piezo voltage difference between start and end points.
  * @returns OK_COMMAND_ACCEPTED if the command is valid and can be performed.
  **/
eErrorCode FluidicLiftUpBladder(Fluidic_t* me,
                                float rampSpeedVoltsPerSec,
                                uint32_t timeout_ms)
{
  eErrorCode error;

  ASSERT_NOT_NULL(me);
  ASSERT_NOT_NULL(me->pParams);

  me->eTargetPos = BC_POS_HOME;           // Has to be home to check against UP, rather than down.
  me->liftUpBlddrMsg.rampSpeedVoltsPerSec = rampSpeedVoltsPerSec;
  me->liftUpBlddrMsg.timeout_ms = timeout_ms;

  // Cannot accept a movement command when the parameters are invalid, or
  // if not in idle state.
  
  error = FluidCheckMoveParams(me,
                               BC_POS_DOWN,
                               rampSpeedVoltsPerSec,
                               timeout_ms,
                               FLUID_OVERSHOOT_COMP_NONE,
                               0.f);
  
  bool isReady = FluidicStateCanAcceptCommand(me);
  
  if((OK_STATUS == error))
  {
    if (isReady)
    {
      XActivePost(&me->super, (XEvent_t const*) &(me->liftUpBlddrMsg));
      error = OK_COMMAND_ACCEPTED;
    }
    else
    {
      error = ERROR_OBJECT_NOT_READY;
    }
  }

  return error;
}




/**
*     @brief API to mix the fluid control instance between the current electrode, 
*            and target, positions. 
*     @param[in]      me - The fluidic controller instance.
*     @param[in]      eTarget - The target position.
*     @param[in]      mixFreq - The frequency of the fluid front movement.
*     @param[in]      mixTimeout - The mixing period in ms.
*     @retval OK_COMMAND_ACCEPTED - The mixing can be performed (command has been received)
*     @retval ERROR_FLUID_CHANNEL_INVALID_MOVE - The mixing parameters are invalid, 
*             or the move cannot be completed
**/
eErrorCode FluidicMix(Fluidic_t* me, 
                      eFluidicPositions_t eTarget, 
                      float mixFreq, 
                      uint32_t mixTimeout, 
                      uint32_t cycles,
                      eFluidMixingType_t  eMixType,
                      float openLoopCompensationFactor,
                      float mixDownstrokeProportion)
{
  eErrorCode error = ERROR_NULL_PTR;
  
  if (NULL != me)
  {
    error =  FluidCheckMixParams(me,
                                 eTarget,
                                 mixFreq,
                                 mixTimeout,
                                 cycles,
                                 eMixType,
                                 openLoopCompensationFactor,
                                 mixDownstrokeProportion);
      
    if (FluidicStateCanAcceptCommand(me))
    {
      if(OK_STATUS == error)
      {
        
        me->mixMsg.eTargetPos                 = eTarget;
        me->mixMsg.mixFrequency               = mixFreq;
        me->mixMsg.mixTime                    = mixTimeout;
        me->mixMsg.mixCycles                  = cycles;
        me->mixMsg.eMixType                   = eMixType;
        me->mixMsg.openLoopCompensationFactor = openLoopCompensationFactor;
        me->mixMsg.mixDownstrokeProportion    = mixDownstrokeProportion;
        XActivePost(&me->super, (XEvent_t const*) &(me->mixMsg)); 
        error = OK_COMMAND_ACCEPTED;
      }
      
    }
    else
    {
      error = ERROR_OBJECT_NOT_READY;
    }
  }
  
  
  
  return error;
}



/**
  * @brief Fluidic API to enable breach monitoring
  * @details Posts message to the fluid controller.
  * @param[in] me - The fluid controller
  * @param[in] enable - Boolean flag to enable contact monitoring after the movement compeltes.
  **/
eErrorCode FluidicEnableBreachMonitoring(Fluidic_t* me,
                                   bool enable)
{
  me->monitorPositionMsg.monitorFluidPosition = enable;
  
  XActivePost(&me->super, (XEvent_t const*) &(me->monitorPositionMsg)); 
  
  return OK_STATUS;
}

                       
/**
  *     @brief Object API to instruct fluid controller to wait for fluid detection at
  *            the specified location.
  *     @param[in] me - The fluid controller
  *     @param[in] eTarget - The position we expect fluid to be detected at
  *     @param[in] timeoutMs - The timeout limit, in ms.
  *     @returns OK_COMMAND_ACCEPTED if the parameters are OK.
  **/
eErrorCode FluidicWaitForFluidAtContact(Fluidic_t * me,
                                        eFluidicPositions_t eTarget,
                                        uint32_t timeoutMs)
{
  ASSERT_NOT_NULL(me);
  
  eErrorCode eError = ERROR_NULL_PTR;
  bool canAcceptCommand = FluidicStateCanAcceptCommand(me);
  bool moveValid = bladderControlCheckMoveValid(me, eTarget);
  
  
  // Default error is NULL_PTR, so already handled.
  if (NULL != me)
  {
    
    // Can't wait for contact at bladder down, othereise use moveValid state
    // to check against "bad-args"
    if ((eTarget >= BC_POS_FLUID_A) && (moveValid))
    {
      
      // Before sending the command, check that we can actually accept it!
      if (canAcceptCommand)
      {
        me->waitForFluidContactMsg.eTargetPos = eTarget;
        me->waitForFluidContactMsg.timeoutMs = timeoutMs;
        
        X_POST(me, me->waitForFluidContactMsg);
        
        eError = OK_COMMAND_ACCEPTED;
      }
      else
      {
        eError = ERROR_OBJECT_NOT_READY;
      }
    }
    else
    {
      eError = ERROR_BAD_ARGS;
    }
  }
  
  return eError;
}

/** @} **/


/**
*     @defgroup fluidicStateMachine   Fluidic Object States
*     @brief Fluidic object state handlers.
*     @{
**/

/**
* @brief  Initialisation state of the Fluidic channel
* @param me The fluidic object. 
* @param pEv The entry event of the state.
* @details Reset the Fluidic channel data, enable the clock, disable the 
*          electrochemical channel and home the Piezo bender.
* @returns Transition to Idle state.
**/  
STATIC XState FluidicState_Init(Fluidic_t* me, XEvent_t* pEv)
{
  ASSERT_NOT_NULL(me);
  XState retCode = X_TRAN(me, &FluidicState_Idle);
  
  // On initialisation we don't know where the benders are, or
  // if a strip is inserted at all.
  FluidicSetCurrentAndTargetPositions(me, BC_POS_UNKNOWN, BC_NONE);
  
  me->status.eFluidFrontPosition    = FD_DATA_INVALID;
  
  return retCode;
}


/**
* @brief  Idle state of the Fluidic channel
* @param me The fluidic object. 
* @param pEv Input events to be handled.
* @details Responds to the following events:

Event                     | Actions
------------------------- | ---------------------------------
X_EV_ENTRY                | Stop the Piezo bender movement (if it is moving), disable the electrochemical channel, erase the fluidic data, and set the target posoition to none.
------------------------- |----------------------------------
XMSG_FLUID_CHANNEL_MOVE_TO| Begins a fluid channel move.
------------------------- |----------------------------------
XMSG_FLUID_MIX            | Starts fluid channel mixing.
------------------------- |----------------------------------
default                   | Calls the default event handler.  

* @returns The state response.
**/  
STATIC XState FluidicState_Idle(Fluidic_t* me, XEvent_t* pEv)
{
  XState retCode = X_RET_HANDLED;
  eErrorCode error = OK_STATUS;
  
  eXEventId eventId = pEv->id;
  
  switch(eventId)
  {
  case X_EV_ENTRY:
    me->timeoutTimer = 0u;
    error = Fluidic_OnIdleEntry(me);
    break;
    
  case XMSG_FLUID_CHANNEL_MOVE_TO:
    retCode = OnMsgBladderControlMoveToPos(me, pEv);
    break;
    
  case XMSG_FLUID_LIFT_UP_BLADDER:
    retCode = OnMsgLiftUpBladders(me, pEv);
    break;
    
  case XMSG_FLUID_MIX:
    retCode = OnMsgBladderControlMix(me, pEv);
    break;
    
  case XMSG_FLUID_WAIT_FOR_CONTACT:
    retCode = OnMsgWaitForFluidAtContact(me, pEv);
    break;
    
  default:
    retCode = Fluidic_defaultEvents(me, pEv);
    break;
  }
  
  retCode = FluidicsErrorSet(me, error, retCode);
  return retCode;
}


/**
  * @brief Actions to perform on entry to idle state.
  * @param[in] me - The fluidic controller.
  * @details -> Ensure that our electrochemical channel is disabled.
             -> Disable the timer.
             -> Stop any Piezo movements (in our Piezo)
             -> Set target position to NONE.
             -> Set fluid detect data to invalid.
             -> Set to not reverse movement.
  * @returns Error codes from disabling echem and stopping piezo movement.
  **/
STATIC eErrorCode Fluidic_OnIdleEntry(Fluidic_t* me)
{
  eErrorCode error = ecDisable(me->pEchem, me->pParams->eChannel);
  
  XTimerStop(&(me->timer)); //Don't need timer in idle
  
  //@todo Check potential errors in ecDisable.
  if(OK_STATUS == error)
  {
    error = FluidicStopMove(me);  //Stop where we are. Just in case the piezo was still moving.
  }
  
  // If an error has occurred then these values are reset in the error state.
  me->status.eFluidFrontPosition = FD_DATA_INVALID;
  me->status.eMoveDirection = FLUID_MOVE_FWD;
  FluidicSetCurrentAndTargetPositions(me, me->eLastKnownPos, BC_NONE);
  
  return error;
}




/**
* @brief  Move state of the Fluidic channel, when moving to a fluid contact (rather than bladder down or home)
* @param me The fluidic object. 
* @param pEv Input events to be handled.
* @details Responds to the following events:

Event                     | Actions
------------------------- | ---------------------------------
X_EV_ENTRY                | Initialises the fluid channel movement.
------------------------- |----------------------------------
X_EV_TIMER                | Processing for the fluidic channel, on the 20ms timer.
------------------------- |----------------------------------
XMSG_PIEZO_MOVE_COMPLTE   | Publish movement failure event, go to idle state.
------------------------- |----------------------------------
default                   | Calls the default event handler.  

* @returns The state response.
**/
STATIC XState FluidicState_MoveContact(Fluidic_t* me, XEvent_t* pEv)
{
  XState retCode = X_RET_HANDLED;
  eErrorCode error = OK_STATUS;
  eXEventId eventId = pEv->id;
  
  switch(eventId)
  {
  case X_EV_ENTRY:
    me->timeoutTimer = 0u;
    error = OnFluidMoveContact_Entry(me);
    break;
    
   // Check the  echem status, successful move will be detected in this
   // time based check.
  case X_EV_TIMER:
    retCode = FluidicMoveContact_OnTick (me);
    break;
    
  case XMSG_PIEZO_MOVE_COMPLTE:
    // A PIEZO complete maybe received, don't want to handle
    // it as a default case (below). Want to allow the timer
    // to complete before declaring a fail
    retCode = X_RET_HANDLED;
    break;
    
  default:
    retCode = Fluidic_defaultEvents(me, pEv);
    break;
  }
  
  retCode = FluidicsErrorSet(me, error, retCode);
  return retCode;
}




/**
* @brief  Move state of the Fluidic channel, when moving to bladder down or home
* @param me The fluidic object. 
* @param pEv Input events to be handled.
* @details Responds to the following events:

Event                     | Actions
------------------------- |----------------------------------
X_EV_ENTRY                | Initialises the fluid channel movement.
------------------------- |----------------------------------
X_EV_TIMER                | Increment timeout timer. If it has exceeded the timeout value publish failure event and go to idle state.
------------------------- |----------------------------------
XMSG_EC_xx_BLDR_xxxx      | Bladder detection feedback msgs
------------------------- |----------------------------------
XMSG_PIEZO_MOVE_COMPLTE   | Publish movement complete event, go to idle state.
------------------------- |----------------------------------
default                   | Calls the default event handler.  

* @returns The state response.
**/
STATIC XState FluidicState_MoveOther(Fluidic_t* me, XEvent_t* pEv)
{
  XState retCode = X_RET_HANDLED;
  eErrorCode error = OK_STATUS;
  
  eXEventId eventId = pEv->id;
  
  PiezoMoveCompltEv_t* pMoveCmplt = (PiezoMoveCompltEv_t*) pEv;
  
  switch (eventId)
  {
  case X_EV_ENTRY:
	  me->chTargetPosReached = false;
    error = OnFluidMoveOther_Entry(me);
    break;
    
  case X_EV_TIMER:
    me->timeoutTimer += FLUIDIC_TIMER_COUNT_MS;
    
    if (me->timeoutTimer >= me->pParams->timeout_ms)
    {
      /* Let the script runner know that this is down to timeout
       * (instrument inactivity). */
      FluidicOnMoveFailMsg(me, me->eTargetPos, ERROR_DXRUNNER_FMOV_TIMEOUT);
      
      X_PUBLISH(X_FRAMEWORK_OF(me), me->fcStopBladderDetectMsg);

      me->eLastKnownPos = BC_POS_UNKNOWN;
      retCode = X_TRAN(me, &FluidicState_Idle);
    }
    /* Give enough time for states of channels to settle
     * before we kick off EC (bladder detection state) and make sure target
     * position is down/pressed. */
    if ((20u == me->timeoutTimer) && (BC_POS_DOWN == me->eTargetPos))
    {
      X_PUBLISH(X_FRAMEWORK_OF(me), me->fcStartBladderDetectMsg);
    }
    break;

  case XMSG_EC_A1_BLDR_UP:
  case XMSG_EC_B2_BLDR_UP:
  case XMSG_EC_A3_BLDR_UP:
  case XMSG_EC_B4_BLDR_UP:
  case XMSG_EC_A1_BLDR_DOWN:
  case XMSG_EC_B2_BLDR_DOWN:
  case XMSG_EC_A3_BLDR_DOWN:
  case XMSG_EC_B4_BLDR_DOWN:
	  error = FluidicMonitorBladderDetection(me, eventId);
	break;

  case XMSG_PIEZO_STOPPED:
  case XMSG_PIEZO_MOVE_COMPLTE:
    if(pMoveCmplt->chan == me->pPiezo->pParams->chan)
    {
      me->status.piezoVoltage = pMoveCmplt->piezoVoltage;  
      FluidicOnMoveCompleteMsg(me);
      
      if (BC_POS_DOWN == me->eTargetPos)
      {
        me->pParams->positionLimits[me->eTargetPos].targetVolts =
          piezoVoltageGet(me->pPiezo);
      }

      retCode = X_TRAN(me, &FluidicState_Idle);
    }
    break; 
    
  default:
    retCode = Fluidic_defaultEvents(me, pEv);
    break;
  }
  
  retCode = FluidicsErrorSet(me, error, retCode);
  return retCode;
}

/**
* @brief  Lifts up the bladders post "down/pressed" detection.
* @param me The fluidic object.
* @param pEv Input events to be handled.
* @details Responds to the following events:

Event                     | Actions
------------------------- |----------------------------------
X_EV_ENTRY                | Initialises the fluid channel movement.
------------------------- |----------------------------------
XMSG_EC_A1_BLDR_UP        |
XMSG_EC_B2_BLDR_UP        |
XMSG_EC_A3_BLDR_UP        |
XMSG_EC_B4_BLDR_UP        |
XMSG_EC_A1_BLDR_DOWN      | Events indicating bladder status
XMSG_EC_B2_BLDR_DOWN      |
XMSG_EC_A3_BLDR_DOWN      |
XMSG_EC_B4_BLDR_DOWN      |
------------------------- |----------------------------------
XMSG_PIEZO_MOVE_COMPLTE   | Publish movement complete event, go to idle state.
------------------------- |----------------------------------
default                   | Calls the default event handler.

* @returns The state response.
**/
STATIC XState FluidicState_LiftUpBladder(Fluidic_t* me, XEvent_t* pEv)
{
  XState retCode = X_RET_HANDLED;
  eErrorCode error = OK_STATUS;

  eXEventId eventId = pEv->id;

  PiezoMoveCompltEv_t* pMoveCmplt = (PiezoMoveCompltEv_t*) pEv;

  switch (eventId)
  {
  case X_EV_ENTRY:
    me->chTargetPosReached = false;
    error = OnLiftUpBladders_Entry(me);
    break;

  case X_EV_TIMER:
    me->timeoutTimer += FLUIDIC_TIMER_COUNT_MS;

    if (me->timeoutTimer >= me->pParams->timeout_ms)
    {
      FluidicOnMoveFailMsg(me, me->eTargetPos, ERROR_COMMAND_TIMEOUT);

      me->eLastKnownPos = BC_POS_UNKNOWN;
      retCode = X_TRAN(me, &FluidicState_Idle);
    }
    /* Give enough time for states of channels to settle
     * before we kick off EC (bladder detection state) and make sure target
     * position is HOME/open */
    if ((20u == me->timeoutTimer) && (BC_POS_HOME == me->eTargetPos))
    {
      X_PUBLISH(X_FRAMEWORK_OF(me), me->fcStartBladderDetectMsg);
    }
    break;

  case XMSG_EC_A1_BLDR_UP:
  case XMSG_EC_B2_BLDR_UP:
  case XMSG_EC_A3_BLDR_UP:
  case XMSG_EC_B4_BLDR_UP:
  case XMSG_EC_A1_BLDR_DOWN:
  case XMSG_EC_B2_BLDR_DOWN:
  case XMSG_EC_A3_BLDR_DOWN:
  case XMSG_EC_B4_BLDR_DOWN:
    error = FluidicMonitorBladderDetection(me, eventId);
  break;

  case XMSG_PIEZO_STOPPED:
  case XMSG_PIEZO_MOVE_COMPLTE:
    if(pMoveCmplt->chan == me->pPiezo->pParams->chan)
    {
      me->eTargetPos =  BC_POS_DOWN;        // Force the resting place to be down. 
      me->status.piezoVoltage = pMoveCmplt->piezoVoltage;
      FluidicOnMoveCompleteMsg(me);

      retCode = X_TRAN(me, &FluidicState_Idle);
    }
    break;

  default:
    retCode = Fluidic_defaultEvents(me, pEv);
    break;
  }

  retCode = FluidicsErrorSet(me, error, retCode);
  return retCode;
}



/**
  *     @brief State handler to wait for fluid detected (passively) at the target position
  *     @details Only failure is if the movement times out.
  * @param me The fluidic object.
  * @param pEv Input events to be handled.
  * @returns State response code.
  **/
STATIC XState FluidicState_WaitForContact(Fluidic_t* me, XEvent_t* pEv)
{
  XState retCode = X_RET_HANDLED;
  
  eErrorCode error = OK_STATUS;
  eElectrochemicalChannel ecChan;
  
  eEcFluidDetectPosition_t eRequirement;
  eEcFluidDetectPosition_t eFluidFrontPos;
  
  
  eXEventId eventId = pEv->id;
  
  PiezoMoveCompltEv_t* pMoveCmplt = (PiezoMoveCompltEv_t*) pEv;
  
  switch (eventId)
  {
  case X_EV_ENTRY:
     ecChan = me->pParams->eChannel;
    
    
    XTimerStart(&(me->timer));    // Need timer if moving. 
    me->timeoutTimer = 0u;      // Reset timeout timer.
    
    error = ecSetModeFillDetect(me->pEchem, ecChan, EC_CHAN_POS_A);    // We're waiting for a contact. monitor all of them!             
    break;

  case X_EV_TIMER:
    me->timeoutTimer += FLUIDIC_TIMER_COUNT_MS;
    
    eRequirement = fluidGetEchemRequirement(me, true);
    eFluidFrontPos = me->status.eFluidFrontPosition;
    
    if (eRequirement == eFluidFrontPos)
    {
      FluidicOnMoveCompleteMsg(me);
      retCode = X_TRAN(me, &FluidicState_Idle);
    }

    else if (me->timeoutTimer >= me->pParams->timeout_ms)
    {
      FluidicOnMoveFailMsg(me, me->eTargetPos, ERROR_COMMAND_TIMEOUT);

      // We're not moving, so do not reset the position as we're in the same place!
      retCode = X_TRAN(me, &FluidicState_Idle);
    }
    else
    {
      retCode = X_RET_HANDLED;
    }
    break;

  case XMSG_PIEZO_STOPPED:
  case XMSG_PIEZO_MOVE_COMPLTE:
    if(pMoveCmplt->chan == me->pPiezo->pParams->chan)
    {
      FluidicOnMoveFailMsg(me, me->eTargetPos, ERROR_FLUIDIC_UNEXPECTED_MSG_PIEZO);
      retCode = X_TRAN(me, &FluidicState_Idle);
    }
    break;

  default:
    retCode = Fluidic_defaultEvents(me, pEv);
    break;
  }

  retCode = FluidicsErrorSet(me, error, retCode);
  
  return retCode;
}




/**
  * @brief Checks the strip status before allowing a move to continue.
  * @details If no strip is detected then the object will go back to idle state.
  * @param[in] me - The fluidic object
  * @param[in] pEv - The event.
  * @returns The state response code.
  **/
STATIC XState FluidicState_CheckForStrip(Fluidic_t* me, XEvent_t* pEv)
{
  XState retCode = X_RET_HANDLED;
  eErrorCode error = OK_STATUS;
  eXEventId eventId = pEv->id;
  
  switch(eventId)
  {
  case X_EV_ENTRY:
    me->timeoutTimer = 0u;
    XTimerStart(&(me->timer));
    error = ecSetModeFillDetect(me->pEchem, me->pParams->eChannel, EC_CHAN_POS_A); 
    break;
    
  case X_EV_TIMER:
    retCode = FluidicState_CheckForStrip_OnTick(me);
    break;
    
  default:
    retCode = Fluidic_defaultEvents(me, pEv);
    break;
    }
  
  retCode = FluidicsErrorSet(me, error, retCode);
  return retCode;
}



/**
  * @brief Helper for the check strip state.
  * @details Checks to make sure that
  *   - A strip is inserted before starting a move
  *   - If moving up the fluid channel, that there is fluid applied to the strip.
  *   Movements to Bladder down position have a few additional checks.
  **/
STATIC XState FluidicState_CheckForStrip_OnTick(Fluidic_t *me)
{
  XState retCode = X_RET_HANDLED;
  eErrorCode error = OK_STATUS;
  
  // Increment timeout.
  me->timeoutTimer += FLUIDIC_TIMER_COUNT_MS;
  
  me->status.eFluidFrontPosition = ecGetFluidPosition(me->pEchem,
                                                      me->pParams->eChannel);
  
  // Wait at least ECHEM_UPDATE_PERIOD_MS before checking the status
  // (This allows the echem long enough to sample the pins.)
  // Check to see if data is not invalid (Echem may not have been serviced yet)
  if((me->timeoutTimer > ECHEM_UPDATE_PERIOD_MS) && (me->status.eFluidFrontPosition != FD_DATA_INVALID))
  {
    // Check the status of the fluid front. Do this on a timer update so that even if the status hasn't
    // been published in event we are still getting an update.
    me->status.eFluidFrontPosition = ecGetFluidPosition(me->pEchem, me->pParams->eChannel);
    
    // If moving to down position then doesn't need to be fluid. But does need a strip.
    if((me->eTargetPos == BC_POS_DOWN) && (me->status.eFluidFrontPosition >= NO_FLUID_DETECTED))
    {
      retCode = X_TRAN(me, &FluidicState_MoveOther);
    }
    // For all moves inside the channel, there must be fluid applied.
    else if((me->status.eFluidFrontPosition >= FLUID_DETECTED) &&  me->eTargetPos > BC_POS_DOWN)
    {
      retCode = X_TRAN(me, &FluidicState_MoveContact);
      me->timeoutTimer = 0u;  // Reset the timeout timer
    }
    // Not a critical error if no strip (someone may have requested the movement
    // before putting strip in instrument)
    else if(me->status.eFluidFrontPosition == NO_STRIP_DETECTED)
    {
      error = ERROR_FLUID_NO_STRIP;
      retCode = X_TRAN(me, &FluidicState_Idle);
    }
    // Otherwise, we were moving further up strip but no fluid has been detected.
    // Movement therefore fails.
    else
    {
      error = ERROR_FLUID_CHANNEL_INVALID_MOVE;
      retCode = X_TRAN(me, &FluidicState_Idle);
    }
  }
  
  
  // We should have received a response from the echem before timeout,
  // if no response is received there is an error.
  if(me->timeoutTimer >= me->pParams->timeout_ms)
  {
    error = ERROR_COMMAND_TIMEOUT;
    retCode = X_TRAN(me, &FluidicState_Idle);
  }
  
  
  if(OK_STATUS != error)
  {
    FluidicOnMoveFailMsg(me, me->eTargetPos, error);
  }
  
  return FluidicsErrorSet(me, error, retCode);
}




/**
* @brief  Move state of the Fluidic channel
* @param me The fluidic object. 
* @param pEv Input events to be handled.
* @details Responds to the following events:

Event                     | Actions
------------------------- |----------------------------------
XMSG_PIEZO_STOPPED        | Record the piezo stopped status, and checks to see whether the fluidic object should transition back to the move state, or idle state. 
------------------------- |----------------------------------
default                   | Calls the default event handler.  

* @returns The state response.
**/
STATIC XState FluidicState_WaitForPiezoStop(Fluidic_t* me, XEvent_t* pEv)
{
  XState retCode;
  eErrorCode error = OK_STATUS;
  
  eXEventId eventId = pEv->id;
  PiezoMoveCompltEv_t* pMoveCmplt = (PiezoMoveCompltEv_t*) pEv;
  PiezoStoppedEv_t * pStoppedEv = (PiezoStoppedEv_t*) pEv;
  
  switch(eventId)
  {    
  case XMSG_PIEZO_STOPPED:
    // Only process Piezo stopped when it's our specified channel.
    if(pStoppedEv->chan == me->pPiezo->pParams->chan)
    {
      retCode =FluidicStateWaitForPiezoStop_OnPiezoStop(me, pEv);
    }
    else
    {
      retCode = X_RET_HANDLED;
    }
    break;
    
  // Only get to move complete with overshoot compensation.
  case XMSG_PIEZO_MOVE_COMPLTE:
    if(pMoveCmplt->chan == me->pPiezo->pParams->chan)
    {
      me->status.piezoVoltage = pMoveCmplt->piezoVoltage;  
      FluidicOnMoveCompleteMsg(me);
      
      // Select state dependant on whether need to detect for breach or not.
      // We can detect breach here as we're holding fluid at a given contact.
      if(me->pParams->monitorBreachAfterMove)
      {
        retCode = X_TRAN(me, FluidicState_MonitorFluidBreach);
      }
      else
      {
        retCode = X_TRAN(me, FluidicState_Idle);
      }
    }
    else
    {
      retCode = X_RET_HANDLED;
    }
    break;
    
  default:
    retCode = Fluidic_defaultEvents(me, pEv);
    break;
  }
  
  retCode = FluidicsErrorSet(me, error, retCode);
  return retCode;
}




/**
  * @brief Helper for XMSG_PIEZO_STOPPED when in the wait for piezo stop
  *        state.
  * @param[in] me - The fluidic instance.
  * @param[in] pEv - The XMSG_PIEZO_STOPPED event.
  * @returns State handler code.
  **/
STATIC XState FluidicStateWaitForPiezoStop_OnPiezoStop(Fluidic_t *me, 
                                                       XEvent_t* pEv)
{
  XState retCode = X_RET_HANDLED;
  uint32_t lowerPos;
  
  bool movementIsComplete = false;
  
  float overshootPiezoVoltage;
  //Normal message processing.
  eErrorCode error = FluidOnPiezoStop(me, pEv);
  peizoMoveParams_t overshootParams;
  
  /// Check the pushback mechanism; this is required when moving forwards.
  
  eFluidOvershootCompensation_t eCompType = me->pParams->eOvershootCompensationType;
  
  // We kknow that we're at the target position. However, we might be performing
  // some additional movements in order to avoid overshoot.
  me->eLastKnownPos = me->eTargetPos;
  
  /// Store the updated piezo voltage.
  me->pParams->positionLimits[me->eTargetPos].targetVolts = 
    piezoVoltageGet(me->pPiezo);
  
  if(me->status.eMoveDirection == FLUID_MOVE_FWD)
  {   
    // Compensation only applied if moving forwards initially.
    if(FLUID_OVERSHOOT_COMP_NONE == eCompType)
    {
      // No compensation, therefore publish completion.
      FluidicOnMoveCompleteMsg(me);
      me->pParams->positionLimits[me->eTargetPos].targetVolts =
        me->status.piezoVoltage;
      movementIsComplete = true;
    }
    else if(FLUID_OVERSHOOT_COMP_PIEZO_VOLTS == eCompType)
    {
      /// Update the piezo voltage.
      /// Wait for completion.
      lowerPos = (uint32_t)me->eTargetPos;
      lowerPos --;
      
      /// Set Piezo voltage to V[pos] - ((V[pos] - V[pos - 1]) * compensationFactor);
      
      overshootPiezoVoltage = me->pParams->positionLimits[me->eTargetPos].targetVolts;
      overshootPiezoVoltage -= me->pParams->positionLimits[lowerPos].targetVolts;
      overshootPiezoVoltage *= me->pParams->compensationProportion;
      
      overshootParams.targetVoltage = me->pParams->positionLimits[me->eTargetPos].targetVolts - overshootPiezoVoltage;
      
      /// This high speed "push-back" has to be at the fastest possible rate.
      overshootParams.rampSpeed = PIEZO_RAMP_MAX;
      overshootParams.publishCompletion = false;
      
      // Save this slightly lower voltage.
      me->pParams->positionLimits[me->eTargetPos].targetVolts = overshootParams.targetVoltage;
      
      error = piezoVoltageSet(me->pPiezo, &overshootParams);
    }
    
    // Only remaining compensation method is to break and remake contact.
    else
    {
      // Move back down to the previous contact.
      me->status.eMoveDirection = FLUID_MOVE_REV;
      // Need to transition to movement state.
      retCode = X_TRAN(me, &FluidicState_MoveContact);
    }
  }
  else if(me->status.eMoveDirection == FLUID_MOVE_REV)
  {
    // Bug observed in units
    // Therefore just stop if performing fluid move.
    FluidicOnMoveCompleteMsg(me);
    me->pParams->positionLimits[me->eTargetPos].targetVolts =
      me->status.piezoVoltage;
    
    movementIsComplete = true;
    
    /// @todo Investigate why lines are commented out.
    ///       Think there was a bug discovered here.
    ///       Raised 1543 to keep track.
    
    
//    // Having completed the reverse move, perform a "return" movement.
//    // This behaves in same way as the forwards move, except that
//    // the speed is halved.
//    // Need to change current position to be one lower than target (as we've overshot)
//    lowerPos = (uint32_t)me->eLastKnownPos;
//    lowerPos --;
//    me->eLastKnownPos = (eFluidicPositions_t)lowerPos;
//    
//    if(me->pParams->returnSpeedRedcutionFactor > 0.f)
//    {
//      me->pParams->rampSpeedVoltsPerSec /= me->pParams->returnSpeedRedcutionFactor;  
//    }
//
//    me->status.eMoveDirection = FLUID_MOVE_RETURN;
//    retCode = X_TRAN(me, &FluidicState_MoveContact);
  }
  
  // Only other option is the return swing.
  // In this case we publish for completion!
  else
  {
    FluidicOnMoveCompleteMsg(me);
    me->pParams->positionLimits[me->eTargetPos].targetVolts =
      me->status.piezoVoltage;
    
    movementIsComplete = true;
    
  }
  
  // 
  // Keep all the state transitions to a single point. 
  // Again, as the movement was to a fixed contact (and will be held there)
  // can transition to breach detect.
  // 
  if(movementIsComplete)
  {
    if(me->pParams->monitorBreachAfterMove)
    {
      retCode = X_TRAN(me, FluidicState_MonitorFluidBreach);
    }
    else
    {
      retCode = X_TRAN(me, FluidicState_Idle);
    }
  }

  return FluidicsErrorSet(me, error, retCode);
}
  


/**
* @brief Mix state of the Fluidic channel
* @param me The fluidic object. 
* @param pEv Input events to be handled.
* @details Responds to the following events:

Event                     | Actions
------------------------- | ---------------------------------
X_EV_ENTRY                | Starts the movement (for this stage of mixing)
------------------------- |----------------------------------
X_EV_TIMER                | Processing for the fluidic channel, on the 20ms timer. Detects the end of the mixing stage.
------------------------- |----------------------------------
XMSG_FLUID_CHANNEL_CANCEL | Stops the fluid channel mixing, and begins a move back to the rest position.
--------------------------|----------------------------------
XMSG_PIEZO_MOVE_COMPLTE   | Used to indicate that the mixing stage movement has failed (no echem change). The fluid will move to the next stage.
------------------------- |----------------------------------
default                   | Calls the default event handler.  

* @returns The state response.
**/               
STATIC XState FluidicState_MixContactControlled(Fluidic_t* me, XEvent_t* pEv)
{
  XState retCode = X_RET_HANDLED;
  eErrorCode error = OK_STATUS;
    PiezoMoveCompltEv_t* pMoveCmplt = (PiezoMoveCompltEv_t*) pEv;
  eXEventId eventId = pEv->id;
  
  switch(eventId)
  {
  case X_EV_ENTRY:
    error = FluidicMixContactControlled_OnEntry(me);
    break;
    
  case X_EV_TIMER:
    retCode = FluidicMixContactControlled_OnTick(me);
    break;
    
  case XMSG_PIEZO_MOVE_COMPLTE:
    if(pMoveCmplt->chan == me->pPiezo->pParams->chan)
    {
      // Move completed, without reaching the contact.
      // Therefore increase the hysterisis on the target contact.
      AdjustHysterisisVoltage(me, FLUID_HYST_INC);
      me->status.piezoVoltage = pMoveCmplt->piezoVoltage;  
      // Stage is complete. Do the next stage!
      retCode = FluidMixOnStageComplete(me);
    }
    else
    {
      retCode = X_RET_HANDLED;
    }
    break;
    
  case XMSG_FLUID_CHANNEL_CANCEL:
    //A cancel fluid mix causes it to create a move command to the last known position.
    //No parameters are involved.
    me->eTargetPos = me->pParams->eMixEndPosition;
    me->status.mixComplete = true;
    
    // Movement is cancelled automatically when new move message is sent to Piezo.
    // If we send cancel command it could end up with Fluidic in odd state!
    retCode = X_TRAN(me, &FluidicState_MoveContact);
    break;
    
  default:
    retCode = Fluidic_defaultEvents(me, pEv);
    break;
  }
  
  retCode = FluidicsErrorSet(me, error, retCode);
  return retCode;
}



/**
  * @brief Mixing state of the fluid controller, when Piezo movement alone is
  *        used to derive the mixing position.
  *       (No electrochemical checks performed to check against)
  * @note This state is used for FLUID_MIX_OPEN_LOOP and FLUID_MIX_SINGLE_POINT_LOOP
  *       mixing.
  * @param me The fluidic object. 
  * @param pEv Input events to be handled.
  * @details
  
  
    Event                     | Actions
    ------------------------- | ---------------------------------
    X_EV_ENTRY                | Initialises the fluid channel mixing process.
    ------------------------- |----------------------------------
    X_EV_TIMER                | Processing for the fluidic channel, on the 20ms timer.
    ------------------------- |----------------------------------
    XMSG_FLUID_CHANNEL_CANCEL | Stops the fluid channel mixing, and begins a move back to the rest position.
    --------------------------|----------------------------------
    XMSG_PIEZO_MOVE_COMPLTE   | Used to indicate that the mixing stage movement has compelted.
    ------------------------- |----------------------------------
    default                   | Calls the default event handler. 
  **/
STATIC XState FluidicState_MixPiezoControlled(Fluidic_t* me, XEvent_t* pEv)
{
  XState retCode = X_RET_HANDLED;
    PiezoMoveCompltEv_t* pMoveCmplt = (PiezoMoveCompltEv_t*) pEv;
  eXEventId eventId = pEv->id;
  
  eErrorCode error = OK_STATUS;
  
  switch(eventId)
  {
  case X_EV_ENTRY:
    error = FluidicMixPiezoControlled_OnEntry(me);
    break;
    
  case X_EV_TIMER:
    me->mixTimer += FLUIDIC_TIMER_COUNT_MS;
    
    if(me->mixTimer >= me->pParams->mixTimeout_ms)
    {
      /* Let the script runner know that this is down to timeout
       * (instrument inactivity). */
      FluidicOnMoveFailMsg(me, me->eTargetPos, ERROR_DXRUNNER_FMIX_TIMEOUT);
      me->eTargetPos = me->pParams->eMixEndPosition;
      
      retCode = X_TRAN(me, &FluidicState_MoveContact);
    }
    break;
    
  case XMSG_PIEZO_MOVE_COMPLTE:  
    if(pMoveCmplt->chan == me->pPiezo->pParams->chan)
    {
      // Move completed.
      me->status.piezoVoltage = pMoveCmplt->piezoVoltage;  
      // Stage is complete. Do the next stage!
      retCode = FluidMixOnStageComplete(me);
    }
    else
    {
      retCode = X_RET_HANDLED;
    }
    break;
    
  case XMSG_FLUID_CHANNEL_CANCEL:
    //A cancel fluid mix causes it to create a move command to the last known position.
    //No parameters are involved.
    me->eTargetPos = me->pParams->eMixEndPosition;
    me->status.mixComplete = true;
    
    // Movement is cancelled automatically when new move message is sent to Piezo.
    // If we send cancel command it could end up with Fluidic in odd state!
    retCode = X_TRAN(me, &FluidicState_MoveContact);
    break;
    
  default:
    retCode = Fluidic_defaultEvents(me, pEv);
    break;
  }
  
  retCode = FluidicsErrorSet(me, error, retCode);
  
  return retCode;
}



/**
  * @brief FluidicState_MixWaitContinue
  * @details Wait state, wait for all fluid channels to be synchronised before continuing mixing.
  **/
STATIC XState FluidicState_MixWaitContinue(Fluidic_t *me, 
                                           XEvent_t* pEv)
{
  XState retCode = X_RET_HANDLED;
  
  eXEventId eventId = pEv->id;
  
  eErrorCode error = OK_STATUS;
  
  switch(eventId)
  {
  case X_EV_ENTRY:
    // Stop any Piezo movement!
    // Waiting for our next command.
    (void)piezoStop(me->pPiezo);
    
    X_PUBLISH(X_FRAMEWORK_OF(me), me->stageCompelteMsg);
    break;
    
  case X_EV_TIMER:
    me->mixTimer += FLUIDIC_TIMER_COUNT_MS;
    
    if(me->mixTimer >= me->pParams->mixTimeout_ms)
    {
      FluidicOnMoveFailMsg(me, me->eTargetPos, ERROR_DXRUNNER_FMIX_TIMEOUT);
      me->eTargetPos = me->pParams->eMixEndPosition;
      
      retCode = X_TRAN(me, &FluidicState_MoveContact);
    }
    break;
    
  case XMSG_FLUID_MIX_CONTINUE:
    retCode = FluidMixMovementContinue(me);
    break;
    
  case XMSG_PIEZO_MOVE_COMPLTE:   
    error = FluidOnPiezoMoveComplete(me, pEv);  
    break;
    
  case XMSG_FLUID_CHANNEL_CANCEL:
    //A cancel fluid mix causes it to create a move command to the last known position.
    //No parameters are involved.
    me->eTargetPos = me->pParams->eMixEndPosition;
    me->status.mixComplete = true;
    
    // Movement is cancelled automatically when new move message is sent to Piezo.
    // If we send cancel command it could end up with Fluidic in odd state!
    retCode = X_TRAN(me, &FluidicState_MoveContact);
    break;
    
  default:
    retCode = Fluidic_defaultEvents(me, pEv);
    break;
  }
  
  retCode = FluidicsErrorSet(me, error, retCode);
  return retCode;
}



/**
  * @brief Fluid breach (overshoot) monitoring state.
  * @details Fluid position is held static (first implementation), the fluid 
  *          position from the electrochem object is monitored to check for breach.
  *          If a breach is detected then publish the event (handled by event sender)
  *          Accepts new move and hold commands.
  * @param me The fluidic object. 
  * @param pEv Input events to be handled.
  * @returns State response code.
  **/
STATIC XState FluidicState_MonitorFluidBreach(Fluidic_t* me, XEvent_t* pEv)
{
  XState retCode = X_RET_HANDLED;
  
  eXEventId eventId = pEv->id;
  
  eElectrochemicalChannel ecChan = me->pParams->eChannel;
  
  switch(eventId)
  {
  case X_EV_ENTRY:
    // piezo movement stopping is handled by another state.
    // This state is only entererd after movement completion
    // Just turn on the echem channel.
    (void)ecSetModeFillDetect(me->pEchem, ecChan, ConvertFluidPosToEchemPos(me->eLastKnownPos));                      
    break;
    
  case XMSG_EC_FLUID_STATUS_CHANGED:
    (void)FluidOnEchemStatusChange(me, pEv);
    
    // If the fluid is not in the right position, publish event and transition to
    // the idle state.
    if(me->status.eFluidFrontPosition != 
       fluidGetEchemRequirement(me, false))
    {
      // Breach detected. Go back to idle and publish a failure event.
      X_PUBLISH(X_FRAMEWORK_OF(me),
                me->breachDetectedMsg);        // Always publish move success message.
      
      FluidicOnMoveFailMsg(me, me->eLastKnownPos, ERROR_FLUID_CHANNEL_FLUID_FRONT);
      retCode = X_TRAN(me, &FluidicState_Idle);
    }
    
    break;
    
  case XMSG_FLUID_CHANNEL_MOVE_TO:
    retCode = OnMsgBladderControlMoveToPos(me, pEv);
    break;
    
  case XMSG_FLUID_MIX:
    retCode = OnMsgBladderControlMix(me, pEv);
    break;
    
  default:
    retCode = Fluidic_defaultEvents(me, pEv);
    break;
  }
  
  return retCode;
}



/**
* @brief Error state of the Fluidic channel
* @param me The fluidic object. 
* @param pEv Input events to be handled.
* @details Responds to the following events:

Event                     | Actions
------------------------- | ---------------------------------
X_EV_ENTRY                | Disables the Piezo bender, fill-detect channel, and publishes an error message.
------------------------- |----------------------------------
XMSG_FLUID_ERR_CLEAR      | Clears the error state, returns to idle.
------------------------- |----------------------------------
default                   | Calls the default event handler.  

* @returns The state response.
* @warn Cannot call ErrorCheck inside this state, as we are already in error state.
**/ 
STATIC XState FluidicState_Err(Fluidic_t* me, XEvent_t* pEv)
{
  XState retCode = X_RET_HANDLED;
  eXEventId eventId = pEv->id;
  
  switch(eventId)
  {
  case X_EV_ENTRY:
    Fluidic_OnErrorStateEntry(me);
    break;
    
  case XMSG_FLUID_ERR_CLEAR:
    retCode = X_TRAN(me, &FluidicState_Idle);
    break;
    
  default:
    retCode = Fluidic_defaultEvents(me, pEv);
    break;
  }
  
  return retCode;
}


/**
  * @brief Helper for entry to error state.
  * @param[in] me - The fluidic instance.
  * @details Actions on entry:
  *   -# Disable Echem and stop Piezo movement.
  *   -# Print any error codes.
  *   -# Reset the error! This is critical, as otherwise may have another critical error transition.
  *   -# Set current position as unknown.
  *   -# Publish critical error event.
  **/
STATIC void Fluidic_OnErrorStateEntry(Fluidic_t *me)
{
  eErrorCode error;
  // Disable echem, and stop piezo movement. Report error (on debug port)
  error = ecDisable(me->pEchem, me->pParams->eChannel);
  ERROR_CHECK(error);
  error = FluidicStopMove(me);
  ERROR_CHECK(error);
  
  me->status.eFluidFrontPosition = FD_DATA_INVALID;
  
  //Forces a homing move after exiting the error state.
  FluidicSetCurrentAndTargetPositions(me, BC_POS_UNKNOWN, BC_NONE);
  
  // Now publish error message.
  me->errorMsg.errorCode =  me->super.super.errorCode;
  X_PUBLISH(X_FRAMEWORK_OF(me), me->errorMsg);
}


/** @} **/

/**
* @defgroup fStateHelper Fluidic State Helper Functions
* @brief Functions which perform processing as part of the State handlers.
* @{
**/


/**
* @brief Default events handler of the fluidic channel
* @param me The fluidic object. 
* @param pEv Input events to be handled.
* @details Responds to the following events:

Event                     | Actions
------------------------- | ---------------------------------
XMSG_FLUID_CHANNEL_CANCEL, XMSG_GLOBAL_HALT | Cancel any movements/mixing. Sets to idle. The data is set invalid, piezo movement is disabled, and the fill detect is disabled.
------------------------- |----------------------------------
XMSG_FLUID_CHANNEL_MOVE_TO,  XMSG_FLUID_MIX , XMSG_FLUID_ERR_CLEAR    | Event is ignored.
------------------------- |----------------------------------
XMSG_PIEZO_MOVE_COMPLTE    | Updates the piezo object status in the channel's status information.
------------------------- |----------------------------------
XMSG_PIEZO_STOPPED        | Updates the piezo object status in the channel's status information.
------------------------- |----------------------------------
XMSG_EC_FLUID_STATUS_CHANGED | Updates the electrochem object status in the channel's status information.
------------------------- |----------------------------------
XMSG_PEIZO_MOVE_FAIL      | Updates the fluidic channel parameters
------------------------- |----------------------------------
XMSG_FLUID_CHANNEL_NEW_PARAMS | Updates the fluidic channel parameters
------------------------- |----------------------------------
XMSG_EC_ERROR             | Updates the echem status in the fluidic object, and checks for critical errors.
------------------------- |----------------------------------
default                   | Ignored.


* @returns The state response.
**/ 
STATIC XState Fluidic_defaultEvents(Fluidic_t* me, XEvent_t* pEv)
{
  XState retCode = X_RET_HANDLED;
  eErrorCode error = OK_STATUS;
  PiezoMoveFailEv_t* pMoveFailMsg;
  EchemErrorMsg_t* pEchemErrMsg;
  FluidicMonitorBreachMsg_t* pMonitorFluidBreachMsg;
  
  
  FluidicMovePositionMsg_t *pFMoveEv;
  
  switch(pEv->id)
  {    
  case XMSG_FLUID_CHANNEL_CANCEL:      
  case XMSG_GLOBAL_HALT:                
    if(me->eTargetPos != BC_NONE)
    {
      FluidicOnMoveCompleteMsg(me);                                             //Publish a move complete message so that other objects know we've stopped moving.
      FluidicSetCurrentAndTargetPositions(me, BC_POS_UNKNOWN, BC_NONE);
    }
    
    // Stopping movement and disabling echem handled in idle state.
    me->status.eFluidFrontPosition = FD_DATA_INVALID;
    retCode = X_TRAN(me, &FluidicState_Idle);
    break;
    
  case XMSG_DOOR_OPENED:
    // No error codes generated, just perform a homing move.
    // Do not need to use OnMsgBladderControlMoveToPos as 
    // target position is known.
    me->eTargetPos = BC_POS_HOME;
    
    me->publishCompletionEvent = false; /// Only case where command complete should not be published.
    
    retCode = X_TRAN(me, &FluidicState_MoveOther);
    break;
    
  // Only handled here if not expecting a new command. 
  // Therefore only allowed to be a homing move.
  case XMSG_FLUID_CHANNEL_MOVE_TO:
    pFMoveEv = (FluidicMovePositionMsg_t*)pEv;
    
    if(pFMoveEv->eTargetPos == BC_POS_HOME)
    {
      retCode = OnMsgBladderControlMoveToPos(me, pEv);
    }
    break;
   
  // Ignore error clear and mix commands if not in appropriate state.
  case XMSG_FLUID_MIX:
  case XMSG_FLUID_ERR_CLEAR:
    error = OK_STATUS;
    retCode = X_RET_IGNORED;
    break;
    
    
    // Update the piezo status / echem status on receipt of update messages.
    // Can also use data requests to get exact voltages as required.
  case XMSG_PIEZO_MOVE_COMPLTE:
    error = FluidOnPiezoMoveComplete(me, pEv);    
    break;
    
  case XMSG_PIEZO_STOPPED:
    error = FluidOnPiezoStop(me, pEv);
    break;
    
  case XMSG_EC_FLUID_STATUS_CHANGED:
    error = FluidOnEchemStatusChange(me, pEv);
    break;  
    
    
    // Parameter update messages.
  case XMSG_FLUID_CHANNEL_NEW_PARAMS:
    OnMsgUpdateParams(me, pEv);
    error = OK_STATUS;
    retCode = X_RET_HANDLED;
    break;
    
    
  case XMSG_PEIZO_MOVE_FAIL:
    pMoveFailMsg = (PiezoMoveFailEv_t *) pEv;
    if(pMoveFailMsg->chan == me->pPiezo->pParams->chan)
    {
      error = pMoveFailMsg->error;
    }
    else
    {
      error = OK_STATUS;
    }
    break;
    
  case XMSG_EC_ERROR:
    pEchemErrMsg = (EchemErrorMsg_t *) pEv;
    error = pEchemErrMsg->errorCode;
    break;
    
    // On every exit transition, turn off the timer.
    // Timer should be re-enabled as required.
  case X_EV_EXIT:
    error = OK_STATUS;
    XTimerStop(&(me->timer));
    break;
    
    // Set the breach detect status to new value in message.
  case XMSG_FLUID_ENABLE_BREACH_DETECT:
    pMonitorFluidBreachMsg = (FluidicMonitorBreachMsg_t*)pEv;
    me->pParams->monitorBreachAfterMove = pMonitorFluidBreachMsg->monitorFluidPosition;
    break; 
    
  default:
    retCode = X_RET_IGNORED;
    error = OK_STATUS;
    break;
  }
  
  retCode = FluidicsErrorSet(me, error, retCode);
  
  return retCode;
}




/**
*     @brief Processes the entry event of state FluidicState_MoveContact
*     @param[in] me - The fluidic controller instance.
*     @details Enables the fluid detect system, and starts the 
*              Piezo movement.
*     @returns Status of the fluid detect start and Piezo move commands.
*     @note   If the movement cannot be completed (error with fluidics or piezo)
*             then a movement fail event is published.
**/
STATIC eErrorCode OnFluidMoveContact_Entry(Fluidic_t* me)
{
  eErrorCode error;
  eElectrochemicalChannel ecChan = me->pParams->eChannel;
  
  XTimerStart(&(me->timer));    // Need timer if moving.
  
  //First - Check whether eChem is needed.
  eFluidicPositions_t eTarget = me->eTargetPos;
  
  ASSERT((me->eTargetPos <= BC_VALID_POS_COUNT) && 
         (me->eTargetPos > BC_POS_DOWN));
  
  me->timeoutTimer = 0u;      // Reset timeout timer.
  
  error = ecSetModeFillDetect(me->pEchem, ecChan, EC_CHAN_POS_A);    // We're moving to a contact, therefore we need to monitor all contacts.                  
  
  
  // If no errors when setting up echem, start piezo movements.
  if(OK_STATUS == error)
  {
    error = FluidicBeginPiezoMoveToTarget(me);
  }
  // If there was an error setting up echem then we have failed moving.
  // publish movement fail event.
  else
  {
    FluidicOnMoveFailMsg(me, eTarget,error);
  }
  
  return error;
}



/**
*     @brief Processes the entry event of state FluidicState_MoveOther
*     @param[in] me - The fluidic controller instance.
*     @details Starts the Piezo movement. If moving to bladder down will enable fill detect.
*     @returns Status of the fluid detect start and Piezo move commands.
*     @note   If the movement cannot be completed (error with fluidics or piezo)
*             then a movement fail event is published.
**/
STATIC eErrorCode OnFluidMoveOther_Entry(Fluidic_t* me)
{
  ASSERT_NOT_NULL(me);
  
  eErrorCode err ;
  
  me->timeoutTimer = 0u;
  XTimerStart(&(me->timer));    // Need timer if moving.
  
  if(me->eTargetPos == BC_POS_HOME)
  {
    err = FluidicHomeMoveBegin(me);
  }
  else
  { 
    // If we're moving to home / down, then we don't want our contacts to be 
    // enabled.
    err = ecDisable(me->pEchem, me->pParams->eChannel);
    
    if (OK_STATUS == err)
    {
      err = FluidicBeginPiezoMoveToTarget(me);
      
      if (err == OK_COMMAND_ACCEPTED)
      {
        err = OK_STATUS;
      }
    }
    
    // If there was an error setting up echem then we have failed moving.
    // publish movement fail event.
    if (OK_STATUS != err)
    {
      FluidicOnMoveFailMsg(me, me->eTargetPos, err);
    }
    else
    {
      err = OK_STATUS;
    }
  }

  return err;
}

/**
*     @brief Processes the entry event of state FluidicState_LiftUpBladder
*     @param[in] me - The fluidic controller instance.
*     @details Starts the Piezo movement
*     @returns Status of the fluid detect start and Piezo move commands.
*     @note   If the movement cannot be completed (error with fluidics or piezo)
*             then a movement fail event is published.
**/
STATIC eErrorCode OnLiftUpBladders_Entry(Fluidic_t* me)
{
  ASSERT_NOT_NULL(me);

  eErrorCode err;

  me->timeoutTimer = 0u;
  XTimerStart(&(me->timer));    // Need timer if moving.

  err = FluidicBeginPiezoMoveToLift(me);

  if(OK_COMMAND_ACCEPTED != err)
  {
    FluidicOnMoveFailMsg(me, me->eTargetPos, err);
  }

  return err;
}


/**
*     @brief Processes the timer tick for the Move state.
*     @param[in] me - The fluidic controller instance.
*     @details Checks the status of the Piezo and Fluid detect system to
*              determine if a move has been completed, or has failed.
*              As we're moving to a channel contact we always 
*     @returns State transition code.
**/
STATIC XState FluidicMoveContact_OnTick (Fluidic_t* me)
{
  XState retCode = X_RET_HANDLED;
  
  ASSERT((me->eTargetPos != BC_VALID_POS_COUNT) && 
         (me->eTargetPos != BC_NONE) && 
           (me->eTargetPos != BC_POS_UNKNOWN));
  
  eEcFluidDetectPosition_t eRequirement = fluidGetEchemRequirement(me,
                                                                   true);

  eErrorCode error = OK_STATUS;
  
  eEcFluidDetectPosition_t eFluidFrontPos = me->status.eFluidFrontPosition;
  
  // Something is wrong in the fluid position.
  if((eFluidFrontPos < FLUID_DETECTED) ||(eFluidFrontPos >= FLD_POS_NUM)) 
  {
    FluidicOnMoveFailMsg(me, me->eTargetPos, ERROR_FLUIDIC_UNKNOWN_MSG_FROM_EC);
    retCode = X_TRAN(me, &FluidicState_Idle);
  }
  
  // The Piezo is still moving. Therefore check the fluid front position.
  else 
  {
    // If the fluid front is where we expect it to be then movement complete.
    if(eRequirement == eFluidFrontPos)
    {
      error = FluidicStopMove(me);                            //Need to stop the Piezo moving, before going idle/ performing next move.
      retCode = X_TRAN(me, &FluidicState_WaitForPiezoStop);   // Go to wait state. (This should only take 1 ThreadX tick,
                                                              // but could take more.
    }
  }
  
  // Increment timer, and check for timer overflow.
  me->timeoutTimer += FLUIDIC_TIMER_COUNT_MS;
  
  if(me->timeoutTimer >= me->pParams->timeout_ms)
  {    
    FluidicOnMoveFailMsg(me, me->eTargetPos, ERROR_DXRUNNER_FMOV_TIMEOUT);
    retCode = X_TRAN(me, &FluidicState_Idle);
  }
  
  return FluidicsErrorSet(me, error, retCode);
}



/**
* @brief      Handles the entry event for the mixing state.
* @details    Starts the movement to the next stage of the mixing; enables the
*             electrochemistry channel for fluid position detection.
* @param[in] me - The fluidic controller
* @returns The state handler code.
**/
STATIC eErrorCode FluidicMixContactControlled_OnEntry(Fluidic_t* me)
{
  eErrorCode error;
  float startVolts;
  
  float endVolts;
  
  if(me->pParams->eMixType == FLUID_MIX_DUAL_POINT_LOOP)
  {
    startVolts = me->pParams->positionLimits[me->eLastKnownPos].targetVolts;
    endVolts = me->pParams->positionLimits[me->eTargetPos].targetVolts;
    
    if(me->status.eMoveDirection == FLUID_MOVE_REV)
    {
      endVolts -= me->pParams->positionLimits[me->eTargetPos].posHysterisis;
    }
    else
    {
      endVolts += me->pParams->positionLimits[me->eTargetPos].posHysterisis;
    }
  }
  else
  {
    if(me->status.eMoveDirection == FLUID_MOVE_REV)
    {
      startVolts =  me->pParams->positionLimits[me->eLastKnownPos].targetVolts;
      endVolts    = FluidicMixCalculateDownStrokeMixProportion(me);
    }
    else
    {
      startVolts = piezoVoltageGet(me->pPiezo);
      endVolts = me->pParams->positionLimits[me->eTargetPos].targetVolts;
      endVolts += me->pParams->positionLimits[me->eTargetPos].posHysterisis;
    }
  }
  
  
  eElectrochemicalChannel ecChan = me->pParams->eChannel;
  
  // Start timer, reset number of mix cycles, timer, and movement failure count.
  XTimerStart(&(me->timer));

  // Calculate Piezo ramp speed from the mixing frequency.
  fluidicMixingCalculaterampSpeedVoltsPerSec(me,
                                             startVolts,
                                             endVolts);
  
  // Enable the fill detect for our channel.
  // As we're moving to a contact, we set the minimum contact to our postion A.
  error = ecSetModeFillDetect(me->pEchem, ecChan, EC_CHAN_POS_A);                      
  
  //No homing move is allowed in mixing.
  if(OK_STATUS == error)
  {    
    // Can use move to target for this type of movement.
    error = FluidicBeginPiezoMoveToTarget(me);
  }
  
  return error;
}




/**
* @brief      Handles the entry event for the mixing state.
* @details    Starts the movement to the next stage of the mixing; no echem used for this move,
* @param[in] me - The fluidic controller
* @returns The state handler code.
**/
STATIC eErrorCode FluidicMixPiezoControlled_OnEntry(Fluidic_t* me)
{
  eErrorCode error;
  
  float startVolts;
  float endVolts;
  
  // If reversing, then we will be at the top position
  // So can sue the explicit position.
  if(me->status.eMoveDirection == FLUID_MOVE_REV)
  {
    startVolts  = me->status.piezoVoltage;
    
    endVolts    = FluidicMixCalculateDownStrokeMixProportion(me);
    
    // If it's the first step back to the top, we need to compensate for additional movement.
    if((me->status.mixingStagesCompleted == 1u) && (me->pParams->eMixType == FLUID_MIX_OPEN_LOOP))
    {
      endVolts -= endVolts * me->pParams->openLoopCompensationFactor;
    }
  }
  else
  {
    endVolts    = me->pParams->positionLimits[me->eTargetPos].targetVolts;
    startVolts  = me->status.piezoVoltage;
  }
  
  
  // Start timer, reset number of mix cycles, timer, and movement failure count.
  XTimerStart(&(me->timer));
  
  // Calculate ramp speed based on movement amount.
  fluidicMixingCalculaterampSpeedVoltsPerSec(me,
                                             startVolts,
                                             endVolts);
  
  // Manually set the Piezo movement.
  peizoMoveParams_t piezoParams;
  
  piezoParams.rampSpeed = me->pParams->rampSpeedVoltsPerSec;
  piezoParams.targetVoltage = endVolts;
  piezoParams.publishCompletion = false;
  
  error = piezoVoltageSet(me->pPiezo, &piezoParams);
  
  return error;
}


/**
  * @brief Helper function to calculate the lower movement position, when performing
  *        and open loop movement (no echem contact change)
  * @param[in] me - The fluidic control object
  * @returns The lower position voltage.
  **/
STATIC float FluidicMixCalculateDownStrokeMixProportion(Fluidic_t *me)
{
  float endVolts = me->pParams->positionLimits[me->eTargetPos].targetVolts;
  float startVolts = me->status.piezoVoltage;
  
  // Direction is always reverse.  
  endVolts = startVolts - endVolts;
  endVolts *= me->pParams->mixDownstrokeProportion;
  
  endVolts = startVolts - endVolts;
  

  return endVolts;
}


/**
* @brief Processing of tick events in the mix state.
* @details    Mixing control is performed using the electrochemical contacts.
*             The Piezo moves to targetVoltage +/- hysterisis, as necessary.
*             If the Piezo movement completes, but the fluid front is not in
*             the expected position we increase the hysterisis level.
*             If the fluid front reaches the correct position, but the Piezo 
*             is still moving, the Piezo is stopped and the hystserisis level
*             decreased.
*             After each swing is completed we invert the movement direction.
*             When the timer expires the target is set to the rest position.
*             We have to wait for "stopped" events to be recieved before
*             generating new moves.
* @param[in] me - The fluidic controller
* @returns The state handler code.
**/
STATIC XState FluidicMixContactControlled_OnTick(Fluidic_t* me)
{
  XState retCode = X_RET_HANDLED;
  
  bool fluidInCorrectPos;
  
  eErrorCode error = OK_STATUS;
  
  ASSERT((me->eTargetPos != BC_VALID_POS_COUNT) && 
         (me->eTargetPos != BC_NONE) && 
           (me->eTargetPos != BC_POS_UNKNOWN));
  
  // Get the fluid front position. This changes depending on the direction of
  // movement.
  eEcFluidDetectPosition_t eTargetFrontPosition = fluidGetEchemRequirement(me,
                                                                           true);
  
  me->mixTimer +=  FLUIDIC_TIMER_COUNT_MS;
  
  // If we've exceeded mixing timeout then we've failed.
  // Publish error
  if(me->mixTimer >= me->pParams->mixTimeout_ms)
  {
    /* Let the script runner know that this is down to timeout
     * (instrument inactivity). */
    FluidicOnMoveFailMsg(me, me->eTargetPos, ERROR_DXRUNNER_FMIX_TIMEOUT);
    me->eTargetPos = me->pParams->eMixEndPosition;
    
    retCode = X_TRAN(me, &FluidicState_MoveContact);
    fluidInCorrectPos = false;
  }
  
  else
  {
    // If the fluid front is in the targeted position, then we know we've completed
    // the movement stage.
    // Check >= target 
    if((eTargetFrontPosition <= me->status.eFluidFrontPosition)
       && (me->status.eMoveDirection == FLUID_MOVE_FWD))
    {
      fluidInCorrectPos = true;
    }
    else if((eTargetFrontPosition >= me->status.eFluidFrontPosition)
       && (me->status.eMoveDirection == FLUID_MOVE_REV))
    {
      fluidInCorrectPos = true;
    }
    else
    {
      fluidInCorrectPos = false;
    }
  }
  
  
  if(fluidInCorrectPos)
  {
    me->pParams->positionLimits[me->eTargetPos].targetVolts = 
      piezoVoltageGet(me->pPiezo);
    
    AdjustHysterisisVoltage(me, FLUID_HYST_DEC);
    
    retCode = FluidMixOnStageComplete(me);
  }

  retCode = FluidicsErrorSet(me, error, retCode);

  
  return retCode;
}


/**
  * @brief Helper function to complete fixed actions at the end of each
  *        fluid mixing stage.
  * @param[in] me - The fluidic controller
  * @details Increments the number of mixing stages complete, checks to see if
  *          mixing has compelted; otherwise the mixing is continued.
  *          If complete, the fluid controller moves back to the mixing end point.
  * @returns State transition code.
  **/
STATIC XState FluidMixOnStageComplete(Fluidic_t *me)
{
  XState retCode;
  
  me->status.mixingStagesCompleted ++;
  
  // Check to see if we've compelted the move.
  if((me->status.mixingStagesCompleted / FLUID_NUM_MIXING_STAGES_PER_CYCLE) 
     >= me->pParams->targetMixCycles)
  {
    FluidicOnMixComplete(me); 
    
    retCode = X_TRAN(me, &FluidicState_Idle);  
  }
  
  // Publish the stage complete command.
  // Go to wait state.
  else
  {
    retCode = X_TRAN(me, &FluidicState_MixWaitContinue);
  }
  
  return retCode;
}




/**
  *   @brief Helper to re-set movement of Piezo for mixing.
  *   @details Need to invert the target and known positions, re-calculate the
  *            mixing speed (to maintain consistent frequency)
  *            and start moving.
  *            At this point increment mixing cycles, and reset failure count.
  *   @returns Error code from Piezo move begin.
  **/
STATIC XState FluidMixMovementContinue(Fluidic_t* me)
{
  XState retCode;
  
  // Swap the current and target positions.
  FluidicSetCurrentAndTargetPositions(me,
                                      me->eTargetPos,
                                      me->eLastKnownPos);
  
  // Invert the direction.
  if(me->status.eMoveDirection == FLUID_MOVE_REV)
  {
    me->status.eMoveDirection = FLUID_MOVE_FWD;
  }
  else
  {
    me->status.eMoveDirection = FLUID_MOVE_REV;
  }
  
  // Now select the appropriate transition for the mixing type.
  if(me->pParams->eMixType == FLUID_MIX_DUAL_POINT_LOOP)
  {
    retCode = X_TRAN(me, &FluidicState_MixContactControlled);
  }
  
  else if(me->pParams->eMixType == FLUID_MIX_SINGLE_POINT_LOOP)
  {
    // If direction now forwards then we need to remain in this state.
    // Otherwise, transition to Piezo controlled.
    if(me->status.eMoveDirection == FLUID_MOVE_FWD)
    {
      retCode = X_TRAN(me, &FluidicState_MixContactControlled);
    }
    else
    {
      retCode = X_TRAN(me, &FluidicState_MixPiezoControlled);
    }
  }
  
  else
  {
    retCode = X_TRAN(me, &FluidicState_MixPiezoControlled);
  }
  
  return retCode;
}


/**
*     @brief Helper fucntion to get the desired echem state.
*     @param[in] me The fluidic object.
*     @param[in] isMoving Boolean flag to determine whether to check the target
*                         position (if true), or the current position.
*     @returns The reverse, or forward, echem status based on the movement direction.
**/
STATIC eEcFluidDetectPosition_t fluidGetEchemRequirement(Fluidic_t* me,
                                                         bool isMoving)
{
  ASSERT_NOT_NULL(me);
  
  
  eFluidicPositions_t     ePosition;
  eFluidicMoveDirection_t eDirection;
  
  // Use boolean flag to select which position should be monitored.
  // If moving, then check target should be true. 
  // The behaviour of checking each target is the same.
  if(isMoving)
  {
    ePosition = me->eTargetPos;
    
    /// @note Moving forward, and "return" must have identical requirements when moving.
    eDirection = (me->status.eMoveDirection == FLUID_MOVE_REV) ? 
                  FLUID_MOVE_REV : FLUID_MOVE_FWD;
    
  }
  else
  {
    ePosition = me->eLastKnownPos;
    eDirection = FLUID_MOVE_FWD;  // If not moving (checking last position)
                                  // Then always check the "forward" element.
  }
  
  // Check the ePosition to prevent reading from non-existant array value.
  // Assertion should be after setting ePosition.
  ASSERT((ePosition != BC_NONE) && 
         (ePosition != BC_POS_UNKNOWN) && 
           (ePosition != BC_VALID_POS_COUNT));
  
  eEcFluidDetectPosition_t eRequirement = 
        me->pParams->positionLimits[ePosition].echemRequirements[eDirection];
  
  return eRequirement;
}

/** @} **/



/**
*  @defgroup fHelpersGen Fluidic Helper Functions
*  @brief General helper fucntions for Fluidic objects.
*  @{
**/


/**
*     @brief Processes the Piezo Stopped message.
*     @param[in]      me - The fluid controller instance.
*     @param[in]      pEv - The event
*     @returns        OK_STATUS
*     @todo           Consider if any error codes could be generated here.
**/
STATIC eErrorCode FluidOnPiezoStop(Fluidic_t* me, XEvent_t* pEv)
{
  PiezoStoppedEv_t* pMoveCmplt = (PiezoStoppedEv_t*) pEv;
  
  if(pMoveCmplt ->chan == me->pPiezo->pParams->chan)
  {
    me->status.piezoVoltage = pMoveCmplt->piezoVoltage;
  }
  
  return OK_STATUS;
}


/**
*     @brief Processes Piezo Move Complete events
*     @param[in] me - The fluidic controller instance.
*     @param[in] pEv - The event
*     @details Checks which Piezo published the event, if it is the Piezo object
*              associated with our fluid channel then the Piezo voltage is stored
*              and the Piezo status is set to PIEZO_MOVE_COMPLETE.
*     @returns State transition code.
**/
STATIC eErrorCode FluidOnPiezoMoveComplete(Fluidic_t* me, XEvent_t* pEv)
{
  PiezoMoveCompltEv_t* pMoveCmplt = (PiezoMoveCompltEv_t*) pEv;
  
  if(pMoveCmplt->chan == me->pPiezo->pParams->chan)
  {
    me->status.piezoVoltage = pMoveCmplt->piezoVoltage;
  }
  
  return OK_STATUS;
}



/**
*     @brief Stores the details of an electrochemical status change message.
*     @param[in]      me - The fluid controller instance.
*     @param[in]      pEv - The event
*     @returns        OK_STATUS
*     @todo           Consider if any error codes could be generated here.
**/
STATIC eErrorCode FluidOnEchemStatusChange(Fluidic_t* me, XEvent_t* pEv)
{   
  FillDetectStatusChange_t* pFdChange = (FillDetectStatusChange_t*)pEv;
  
  me->status.eFluidFrontPosition =  pFdChange->results.fluidPositions[me->pParams->eChannel];
  
  return OK_STATUS;
}



/**
*     @brief Starts moving the Piezo to the target.
*     @param[in]      me - The fluid controller instance.
*     @details        Sets the piezo movement voltage to the position's known 
*                     value +/- the hysterisis voltage depending on whether it
*                     is a forward, or reverse move. Then updates the fluid
*                     controller status flag.
*     @returns        Error code from the piezoVoltageSet API.
**/
STATIC eErrorCode FluidicBeginPiezoMoveToTarget(Fluidic_t* me)
{
  eErrorCode error;
  peizoMoveParams_t piezoParams;
  eFluidicPositions_t eTarget = me->eTargetPos;
  
  ASSERT((eTarget != BC_POS_UNKNOWN) && (eTarget != BC_NONE) && (eTarget != BC_VALID_POS_COUNT));
  
  piezoParams.targetVoltage = me->pParams->positionLimits[eTarget].targetVolts;
  piezoParams.rampSpeed     = me->pParams->rampSpeedVoltsPerSec;
  piezoParams.publishCompletion = false;
  
  
  if(me->status.eMoveDirection == FLUID_MOVE_REV)//Moving towards bladders     
  {
    piezoParams.targetVoltage = PIEZO_MIN_VOLTAGE;  // Force to be zero moving backwards.
  }
  else        //Moving the fluid towards bladders.
  {
    piezoParams.targetVoltage += me->pParams->positionLimits[eTarget].posHysterisis;
  }
  
  error = piezoVoltageSet(me->pPiezo, &piezoParams);
  
  return error;
}

/**
*     @brief          Starts moving the Piezo to HOME/open.
*     @param[in]      me - The fluid controller instance.
*     @details        Sets the piezo movement voltage to the position's known
*                     value +/- the hysterisis voltage (reverse move).
*                     Then updates the fluid controller status flag.
*     @returns        Error code from the piezoVoltageSet API.
**/
STATIC eErrorCode FluidicBeginPiezoMoveToLift(Fluidic_t* me)
{
  eErrorCode error;
  peizoMoveParams_t piezoParams;
  eFluidicPositions_t eTarget = me->eTargetPos;

  ASSERT((eTarget != BC_POS_UNKNOWN) && (eTarget != BC_NONE) && (eTarget != BC_VALID_POS_COUNT));

  piezoParams.targetVoltage     = me->pParams->positionLimits[eTarget].targetVolts;
  piezoParams.rampSpeed         = me->pParams->rampSpeedVoltsPerSec;
  piezoParams.publishCompletion = false;
  piezoParams.targetVoltage = PIEZO_RAMP_MAX +
                              me->pParams->positionLimits[eTarget].posHysterisis;

  error = piezoVoltageSet(me->pPiezo, &piezoParams);

  return error;
}


/**
*     @brief Calculates the ramp rate required to achieve the desired frequency,
*             based on the target voltages and hysterisis.
*     @param[in]      me - The fluid controller instance.
**/
STATIC void fluidicMixingCalculaterampSpeedVoltsPerSec(Fluidic_t* me,
                                                       float startVolts,
                                                       float endVolts)
{
  
  // Need to multiply frequency by 2 so that each movement is equal to half the
  // mixing period.
  me->pParams->rampSpeedVoltsPerSec = (float)fabsf(startVolts - endVolts) 
                                                 * 2.f * 
                                                  me->pParams->mixFrequency_Hz;
}



/**
*     @brief Processes the XMSG_FLUID_CHANNEL_MOVE_TO event.
*     @param[in]      me - The fluid controller instance.
*     @param[in]      pEv - The message contents.
*     @returns        If the move can be compelted, transition to Move state.
*                     Otherwise returns an ignored status.
**/
STATIC XState OnMsgBladderControlMoveToPos(Fluidic_t  *me, 
                                           const XEvent_t *pEv)
{
  XState retCode = X_RET_IGNORED;
  
#pragma cstat_suppress="MISRAC2012-Rule-11.3"
  const FluidicMovePositionMsg_t *pBCMsg = 
    (const FluidicMovePositionMsg_t *) pEv;
  
  if (BC_POS_HOME == pBCMsg->eTargetPos)
  {
    me->eTargetPos = pBCMsg->eTargetPos;
    me->pParams->rampSpeedVoltsPerSec = FLUID_SPEED_HIGH_DEFAULT_V_PER_S;
    me->pParams->timeout_ms = 1000u;  //1s
    me->pParams->compensationProportion = 0.f;
    me->pParams->eOvershootCompensationType = FLUID_OVERSHOOT_COMP_NONE;
    
    retCode = X_TRAN(me, &FluidicState_MoveOther);
  }
  
  else if(OK_STATUS ==  FluidCheckMoveParams(me,
                                        pBCMsg->eTargetPos,
                                        pBCMsg->rampSpeedVoltsPerSec,
                                        pBCMsg->timeout_ms,
                                        pBCMsg->eOvershootComp,
                                        pBCMsg->overshootCompProportion))
  {
    
    me->eTargetPos = pBCMsg->eTargetPos;
    me->pParams->rampSpeedVoltsPerSec = pBCMsg->rampSpeedVoltsPerSec;
    me->pParams->timeout_ms = pBCMsg->timeout_ms;
    me->pParams->compensationProportion = pBCMsg->overshootCompProportion;
    me->pParams->eOvershootCompensationType = pBCMsg->eOvershootComp;
    
    /// When we're processing the movement command, work out whether this is a forward or reverse move.
    if(me->eLastKnownPos >= me->eTargetPos)
    {
      me->status.eMoveDirection = FLUID_MOVE_REV;
    }
    
    retCode = X_TRAN(me, &FluidicState_CheckForStrip);
  }
  else
  {
    FluidicOnMoveFailMsg(me, pBCMsg->eTargetPos,ERROR_FLUID_CHANNEL_INVALID_MOVE);
  }
  
  return retCode;
}

/**
*     @brief          Processes the XMSG_FLUID_LIFT_UP_BLADDER event.
*     @param[in]      me - The fluid controller instance.
*     @param[in]      pEv - The message contents.
*     @returns        If the move can be compelted, transition to Move state.
*                     Otherwise returns an ignored status.
**/
STATIC XState OnMsgLiftUpBladders(Fluidic_t  *me,
                                           const XEvent_t *pEv)
{
  XState retCode = X_RET_IGNORED;
#pragma cstat_suppress="MISRAC2012-Rule-11.3"
  const FluidicMovePositionMsg_t *pBCMsg = (const FluidicMovePositionMsg_t *) pEv;

  if(OK_STATUS ==  FluidCheckMoveParams(me,
                                        pBCMsg->eTargetPos,
                                        pBCMsg->rampSpeedVoltsPerSec,
                                        pBCMsg->timeout_ms,
                                        pBCMsg->eOvershootComp,
                                        pBCMsg->overshootCompProportion))
  {

    me->eTargetPos = pBCMsg->eTargetPos;
    me->pParams->rampSpeedVoltsPerSec = pBCMsg->rampSpeedVoltsPerSec;
    me->pParams->timeout_ms = pBCMsg->timeout_ms;
    me->pParams->compensationProportion = pBCMsg->overshootCompProportion;
    me->pParams->eOvershootCompensationType = pBCMsg->eOvershootComp;

    /// When we're processing the movement command, work out whether this is a forward or reverse move.
    if(me->eLastKnownPos >= me->eTargetPos)
    {
      me->status.eMoveDirection = FLUID_MOVE_REV;
    }

    retCode = X_TRAN(me, &FluidicState_LiftUpBladder);
  }

  return retCode;
}

/**
*     @brief Processes the XMSG_FLUID_MIX message.
*     @param[in]      me - The fluid controller instance.
*     @param[in]      pEv - The message contents.
**/
STATIC XState OnMsgBladderControlMix(Fluidic_t  *me, 
                                     const XEvent_t *pEv)
{
#pragma cstat_suppress="MISRAC2012-Rule-11.3"
  const FluidicMixMsg_t *pBCMsg = 
    (const FluidicMixMsg_t *) pEv;
  
  XState retCode = X_RET_IGNORED;
  
  if(OK_STATUS ==  FluidCheckMixParams(me, pBCMsg->eTargetPos, 
                                       pBCMsg->mixFrequency,
                                       pBCMsg->mixTime,
                                       pBCMsg->mixCycles,
                                       pBCMsg->eMixType,
                                       pBCMsg->openLoopCompensationFactor,
                                       pBCMsg->mixDownstrokeProportion))
  {
    
    //All params are okay. Store them.
    me->pParams->mixFrequency_Hz = pBCMsg->mixFrequency;
    me->eTargetPos = pBCMsg->eTargetPos;
    me->pParams->mixTimeout_ms = pBCMsg->mixTime;
    me->pParams->targetMixCycles = pBCMsg->mixCycles;
    me->pParams->eMixType = pBCMsg->eMixType;
    me->pParams->mixDownstrokeProportion = pBCMsg->mixDownstrokeProportion;
    me->pParams->openLoopCompensationFactor = pBCMsg->openLoopCompensationFactor;
    
    // Always start by moving updards.
    me->status.eMoveDirection = FLUID_MOVE_REV;
    
    me->pParams->eMixEndPosition = me->eLastKnownPos;
    
    me->status.mixingStagesCompleted = 0;
    
    me->mixTimer = 0;
    
    if(me->pParams->eMixType == FLUID_MIX_DUAL_POINT_LOOP)
    {
      retCode =  X_TRAN(me, &FluidicState_MixContactControlled);
    }
    // Reverse move of other types is Piezo controlled.
    else
    {
      retCode =  X_TRAN(me, &FluidicState_MixPiezoControlled);
    }
  }
  
  return retCode;
  
}



/**
*     @brief Processes the XMSG_FLUID_CHANNEL_NEW_PARAMS message.
*     @param[in]       me - The fluid controller instance.
*     @param[in]       pEv - The message contents.
**/
STATIC void OnMsgUpdateParams(Fluidic_t *me, XEvent_t* pEv)
{
  FluidicUpdateParamsMsg_t* pUpdateEv = (FluidicUpdateParamsMsg_t*)pEv;
  
  me->pParams->positionLimits[BC_POS_FLUID_A].targetVolts = pUpdateEv->flAVal;
  me->pParams->positionLimits[BC_POS_FLUID_B].targetVolts = pUpdateEv->flBVal;
  me->pParams->positionLimits[BC_POS_FLUID_C].targetVolts = pUpdateEv->flCVal;
}



/**
*     @brief Logs a completed move. Sends a message to the master object to notify
*            the completion.
*     @param[in] me - The fluidic controller object.
**/
STATIC void FluidicOnMoveCompleteMsg(Fluidic_t *me)
{
  FluidicSetCurrentAndTargetPositions(me, me->eTargetPos, BC_NONE);
  
  me->moveSuccessMsg.eChannel = me->pParams->eChannel;
  me->moveSuccessMsg.eRestPosition = me->eLastKnownPos;
  me->moveSuccessMsg.completionTimeMs = me->timeoutTimer; 
  me->moveSuccessMsg.piezoVolts = piezoVoltageGet(me->pPiezo);
  X_PUBLISH(X_FRAMEWORK_OF(me), me->moveSuccessMsg);        // Always publish move success message.
  
  ///
  /// Logging for debugging purposes
  ///
  LOG_TRACE("PV = %.1f POS = %u",
      me->pPiezo->currentVoltage,
      me->status.eFluidFrontPosition);
  
  me->publishCompletionEvent = true;                        // Always reset publication rights
}



/**
*     @brief Logs a failed move. Sends a message to the master object to notify
*            the failure.
*     @param[in] me - The fluidic controller object.
*     @param[in] pos - The targeted position.
**/
STATIC void FluidicOnMoveFailMsg(Fluidic_t *me, eFluidicPositions_t pos, eErrorCode eError)
{
  me->moveFailMsg.eChannel = me->pParams->eChannel;
  me->moveFailMsg.eTargetPosition = pos;

  X_PUBLISH(X_FRAMEWORK_OF(me), me->moveFailMsg);

  
  ///
  /// Logging for debugging purposes
  ///
  LOG_TRACE("PV = %.1f POS = %u",
      me->pPiezo->currentVoltage,
      me->status.eFluidFrontPosition);
  
  me->cmdFail.eError = eError;       // Append the error to the fail message.
  X_PUBLISH(X_FRAMEWORK_OF(me), me->cmdFail);
}


/**
*     @brief Actions when mixing is completed.
*     @param[in] me - The fluidic controller object.
**/
STATIC void FluidicOnMixComplete(Fluidic_t* me)
{
  me->mixTimer = 0u;    // Reset the timer, otherwise we'll just end up sending this a lot!
  me->status.mixComplete = true;
  
  //Save our current position (the target), and begin a move to the end position.
  FluidicSetCurrentAndTargetPositions(me,  me->eTargetPos, me->pParams->eMixEndPosition);
  
  //Prepare and send the event.
  me->mixCmpltMsg.eChannel = me->pParams->eChannel;
  me->mixCmpltMsg.eRestPosition = me->pParams->eMixEndPosition;
  X_PUBLISH(X_FRAMEWORK_OF(me), me->mixCmpltMsg);
}

/**
* @}
**/


/**
*     @brief Initiates a move to any other fluid position.
*     @param[in]      me - The fluid controller instance.
*     @param[in]      eTarget - The target position
*     @param[in]      erampSpeedVoltsPerSec - The abstract ramp speed which should be 
*                                  used during the movement.
*     @retval         OK_STATUS The movement parameters are okay to use.
*     @retval         ERROR_FLUID_CHANNEL_INVALID_MOVE - The parameters for
*                     mixing are invalid, movement should be aborted.
*     @retval         ERROR_FLUID_SPEED - Attempted to use the mixing speed,
*                     an invalid speed for the move.
*     @note           The movement parameters are not stored at this point!                
**/
STATIC eErrorCode FluidCheckMoveParams(Fluidic_t *me,
                                       eFluidicPositions_t eTarget,
                                       float rampSpeedVoltsPerSec,
                                       uint32_t timeout_ms,
                                       eFluidOvershootCompensation_t   eOvershootCompMode,
                                       float compensationProportion)
{
  bool moveValid =  bladderControlCheckMoveValid(me, eTarget);
  eErrorCode error = OK_STATUS;
  
  
  if(moveValid)
  {
    if(rampSpeedVoltsPerSec > PIEZO_RAMP_MAX)
    {
      error = ERROR_FLUID_SPEED;
    }
    
    if(eOvershootCompMode >= FLUID_OVERSHOOT_COMP_NUM)
    {
      error = ERROR_BAD_ARGS;
    }
    
    if(compensationProportion > FLUIDIC_MAX_COMPENSATION_FACTOR)
    {
      error = ERROR_BAD_ARGS;
    }
    
  }
  else
  {
    error = ERROR_FLUID_CHANNEL_INVALID_MOVE;
  }
  
  return error;
}


/**
*     @brief Initiates a move to any other fluid position.
*     @param[in]      me - The fluid controller instance.
*     @param[in]      eTarget - The target position
*     @param[in]      mixFrequency_Hz - The frequency the mix should occur at.
*     @param[in]      mixTimeout_ms - The mixing period, in ms.
*     @retval         OK_STATUS The mixing parameters are okay to use.
*     @retval         ERROR_FLUID_CHANNEL_INVALID_MOVE - The parameters for
*                     mixing are invalid, mixing should be aborted.
**/
STATIC eErrorCode FluidCheckMixParams(Fluidic_t *me,
                                      eFluidicPositions_t eTarget,
                                      float mixFrequency_Hz,
                                      uint32_t mixTimeout_ms,
                                      uint32_t numCycles,
                                      eFluidMixingType_t  eMixType,
                                      float openLoopCompensationFactor,
                                      float mixDownstrokeProportion)
{
  eErrorCode error = ERROR_BAD_ARGS;
  
  bool freqOk = isFrequencyOk(me, eTarget, mixFrequency_Hz);
  
  bool posOk = isMixPositionOk(me, eTarget);
  
  bool timeoutOk = isMixTimeoutOk(me, mixTimeout_ms);
  
  if(freqOk && posOk && timeoutOk)
  {   
    if(eMixType != FLUID_MIX_DUAL_POINT_LOOP)   // Dual point loop doesn't need a "downstroke proportion".
    {
      if(mixDownstrokeProportion > 0.f)
      {
        error = OK_STATUS;
      }
    }
    else
    {
      error = OK_STATUS;                        // So long as other parameters are OK, and mode is not FLUID_MIX_DUAL_POINT_LOOP.
                                                // We're good to go!
    }
  }
  
  // ERROR_FLUID_CHANNEL_INVALID_MOVE is reported exclusively if the position isn't valid for a command set.
  if (false == posOk)
  {
    error = ERROR_FLUID_CHANNEL_INVALID_MOVE;
  }
  
  return error;
}

/**
*     @brief  Checks whether the mixing frequency specified in the mix command 
*             is suitable.
*     @details        The ramp rate is determined from the frequency and the
*                     mixing end stops. If the ramp rate exceeds the maximum ramp
*                     rate of the piezo's maximum ramp rate.
*     @param[in]      me - The fluid controller instance.
*     @param[in]      eTargetPos - The desired movement positionl.
*     @param[in]      mixFrequency_Hz - Mixing frequency in Hz.
*     @retval         true - The timeout value is okay to use.
*     @retval         false - Movement should not be completed.
**/
STATIC bool isFrequencyOk(Fluidic_t* me, 
                          eFluidicPositions_t eTargetPos,
                          float mixFrequency_Hz)
{
  bool isOk;
  eFluidicPositions_t eLastPos = me->eLastKnownPos;
  float maxRampRate = me->pPiezo->pParams->maxRampRate;
  
  float targetPosVoltage = me->pParams->positionLimits[eTargetPos].targetVolts;
  float currentPosVoltage = me->pParams->positionLimits[eLastPos].targetVolts;
  
  float rampRateForMix = fabsf(currentPosVoltage - targetPosVoltage)*mixFrequency_Hz;
  
  if(rampRateForMix >= maxRampRate)
  {
    isOk = false;
  }
  else if(mixFrequency_Hz == 0.f)
  {
    isOk = false;
  }
  else
  {
    isOk = true;
  }
  
  return isOk;
}


/**
*     @brief  Checks whether the timeout specified in the mix command is suitable.
*     @param[in]      me - The fluid controller instance.
*     @param[in]      mixTimeout_ms - The desired movement positionl.
*     @retval         true - The timeout value is okay to use.
*     @retval         false - Movement should not be completed.
**/
STATIC bool isMixTimeoutOk(Fluidic_t* me, uint32_t mixTimeout_ms)
{
  bool isOk;
  
  //Should check for erronious commands. Default limit is 60-minutes.
  if(mixTimeout_ms > me->pParams->mixTimeoutMax_ms)
  {
    isOk = false;
  }
  else if(mixTimeout_ms == 0u)
  {
    isOk = false;
  }
  else
  {
    isOk = true;
  }
  
  return isOk;
}


/**
*     @brief  Checks whether the fluidic channel can mix between the ranges specified
*     @details A mix must be performed between the current position and a position
*              earlier in the strip. And must not be performed between HOME.
*     @param[in]      me - The fluid controller instance.
*     @param[in]      eTargetPos - The desired movement positionl.
*     @retval         true - The mix position is okay to use, mixing can be performed.
*     @retval         false - Movement should not be completed.
**/
STATIC bool isMixPositionOk(Fluidic_t* me, eFluidicPositions_t eTargetPos)
{
  bool isOk;
  eFluidicPositions_t eLastPosition = me->eLastKnownPos;
  
  //Can only mix with a position which is lower.
  if((eTargetPos >= eLastPosition) || (BC_POS_HOME == eTargetPos))
  {
    isOk = false;
  }
  else if((BC_POS_UNKNOWN == eTargetPos) || (BC_NONE == eTargetPos))
  {
    isOk = false;
  }
  else
  {
    isOk = true;
  }
  
  return isOk;
}



/**
*     @brief  Checks whether the requested movement can be completed.
*     @details Movements can only be completed if the controller is in idle, or
*             the intended movement is to return to home. Then check whether
*             it is possible to move to the desired position.
*     @param[in]      me - The fluid controller instance.
*     @param[in]      eTargetPos - The desired movement positionl.
*     @retval         true - Movement can be performed.
*     @retval         false - Movement cannot be completed.
**/
STATIC bool bladderControlCheckMoveValid(
                                         Fluidic_t *me,
                                         eFluidicPositions_t eTargetPos)
{
  bool retState;
  
  eFluidicPositions_t eCurrentPos   =  me->eLastKnownPos;
  
  if(BC_POS_HOME == eTargetPos)
  {
    retState = true;
  }
  else 
  {
    retState = bladderControlCheckValidPosChange(eCurrentPos, eTargetPos);
  }
  
  
  return retState;
}



/**
*     @brief          Checks whether it is possible to move from the current
*                     position to the desired position.
*     @details        Allowable position changes
*                     
*                     Current Position       | Allowable Positions
*                     -----------------------|---------------------
*                     HOME                   | Bladder Down, Home    
*                     -----------------------|---------------------
*                     Bladder Down           | All
*                     -----------------------|---------------------
*                     Fluid A B C            | All
*                     -----------------------|---------------------
*                     Unknown                | Home
*
*     @param[in]      eCurrentPos - The fluid controller's current position.
*     @param[in]      eTargetPos - The desired movement positionl.
*     @retval          true - Movement can be performed.
*     @retval          false - Movement cannot be completed.
**/
STATIC bool bladderControlCheckValidPosChange(eFluidicPositions_t eCurrentPos ,
                                              eFluidicPositions_t eTargetPos)
{
  bool retState;
  
  if((eCurrentPos == BC_NONE) || (eCurrentPos == BC_POS_UNKNOWN))
  {
    ///
    /// Currently have a MISRA warning that statement always true.
    /// This is meant to be a check against command line parameters.
    /// 
    if(eTargetPos != BC_POS_HOME)
    {
      retState = false;
    }
    else
    {
      retState = true;
    }
  }
  else if(eCurrentPos == BC_POS_HOME)
  {
    if((eTargetPos == BC_POS_DOWN) || (eTargetPos == BC_POS_HOME))
    {
      retState = true;
    }
    else
    {
      retState = false;
    }
  }
  else
  {
    if((eTargetPos != BC_POS_UNKNOWN) && (eTargetPos != BC_NONE))
    {
      retState = true;
    }
    else
    {
      retState = false;
    }
  }
  
  return retState;
}


/**
*     @brief  Stops piezo movement and updates the fluidic status.
*     @param[in]  me - The fluid controller instance.
*     @returns The error code from piezoStop.
**/
STATIC eErrorCode FluidicStopMove(Fluidic_t* me)
{  
  return piezoStop(me->pPiezo);  //Stop where we are. Just in case the piezo was still moving.
}



/**
*     @brief  Starts a Piezo homing move, and updates the Fluidic object's status.
*     @param[in]  me - The fluid controller instance.
*     @returns The error code from piezoHome.
**/
STATIC eErrorCode FluidicHomeMoveBegin(Fluidic_t* me)
{
	/// When homing, reset the target positions for Fluid A -> C.
  me->pParams->positionLimits[BC_POS_FLUID_A].targetVolts = FLUIDIC_MAX_VOLTS_BEFORE_LIFT;
  me->pParams->positionLimits[BC_POS_FLUID_B].targetVolts = FLUIDIC_MAX_VOLTS_BEFORE_LIFT;
  me->pParams->positionLimits[BC_POS_FLUID_C].targetVolts = FLUIDIC_MAX_VOLTS_BEFORE_LIFT;
  
  return piezoHome(me->pPiezo);
}



/**
*     @brief  Helper to update the current and target positions of the fluidic
*             object.
*     @param[in]  me - The fluid controller instance.
*     @param[in] eCurrent - The current position of the fluidic controller. (To be stored)
*     @param[in] eTarget - Target position of the fluidic controller to be stored.
**/   
STATIC void FluidicSetCurrentAndTargetPositions(Fluidic_t* me, eFluidicPositions_t eCurrent,
                                                eFluidicPositions_t eTarget)
{
  me->eLastKnownPos = eCurrent;
  me->eTargetPos = eTarget;
}


/**
*     @brief  Used to modify the hyseteris voltage of the target position,
*             and will recalculate the ramp speed needed for the mixing.
*     @note Hysteris is limited to 1V <= Hysetrisis <= 10V
*     @param[in]  me - Pointer to the hysterisis voltage object.
*     @param[in] multiplierType - The multiplication factor for the hysterisis voltage.
**/ 
STATIC void AdjustHysterisisVoltage(Fluidic_t* me, 
                                    eFluidicHysterisisChangeType_t multiplierType)
{
  ASSERT_NOT_NULL(me);
  ASSERT(multiplierType != FLUID_HYST_COUNT);
  
  float hystVoltage = me->pParams->positionLimits[me->eTargetPos].posHysterisis;
  hystVoltage *= me->pParams->hysterisisMultipliersVolts[multiplierType];
  
  if(FLUIDIC_HYSETRISIS_MAX < hystVoltage)
  {
    hystVoltage = FLUIDIC_HYSETRISIS_MAX;
  }
  else if(FLUIDIC_HYSETRISIS_MIN > hystVoltage)
  {
    hystVoltage = FLUIDIC_HYSETRISIS_MIN;
  }
  else
  {
    //Do nothing.
  }
  
  me->pParams->positionLimits[me->eTargetPos].posHysterisis = hystVoltage;
}




/**
*     @brief  Sets the error code of the fluidic controller.
*     @note   An error code of OK_STATUS will not clear error codes.
*     @note   The function checks the movement failure count. If this has 
*             exceeded the limit then a critical error occurs.
*     @param[in]  me - The fluid controller instance.
*     @param[in] err - The new error code. A value of eErrorCode
*     @param[in] retCode - The current state response code.
*     @returns State process code. If a critical error occurs then the object
*              transititions to the error state.
**/
STATIC XState FluidicsErrorSet(Fluidic_t *me, eErrorCode err, XState retCode)
{
   
  if(OK_STATUS != err)
  {
    ERROR_CHECK(err);
    
    // DO NOT enter error state if already transitioning to error state!
    if(FLUIDIC_CIRITICAL_ERR((err)) && (false == XFSM_IS_STATE(me, &FluidicState_Err)))
    {
      retCode = X_TRAN_ERROR(me, &FluidicState_Err, err);
    }
  }
  
  return retCode;
}


/**
  * @brief Helper to check if the Fluid controller is in an appropriate state to
  *        accept a movement command
  * @param[in] me - The fluid controller
  * @returns True if a command can be accepted.
  **/
STATIC bool  FluidicStateCanAcceptCommand(Fluidic_t * me)
{
  bool canAcceptCommand = false;
  
  if(XFSM_IS_STATE(me, &FluidicState_Idle)
         ||
            XFSM_IS_STATE(me, &FluidicState_MonitorFluidBreach))
  {
    canAcceptCommand = true;
  }
  
  return canAcceptCommand;
}

/**
  * @brief Helper call which monitors the status of bladders
  *        based on feedback messages coming from bladder
  *        detection engine in electrochemistry object.
  *
  * @param[in] me - The fluid controller
  * @param[in] eventId - message per channel.
  * @returns   eErrorCode.
  **/
STATIC eErrorCode FluidicMonitorBladderDetection(Fluidic_t * me, eXEventId eventId)
{
	eErrorCode error = OK_STATUS;

	if (!me->chTargetPosReached)
	{
		if (BC_POS_DOWN == me->eTargetPos)
		{
			switch(eventId)
			{
			case XMSG_EC_A1_BLDR_DOWN:
				if (EC_STRIP_CHAN_1 == me->pParams->eChannel)
				{
					error = piezoStop(me->pPiezo);
					me->chTargetPosReached = true;
				}
        break;
        
			case XMSG_EC_B2_BLDR_DOWN:
				if (EC_STRIP_CHAN_2 == me->pParams->eChannel)
				{
					error = piezoStop(me->pPiezo);
					me->chTargetPosReached = true;
				}
        break;
        
      case XMSG_EC_A3_BLDR_DOWN:
				if (EC_STRIP_CHAN_3 == me->pParams->eChannel)
				{
					error = piezoStop(me->pPiezo);
					me->chTargetPosReached = true;
				}
        break;
        
			case XMSG_EC_B4_BLDR_DOWN:
				if (EC_STRIP_CHAN_4 == me->pParams->eChannel)
				{
					error = piezoStop(me->pPiezo);
					me->chTargetPosReached = true;
				}
        break;
        
			case XMSG_EC_A1_BLDR_UP:
			case XMSG_EC_B2_BLDR_UP:
			case XMSG_EC_A3_BLDR_UP:
			case XMSG_EC_B4_BLDR_UP:
				/* Not handled here. */
        break;
        
			default:
				error = ERROR_FLUIDIC_UNKNOWN_MSG_FROM_EC;
        break;
			}
		}
		else
		{
      switch(eventId)
      {
      case XMSG_EC_A1_BLDR_UP:
        if (EC_STRIP_CHAN_1 == me->pParams->eChannel)
        {
          error = piezoStop(me->pPiezo);
          me->chTargetPosReached = true;
        }
        break;
        
      case XMSG_EC_B2_BLDR_UP:
        if (EC_STRIP_CHAN_2 == me->pParams->eChannel)
        {
          error = piezoStop(me->pPiezo);
          me->chTargetPosReached = true;
        }
        break;
        
      case XMSG_EC_A3_BLDR_UP:
        if (EC_STRIP_CHAN_3 == me->pParams->eChannel)
        {
          error = piezoStop(me->pPiezo);
          me->chTargetPosReached = true;
        }
        break;
        
      case XMSG_EC_B4_BLDR_UP:
        if (EC_STRIP_CHAN_4 == me->pParams->eChannel)
        {
          error = piezoStop(me->pPiezo);
          me->chTargetPosReached = true;
        }
        break;
        
      case XMSG_EC_A1_BLDR_DOWN:
      case XMSG_EC_B2_BLDR_DOWN:
      case XMSG_EC_A3_BLDR_DOWN:
      case XMSG_EC_B4_BLDR_DOWN:
        /* Not handled here. */
        break;
        
      default:
        error = ERROR_FLUIDIC_UNKNOWN_MSG_FROM_EC;
        break;
      }
		}
	}

	return error;
}


/**
  * @brief Helper function to convert between Fluidic positions (home, down, etc)
  *        to an echem position type
  * @param[in] eFluidPos - The fluid postion to convert
  * @returns eElectrochemicalChannelPos
  **/
STATIC eElectrochemicalChannelPos ConvertFluidPosToEchemPos(eFluidicPositions_t eFluidPos)
{
  eElectrochemicalChannelPos eEchemPos;
  
  switch (eFluidPos)
  {
  case BC_POS_FLUID_A:
    eEchemPos = EC_CHAN_POS_A;
    break;
    
  case BC_POS_FLUID_B:
    eEchemPos = EC_CHAN_POS_B;
    break;
    
  case BC_POS_FLUID_C:
    eEchemPos = EC_CHAN_POS_C;
    break;
    
    
  case BC_POS_DOWN: 
  case BC_POS_HOME:
  default:
    eEchemPos = EC_CHAN_POS_NONE;
    break;  
  }
  
  return eEchemPos;
}



/**
  * @brief Helper function to handle a wait for fluid at contact message.
  * @param[in] me - The fluid controller
  * @param[in] pEv - The event.
  * @returns Transition to wait for contact state.
  **/
STATIC XState OnMsgWaitForFluidAtContact(Fluidic_t * me, const XEvent_t *pEv)
{
  const FluidicWaitForFluidAtContactMsg_t * pWaitMsg = (const FluidicWaitForFluidAtContactMsg_t *)pEv;
  
  me->eTargetPos = pWaitMsg->eTargetPos;
  me->pParams->timeout_ms = pWaitMsg->timeoutMs;
  
  return X_TRAN(me, &FluidicState_WaitForContact);
}


/**
* @}
*/
/**
* @}
*/


/********************************** End Of File ******************************/
