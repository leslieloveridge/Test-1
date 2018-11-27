/**
 ******************************************************************************
 * @file   fluidics.h 
 * @brief  Header file for fluidics.c 
 ******************************************************************************
 */


#ifndef FLUIDICS_H_
#define FLUIDICS_H_
   
#include "poci.h"
#include "electrochemical.h"
#include "piezo.h"
#include "fluidicsTypes.h"



///Max timeout period is one hour. Should allow for all chemistry systems.
/// @note This is an arbitrary value, needs to be specified by Lumira.
#define FLUIDIC_MAX_MIX_TIMEOUT_DEFAULT_MS (uint32_t)(3600u*1000u) 

#define FLUID_NUM_MIXING_STAGES_PER_CYCLE 2u

// Low speed. 2.5V/s change. This allows move to be completed in 3- 4s
#define FLUID_SPEED_LOW_DEFAULT_V_PER_S   2.5f

// High speed moves @ Maximumm transition rate.
#define FLUID_SPEED_HIGH_DEFAULT_V_PER_S  PIEZO_RAMP_MAX

// Flush speed needs to be faster than slow speed. Used for bead band wash.
#define FLUID_SPEED_FLUSH_DEFAULT_V_PER_S 10.f

// Default Hysterisis multipliers 10% change.
#define FLUID_HYST_MULTIPLIER_INC_DEFAULT 1.1f
#define FLUID_HYST_MULTIPLIER_DEC_DEFAULT 0.9f

// By default all channels set to move until Piezo reaches 50V. (Prevents lifting off bladders)
// This will be overwritten when movement is completed.
#define FLUIDIC_DEFAULT_TARGET_POSITION 60.f

// By default Hysterisis is 5V about target. Used for mixing.
// However contact A requires a much higher Hysterisis (as compression of bladders reduces at lower voltages)
#define FLUIDIC_DEFAULT_HYSTERISIS_V  5.f
#define FLUIDIC_POS_A_HYSTERISIS_V    10.f
#define FLUIDIC_HYSTERISIS_NONE       0.f

// Default timeout for movement is 30s.
#define FLUIDIC_DEFAULT_TIMEOUT_30S (uint32_t)(1000u*30u)

// Default mixing frequency 1Hz.
#define FLUIDIC_DEFAULT_MIX_FREQ    1.f


#define FLUIDIC_HYSETRISIS_MAX   10.f
#define FLUIDIC_HYSETRISIS_MIN   1.f


/// Critical errors which impact Fluidic Objects.
#define FLUIDIC_CIRITICAL_ERR(errorCode) ((errorCode == ERROR_FLUID_CHANNEL_ECHEM_BUSY || \
                                              errorCode == ERROR_FLUID_CHANNEL_FLUID_FRONT) \
                                               || (errorCode == ERROR_PIEZO_UNKNOWN)    \
                                                 || (errorCode == ERROR_FLUIDIC_ERR_CNT))

#define FLUIDIC_MAX_VOLTS_BEFORE_LIFT   50.f


/// Tick count of Fluidic objects.
#define FLUIDIC_TIMER_COUNT_MS        (uint32_t) 20

///Maximum error occurrence. Stops the system bouncing between points in the event of a movement failure.
#define FLUIDIC_MAX_FAIL_COUNT    (uint32_t) 2u

#define FLUIDIC_CONVERT_S_TO_MS(t_)   (float)((t_) * 1000.f)

#define FLUIDIC_MAX_COMPENSATION_FACTOR 1.f

#define FLUID_RETURN_SPEED_REDUCTION_FACTOR 2.f

/// Events which Fluidic objects must subscribe to.
#define X_SUBSCRIBE_TO_FLUIDIC_EVENTS(me_) \
		X_SUBSCRIBE(me_, XMSG_FLUID_CHANNEL_MOVE_TO) \
		X_SUBSCRIBE(me_, XMSG_FLUID_CHANNEL_CANCEL) \
		X_SUBSCRIBE(me_, XMSG_FLUID_CHANNEL_DEBUG) \
		X_SUBSCRIBE(me_, XMSG_FLUID_CHANNEL_NEW_PARAMS) \
        X_SUBSCRIBE(me_,XMSG_FLUID_MIX) \
		X_SUBSCRIBE(me_, XMSG_FLUID_ERR_CLEAR) \
		X_SUBSCRIBE(me_, XMSG_EC_FLUID_STATUS_CHANGED) \
		X_SUBSCRIBE(me_, XMSG_EC_ERROR) \
		X_SUBSCRIBE(me_, XMSG_PIEZO_MOVE_COMPLTE) \
		X_SUBSCRIBE(me_, XMSG_PEIZO_MOVE_FAIL) \
		X_SUBSCRIBE(me_, XMSG_PIEZO_STOPPED) \
		X_SUBSCRIBE(me_, XMSG_DOOR_OPENED) \
		X_SUBSCRIBE(me_, XMSG_FLUID_MIX_CONTINUE) \
		X_SUBSCRIBE(me_, XMSG_EC_A1_BLDR_DOWN) \
		X_SUBSCRIBE(me_, XMSG_EC_A1_BLDR_UP) \
		X_SUBSCRIBE(me_, XMSG_EC_A3_BLDR_DOWN) \
		X_SUBSCRIBE(me_, XMSG_EC_A3_BLDR_UP) \
		X_SUBSCRIBE(me_, XMSG_EC_B2_BLDR_DOWN) \
		X_SUBSCRIBE(me_, XMSG_EC_B2_BLDR_UP) \
		X_SUBSCRIBE(me_, XMSG_EC_B4_BLDR_DOWN) \
		X_SUBSCRIBE(me_, XMSG_EC_B4_BLDR_UP) \
		X_SUBSCRIBE_TO_GLOBAL_EVENTS(me)


/**
 * @defgroup Fluidics Fluidics Control
 * @brief Responsible for control of fluid position in a single channel within the strip.
 * @details Uses an instance of the Piezo bender interface, and the electrochemical interface
 *         to move fluid between strip contact electrodes, or mix fluid between the electrodes.
 *  @{
 */


/**
  *     @brief  Movement limits used to indicate the allowable position range
  *             of a given position.
  **/
typedef struct FluidicPositionLimits_tag
{
  float targetVolts;                          ///< The expected voltage for the position.
  float posHysterisis;                        ///< Look at what can be done to remove this.
  eEcFluidDetectPosition_t echemRequirements[FLUID_MOVE_COUNT];  ///< Array of the contact requirements when moving forward, or in reverse.
}
FluidicPositionLimits_t;



/**
  *     @brief Parameters used by a fluidics instance.
  */
typedef struct FluidicParams_tag
{
  FluidicPositionLimits_t        positionLimits[BC_VALID_POS_COUNT];    ///< The piezo voltages which will be used to define each fluid position.
  
  eElectrochemicalChannel        eChannel;                      ///< Fluid channel which the bladder controls.
  uint32_t                       timeout_ms;                    ///< Timeout whilst waiting for fill detection to occur.
  float                          mixFrequency_Hz;               ///< The default mixing frequency. This is changed when a new mix command is received.
  uint32_t                       mixTimeout_ms;                 ///< The default mixing timeout. This is changed when a new mix command is received. 
  uint32_t                       targetMixCycles;                     ///< The number of complete cycles which must be completed during the mix operation.
  float                          rampSpeedVoltsPerSec;                    ///< Can be @ref FLUIDIC_SPEED_LOW, @ref FLUIDIC_SPEED_HIGH, or @ref FLUIDIC_SPEED_FLUSH
  uint32_t                       mixTimeoutMax_ms;              ///< Maximum mix timeout which can be used.
  eFluidicPositions_t            eMixEndPosition;                ///< End position when mixing.
  float                          hysterisisMultipliersVolts[FLUID_HYST_COUNT];
  eFluidOvershootCompensation_t  eOvershootCompensationType;
  float                          compensationProportion;        ///< 0.0 - 1.0, proportion of the difference in Piezo voltage applied to reach the current contact (and it's previous).
  float                          returnSpeedRedcutionFactor;
  
  eFluidMixingType_t             eMixType;
  float                          openLoopCompensationFactor;
  float                          mixDownstrokeProportion;
  
  bool                           monitorBreachAfterMove;      ///< Boolean flag to monitor the contacts for breach after completing the move.
}
FluidicParams_t;



/**
  *     @brief Initialisation parameters for a fluidics instance.
  */
typedef struct FluidicInitParams_tag
{
  piezo_t*                      pPiezo;              ///< The piezo object
  Electrochemical_t*            pEchem;               ///< The electrochemical object.
                                                      
  FluidicParams_t*              pParams;             ///< The initialisation parameters.
                                                      
  char*                         name;                 ///< The name which will be stored within the XObj base.
  uint8_t                       prio;                 ///< Priority.
}
FluidicInitParams_t;


/**
  *     @brief  Message used to instruct the fluidics which position 
  *             to move to.
  */
typedef struct FluidicMovePositionMsg_tag
{
  XEvent_t                        super;                  ///< Base XEvent object
  eFluidicPositions_t             eTargetPos;             ///< The target movement position
  float                           rampSpeedVoltsPerSec;   ///< The rate of change of piezo voltage which is used.
  eFluidOvershootCompensation_t   eOvershootComp;
  float                           overshootCompProportion;
  uint32_t                        timeout_ms;
}
FluidicMovePositionMsg_t;
  
/**
  *     @brief  Message used to instruct the lift up of bladders.
  */
typedef struct FluidicLiftUpBladderMsg_tag
{
  XEvent_t                        super;                  ///< Base XEvent object
  eFluidicPositions_t             eTargetPos;             ///< The target movement position
  float                           rampSpeedVoltsPerSec;   ///< The rate of change of piezo voltage which is used.
  eFluidOvershootCompensation_t   eOvershootComp;
  float                           overshootCompProportion;
  uint32_t                        timeout_ms;
}
FluidicLiftUpBladderMsg_t;

/**
  *     @brief  Message used to instruct the fluidics to mix between the current
  *             and target positions.
  **/
typedef struct FluidicMixMsg_tag
{
  XEvent_t      super;                            ///< Base XEvent object
  eFluidicPositions_t eTargetPos;                 ///< The mix/oscillation limit
  float               mixFrequency;               ///< The movement frequency.
  uint32_t            mixTime;                    ///< The time that mixing should be completed by.
  uint32_t            mixCycles;                  ///< The number of complete cycles which must be completed during the mix operation.
  eFluidMixingType_t  eMixType;                   ///< The type of mixing to be performed - Open loop, Single point closed loop, Dual point closed loop.
  float               openLoopCompensationFactor; ///< Compensation factor to account for observed additional movement of fluid during first stroke of mixing in open loop.
  float               mixDownstrokeProportion;    ///< Proportion of the channel region to be mixed in. (The proportion of Piezo voltage between contacts to be used)
}
FluidicMixMsg_t;



/**
  *     @brief  Message used to update fluidics configuration params.
  */
typedef struct FluidicUpdateParamsMsg_tag
{
  XEvent_t super;  ///< Base XEvent object
  float flAVal;    ///< Piezo driver voltage for fluid position A
  float flBVal;    ///< Piezo driver voltage for fluid position B                
  float flCVal;    ///< Piezo driver voltage for fluid position C
}
FluidicUpdateParamsMsg_t;



/**
  *     @brief  Message used to indicate that a move was completed
  */
typedef struct FluidicMoveSuccessMsg_tag
{
  XEvent_t                      super;        ///< Base XEvent object
  eFluidicPositions_t           eRestPosition; ///< Position at the end of the fluidic movement.
  eElectrochemicalChannel       eChannel;     ///< Electrochemical channel - Indicates which fluid channel has concluded moving.
  uint32_t                      completionTimeMs; ///< Time taken to complete the movement, in ms.
  float                         piezoVolts;
}
FluidicMoveSuccessMsg_t;


/**
  *     @brief  Message used to indicate that a move failed
  */
typedef struct FluidicMoveFailMsg_tag
{
  XEvent_t                      super;          ///< Base XEvent object
  eFluidicPositions_t           eTargetPosition; ///< Position at the end of the fluidic movement.
  eElectrochemicalChannel       eChannel;       ///< Electrochemical channel - Indicates which fluid channel has concluded moving.
}
FluidicMoveFailMsg_t;


/**
  *     @brief  Message used to indicate that the mixing request has completed
  */
typedef struct FluidicMixCompleteMsg_tag
{
  XEvent_t                      super;        ///< Base XEvent object
  eFluidicPositions_t           eRestPosition;///< Position at the end of the fluidic movement.
  eElectrochemicalChannel       eChannel;     ///< Electrochemical channel - Indicates which fluid channel has concluded moving.
}
FluidicMixCompleteMsg_t;

/**
  *@brief FluidicMixStageCompleteMsg_tag
  *@details Information in an error message.
  **/
typedef struct FluidicMixStageCompleteMsg_tag
{
  XEvent_t                      super;        ///< Base XEvent object
  eElectrochemicalChannel       eChannel;     ///< Electrochemical channel - Indicates which fluid channel has concluded moving.
}
FluidicMixStageCompleteMsg_t;


/**
  *@brief Message to enable / disable breach detection in the fluid controller.
  **/
typedef struct FluidicMonitorBreachMsg_tag
{
  XEvent_t                      super;                 ///< Base XEvent object
  bool                          monitorFluidPosition;  ///< Boolean flag to monitor the fluid position after
}
FluidicMonitorBreachMsg_t;


/**
  *     @brief Information in an error message.
  **/
typedef struct FluidicErrorMsg_tag
{
  XEvent_t                      super;     ///< Base XEvent object
  eErrorCode                    errorCode; ///< Error code which caused the message.
}
FluidicErrorMsg_t;




typedef struct FluidicWaitForFluidAtContactMsg_tag
{
  XEvent_t                      super;     ///< Base XEvent object
  uint32_t                      timeoutMs;
  eFluidicPositions_t           eTargetPos; 
}
FluidicWaitForFluidAtContactMsg_t;


/**
  *     @brief Status information of a Fluidic Object
  **/
typedef struct FluididStatus_tag
{
  eEcFluidDetectPosition_t      eFluidFrontPosition;      ///< Last recorded position of the fluid front in that fluidic channel
  float                         piezoVoltage;             ///< Voltage applied to the Piezo object when a move was completed, or stopped.
  bool                          mixComplete;              ///< Flag set by the fluidic object when mixing is complete, and the channel is waiting for the Piezo to stop.
  uint32_t                      mixingStagesCompleted;    ///< The number of stages (movements) completed during the mixing operation.
  eFluidicMoveDirection_t       eMoveDirection;           ///< Current direction of fluid movement.
}
FluididStatus_t;


/**
* @brief  A class wrapper for the Spectro Scan controller
*/
typedef struct Fluidic_tag
{   
  XActive_t                     super;             ///< Active framework top-level object
  XTimer_t                      timer;             ///< Fluidic object's timer.
  uint32_t                      evQueueBytes[64];  ///< Data queue for events.
  

  //Need a piezo object.
  piezo_t                       *pPiezo;           ///< Pointer to the Piezo object.
  Electrochemical_t             *pEchem;           ///< Pointer to the electrochemistry system.
  
  eFluidicPositions_t           eLastKnownPos;     ///< Last known position of the fluidics. 
  eFluidicPositions_t           eTargetPos;        ///< Target position for the current movement
    
  FluidicParams_t               *pParams;          ///< Fluidics parameters
    
  uint32_t                      timeoutTimer;      ///< Timer used when waiting for fill detection to occur. If this exceeds a threshold value the fluidics will cancel the last move request.
  uint32_t                      mixTimer;
  
  FluididStatus_t               status;            ///< Status information of the fluidic channel
  
  bool                          publishCompletionEvent; ///< Boolean flag, if not set then do not publish completeMsg. This is cleared when "door open" events are received, otherwise is set.
  bool                   		    chTargetPosReached;

  FluidicMoveSuccessMsg_t       moveSuccessMsg;    ///< Message published when a move has been completed successfully.
  FluidicMoveFailMsg_t          moveFailMsg;       ///< Message published when a move has failed.
  FluidicMixCompleteMsg_t       mixCmpltMsg;       ///< Message published when a mix has been completed successfully.
  FluidicErrorMsg_t             errorMsg;          ///< Message published when an error has occurred.
  XEvent_t                      cmdAccepted;       ///< Message to indicate that the object has accepted the command.
  XMsgCmdFail_t                 cmdFail;           ///< Message to indicate that the object command failed.
  FluidicLiftUpBladderMsg_t     liftUpBlddrMsg;    ///< Message to indicate the lift up of bladders.
  FluidicWaitForFluidAtContactMsg_t waitForFluidContactMsg;
  
  FluidicMixStageCompleteMsg_t  stageCompelteMsg;
  //Control events. Removes the need for function static events.
  FluidicMixMsg_t               mixMsg;            ///< Message sent to the object to trigger a mix command.
  FluidicMovePositionMsg_t      moveMsg;           ///< Message sent to the object to trigger a movement.
  FluidicUpdateParamsMsg_t      paramsMsg;         ///< Message sent to the object to update the parameters.
  XEvent_t                      stopMsg;           ///< Message sent to the object to update the parameters.
  XEvent_t                      errClearMsg;       ///< Message sent to the object to exit the error state..
  FluidicMonitorBreachMsg_t     monitorPositionMsg;
  
  XEvent_t                      breachDetectedMsg; ///< Message sent to the framework to indicate that a fluid breach has been detected.
  XEvent_t                      fcStartBladderDetectMsg;  ///< Kick off bladder detection.
  XEvent_t                      fcStopBladderDetectMsg;   ///< Stop bladder detection.
}
Fluidic_t;


/** @} */
void       FluidicInit(Fluidic_t* me, 
                        const FluidicInitParams_t* pInitParams,
                        XActiveFramework_t*     XFwk);

eErrorCode FluidicMove(Fluidic_t* me, 
                       eFluidicPositions_t eTarget, 
                       float rampSpeedVoltsPerSec,
                       uint32_t timeout,
                       eFluidOvershootCompensation_t   eOvershootComp,
                       float overshootCompProportion);

eErrorCode FluidicLiftUpBladder(Fluidic_t* me,
                                float rampSpeedVoltsPerSec,
                                uint32_t timeout_ms);

eErrorCode FluidicMix(Fluidic_t* me, 
                      eFluidicPositions_t eTarget, 
                      float mixFreq, 
                      uint32_t mixTimeout, 
                      uint32_t cycles,
                      eFluidMixingType_t  eMixType,
                      float openLoopCompensationFactor,
                      float mixDownstrokeProportion);

eErrorCode FluidicStop(Fluidic_t *me);

eErrorCode FluidicParamsSet(Fluidic_t* me, FluidicParams_t *pParams);
eErrorCode FluidicErrorClear(Fluidic_t * me);

eErrorCode FluidicEnableBreachMonitoring(Fluidic_t* me,
                                         bool enable);

eErrorCode FluidicWaitForFluidAtContact(Fluidic_t * me,
                                        eFluidicPositions_t eTarget,
                                        uint32_t timeoutMs);


#endif

/********************************** End Of File ******************************/

