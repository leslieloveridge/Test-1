/**
 ******************************************************************************
 * @file    Electrochemical.h
 * @author DEW 
 * @brief   Electrochemical  
 ******************************************************************************
*/



#ifndef ELECTROCHEM_H_
#define ELECTROCHEM_H_



/**
 * @addtogroup  Electrochemical
*  @{
*/ 


#include "poci.h"
#include "xActive.h"
#include "electrochemicalTypes.h"
#include "ecFluidDetect.h"
#include "ecPotentiostat.h"
#include "ecPinMapping.h"


#define DEFAULT_PSTAT_A_REF_VOLTS (SD_ADC_REF_VOLTAGE)
#define DEFAULT_PSTAT_B_REF_VOLTS (SD_ADC_REF_VOLTAGE)
#define DEFAULT_PSTAT_BIAS_VOLTS  (SD_ADC_REF_VOLTAGE / 2.0f)
#define MAX_PSTAT_VOLTAGE         (SD_ADC_REF_VOLTAGE)
#define ECHEM_LOGGING_ENABLE_ALL  ((uint16_t)0xFFFFu);

/// Electrochemical events to subscribe to.
/// This is at the top of the file so that you can read it straight away! (Not need to dive into the file)
#define X_SUBSCRIBE_TO_ECHEM_EVENTS(me_) \
X_SUBSCRIBE(me_, XMSG_EC_DISABLE) \
X_SUBSCRIBE(me_, XMSG_EC_FLUID_DETECT_BEGIN) \
X_SUBSCRIBE(me_, XMSG_EC_POTENTIOSTAT_BEGIN) \
X_SUBSCRIBE(me_, XMSG_EC_SELFTEST_BEGIN) \
X_SUBSCRIBE(me_, XMSG_EC_WAIT_FOR_STRIP_DETECT) \
X_SUBSCRIBE(me_, XMSG_EC_WAIT_FOR_FILL_DETECT) \
X_SUBSCRIBE(me_, XMSG_EC_ERROR_CLEAR) \
X_SUBSCRIBE(me_, XMSG_EC_UPDATE_SAMPLE_TYPE) \
X_SUBSCRIBE(me_, XMSG_EC_UPDATE_CONTACT_CONFIG) \
X_SUBSCRIBE(me_, XMSG_LOGGING_DISABLE )\
X_SUBSCRIBE(me_, XMSG_FLUID_START_BLDDR_DETECT) \
X_SUBSCRIBE(me_, XMSG_FLUID_STOP_BLDDR_DETECT) \
X_SUBSCRIBE(me_, XMSG_FLUID_MONITOR_STOP_BLDDR_DETECT) \
X_SUBSCRIBE_TO_GLOBAL_EVENTS(me_)

// 5ms per electrode, 15 pins to sample. Therefore 75ms update period in CPLD.
// Can only sample on 20ms intervals.
#define ECHEM_UPDATE_PERIOD_MS 80u

#define BLADDER_DOWN_LOW_END	  (1.7f) ///< in Volts, based on 2.5V ref and 1.5V Bias voltage
#define BLADDER_DOWN_UPPER_END  (3.3f) ///< in Volts, based on 2.5V ref and 1.5V Bias voltage
#define BLADDER_UP_LOW_END		  (0.8f) ///< in Volts, based on 2.5V ref and 1.5V Bias voltage
#define BLADDER_UP_UPPER_END	  (1.4f) ///< in Volts, based on 2.5V ref and 1.5V Bias voltage

/**
  *@brief	ecBladderDownDetectionParams_tag
  *@details Parameters of bladder detection command
  **/
typedef struct ecBladderDownDetectionParams_tag
{
    bool enable;            ///< Enable <1>, disable <0>
    float pstatARefVolts;
    float pstatBRefVolts;
    float pstatBiasVolts;
    float contactClosedLowEndThresh;
    float contactClosedUpperEndThresh;
    float contactOpenLowEndThresh;
    float contactOpenUpperEndThresh;
}ecBladderDownDetectionParams_t;

/**
  *@brief	EcBladderDetectChannelPair_tag
  *@details Pair of channels to read each time
  **/
typedef enum EcBladderDetectChannelPair_tag
{
    EC_PAIR_A_CH_1_AND_B_CH_2 = 0,
    EC_PAIR_A_CH_3_AND_B_CH_4 = 1
}
eEcBladderDetectChannelPair;

/**
  *@brief FillDetectStart_tag
  *@details Event used to start fluid detection in the electrochemical object.
  **/
typedef struct FillDetectStart_tag
{
  XEvent_t                      super;                                          ///< Base event class
  eElectrochemicalChannel       eChan;                                          ///< Electrochemical channel to enable
  eElectrochemicalChannelPos    minimumPosition;
  
}
FillDetectStart_t;


/**
  *@brief UpdateSampleType_tag
  *@details Message used to update the sample type for fill detection and fluid control.
  **/
typedef struct UpdateSampleType_tag
{
  XEvent_t                      super;                                          ///< Base event class
  ElectrochemicalSampleTypes_t  eSampleType;                                    ///< The sample type to use.
}
UpdateSampleType_t;


/**
  *@brief UpdateContactConfiguration_tag
  *@details Message used to modify the configurations of the electrochemical contacts.
  **/
typedef struct UpdateContactConfiguration_tag
{
  XEvent_t                      super;
  ElectrochemicalPin_t          ePinNumber;                                     ///< The pin to be updated.
  ElectrochemicalContact_t      newConfig;                                      ///< The new configuration information.
}
UpdateContactConfiguration_t;


/**
  *@brief UpdateSampleThresholds_tag
  *@details Structure used to hold threshold contact resistances for a variety of sample types.
  **/
typedef struct UpdateSampleThresholds_tag
{
  ElectrochemicalPin_t          ePinNumber;                                ///< The pin number to be updated.
  uint8_t                       sampleType;                                ///< Sample types the new thresholds would apply to.
  float                         thresholdVoltsNoContact;                   ///< Minimum "open circuit" contact resistance.
  float                         thresholdVoltsContact;                     ///< Minimum resistance for contact to be made.
}
UpdateSampleThresholds_t;



/**
  *@brief WaitForStripEv_tag
  *@details Event used to start strip detection in the electrochemical object.
  **/
typedef struct WaitForStripEv_tag
{
  XEvent_t                      super;                                          ///< Base event class
  bool                          state;                                          ///< The desired strip state. If true the electrochemical object will wait until the strip is inserted.
}
WaitForStripEv_t;


/**
  *@brief EcDisableMsg_tag
  *@details Event used to stop the current activity of the electrochemical object.
  **/
typedef struct EcDisableMsg_tag
{
  XEvent_t                      super;                                          ///< Base event class
  eElectrochemicalChannel       eChan;                                          ///< Electrochemical channel to disable.
}
EcDisableMsg_t;



/**
  *@brief    Event used to indicate that the fluid detect status has changed.
  *@details  This is used to indicate that any channel has been updated.
  *           The initial fill, and strip status, are published separately.
  **/
typedef struct FillDetectStatusChange_tag
{
    XEvent_t                    super;                                          ///< Base event class
    EcFluidDetectResults_t              results;                                ///< Fill detect results.
}
FillDetectStatusChange_t;



/**
  *@brief EchemErrorMsg_tag   
  *@details Event used to indicate that an error has occurred in the electrochemistry.
  **/
typedef struct EchemErrorMsg_tag
{
  XEvent_t                      super;                                          ///< Base event class
  eErrorCode                    errorCode;                                      ///< The error code.
}
EchemErrorMsg_t;



/**
  *@brief  PotentiostatDataAvilEv_tag  
  *@details EEvent used to publish the potentiostat data.
  **/
typedef struct PotentiostatDataAvilEv_tag
{
  XEvent_t                    super;                                            ///< Base event class
  EcPotentiostatResults_t     res;                                              ///< The potentiostat results.
}
PotentiostatDataAvilEv_t;


/**
  * @brief Event to modify fluid detection parameters.
  **/
typedef struct FluidDetectParamsUpdate_tag
{
  XEvent_t                    super;                                            ///< Base event class
  float                       modulationAmplitude;                              ///< Amplitude of the AC excitation signal
  float                       excitationBias;                                   ///< Bias applied to AC excitation signal
  float                       opAmpBias;                                        ///< Bias applied to signal conditioning amplifiers
  float                       feedbackResistor;                                 ///< Feedback resistor value (Ohms)
  
}
FluidDetectParamsUpdate_t;

/**
  *@brief  ElectrochemicalInitParams_tag  
  *@details Electrochemical object initialisation parameters.
  **/
typedef struct ElectrochemicalInitParams_tag
{
  const ElectrochemicalCalibration_t*       pCal;                               ///< Calibration information.   

  Potentiostat_t*                           pPotentiostat;                      ///< Pointer to the potentiostat interface
  FluidDetect_t*                            pFluidDetect;                       ///< Pointer to the fluid detect interface
  ElectrochemicalContact_t                  *pContacts;                        ///< Pointer to the electrochem contact definition table.
  const EcFluidDetectParams_t               *pFdParams;                     ///< Initialisation parameters for fill detection.
}
ElectrochemicalInitParams_t;

typedef enum ElectrochemicalBladderDownStatus_tag
{
  ECHEM_BLD_UNKOWN = 0,
  ECHEM_BLD_UP,
  ECHEM_BLD_DOWN,
}
ElectrochemicalBladderDownStatus_t;


typedef struct ElectrochemicalEnableLogging_tag
{
  XEvent_t              super;
  uint16_t              contactsToEnable;
}
ElectrochemicalEnableLogging_t;


/**
  *@brief  Electrochemical_tag   
  *@details The electrochemical object type.
  **/
typedef struct Electrochemical_tag
{
  XActive_t                             super;                                  ///< XActive base class
  XTimer_t                              timer;                                  ///< Electrochemical object's timer.
  
  uint32_t                              evQueueBytes[64];                       ///< Data queue for events.
    
  const ElectrochemicalCalibration_t*   pCal;                                   ///< Pointer to the electrochemical calibration information
  
  ElectrochemicalContact_t              *pContacts;                             ///< Pointer to the electrochem contact definition table
  ElectrochemicaStrip_t                  strip;                                 ///< The status of the strip (abstracted).
 
  
  Potentiostat_t*                       pPotentiostat;                          ///< The potentiostat interface
  FluidDetect_t*                        pFluidDetect;                           ///< The fluid detect interface
  
  StripDetectState_t                    targetStripState;                       ///< The desired strip state when waiting for strip insertion / removal
  StripDetectState_t                    stripDetectState;                       ///< The current strip detect state.
  FillDetectState_t                     fillDetectState;                        ///< The current fill detection state.
  
  ElectrochemicalSampleTypes_t          eSampleType;                            ///< The sample type being used for the current measurement types.
  
  ElectrochemicalBladderDownStatus_t    bladderDownStatuses[EC_STRIP_CHAN_COUNT];
  float                                 bladderDownLastVolts[EC_STRIP_CHAN_COUNT];
  
  float 								ecPstatARefVolts;						///< Configurable ref voltage for potentiostat A
  float 								ecPstatBRefVolts;                       ///< Configurable ref voltage for potentiostat B
  float 								ecPstatBiasVolts;                       ///< Configurable bias voltage for potentiostat
  float 								ecContactClosedLowEndThresh;
  float 								ecContactClosedUpperEndThresh;
  float 								ecContactOpenLowEndThresh;
  float 								ecContactOpenUpperEndThresh;
  
  
  uint8_t                             numSamplesAtCurrentFillStatus;
  uint8_t                             numSamplesAtCurrentStripStatus;
  bool                                logEnable;
  
  eEcBladderDetectChannelPair         bldDetectchannelPair;
  UpdateSampleThresholds_t            updateSampleThresholds;                 ///< Structure to hold contact thresholds.

  //-------------------------------- Events.
  
  UpdateSampleType_t                    updateSampelTypeEv;                     ///< Event sent to the electrochem object to update the sample type
  UpdateContactConfiguration_t          updateContactConfigEv;                  ///< Event sent to the electrochem object to configure an electrochemical contact
  FillDetectStart_t                     fillDetectBeginEv[EC_STRIP_CHAN_COUNT]; ///< Event sent to the electrochem object to start fill detection.
  EcDisableMsg_t                        disableEv[EC_STRIP_CHAN_COUNT];         ///< Event sent to the electrochem object to disable the current activity
  XEvent_t                              waitForFillEv;                          ///< Event sent to the electrochem object to start waiting for strip insertion / removal
  WaitForStripEv_t                      waitForStripEv;                         ///< Event sent to the electrochem object to start waiting for initial fill detect.
  
  FillDetectStatusChange_t              fdStatusChangeEv;                       ///< Event sent by the electrochem to indicate that at least one of the channel's fluid position has changed.
  PotentiostatDataAvilEv_t              potDataEv;                              ///< Event sent by the electrochem to publish potentiostat data.
  EchemErrorMsg_t                       errorEv;                                ///< Event sent by the electrochem to publish an error event.
  FluidDetectParamsUpdate_t             fdParamsUpdateMsg;
  ElectrochemicalEnableLogging_t        enableLoggingMsg;
  
  
  
  XEvent_t                              stripInsertedEvent;                     ///< Transition to strip inserted state (published by the electrochem)
  XEvent_t                              stripRemovedEvent;                      ///< Transition to strip inserted state (published by the electrochem)
  XEvent_t                              fillUndetectEvent;                      ///< Transition to not filled state (published by the electrochem)
  XEvent_t                              fillDetectedEvent;                      ///< Transition to filled state (published by the electrochem)
  XEvent_t                              bladderDetectionStartEvent;             ///< Start bladder detection engine
  XEvent_t                              bladderDetectionStopEvent;				///< Stop bladder detection engine
  XEvent_t                              bladderStatusACH1Down;					///< Status of bladder published per channel
  XEvent_t								bladderStatusACH1Up;					///< Status of bladder published per channel
  XEvent_t								bladderStatusACH3Down;					///< Status of bladder published per channel
  XEvent_t							  bladderStatusACH3Up;					///< Status of bladder published per channel
  XEvent_t								bladderStatusBCH2Down;					///< Status of bladder published per channel
  XEvent_t								bladderStatusBCH2Up;					///< Status of bladder published per channel
  XEvent_t								bladderStatusBCH4Down;					///< Status of bladder published per channel
  XEvent_t								bladderStatusBCH4Up;					///< Status of bladder published per channel


}Electrochemical_t;

/**
  * @}
 */


void ElectrochemicalInit(Electrochemical_t* me, 
                         const ElectrochemicalInitParams_t* pInitParams, 
                         XActiveFramework_t* XFwk);

eErrorCode ecLock(Electrochemical_t* me);
eErrorCode ecUnlock(Electrochemical_t* me);

void ecDebugPrintFluidDetectResults(const EcFluidDetectResults_t* pResults);

/// @todo When potentiostat is required in the project, add this back in.
//eErrorCode ecSetModePotentiostat(Electrochemical_t* me,
//                        const EcPotentiostatParams_t* meParams);

eElectrochemicalMode ecExitModePotentiostat(Electrochemical_t* me);

eErrorCode ecSamplePotentiostat(Electrochemical_t* me);
void ecDebugPrintPstatResults(const EcPotentiostatResults_t* pResults);


eErrorCode ecSetModeSelfTest(Electrochemical_t* me );
eErrorCode ecExitModeSelfTest(Electrochemical_t* me );
eErrorCode ecSampleSelfTest(Electrochemical_t* me);


eErrorCode ecSetModeFillDetect(Electrochemical_t *me, 
                               eElectrochemicalChannel eChan,
                               eElectrochemicalChannelPos  minimumPosition);

eErrorCode EcWaitForStripInserted(Electrochemical_t *me);
eErrorCode EcWaitForStripRemoved(Electrochemical_t *me);
eErrorCode EcWaitForFillDetect(Electrochemical_t *me);

eErrorCode ecDisable(Electrochemical_t *me, eElectrochemicalChannel eChan);

eErrorCode ecSetModePotentiostat(Electrochemical_t *me,
                                 eElectrochemicalChannel eChan,
                                 EcPotentiostatParams_t *pParams);



StripDetectState_t  ecGetStripInsertionState(const Electrochemical_t *me);
FillDetectState_t   ecGetFillDetectState(const Electrochemical_t *me);

eEcFluidDetectPosition_t ecGetFluidPosition(const Electrochemical_t *me, eElectrochemicalChannel eChan);

eErrorCode ecSetSampleType(Electrochemical_t *me,
                           ElectrochemicalSampleTypes_t eSampleType);

eErrorCode EcUpdateContactConfig(Electrochemical_t *me, 
                                 const ElectrochemicalPin_t ePin,
                                 const ElectrochemicalContact_t contactConfig);

eErrorCode EcUpdateSampleThresholds(Electrochemical_t *me,
                                    const ElectrochemicalPin_t ePin,
                                    const uint8_t sampleType,
                                    const float thresholdVoltsNoContact,
                                    const float thresholdVoltsContact);
 
void EcModifyFluidDetectionParams(Electrochemical_t *me,
                                  float modulationAmplitude,
                                  float excitationBias,
                                  float opAmpBias,
                                  float Rf);
                                  
const EcFluidDetectParams_t * EcGetFluidDetectionParams(Electrochemical_t *me);

void ecStartStopBladderDetection(Electrochemical_t *me, 
                                 ecBladderDownDetectionParams_t params);

void EcEnableLogging(Electrochemical_t *me,
                     uint16_t contactsToLog);
#endif 

/********************************** End Of File ******************************/
