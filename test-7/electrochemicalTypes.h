/**
 ******************************************************************************
 * @file    electrochemicalTypes.h
 * @author DEW
 * @brief   Electrochemical Type defs - public definitions of EC types
 ******************************************************************************
*/



#ifndef ECHEM_TYPES_H
#define ECHEM_TYPES_H


 

/**
 * @addtogroup  Electrochemical
*  @{
*/ 


#define ECHEM_NUM_SAMPLES_TO_REGISTER_CHANGE (5u)


/**
  * @brief Defined strip sample types
  **/
typedef enum ElectrochemicalSampleTypes_tag
{
  SAMPLE_TYPE_FINGER_STICK = 0,           ///< Blood sample from finger stick (cappillary blood)
  SAMPLE_TYPE_VENUOUS_BLOOD,              ///< Venuous blood sample
  SAMPLE_TYPE_PLASMA,                     ///< Plasma sample, expect < 10% HCT
  SAMPLE_TYPE_SERUM,                      ///< Serum sample
  SAMPLE_TYPE_CONTROL_SOLUTION,           ///< Sample is from control solutions. Should only occur in QC mode.
  
  SAMPLE_TYPE_COUNT
}
ElectrochemicalSampleTypes_t;


/**
  * @brief Electrochemical mode types.
  **/
typedef enum ElectrochemicalMode_tag
{
    EC_MODE_MANUAL              = 0,        //!< Switches in isolated and quiescent pos
    EC_MODE_FLUID_DETECT        = 1,        //!< PStat isolated, POS Sense DAC routed to strip
    EC_MODE_POTENTIOSTAT        = 2,        //!< Potentiostat mode, set chan and gain via switches
    EC_MODE_SELFTEST            = 3,        //!< Potentiostat mode test
    EC_MODE_HCT                 = 4,        /// HCT Mode.
    
    
    EC_MODE_COUNT
}
eElectrochemicalMode;


/**
  * @brief Potential fluid positions in an electrochemical channel.
  **/
typedef enum ElectrochemicalChannelPos_tag
{ 
    // Standard positions = A, B, C
    EC_CHAN_POS_A     = 0,              ///< Position A definition (always the first contact)
    EC_CHAN_POS_B     = 1,              ///< Position B definition 
    EC_CHAN_POS_C     = 2,              ///< Position C definition 
    
    // Extended positions, this accounts for all electrodes mapping to a single
    // channel!
    // Maximum 12 positions as one contact is always fill detect, another is always
    // strip detect (in 15-contact strip).
    EC_CHAN_POS_D     = 3,              ///< Position D definition
    EC_CHAN_POS_E     = 4,              ///< Position E definition  
    EC_CHAN_POS_F     = 5,              ///< Position F definition 
    EC_CHAN_POS_G     = 6,              ///< Position G definition   
    EC_CHAN_POS_H     = 7,              ///< Position H definition  
    EC_CHAN_POS_I     = 8,              ///< Position I definition 
    EC_CHAN_POS_J     = 9,              ///< Position J definition   
    EC_CHAN_POS_K     = 10,             ///< Position K definition   
    EC_CHAN_POS_L     = 11,             ///< Position L definition   
    // Last position in 15-contact strip.
    
    EC_CHAN_POS_COUNT ,
    
    // Position NONE is used ONLY if the fill detect or strip detect contact.
    EC_CHAN_POS_NONE  = 255           ///< Set to this if the electrode cannot be mapped to a position in a channel.
}
eElectrochemicalChannelPos;



/**
  * @brief Define the Strip channels
  * @note Maximum 4 channels in 15-contact strip.
  **/ 
typedef enum ElectrochemicalChannel_tag
{ 
    EC_STRIP_CHAN_1     = 0,  
    EC_STRIP_CHAN_2     = 1,  
    EC_STRIP_CHAN_3     = 2,  
    EC_STRIP_CHAN_4     = 3,
    // Add more channels if available in hardware.

    EC_STRIP_CHAN_COUNT
}
eElectrochemicalChannel;


typedef enum eTestTypeWetDry_tag
{
  EC_TEST_TYPE_DRY = 0,
  EC_TEST_TYPE_WET = 1,
  
  EC_TEST_TYPE_COUNT
}  
eTestTypeWetDry_t;



/* From 8256 0535, Sample type bitfield values :
Bit[0]: Finger stick
Bit[1]: Venous Blood
Bit[2]: Plasma
Bit[3]: Serum
Bit[4]: Control Solution
*/
#define EC_SAMPLE_TYPE_FINGER_STICK   (1u << 0u)
#define EC_SAMPLE_TYPE_VENOUS_BLOOD   (1u << 1u)
#define EC_SAMPLE_TYPE_PLASMA         (1u << 2u)
#define EC_SAMPLE_TYPE_SERUM          (1u << 3u)
#define EC_SAMPLE_TYPE_CONTROL        (1u << 4u)




/**
  * @brief Define the Strip Insertion State
  **/  
typedef enum StripDetectState_tag
{ 
    EC_STRIP_DETECT_STATE_INVALID               = 0,  ///< The value is not yet known
    EC_STRIP_DETECT_STATE_STRIP_NOT_INSERTED    = 1,  ///< The strip is not inserted
    EC_STRIP_DETECT_STATE_STRIP_INSERTED        = 2,  ///< The strip is inserted
    
    EC_STRIP_DETECT_STATE_COUNT 
}
StripDetectState_t;


/**
  * @brief Define the Fill Detect State
  **/ 
typedef enum FillDetectState_tag
{ 
    EC_FILL_DETECT_STATE_INVALID            = 0,  ///< The value is not yet known
    EC_FILL_DETECT_STATE_NO_STRIP           = 1,  ///< No strip present (so cannot be filled.)
    EC_FILL_DETECT_STATE_STRIP_NOT_FILLED   = 2,  ///< Strip present but no sample detected
    EC_FILL_DETECT_STATE_STRIP_FILLED       = 3,  ///< Sample fill detected 
    
    EC_FILL_DETECT_STATE_COUNT 
}
FillDetectState_t;

/**
  *     @brief Definition of a srip contact.
  **/
typedef struct ElectrochemicalContact_tag
{
  bool                        mapToChannels[EC_STRIP_CHAN_COUNT];   ///< Each electrode can be mapped to multiple channels. Therefore an array of booleans to map to the channel.
  eElectrochemicalChannelPos  positionInChannel;                    ///< Each electrode can only represent a single position in a given channel. This must be consistent across multiple channels.
  bool                        isFillDetectPin;                      ///< Electrode is the fill detection pin                 
  bool                        isStripDetectPin;                     ///< Electrode is the strip detection pin.
  float                       thresholdVoltsNoContact[SAMPLE_TYPE_COUNT];  ///< Lowest resistance where no contact is made (no contact if > val)
  float                       thresholdVoltsContact[SAMPLE_TYPE_COUNT];    ///< Maximum resistance where contact is made. (Contact if < val)
}
ElectrochemicalContact_t;



/**
  *     @brief Define a structure for the EC to hold all calibration settings
  **/
typedef struct ElectrochemicalCalibration_tag
{
  
  float                    qdacReferenceVoltage;                  ///< qDAC reference voltages
  float                    sdAdcReferenceVoltage;                 ///< Sigma Delta ADC ref V
  
  float                    potentiostatTransimpedanceOhms;        ///< Impedance of transimpedance amplifier
}
ElectrochemicalCalibration_t;





/**
  * @}
 */
 
#endif

/********************************** End Of File ******************************/
