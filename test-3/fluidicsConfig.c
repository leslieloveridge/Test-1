/**
 ******************************************************************************
 * @file        fluidicsConfig.c
 * @author      TMW
 * @brief       Fluidics - Configuration settings.      
 ******************************************************************************
 */


#include "poci.h"
#include "electrochemicalTypes.h"
#include "fluidics.h"
#include "fluidicsConfig.h"


/**
 *  @defgroup fluidicsConfiguration Fluidics
 *  @{
*/



/**
  *     @brief Configuration for Bladder 1.
  *     @note All distances are based on experimentation with drivers.
  */
FluidicParams_t bladder1DefaultParams = 
{
   .positionLimits[BC_POS_HOME].targetVolts                       = PIEZO_VOLT_MAX,
   .positionLimits[BC_POS_HOME].posHysterisis                         = FLUIDIC_HYSTERISIS_NONE,
   
   .positionLimits[BC_POS_HOME].echemRequirements[FLUID_MOVE_FWD]     = FD_DATA_INVALID,
   .positionLimits[BC_POS_HOME].echemRequirements[FLUID_MOVE_REV]     = FD_DATA_INVALID,
    
   .positionLimits[BC_POS_DOWN].targetVolts                       = PIEZO_MIN_VOLTAGE,
   .positionLimits[BC_POS_DOWN].posHysterisis                         = FLUIDIC_HYSTERISIS_NONE,
   
   .positionLimits[BC_POS_DOWN].echemRequirements[FLUID_MOVE_FWD]     = NO_FLUID_DETECTED,
   .positionLimits[BC_POS_DOWN].echemRequirements[FLUID_MOVE_REV]     = FLUID_DETECTED,
   
   .positionLimits[BC_POS_FLUID_A].targetVolts                    = FLUIDIC_DEFAULT_TARGET_POSITION,
   .positionLimits[BC_POS_FLUID_A].posHysterisis                      = FLUIDIC_POS_A_HYSTERISIS_V,
   
   .positionLimits[BC_POS_FLUID_A].echemRequirements[FLUID_MOVE_FWD]  = FLUID_POSITION_A,
   .positionLimits[BC_POS_FLUID_A].echemRequirements[FLUID_MOVE_REV]  = FLUID_DETECTED,
   
   .positionLimits[BC_POS_FLUID_B].targetVolts                    = FLUIDIC_DEFAULT_TARGET_POSITION,
   .positionLimits[BC_POS_FLUID_B].posHysterisis                      = FLUIDIC_DEFAULT_HYSTERISIS_V,
   
   .positionLimits[BC_POS_FLUID_B].echemRequirements[FLUID_MOVE_FWD]  = FLUID_POSITION_B,
   .positionLimits[BC_POS_FLUID_B].echemRequirements[FLUID_MOVE_REV]  = FLUID_POSITION_A,
   
   .positionLimits[BC_POS_FLUID_C].targetVolts                    = FLUIDIC_DEFAULT_TARGET_POSITION,
   .positionLimits[BC_POS_FLUID_C].posHysterisis                      = FLUIDIC_DEFAULT_HYSTERISIS_V,
   
   .positionLimits[BC_POS_FLUID_C].echemRequirements[FLUID_MOVE_FWD]  = FLUID_POSITION_C,
   .positionLimits[BC_POS_FLUID_C].echemRequirements[FLUID_MOVE_REV]  = FLUID_POSITION_B,   
  
   .hysterisisMultipliersVolts[FLUID_HYST_INC]                             = FLUID_HYST_MULTIPLIER_INC_DEFAULT,
   .hysterisisMultipliersVolts[FLUID_HYST_DEC]                             = FLUID_HYST_MULTIPLIER_DEC_DEFAULT,
    
  .eChannel                                                           = EC_STRIP_CHAN_1,
                                      
  .timeout_ms                                                         = FLUIDIC_DEFAULT_TIMEOUT_30S,
  .mixFrequency_Hz                                                    = FLUIDIC_DEFAULT_MIX_FREQ,
  .rampSpeedVoltsPerSec                                               = FLUID_SPEED_LOW_DEFAULT_V_PER_S,
  .mixTimeoutMax_ms                                                   = FLUIDIC_MAX_MIX_TIMEOUT_DEFAULT_MS,
  .eMixEndPosition                                                    = BC_POS_UNKNOWN,
  .returnSpeedRedcutionFactor                                        = FLUID_RETURN_SPEED_REDUCTION_FACTOR,
};


/**
  *     @brief Configuration for Bladder 2.
  *     @note All distances are based on experimentation with drivers.
  */
FluidicParams_t bladder2DefaultParams = 
{
   .positionLimits[BC_POS_HOME].targetVolts                       = PIEZO_VOLT_MAX,
   .positionLimits[BC_POS_HOME].posHysterisis                         = FLUIDIC_HYSTERISIS_NONE,
   
   .positionLimits[BC_POS_HOME].echemRequirements[FLUID_MOVE_FWD]     = FD_DATA_INVALID,
   .positionLimits[BC_POS_HOME].echemRequirements[FLUID_MOVE_REV]     = FD_DATA_INVALID,
    
   .positionLimits[BC_POS_DOWN].targetVolts                       = PIEZO_MIN_VOLTAGE,
   .positionLimits[BC_POS_DOWN].posHysterisis                         = FLUIDIC_HYSTERISIS_NONE,
   
   .positionLimits[BC_POS_DOWN].echemRequirements[FLUID_MOVE_FWD]     = NO_FLUID_DETECTED,
   .positionLimits[BC_POS_DOWN].echemRequirements[FLUID_MOVE_REV]     = FLUID_DETECTED,
   
   .positionLimits[BC_POS_FLUID_A].targetVolts                    = FLUIDIC_DEFAULT_TARGET_POSITION,
   .positionLimits[BC_POS_FLUID_A].posHysterisis                      = FLUIDIC_POS_A_HYSTERISIS_V,
   
   .positionLimits[BC_POS_FLUID_A].echemRequirements[FLUID_MOVE_FWD]  = FLUID_POSITION_A,
   .positionLimits[BC_POS_FLUID_A].echemRequirements[FLUID_MOVE_REV]  = FLUID_DETECTED,
   
   .positionLimits[BC_POS_FLUID_B].targetVolts                    = FLUIDIC_DEFAULT_TARGET_POSITION,
   .positionLimits[BC_POS_FLUID_B].posHysterisis                      = FLUIDIC_DEFAULT_HYSTERISIS_V,
   
   .positionLimits[BC_POS_FLUID_B].echemRequirements[FLUID_MOVE_FWD]  = FLUID_POSITION_B,
   .positionLimits[BC_POS_FLUID_B].echemRequirements[FLUID_MOVE_REV]  = FLUID_POSITION_A,
   
   .positionLimits[BC_POS_FLUID_C].targetVolts                    = FLUIDIC_DEFAULT_TARGET_POSITION,
   .positionLimits[BC_POS_FLUID_C].posHysterisis                      = FLUIDIC_DEFAULT_HYSTERISIS_V,
   
   .positionLimits[BC_POS_FLUID_C].echemRequirements[FLUID_MOVE_FWD]  = FLUID_POSITION_C,
   .positionLimits[BC_POS_FLUID_C].echemRequirements[FLUID_MOVE_REV]  = FLUID_POSITION_B,
  
   .hysterisisMultipliersVolts[FLUID_HYST_INC]                             = FLUID_HYST_MULTIPLIER_INC_DEFAULT,
   .hysterisisMultipliersVolts[FLUID_HYST_DEC]                             = FLUID_HYST_MULTIPLIER_DEC_DEFAULT,

  .eChannel                                                           = EC_STRIP_CHAN_2,
                                      
  .timeout_ms                                                         = FLUIDIC_DEFAULT_TIMEOUT_30S,
  .mixFrequency_Hz                                                    = FLUIDIC_DEFAULT_MIX_FREQ,
  .rampSpeedVoltsPerSec                                                          = FLUID_SPEED_LOW_DEFAULT_V_PER_S,
  .mixTimeoutMax_ms                                                   = FLUIDIC_MAX_MIX_TIMEOUT_DEFAULT_MS,
  .eMixEndPosition                                                    = BC_POS_UNKNOWN,
  .returnSpeedRedcutionFactor                                        = FLUID_RETURN_SPEED_REDUCTION_FACTOR,
};


/**
  *     @brief Configuration for Bladder 1.
  *     @note All distances are based on experimentation with drivers.
  */
FluidicParams_t bladder3DefaultParams = 
{
   .positionLimits[BC_POS_HOME].targetVolts                       = PIEZO_VOLT_MAX,
   .positionLimits[BC_POS_HOME].posHysterisis                         = FLUIDIC_HYSTERISIS_NONE,
   
   .positionLimits[BC_POS_HOME].echemRequirements[FLUID_MOVE_FWD]     = FD_DATA_INVALID,
   .positionLimits[BC_POS_HOME].echemRequirements[FLUID_MOVE_REV]     = FD_DATA_INVALID,
    
   .positionLimits[BC_POS_DOWN].targetVolts                       = PIEZO_MIN_VOLTAGE,
   .positionLimits[BC_POS_DOWN].posHysterisis                         = FLUIDIC_HYSTERISIS_NONE,
   
   .positionLimits[BC_POS_DOWN].echemRequirements[FLUID_MOVE_FWD]     = NO_FLUID_DETECTED,
   .positionLimits[BC_POS_DOWN].echemRequirements[FLUID_MOVE_REV]     = FLUID_DETECTED,
   
   .positionLimits[BC_POS_FLUID_A].targetVolts                    = FLUIDIC_DEFAULT_TARGET_POSITION,
   .positionLimits[BC_POS_FLUID_A].posHysterisis                      = FLUIDIC_POS_A_HYSTERISIS_V,
   
   .positionLimits[BC_POS_FLUID_A].echemRequirements[FLUID_MOVE_FWD]  = FLUID_POSITION_A,
   .positionLimits[BC_POS_FLUID_A].echemRequirements[FLUID_MOVE_REV]  = FLUID_DETECTED,
   
   .positionLimits[BC_POS_FLUID_B].targetVolts                    = FLUIDIC_DEFAULT_TARGET_POSITION,
   .positionLimits[BC_POS_FLUID_B].posHysterisis                      = FLUIDIC_DEFAULT_HYSTERISIS_V,
   
   .positionLimits[BC_POS_FLUID_B].echemRequirements[FLUID_MOVE_FWD]  = FLUID_POSITION_B,
   .positionLimits[BC_POS_FLUID_B].echemRequirements[FLUID_MOVE_REV]  = FLUID_POSITION_A,
   
   .positionLimits[BC_POS_FLUID_C].targetVolts                    = FLUIDIC_DEFAULT_TARGET_POSITION,
   .positionLimits[BC_POS_FLUID_C].posHysterisis                      = FLUIDIC_DEFAULT_HYSTERISIS_V,
   
   .positionLimits[BC_POS_FLUID_C].echemRequirements[FLUID_MOVE_FWD]  = FLUID_POSITION_C,
   .positionLimits[BC_POS_FLUID_C].echemRequirements[FLUID_MOVE_REV]  = FLUID_POSITION_B,   
  
   .hysterisisMultipliersVolts[FLUID_HYST_INC]                             = FLUID_HYST_MULTIPLIER_INC_DEFAULT,
   .hysterisisMultipliersVolts[FLUID_HYST_DEC]                             = FLUID_HYST_MULTIPLIER_DEC_DEFAULT,
  
  .eChannel                                                           = EC_STRIP_CHAN_3,
                                    
  .timeout_ms                                                         = FLUIDIC_DEFAULT_TIMEOUT_30S,
  .mixFrequency_Hz                                                    = FLUIDIC_DEFAULT_MIX_FREQ,
  .rampSpeedVoltsPerSec                                                          = FLUID_SPEED_LOW_DEFAULT_V_PER_S,
  .mixTimeoutMax_ms                                                   = FLUIDIC_MAX_MIX_TIMEOUT_DEFAULT_MS,
  .eMixEndPosition                                                    = BC_POS_UNKNOWN,
  .returnSpeedRedcutionFactor                                        = FLUID_RETURN_SPEED_REDUCTION_FACTOR,
};


/**
  *     @brief Configuration for Bladder 4.
  *     @note All distances are based on experimentation with drivers.
  */
FluidicParams_t bladder4DefaultParams = 
{
   .positionLimits[BC_POS_HOME].targetVolts                       = PIEZO_VOLT_MAX,
   .positionLimits[BC_POS_HOME].posHysterisis                         = FLUIDIC_HYSTERISIS_NONE,
   
   .positionLimits[BC_POS_HOME].echemRequirements[FLUID_MOVE_FWD]     = FD_DATA_INVALID,
   .positionLimits[BC_POS_HOME].echemRequirements[FLUID_MOVE_REV]     = FD_DATA_INVALID,
    
   .positionLimits[BC_POS_DOWN].targetVolts                       = PIEZO_MIN_VOLTAGE,
   .positionLimits[BC_POS_DOWN].posHysterisis                         = FLUIDIC_HYSTERISIS_NONE,
   
   .positionLimits[BC_POS_DOWN].echemRequirements[FLUID_MOVE_FWD]     = NO_FLUID_DETECTED,
   .positionLimits[BC_POS_DOWN].echemRequirements[FLUID_MOVE_REV]     = FLUID_DETECTED,
   
   .positionLimits[BC_POS_FLUID_A].targetVolts                    = FLUIDIC_DEFAULT_TARGET_POSITION,
   .positionLimits[BC_POS_FLUID_A].posHysterisis                      = FLUIDIC_POS_A_HYSTERISIS_V,
   
   .positionLimits[BC_POS_FLUID_A].echemRequirements[FLUID_MOVE_FWD]  = FLUID_POSITION_A,
   .positionLimits[BC_POS_FLUID_A].echemRequirements[FLUID_MOVE_REV]  = FLUID_DETECTED,
   
   .positionLimits[BC_POS_FLUID_B].targetVolts                    = FLUIDIC_DEFAULT_TARGET_POSITION,
   .positionLimits[BC_POS_FLUID_B].posHysterisis                      = FLUIDIC_DEFAULT_HYSTERISIS_V,
   
   .positionLimits[BC_POS_FLUID_B].echemRequirements[FLUID_MOVE_FWD]  = FLUID_POSITION_B,
   .positionLimits[BC_POS_FLUID_B].echemRequirements[FLUID_MOVE_REV]  = FLUID_POSITION_A,
   
   .positionLimits[BC_POS_FLUID_C].targetVolts                    = FLUIDIC_DEFAULT_TARGET_POSITION,
   .positionLimits[BC_POS_FLUID_C].posHysterisis                      = FLUIDIC_DEFAULT_HYSTERISIS_V,
   
   .positionLimits[BC_POS_FLUID_C].echemRequirements[FLUID_MOVE_FWD]  = FLUID_POSITION_C,
   .positionLimits[BC_POS_FLUID_C].echemRequirements[FLUID_MOVE_REV]  = FLUID_POSITION_B,  
  
   .hysterisisMultipliersVolts[FLUID_HYST_INC]                             = FLUID_HYST_MULTIPLIER_INC_DEFAULT,
   .hysterisisMultipliersVolts[FLUID_HYST_DEC]                             = FLUID_HYST_MULTIPLIER_DEC_DEFAULT,
  
   .eChannel                                                          = EC_STRIP_CHAN_4,
                                      
   .timeout_ms                                                        = FLUIDIC_DEFAULT_TIMEOUT_30S,
   .mixFrequency_Hz                                                   = FLUIDIC_DEFAULT_MIX_FREQ,
   .rampSpeedVoltsPerSec                                                         = FLUID_SPEED_LOW_DEFAULT_V_PER_S,
   .mixTimeoutMax_ms                                                  = FLUIDIC_MAX_MIX_TIMEOUT_DEFAULT_MS,
   .eMixEndPosition                                                   = BC_POS_UNKNOWN,
   .returnSpeedRedcutionFactor                                        = FLUID_RETURN_SPEED_REDUCTION_FACTOR,
};



/**
  * @}
  */

/********************************** End Of File ******************************/

