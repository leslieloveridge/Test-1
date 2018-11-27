/**
 ******************************************************************************
 * @file    eventSender.h
 * @author DEW
 * 
 *    
 * @brief Relays (selected) XActive events to the scheduler 
 ******************************************************************************
 */


#ifndef EVENT_SENDER_H_
#define EVENT_SENDER_H_


/**
 * @addtogroup eventSender
*  @{
*/ 


#include "poci.h"
#include "xActive.h"
 



/**
* @brief eventSender 
*/  
typedef struct EventSenderParams_tag
{
  uint8_t       priority;
}
EventSenderParams_t;




/**
* @brief  A class wrapper for the Event Sender. Relays (selected) XActive
* events to the scheduler
**/
typedef struct EventSender_tag
{
  XActive_t     super;                //!< The base XActive class we inherit from
  uint32_t      evQueueBytes[256];    //!< The queue data buffer
  
  char          eventPayloadBuffer[150u];
  
  const EventSenderParams_t* pParams;  //!< Contains setup/operational options
}
EventSender_t;



void EventSenderInit(EventSender_t* me, 
                     const EventSenderParams_t* pParams,
                     XActiveFramework_t* pXActiveFramework);


/**
  * @}
 */
 
#endif 

/********************************** End Of File ******************************/
