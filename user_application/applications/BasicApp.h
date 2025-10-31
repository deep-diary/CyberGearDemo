#ifndef __BasicApp_h
#define __BasicApp_h
#endif /* __BasicApp_h */


#ifdef __cplusplus
extern "C" {        
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "user_application.h"  

/* Exported constants --------------------------------------------------------*/

/* Exported type -------------------------------------------------------------*/
typedef struct {
  UserApplication_Handle_t _Super;
  
  int16_t Iref;
  int16_t SpeedRef;
  int16_t PrevSpeedRef;
  int16_t Durationms;

  union UFlags
  {
    uint16_t all;
    struct
    {
      uint16_t MotorOn : 1;
      uint16_t FaultReset : 1;
    } bits;
  } Flags;
  

} BasicApp_Handle_t;

void BasicApp_OnReset(UserApplication_Handle_t* pSuper);
void BasicApp_OnExit(UserApplication_Handle_t* pSuper);
void BasicApp_OnBackground(UserApplication_Handle_t* pSuper);


#ifdef __cplusplus
}   
#endif /* __cplusplus */