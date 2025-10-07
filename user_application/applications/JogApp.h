/**
  ******************************************************************************
  * @file    JogApp.h,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   Declaration of jog application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
  
#ifndef __JogApp_h
#define __JogApp_h



#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "user_application.h"
#include "mc_interface.h"
/* Exported constants --------------------------------------------------------*/

/* Exported Variables --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct 
{
  UserApplication_Handle_t _Super;
  MCI_Handle_t*            pMCI;

  int16_t Idref;
  int16_t JogSpeed;
  uint16_t Acc;         /* unit: 0.01Hz/s*/

  union {
    uint16_t all;
    struct {
      uint16_t MotorOn    : 1;
      uint16_t FaultReset : 1;
    } bits;

  } flags;

} JogApp_Handle_t;


void JogApp_OnReset(UserApplication_Handle_t* pSuper);
void JogApp_OnBackground(UserApplication_Handle_t* pSuper);

#ifdef __cplusplus
}
#endif /* __cpluplus */



#endif