/**
 ******************************************************************************
 * @file    PositionCtrlApp.h,
 * @author  Motor Control Comptence Center, ST Microelectronics
 * @brief   Declaration of position control app
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

#ifndef __PositionCtrlApp_h
#define __PositionCtrlApp_h

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_interface.h"
#include "user_application.h"
#include "SineGenerator.h"
/* Exported constants --------------------------------------------------------*/

/* Exported Variables --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct {
  UserApplication_Handle_t _Super;
  MCI_Handle_t*            pMCI;
  SineGenerator_Handle_t*  pSin;

  float_t PosRef;
  float_t PrevPosRef;
  float_t duration;

  float_t SinRefOffset;
  
  float_t RefSinAmp;
  uint16_t Fs;
  uint16_t RefSinStartFreq01Hz;
  uint16_t RefSinEndFreq01Hz;
  uint16_t SweepDuration_ms;


  union 
  {
    uint16_t all;
    struct
    {
      uint16_t MotorOn       : 1;
      uint16_t FaultReset    : 1;
      uint16_t EnableSineRef : 1;
    } bits;

  } flags;
  
} PositionCtrlApp_Handle_t;

void PositionCtrlApp_OnReset(UserApplication_Handle_t* pSuper);
void PositionCtrlApp_OnStart(UserApplication_Handle_t* pSuper);
void PositionCtrlApp_OnBackground(UserApplication_Handle_t* pSuper);
void PositionCtrlApp_OnLowFrequencyUpdate(UserApplication_Handle_t* pSuper);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif