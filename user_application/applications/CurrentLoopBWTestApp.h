/**
  ******************************************************************************
  * @file    CurrentloopBWTestApp.h,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   Declaration of current loop bandwidth test application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CURRENTLOOPBWTESTApp_H
#define __CURRENTLOOPBWTESTApp_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_interface.h"
#include "user_application.h"
#include "SineGenerator.h"

/* Exported constants --------------------------------------------------------*/

/* Exported type -------------------------------------------------------------*/
typedef struct {
  UserApplication_Handle_t _Super;
  SineGenerator_Handle_t* pSin;

  uint16_t Fs;
  uint16_t CurRefSinStartFreq01Hz;
  uint16_t CurRefSinEndFreq01Hz;
  int16_t  CurRefSinAmp_CurrentUnit;
  uint16_t CurSweepDuration_ms;
  
  int16_t CurrentRef;                    // 电流参考值 (电流单位)
  int16_t PrevCurrentRef;                // 上一电流参考值
  int16_t IdCurrentRef;                 // d轴电流参考值 (电流单位)
  int16_t CurrentCurrentRef;                 // 当前电流参考值 (电流单位)
  union
  {
    uint16_t all;
    struct
    {
      uint16_t MotorOn : 1;
      uint16_t FaultReset : 1;
      uint16_t EnableSineRef : 1;      // 正弦测试使能标志
      uint16_t PrevEnableSineRef : 1;  // 上一正弦测试使能状态
    } bits;
  } flags;

} CurrentLoopBWTestApp_Handle_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void CurrentloopBWTestApp_OnReset(UserApplication_Handle_t* pSuper);
void CurrentloopBWTestApp_OnStart(UserApplication_Handle_t* pSuper);
void CurrentloopBWTestApp_OnExit(UserApplication_Handle_t* pSuper);
void CurrentloopBWTestApp_OnBackground(UserApplication_Handle_t* pSuper);
void CurrentloopBWTestApp_PreMediumFrequencyUpdate(UserApplication_Handle_t* pSuper);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __CurrentLoopBWTestApp_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
