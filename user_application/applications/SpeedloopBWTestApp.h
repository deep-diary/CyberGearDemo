/**
 ******************************************************************************
 * @file    SpeedloopBWTestApp.h,
 * @author  Motor Control Comptence Center, ST Microelectronics
 * @brief   Declaration of speed loop bandwidth test.
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
#ifndef __SPEEDLOOPBWTESTAPP_H
#define __SPEEDLOOPBWTESTAPP_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "SineGenerator.h"
#include "mc_interface.h"
#include "user_application.h"


/* Exported constants --------------------------------------------------------*/

/* Exported type -------------------------------------------------------------*/
typedef struct {
  UserApplication_Handle_t       _Super;
  SineGenerator_Handle_t* pSin;

  uint16_t Fs;
  uint16_t SpdRefSinStartFreq01Hz;
  uint16_t SpdRefSinEndFreq01Hz;
  int16_t SpdRefSinAmp_SpeedUnit;
  uint16_t SpdSweepDuration_ms;
  
  int16_t SpeedRef;                    // 速度参考值 (速度单位)
  int16_t PrevSpeedRef;                // 上一速度参考值
  int16_t CurrentSpeedRef;             // 当前速度参考值 (速度单位)
  
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

} SpeedloopBWTestApp_Handle_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void SpeedloopBWTestApp_OnReset(UserApplication_Handle_t* pSuper);
void SpeedloopBWTestApp_OnStart(UserApplication_Handle_t* pSuper);
void SpeedloopBWTestApp_OnBackground(UserApplication_Handle_t* pSuper);
void SpeedloopBWTestApp_PreMediumFrequencyUpdate(UserApplication_Handle_t* pSuper);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __SpeedloopBWTestApp_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
