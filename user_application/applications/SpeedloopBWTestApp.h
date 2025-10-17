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
#ifndef __SpeedloopBWTestApp_H
#define __SpeedloopBWTestApp_H

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

} SpeedloopBWTestApp_Handle_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void SpeedloopBWTestApp_OnStart(UserApplication_Handle_t* pSuper);
void SpeedloopBWTestApp_PreMediumFrequencyUpdate(UserApplication_Handle_t* pSuper);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __SpeedloopBWTestApp_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
