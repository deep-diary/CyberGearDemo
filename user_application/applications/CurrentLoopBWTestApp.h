/**
 ******************************************************************************
 * @file    CurrentloopBWTestApp.h,
 * @author  Motor Control Comptence Center, ST Microelectronics
 * @brief   Declaration of
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
#ifndef __CurrentloopBWTestApp_H
#define __CurrentloopBWTestApp_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_interface.h"
#include "user_application.h"

/* Exported constants --------------------------------------------------------*/

/* Exported type -------------------------------------------------------------*/
typedef struct {
  UserApplication_Handle_t _Super;

  uint32_t TimeStamp;
  uint16_t PulseWidth_ms;
  uint16_t IdRef_10BitRes;
  int16_t IdRef;
  bool PulseLevel;

} CurrentloopBWTestApp_Handle_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void CurrentloopBWTestApp_OnStart(UserApplication_Handle_t* pSuper);
void CurrentloopBWTestApp_MediumFreqUpdate(UserApplication_Handle_t* pSuper);
void CurrentloopBWTestApp_SetIdRef10BitRes(CurrentloopBWTestApp_Handle_t* pHandle, uint16_t IdRef_10BitRes);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __CurrentloopBWTestApp_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
