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

#ifndef __HomingApp_h
#define __HomingApp_h

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_interface.h"
#include "user_application.h"
#include "SineGenerator.h"
#include "PositionProfileGenerator.h"
/* Exported constants --------------------------------------------------------*/

/* Exported Variables --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/


void HomingApp_OnReset(UserApplication_Handle_t* pSuper);
void HomingApp_OnStart(UserApplication_Handle_t* pSuper);
void HomingApp_OnBackground(UserApplication_Handle_t* pSuper);
void HomingApp_OnLowFrequencyUpdate(UserApplication_Handle_t* pSuper);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif