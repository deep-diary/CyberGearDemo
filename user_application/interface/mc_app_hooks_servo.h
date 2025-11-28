/**
 ******************************************************************************
 * @file    mc_app_hooks.h
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file implements tasks definition.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 * @ingroup MCAppHooks
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MC_APP_HOOKS_SERVO_H
#define MC_APP_HOOKS_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "CurrentloopBWTestApp.h"
#include "SpeedloopBWTestApp.h"
#include "PositionCtrlApp.h"
#include "EncoderAlignmentApp.h"
#include "JogApp.h"
#include "MITControlApp.h"
#include "HomingApp.h"
/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup MCTasks
 * @{
 */

/** @addtogroup MCAppHooks
 * @{
 */

/* Exported functions ------------------------------------------------------- */

typedef enum {
  USER_APP_NONE,
  USER_APP_NORMAL_POS_CTRL,
  USER_APP_ENCODER_ALIGNMENT,
  USER_APP_CURRENTLOOP_BW_TEST,
  USER_APP_SPEEDLOOP_BW_TEST,
  USER_APP_JOG,
  USER_APP_MIT_CONTROL,
  USER_APP_HOMING,
  USER_APP_COUNT,
} USER_APP_ID;

extern PositionCtrlApp_Handle_t PositionCtrolApp;
extern JogApp_Handle_t JogApp;
extern EncoderAlignmentApp_Handle_t EncoderAlignmentApp;
extern CurrentloopBWTestApp_Handle_t CurrentLoopBWTest;
extern SpeedloopBWTestApp_Handle_t SpeedLoopBWTest;
extern MITControlApp_Handle_t MITControlApp;
extern PositionCtrlApp_Handle_t HomingApp;

extern USER_APP_ID UserAppID;
extern USER_APP_ID RequestedUserAppID;


/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* MC_APP_HOOKS_H */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
