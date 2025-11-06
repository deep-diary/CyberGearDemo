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
#ifdef ENABLE_MECH_ID_APP
  USER_APP_MECH_ID,
#endif
  USER_APP_JOG,
  USER_APP_COUNT,
} USER_APP_ID;

extern PositionCtrlApp_Handle_t PositionCtrolApp;
extern JogApp_Handle_t JogApp;
extern EncoderAlignmentApp_Handle_t EncoderAlignmentApp;
extern CurrentloopBWTestApp_Handle_t CurrentLoopBWTest;
extern SpeedloopBWTestApp_Handle_t SpeedLoopBWTest;

extern USER_APP_ID UserAppID;
extern USER_APP_ID RequestedUserAppID;

/* 回零和零点设置功能结构体定义 */
typedef struct {
    uint8_t homingModeFlag;     // 回零模式标志位
    uint8_t homingStartFlag;    // 回零开始标志位
    uint16_t homingCounter;     // 回零计数器
    uint8_t posRefSetSuccessFlag; // PosRef设置成功标志位
    uint8_t posRefSetExecuted;  // PosRef设置已执行标志位，防止重复执行
    uint8_t setZeroFlag;        // 设置零点标志位
} HomingControl_t;

/* 回零功能结构体变量声明 */
extern HomingControl_t s_HomingControl;

/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* MC_APP_HOOKS_H */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
