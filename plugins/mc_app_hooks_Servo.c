
/**
 ******************************************************************************
 * @file    mc_app_hooks.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file implements default motor control app hooks.
 *
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

/* Includes ------------------------------------------------------------------*/
#include "Scheduler.h"
#include "mc_api.h"
#include "mc_app_hooks.h"
#include "mc_config.h"
#include "mc_config_ext.h"
#include "mc_type.h"

#include "arm_math.h"
#include"can_interface.h"


#include <math.h> 
#define ABS(x) ((x) > 0 ? (x) : -(x))
/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup MCTasks
 * @{
 */

/**
 * @defgroup MCAppHooks Motor Control Applicative hooks
 * @brief User defined functions that are called in the Motor Control tasks.
 *
 *
 * @{
 */

/**
 * @brief Hook function called right before the end of the MCboot function.
 *
 *
 *
 */
void MC_APP_BootHook(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed at the end of MCboot().
   */

  /* USER CODE BEGIN BootHook */
  Scheduler_Init();

  /* USER CODE END BootHook */
}

/**
 * @brief Hook function called right after the Medium Frequency Task for Motor 1.
 *
 *
 *
 */
void MC_APP_PostMediumFrequencyHook_M1(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed right after the Medium
   * Frequency Task of Motor 1
   */

  /* USER SECTION BEGIN PostMediumFrequencyHookM1 */
  Scheduler_MediumFrequencyUpdate();
  /* USER SECTION END PostMediumFrequencyHookM1 */
}

/**
 * @brief Hook function called right after the Medium Frequency Task for Motor 1.
 *
 *
 *
 */
void MC_APP_LowFrequencyHook_M1(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed right after the Medium
   * Frequency Task of Motor 1
   */

  /* USER SECTION BEGIN PostMediumFrequencyHookM1 */
  
  if (MC_GetSTMStateMotor1() == RUN) {
  
    TC_PositionRegulation(pPosCtrl[M1]);

  }

  /* USER SECTION END PostMediumFrequencyHookM1 */
}

void MC_APP_StartRunHook_M1(void)
{
  // PositionProfileGenerator_Reset(&PosProfile_M1);
  // PositionControl_Reset(&PosCtrl_M1);
}

uint8_t motorOn = false;
uint8_t faultReset = false;
int16_t speedRef = 300;
int16_t prevSpeedRef = 300;
uint16_t Duration = 5000;
float32_t posRef = 0;
float32_t prevPosRef = 0;
float32_t speedMax = 0.0;
float32_t prevSpeedMax = 0.0;
float32_t PosDuration =2;
bool flag =1;

void UserApp(void)
{
switch (MC_GetSTMStateMotor1())
{
  case IDLE:
    if (motorOn) {
    MC_StartMotor1();
     //MC_ProgramPositionCommandMotor1(posRef, 0);
   

    }
    break;
  
  case RUN:
    if (!motorOn) {
    MC_StopMotor1();
    }
  

    if (posRef != prevPosRef ) {
      //PosDuration = ABS(3*(posRef-prevPosRef)/(2*speedMax)) ;
      MC_ProgramPositionCommandMotor1(posRef, PosDuration);
      prevPosRef = posRef;
    }


    break;
  
  case FAULT_NOW:
  case FAULT_OVER:
    if (faultReset) {
    MC_AcknowledgeFaultMotor1();
    faultReset = false;
    }
    break;
  
  default:
    break;
  }
}

void MC_APP_BackgroundHook_M1(void) 
{
  CAN_ProcessMessages();
  UserApp();
}

/** @} */

/** @} */

/** @} */

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
