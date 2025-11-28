/**
  ******************************************************************************
  * @file    PositionCtrlApp.c,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   Implementation of position control application
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
  

#include "user_application.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "HomingApp.h"
extern bool motor_start;
extern bool FaultReset;

/* private constants --------------------------------------------------------*/

/* private Variables --------------------------------------------------------*/

/* Exported Variables --------------------------------------------------------*/

/* Function implementation ---------------------------------------------------*/


void HomingApp_OnReset(UserApplication_Handle_t* pSuper)
{
  PositionCtrlApp_Handle_t* pHandle = (PositionCtrlApp_Handle_t*)pSuper;
  STC_SetSpeedSensor(pSuper->pMCI->pSTC, pSuper->pMCI->pPosCtrl->pSPD);
  MCI_SetSpeedMode(pSuper->pMCI);
}

void HomingApp_OnStart(UserApplication_Handle_t* pSuper)
{
  PositionCtrlApp_Handle_t* pHandle = (PositionCtrlApp_Handle_t*)pSuper;

  /* This will set the control mode to speed mode */
  MCI_ExecSpeedRamp(pSuper->pMCI, 0, 0);

  int32_t currentPosition = MCI_GetCurrentPosition(pSuper->pMCI);
  pHandle->PosRef = 0;
  pHandle->PrevPosRef = currentPosition;
  PositionProfileGenerator_PresetPosition(pHandle->pPosGen, currentPosition);
  PosCtrl_Reset(pSuper->pMCI->pPosCtrl);

}

void HomingApp_OnBackground(UserApplication_Handle_t* pSuper)
{
  PositionCtrlApp_Handle_t* pHandle = (PositionCtrlApp_Handle_t*)pSuper;
  pHandle->flags.bits.MotorOn = motor_start;
  pHandle->flags.bits.FaultReset = FaultReset;
  switch (MCI_GetSTMState(pSuper->pMCI)) {
    case IDLE:
      if (pHandle->flags.bits.MotorOn) {
        MCI_StartMotor(pSuper->pMCI);
      }
      break;

    case RUN:
      if (!pHandle->flags.bits.MotorOn) {
        MCI_StopMotor(pSuper->pMCI);
      }

      break;

    case FAULT_NOW:
    case FAULT_OVER:
      if (pHandle->flags.bits.FaultReset) {
        MCI_FaultAcknowledged(pSuper->pMCI);
        pHandle->flags.bits.FaultReset = false;
        FaultReset = false;
      }
      break;

    default:
      break;
  }
}

void HomingApp_OnLowFrequencyUpdate(UserApplication_Handle_t* pSuper)
{
    PositionCtrlApp_Handle_t* pHandle = (PositionCtrlApp_Handle_t*)pSuper;
    int32_t deltaPos = 0;
    if (RUN == MCI_GetSTMState(pSuper->pMCI)) {
       if (pHandle->PosRef != pHandle->PrevPosRef) {
            PositionProfileGenerator_Replan(pHandle->pPosGen, pHandle->PosRef, PPG_ABSOLUTE);
            pHandle->PrevPosRef = pHandle->PosRef;    
          }
          deltaPos = PositionProfileGenerator_Update(pHandle->pPosGen);
        
        int16_t SpeedRef = PosCtrl_Update(pSuper->pMCI->pPosCtrl, deltaPos);
        MCI_ExecSpeedRamp(pSuper->pMCI, SpeedRef, 0);
    }
}

#ifdef __cplusplus
}
#endif /* __cpluplus */


