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
#include "PositionCtrlApp.h"


/* private constants --------------------------------------------------------*/

/* private Variables --------------------------------------------------------*/

/* Exported Variables --------------------------------------------------------*/

/* Function implementation ---------------------------------------------------*/

static void PositionCtrlApp_SetupSineGenerator(PositionCtrlApp_Handle_t* pHandle)
{
  if (pHandle->RefSinStartFreq01Hz > pHandle->RefSinEndFreq01Hz) {
    pHandle->pSin->hStartFrequency01Hz = pHandle->RefSinEndFreq01Hz;
    pHandle->pSin->hEndFrequency01Hz   = pHandle->RefSinStartFreq01Hz;
  } else {
    pHandle->pSin->hStartFrequency01Hz = pHandle->RefSinStartFreq01Hz;
    pHandle->pSin->hEndFrequency01Hz   = pHandle->RefSinEndFreq01Hz;
  }
  pHandle->pSin->hAmplitude  = INT16_MAX;
  pHandle->pSin->hDurationms = pHandle->SweepDuration_ms;
  pHandle->pSin->OutputMode = CPG_SINGLEPOLAR_MODE;

  SineGenerator_SetUpdateFrequency(pHandle->pSin, pHandle->Fs);
  SineGenerator_Reset(pHandle->pSin);
}

void PositionCtrlApp_OnReset(UserApplication_Handle_t* pSuper)
{
  PositionCtrlApp_Handle_t* pHandle = (PositionCtrlApp_Handle_t*)pSuper;
  STC_SetSpeedSensor(pSuper->pMCI->pSTC, pSuper->pMCI->pPosCtrl->pSPD);
  MCI_SetSpeedMode(pSuper->pMCI);
}

void PositionCtrlApp_OnStart(UserApplication_Handle_t* pSuper)
{
  PositionCtrlApp_Handle_t* pHandle = (PositionCtrlApp_Handle_t*)pSuper;

  if (pHandle->flags.bits.EnableSineRef) { /* Setup sine generator */
    PositionCtrlApp_SetupSineGenerator(pHandle);
  }
  pHandle->flags.bits.PrevEnableSineRef = pHandle->flags.bits.EnableSineRef;
  /* This will set the control mode to speed mode */
  MCI_ExecSpeedRamp(pSuper->pMCI, 0, 0);

  int32_t currentPosition = MCI_GetCurrentPosition(pSuper->pMCI);
  pHandle->PosRef = currentPosition;
  pHandle->PrevPosRef = currentPosition;
  pHandle->SinRefOffset = currentPosition;
  PositionProfileGenerator_PresetPosition(pHandle->pPosGen, currentPosition);
  PosCtrl_Reset(pSuper->pMCI->pPosCtrl);

}

void PositionCtrlApp_OnBackground(UserApplication_Handle_t* pSuper)
{
  PositionCtrlApp_Handle_t* pHandle = (PositionCtrlApp_Handle_t*)pSuper;
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
      }
      break;

    default:
      break;
  }
}

void PositionCtrlApp_OnLowFrequencyUpdate(UserApplication_Handle_t* pSuper)
{
    PositionCtrlApp_Handle_t* pHandle = (PositionCtrlApp_Handle_t*)pSuper;
    int32_t deltaPos = 0;
    if (RUN == MCI_GetSTMState(pSuper->pMCI)) {
        if (pHandle->flags.bits.EnableSineRef) {
          if (!pHandle->flags.bits.PrevEnableSineRef) {
            /* Sine ref enable state changed, reset the sine generator */
            PositionCtrlApp_SetupSineGenerator(pHandle);
            pHandle->flags.bits.PrevEnableSineRef = true;
            pHandle->SinRefOffset = MCI_GetCurrentPosition(pSuper->pMCI);
            pHandle->PrevPosRef = pHandle->SinRefOffset;
          }
          int32_t PosRef = ((int64_t)SineGenerator_Update(pHandle->pSin) * pHandle->RefSinAmp) >> 15; // Q15
          PosRef += pHandle->SinRefOffset;
          pHandle->PosRef = PosRef;
          deltaPos = PosRef - pHandle->PrevPosRef;
          pHandle->PrevPosRef = PosRef;
          if (pHandle->pSin->chirpMode && SineGenerator_IsCompleted(pHandle->pSin)) {
              MCI_StopMotor(pSuper->pMCI);
          }
        } else {
          if (pHandle->flags.bits.PrevEnableSineRef) {
            int32_t currentPosition = MCI_GetCurrentPosition(pSuper->pMCI);
            pHandle->PosRef = currentPosition;
            pHandle->PrevPosRef = currentPosition;
            /* to clear the retained acc and speed */
            PositionProfileGenerator_PresetPosition(pHandle->pPosGen, currentPosition);
            PositionProfileGenerator_Replan(pHandle->pPosGen, pHandle->PosRef, PPG_ABSOLUTE);
          } else if (pHandle->PosRef != pHandle->PrevPosRef) {
            PositionProfileGenerator_Replan(pHandle->pPosGen, pHandle->PosRef, PPG_ABSOLUTE);
            pHandle->PrevPosRef = pHandle->PosRef;    
          }
          pHandle->flags.bits.PrevEnableSineRef = false;
          deltaPos = PositionProfileGenerator_Update(pHandle->pPosGen);
        }
        int16_t SpeedRef = PosCtrl_Update(pSuper->pMCI->pPosCtrl, deltaPos);
        MCI_ExecSpeedRamp(pSuper->pMCI, SpeedRef, 0);
    }
}

#ifdef __cplusplus
}
#endif /* __cpluplus */


