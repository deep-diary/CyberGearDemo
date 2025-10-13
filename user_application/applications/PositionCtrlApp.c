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

  SineGenerator_SetUpdateFrequency(pHandle->pSin, pHandle->Fs);
  SineGenerator_Reset(pHandle->pSin);
}

void PositionCtrlApp_OnReset(UserApplication_Handle_t* pSuper)
{
  PositionCtrlApp_Handle_t* pHandle = (PositionCtrlApp_Handle_t*)pSuper;
  STC_SetSpeedSensor(pHandle->pMCI->pSTC, &pHandle->pMCI->pPosCtrl->pENC->_Super);
  MCI_SetSpeedMode(pHandle->pMCI);
}

void PositionCtrlApp_OnStart(UserApplication_Handle_t* pSuper)
{
  PositionCtrlApp_Handle_t* pHandle = (PositionCtrlApp_Handle_t*)pSuper;

  if (pHandle->flags.bits.EnableSineRef) { /* Setup sine generator */
    PositionCtrlApp_SetupSineGenerator(pHandle);
  }
  pHandle->flags.bits.PrevEnableSineRef = pHandle->flags.bits.EnableSineRef;
  /* This will set the control mode to speed mode */
  MCI_ExecSpeedRamp(pHandle->pMCI, 0, 0);
}

void PositionCtrlApp_OnBackground(UserApplication_Handle_t* pSuper)
{
  PositionCtrlApp_Handle_t* pHandle = (PositionCtrlApp_Handle_t*)pSuper;
  switch (MCI_GetSTMState(pHandle->pMCI)) {
    case IDLE:
      if (pHandle->flags.bits.MotorOn) {
        float_t currentPosition = MCI_GetCurrentPosition(pHandle->pMCI);
        pHandle->PosRef = currentPosition;
        pHandle->PrevPosRef = currentPosition;
        pHandle->SinRefOffset = currentPosition;
        MCI_ExecPositionCommand(pHandle->pMCI, currentPosition, 0.002f);
        // qd_t Iqd = {0,0};
        // MCI_SetCurrentReferences(pHandle->pMCI, Iqd);
        MCI_StartMotor(pHandle->pMCI);
      }
      break;

    case RUN:
      if (!pHandle->flags.bits.MotorOn) {
        MCI_StopMotor(pHandle->pMCI);
      }

      break;

    case FAULT_NOW:
    case FAULT_OVER:
      if (pHandle->flags.bits.FaultReset) {
        MCI_FaultAcknowledged(pHandle->pMCI);
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
    if (RUN == MCI_GetSTMState(pHandle->pMCI)) {
        if (pHandle->flags.bits.EnableSineRef) {
          if (!pHandle->flags.bits.PrevEnableSineRef) {
            /* Sine ref enable state changed, reset the sine generator */
            PositionCtrlApp_SetupSineGenerator(pHandle);
            pHandle->flags.bits.PrevEnableSineRef = true;
            pHandle->SinRefOffset = MCI_GetCurrentPosition(pHandle->pMCI);
          }
          float_t PosRef = SineGenerator_Update(pHandle->pSin) * (1.0f/INT16_MAX )* pHandle->RefSinAmp;
          PosRef += pHandle->SinRefOffset;
          MCI_ExecPositionCommand(pHandle->pMCI, PosRef, 0.0f);
          pHandle->PosRef = PosRef;
          pHandle->PrevPosRef = PosRef;
          if (pHandle->pSin->chirpMode && SineGenerator_IsCompleted(pHandle->pSin)) {
              MCI_StopMotor(pHandle->pMCI);
          }
        } else {
          if (pHandle->flags.bits.PrevEnableSineRef) {
            /* to clear the retained acc and speed */
            MCI_ExecPositionCommand(pHandle->pMCI, pHandle->PosRef, 0.01f);
            pHandle->PrevPosRef = pHandle->PosRef;    
          } else if (pHandle->PosRef != pHandle->PrevPosRef) {
            // PosDuration = ABS(3*(posRef-prevPosRef)/(2*speedMax)) ;
            MCI_ExecPositionCommand(pHandle->pMCI, pHandle->PosRef, pHandle->duration);
            // MC_ProgramPositionCommandMotor1New(posRef,speedNext);
            pHandle->PrevPosRef = pHandle->PosRef;    
          }
          pHandle->flags.bits.PrevEnableSineRef = false;
        }

        TC_PositionRegulation(pHandle->pMCI->pPosCtrl);
    }
}

#ifdef __cplusplus
}
#endif /* __cpluplus */


