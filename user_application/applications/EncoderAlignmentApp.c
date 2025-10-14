/**
  ******************************************************************************
  * @file    EncoderAlignmentApp.c,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   Implementation of encoder alignment application
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
  



#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "EncoderAlignmentApp.h"
#include "stm32g4xx_hal.h"
#include "param_manager.h"
/* private constants --------------------------------------------------------*/
#define EAA_ID_SCALE_BITS   15
/* private Variables --------------------------------------------------------*/

/* Exported Variables --------------------------------------------------------*/

/* Function implementation ---------------------------------------------------*/
void EncoderAlignmentApp_Init(UserApplication_Handle_t* pSuper)
{
//  EncoderAlignmentApp_Handle_t* pHandle = (EncoderAlignmentApp_Handle_t*)pSuper;

}

void EncoderAlignmentApp_Reset(UserApplication_Handle_t* pSuper)
{
    EncoderAlignmentApp_Handle_t* pHandle = (EncoderAlignmentApp_Handle_t*)pSuper;
    pSuper->OneShootTaskFinished = false;
    pHandle->state = EAA_IDLE;
}

void EncoderAlignmentApp_OnBackground(UserApplication_Handle_t* pSuper)
{
    EncoderAlignmentApp_Handle_t* pHandle = (EncoderAlignmentApp_Handle_t*)pSuper;

    switch (pHandle->state)
    {
    case EAA_IDLE:
        if (pHandle->flags.bits.Start) {
            qd_t Iqd = {0,0};
            pHandle->IdrefStep = ((int32_t)pHandle->Idref << EAA_ID_SCALE_BITS) / pHandle->AlignmentDuration;
            if (0 == pHandle->IdrefStep) {
                pHandle->IdrefStep = 1;
            }
            pHandle->pEncoder->zeroAngleOffset = 0;
            pHandle->pEncoder->direction = 1;
            MCI_SetCurrentReferences(pSuper->pMCI, Iqd);
            MCI_SetOpenLoopCurrentMode(pSuper->pMCI);
            VSS_SetMecAcceleration(pSuper->pMCI->pVSS, 0, 0);
            VSS_SetElAngle(pSuper->pMCI->pVSS, 0);
            MCI_StartMotor(pSuper->pMCI);            
            pHandle->state = EAA_ALIGNMENT;
            pHandle->flags.bits.Start = false;
            pHandle->TimeStamp = HAL_GetTick();
        }
        break;

    case EAA_ALIGNMENT: {
        uint32_t duration = HAL_GetTick() - pHandle->TimeStamp;
        if (duration > pHandle->AlignmentDuration) {
            if (RUN == MCI_GetSTMState(pSuper->pMCI)) {
                pHandle->pEncoder->zeroAngleOffset = -pHandle->pEncoder->_Super.hMecAngle;
                /* Delay 100ms to allow the wmecangle being updated, this routine can only be called in background*/
                HAL_Delay(100);
                pHandle->AlignedMecAngle = pHandle->pEncoder->_Super.hMecAngle;
                VSS_SetMecAcceleration(pSuper->pMCI->pVSS, (int16_t)(0.4f * SPEED_UNIT), 1.0f);
                pHandle->TimeStamp = HAL_GetTick();
                /* without saving result */
                pHandle->state = EAA_SPINING;
            } else {
                MCI_StopMotor(pSuper->pMCI);
                pHandle->state = EAA_IDLE;
            }
        } else {
            qd_t Iqd = {0,0};
            Iqd.d = (duration * pHandle->IdrefStep) >> EAA_ID_SCALE_BITS;
            MCI_SetCurrentReferences(pSuper->pMCI, Iqd);
        }

    }
        
    break;


    case EAA_SPINING:
        if (HAL_GetTick() - pHandle->TimeStamp < pHandle->SpinningDuration) {
            int16_t deltaMecAngle = pHandle->pEncoder->_Super.hMecAngle - pHandle->AlignedMecAngle;
            if (deltaMecAngle < -EAA_DIR_DETECT_MOVEMENT_TH) {
                pHandle->pEncoder->direction = -1;
                pHandle->state = EAA_FINISHED;
            } else if (deltaMecAngle > EAA_DIR_DETECT_MOVEMENT_TH) {
                pHandle->pEncoder->direction = 1;
                pHandle->state = EAA_FINISHED;
            }

            if (EAA_FINISHED == pHandle->state) {
                MCI_StopMotor(pSuper->pMCI);
                pSuper->OneShootTaskFinished = true;
                if (pHandle->flags.bits.EnableSave2EE) {
                    ParamManager_RequestParamSaving();
                }
            }
        } else {
            MCI_StopMotor(pSuper->pMCI);
            pSuper->OneShootTaskFinished = true;
            pHandle->state = EAA_IDLE;
        }

        break;

    default:
        break;
    }

}

#ifdef __cplusplus
}
#endif /* __cpluplus */


