/**
  ******************************************************************************
  * @file    JogApp.c,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   Implementation of jog application
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
#include "JogApp.h"


/* private constants --------------------------------------------------------*/

/* private Variables --------------------------------------------------------*/

/* Exported Variables --------------------------------------------------------*/

/* Function implementation ---------------------------------------------------*/

// void JogApp_Init(UserApplication_Handle_t* pSuper)
// {

// }

void JogApp_OnReset(UserApplication_Handle_t* pSuper)
{
    JogApp_Handle_t* pHandle = (JogApp_Handle_t*)pSuper;
    MCI_SetOpenLoopCurrentMode(pHandle->pMCI);
}

void JogApp_OnBackground(UserApplication_Handle_t* pSuper)
{
    JogApp_Handle_t* pHandle = (JogApp_Handle_t*)pSuper;

    switch (MCI_GetSTMState(pHandle->pMCI))
    {
    case IDLE:
        if (pHandle->flags.bits.MotorOn) {
            qd_t Iqd = {0, pHandle->Idref};
            int32_t temp = pHandle->JogSpeed;
            temp = temp >= 0 ? temp : -temp;
            temp *= pHandle->Acc;
            MCI_SetCurrentReferences(pHandle->pMCI, Iqd);
            MCI_ExecSpeedRamp(pHandle->pMCI, pHandle->JogSpeed, temp);
            MCI_StartMotor(pHandle->pMCI);
        }
        break;

    case RUN:
        if (!pHandle->flags.bits.MotorOn) {
            MCI_StopMotor(pHandle->pMCI);
        }
    
    default:
        break;
    }
}


#ifdef __cplusplus
}
#endif /* __cpluplus */


