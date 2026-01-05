/**
  ******************************************************************************
  * @file    CurrentloopBWTestApp.c
  * @author  Motor Control Competence Center, ST Microelectronics
  * @brief   Implementation of current loop bandwidth test application
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "CurrentLoopBWTestApp.h"
#include "SineGenerator.h"
#include "mc_app_hooks_servo.h"
#include "user_application.h"
#include "speed_torq_ctrl.h"
/* Extra Includes -------------------------------------------------------------*/

/* Private constants --------------------------------------------------------*/
extern bool motor_start;
extern bool FaultReset;

/* Private type -------------------------------------------------------------*/

/* Private variables --------------------------------------------------------*/

/* Private functions ------------------------------------------------------- */
static void CurrentloopBWTestApp_SetupSineGenerator(CurrentLoopBWTestApp_Handle_t* pHandle)
{
  if (pHandle->CurRefSinStartFreq01Hz > pHandle->CurRefSinEndFreq01Hz) {
    pHandle->pSin->hStartFrequency01Hz = pHandle->CurRefSinEndFreq01Hz;
    pHandle->pSin->hEndFrequency01Hz   = pHandle->CurRefSinStartFreq01Hz;
  } else {
    pHandle->pSin->hStartFrequency01Hz = pHandle->CurRefSinStartFreq01Hz;
    pHandle->pSin->hEndFrequency01Hz   = pHandle->CurRefSinEndFreq01Hz;
  }
  pHandle->pSin->hAmplitude          = pHandle->CurRefSinAmp_CurrentUnit;
  pHandle->pSin->hDurationms         = pHandle->CurSweepDuration_ms;
  pHandle->pSin->OutputMode          = CPG_BIPOLAR_MODE;

  SineGenerator_SetUpdateFrequency(pHandle->pSin, pHandle->Fs);
  SineGenerator_Reset(pHandle->pSin);
}

/* Global functions ------------------------------------------------------- */

/**
  * @brief  电流环带宽测试应用重置函数
  * @param  pSuper: 用户应用句柄指针
  * @retval None
  */
void CurrentloopBWTestApp_OnReset(UserApplication_Handle_t* pSuper)
{
  CurrentLoopBWTestApp_Handle_t* pHandle = (CurrentLoopBWTestApp_Handle_t*)pSuper;
  pSuper->OneShootTaskFinished = false;
  pHandle->flags.all = 0;
  pHandle->CurrentRef = 0;
  pHandle->PrevCurrentRef = 0;
  pHandle->IdCurrentRef = 0;
  pHandle->CurrentCurrentRef = 0;
}

/**
  * @brief  电流环带宽测试应用启动函数
  * @param  pSuper: 用户应用句柄指针
  * @retval None
  */
void CurrentloopBWTestApp_OnStart(UserApplication_Handle_t* pSuper)
{
  CurrentLoopBWTestApp_Handle_t* pHandle = (CurrentLoopBWTestApp_Handle_t*)pSuper;

  if (pHandle->flags.bits.EnableSineRef) {
    /* Setup sine generator */
    CurrentloopBWTestApp_SetupSineGenerator(pHandle);
  }
  pHandle->flags.bits.PrevEnableSineRef = pHandle->flags.bits.EnableSineRef;
  
  /* This will set the control mode to torque mode */
  STC_SetSpeedSensor(pSuper->pMCI->pSTC, pSuper->pMCI->pPosCtrl->pSPD);
  MCI_ExecTorqueRamp(pSuper->pMCI, 0, 0);
}

/**
  * @brief  电流环带宽测试应用退出函数
  * @param  pSuper: 用户应用句柄指针
  * @retval None
  */
void CurrentloopBWTestApp_OnExit(UserApplication_Handle_t* pSuper)
{
  CurrentLoopBWTestApp_Handle_t* pHandle = (CurrentLoopBWTestApp_Handle_t*)pSuper;
  pHandle->flags.all = 0;
  MCI_StopMotor(pSuper->pMCI);
  MCI_ExecSpeedRamp(pSuper->pMCI, 0, 0);
}

/**
  * @brief  电流环带宽测试应用后台更新函数
  * @param  pSuper: 用户应用句柄指针
  * @retval None
  */
void CurrentloopBWTestApp_OnBackground(UserApplication_Handle_t* pSuper)
{
  CurrentLoopBWTestApp_Handle_t* pHandle = (CurrentLoopBWTestApp_Handle_t*)pSuper;
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
      }
      break;

    default:
      break;
  }
}

/**
  * @brief  电流环带宽测试应用中频更新函数
  * @param  pSuper: 用户应用句柄指针
  * @retval None
  */
void CurrentloopBWTestApp_PreMediumFrequencyUpdate(UserApplication_Handle_t* pSuper)
{
  CurrentLoopBWTestApp_Handle_t* pHandle = (CurrentLoopBWTestApp_Handle_t*)pSuper;
  
  if (MCI_GetSTMState(pSuper->pMCI) == RUN) {
    if (pHandle->flags.bits.EnableSineRef) {
      if (!pHandle->flags.bits.PrevEnableSineRef) {
        /* 正弦测试使能状态改变，重置正弦发生器 */
        CurrentloopBWTestApp_SetupSineGenerator(pHandle);
        pHandle->flags.bits.PrevEnableSineRef = true;
      }
      
      /* 生成正弦扫频信号 */
      pHandle->CurrentCurrentRef = SineGenerator_Update(pHandle->pSin);
      
      /* 如果扫频完成，停止电机 */
      if (pHandle->pSin->chirpMode && SineGenerator_IsCompleted(pHandle->pSin)) {
        MCI_StopMotor(pSuper->pMCI);
      }
    } else {
      if (pHandle->flags.bits.PrevEnableSineRef) {
        /* 关闭正弦测试时，以关闭时的正弦测试电流为电流指令 */
        pHandle->CurrentRef = pHandle->CurrentCurrentRef;
        pHandle->PrevCurrentRef = pHandle->CurrentCurrentRef;
      } else if (pHandle->CurrentRef != pHandle->PrevCurrentRef) {
        /* 用户更新了新的电流控制指令 */
        pHandle->PrevCurrentRef = pHandle->CurrentRef;
      }
      pHandle->flags.bits.PrevEnableSineRef = false;
      pHandle->CurrentCurrentRef = pHandle->CurrentRef;
    }
    
    /* 执行电流控制 */
    qd_t IqdRef = {0, 0};
    
    if (pHandle->flags.bits.EnableSineRef) {
      /* 正弦测试使能时使用d轴电流 */
      IqdRef.q = 0;
      IqdRef.d = pHandle->CurrentCurrentRef;
    } else {
      /* 正弦测试未使能时使用q轴电流 */
      IqdRef.q = pHandle->CurrentCurrentRef;
      IqdRef.d = pHandle->IdCurrentRef;
     }
    //STC_SetSpeedSensor(pHandle->_Super->pMCI->*pSTC, &ENCODER_M1._Super);
    MCI_SetCurrentReferences(pSuper->pMCI, IqdRef);
    //MCI_ExecTorqueRamp(pSuper->pMCI, pHandle->CurrentCurrentRef, 1000);
  }
}
