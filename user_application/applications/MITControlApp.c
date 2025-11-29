/**
  ******************************************************************************
  * @file    MITControlApp.c,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   Implementation of MIT motion control application
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

#include "MITControlApp.h"
#include "parameters_conversion.h"
#include "can_interface.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Private constants --------------------------------------------------------*/
extern bool motor_start;
extern bool FaultReset;

/* Private Variables --------------------------------------------------------*/

/* Exported Variables --------------------------------------------------------*/

/* Function implementation ---------------------------------------------------*/

/**
  * @brief  MIT控制应用重置函数
  * @param  pSuper: 用户应用句柄指针
  * @retval None
  */
void MITControlApp_OnReset(UserApplication_Handle_t* pSuper)
{
    MITControlApp_Handle_t* pHandle = (MITControlApp_Handle_t*)pSuper;
    pSuper->OneShootTaskFinished = false;
    pHandle->flags.all = 0;
    
    /* 初始化MIT控制参数 */
    pHandle->PosRef           = 0.0f;     // 位置参考值 (rad)
    pHandle->VelRef           = 0.0f;     // 速度参考值 (rad/s)
    pHandle->Kp               = 100.0f;   // 位置比例增益
    pHandle->Kd               = 1.0f;     // 速度微分增益
    pHandle->TorqueFF         = 0.0f;     // 前馈转矩 (Nm)
    pHandle->TorqueRef        = 0.0f;     // 转矩参考值 (Nm)
    pHandle->iqRef            = 0.0f;     // 电流参考值 (A)
    pHandle->iqRefTran        = 0;        // 转换后的电流参考值 (MCSDK内部单位)
}

void MITControlApp_OnStart(UserApplication_Handle_t* pSuper)
{
  MITControlApp_Handle_t* pHandle = (MITControlApp_Handle_t*)pSuper;

  /* This will set the control mode to speed mode */
  MCI_ExecSpeedRamp(pSuper->pMCI, 0, 0);

  int32_t currentPosition = MCI_GetCurrentPosition(pSuper->pMCI);
  pHandle->PosRef = (float)(currentPosition*POS_FACTOR);

}

/**
  * @brief  MIT控制应用退出函数
  * @param  pSuper: 用户应用句柄指针
  * @retval None
  */
void MITControlApp_OnExit(UserApplication_Handle_t* pSuper)
{
    MITControlApp_Handle_t* pHandle = (MITControlApp_Handle_t*)pSuper;
    pHandle->flags.all = 0;
    MCI_StopMotor(pSuper->pMCI);
    MCI_SetSpeedMode(pSuper->pMCI);
}

/**
  * @brief  MIT控制应用后台更新函数
  * @param  pSuper: 用户应用句柄指针
  * @retval None
  */
void MITControlApp_OnBackground(UserApplication_Handle_t* pSuper)
{
    MITControlApp_Handle_t* pHandle = (MITControlApp_Handle_t*)pSuper;
   pHandle->flags.bits.MotorOn = motor_start;
   pHandle->flags.bits.FaultReset = FaultReset;
    switch (MCI_GetSTMState(pSuper->pMCI))
    {
    case IDLE:
        if (pHandle->flags.bits.MotorOn) {
            qd_t Iqd = {0, 0};
            MCI_SetCurrentReferences(pSuper->pMCI, Iqd);
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
            pHandle->flags.bits.FaultReset = 0;
            MCI_FaultAcknowledged(pSuper->pMCI);
        }
        break;
    
    default:
        break;
    }
}

/**
  * @brief  MIT控制应用低频更新函数
  * @param  pSuper: 用户应用句柄指针
  * @retval None
  */
int64_t pos_term,vel_term;
int32_t pos_error,vel_error;
qd_t Iqd = {0,0} ;  

void MITControlApp_OnLowFrequencyUpdate(UserApplication_Handle_t* pSuper)
{
    MITControlApp_Handle_t* pHandle = (MITControlApp_Handle_t*)pSuper;

    if (RUN == MCI_GetSTMState(pSuper->pMCI)) {
        // 获取当前位置和速度
        pHandle->CurrentPosition = MCI_GetCurrentPosition(pSuper->pMCI)*POS_FACTOR;
        pHandle->CurrentVelocity = MCI_GetAvrgMecSpeedUnit(pSuper->pMCI)*SPD_FACTOR;

        // MIT控制算法: torque_ref = Kp*(p_des - theta_mech) + TorqueFF + Kd*(v_des - dtheta_mech)    
        pHandle->TorqueRef = pHandle->Kp * (pHandle->PosRef - pHandle->CurrentPosition) 
                          + pHandle->TorqueFF 
                          + pHandle->Kd * (pHandle->VelRef - pHandle->CurrentVelocity);
        
        // 将转矩参考值转换为电流参考值: TorqueRef  = torque_ref / Kt
        pHandle->iqRef = (pHandle->TorqueRef * KT_OUT);//单位A
        pHandle->iqRefTran = (int16_t)(pHandle->iqRef * CURRENT_CONV_FACTOR); //转换为MCSDK内部电流单位 
            // 限制电流参考值范围
            if (pHandle->iqRefTran > IQMAX) {
                pHandle->iqRefTran = IQMAX;
            } else if (pHandle->iqRefTran < -IQMAX) {
                pHandle->iqRefTran = -IQMAX;
            }
            
            // 设置电流参考值 (d轴电流设为0)
            Iqd.q = pHandle->iqRefTran;
            Iqd.d = 0;
            MCI_SetCurrentReferences(pSuper->pMCI, Iqd);
    }
}

#ifdef __cplusplus
}
#endif /* __cpluplus */