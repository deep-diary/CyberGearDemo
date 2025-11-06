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

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Private constants --------------------------------------------------------*/
#define DEFAULT_KT_VALUE     (100)  // 默认转矩常数，需要根据实际电机调整

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
    
    // 重置控制参数
    pHandle->PosRef = 0;
    pHandle->VelRef = 0;
    pHandle->Kp = 2000;    
    pHandle->Kd = 15000;     
    pHandle->TorqueFF = 0;
    pHandle->Kt = DEFAULT_KT_VALUE;
    
    // 重置状态变量
    pHandle->CurrentPosition = 0;
    pHandle->CurrentVelocity = 0;
    pHandle->TorqueRef = 0;
    
    // 重置标志位
    pHandle->flags.all = 0;
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
}

/**
  * @brief  MIT控制应用后台更新函数
  * @param  pSuper: 用户应用句柄指针
  * @retval None
  */
void MITControlApp_OnBackground(UserApplication_Handle_t* pSuper)
{
    MITControlApp_Handle_t* pHandle = (MITControlApp_Handle_t*)pSuper;

    switch (MCI_GetSTMState(pSuper->pMCI))
    {
    case IDLE:
        if (pHandle->flags.bits.MotorOn) {
            // 在IDLE状态时，读取当前位置作为PosRef，保证运动控制刚使能时停在原地
            pHandle->PosRef = MCI_GetCurrentPosition(pSuper->pMCI);
            pSuper->pMCI->LastModalitySetByUser = MCM_TORQUE_MODE;           // 切换到转矩控制模式
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
        pHandle->CurrentPosition = MCI_GetCurrentPosition(pSuper->pMCI);
        pHandle->CurrentVelocity = MCI_GetAvrgMecSpeedUnit(pSuper->pMCI)*6.28/10;//转换为rad/s
        
        // MIT控制算法: torque_ref = Kp*(p_des - theta_mech) + TorqueFF + Kd*(v_des - dtheta_mech)
        pos_error = pHandle->PosRef - pHandle->CurrentPosition;
        vel_error = pHandle->VelRef - pHandle->CurrentVelocity;
        
        // 计算转矩参考值
        pos_term = (pHandle->Kp * pos_error) / 4000;  
        vel_term = (pHandle->Kd * vel_error) ; 
        
        pHandle->TorqueRef = pos_term + pHandle->TorqueFF + vel_term;
        
        // 将转矩参考值转换为电流参考值: i_q_ref = torque_ref / Kt
        if (pHandle->Kt != 0) {
            int16_t i_q_ref = (int16_t)(pHandle->TorqueRef / pHandle->Kt);
            
            // 限制电流参考值范围
            if (i_q_ref > IQMAX) {
                i_q_ref = IQMAX;
            } else if (i_q_ref < -IQMAX) {
                i_q_ref = -IQMAX;
            }
            
            // 设置电流参考值 (d轴电流设为0)
            Iqd.q = i_q_ref;
            Iqd.d = 0;
            MCI_SetCurrentReferences(pSuper->pMCI, Iqd);
        }
    }
}

#ifdef __cplusplus
}
#endif /* __cpluplus */