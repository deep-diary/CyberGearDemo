/**
  ******************************************************************************
  * @file    MITControlApp.h,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   Declaration of MIT motion control application
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

#ifndef __MITControlApp_h
#define __MITControlApp_h

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_interface.h"
#include "user_application.h"

/* Exported constants --------------------------------------------------------*/
#define GR 9.0f                 //Gear ratio
#define KT_OUT 1.0f            //KT*GR
/* Exported Variables --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct {
  UserApplication_Handle_t _Super;

  /* MIT控制参数 */
  float_t PosRef;           // 位置参考值
  float_t VelRef;           // 速度参考值
  float_t Kp;              // 位置比例增益
  float_t Kd;              // 速度微分增益
  float_t TorqueFF;         // 前馈转矩
  
  /* 状态变量 */
  float_t CurrentPosition;  // 当前位置
  float_t CurrentVelocity;  // 当前速度
  float_t TorqueRef;        // 计算得到的转矩参考值
  float_t iqRef;
  int16_t iqRefTran;        //将以A为单位的电流转换为MCSDK内部电流
  
  union 
  {
    uint16_t all;
    struct
    {
      uint16_t MotorOn : 1;
      uint16_t FaultReset : 1;
    } bits;
  } flags;
  
} MITControlApp_Handle_t;

/* Exported functions ------------------------------------------------------- */
void MITControlApp_OnReset(UserApplication_Handle_t* pSuper);
void MITControlApp_OnStart(UserApplication_Handle_t* pSuper);
void MITControlApp_OnExit(UserApplication_Handle_t* pSuper);
void MITControlApp_OnBackground(UserApplication_Handle_t* pSuper);
void MITControlApp_OnLowFrequencyUpdate(UserApplication_Handle_t* pSuper);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif