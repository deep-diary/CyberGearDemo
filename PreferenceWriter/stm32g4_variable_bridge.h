/**
  ******************************************************************************
  * @file    stm32g4_variable_bridge.h
  * @brief   Variable to register mapping for STM32G431 flash configuration
  *          模仿Hobbyking_Cheetah_Compact_DRV8323的简洁绑定方式
  ******************************************************************************
  */

#ifndef __STM32G4_VARIABLE_BRIDGE_H
#define __STM32G4_VARIABLE_BRIDGE_H

/* 全局寄存器数组声明（必须在main.c中定义） */
extern float __float_reg[32];
extern int32_t __int32_reg[128];
extern int16_t  __int16_reg[128];

/* 浮点寄存器绑定 */
#define SPEED_SCALE_FACTOR       __float_reg[0]      // 速度比例因子
#define POSITION_DEADZONE        __float_reg[1]      // 位置死区
#define CURRENT_FILTER_COEFF     __float_reg[2]      // 电流滤波系数
#define ENCODER_OFFSET           __float_reg[3]      // 编码器偏移
#define TORQUE_LIMIT             __float_reg[4]      // 扭矩限制
#define VELOCITY_LIMIT           __float_reg[5]      // 速度限制
#define ACCELERATION_LIMIT       __float_reg[6]      // 加速度限制
#define DECELERATION_LIMIT       __float_reg[7]      // 减速度限制

/* 整数寄存器绑定 */
#define MOTOR_DIRECTION          __int32_reg[0]        // 电机方向   未使用
#define CAN_BUS_ID               __int32_reg[1]        // CAN总线ID    未使用
#define CAN_MASTER_ID            __int32_reg[2]        // CAN主站ID   未使用
#define CAN_TIMEOUT_MS           __int32_reg[3]        // CAN超时时间(ms)  未使用
#define ENCODER_RESOLUTION       __int32_reg[4]        // 编码器分辨率  未使用
#define CONTROL_MODE             __int32_reg[5]        // 控制模式      未使用
#define MAX_CURRENT_MA           __int32_reg[6]        // 最大电流(mA)  未使用
#define MAX_VELOCITY_RPM         __int32_reg[7]        // 最大转速(RPM) 未使用

#define MechOffset               __int16_reg[0] 

#endif /* __STM32G4_VARIABLE_BRIDGE_H */