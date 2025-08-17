/**
  ******************************************************************************
  * @file    mc_config_ext.h,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   Declaration of extra instances
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_INTERFACE_H
#define __CAN_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math_ops.h"
/* Exported constants --------------------------------------------------------*/
 #define P_MIN -12.5f
 #define P_MAX 12.5f 
 #define V_MIN -30.0f
 #define V_MAX 30.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -12.0f
 #define T_MAX  12.0f
/* Exported type -------------------------------------------------------------*/

// 扩展帧ID结构体 (29位协议)
typedef struct {
    uint32_t target_id : 8;   // 目标地址 (Bit0-7)
    uint32_t data2     : 16;  // 数据区2 (Bit8-23)
    uint32_t comm_type : 5;   // 指令类型 (Bit24-28)
    uint32_t res       : 3;   // 保留位 (Bit29-31)
} ExCanIdInfo;

// ID与uint32_t互转联合体
typedef union {
    uint32_t ext_id;
    ExCanIdInfo id_info;
} CanIdUnion;

// 指令类型枚举 (10种)
typedef enum {
    CMD_GET_ID      = 0,   // 获取设备ID
    CMD_MOTOR_CTRL  = 1,   // 运控模式指令
    CMD_MOTOR_STATE = 2,   // 电机状态应答
    CMD_ENABLE      = 3,   // 电机使能
    CMD_STOP        = 4,   // 电机停止
    CMD_SET_ZERO    = 6,   // 设置机械零位
    CMD_SET_CANID   = 7,   // 设置CAN ID
    CMD_READ_PARAM  = 17,  // 读取参数
    CMD_WRITE_PARAM = 18,  // 写入参数
    CMD_SAVE_PARAM  = 19,  // 保存参数到Flash
    CMD_FAULT       = 21   // 故障反馈
} CanCmdType;

// 故障标志结构体
typedef struct {
    bool uncalibrated;   // bit21: 未标定
    bool hall_error;     // bit20: HALL编码故障
    bool mag_error;      // bit19: 磁编码故障
    bool over_temp;      // bit18: 过温
    bool over_current;   // bit17: 过流
    bool under_voltage;  // bit16: 欠压故障
} FaultFlags;

// 参数索引枚举（基于表格）
typedef enum {
    PARAM_RUN_MODE       = 0x7005, // 运控模式
    PARAM_IQ_REF         = 0x7006, // 电流模式Iq指令
    PARAM_SPD_REF        = 0x700A, // 转速模式转速指令
    PARAM_TORQUE_LIMIT   = 0x700B, // 转矩限制
    PARAM_CUR_KP         = 0x7010, // 电流Kp
    PARAM_CUR_KI         = 0x7011, // 电流Ki
    PARAM_CUR_FILT_GAIN  = 0x7014, // 电流滤波系数
    PARAM_LOC_REF        = 0x7016, // 位置模式角度指令
    PARAM_LIMIT_SPD      = 0x7017, // 位置模式速度限制
    PARAM_LIMIT_CUR      = 0x7018, // 速度位置模式电流限制
    PARAM_ROTATION       = 0x701D, // 圈数
    PARAM_LOC_KP         = 0x701E, // 位置Kp
    PARAM_SPD_KP         = 0x701F, // 速度Kp
    PARAM_SPD_KI         = 0x7020  // 速度Ki
} ParamIndex;

// 电机运行模式（与PARAM_RUN_MODE对应）
typedef enum {
    MODE_MOTION_CTRL = 0, // 运控模式
    MODE_POSITION    = 1, // 位置模式
    MODE_SPEED       = 2, // 速度模式
    MODE_CURRENT     = 3  // 电流模式
} MotorRunMode;

// 参数写入结果枚举
typedef enum {
    PARAM_WRITE_OK      = 0x00, // 写入成功
    PARAM_UNKNOWN_ID    = 0x01, // 未知参数ID
    PARAM_OUT_OF_RANGE  = 0x02, // 值超出范围
    PARAM_READ_ONLY     = 0x03, // 参数只读
    PARAM_TYPE_MISMATCH = 0x04  // 类型不匹配
} ParamWriteResult;

// 接收报文结构体
typedef struct {
    uint32_t ext_id;
    uint8_t data[8];
} CanRxMsg;

// 发送报文结构体
typedef struct {
    uint32_t ext_id;
    uint8_t data[8];
} CanTxMsg;

// 参数存储结构体（基于表格）
typedef struct {
    MotorRunMode run_mode;      // 0x7005: 运控模式
    float iq_ref;               // 0x7006: 电流Iq参考 (A)
    float spd_ref;              // 0x700A: 转速参考 (rad/s)
    float torque_limit;          // 0x700B: 转矩限制 (Nm)
    float cur_kp;               // 0x7010: 电流Kp
    float cur_ki;               // 0x7011: 电流Ki
    float cur_filt_gain;        // 0x7014: 电流滤波系数
    float loc_ref;              // 0x7016: 位置参考 (rad)
    float limit_spd;            // 0x7017: 位置模式速度限制 (rad/s)
    float limit_cur;            // 0x7018: 速度位置模式电流限制 (A)
    int16_t rotation_count;     // 0x701D: 圈数
    float loc_kp;               // 0x701E: 位置Kp
    float spd_kp;               // 0x701F: 速度Kp
    float spd_ki;               // 0x7020: 速度Ki
} MotorParams;

/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

// 函数声明
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void CAN_ProcessMessages(void);
void CAN_SendResponseCmdType0(uint16_t host_id, uint8_t* data);
void CAN_SendResponseCmdType2(uint16_t host_id,uint8_t motor_id);
ParamWriteResult Write_Parameter(uint8_t data_bytes[8]);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __mc_config_ext_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/