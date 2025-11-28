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
#include "param_manager.h"
/* Exported constants --------------------------------------------------------*/
 #define P_MIN -12.5f
 #define P_MAX 12.5f 
 #define V_MIN -30.0f
 #define V_MAX 30.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define TORQUE_MIN -12.0f
 #define TORQUE_MAX  12.0f

 #define POS_FACTOR (TWO_PI/(9 * 65536.0f))  //位置转换系数，65536对应减速前一圈
 #define SPD_FACTOR (TWO_PI/(9 * 10.0f))
 #define CUR_FACTOR (0.87 * 9.0f)              //转矩常数*减速比

 #define JOG_FACTOR (9 * 30.0f / 32768.0f)

/* Exported type -------------------------------------------------------------*/

#define  CAN_ID_MASTER          (0X00)   //控制主机地址 - SPIE
#define  CAN_ID_MOTOR_DEFAULT   (0X02)   //电机默认地址 - 未配置id
#define  CAN_ID_BROADCAST       (0XFE)   //广播地址     - 默认接收地址
#define  CAN_ID_DEBUG_UI        (0XFD)   //调试地址     - 上位机地址

// 参数索引枚举（基于表格）
typedef enum {
    PARAM_SIN_SWITCH     = 0x7001, // Sin on OFF CONTROAL
    PARAM_SIN_FREQ       = 0x7002, // 设置sin测试的频率
    PARAM_SIN_AMP        = 0x7003, // 设置sin测试的幅度
    PARAM_RUN_MODE       = 0x7005, // 控制模式切换
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
    MODE_CURRENT     = 3, // 电流模式
    MODE_HOMING      = 4, // 回零模式
    MODE_JOG         = 7  // JOG模式
} MotorRunMode;

// 参数写入结果枚举
typedef enum {
    PARAM_WRITE_OK      = 0x00, // 写入成功
    PARAM_UNKNOWN_ID    = 0x01, // 未知参数ID
    PARAM_OUT_OF_RANGE  = 0x02, // 值超出范围
    PARAM_READ_ONLY     = 0x03, // 参数只读
    PARAM_TYPE_MISMATCH = 0x04  // 类型不匹配
} ParamWriteResult;

// ----------------------------------------------------Blue---------------------
enum canComMode{   //定义id中24-28位
    CANCOM_ANNOUNCE_DEVID = 0,//通告设备ID
	
    CANCOM_MOTOR_CTRL,       //MOTOR-电机控制
    CANCOM_MOTOR_FEEDBACK,   //MOTOR-电机反馈
	CANCOM_MOTOR_IN,         //MOTOR-进入电机模式
	CANCOM_MOTOR_RESET,      //MOTOR-电机复位模式
	CANCOM_MOTOR_CALI,       //MOTOR-高速编码器标定
	CANCOM_MOTOR_ZERO,       //MOTOR-设置机械零位	
	CANCOM_MOTOR_ID,         //MOTOR-设置ID	
	CANCOM_PARA_WRITE,       //整体参数-写入
	CANCOM_PARA_READ,        //整体参数-读取
	CANCOM_PARA_UPDATE,      //示波器参数-更新上传    10
    CANCOM_OTA_START,        //OTA-启动
    CANCOM_OTA_INFO,         //OTA-升级文件描述
    CANCOM_OTA_ING,          //OTA-升级中
    CANCOM_OTA_END,          //OTA-升级完成	
	CANCOM_CALI_ING,         //编码器标定中
	CANCOM_CALI_RST,         //编码器标定结果
	CANCOM_SDO_READ,     			//sdo 读
	CANCOM_SDO_WRITE,     		//sdo 写
	CANCOM_PARA_STR_INFO,    //参数-字符串信息
    CANCOM_MOTOR_BRAKE,      //MOTOR-进入刹车模式，20
    CANCOM_FAULT_WARN,       //故障和警告信息 
    CANCOM_MODE_TOTAL,
};


// ----------------------------------------------------Blue---------------------


// 指令类型枚举 (10种)
typedef enum {
    CMD_GET_ID            = 0,   // 获取设备ID
    CMD_MOTOR_CTRL        = 1,   // 运控模式指令
    CMD_MOTOR_STATE       = 2,   // 电机运行状态应答
    CMD_ENABLE            = 3,   // 电机使能
    CMD_STOP              = 4,   // 电机停止
    CMD_CALI              = 5,   // 编码器标定
    CMD_SET_ZERO          = 6,   // 设置机械零位
    CMD_SET_CANID         = 7,   // 设置CAN ID
    CMD_SET_ZERINGMODE    = 12,   // 切换回零模式
    CMD_READ_PARAM        = 17,  // 读取参数
    CMD_WRITE_PARAM       = 18,  // 写入参数
    CMD_SAVE_PARAM        = 19,  // 保存参数到Flash
    CMD_FAULT             = 21,  // 故障反馈
    CMD_WRITE_SN          = 22,  // 写入SN
    CMD_SEND_VERSION      = 23   // 发送版本号， 最多是32，因为是5位
} CanCmdType;

// 状态类型枚举
typedef enum {
    STATE_RESET = 0,
    STATE_CALI  = 1,
    STATE_MOTOR = 2
} stateType;

// 电机状态结构体
struct motoStatus{
	unsigned char underVoltFault:1; //欠压故障
	unsigned char overCurFault:1;  //过流故障
	unsigned char overTempFault:1; //过温故障
	unsigned char encoderFault:1;  //编码器故障
	unsigned char ol7Fault:1;     //i2t过载故障
	unsigned char noCaliFault:1;  //未校正磁编故障
	stateType  mtMode:2;	
};

extern struct motoStatus mtStatus;

// 扩展帧ID结构体 (29位协议)
typedef struct {
    uint32_t target_id : 8;   // 目标地址 (Bit0-7)
    uint32_t data2     : 16;  // 数据区2 (Bit8-23)
    CanCmdType comm_type : 5;   // 指令类型 (Bit24-28)
    uint32_t res       : 3;   // 保留位 (Bit29-31)
} ExCanIdInfo;

// ID与uint32_t互转联合体
typedef union {
    uint32_t ext_id;
    ExCanIdInfo id_info;
} CanIdUnion;

// 接收报文结构体
typedef struct {
    CanIdUnion ext_id;
    uint8_t data[8];
} CanRxMsg;

// 发送报文结构体
typedef struct {
    CanIdUnion ext_id;
    uint8_t data[8];
} CanTxMsg;

#define txCanIdEx   (*((ExCanIdInfo*)&(can_tx_buffer.ext_id)))
#define rxCanIdEx   (*((ExCanIdInfo*)&(can_rx_buffer.ext_id))) //将扩展帧id解析为自定义数据结构

#define can_txd()   can_message_transmit(&hfdcan1, &can_tx_buffer)


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
extern uint8_t my_can_id;
/* Exported functions ------------------------------------------------------- */

// 函数声明
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void CAN_ProcessMessages(void);
void CAN_SendResponseCmdType0(uint8_t host_id, uint8_t* data);
void CAN_SendResponseCmdType2(uint16_t host_id,uint8_t motor_id);
void CAN_SendResponseCmdType5(uint16_t host_id,uint8_t motor_id,uint8_t* data);
ParamWriteResult Write_Parameter(uint8_t data_bytes[8]);
void can_message_transmit(FDCAN_HandleTypeDef *hfdcan, CanTxMsg *tx_msg);
void factory_test(void);
void can_broadcast_devInfo(void);
void read_SN(void);
void write_SN_flash(void);
void send_SN_to_master(uint8_t flag);
void send_version_to_master(uint8_t flag);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __mc_config_ext_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/