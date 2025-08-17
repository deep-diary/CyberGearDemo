/**
 ******************************************************************************
 * @file    can_interface.c
 * @author  Motor Control Competence Center, ST Microelectronics
 * @brief   extra instance creation
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
/* Includes ------------------------------------------------------------------*/
#include "can_interface.h"

/* Extra Includes -------------------------------------------------------------*/
#include "arm_math.h"

/* Private constants --------------------------------------------------------*/
#define UID_BASE 0x1FFF7590

/* Private type -------------------------------------------------------------*/

/* Private variables --------------------------------------------------------*/

uint8_t my_can_id = 0x01;              // 本机默认ID
volatile uint8_t can_rx_flag = 0;      // 接收标志
CanRxMsg can_rx_buffer;                // 接收缓冲区

MotorParams motor_params; // 全局参数实例

extern uint8_t motorOn;
extern float32_t posRef;
extern float32_t speedMax;

extern FDCAN_HandleTypeDef hfdcan1;

/* Private functions ------------------------------------------------------- */

// ====================== 中断回调函数 ======================
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    FDCAN_RxHeaderTypeDef rx_header;
    
    // 仅读取报文到缓冲区（耗时<2μs）
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, 
                               &rx_header, can_rx_buffer.data) == HAL_OK) {
        can_rx_buffer.ext_id = rx_header.Identifier; // 保存完整ID
        can_rx_flag = 1;  // 触发主循环处理
    }
}


/**
 * @brief 写入参数值（带范围校验）
 * @param param_index 参数索引（16位）
 * @param data_bytes 参数数据（4字节数组）
 * @return 写入结果（ParamWriteResult）
 */
ParamWriteResult Write_Parameter(uint8_t data_bytes[8]) {
    
    uint16_t param_index = (data_bytes[1] << 8) | data_bytes[0];  // Byte0-1: 参数索引    
    
    // 转换4字节数组到实际类型
    float float_value;
    int16_t int16_value;
    uint8_t uint8_value;
    memcpy(&float_value, data_bytes, sizeof(float));
    memcpy(&int16_value, data_bytes, sizeof(int16_t));
    uint8_value = data_bytes[0]; // 单字节类型直接用第一个字节
    
    ParamWriteResult result = PARAM_WRITE_OK;
    float Parameter_data;
    // 参数写入分支（基于表格索引）
    switch (param_index) {
        // ======== 运行模式 (uint8) ========
        case PARAM_RUN_MODE:
            if (uint8_value < MODE_MOTION_CTRL || uint8_value > MODE_CURRENT) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                motor_params.run_mode = (MotorRunMode)uint8_value;
            }
            break;
            
        // ======== 浮点参数 (float) ========
        case PARAM_IQ_REF: // 电流参考 (-23~23A)
            if (float_value < -23.0f || float_value > 23.0f) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                motor_params.iq_ref = float_value;
            }
            break;
            
        case PARAM_SPD_REF: // 转速参考 (-30~30rad/s)
            if (float_value < -30.0f || float_value > 30.0f) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                motor_params.spd_ref = float_value;
            }
            break;
            
        case PARAM_TORQUE_LIMIT: // 转矩限制 (0~12Nm)
            if (float_value < 0 || float_value > 12.0f) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                motor_params.torque_limit = float_value;
            }
            break;
            
        // ======== 只读参数校验 ========
        case 0x7019: // mechPos (只读)
        case 0x701A: // iqf (只读)
        case 0x701B: // mechVel (只读)
        case 0x701C: // VBUS (只读)
            result = PARAM_READ_ONLY;
            break;
            
        // ======== 16位整型参数 ========
        case PARAM_ROTATION: // 圈数
            motor_params.rotation_count = int16_value;
            break;
            
        // ======== 默认有范围要求的浮点参数 ========
        case PARAM_CUR_FILT_GAIN: // 电流滤波系数 (0~1.0)
            if (float_value < 0 || float_value > 1.0f) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                motor_params.cur_filt_gain = float_value;
            }
            break;
            
        // ======== 其他浮点参数（无特定范围限制） ========
        case PARAM_LOC_REF://位置模式参考位置

            Parameter_data  = *(float *)(data_bytes+4);
            posRef = Parameter_data*9;
            break;
            
        case PARAM_LIMIT_SPD: // 位置模式速度限制
            Parameter_data  = *(float *)(data_bytes+4);
            speedMax = Parameter_data*9;
        if (speedMax > 270.0)
           {
            speedMax = 270.0;
           }
           else if (speedMax < -270.0) 
           {
            speedMax = -270.0;
           }
            break;
            
        case PARAM_LIMIT_CUR: // 电流限制
            if (float_value < 0 || float_value > 23.0f) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                motor_params.limit_cur = float_value;
            }
            break;
            
        case PARAM_CUR_KP:
            motor_params.cur_kp = float_value;
            break;
            
        case PARAM_CUR_KI:
            motor_params.cur_ki = float_value;
            break;
            
        case PARAM_LOC_KP:
            motor_params.loc_kp = float_value;
            break;
            
        case PARAM_SPD_KP:
            motor_params.spd_kp = float_value;
            break;
            
        case PARAM_SPD_KI:
            motor_params.spd_ki = float_value;
            break;
            
        default:
            result = PARAM_UNKNOWN_ID; // 未知参数ID
    }
    
    return result;
}

/* Global functions ------------------------------------------------------- */

// CAN报文处理函数
void CAN_ProcessMessages(void) {
    if (!can_rx_flag) return;
    can_rx_flag = 0;

    // 解析接收到的ID
    CanIdUnion rx_id_union;
    rx_id_union.ext_id = can_rx_buffer.ext_id;
    
    // 忽略非本机消息（广播0xFE除外）
    uint8_t target_id = rx_id_union.id_info.target_id;
    if (target_id != my_can_id && target_id != 0xFE) return;

    // 提取指令类型和主机ID
    uint8_t cmd_type = rx_id_union.id_info.comm_type;
    uint16_t host_id = rx_id_union.id_info.data2;
    uint8_t* rx_data = can_rx_buffer.data;
    uint8_t mymotorcanid;

switch (cmd_type) {
        // ---- 类型0：获取设备ID ----
        case CMD_GET_ID: {
            uint32_t UID[3] = {0};// 读取芯片唯一ID
            UID[0] = HAL_GetUIDw0();
            UID[1] = HAL_GetUIDw1();
            UID[2] = HAL_GetUIDw2();
            uint8_t resp_data[8] = {
                // UID[0] 低字节在前 (小端模式)
                (uint8_t)(UID[0]),        (uint8_t)(UID[0] >> 8),
                (uint8_t)(UID[0] >> 16),  (uint8_t)(UID[0] >> 24),
                // UID[1] 低字节在前 (小端模式)
                (uint8_t)(UID[1]),        (uint8_t)(UID[1] >> 8),
                (uint8_t)(UID[1] >> 16),  (uint8_t)(UID[1] >> 24)
            };
            host_id = my_can_id;//按照手册应答帧格式更新本机ID
            CAN_SendResponseCmdType0(host_id, resp_data); // 类型0应答
            break;
        }

        // ---- 类型1：运控指令 ----
        case CMD_MOTOR_CTRL:{

            // 无需立即应答（由周期反馈帧响应）

            break;
        }
        // ---- 类型3：电机使能 ----
        case CMD_ENABLE:
            motorOn =true;
            mymotorcanid = my_can_id;//按照手册应答帧格式更新本机ID
            CAN_SendResponseCmdType2(host_id, mymotorcanid); // 应答使能状态
            break;

        // ---- 类型4：紧急停止 ----
        case CMD_STOP:
            motorOn =false;
           mymotorcanid = my_can_id;//按照手册应答帧格式更新本机ID
            CAN_SendResponseCmdType2(host_id, mymotorcanid); // 应答使能状态
            break;

        // ---- 类型6：设机械零位 ----
        case CMD_SET_ZERO:
            //Motor_CalibrateZeroPoint();
            //CAN_SendResponse(CMD_SET_ZERO, host_id, NULL, 0); // 空数据应答
            break;

        // ---- 类型7：修改CAN ID ----
        case CMD_SET_CANID:
            //my_can_id = rx_data[0]; // 新ID存首字节
            //SaveConfigToFlash();    // 保存配置
            //CAN_SendResponse(CMD_SET_CANID, host_id, &my_can_id, 1); // 回传新ID
            break;

        // ---- 类型17：参数读取 ----
        case CMD_READ_PARAM: {
            // uint8_t param_id = rx_data[0];
            // float param_val = ReadMotorParam(param_id);
            // uint8_t resp[4];
            // memcpy(resp, &param_val, 4); // 浮点转字节流
            // CAN_SendResponse(CMD_READ_PARAM, host_id, resp, 4); // 返回参数值
            break;
        }

        // ---- 类型18：参数写入 ----
        case CMD_WRITE_PARAM: {

            // 调用参数写入函数
            ParamWriteResult result = Write_Parameter(rx_data);
            mymotorcanid = my_can_id;
            CAN_SendResponseCmdType2(host_id, mymotorcanid); // 应答使能状态
            break;
        }

        // ---- 类型19：保存参数到Flash ★ 关键新增 ----
        case CMD_SAVE_PARAM:
            // SaveAllParamsToFlash(); // 耗时操作（避免在中断调用）
            // CAN_SendResponse(CMD_SAVE_PARAM, host_id, (uint8_t[]){0xDD}, 1); // 保存成功
            break;

        // ---- 类型21：故障反馈（仅主动发送，不处理接收） ----
        case CMD_FAULT:
            // 主机下发的故障帧忽略（电机自身触发故障上报）
            break;

        default: // 未知指令处理
            // uint8_t err_code[2] = {0xFF, cmd_type};
            // CAN_SendResponse(cmd_type, host_id, err_code, 2); // 返回错误码
            break;
    }
}

// ====================== 反馈帧发送函数 ======================
// 通讯类型0：响应帧（目标地址0xFE）
void CAN_SendResponseCmdType0(uint16_t host_id, uint8_t* data) {
    FDCAN_TxHeaderTypeDef tx_header;
    CanIdUnion tx_id_union;

    // 构造扩展帧ID
    tx_id_union.id_info.comm_type = CMD_GET_ID;
    tx_id_union.id_info.data2     = host_id;
    tx_id_union.id_info.target_id = 0xFE;   // 广播地址
    tx_id_union.id_info.res       = 0;

    // 填充帧头
    tx_header.Identifier  = tx_id_union.ext_id;
    tx_header.IdType      = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength  = FDCAN_DLC_BYTES_8;      // 数据长度（单位：字节）
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;  

    // 非阻塞发送
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, data);
}

extern float_t theta_mech ;
extern float_t dtheta_mech ;
extern qd_f_t i_q ;
//int float_to_uint(float x, float x_min, float x_max, int bits);
/**
 * @brief 发送类型2反馈帧（电机运行状态）
 * @param host_id 主机CAN ID（从接收帧中提取）
 * @param motor_can_id 当前电机CAN ID
 * @param fault_flags 故障标志位（6位掩码）
 * @param mode_state 模式状态（0=Reset, 1=Cali, 2=Motor）
 * @param current_angle 当前角度（弧度）
 * @param current_speed 当前角速度（rad/s）
 * @param current_torque 当前力矩（Nm）
 * @param temperature 当前温度（摄氏度）
 */
void CAN_SendResponseCmdType2(uint16_t host_id,uint8_t motor_id) {
    FDCAN_TxHeaderTypeDef tx_header;
    CanIdUnion tx_id_union;
    uint8_t tx_data[8];  // 8字节数据区
    uint8_t mode_state =2;

    // 初始化故障标志结构体
    FaultFlags fault_flags = {
        .uncalibrated = false,
        .hall_error = false,
        .mag_error = false,
        .over_temp = false,
        .over_current = false,
        .under_voltage = false
    };

    // ===== 1. 构造29位扩展帧ID =====
    tx_id_union.id_info.comm_type = CMD_MOTOR_STATE;  // 通信类型2
    tx_id_union.id_info.target_id = host_id;          // 目标主机ID
    tx_id_union.id_info.res       = 0;
    
    // 构造数据区2（16位：bit23~8）
    uint16_t data2 = 0;
    // Bit8~15: 当前电机CAN ID
    data2 |= motor_id ;
    
    // Bit16~21: 故障信息（按bit位定义）
    data2 |= (fault_flags.under_voltage ? 1 : 0) << 8; // bit16: 欠压故障
    data2 |= (fault_flags.over_current ? 1 : 0) << 9;  // bit17: 过流
    data2 |= (fault_flags.over_temp ? 1 : 0) << 10;     // bit18: 过温
    data2 |= (fault_flags.mag_error ? 1 : 0) << 11;     // bit19: 磁编码故障
    data2 |= (fault_flags.hall_error ? 1 : 0) << 12;    // bit20: HALL编码故障
    data2 |= (fault_flags.uncalibrated ? 1 : 0) << 13;  // bit21: 未标定
  
    // Bit22~23: 模式状态（2位）
    data2 |= (mode_state & 0x03) << 14;   // 0x03 = 00000011
    
    tx_id_union.id_info.data2 = data2;
    tx_id_union.id_info.res = 0;          // 保留位清零

    // ===== 2. 填充8字节数据区 =====
    int p_int = float_to_uint(theta_mech, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(dtheta_mech, V_MIN, V_MAX, 16);
    int t_int = float_to_uint(i_q.q, T_MIN, T_MAX, 16);  // =（torque/12）*32767+32767
        //if(pos.MotorType==0) t_int = float_to_uint(t, -10.8f, 10.8f, 16);
        //else if(pos.MotorType==1) t_int = float_to_uint(t, -8.4f, 8.4f, 16);
    int temperature =25*10;


    tx_data[0] = p_int>>8;
    tx_data[1] = p_int;//Byte0~1: 当前角度（-4π~4π → 0~65535）
    tx_data[2] = v_int>>8;
    tx_data[3] = v_int;// Byte2~3: 当前角速度（-30~30 rad/s → 0~65535）
    tx_data[4] = t_int>>8;
    tx_data[5] = t_int;// Byte4~5: 当前力矩（-12~12 Nm → 0~65535）
    tx_data[6] = temperature>>8;
    tx_data[7] = temperature;// Byte6~7: 当前温度

    // ===== 3. 配置帧头参数 =====
    tx_header.Identifier  = tx_id_union.ext_id;
    tx_header.IdType      = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength  = FDCAN_DLC_BYTES_8;      // 数据长度（单位：字节）
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;  

    // ===== 4. 发送帧 =====
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
}

