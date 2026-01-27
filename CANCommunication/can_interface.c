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
#include <string.h>

/* Extra Includes -------------------------------------------------------------*/
#include "arm_math.h"
#include "mc_app_hooks_servo.h"
#include "mc_api.h"
/* Private constants --------------------------------------------------------*/
//#define UID_BASE 0x1FFF7590

/* Private type -------------------------------------------------------------*/

/* Private variables --------------------------------------------------------*/
#define DEFAULT_CAN_ID  CAN_ID_MOTOR_DEFAULT


uint8_t my_can_id = CAN_ID_MOTOR_DEFAULT;              // 本机默认ID
bool motor_start = false;                              // 电机启动标志
bool FaultReset  = false;                              // 故障复位标志  
volatile uint8_t can_rx_flag = 0;                      // 接收标志
CanRxMsg can_rx_buffer;                                // 接收缓冲区

//----------------------------------------------------Blue---------------------
CanTxMsg can_tx_buffer;                // 发送缓冲区
uint8_t canId = CAN_ID_MOTOR_DEFAULT;
uint8_t canMasterId = CAN_ID_DEBUG_UI;
struct motoStatus mtStatus;


/* ======================= SCOPE (CAN DAQ) =======================
 * Protocol summary (rx extended ID):
 *   comm_type = 0x0A (CMD_SCOPE_DATA)
 *   subcmd    = high byte of data2: (data2 >> 8) & 0xFF
 *     0x00 : set bandwidth/sample rate (Hz, 1..1000)
 *     0x02 : start
 *     0x03 : stop
 *     others: set variable IDs, encoded as:
 *         hi_nibble = total_vars_minus_1  (total = (subcmd>>4) + 1), max 8
 *         lo_nibble = start_index (1-based), typically 1 or 5 for <=8 vars
 *         payload contains up to 4x uint16 var IDs (little-endian), in order
 *
 * TX data frames:
 *   1 sample may be split into multiple CAN frames.
 *   First frame of each sample includes timestamp16 at Byte0-1 (HAL_GetTick()&0xFFFF).
 *   Continuation frames contain only data bytes (no timestamp).
 *   Variable bytes are packed strictly in the order configured by the host.
 *   Types/sizes:
 *     uint8/int8 : 1 byte
 *     uint16/int16 : 2 bytes
 *     uint32/int32/float : 4 bytes
 *   String is not supported for sampling.
 * =============================================================== */
#ifndef SCOPE_MAX_VARS
#define SCOPE_MAX_VARS 8u
#endif

typedef enum {
    SCOPE_VT_U8 = 0,
    SCOPE_VT_I8,
    SCOPE_VT_U16,
    SCOPE_VT_I16,
    SCOPE_VT_U32,
    SCOPE_VT_I32,
    SCOPE_VT_F32
} ScopeVarType;

typedef struct {
    uint16_t id;
    uint8_t  type; /* ScopeVarType */
    uint8_t  size; /* bytes: 1/2/4 */
} ScopeVarDesc;

/* ID descriptor table (from host parameter table screenshots, 0x2000~0x2019). */
static const ScopeVarDesc g_scope_desc_tbl[] = {
    {0x2000u, SCOPE_VT_U16, 2u}, /* echoPara1 */
    {0x2001u, SCOPE_VT_U16, 2u}, /* echoPara2 */
    {0x2002u, SCOPE_VT_U16, 2u}, /* echoPara3 */
    {0x2003u, SCOPE_VT_U16, 2u}, /* echoPara4 */
    {0x2004u, SCOPE_VT_U32, 4u}, /* echoFreHz */
    {0x2005u, SCOPE_VT_F32, 4u}, /* MechOffset */
    {0x2006u, SCOPE_VT_F32, 4u}, /* MechPos_init */
    {0x2007u, SCOPE_VT_F32, 4u}, /* limit_torque */
    {0x2008u, SCOPE_VT_F32, 4u}, /* I_FW_MAX */
    {0x2009u, SCOPE_VT_U8,  1u}, /* motor_index */
    {0x200Au, SCOPE_VT_U8,  1u}, /* CAN_ID */
    {0x200Bu, SCOPE_VT_U8,  1u}, /* CAN_MASTER */
    {0x200Cu, SCOPE_VT_U32, 4u}, /* CAN_TIMEOUT */
    {0x200Du, SCOPE_VT_I16, 2u}, /* motorOverTemp */
    {0x200Eu, SCOPE_VT_U32, 4u}, /* overTempTime */
    {0x200Fu, SCOPE_VT_F32, 4u}, /* GearRatio */
    {0x2010u, SCOPE_VT_U8,  1u}, /* Tq_caliType */
    {0x2011u, SCOPE_VT_F32, 4u}, /* cur_filt_gain */
    {0x2012u, SCOPE_VT_F32, 4u}, /* cur_kp */
    {0x2013u, SCOPE_VT_F32, 4u}, /* cur_ki */
    {0x2014u, SCOPE_VT_F32, 4u}, /* spd_kp */
    {0x2015u, SCOPE_VT_F32, 4u}, /* spd_ki */
    {0x2016u, SCOPE_VT_F32, 4u}, /* loc_kp */
    {0x2017u, SCOPE_VT_F32, 4u}, /* spd_filt_gain */
    {0x2018u, SCOPE_VT_F32, 4u}, /* Limit_spd */
    {0x2019u, SCOPE_VT_F32, 4u}, /* limit_cur */
};

typedef struct {
    uint8_t  enabled;
    uint8_t  running;

    /* User configuration */
    uint16_t sample_hz;               /* 1..1000 */
    uint8_t  var_count;               /* 0..SCOPE_MAX_VARS */
    uint16_t var_id[SCOPE_MAX_VARS];  /* configured order (as host sets) */

    /* Precomputed plan (recomputed when var_id changes) */
    uint16_t payload_len;             /* total bytes of all variables (without timestamp) */
    uint8_t  frames_per_sample;       /* number of CAN frames to send for one sample group */

    /* 1kHz scheduler state */
    uint16_t sample_phase_acc;        /* 0..999: sample timing accumulator */
    uint32_t tx_phase_acc;            /* send timing accumulator (0..999 with wrap), allow big increments */

    /* One-sample snapshot + TX state */
    uint16_t last_ts16;               /* timestamp captured at sample moment */
    uint8_t  sample_buf[64];          /* concatenated variable bytes in order (allow cross-frame split) */
    uint16_t sample_buf_len;          /* equals payload_len at capture time */
    uint8_t  tx_in_progress;          /* 1 when there is pending frames to send */
    uint8_t  tx_frame_idx;            /* next frame index to send: 0..frames_per_sample-1 */
} ScopeState;

ScopeState g_scope = {0};

bool Scope_ReadVarBytes(uint16_t id, uint8_t *out, uint8_t size)
{
    /* User should override this function in application to bind real variables. */
    (void)id;
    if (out && size) {
        memset(out, 0, size);
    }
    return false;
}

static uint8_t Scope_GetSizeById(uint16_t id)
{
    for (uint32_t i = 0; i < (uint32_t)(sizeof(g_scope_desc_tbl)/sizeof(g_scope_desc_tbl[0])); i++) {
        if (g_scope_desc_tbl[i].id == id) {
            return g_scope_desc_tbl[i].size;
        }
    }
    /* Default: treat unknown IDs as 4 bytes */
    return 4u;
}

static void Scope_SendFrame(const uint8_t payload[8])
{
    uint8_t data[8];
    memcpy(data, payload, 8u);
    /* status_subcmd = 0x02 indicates scope streaming data */
    CAN_SendScopeData((uint16_t)canMasterId, (uint8_t)my_can_id, 0x02u, data);
}

static void Scope_RecomputePlan(void)
{
    uint16_t len = 0u;

    uint8_t n = g_scope.var_count;
    if (n > SCOPE_MAX_VARS) {
        n = SCOPE_MAX_VARS;
    }

    for (uint8_t i = 0u; i < n; i++) {
        uint16_t id = g_scope.var_id[i];
        if (id == 0u) {
            continue;
        }

        uint8_t sz = Scope_GetSizeById(id);
        if (sz != 1u && sz != 2u && sz != 4u) {
            continue;
        }

        /* Safety: do not exceed snapshot buffer. */
        if ((uint16_t)(len + (uint16_t)sz) > (uint16_t)sizeof(g_scope.sample_buf)) {
            break;
        }
        len = (uint16_t)(len + (uint16_t)sz);
    }

    g_scope.payload_len = len;

    /* One sample group is sent across:
       - frame0: timestamp(2B) + up to 6B data
       - continuation frames: 8B data each
       Variable bytes are allowed to cross frame boundaries. */
    if (len == 0u) {
        g_scope.frames_per_sample = 0u;
    } else if (len <= 6u) {
        g_scope.frames_per_sample = 1u;
    } else {
        uint16_t rem = (uint16_t)(len - 6u);
        g_scope.frames_per_sample = (uint8_t)(1u + (uint8_t)((rem + 7u) / 8u));
    }
}

static void Scope_CaptureSnapshot(void)
{
    /* Capture time + all variable bytes into a contiguous buffer (in host-configured order). */
    g_scope.last_ts16 = (uint16_t)(HAL_GetTick() & 0xFFFFu);

    uint16_t w = 0u;

    uint8_t n = g_scope.var_count;
    if (n > SCOPE_MAX_VARS) {
        n = SCOPE_MAX_VARS;
    }

    for (uint8_t i = 0u; i < n; i++) {
        uint16_t id = g_scope.var_id[i];
        if (id == 0u) {
            continue;
        }

        uint8_t sz = Scope_GetSizeById(id);
        if (sz != 1u && sz != 2u && sz != 4u) {
            continue;
        }

        if ((uint16_t)(w + (uint16_t)sz) > (uint16_t)sizeof(g_scope.sample_buf)) {
            break;
        }

        uint8_t tmp[4] = {0};
        Scope_ReadVarBytes(id, tmp, sz);

        memcpy(&g_scope.sample_buf[w], tmp, sz);
        w = (uint16_t)(w + (uint16_t)sz);
    }

    g_scope.sample_buf_len = w;
    g_scope.payload_len = w;

    /* Recompute frames based on actual captured length (robust against partially configured IDs). */
    if (w == 0u) {
        g_scope.frames_per_sample = 0u;
        g_scope.tx_in_progress = 0u;
        return;
    } else if (w <= 6u) {
        g_scope.frames_per_sample = 1u;
    } else {
        uint16_t rem = (uint16_t)(w - 6u);
        g_scope.frames_per_sample = (uint8_t)(1u + (uint8_t)((rem + 7u) / 8u));
    }

    /* Start TX state machine for this sample. */
    g_scope.tx_frame_idx = 0u;
    g_scope.tx_in_progress = 1u;

    /* Force immediate sending of frame0 at this tick. */
    g_scope.tx_phase_acc = 1000u;
}

static void Scope_SendFrameByIndex(uint8_t frame_idx)
{
    uint8_t payload[8];
    memset(payload, 0, sizeof(payload));

    if (g_scope.sample_buf_len == 0u) {
        return;
    }

    if (frame_idx == 0u) {
        /* First frame carries timestamp at bytes 0..1 and up to 6 bytes data at bytes 2..7. */
        uint16_t ts = g_scope.last_ts16;
        payload[0] = (uint8_t)(ts & 0xFFu);
        payload[1] = (uint8_t)((ts >> 8) & 0xFFu);

        uint16_t copy = g_scope.sample_buf_len;
        if (copy > 6u) {
            copy = 6u;
        }
        memcpy(&payload[2], &g_scope.sample_buf[0], copy);
    } else {
        /* Continuation frames carry pure data (8 bytes each). */
        uint16_t off = (uint16_t)(6u + (uint16_t)(frame_idx - 1u) * 8u);
        if (off >= g_scope.sample_buf_len) {
            return;
        }
        uint16_t remain = (uint16_t)(g_scope.sample_buf_len - off);
        uint16_t copy = remain;
        if (copy > 8u) {
            copy = 8u;
        }
        memcpy(&payload[0], &g_scope.sample_buf[off], copy);
    }

    Scope_SendFrame(payload);
}


/* Called from 1kHz hook (MC_APP_LowFrequencyHook_M1). */
void CAN_Scope_Tick1kHz(void)
{
    if (!g_scope.enabled || !g_scope.running || g_scope.sample_hz == 0u) {
        return;
    }

    /* 1) Sample scheduler: phase accumulator in Hz-domain (DDS). */
    g_scope.sample_phase_acc = (uint16_t)(g_scope.sample_phase_acc + g_scope.sample_hz);
    if (g_scope.sample_phase_acc >= 1000u) {
        g_scope.sample_phase_acc = (uint16_t)(g_scope.sample_phase_acc - 1000u);

        /* At each sample instant, capture timestamp + all variable bytes (one snapshot). */
        Scope_CaptureSnapshot();
    }

    /* 2) TX scheduler: evenly send frames of the latest snapshot within each sample period.
          We advance a phase accumulator with send_rate = sample_hz * frames_per_sample.
          This may send multiple frames in one tick if needed (e.g. high rate / many frames). */
    if (g_scope.tx_in_progress && g_scope.frames_per_sample > 0u) {
        uint32_t send_rate = (uint32_t)g_scope.sample_hz * (uint32_t)g_scope.frames_per_sample;
        g_scope.tx_phase_acc += send_rate;

        while (g_scope.tx_phase_acc >= 1000u && g_scope.tx_in_progress) {
            g_scope.tx_phase_acc -= 1000u;

            if (g_scope.tx_frame_idx < g_scope.frames_per_sample) {
                Scope_SendFrameByIndex(g_scope.tx_frame_idx);
                g_scope.tx_frame_idx++;
            }

            if (g_scope.tx_frame_idx >= g_scope.frames_per_sample) {
                g_scope.tx_in_progress = 0u;
            }
        }
    }
}

static void Scope_HandleSetBandwidth(const uint8_t *rx_data)
{
    /* Hz in little-endian: use uint16 from bytes 0..1 (compat with host sending u8). */
    uint16_t hz = (uint16_t)rx_data[0] | (uint16_t)((uint16_t)rx_data[1] << 8);
    if (hz == 0u) {
        hz = 1u;
    }
    if (hz > 1000u) {
        hz = 1000u;
    }
    g_scope.sample_hz = hz;
}

static void Scope_HandleSetVarIds(uint8_t subcmd, const uint8_t *rx_data)
{
    uint8_t total = (uint8_t)((subcmd >> 4) + 1u); /* 1..16, but we clamp to 8 */
    uint8_t start = (uint8_t)(subcmd & 0x0Fu);     /* 1-based start index */

    if (total > SCOPE_MAX_VARS) {
        total = SCOPE_MAX_VARS;
    }
    if (start == 0u) {
        start = 1u;
    }
    if (start > total) {
        /* invalid, ignore */
        return;
    }

    if (start == 1u) {
        memset(g_scope.var_id, 0, sizeof(g_scope.var_id));
        g_scope.var_count = total;
    } else {
        /* keep existing total if already set */
        if (g_scope.var_count < total) {
            g_scope.var_count = total;
        }
    }

    uint8_t base = (uint8_t)(start - 1u);
    for (uint8_t i = 0u; i < 4u; i++) {
        uint8_t idx = (uint8_t)(base + i);
        if (idx >= g_scope.var_count || idx >= SCOPE_MAX_VARS) {
            break;
        }
        uint16_t id = (uint16_t)rx_data[i*2u] | (uint16_t)((uint16_t)rx_data[i*2u + 1u] << 8);
        if (id == 0u) {
            break;
        }
        g_scope.var_id[idx] = id;
    }


    /* Recompute payload length and frames-per-sample whenever var list changes. */
    Scope_RecomputePlan();

    /* Reset TX state so next sample uses new plan. */
    g_scope.tx_in_progress = 0u;
    g_scope.tx_frame_idx = 0u;
    g_scope.tx_phase_acc = 0u;

    g_scope.enabled = 1u;
}

//----------------------------------------------------Blue---------------------

MotorParams motor_params; // 全局参数实例

extern bool setZeroFlag;
extern FDCAN_HandleTypeDef hfdcan1;

/* Private functions ------------------------------------------------------- */


// ====================== 中断回调函数 ======================
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    FDCAN_RxHeaderTypeDef rx_header;
    
    // 仅读取报文到缓冲区（耗时<2μs）
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, 
                               &rx_header, can_rx_buffer.data) == HAL_OK) {
        can_rx_buffer.ext_id.ext_id = rx_header.Identifier; // 保存完整ID
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
    float jog_spd;
    uint8_t jog_cmd;
    // memcpy(&float_value, data_bytes, sizeof(float));
    // memcpy(&int16_value, data_bytes, sizeof(int16_t));
    uint8_value = data_bytes[4]; // 单字节类型直接用第一个字节
    float_value = *(float *)(data_bytes+4);
    
    ParamWriteResult result = PARAM_WRITE_OK;
    float Parameter_data;
    // 参数写入分支（基于表格索引）
    switch (param_index) {
        // ======== 运行模式 (uint8) ========

        case PARAM_SIN_SWITCH:
            if (uint8_value < 0 || uint8_value > 1) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                // 位置环正弦测试
                if(UserAppID == USER_APP_NORMAL_POS_CTRL)
                {
                    PositionCtrolApp.flags.bits.EnableSineRef = uint8_value;
                }
                // 速度环正弦测试
                if(UserAppID == USER_APP_SPEEDLOOP_BW_TEST)
                {
                    SpeedLoopBWTest.flags.bits.EnableSineRef = uint8_value;
                }
                // 电流环带宽测试
                if(UserAppID == USER_APP_CURRENTLOOP_BW_TEST)
                {
                    CurrentLoopBWTest.flags.bits.EnableSineRef = uint8_value;
                }
            }
            break;

        case PARAM_SIN_FREQ:
            if (float_value < 0 || float_value > 1000.0f) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                // 位置环正弦测试
                if(UserAppID == USER_APP_NORMAL_POS_CTRL)
                {
                    uint16_t freq = (uint16_t)(float_value * SPEED_UNIT); // 频率转换为单位0.1Hz
                    PositionCtrolApp.RefSinStartFreq01Hz = freq ;  // Fs 为时间参数，单位是ms，而传递过来的参数是频率
                    PositionCtrolApp.RefSinEndFreq01Hz = freq ;  // Fs 为时间参数，单位是ms，而传递过来的参数是频率
                }
                // 速度环正弦测试
                if(UserAppID == USER_APP_SPEEDLOOP_BW_TEST)
                {
                    uint16_t freq = (uint16_t)(float_value * SPEED_UNIT); // 频率转换为单位0.1Hz
                    SpeedLoopBWTest.SpdRefSinStartFreq01Hz = freq;
                    SpeedLoopBWTest.SpdRefSinEndFreq01Hz = freq;
                }
                // 电流环带宽测试
                if(UserAppID == USER_APP_CURRENTLOOP_BW_TEST)
                {
                    uint16_t freq = (uint16_t)(float_value * SPEED_UNIT); // 频率转换为单位0.1Hz
                    CurrentLoopBWTest.CurRefSinStartFreq01Hz = freq;
                    CurrentLoopBWTest.CurRefSinEndFreq01Hz = freq;
                }
            }
            break;

        case PARAM_SIN_AMP:
            if (float_value < 0 || float_value > 100.0f) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                // 位置环正弦测试
                if(UserAppID == USER_APP_NORMAL_POS_CTRL)
                {
                    PositionCtrolApp.RefSinAmp = float_value * POS_FACTOR_INV;
                }
                // 速度环正弦测试
                if(UserAppID == USER_APP_SPEEDLOOP_BW_TEST)
                {
                    // 将幅度转换为速度单位 (rad/s -> 0.1Hz)
                    SpeedLoopBWTest.SpdRefSinAmp_SpeedUnit = (int16_t)(float_value * JOG_FACTOR);
                }
                // 电流环带宽测试
                if(UserAppID == USER_APP_CURRENTLOOP_BW_TEST)
                {
                    // 将幅度转换为电流单位 (A -> 内部电流单位)
                    CurrentLoopBWTest.CurRefSinAmp_CurrentUnit = (int16_t)(float_value * CURRENT_CONV_FACTOR);
                }
            }
            break;
            
        case PARAM_RUN_MODE:
            if (uint8_value < MODE_MITCTRL || uint8_value > MODE_JOG) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                motor_params.run_mode = (MotorRunMode)uint8_value;

                switch (motor_params.run_mode) {
                    
                    case MODE_MITCTRL:
                    RequestedUserAppID = USER_APP_MIT_CONTROL;
                    break;

                    case MODE_POSITION:
                        RequestedUserAppID = USER_APP_NORMAL_POS_CTRL;
                        break;
                    
                    case MODE_SPEED:
                        RequestedUserAppID = USER_APP_SPEEDLOOP_BW_TEST;
                        break;

                    case MODE_CURRENT:
                        RequestedUserAppID = USER_APP_CURRENTLOOP_BW_TEST;
                        break;

                    case MODE_HOMING:
                        RequestedUserAppID = USER_APP_HOMING;
                        break;

                    case MODE_JOG:  // 临时转换
                        RequestedUserAppID = USER_APP_JOG;
                        jog_cmd = data_bytes[5];

                        uint16_t jog_spd_data = data_bytes[6] << 8 | data_bytes[7];
                        float jog_spd_cmd = uint_to_float(jog_spd_data, V_MIN, V_MAX, 16);
                        jog_spd = jog_spd_cmd*JOG_FACTOR;
                        if(jog_cmd == 1)
                        {
                            JogApp.flags.bits.MotorOn = true;
                            JogApp.JogSpeed = jog_spd;
                        }
                        else
                        {
                            JogApp.flags.bits.MotorOn = false;
                            JogApp.JogSpeed = 0;
                        }
                        break;
                    default:
                        // RequestedUserAppID is already set to (USER_APP_ID)uint8_value
                        // No further action needed for other modes if they map directly.
                        break;
                }
            }
            break;
            
        // ======== 浮点参数 (float) ========
        case PARAM_CURRENTCTR_IQREF: // 电流参考 (-23~23A)
            if (float_value < -23.0f || float_value > 23.0f) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                // 将A单位的电流转换为内部电流值，传给电流环控制
                int16_t internal_iq_ref = (int16_t)(float_value * CURRENT_CONV_FACTOR);
                
                // 给电流控制的CurrentRef (q轴)
                if (UserAppID == USER_APP_CURRENTLOOP_BW_TEST) {
                    CurrentLoopBWTest.CurrentRef = internal_iq_ref;
                }
            }
            break;

        case PARAM_CURRENTCTR_IDREF: // 电流参考 (-23~23A)
            if (float_value < -23.0f || float_value > 23.0f) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                // 将A单位的电流转换为内部电流值，传给电流环控制
                int16_t internal_id_ref = (int16_t)(float_value * CURRENT_CONV_FACTOR);
                
                // 给电流控制的IdCurrentRef (d轴)
                if (UserAppID == USER_APP_CURRENTLOOP_BW_TEST) {
                    CurrentLoopBWTest.IdCurrentRef = internal_id_ref;
                }
            }
            break;
            
        case PARAM_SPEEDCTR_SPDREF: // 转速参考 (-30~30rad/s)
            if (float_value < -30.0f || float_value > 30.0f) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                // 将rad为单位的浮点数转换为电机内部0.1Hz的值
                int16_t internal_speed_ref = (int16_t)(float_value * JOG_FACTOR);
                
                // 给速度控制的speedref
                if (UserAppID == USER_APP_SPEEDLOOP_BW_TEST) {
                    SpeedLoopBWTest.SpeedRef = internal_speed_ref;
                }

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
        case PARAM_POSCTR_POSREF://位置模式参考位置
            Parameter_data  = *(float *)(data_bytes+4);  // 输出端机械弧度
            // 位置限制在+— 12.56rad 内
            limit(&Parameter_data, -P_MAX, P_MAX);
            PositionCtrolApp.PosRef = Parameter_data * POS_FACTOR_INV;  // 转换成内部量，2pi = 65536
            break;
            
        case PARAM_POSCTR_SPDLIM: // 位置模式速度限制
            Parameter_data  = *(float *)(data_bytes+4);   // 低速端速度 -30rad/s ~ 30rad/s
            limit(&Parameter_data, -V_MAX, V_MAX);
            PositionCtrolApp.pPosGen->setVelocity = Parameter_data * POS_SPD_FACTOR;   // 转换成内部量，2pi rad / s = 65536/1000，9为减速比
            break;
            
        case PARAM_SPEEDCTR_CURLIM: // 速度环电流限制
            if (float_value < 0 || float_value > 23.0f) {
                result = PARAM_OUT_OF_RANGE;
            } else {
                // 限制在-5A到5A范围内
                if (float_value >= IQMAX_A) {
                    float_value = IQMAX_A; // 限制最大值为5A
                }   
                // 将单位A的电流转换为内部电流大小
                int16_t internal_current_limit = (int16_t)(float_value * CURRENT_CONV_FACTOR);
                
                // 通过PID_SetUpperOutputLimit和PID_SetLowerOutputLimit函数将电流限幅至PIDSpeedHandle_M1参数
                PID_SetUpperOutputLimit(&PIDSpeedHandle_M1, internal_current_limit);
                PID_SetLowerOutputLimit(&PIDSpeedHandle_M1, -internal_current_limit);

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

    // 解析接收到的ID
    CanIdUnion rx_id_union;
    rx_id_union.ext_id = can_rx_buffer.ext_id.ext_id;

    if(rxCanIdEx.comm_type != CMD_MOTOR_CTRL)
    {
        canMasterId = rxCanIdEx.data2 & 0X00FF;
    }    
    
    // 忽略非本机消息（广播0xFE除外）
    uint8_t target_id = rx_id_union.id_info.target_id;
    if (target_id != my_can_id && target_id != 0xFE) return;

    // 提取指令类型和主机ID
    // uint8_t cmd_type = rx_id_union.id_info.comm_type;
    // uint16_t host_id = rx_id_union.id_info.data2;
    uint8_t cmd_type = rxCanIdEx.comm_type;
    uint16_t host_id = canMasterId;
    uint8_t* rx_data = can_rx_buffer.data;
    uint8_t mymotorcanid = my_can_id;
    uint8_t resp_data[8];
    switch (cmd_type) {
        // ---- 类型0：获取设备ID ----
        case CMD_GET_ID: {
            memcpy(resp_data,(const void*)UID_BASE,8); //设备ID
            CAN_SendResponseCmdType0(mymotorcanid, resp_data); // 类型0应答
            break;
        }

        // ---- 类型1：运控指令 ----
        case CMD_MOTOR_CTRL:{
            // 解析8字节数据区
            // Byte0~1: 目标角度[0~65535]对应(-4π~4π)
            uint16_t pos_data = (rx_data[0] << 8) | rx_data[1];
            float pos_ref = uint_to_float(pos_data, P_MIN, P_MAX, 16);
            
            // Byte2~3: 目标角速度[0~65535]对应(-30rad/s~30rad/s)
            uint16_t vel_data = (rx_data[2] << 8) | rx_data[3];
            float vel_ref = uint_to_float(vel_data, V_MIN, V_MAX, 16);
            
            // Byte4~5: Kp [0~65535]对应(0.0~500.0)
            uint16_t kp_data = (rx_data[4] << 8) | rx_data[5];
            float kp_ref = uint_to_float(kp_data, KP_MIN, KP_MAX, 16);
            
            // Byte6~7: Kd [0~65535]对应(0.0~5.0)
            uint16_t kd_data = (rx_data[6] << 8) | rx_data[7];
            float kd_ref = uint_to_float(kd_data, KD_MIN, KD_MAX, 16);
            
            // 解析力矩指令：rxCanIdEx.data2 [0~65535]对应转矩12Nm
            uint16_t torque_data = rxCanIdEx.data2;
            float torque_ref = uint_to_float(torque_data, TORQUE_MIN, TORQUE_MAX, 16);
            
            // 赋值给MITControlApp参数
            if(UserAppID == USER_APP_MIT_CONTROL) {
                MITControlApp.PosRef = pos_ref;
                MITControlApp.VelRef = vel_ref;
                MITControlApp.Kp = kp_ref;
                MITControlApp.Kd = kd_ref;
                MITControlApp.TorqueFF = torque_ref;
            }
            
            break;
        }
        // ---- 类型3：电机启动 ----
        case CMD_ENABLE:
            motor_start = true;
            CAN_SendResponseCmdType2(host_id, mymotorcanid); // 应答使能状态
            break;

        // ---- 类型4：电机停止 ----
        case CMD_STOP:
            motor_start = false;
            if(rx_data[0]==0x01)
			{
			   FaultReset = true; // 故障复位
			}
            CAN_SendResponseCmdType2(canMasterId, mymotorcanid); // 应答使能状态

            break;
        // ---- type 0x14: emergency stop ----
        case CMD_EMERGENCY_STOP:
            motor_start = false;
            FaultReset = true;
            MC_StopMotor1();
            MC_AcknowledgeFaultMotor1();
            CAN_SendResponseCmdType2(host_id, mymotorcanid);
            break;

        // ---- type 5: encoder calibration ----
        case CMD_CALI:
            RequestedUserAppID = USER_APP_ENCODER_ALIGNMENT;
            EncoderAlignmentApp.flags.bits.Start = true;
            EncoderAlignmentApp.flags.bits.EnableSave2EE = true;
            resp_data[0] =0x71;
            resp_data[1] =0x7A;
            resp_data[2] =0x72;
            resp_data[3] =0xBC;
            resp_data[4] =0xC0;
            resp_data[5] =0xF9;
            resp_data[6] =0xFF;
            resp_data[7] =0xFF;
            CAN_SendResponseCmdType5(host_id, mymotorcanid,resp_data); // 应答使能状态
            break;

        // ---- 类型6：设机械零位 ----
        case CMD_SET_ZERO:
            if(can_rx_buffer.data[0]==0x01)
			{
            setZeroFlag = true;
            CAN_SendResponseCmdType2(host_id,mymotorcanid);
			}
            break;

        // ---- 类型7：修改CAN ID ----
        case CMD_SET_CANID:
            my_can_id = (rxCanIdEx.data2 & 0XFF00)>>8; 
            ParamManager_RequestParamSaving();//保存变量至flash配置
            memcpy(resp_data,(const void*)UID_BASE,8); //设备ID
            CAN_SendResponseCmdType0(my_can_id, resp_data); // 类型0应答    
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

        
        // ---- 类型10：示波器/数据采集 ----
        case CMD_SCOPE_DATA: {
            /* subcmd in high byte of data2 */
            uint8_t subcmd = (uint8_t)((rxCanIdEx.data2 >> 8) & 0xFFu);

            if (subcmd == 0x00u) {
                /* Set bandwidth/sample rate */
                Scope_HandleSetBandwidth(rx_data);
                static const uint8_t k_scope_ack[8] = {0xABu, 0xAAu, 0xA6u, 0x42u, 0x0Au, 0x1Au, 0x01u, 0x00u};
                memcpy(resp_data, k_scope_ack, sizeof(k_scope_ack));
                SendResponseCmdType0A(host_id, my_can_id, 0x00, resp_data);
            } else if (subcmd == 0x02u) {
                /* Start */
                g_scope.running = 1u;
                g_scope.enabled = 1u;
                g_scope.sample_phase_acc = 0u;
                g_scope.tx_in_progress = 0u;
                g_scope.tx_frame_idx = 0u;
                g_scope.tx_phase_acc = 0u;
                SendResponseCmdType0A(host_id, my_can_id, 0x02, resp_data);
            } else if (subcmd == 0x03u) {
                /* Stop */
                g_scope.running = 0u;
                g_scope.tx_in_progress = 0u;
                g_scope.tx_frame_idx = 0u;
                g_scope.tx_phase_acc = 0u;
                SendResponseCmdType0A(host_id, my_can_id, 0x03, resp_data);
            } else {
                /* Set variable IDs (supports up to 8 vars, multi-frame using start index 1/5) */
                Scope_HandleSetVarIds(subcmd, rx_data);
                subcmd = (uint8_t)(subcmd + 0x10u);
                subcmd = (subcmd & 0xF0) | 0x01;  // 强制低半字节为 0x1
                SendResponseCmdType0A(host_id, my_can_id, subcmd, resp_data);
            }
            break;
        }

        default: // 未知指令处理
            // uint8_t err_code[2] = {0xFF, cmd_type};
            // CAN_SendResponse(cmd_type, host_id, err_code, 2); // 返回错误码
            break;
    }
        can_rx_flag = 0; // 处理完can指令，清除接收标志
}

// ====================== 反馈帧发送函数 ======================
// 通讯类型0：响应帧（目标地址0xFE）
void CAN_SendResponseCmdType0(uint8_t board_can_id, uint8_t* data) {
    FDCAN_TxHeaderTypeDef tx_header;
    CanIdUnion tx_id_union;

    // 构造扩展帧ID
    tx_id_union.id_info.comm_type = CMD_GET_ID;
    tx_id_union.id_info.data2     = board_can_id;
    tx_id_union.id_info.target_id = CAN_ID_BROADCAST;   // 广播地址
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
    
    // ===== 1. 构造29位扩展帧ID =====
    // Bit7~0: 主机CAN ID (target_id)
    tx_id_union.id_info.target_id = host_id;          // Bit7~0: 目标主机ID
    
    // Bit23~8: 数据区2
    uint16_t data2 = 0;
    // Bit8~15: 当前电机CAN ID
    data2 |= (motor_id & 0xFF) << 0;                  // Bit8~15: 电机CAN ID
    
    // Bit16~21: 故障信息（0无1有）
    uint16_t fault_status = MC_GetOccurredFaultsMotor1();
    
    data2 |= ((fault_status & MC_UNDER_VOLT) ? 1 : 0) << 8;  // bit16: 欠压故障
    data2 |= ((fault_status & MC_DP_FAULT) ? 1 : 0) << 9;    // bit17: 过流 (对应MC_DP_FAULT)
    data2 |= ((fault_status & MC_OVER_TEMP) ? 1 : 0) << 10;  // bit18: 过温
    data2 |= 0 << 11;                                        // bit19: 磁编码故障 (暂未实现)
    data2 |= 0 << 12;                                        // bit20: HALL编码故障 (暂未实现)
    data2 |= ((ENCODER_M1.iSCalibrationCompletedFlag == 0) ? 1 : 0) << 13;  // bit21: 未标定
    
    // Bit22~23: 模式状态
    uint8_t mode_state = 0;  // 默认Reset模式
    if (UserAppID == USER_APP_ENCODER_ALIGNMENT || RequestedUserAppID == USER_APP_ENCODER_ALIGNMENT) {
        mode_state = 1;  // Cali模式[标定]
    }
    else if (motor_start == true) {
        mode_state = 2;  // Motor模式[运行]
    }
    data2 |= (mode_state & 0x03) << 14;          // bit22~23: 模式状态
    tx_id_union.id_info.data2 = data2;
    
    // Bit28~24: 通信类型2
    tx_id_union.id_info.comm_type = CMD_MOTOR_STATE;  // Bit28~24: 通信类型2
    tx_id_union.id_info.res = 0;                      // 保留位清零

    // ===== 2. 填充8字节数据区 =====
    // Byte0~1: 当前角度[0~65535]对应(-4π~4π)
    // 使用ENCODER_M1._Super.wMecAngle乘以角度转换系数计算当前角度（优化为乘法提升速度）
    float current_angle = (float)ENCODER_M1._Super.wMecAngle * POS_FACTOR;
    int p_int = float_to_uint(current_angle, P_MIN, P_MAX, 16);
    tx_data[0] = (p_int >> 8) & 0xFF;
    tx_data[1] = p_int & 0xFF;
    
    // Byte2~3: 当前角速度[0~65535]对应(-30rad/s~30rad/s)
    // 使用ENCODER_M1._Super.hAvrMecSpeedUnit乘以速度转换系数计算当前速度（优化为乘法提升速度）
    float current_speed = (float)ENCODER_M1._Super.hAvrMecSpeedUnit * SPD_FACTOR;
    int v_int = float_to_uint(current_speed, V_MIN, V_MAX, 16);
    tx_data[2] = (v_int >> 8) & 0xFF;
    tx_data[3] = v_int & 0xFF;
    
    // Byte4~5: 当前力矩[0~65535]对应(-12Nm~12Nm)
    // 使用MC_GetIqdMotor1_F获取iq和id，iq乘以转换系数转换为转矩
    qd_f_t iqd = MC_GetIqdMotor1_F();
    float current_torque = iqd.q * CUR_FACTOR;  // iq乘以电流转换系数得到转矩
    int t_int = float_to_uint(current_torque, TORQUE_MIN, TORQUE_MAX, 16);
    tx_data[4] = (t_int >> 8) & 0xFF;
    tx_data[5] = t_int & 0xFF;
    
    // Byte6~7: 当前温度：Temp(摄氏度)*10
    // TODO: 需要从温度传感器获取实际温度
    int temperature = 25 * 10;  // 临时使用25°C，需要替换为实际温度
    tx_data[6] = (temperature >> 8) & 0xFF;
    tx_data[7] = temperature & 0xFF;

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

// 通讯类型0A：示波器反馈（目标地址0xFE）

void CAN_SendScopeData(uint16_t host_id, uint8_t motor_id, uint8_t status_subcmd, uint8_t* data)
{
    FDCAN_TxHeaderTypeDef tx_header;
    CanIdUnion tx_id_union;

    /* Build extended ID: comm_type=CMD_SCOPE_DATA, data2 = (status_subcmd<<8)|motor_id, target_id=host_id */
    tx_id_union.id_info.comm_type = CMD_SCOPE_DATA;
    tx_id_union.id_info.data2     = (uint16_t)(((uint16_t)status_subcmd << 8) | (uint16_t)motor_id);
    tx_id_union.id_info.target_id = (uint8_t)host_id;
    tx_id_union.id_info.res       = 0;

    tx_header.Identifier = tx_id_union.ext_id;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    /* Non-blocking enqueue */
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, data);
}

void SendResponseCmdType0A(uint16_t host_id, uint8_t motor_id, uint8_t status_subcmd, uint8_t* data)
{
    FDCAN_TxHeaderTypeDef tx_header;
    CanIdUnion tx_id_union;

    /* Build extended ID: comm_type=CMD_SCOPE_DATA, data2 = (status_subcmd<<8)|motor_id, target_id=host_id */
    tx_id_union.id_info.comm_type = CMD_SCOPE_DATA;
    tx_id_union.id_info.data2     = (uint16_t)(((uint16_t)status_subcmd << 8) | (uint16_t)motor_id);
    tx_id_union.id_info.target_id = (uint8_t)host_id;
    tx_id_union.id_info.res       = 0;

    tx_header.Identifier = tx_id_union.ext_id;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    /* Non-blocking enqueue */
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, data);
}

void CAN_SendResponseCmdType5(uint16_t host_id,uint8_t motor_id,uint8_t* data) {
    FDCAN_TxHeaderTypeDef tx_header;
    CanIdUnion tx_id_union;

    // 构造扩展帧ID
    tx_id_union.id_info.comm_type = CMD_CALI;
    tx_id_union.id_info.data2     = motor_id;
    tx_id_union.id_info.target_id = host_id;   // 广播地址
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