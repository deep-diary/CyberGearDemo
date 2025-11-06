#include "param_manager.h"
#include <stddef.h>
#include "can_interface.h"
#include "mc_config.h"

// 示例：可扩展
const ParamCallbackEntry_t g_param_callback_table[] = {
     { PARAM_0000_CAN_ID, NULL, NULL },
     { PARAM_0001_ENC_ZERO_OFFSET, NULL, NULL },
    // ...
};


// 参数描述表（按组顺序排列，组0有3个参数，组1/2/3可扩展）
// 需在头文件定义参数变量，参数描述表用外部变量指针

const ParamDesc_t g_param_desc[] = {
    // var_ptr, min_val, max_val, attr, default_val（严格顺序初始化，union直接用数值）
    {&my_can_id, 0, 127, {PARAM_TYPE_UINT8, PARAM_ACCESS_READWRITE, 0}, {.i32 = 127}},
    {&ENCODER_M1.zeroAngleOffset, -32768, 32767, {PARAM_TYPE_INT16, PARAM_ACCESS_READWRITE, 0}, {.i32 = 0}},
    {&ENCODER_M1.direction, -1, 1, {PARAM_TYPE_INT16, PARAM_ACCESS_READONLY, 0}, {.i32 = 1}},
    {&ENCODER_M1.hAngleErrorCRC, 0, 65535, {PARAM_TYPE_UINT16, PARAM_ACCESS_READWRITE, 0}, {.u32 = 0}},
    {&ENCODER_M1.CalibrationCompletedFlag, 0, 1, {PARAM_TYPE_UINT8, PARAM_ACCESS_READWRITE, 0}, {.u32 = 0}},
    {&ENCODER_M1.iSCalibrationCompletedFlag, 0, 65535, {PARAM_TYPE_UINT16, PARAM_ACCESS_READWRITE, 0}, {.u32 = 0}},
};


/* Put this function here so that it can be aware of the size of g_param_callback_table*/
const ParamCallbackEntry_t *find_param_callback(uint16_t param_id)
{
  for (size_t i = 0; i < sizeof(g_param_callback_table) / sizeof(g_param_callback_table[0]); ++i) {
    if (g_param_callback_table[i].param_id == param_id) return &g_param_callback_table[i];
  }
  return NULL;
}
