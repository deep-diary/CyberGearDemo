
#ifndef PARAM_MANAGER_H
#define PARAM_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#define PARAM_BLOCK_VERSION 0x00010001u
#define SOFTWARE_VERSION "20251007"

// 参数名枚举，带组别前缀，并定义每组起始索引
typedef enum {
  // Group0
  PARAM_GROUP0_START = 0,
  PARAM_0000_CAN_ID  = PARAM_GROUP0_START,
  PARAM_0001_ENC_ZERO_OFFSET,
  PARAM_0002_ENC_DIR,
  PARAM_GROUP0_END,

  // Group1
  PARAM_GROUP1_START = PARAM_GROUP0_END,
  // ...
  PARAM_GROUP1_END   = PARAM_GROUP1_START,

  // Group2
  PARAM_GROUP2_START = PARAM_GROUP1_END,
  // ...
  PARAM_GROUP2_END   = PARAM_GROUP2_START,

  // Group3
  PARAM_GROUP3_START = PARAM_GROUP2_END,
  // ...
  PARAM_GROUP3_END   = PARAM_GROUP3_START,

  PARAM_ENUM_COUNT = PARAM_GROUP3_END
} ParamName_t;

// 参数类型枚举（32位以内类型+字符串）
typedef enum {
  PARAM_TYPE_INT8   = 0,
  PARAM_TYPE_UINT8  = 1,
  PARAM_TYPE_INT16  = 2,
  PARAM_TYPE_UINT16 = 3,
  PARAM_TYPE_INT32  = 4,
  PARAM_TYPE_UINT32 = 5,
  PARAM_TYPE_FLOAT  = 6,
  PARAM_TYPE_STRING = 7,  // 字符串类型
                          // 可扩展
} ParamType_t;

// 参数访问权限枚举
typedef enum {
  PARAM_ACCESS_READONLY  = 0,
  PARAM_ACCESS_READWRITE = 1,
  // 可扩展
} ParamAccess_t;

// 参数属性位域
typedef struct {
  uint32_t type     : 4;  // 0-15
  uint32_t access   : 2;  // 0-3
  uint32_t reserved : 2;  // 保留
} ParamAttr_t;

// 单个参数描述结构体
typedef struct {
  void       *var_ptr;  // 指向参数变量的指针
  uint32_t    min_val;  // 最小值（如为float，写入时强转）
  uint32_t    max_val;  // 最大值（如为float，写入时强转）
  ParamAttr_t attr;     // 属性
  union {
    int32_t  i32;
    uint32_t u32;
    float    f32;
  } default_val;
} ParamDesc_t;

// 参数块结构体，第一个元素为参数版本号，末尾2字节为CRC
typedef struct {
  uint32_t version;
  uint32_t param32[PARAM_ENUM_COUNT];
  uint16_t crc;
} ParamBlockWithMeta_t;

// 参数回调结构体
typedef bool (*ParamReadCallback)(void *out_val);
typedef bool (*ParamWriteCallback)(const void *in_val);
typedef struct {
  uint16_t           param_id;
  ParamReadCallback  read_cb;
  ParamWriteCallback write_cb;
} ParamCallbackEntry_t;

extern const ParamDesc_t g_param_desc[];

// 参数管理接口
void ParamManager_Init(void);
bool ParamManager_Read(uint16_t idx, void *out_val);
bool ParamManager_Write(uint16_t idx, const void *in_val);
bool ParamManager_LoadFromNV(void);
bool ParamManager_SaveToNV(void);
const ParamCallbackEntry_t *find_param_callback(uint16_t param_id);

// 获取参数数量
uint16_t ParamManager_GetParamCount(void);

// 获取参数描述
const ParamDesc_t *ParamManager_GetParamDesc(uint16_t idx);

void ParamManager_RequestParamSaving(void);

bool ParamManager_IsParamSavePending(void);


#endif  // PARAM_MANAGER_H
