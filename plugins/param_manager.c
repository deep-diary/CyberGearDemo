
/**
 * @file param_manager.c
 * @brief 参数管理模块实现，支持分组、回调、NV存储、CRC校验等。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifndef NULL
#define NULL ((void *)0)
#endif
#include "CRC16.h"
#include "param_manager.h"


// STM32G4 Flash NV存储实现
#include <string.h>
#include "stm32g4xx_hal.h"
/**
 * @brief Flash参数存储区起始地址(需根据芯片实际容量调整)
 */
#define PARAM_FLASH_ADDR ((uint32_t)0x0801F800)


/** @brief Group0参数数量 */
#define PARAM_GROUP0_SIZE (PARAM_GROUP1_START - PARAM_GROUP0_START)
/** @brief Group1参数数量 */
#define PARAM_GROUP1_SIZE (PARAM_GROUP2_START - PARAM_GROUP1_START)
/** @brief Group2参数数量 */
#define PARAM_GROUP2_SIZE (PARAM_GROUP3_START - PARAM_GROUP2_START)
/** @brief Group3参数数量 */
#define PARAM_GROUP3_SIZE (PARAM_ENUM_COUNT - PARAM_GROUP3_START)


static bool ParamSaveRequested = false;




#pragma location = ".ccmram_code"
/**
 * @brief 写入数据到Flash NV存储区
 * @param buf 数据缓冲区指针
 * @param size 字节数
 * @retval true 写入成功
 * @retval false 写入失败
 */
static bool NV_Write(const void *buf, uint32_t size)
{
  if (buf == NULL || size == 0) return false;
  HAL_FLASH_Unlock();
  // 擦除Flash页
  FLASH_EraseInitTypeDef erase;
  uint32_t               pageError = 0;
  erase.TypeErase                  = FLASH_TYPEERASE_PAGES;
  erase.Banks                      = FLASH_BANK_1;
  erase.Page                       = (PARAM_FLASH_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
  erase.NbPages                    = 1;
  __disable_irq();
  if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK) {
    HAL_FLASH_Lock();
    __enable_irq();
    return false;
  }
  __enable_irq();
  // 按字写入
  uint64_t *src  = (uint64_t *)buf;
  uint32_t  addr = PARAM_FLASH_ADDR;
  uint16_t  bulkCount = (size + 7) >> 3;
  for (uint32_t i = 0; i < bulkCount; i++) {
    __disable_irq();
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]) != HAL_OK) {
      HAL_FLASH_Lock();
      __enable_irq();
      return false;
    }
    __enable_irq();
    addr += 8;
  }
  HAL_FLASH_Lock();
  return true;
}


/**
 * @brief 通过idx获取参数描述指针（支持分组）
 * @param idx 参数全局索引（高4位为组，低12位为组内索引）
 * @return 参数描述指针，失败返回NULL
 */
static const ParamDesc_t *get_param_desc_by_idx(uint16_t idx)
{
  uint8_t  group  = (idx >> 12) & 0x0F;
  uint16_t subidx = idx & 0x0FFF;
  uint16_t offset = 0;
  uint16_t size   = 0;
  switch (group) {
    case 0:
      offset = PARAM_GROUP0_START;
      size   = PARAM_GROUP0_SIZE;
      break;
    case 1:
      offset = PARAM_GROUP1_START;
      size   = PARAM_GROUP1_SIZE;
      break;
    case 2:
      offset = PARAM_GROUP2_START;
      size   = PARAM_GROUP2_SIZE;
      break;
    case 3:
      offset = PARAM_GROUP3_START;
      size   = PARAM_GROUP3_SIZE;
      break;
    default:
      return NULL;
  }
  if (subidx >= size) return NULL;
  return &g_param_desc[offset + subidx];
}


/**
 * @brief 参数管理初始化，尝试从NV加载，失败则恢复默认值
 */
void ParamManager_Init(void)
{
  // 先尝试从NV加载
  if (!ParamManager_LoadFromNV()) {
    // 加载失败，恢复默认值
    for (uint16_t i = 0; i < PARAM_ENUM_COUNT; ++i) {
      const ParamDesc_t *desc = &g_param_desc[i];
      switch (desc->attr.type) {
        case PARAM_TYPE_INT8:
          *(int8_t *)desc->var_ptr = (int8_t)desc->default_val.i32;
          break;
        case PARAM_TYPE_UINT8:
          *(uint8_t *)desc->var_ptr = (uint8_t)desc->default_val.u32;
          break;
        case PARAM_TYPE_INT16:
          *(int16_t *)desc->var_ptr = (int16_t)desc->default_val.i32;
          break;
        case PARAM_TYPE_UINT16:
          *(uint16_t *)desc->var_ptr = (uint16_t)desc->default_val.u32;
          break;
        case PARAM_TYPE_INT32:
          *(int32_t *)desc->var_ptr = desc->default_val.i32;
          break;
        case PARAM_TYPE_UINT32:
          *(uint32_t *)desc->var_ptr = desc->default_val.u32;
          break;
        case PARAM_TYPE_FLOAT:
          *(float *)desc->var_ptr = desc->default_val.f32;
          break;
        default:
          break;
      }
    }
  }
}

/**
 * @brief 读取参数值
 * @param idx 参数全局索引
 * @param out_val 输出缓冲区指针
 * @retval true 读取成功
 * @retval false 失败
 */
bool ParamManager_Read(uint16_t idx, void *out_val)
{
  if (out_val) {
    const ParamDesc_t *desc = get_param_desc_by_idx(idx);
    if (!desc) return false;
    if (desc->var_ptr == NULL) {
      const ParamCallbackEntry_t *cb = find_param_callback(idx);
      if (cb && cb->read_cb) {
        return cb->read_cb(out_val);
      }
      return false;
    }
    switch (desc->attr.type) {
      case PARAM_TYPE_INT8:
        *(int8_t *)out_val = *(int8_t *)desc->var_ptr;
        break;
      case PARAM_TYPE_UINT8:
        *(uint8_t *)out_val = *(uint8_t *)desc->var_ptr;
        break;
      case PARAM_TYPE_INT16:
        *(int16_t *)out_val = *(int16_t *)desc->var_ptr;
        break;
      case PARAM_TYPE_UINT16:
        *(uint16_t *)out_val = *(uint16_t *)desc->var_ptr;
        break;
      case PARAM_TYPE_INT32:
        *(int32_t *)out_val = *(int32_t *)desc->var_ptr;
        break;
      case PARAM_TYPE_UINT32:
        *(uint32_t *)out_val = *(uint32_t *)desc->var_ptr;
        break;
      case PARAM_TYPE_FLOAT:
        *(float *)out_val = *(float *)desc->var_ptr;
        break;
      case PARAM_TYPE_STRING:
        strcpy((char *)out_val, (char *)desc->var_ptr);
        break;
      default:
        return false;
    }
    return true;
  } else {
    // out_val为NULL，查找回调
    // const ParamCallbackEntry_t *cb = find_param_callback(idx);
    // if (cb && cb->read_cb) {
    //   return cb->read_cb(NULL);
    // }
    return false;
  }
}

/**
 * @brief 写入参数值
 * @param idx 参数全局索引
 * @param in_val 输入缓冲区指针
 * @retval true 写入成功
 * @retval false 失败
 */
bool ParamManager_Write(uint16_t idx, const void *in_val)
{
  if (in_val) {
    const ParamDesc_t *desc = get_param_desc_by_idx(idx);
    if (!desc) return false;
    if (desc->var_ptr == NULL) {
      const ParamCallbackEntry_t *cb = find_param_callback(idx);
      if (cb && cb->write_cb) {
        return cb->write_cb(in_val);
      }
      return false;
    }
    if (desc->attr.access != PARAM_ACCESS_READWRITE) return false;
    switch (desc->attr.type) {
      case PARAM_TYPE_INT8:
        if (*(int8_t *)in_val < (int8_t)desc->min_val || *(int8_t *)in_val > (int8_t)desc->max_val) return false;
        *(int8_t *)desc->var_ptr = *(int8_t *)in_val;
        break;
      case PARAM_TYPE_UINT8:
        if (*(uint8_t *)in_val < (uint8_t)desc->min_val || *(uint8_t *)in_val > (uint8_t)desc->max_val) return false;
        *(uint8_t *)desc->var_ptr = *(uint8_t *)in_val;
        break;
      case PARAM_TYPE_INT16:
        if (*(int16_t *)in_val < (int16_t)desc->min_val || *(int16_t *)in_val > (int16_t)desc->max_val) return false;
        *(int16_t *)desc->var_ptr = *(int16_t *)in_val;
        break;
      case PARAM_TYPE_UINT16:
        if (*(uint16_t *)in_val < (uint16_t)desc->min_val || *(uint16_t *)in_val > (uint16_t)desc->max_val)
          return false;
        *(uint16_t *)desc->var_ptr = *(uint16_t *)in_val;
        break;
      case PARAM_TYPE_INT32:
        if (*(int32_t *)in_val < (int32_t)desc->min_val || *(int32_t *)in_val > (int32_t)desc->max_val) return false;
        *(int32_t *)desc->var_ptr = *(int32_t *)in_val;
        break;
      case PARAM_TYPE_UINT32:
        if (*(uint32_t *)in_val < (uint32_t)desc->min_val || *(uint32_t *)in_val > (uint32_t)desc->max_val)
          return false;
        *(uint32_t *)desc->var_ptr = *(uint32_t *)in_val;
        break;
      case PARAM_TYPE_FLOAT:
        if (*(float *)in_val < *(float *)&desc->min_val || *(float *)in_val > *(float *)&desc->max_val) return false;
        *(float *)desc->var_ptr = *(float *)in_val;
        break;
      case PARAM_TYPE_STRING:
        strcpy((char *)desc->var_ptr, (const char *)in_val);
        break;
      default:
        return false;
    }
    return true;
  } else {
    // // in_val为NULL，查找回调
    // const ParamCallbackEntry_t *cb = find_param_callback(idx);
    // if (cb && cb->write_cb) {
    //   return cb->write_cb(NULL);
    // }
    return false;
  }
}

/**
 * @brief 从Flash NV存储区加载参数
 * @retval true 加载成功
 * @retval false 校验失败或版本不符
 */
bool ParamManager_LoadFromNV(void)
{
  ParamBlockWithMeta_t *pParaBlock = (ParamBlockWithMeta_t *)PARAM_FLASH_ADDR;
  if (pParaBlock->version != PARAM_BLOCK_VERSION) return false;
  // CRC校验
  uint16_t crc = modbus_crc_calculate((uint8_t *)pParaBlock->param32, sizeof(uint32_t) * PARAM_ENUM_COUNT);
  if (crc != pParaBlock->crc) return false;
  // 按参数描述表写入变量
  uint8_t *p = (uint8_t *)pParaBlock->param32;
  for (uint16_t i = 0; i < PARAM_ENUM_COUNT; ++i) {
    const ParamDesc_t *desc = &g_param_desc[i];
    switch (desc->attr.type) {
      case PARAM_TYPE_INT8:
        *(int8_t *)desc->var_ptr = *(int8_t *)p;
        p += 4;
        break;
      case PARAM_TYPE_UINT8:
        *(uint8_t *)desc->var_ptr = *(uint8_t *)p;
        p += 4;
        break;
      case PARAM_TYPE_INT16:
        *(int16_t *)desc->var_ptr = *(int16_t *)p;
        p += 4;
        break;
      case PARAM_TYPE_UINT16:
        *(uint16_t *)desc->var_ptr = *(uint16_t *)p;
        p += 4;
        break;
      case PARAM_TYPE_INT32:
        *(int32_t *)desc->var_ptr = *(int32_t *)p;
        p += 4;
        break;
      case PARAM_TYPE_UINT32:
        *(uint32_t *)desc->var_ptr = *(uint32_t *)p;
        p += 4;
        break;
      case PARAM_TYPE_FLOAT:
        *(float *)desc->var_ptr = *(float *)p;
        p += 4;
        break;
      default:
        p += 4;
        break;
    }
  }
  return true;
}


/**
 * @brief 保存参数到Flash NV存储区
 * @retval true 保存成功
 * @retval false 失败
 */
bool ParamManager_SaveToNV(void)
{
  union ParamBuffer_t
  {
    ParamBlockWithMeta_t param;
    uint64_t u64Buffer[(sizeof(ParamBlockWithMeta_t) + 7) >> 3];
  } ParamBuffer;
  
  
  ParamBuffer.param.version = PARAM_BLOCK_VERSION;
  uint32_t *p   = ParamBuffer.param.param32;
  for (uint16_t i = 0; i < PARAM_ENUM_COUNT; ++i) {
    const ParamDesc_t *desc = &g_param_desc[i];
    switch (desc->attr.type) {
      case PARAM_TYPE_INT8:
        *p = *(int8_t *)desc->var_ptr;
        break;
      case PARAM_TYPE_UINT8:
        *p = *(uint8_t *)desc->var_ptr;
        break;
      case PARAM_TYPE_INT16:
        *p = *(int16_t *)desc->var_ptr;
        break;
      case PARAM_TYPE_UINT16:
        *p = *(uint16_t *)desc->var_ptr;
        break;
      case PARAM_TYPE_INT32:
        *p = *(int32_t *)desc->var_ptr;
        break;
      case PARAM_TYPE_UINT32:
        *p = *(uint32_t *)desc->var_ptr;
        break;
      case PARAM_TYPE_FLOAT:
        *p = *(float *)desc->var_ptr;
        break;
      default:
        *p = 0xFFFFFFFF;
        break;
    }
    p++;
  }
  ParamBuffer.param.crc = modbus_crc_calculate((uint8_t *)ParamBuffer.param.param32, sizeof(ParamBuffer.param.param32));
  bool success = NV_Write(&ParamBuffer.u64Buffer, sizeof(ParamBuffer));
  ParamSaveRequested = false;
  return success;
}

/**
 * @brief 获取参数总数量
 * @return 参数数量
 */
uint16_t ParamManager_GetParamCount(void) { return (uint16_t)PARAM_ENUM_COUNT; }

/**
 * @brief 获取参数描述指针
 * @param idx 参数全局索引
 * @return 参数描述指针
 */
const ParamDesc_t *ParamManager_GetParamDesc(uint16_t idx) { return get_param_desc_by_idx(idx); }

void ParamManager_RequestParamSaving(void) { ParamSaveRequested = true; }

bool ParamManager_IsParamSavePending(void) { return ParamSaveRequested; }
