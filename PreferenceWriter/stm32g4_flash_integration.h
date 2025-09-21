/**
  ******************************************************************************
  * @file    stm32g4_flash_integration.h
  * @brief   Integrated flash storage solution for STM32G431
  ******************************************************************************
  */

#ifndef __STM32G4_FLASH_INTEGRATION_H
#define __STM32G4_FLASH_INTEGRATION_H

#include "stm32g4xx_hal.h"  // 包含HAL库定义
#include <stdint.h>         // 包含标准整数类型
#include <stdbool.h>        // 包含布尔类型

#ifdef __cplusplus
extern "C" {
#endif

/* Flash configuration */
#define FLASH_BASE                 (0x08000000UL) /**< FLASH base address */
#define FLASH_PAGE_SIZE            0x800U         /**< Flash page size (2KB) */
#define FLASH_USER_CONFIG_PAGE     63   /**< STM32G431最后一页用于配置存储（共64页，0-63） */
#define FLASH_MAX_INT16_INDEX      128  /**< 最大int16索引 */
#define FLASH_MAX_INT_INDEX        128  /**< 最大整数索引 */
#define FLASH_MAX_FLOAT_INDEX      32   /**< 最大浮点索引 */

/**
  * @brief  Preference存储结构体
  */
typedef struct {
    uint32_t page;          /**< Flash存储页面 */
    uint32_t max_index;     /**< 页面内最大索引 */
    bool ready;             /**< 就绪状态 */
} PreferenceWriter_t;

// 全局配置实例
extern PreferenceWriter_t g_preferenceWriter;

// 全局寄存器数组（在main.c中定义）
extern int16_t __int16_reg[128];
extern int32_t __int32_reg[128];
extern float __float_reg[32];

/* Core flash functions */

/**
  * @brief  初始化PreferenceWriter实例
  * @param  writer: PreferenceWriter实例指针
  * @param  page: 使用的flash页面号
  */
void stm32g4_preferenceInit(PreferenceWriter_t* writer, uint32_t page);

/**
  * @brief  打开PreferenceWriter（验证页面有效性）
  * @param  writer: PreferenceWriter实例指针
  */
void stm32g4_preferenceOpen(PreferenceWriter_t* writer);

/**
  * @brief  检查PreferenceWriter是否就绪
  * @param  writer: PreferenceWriter实例指针
  * @retval 就绪状态
  */
bool stm32g4_preferenceReady(PreferenceWriter_t* writer);

/**
  * @brief  写入int16值到flash
  * @param  writer: PreferenceWriter实例指针
  * @param  value: 要写入的int16值
  * @param  index: 存储索引位置
  * @retval HAL状态
  */
HAL_StatusTypeDef stm32g4_preferenceWriteInt16(PreferenceWriter_t* writer, int16_t value, uint32_t index);

/**
  * @brief  写入整数值到flash
  * @param  writer: PreferenceWriter实例指针
  * @param  value: 要写入的整数值
  * @param  index: 存储索引位置
  * @retval HAL状态
  */
HAL_StatusTypeDef stm32g4_preferenceWriteInt(PreferenceWriter_t* writer, int32_t value, uint32_t index);

/**
  * @brief  写入浮点值到flash
  * @param  writer: PreferenceWriter实例指针
  * @param  value: 要写入的浮点值
  * @param  index: 存储索引位置
  * @retval HAL状态
  */
HAL_StatusTypeDef stm32g4_preferenceWriteFloat(PreferenceWriter_t* writer, float value, uint32_t index);

/**
  * @brief  从flash读取int16值
  * @param  writer: PreferenceWriter实例指针
  * @param  index: 读取索引位置
  * @retval 读取的int16值
  */
int16_t stm32g4_preferenceReadInt16(PreferenceWriter_t* writer, uint32_t index);

/**
  * @brief  从flash读取整数值
  * @param  writer: PreferenceWriter实例指针
  * @param  index: 读取索引位置
  * @retval 读取的整数值
  */
int32_t stm32g4_preferenceReadInt(PreferenceWriter_t* writer, uint32_t index);

/**
  * @brief  从flash读取浮点值
  * @param  writer: PreferenceWriter实例指针
  * @param  index: 读取索引位置
  * @retval 读取的浮点值
  */
float stm32g4_preferenceReadFloat(PreferenceWriter_t* writer, uint32_t index);

/* Hobbyking-style batch operations */

/**
  * @brief  写入int16值到缓存（Hobbyking风格）
  * @param  writer: PreferenceWriter实例指针
  * @param  value: 要写入的int16值
  * @param  index: 寄存器索引
  */
void stm32g4_preferenceWriteInt16ToCache(PreferenceWriter_t* writer, int16_t value, uint32_t index);

/**
  * @brief  写入整数值到缓存（Hobbyking风格）
  * @param  writer: PreferenceWriter实例指针
  * @param  value: 要写入的整数值
  * @param  index: 寄存器索引
  */
void stm32g4_preferenceWriteIntToCache(PreferenceWriter_t* writer, int32_t value, uint32_t index);

/**
  * @brief  写入浮点值到缓存（Hobbyking风格）
  * @param  writer: PreferenceWriter实例指针
  * @param  value: 要写入的浮点值
  * @param  index: 寄存器索引
  */
void stm32g4_preferenceWriteFloatToCache(PreferenceWriter_t* writer, float value, uint32_t index);

/**
  * @brief  将缓存数据刷新到flash（批量保存）
  * @param  writer: PreferenceWriter实例指针
  * @retval HAL状态
  */
HAL_StatusTypeDef stm32g4_preferenceFlushCache(PreferenceWriter_t* writer);

/**
  * @brief  从flash加载数据到缓存（批量加载）
  * @param  writer: PreferenceWriter实例指针
  */
void stm32g4_preferenceLoadCache(PreferenceWriter_t* writer);

/**
  * @brief  检查flash数据是否有效
  * @param  writer: PreferenceWriter实例指针
  * @retval 数据有效性
  */
bool stm32g4_preferenceIsDataValid(PreferenceWriter_t* writer);

/**
  * @brief  使用默认值初始化flash
  * @param  writer: PreferenceWriter实例指针
  * @retval HAL状态
  */
HAL_StatusTypeDef stm32g4_preferenceInitWithDefaults(PreferenceWriter_t* writer);

/* Variable bridge functions */

/**
  * @brief  初始化变量桥接系统
  * @retval HAL状态
  */
HAL_StatusTypeDef initVariableBridge(void);

/**
  * @brief  保存所有变量到flash
  * @retval HAL状态
  */
HAL_StatusTypeDef saveAllVariablesToFlash(void);

/**
  * @brief  从flash加载所有变量
  */
void loadAllVariablesFromFlash(void);

/**
  * @brief  打印所有变量值（调试用）
  */
void printAllVariableValues(void);

/* Default configuration initialization */

/**
  * @brief  初始化用户配置系统
  */
void stm32g4_initUserConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32G4_FLASH_INTEGRATION_H */