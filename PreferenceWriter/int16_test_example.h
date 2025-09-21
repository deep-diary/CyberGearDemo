/**
  ******************************************************************************
  * @file    int16_test_example.h
  * @brief   Header for int16 flash storage test functionality
  ******************************************************************************
  */

#ifndef __INT16_TEST_EXAMPLE_H
#define __INT16_TEST_EXAMPLE_H

#include "stm32g4_flash_integration.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  测试int16功能示例
  */
void testInt16Functionality(void);

/**
  * @brief  打印所有int16寄存器值
  */
void printAllInt16Registers(void);

/**
  * @brief  更新单个int16变量并保存
  * @param  index: 寄存器索引
  * @param  value: 要设置的值
  * @retval HAL状态
  */
HAL_StatusTypeDef updateInt16Variable(uint32_t index, int16_t value);

/**
  * @brief  获取单个int16变量
  * @param  index: 寄存器索引
  * @retval 变量值
  */
int16_t getInt16Variable(uint32_t index);

#ifdef __cplusplus
}
#endif

#endif /* __INT16_TEST_EXAMPLE_H */