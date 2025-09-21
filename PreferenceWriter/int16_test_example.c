/**
  ******************************************************************************
  * @file    int16_test_example.c
  * @brief   Example code for testing int16 flash storage functionality
  ******************************************************************************
  */

#include "stm32g4_flash_integration.h"
#include <stdio.h>

/**
  * @brief  测试int16功能示例
  */
void testInt16Functionality(void)
{
    printf("Testing int16 flash functionality...\n");
    
    // 初始化用户配置系统
    stm32g4_initUserConfig();
    
    // 测试写入int16值到缓存
    printf("Writing int16 values to cache...\n");
    for (int i = 0; i < FLASH_MAX_INT16_INDEX; i++) {
        stm32g4_preferenceWriteInt16ToCache(&g_preferenceWriter, (int16_t)(i * 100 + 50), i);
        printf("  int16_reg[%d] = %d\n", i, __int16_reg[i]);
    }
    
    // 保存所有变量到flash
    printf("Saving all variables to flash...\n");
    HAL_StatusTypeDef status = saveAllVariablesToFlash();
    if (status == HAL_OK) {
        printf("Save successful!\n");
    } else {
        printf("Save failed with error: %d\n", status);
        return;
    }
    
    // 修改缓存值以验证加载功能
    printf("Modifying cache values...\n");
    for (int i = 0; i < FLASH_MAX_INT16_INDEX; i++) {
        __int16_reg[i] = -1;
        printf("  int16_reg[%d] = %d (modified)\n", i, __int16_reg[i]);
    }
    
    // 从flash加载所有变量
    printf("Loading all variables from flash...\n");
    loadAllVariablesFromFlash();
    
    // 验证加载的值
    printf("Verifying loaded values...\n");
    for (int i = 0; i < FLASH_MAX_INT16_INDEX; i++) {
        printf("  int16_reg[%d] = %d (should be %d)\n", 
               i, __int16_reg[i], (int16_t)(i * 100 + 50));
        
        if (__int16_reg[i] != (int16_t)(i * 100 + 50)) {
            printf("  ERROR: Value mismatch!\n");
        }
    }
    
    // 测试直接读写功能
    printf("Testing direct read/write functions...\n");
    for (int i = 0; i < FLASH_MAX_INT16_INDEX; i++) {
        int16_t test_value = (int16_t)(i * 200 - 100);
        
        // 直接写入flash
        status = stm32g4_preferenceWriteInt16(&g_preferenceWriter, test_value, 
                                            FLASH_MAX_INT_INDEX + FLASH_MAX_FLOAT_INDEX + i);
        if (status != HAL_OK) {
            printf("  Direct write failed for index %d\n", i);
            continue;
        }
        
        // 直接读取flash
        int16_t read_value = stm32g4_preferenceReadInt16(&g_preferenceWriter, 
                                                       FLASH_MAX_INT_INDEX + FLASH_MAX_FLOAT_INDEX + i);
        
        printf("  Direct test: wrote %d, read %d %s\n", 
               test_value, read_value, 
               (test_value == read_value) ? "(OK)" : "(ERROR)");
    }
    
    printf("Int16 functionality test completed!\n");
}

/**
  * @brief  打印所有int16寄存器值
  */
void printAllInt16Registers(void)
{
    printf("Current int16 register values:\n");
    for (int i = 0; i < FLASH_MAX_INT16_INDEX; i++) {
        printf("  int16_reg[%d] = %d\n", i, __int16_reg[i]);
    }
}

/**
  * @brief  更新单个int16变量并保存
  * @param  index: 寄存器索引
  * @param  value: 要设置的值
  * @retval HAL状态
  */
HAL_StatusTypeDef updateInt16Variable(uint32_t index, int16_t value)
{
    if (index >= FLASH_MAX_INT16_INDEX) {
        return HAL_ERROR;
    }
    
    // 更新缓存
    stm32g4_preferenceWriteInt16ToCache(&g_preferenceWriter, value, index);
    
    // 保存到flash
    return saveAllVariablesToFlash();
}

/**
  * @brief  获取单个int16变量
  * @param  index: 寄存器索引
  * @retval 变量值
  */
int16_t getInt16Variable(uint32_t index)
{
    if (index >= FLASH_MAX_INT16_INDEX) {
        return 0;
    }
    
    return __int16_reg[index];
}