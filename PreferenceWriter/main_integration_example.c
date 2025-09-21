/**
  ******************************************************************************
  * @file    main_integration_example.c
  * @brief   Main program integration example for STM32G431 flash configuration
  ******************************************************************************
  */

#include "main.h"
#include "stm32g4_flash_integration.h"
#include <stdint.h>

/* 全局寄存器数组定义（必须在main.c中定义） */
float __float_reg[64] = {0};
int __int_reg[256] = {0};
int16_t __int16_reg[10] = {0};

/**
  * @brief  System initialization with flash configuration
  */
void SystemInitWithFlashConfig(void)
{
    // 初始化HAL库和其他系统组件
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_FDCAN1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM8_Init();
    MX_USART1_UART_Init();
    
    // 初始化flash配置系统
    stm32g4_initUserConfig();
    
    printf("Flash configuration system initialized\n");
    printf("Loaded configuration values:\n");
    printf("Speed: %d, Acceleration: %d, Scale: %.2f\n",
           __int_reg[1], __int_reg[2], __float_reg[1]);
    printf("Int16 values loaded: ");
    for (int i = 0; i < 5; i++) {
        printf("%d ", __int16_reg[i]);
    }
    printf("...\n");
}

/**
  * @brief  Save current configuration to flash
  */
void SaveConfiguration(void)
{
    HAL_StatusTypeDef status = saveAllVariablesToFlash();
    if (status == HAL_OK) {
        printf("Configuration saved to flash successfully\n");
    } else {
        printf("Error saving configuration: %d\n", status);
    }
}

/**
  * @brief  Load configuration from flash
  */
void LoadConfiguration(void)
{
    loadAllVariablesFromFlash();
    printf("Configuration loaded from flash\n");
}

/**
  * @brief  Update configuration parameter
  */
void UpdateSpeedParameter(int newSpeed)
{
    // 更新缓存中的值（函数内部已经会设置__int_reg[1]）
    stm32g4_preferenceWriteIntToCache(&g_preferenceWriter, newSpeed, 1);
    
    printf("Speed parameter updated to: %d\n", newSpeed);
}

/**
  * @brief  Update float parameter
  */
void UpdateScaleParameter(float newScale)
{
    // 更新缓存中的值（函数内部已经会设置__float_reg[1]）
    stm32g4_preferenceWriteFloatToCache(&g_preferenceWriter, newScale, 1);
    
    printf("Scale parameter updated to: %.2f\n", newScale);
}

/**
  * @brief  Update int16 parameter
  */
void UpdateInt16Parameter(unsigned int index, int16_t newValue)
{
    if (index < FLASH_MAX_INT16_INDEX) {
        // 更新缓存中的值
        stm32g4_preferenceWriteInt16ToCache(&g_preferenceWriter, newValue, index);
        printf("Int16 parameter [%d] updated to: %d\n", index, newValue);
    } else {
        printf("Error: Invalid int16 index %d\n", index);
    }
}

/**
  * @brief  Print current configuration
  */
void PrintConfiguration(void)
{
    printf("\nCurrent Configuration:\n");
    printf("=====================\n");
    printf("Speed:        %d\n", __int_reg[1]);
    printf("Acceleration: %d\n", __int_reg[2]);
    printf("Deceleration: %d\n", __int_reg[3]);
    printf("Scale:        %.2f\n", __float_reg[1]);
    printf("Deadzone:     %.2f\n", __float_reg[2]);
    printf("Filter:       %.2f\n", __float_reg[3]);
    
    printf("\nInt16 Parameters:\n");
    for (int i = 0; i < FLASH_MAX_INT16_INDEX; i++) {
        printf("  int16[%d]:    %d\n", i, __int16_reg[i]);
    }
}

// 示例使用场景
void ExampleUsageScenario(void)
{
    // 1. 系统启动时自动加载配置
    SystemInitWithFlashConfig();
    
    // 2. 运行时修改参数
    UpdateSpeedParameter(1800);
    UpdateScaleParameter(1.5f);
    
    // 3. 修改int16参数
    UpdateInt16Parameter(0, 123);
    UpdateInt16Parameter(1, 456);
    UpdateInt16Parameter(2, 789);
    
    // 4. 查看当前配置
    PrintConfiguration();
    
    // 5. 保存配置到flash（通常在配置修改后调用）
    SaveConfiguration();
    
    // 6. 重新加载验证
    LoadConfiguration();
    PrintConfiguration();
}

/* 在主循环中可以这样使用：
void main_loop(void)
{
    while (1)
    {
        // 检测到配置修改时保存
        if (config_modified) {
            SaveConfiguration();
            config_modified = false;
        }
        
        // 其他应用逻辑...
        HAL_Delay(10);
    }
}
*/