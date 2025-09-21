/**
  ******************************************************************************
  * @file    stm32g4_flash_integration.c
  * @brief   Integrated flash storage implementation for STM32G431
  ******************************************************************************
  */

#include "stm32g4_flash_integration.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include <string.h>
#include "cmsis_compiler.h"  // 用于__disable_irq()和__enable_irq()

// STM32G431 flash配置
// FLASH_BASE 和 FLASH_PAGE_SIZE 现在在头文件中定义

// 全局配置实例
PreferenceWriter_t g_preferenceWriter = {0};

// 全局寄存器数组（在main.c中定义）
extern int16_t __int16_reg[128];
extern int32_t __int32_reg[128];
extern float __float_reg[32];

// 默认配置值
static const int32_t DEFAULT_INT_VALUES[] = {
    0,      // 0: 保留
    1000,   // 1: 默认速度
    500,    // 2: 默认加速度
    100,    // 3: 默认减速度
    0,      // 4: 默认位置
    1,      // 5: 默认方向
    0,      // 6: 默认使能状态
    100,    // 7: 默认电流限制
    // 其他默认值...
};

static const float DEFAULT_FLOAT_VALUES[] = {
    0.0f,       // 0: 保留
    1.0f,       // 1: 默认比例因子
    0.1f,       // 2: 默认死区
    0.5f,       // 3: 默认滤波系数
    0.0f,       // 4: 默认偏移
    1.0f,       // 5: 默认增益
    // 其他默认值...
};

static const int16_t DEFAULT_INT16_VALUES[] = {
    0,      // 0: 保留
    100,    // 1: 默认值1
    200,    // 2: 默认值2
    300,    // 3: 默认值3
    400,    // 4: 默认值4
    500,    // 5: 默认值5
    600,    // 6: 默认值6
    700,    // 7: 默认值7
    800,    // 8: 默认值8
    900,    // 9: 默认值9
    // 继续到128个默认值...
};

/**
  * @brief  擦除指定flash页面
  * @param  page: 要擦除的页面号
  * @retval HAL状态
  */
static HAL_StatusTypeDef flashErasePage(uint32_t page)
{
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError;
    
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.Banks = FLASH_BANK_1;
    eraseInit.Page = page;
    eraseInit.NbPages = 1;
    
    // 保存当前中断状态
    uint32_t primask_bit = __get_PRIMASK();
    
    // 禁用所有中断
    __disable_irq();
    
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
    
    // 恢复中断状态
    if (!primask_bit) {
        __enable_irq();
    }
    
    return status;
}

/**
  * @brief  清除Flash错误标志
  */
static void flashClearErrorFlags(void)
{
    // 直接清除SR寄存器中的所有错误标志
    // 根据STM32G4参考手册，写入1来清除SR寄存器中的标志位
    // 错误标志位：OPERR(0x01), PROGERR(0x08), WRPERR(0x10), PGAERR(0x20),
    // SIZERR(0x40), PGSERR(0x80), MISERR(0x100), FASTERR(0x200), RDERR(0x400), OPTVERR(0x800)
    // 总和：0x01 + 0x08 + 0x10 + 0x20 + 0x40 + 0x80 + 0x100 + 0x200 + 0x400 + 0x800 = 0xFBB
    *(volatile uint32_t *)0x4002200C = 0x00000FBBU; // FLASH->SR 地址
}

/**
  * @brief  调试Flash错误信息
  * @param  error: HAL Flash错误代码
  * @param  address: 发生错误的地址
  */
static void debugFlashError(uint32_t error, uint32_t address)
{
    // 这里可以添加实际的调试输出，例如通过串口
    // printf("Flash Error at 0x%08lX: 0x%08lX\n", address, error);
    
    if (error & 0x00000020U) { // PGAERR
        // printf("  PGAERR: Programming alignment error (address not 8-byte aligned)\n");
    }
    if (error & 0x00000080U) { // PGSERR
        // printf("  PGSERR: Programming sequence error (incorrect programming sequence)\n");
    }
    if (error & 0x00000008U) { // PROGERR
        // printf("  PROGERR: Programming error (trying to program non-erased location)\n");
    }
    if (error & 0x00000010U) { // WRPERR
        // printf("  WRPERR: Write protection error (page is write-protected)\n");
    }
    if (error & 0x00000001U) { // OPERR
        // printf("  OPERR: Operation error (general operation error)\n");
    }
}

/**
  * @brief  编程flash数据（双字模式）
  * @param  address: flash地址
  * @param  data: 要写入的数据（64位）
  * @retval HAL状态
  */
static HAL_StatusTypeDef flashProgram(uint32_t address, uint64_t data)
{
    // 清除之前的错误标志
    flashClearErrorFlags();
    
    // 检查地址是否8字节对齐（STM32G4要求）
    if ((address & 0x7) != 0) {
        // printf("Flash programming error: Address 0x%08lX not 8-byte aligned\n", address);
        return HAL_ERROR; // 地址不对齐
    }
    
    // 检查目标地址是否已经被擦除（全1）
    uint64_t current_data = *(uint64_t*)address;
    if (current_data != 0xFFFFFFFFFFFFFFFF) {
        // printf("Flash programming warning: Address 0x%08lX not erased (0x%016llX)\n", address, current_data);
        // 这里可以决定是否继续编程或返回错误
        // 对于STM32G4，编程非擦除位置会导致PROGERR
    }
    
    // 直接调用HAL Flash编程函数，HAL库内部已经包含了等待操作完成的逻辑
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);
    
    // 如果编程失败，获取详细的错误信息
    if (status != HAL_OK) {
        uint32_t error = HAL_FLASH_GetError();
        debugFlashError(error, address);
    }
    
    // 添加足够的延迟以确保Flash操作完成
    // STM32G4 Flash编程需要较长时间来完成内部操作
    for (volatile uint32_t i = 0; i < 1000; i++);
    
    // 检查Flash是否繁忙，等待操作完成
    // FLASH_FLAG_BSY = 0x00010000 (根据STM32G4参考手册)
    while ((*(volatile uint32_t *)0x4002200C & 0x00010000) != 0) {
        // 等待Flash操作完成
    }
    
    // 清除任何可能出现的错误标志
    flashClearErrorFlags();
    
    return status;
}

/**
  * @brief  获取flash物理地址
  * @param  page: 页面号
  * @param  offset: 页面内偏移量
  * @retval 物理地址
  */
static uint32_t getFlashAddress(uint32_t page, uint32_t offset)
{
    return FLASH_BASE + (page * FLASH_PAGE_SIZE) + (offset * sizeof(uint64_t));
}

/* 核心功能实现 */

/**
  * @brief  初始化PreferenceWriter实例
  * @param  writer: PreferenceWriter实例指针
  * @param  page: 使用的flash页面号
  */
void stm32g4_preferenceInit(PreferenceWriter_t* writer, uint32_t page)
{
    writer->page = page;
    writer->max_index = FLASH_PAGE_SIZE / sizeof(uint64_t) - 1;
    writer->ready = false;
}

/**
  * @brief  打开PreferenceWriter（验证页面有效性）
  * @param  writer: PreferenceWriter实例指针
  */
void stm32g4_preferenceOpen(PreferenceWriter_t* writer)
{
    if (writer->page <= 63) { // STM32G431有64页（0-63）
        writer->ready = true;
    }
}

/**
  * @brief  检查PreferenceWriter是否就绪
  * @param  writer: PreferenceWriter实例指针
  * @retval 就绪状态
  */
bool stm32g4_preferenceReady(PreferenceWriter_t* writer)
{
    return writer->ready;
}

/**
  * @brief  写入int16值到flash（高效存储：每8字节存储4个int16）
  * @param  writer: PreferenceWriter实例指针
  * @param  value: 要写入的int16值
  * @param  index: 存储索引位置
  * @retval HAL状态
  */
HAL_StatusTypeDef stm32g4_preferenceWriteInt16(PreferenceWriter_t* writer, int16_t value, uint32_t index)
{
    if (index > writer->max_index * 4) return HAL_ERROR;
    
    // 每8字节存储4个int16，索引0-3在第一个双字，4-7在第二个双字，以此类推
    uint32_t doubleword_index = index / 4;
    uint32_t sub_index = index % 4;
    
    uint32_t address = getFlashAddress(writer->page, doubleword_index);
    
    // 读取现有的双字数据（如果存在）
    uint64_t existing_data = *(uint64_t*)address;
    uint16_t data_parts[4];
    
    // 检查Flash是否已经被擦除（全1）
    if (existing_data == 0xFFFFFFFFFFFFFFFF) {
        // Flash被擦除，初始化所有部分为0xFFFF
        data_parts[0] = 0xFFFF;
        data_parts[1] = 0xFFFF;
        data_parts[2] = 0xFFFF;
        data_parts[3] = 0xFFFF;
    } else {
        // 将64位数据分解为4个16位部分
        data_parts[0] = (existing_data & 0xFFFF);
        data_parts[1] = ((existing_data >> 16) & 0xFFFF);
        data_parts[2] = ((existing_data >> 32) & 0xFFFF);
        data_parts[3] = ((existing_data >> 48) & 0xFFFF);
    }
    
    // 更新对应的16位部分
    data_parts[sub_index] = (uint16_t)value;
    
    // 重新组合为64位数据
    uint64_t new_data = ((uint64_t)data_parts[3] << 48) |
                        ((uint64_t)data_parts[2] << 32) |
                        ((uint64_t)data_parts[1] << 16) |
                        data_parts[0];
    
    return flashProgram(address, new_data);
}

/**
  * @brief  写入整数值到flash（高效存储：每8字节存储2个int）
  * @param  writer: PreferenceWriter实例指针
  * @param  value: 要写入的整数值
  * @param  index: 存储索引位置
  * @retval HAL状态
  */
HAL_StatusTypeDef stm32g4_preferenceWriteInt(PreferenceWriter_t* writer, int32_t value, uint32_t index)
{
    if (index > writer->max_index * 2) return HAL_ERROR;
    
    // 每8字节存储2个int，索引0-1在第一个双字，2-3在第二个双字，以此类推
    uint32_t doubleword_index = index / 2;
    uint32_t sub_index = index % 2;
    
    uint32_t address = getFlashAddress(writer->page, doubleword_index);
    
    // 读取现有的双字数据（如果存在）
    uint64_t existing_data = *(uint64_t*)address;
    uint32_t data_low, data_high;
    
    // 检查Flash是否已经被擦除（全1）
    if (existing_data == 0xFFFFFFFFFFFFFFFF) {
        // Flash被擦除，初始化所有部分为0xFFFFFFFF
        data_low = 0xFFFFFFFF;
        data_high = 0xFFFFFFFF;
    } else {
        // 将64位数据分解为2个32位部分
        data_low = existing_data & 0xFFFFFFFF;
        data_high = (existing_data >> 32) & 0xFFFFFFFF;
    }
    
    // 更新对应的32位部分
    if (sub_index == 0) {
        data_low = (uint32_t)value;
    } else {
        data_high = (uint32_t)value;
    }
    
    uint64_t new_data = ((uint64_t)data_high << 32) | data_low;
    
    return flashProgram(address, new_data);
}

/**
  * @brief  写入浮点值到flash（高效存储：每8字节存储2个float）
  * @param  writer: PreferenceWriter实例指针
  * @param  value: 要写入的浮点值
  * @param  index: 存储索引位置
  * @retval HAL状态
  */
HAL_StatusTypeDef stm32g4_preferenceWriteFloat(PreferenceWriter_t* writer, float value, uint32_t index)
{
    if (index > writer->max_index * 2) return HAL_ERROR;
    
    // 每8字节存储2个float，索引0-1在第一个双字，2-3在第二个双字，以此类推
    uint32_t doubleword_index = index / 2;
    uint32_t sub_index = index % 2;
    
    uint32_t address = getFlashAddress(writer->page, doubleword_index);
    
    // 读取现有的双字数据（如果存在）
    uint64_t existing_data = *(uint64_t*)address;
    uint32_t data_low, data_high;
    
    // 检查Flash是否已经被擦除（全1）
    if (existing_data == 0xFFFFFFFFFFFFFFFF) {
        // Flash被擦除，初始化所有部分为0xFFFFFFFF
        data_low = 0xFFFFFFFF;
        data_high = 0xFFFFFFFF;
    } else {
        // 将64位数据分解为2个32位部分
        data_low = existing_data & 0xFFFFFFFF;
        data_high = (existing_data >> 32) & 0xFFFFFFFF;
    }
    
    // 将float转换为uint32_t
    uint32_t floatAsInt;
    memcpy(&floatAsInt, &value, sizeof(float));
    
    // 更新对应的32位部分
    if (sub_index == 0) {
        data_low = floatAsInt;
    } else {
        data_high = floatAsInt;
    }
    
    uint64_t new_data = ((uint64_t)data_high << 32) | data_low;
    
    return flashProgram(address, new_data);
}

/**
  * @brief  从flash读取int16值（高效存储：每8字节存储4个int16）
  * @param  writer: PreferenceWriter实例指针
  * @param  index: 读取索引位置
  * @retval 读取的int16值
  */
int16_t stm32g4_preferenceReadInt16(PreferenceWriter_t* writer, uint32_t index)
{
    if (index > writer->max_index * 4) return 0;
    
    // 每8字节存储4个int16，索引0-3在第一个双字,4-7在第二个双字，以此类推
    uint32_t doubleword_index = index / 4;
    uint32_t sub_index = index % 4;
    
    uint32_t address = getFlashAddress(writer->page, doubleword_index);
    uint64_t data = *(uint64_t*)address;
    
    // 提取对应的16位部分
    switch (sub_index) {
        case 0: return (int16_t)(data & 0xFFFF);
        case 1: return (int16_t)((data >> 16) & 0xFFFF);
        case 2: return (int16_t)((data >> 32) & 0xFFFF);
        case 3: return (int16_t)((data >> 48) & 0xFFFF);
        default: return 0;
    }
}

/**
  * @brief  从flash读取整数值（高效存储：每8字节存储2个int）
  * @param  writer: PreferenceWriter实例指针
  * @param  index: 读取索引位置
  * @retval 读取的整数值
  */
int32_t stm32g4_preferenceReadInt(PreferenceWriter_t* writer, uint32_t index)
{
    if (index > writer->max_index * 2) return 0;
    
    // 每8字节存储2个int，索引0-1在第一个双字，2-3在第二个双字，以此类推
    uint32_t doubleword_index = index / 2;
    uint32_t sub_index = index % 2;
    
    uint32_t address = getFlashAddress(writer->page, doubleword_index);
    uint64_t data = *(uint64_t*)address;
    
    if (sub_index == 0) {
        return (int32_t)(data & 0xFFFFFFFF);
    } else {
        return (int32_t)((data >> 32) & 0xFFFFFFFF);
    }
}

/**
  * @brief  从flash读取浮点值（高效存储：每8字节存储2个float）
  * @param  writer: PreferenceWriter实例指针
  * @param  index: 读取索引位置
  * @retval 读取的浮点值
  */
float stm32g4_preferenceReadFloat(PreferenceWriter_t* writer, uint32_t index)
{
    if (index > writer->max_index * 2) return 0.0f;
    
    // 每8字节存储2个float，索引0-1在第一个双字，2-3在第二个双字，以此类推
    uint32_t doubleword_index = index / 2;
    uint32_t sub_index = index % 2;
    
    uint32_t address = getFlashAddress(writer->page, doubleword_index);
    uint64_t data = *(uint64_t*)address;
    
    uint32_t intValue;
    if (sub_index == 0) {
        intValue = data & 0xFFFFFFFF;
    } else {
        intValue = (data >> 32) & 0xFFFFFFFF;
    }
    
    float result;
    memcpy(&result, &intValue, sizeof(float));
    return result;
}

/* Hobbyking风格批量操作 */

/**
  * @brief  写入int16值到缓存（Hobbyking风格）
  * @param  writer: PreferenceWriter实例指针
  * @param  value: 要写入的int16值
  * @param  index: 寄存器索引
  */
void stm32g4_preferenceWriteInt16ToCache(PreferenceWriter_t* writer, int16_t value, uint32_t index)
{
    if (index < FLASH_MAX_INT16_INDEX) {
        __int16_reg[index] = value;
    }
}

/**
  * @brief  写入整数值到缓存（Hobbyking风格）
  * @param  writer: PreferenceWriter实例指针
  * @param  value: 要写入的整数值
  * @param  index: 寄存器索引
  */
void stm32g4_preferenceWriteIntToCache(PreferenceWriter_t* writer, int32_t value, uint32_t index)
{
    if (index < FLASH_MAX_INT_INDEX) {
        __int32_reg[index] = value;
    }
}

/**
  * @brief  写入浮点值到缓存（Hobbyking风格）
  * @param  writer: PreferenceWriter实例指针
  * @param  value: 要写入的浮点值
  * @param  index: 寄存器索引
  */
void stm32g4_preferenceWriteFloatToCache(PreferenceWriter_t* writer, float value, uint32_t index)
{
    if (index < FLASH_MAX_FLOAT_INDEX) {
        __float_reg[index] = value;
    }
}

/**
  * @brief  将缓存数据刷新到flash（批量保存）
  * @param  writer: PreferenceWriter实例指针
  * @retval HAL状态
  */
HAL_StatusTypeDef stm32g4_preferenceFlushCache(PreferenceWriter_t* writer)
{
    HAL_StatusTypeDef status = flashErasePage(writer->page);
    if (status != HAL_OK) return status;
    
    // 写入int16寄存器（放在最前面）
    for (uint32_t i = 0; i < FLASH_MAX_INT16_INDEX; i++) {
        status = stm32g4_preferenceWriteInt16(writer, __int16_reg[i], i);
        if (status != HAL_OK) return status;
    }
    
    // 写入整数寄存器
    for (uint32_t i = 0; i < FLASH_MAX_INT_INDEX; i++) {
        status = stm32g4_preferenceWriteInt(writer, __int32_reg[i], FLASH_MAX_INT16_INDEX + i);
        if (status != HAL_OK) return status;
    }
    
    // 写入浮点寄存器
    for (uint32_t i = 0; i < FLASH_MAX_FLOAT_INDEX; i++) {
        status = stm32g4_preferenceWriteFloat(writer, __float_reg[i], FLASH_MAX_INT16_INDEX + FLASH_MAX_INT_INDEX + i);
        if (status != HAL_OK) return status;
    }
    
    return HAL_OK;
}

/**
  * @brief  从flash加载数据到缓存（批量加载）
  * @param  writer: PreferenceWriter实例指针
  */
void stm32g4_preferenceLoadCache(PreferenceWriter_t* writer)
{
    // 读取int16寄存器（放在最前面
    for (uint32_t i = 0; i < FLASH_MAX_INT16_INDEX; i++) {
        __int16_reg[i] = stm32g4_preferenceReadInt16(writer, i);
    }
    
    // 读取整数寄存器
    for (uint32_t i = 0; i < FLASH_MAX_INT_INDEX; i++) {
        __int32_reg[i] = stm32g4_preferenceReadInt(writer, FLASH_MAX_INT16_INDEX + i);
    }
    
    // 读取浮点寄存器
    for (uint32_t i = 0; i < FLASH_MAX_FLOAT_INDEX; i++) {
        __float_reg[i] = stm32g4_preferenceReadFloat(writer, FLASH_MAX_INT16_INDEX + FLASH_MAX_INT_INDEX + i);
    }
}

/**
  * @brief  检查flash数据是否有效
  * @param  writer: PreferenceWriter实例指针
  * @retval 数据有效性
  */
bool stm32g4_preferenceIsDataValid(PreferenceWriter_t* writer)
{
    // 检查第一个位置是否有有效数据（非0xFFFFFFFF）
    uint32_t address = getFlashAddress(writer->page, 0);
    return (*(uint32_t*)address != 0xFFFFFFFF);
}

/**
  * @brief  使用默认值初始化flash
  * @param  writer: PreferenceWriter实例指针
  * @retval HAL状态
  */
HAL_StatusTypeDef stm32g4_preferenceInitWithDefaults(PreferenceWriter_t* writer)
{
    HAL_StatusTypeDef status;
    
    // 保存当前中断状态
    uint32_t primask_bit = __get_PRIMASK();
    
    // 禁用所有中断
    __disable_irq();

    HAL_FLASH_Unlock();

    status = flashErasePage(writer->page);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        if (!primask_bit) {
            __enable_irq();
        }
        return status;
    }
    
    // 在擦除后添加足够的延迟，确保Flash完全准备好
    for (volatile uint32_t i = 0; i < 5000; i++);
    
    // 设置默认int16值（放在最前面）
    for (uint32_t i = 0; i < sizeof(DEFAULT_INT16_VALUES)/sizeof(DEFAULT_INT16_VALUES[0]); i++) {
        status = stm32g4_preferenceWriteInt16(writer, DEFAULT_INT16_VALUES[i], i);
        if (status != HAL_OK) {
            // 详细错误调试
            uint32_t flash_error = HAL_FLASH_GetError();
            // 可以在这里添加错误日志
            HAL_FLASH_Lock();
            if (!primask_bit) {
                __enable_irq();
            }
            return status;
        }
        
        // 在每个编程操作后添加小延迟，确保Flash稳定
        for (volatile uint32_t j = 0; j < 100; j++);
    }
    
    // 设置默认整数值
    for (uint32_t i = 0; i < sizeof(DEFAULT_INT_VALUES)/sizeof(DEFAULT_INT_VALUES[0]); i++) {
        status = stm32g4_preferenceWriteInt(writer, DEFAULT_INT_VALUES[i], FLASH_MAX_INT16_INDEX + i);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            if (!primask_bit) {
                __enable_irq();
            }
            return status;
        }
        
        // 在每个编程操作后添加小延迟
        for (volatile uint32_t j = 0; j < 100; j++);
    }
    
    // 设置默认浮点值
    for (uint32_t i = 0; i < sizeof(DEFAULT_FLOAT_VALUES)/sizeof(DEFAULT_FLOAT_VALUES[0]); i++) {
        status = stm32g4_preferenceWriteFloat(writer, DEFAULT_FLOAT_VALUES[i], FLASH_MAX_INT16_INDEX + FLASH_MAX_INT_INDEX + i);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            if (!primask_bit) {
                __enable_irq();
            }
            return status;
        }
        
        // 在每个编程操作后添加小延迟
        for (volatile uint32_t j = 0; j < 100; j++);
    }
    
    HAL_FLASH_Lock();
    
    // 恢复中断状态
    if (!primask_bit) {
        __enable_irq();
    }
    
    return HAL_OK;
}

/* 变量桥接功能 */

/**
  * @brief  初始化变量桥接系统
  * @retval HAL状态
  */
HAL_StatusTypeDef initVariableBridge(void)
{
    stm32g4_preferenceInit(&g_preferenceWriter, FLASH_USER_CONFIG_PAGE);
    stm32g4_preferenceOpen(&g_preferenceWriter);
    
    // if (!stm32g4_preferenceIsDataValid(&g_preferenceWriter)) {
    //     return stm32g4_preferenceInitWithDefaults(&g_preferenceWriter);
    // }
    stm32g4_preferenceInitWithDefaults(&g_preferenceWriter);
    stm32g4_preferenceLoadCache(&g_preferenceWriter);
    return HAL_OK;
}

/**
  * @brief  保存所有变量到flash
  * @retval HAL状态
  */
HAL_StatusTypeDef saveAllVariablesToFlash(void)
{
    return stm32g4_preferenceFlushCache(&g_preferenceWriter);
}

/**
  * @brief  从flash加载所有变量
  */
void loadAllVariablesFromFlash(void)
{
    stm32g4_preferenceLoadCache(&g_preferenceWriter);
}


/**
  * @brief  初始化用户配置系统
  */
void stm32g4_initUserConfig(void)
{
    // 初始化flash配置
    initVariableBridge();
}