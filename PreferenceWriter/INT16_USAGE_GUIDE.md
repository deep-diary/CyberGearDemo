# Int16 Flash Storage Usage Guide

## 概述

本文档介绍了如何在STM32G431的flash存储系统中使用int16数据类型。系统支持10个int16变量的存储和读写操作。

## 功能特性

- **存储容量**: 10个int16变量 (索引0-9)
- **存储效率**: 每8字节存储4个int16变量
- **存储位置**: 在flash中位于int和float变量之后
- **索引范围**: 0-9 (FLASH_MAX_INT16_INDEX = 10)

## 使用方法

### 1. 基本读写操作

```c
#include "stm32g4_flash_integration.h"

// 写入int16值到缓存
stm32g4_preferenceWriteInt16ToCache(&g_preferenceWriter, 123, 0);

// 从缓存读取int16值
int16_t value = __int16_reg[0];

// 直接写入flash（不推荐频繁使用）
HAL_StatusTypeDef status = stm32g4_preferenceWriteInt16(&g_preferenceWriter, 456, 0);

// 直接读取flash
int16_t read_value = stm32g4_preferenceReadInt16(&g_preferenceWriter, 0);
```

### 2. 批量操作

```c
// 保存所有变量到flash（包括int16）
saveAllVariablesToFlash();

// 从flash加载所有变量（包括int16）
loadAllVariablesFromFlash();
```

### 3. 示例代码

```c
// 初始化系统
stm32g4_initUserConfig();

// 设置int16变量
__int16_reg[0] = 100;   // 变量0
__int16_reg[1] = 200;   // 变量1
__int16_reg[2] = 300;   // 变量2

// 保存到flash
saveAllVariablesToFlash();

// 重新加载验证
loadAllVariablesFromFlash();
printf("Variable 0: %d\n", __int16_reg[0]);  // 输出: 100
```

## 存储布局

flash存储页面按照以下顺序组织数据：

1. **整数变量**: 256个int (索引0-255)
2. **浮点变量**: 64个float (索引256-319)  
3. **int16变量**: 10个int16 (索引320-329)

实际存储索引计算：
```c
int16_index = FLASH_MAX_INT_INDEX + FLASH_MAX_FLOAT_INDEX + int16_reg_index
```

## 性能考虑

1. **批量操作**: 推荐使用批量保存/加载函数，避免频繁的flash擦写
2. **缓存使用**: 修改变量时先更新缓存，最后批量保存
3. **中断保护**: flash操作已包含中断保护机制

## 错误处理

所有flash操作函数返回HAL_StatusTypeDef，建议检查返回值：

```c
HAL_StatusTypeDef status = saveAllVariablesToFlash();
if (status != HAL_OK) {
    // 错误处理
    printf("Save failed: %d\n", status);
}
```

## 默认值

系统包含10个int16默认值：
- 索引0: 0 (保留)
- 索引1: 100
- 索引2: 200
- 索引3: 300
- 索引4: 400
- 索引5: 500
- 索引6: 600
- 索引7: 700
- 索引8: 800
- 索引9: 900

## 注意事项

1. **索引范围**: 确保索引在0-9范围内，否则会返回错误
2. **flash寿命**: STM32G431 flash擦写寿命约10,000次，避免频繁保存
3. **数据一致性**: 在修改多个变量后一次性保存，确保数据一致性
4. **中断安全**: flash操作期间会自动禁用中断，无需额外处理

## 测试函数

系统提供了测试函数来验证int16功能：

```c
#include "int16_test_example.h"

// 运行完整测试
testInt16Functionality();

// 打印所有int16寄存器值
printAllInt16Registers();

// 更新单个变量并保存
updateInt16Variable(0, 123);

// 获取单个变量值
int16_t value = getInt16Variable(0);
```

## 故障排除

### 常见问题

1. **变量值不正确**: 确保在修改后调用了保存函数
2. **保存失败**: 检查flash页面是否有效（0-63）
3. **读取值为0**: 可能是第一次使用，flash中无有效数据

### 调试建议

1. 使用`printAllInt16Registers()`函数查看当前值
2. 检查函数返回值
3. 验证flash页面设置是否正确