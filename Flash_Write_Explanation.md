# Flash 写入原理和解决方案

## 问题分析

您遇到的问题是典型的 Flash 写入问题：**第一次可以写入，第二次无法写入**。

## Flash 存储器特性

### 基本特性

- Flash 存储器只能将 **1 变成 0**，不能将 **0 变成 1**
- 要写入新数据，必须先擦除整个页面（将整个页面设为 0xFF）

### 写入过程

1. **擦除页面**：将整个页面设为 0xFF (全 1)
2. **写入数据**：将需要的位设为 0

## 问题原因

```c
// 第一次写入（页面是擦除状态 0xFFFFFFFFFFFFFFFF）
uint64_t data1 = 0x123456789ABCDEF0;  // 可以写入

// 第二次写入（页面已有数据，某些位已经是0）
uint64_t data2 = 0x87654321FEDCBA09;  // 无法写入！
```

## 解决方案

### 1. 修复后的代码

```c
HAL_StatusTypeDef Flash_Write64BitData(uint32_t address, uint64_t data) {
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseConfig;
    uint32_t pageError;

    __disable_irq();
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    // 计算页面号
    uint32_t pageNumber = (address - FLASH_BASE) / FLASH_PAGE_SIZE;

    // 配置擦除参数
    eraseConfig.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseConfig.Banks = FLASH_BANK_1;
    eraseConfig.Page = pageNumber;
    eraseConfig.NbPages = 1;

    // 必须先擦除页面
    status = HAL_FLASHEx_Erase(&eraseConfig, &pageError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        __enable_irq();
        return status;
    }

    // 然后写入数据
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);

    HAL_FLASH_Lock();
    __enable_irq();
    return status;
}
```

### 2. 关键改进

- **恢复擦除逻辑**：每次写入前都擦除页面
- **正确计算页面号**：`(address - FLASH_BASE) / FLASH_PAGE_SIZE`
- **添加宏定义**：`FLASH_BASE` 和 `FLASH_PAGE_SIZE`

## 测试方法

### 1. 基本测试

```c
// 在main.c中测试
uint8_t testResult = Flash_TestWrite();
if (testResult == 0) {
    printf("Flash写入测试成功\n");
} else {
    printf("Flash写入测试失败\n");
}
```

### 2. 多次写入测试

```c
// 测试多次写入
for (int i = 0; i < 5; i++) {
    uint64_t testData = 0x1000000000000000 + i;
    HAL_StatusTypeDef status = Flash_Write64BitData(ADDR_FLASH_PAGE_62, testData);
    if (status != HAL_OK) {
        printf("第%d次写入失败\n", i+1);
        break;
    }
    printf("第%d次写入成功\n", i+1);
}
```

## 注意事项

### 1. Flash 寿命

- 每次写入都会擦除整个页面
- STM32G4 Flash 典型擦除次数：10,000 次
- 频繁写入会影响 Flash 寿命

### 2. 地址对齐

- 64 位数据需要 8 字节对齐
- `ADDR_FLASH_PAGE_62` 是 2KB 对齐的，满足要求

### 3. 中断安全

- 写入过程中禁用中断
- 确保数据一致性

## 优化建议

### 1. 减少写入频率

```c
// 只在数据真正改变时才写入
static uint8_t last_can_id = 0;
if (can_id != last_can_id) {
    CAN_SaveIdToFlash(can_id);
    last_can_id = can_id;
}
```

### 2. 批量写入

```c
// 如果需要保存多个参数，可以批量写入
typedef struct {
    uint8_t can_id;
    uint8_t master_id;
    uint16_t timeout;
    uint32_t reserved;
} ConfigData_t;

void SaveConfigToFlash(ConfigData_t* config) {
    uint64_t data = *(uint64_t*)config;
    Flash_Write64BitData(ADDR_FLASH_PAGE_62, data);
}
```

## 总结

Flash 写入必须遵循 **擦除 → 写入** 的流程，这是 Flash 存储器的基本特性。修复后的代码应该可以正常进行多次写入操作。
