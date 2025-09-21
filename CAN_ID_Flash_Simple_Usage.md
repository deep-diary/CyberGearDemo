# CAN ID Flash 保存功能 - 简化版本

## 功能概述

基于 calibration.c 的实现方式，重新实现了 CAN ID 的断电保存功能。使用简单的 64 位数据写入方式，避免了复杂的 PreferenceWriter 系统。

## 实现特点

1. **简单可靠**: 模仿 calibration.c 的 Flash 操作方式，经过验证的稳定实现
2. **独立实现**: 不依赖 PreferenceWriter 系统，避免驱动问题
3. **数据验证**: 使用标识符验证数据有效性
4. **地址灵活**: 使用`FLASH_USER_START_ADDR`，可自定义地址

## 核心函数

### 1. 通用 Flash 操作函数

```c
// 写入64位数据到指定Flash地址
HAL_StatusTypeDef Flash_Write64BitData(uint32_t address, uint64_t data);

// 从指定Flash地址读取64位数据
uint64_t Flash_Read64BitData(uint32_t address);
```

### 2. CAN ID 专用函数

```c
// 保存CAN ID到Flash
void CAN_SaveIdToFlash(uint8_t can_id);

// 从Flash加载CAN ID
void CAN_LoadIdFromFlash(void);
```

## 数据格式

64 位数据格式：

- **位 0-7**: CAN ID (1-127)
- **位 48-63**: 标识符 (0xCAFE) - 用于验证数据有效性
- **其他位**: 保留

## 使用流程

### 系统启动时

```c
// 在main.c的初始化部分
CAN_LoadIdFromFlash();  // 自动加载保存的CAN ID
```

### 收到更改 ID 指令时

```c
// 在CMD_SET_CANID处理中
my_can_id = new_id;
canId = my_can_id;
CAN_SaveIdToFlash(canId);  // 自动保存到Flash
```

## 错误处理

1. **Flash 写入失败**: 函数返回 HAL_ERROR，但不影响系统运行
2. **数据无效**: 自动使用默认 ID `CAN_ID_MOTOR_DEFAULT` (0x7F)
3. **标识符不匹配**: 自动使用默认 ID

## 优势

1. **稳定性**: 基于 calibration.c 的成熟实现
2. **简单性**: 不依赖复杂的配置系统
3. **可靠性**: 使用标识符验证，避免读取无效数据
4. **灵活性**: 通用函数可用于其他 64 位数据存储

## 测试方法

1. **基本功能测试**:

   ```c
   // 测试保存
   CAN_SaveIdToFlash(0x10);

   // 测试加载
   CAN_LoadIdFromFlash();
   // 检查 my_can_id 是否等于 0x10
   ```

2. **重启测试**:

   - 保存一个 ID
   - 重启系统
   - 验证 ID 是否正确加载

3. **错误处理测试**:
   - 手动修改 Flash 数据
   - 验证是否使用默认值

## 扩展使用

通用函数可用于其他数据存储：

```c
// 保存其他64位数据
uint64_t myData = 0x123456789ABCDEF0;
Flash_Write64BitData(FLASH_USER_START_ADDR + 8, myData);

// 读取数据
uint64_t readData = Flash_Read64BitData(FLASH_USER_START_ADDR + 8);
```

## 注意事项

1. **Flash 寿命**: 每次保存都会擦除整个页面，频繁操作可能影响 Flash 寿命
2. **地址冲突**: 确保使用的地址不与 calibration.c 冲突
3. **中断安全**: 写入时禁用中断，确保数据一致性
4. **数据对齐**: 64 位数据需要 8 字节对齐

## 与 calibration.c 的对比

| 特性     | calibration.c   | 新实现          |
| -------- | --------------- | --------------- |
| 数据格式 | 自定义 64 位    | 标准 64 位      |
| 标识符   | CALIB_ID        | 0xCAFE          |
| 地址     | 固定 0x0801F800 | 可配置          |
| 验证     | 简单 ID 检查    | 标识符+范围验证 |
| 通用性   | 专用            | 通用函数        |
