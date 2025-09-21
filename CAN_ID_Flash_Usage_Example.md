# CAN ID Flash 保存功能使用说明

## 功能概述

本功能实现了 CAN ID 的断电保存，当收到更改 CAN ID 指令时，会自动将新的 ID 保存到 Flash 中，系统重启后会自动从 Flash 加载保存的 ID。

## 实现原理

1. **变量桥接系统**: 使用`stm32g4_variable_bridge.h`中的`CAN_BUS_ID`宏定义，将 CAN ID 映射到全局寄存器数组`__int32_reg[1]`
2. **Flash 存储**: 使用 STM32G4 的最后一页 Flash（第 63 页）存储配置数据
3. **自动加载**: 系统启动时自动从 Flash 加载保存的 CAN ID

## 代码修改说明

### 1. 变量桥接定义 (stm32g4_variable_bridge.h)

```c
#define CAN_BUS_ID __int32_reg[1]  // CAN总线ID，已使用
```

### 2. CAN 接口函数 (can_interface.c)

```c
// 保存CAN ID到Flash
void CAN_SaveIdToFlash(uint8_t can_id);

// 从Flash加载CAN ID
void CAN_LoadIdFromFlash(void);
```

### 3. 主程序初始化 (main.c)

```c
// 在main函数中启用flash配置系统
stm32g4_initUserConfig();

// 从Flash加载CAN ID
CAN_LoadIdFromFlash();
```

## 使用流程

### 系统启动时

1. 系统初始化时调用`stm32g4_initUserConfig()`初始化 Flash 配置系统
2. 调用`CAN_LoadIdFromFlash()`从 Flash 加载保存的 CAN ID
3. 如果 Flash 中没有有效数据，使用默认 ID `CAN_ID_MOTOR_DEFAULT` (0x7F)

### 收到更改 ID 指令时

1. 解析 CAN 指令获取新的 ID
2. 更新全局变量`my_can_id`和`canId`
3. 调用`CAN_SaveIdToFlash(canId)`保存到 Flash
4. 发送应答帧确认 ID 更改

## 数据验证

- CAN ID 范围验证：1-127
- 如果保存的 ID 无效，自动使用默认值
- Flash 写入失败时会有错误处理（可扩展日志输出）

## 注意事项

1. **Flash 寿命**: STM32G4 的 Flash 有写入次数限制，频繁更改 ID 可能影响 Flash 寿命
2. **中断安全**: Flash 写入操作会禁用中断，确保数据一致性
3. **错误处理**: 当前实现中 Flash 写入失败时只记录状态，可根据需要添加重试机制

## 测试方法

1. 发送 CAN ID 更改指令，验证 ID 是否正确保存
2. 重启系统，验证 ID 是否正确加载
3. 发送无效 ID（如 0 或>127），验证是否使用默认值
4. 多次更改 ID，验证 Flash 写入的稳定性

## 扩展功能

可以类似地实现其他参数的 Flash 保存：

- CAN 主站 ID (`CAN_MASTER_ID`)
- 电机参数
- 控制参数等

只需要在`stm32g4_variable_bridge.h`中添加相应的宏定义，并在需要时调用`saveAllVariablesToFlash()`即可。
