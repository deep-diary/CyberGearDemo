/**
  ******************************************************************************
  * @file    outAngleCalculate.h
  * @brief   输出角度计算模块头文件
  * 
  * 功能描述：
  * 1. 将采集的Va和Vb进行归一化处理到(-1,1)范围
  * 2. 使用反正切函数计算角度并转换至0-65535范围
  * 3. 添加偏置校准角度
  * 4. 根据扇区范围计算扇区
  * 5. 根据hMecAngle进行进一步的校准
  * 
  * 基于MATLAB程序angleCalculate.m和angleCalculateTest2.m的功能实现
  ******************************************************************************
  */

#ifndef __OUT_ANGLE_CALCULATE_H
#define __OUT_ANGLE_CALCULATE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/* Exported constants --------------------------------------------------------*/
#define ANGLE_MAX_VALUE       65535U     ///< 角度最大值 (0-65535对应0-2π)
#define PI_VALUE              3.14159265358979323846f  ///< π值
#define TWO_PI_VALUE          (2.0f * PI_VALUE)        ///< 2π值
#define SECTOR_COUNT          10         ///< 扇区数量
#define SECTOR_BOUNDARY_COUNT (SECTOR_COUNT + 1) ///< 扇区边界数量

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  角度计算配置参数结构体
  */
typedef struct {
    uint16_t Va_max;        ///< Va最大值 (ADC采样值)
    uint16_t Va_min;        ///< Va最小值 (ADC采样值)
    uint16_t Vb_max;        ///< Vb最大值 (ADC采样值)
    uint16_t Vb_min;        ///< Vb最小值 (ADC采样值)
    int16_t bias_angle;     ///< 偏置校准角度 (0-65535范围)
    uint16_t sector_bounds[SECTOR_BOUNDARY_COUNT]; ///< 扇区边界数组
} AngleCalcConfig_t;

/**
  * @brief  角度计算中间量结构体 - 包含所有计算过程中的中间值
  */
typedef struct {
    uint16_t Va_raw;           ///< 原始Va ADC值
    uint16_t Vb_raw;           ///< 原始Vb ADC值
    float Va_norm;             ///< 归一化后的Va值 (-1.0到1.0)
    float Vb_norm;             ///< 归一化后的Vb值 (-1.0到1.0)
    float angle_rad;           ///< 弧度制角度 (-π到π)
    float angle_rad_adj;       ///< 调整后的弧度制角度 (0到2π)
    uint16_t angle_before_bias; ///< 偏置校准前的角度 (0-65535)
    uint16_t angle_after_bias;  ///< 偏置校准后的角度 (0-65535)
    uint8_t sector_initial;    ///< 初始扇区编号 (0-9)
    uint8_t sector_final;      ///< 最终扇区编号 (0-9)
    int16_t hMecAngle_input;   ///< 输入的机械角度
    unsigned int unsigned_hMecAngle; ///< 无符号机械角度
    uint16_t calibrated_angle; ///< 校准后的角度中间值
} AngleCalcIntermediate_t;

/**
  * @brief  角度计算句柄结构体 - 包含配置、中间量和输出结果
  */
typedef struct {
    AngleCalcConfig_t config;           ///< 配置参数
    AngleCalcIntermediate_t intermediate; ///< 计算中间量
    uint16_t outhMecAngle;              ///< 输出机械角度 (0-65535)
    uint8_t initialized;                ///< 初始化标志
} AngleCalcHandle_t;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化角度计算模块
  * @param  pHandle 角度计算句柄指针
  * @param  pConfig 配置参数指针
  * @retval 0: 成功, -1: 失败
  */
int OutAngleCalc_Init(AngleCalcHandle_t* pHandle, AngleCalcConfig_t* pConfig);

/**
  * @brief  计算输出角度
  * @param  pHandle 角度计算句柄指针
  * @param  Va 输入Va电压值 (uint16类型，通常是ADC采样值)
  * @param  Vb 输入Vb电压值 (uint16类型，通常是ADC采样值)
  * @param  hMecAngle 输入机械角度
  * @retval 0: 成功, -1: 失败
  */
int OutAngleCalc_Compute(AngleCalcHandle_t* pHandle, uint16_t Va, uint16_t Vb, int16_t hMecAngle);

/**
  * @brief  设置偏置角度
  * @param  pHandle 角度计算句柄指针
  * @param  bias_angle 新的偏置角度 (0-65535)
  */
void OutAngleCalc_SetBiasAngle(AngleCalcHandle_t* pHandle, int16_t bias_angle);

/**
  * @brief  获取当前配置
  * @param  pHandle 角度计算句柄指针
  * @param  pConfig 配置参数指针
  */
void OutAngleCalc_GetConfig(AngleCalcHandle_t* pHandle, AngleCalcConfig_t* pConfig);

/**
  * @brief  获取计算结果
  * @param  pHandle 角度计算句柄指针
  * @param  pResult 计算结果指针
  */
void OutAngleCalc_GetOutputAngle(AngleCalcHandle_t* pHandle, uint16_t* outhMecAngle);

/**
  * @brief  获取计算中间量
  * @param  pHandle 角度计算句柄指针
  * @param  pIntermediate 中间量指针
  */
void OutAngleCalc_GetIntermediate(AngleCalcHandle_t* pHandle, AngleCalcIntermediate_t* pIntermediate);

/**
  * @brief  重置角度计算模块
  * @param  pHandle 角度计算句柄指针
  */
void OutAngleCalc_Reset(AngleCalcHandle_t* pHandle);

#ifdef __cplusplus
}
#endif

#endif /* __OUT_ANGLE_CALCULATE_H */