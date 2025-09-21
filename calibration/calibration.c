/**
  ******************************************************************************
  * @file    calibration.c
  * @author  Motor Control Competence Center, ST Microelectronics
  * @brief   Used for the calibration of magnetic encoders
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "calibration.h"
/* Extra Includes ------------------------------------------------------------*/
#include"mc_tasks.h"
#include"mc_config.h"
/* Private constants ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
#define CALIBRATION_FLASH_ADDRESS 0x0801F800 // 确保该地址在Flash可用区域

#define CALIB_PAGE_SIZE 2048 // STM32G4系列Flash页大小
/* Private functions ---------------------------------------------------------*/

/* Global functions ----------------------------------------------------------*/

/**
 * @brief 这个函数会识别电机相序和磁编方向是否一致
 * 
 * 
 */
extern uint16_t CalibrationTime;
extern uint8_t calibrationflag;
extern ENCODER_Handle_t ENCODER_M1;
extern float_t theta_ref;
uint16_t spiAngle =0;

HAL_StatusTypeDef Save_Calibration_To_Flash(void)
{
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef eraseConfig;
  uint32_t pageError;
 __disable_irq();
  // 解锁Flash
  HAL_FLASH_Unlock();
  
  // 配置擦除参数
  eraseConfig.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseConfig.Banks = FLASH_BANK_1; // 根据实际芯片选择
  eraseConfig.Page = 63;
  eraseConfig.NbPages = 1;
  
  // 擦除Flash页
  status = HAL_FLASHEx_Erase(&eraseConfig, &pageError);
  if (status != HAL_OK) {
    HAL_FLASH_Lock();
    return status;
  }
  
  // 准备64位数据（使用双字编程）
  uint64_t dataToWrite = 0;
  
  // 将16位标定值放入64位数据的低16位, 方向值放入
  dataToWrite = (uint64_t)((uint16_t)ENCODER_M1.zeroAngleOffset) ;
  dataToWrite = dataToWrite |((uint64_t)ENCODER_M1.direction << 16);
  dataToWrite = (dataToWrite & 0x0000FFFFFFFFFFFFULL) | ((uint64_t)CALIB_ID << 48);
  
  
  // 写入数据（使用双字编程）
  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                            CALIBRATION_FLASH_ADDRESS, 
                            dataToWrite);

  // 锁定Flash
  HAL_FLASH_Lock();
  __enable_irq();
  return status;
}

/**
  * @brief  从Flash加载标定值
  * @retval 加载的电角度偏移值
  */
int16_t Load_Calibration_From_Flash(void)
{
  // 直接读取Flash地址的值
  return *(volatile int16_t*)CALIBRATION_FLASH_ADDRESS;
}

int16_t Load_CalibrationDirection_From_Flash(void)
{
    // 读取Flash地址+2的位置，对应64位数据的第16-31位（方向值）
    return *(volatile int16_t*)(CALIBRATION_FLASH_ADDRESS + 2);
}


int16_t Load_CalibID_From_Flash(void)
{
  // 直接读取Flash地址的值
  return *(volatile int16_t*)(CALIBRATION_FLASH_ADDRESS+6);
}

void MCalculateMotorPhaseInt(void){
uint16_t calib_flag;
calib_flag = Load_CalibID_From_Flash(); 
if (calib_flag != CALIB_ID)		//first time to calibraion.
	{calibrationflag =1;}
        else
        { 
        ENCODER_M1.zeroAngleOffset = Load_Calibration_From_Flash();
        ENCODER_M1.direction = Load_CalibrationDirection_From_Flash();
        }  
}


void MCalculateMotorPhase(void)
{
  int16_t hElAngle = 0;

  qd_t Vqd;
  alphabeta_t  Valphabeta;
  Vqd.q = 0; Vqd.d = 3000;//2000对应5A
  PWMC_Handle_t *pwmcHandleCali;
  pwmcHandleCali = pwmcHandle[M1];

TIM1->BDTR |= TIM_BDTR_MOE; // 强制使能主输出


 //D轴产生10A电流6S，锁轴


if (CalibrationTime<=3000)
{
Vqd.d = CalibrationTime;
Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
PWMC_SetPhaseVoltage(pwmcHandleCali, Valphabeta);
TIM1->CCR3 = pwmcHandleCali->CntPhC;
TIM1->CCR2 = pwmcHandleCali->CntPhB;
TIM1->CCR1 = pwmcHandleCali->CntPhA;
}
else if (CalibrationTime > 3000 && CalibrationTime <= 8000)
{
        Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
        PWMC_SetPhaseVoltage(pwmcHandleCali, Valphabeta);
        TIM1->CCR3 = pwmcHandleCali->CntPhC;
        TIM1->CCR2 = pwmcHandleCali->CntPhB;
        TIM1->CCR1 = pwmcHandleCali->CntPhA;
}
else
{
ENCODER_M1.zeroAngleOffset = (-ENCODER_M1._Super.hMecAngle);
HAL_Delay(1000);
theta_ref =0;
while(theta_ref < 4){ 
 hElAngle = (int16_t)(theta_ref * 32767);
      
 Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
 PWMC_SetPhaseVoltage(pwmcHandleCali, Valphabeta);
        TIM1->CCR3 = pwmcHandleCali->CntPhC;
        TIM1->CCR2 = pwmcHandleCali->CntPhB;
        TIM1->CCR1 = pwmcHandleCali->CntPhA;
}  
if (theta_ref ){
     if (ENCODER_M1._Super.wMecAngle >0)
         {
          ENCODER_M1.direction =  2;
         } else{
          ENCODER_M1.direction = -2;
         }
 Vqd.d =0;
 Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
 PWMC_SetPhaseVoltage(pwmcHandleCali, Valphabeta);
        TIM1->CCR3 = pwmcHandleCali->CntPhC;
        TIM1->CCR2 = pwmcHandleCali->CntPhB;
        TIM1->CCR1 = pwmcHandleCali->CntPhA;
 
        calibrationflag =0;
        HAL_Delay(1000);
Save_Calibration_To_Flash();
}
        CalibrationTime =0; 
}       
}

void CalculateOffsetAngle(void);




