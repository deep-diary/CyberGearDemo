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

void MCalculateMotorPhase(void)
{
  int16_t hElAngle = 0;
  //int16_t hMecAngle =0;

  // float theta_actual = 0;
  // float v_d = V_CAL;                                                             //Put all volts on the D-Axis
  // float v_q = 0.0f;
  // float v_u, v_v, v_w = 0;
  // float dtc_u, dtc_v, dtc_w = .5f;
  // int sample_counter = 0;

  qd_t Vqd;
  alphabeta_t  Valphabeta;
  Vqd.q = 0; Vqd.d = 3000;//2000对应5A
  PWMC_Handle_t *pwmcHandleCali;
  pwmcHandleCali = pwmcHandle[M1];
  //int16_t zeroAngleOffset=0;
  int32_t theta_end =0 ,theta_start =0;



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
}
        CalibrationTime =0; 
}       
}

void CalculateOffsetAngle(void);




