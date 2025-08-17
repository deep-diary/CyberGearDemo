/**
  ******************************************************************************
  * @file    Scheduler.c
  * @author  Motor Control Competence Center, ST Microelectronics
  * @brief   Implementation of the schedular beyond the one used in MCSDK
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
#include "Scheduler.h"

/* Extra Includes -------------------------------------------------------------*/
#include "stm32g4xx_hal_conf.h"
#include "drive_parameters.h"
#include "stm32g4xx.h"
#include "core_cm4.h"
#include "parameters_conversion.h"
#include "mc_tasks.h"

/* Private constants --------------------------------------------------------*/
#ifndef LOW_FREQUENCY_TASK_RATE
#define LOW_FREQUENCY_TASK_RATE           (POSITION_LOOP_FREQUENCY_HZ)
#endif
#define LOW_FREQENCY_TASK_DIVIDER         ((MEDIUM_FREQUENCY_TASK_RATE / LOW_FREQUENCY_TASK_RATE))

#define LowFrequencyTaskISR 			        CRS_IRQHandler
#define FIRE_LOW_FREQENCY_ISR()  		      NVIC->STIR = CRS_IRQn

#define ENABLE_LOW_FREQENCY_TASK_ISR()   do {\
                                      NVIC_SetPriority(CRS_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),TICK_INT_PRIORITY+1, 0));\
                                      NVIC_EnableIRQ(CRS_IRQn);\
                                    } while(0)

#define LOW_FREQENCY_TASK_UPDATE_PHASE			0


/* Private type -------------------------------------------------------------*/

/* Private variables --------------------------------------------------------*/
static uint8_t mediumFrequencyTaskCounter = 0;

/* Private functions ------------------------------------------------------- */



/* Global functions ------------------------------------------------------- */

/**
 * @brief Initialization of the schedular.
 */
void Scheduler_Init(void)
{
  // ENABLE_MEDIUM_FREQENCY_TASK_ISR();
  ENABLE_LOW_FREQENCY_TASK_ISR();
}



/**
 * @brief scheduler update at the high frequency task, basically it means current 
 * 		  loop. so this routine should be called at the end of current loop ISR / 
 * 		  PWM ISR 
 */
void Scheduler_MediumFrequencyUpdate(void)
{
	if (mediumFrequencyTaskCounter == LOW_FREQENCY_TASK_UPDATE_PHASE) {
		FIRE_LOW_FREQENCY_ISR();
	}

	mediumFrequencyTaskCounter++;
	if (mediumFrequencyTaskCounter >= LOW_FREQENCY_TASK_DIVIDER) {
		mediumFrequencyTaskCounter = 0;
	}
}
