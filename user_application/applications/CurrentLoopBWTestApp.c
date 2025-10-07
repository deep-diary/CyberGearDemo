/**
  ******************************************************************************
  * @file    CurrentloopBWTestApp.c
  * @author  Motor Control Competence Center, ST Microelectronics
  * @brief   Implementation of current loop bandwidth test task
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
#include "CurrentloopBWTestApp.h"

/* Extra Includes -------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "parameters_conversion.h"
/* Private constants --------------------------------------------------------*/

/* Private type -------------------------------------------------------------*/

/* Private variables --------------------------------------------------------*/

/* Private functions ------------------------------------------------------- */

/* Global functions ------------------------------------------------------- */
void CurrentloopBWTestApp_OnStart(UserApplication_Handle_t* pSuper)
{
	CurrentloopBWTestApp_Handle_t* pHandle = (CurrentloopBWTestApp_Handle_t*)pSuper;
	qd_t Iqdref = {0,0};
	pHandle->TimeStamp = HAL_GetTick();
	pHandle->PulseLevel = false;
  pHandle->IdRef = ((int32_t)pHandle->IdRef_10BitRes * IQMAX) >> 10;
	/* This will set the control mode to torque mode */
	MCI_ExecTorqueRamp(pHandle->pMCI, 0, 0);
	MCI_SetCurrentReferences(pHandle->pMCI, Iqdref);

}

void CurrentloopBWTestApp_MediumFreqUpdate(UserApplication_Handle_t* pSuper)
{
	CurrentloopBWTestApp_Handle_t* pHandle = (CurrentloopBWTestApp_Handle_t*)pSuper;
	if (MCI_GetSTMState(pHandle->pMCI) == RUN) {
		if (HAL_GetTick() - pHandle->TimeStamp > pHandle->PulseWidth_ms) {
			pHandle->TimeStamp = HAL_GetTick();
			qd_t Iqdref        = {0, 0};
			if (pHandle->PulseLevel) {
				Iqdref.d = pHandle->IdRef;
			}
			pHandle->PulseLevel = !pHandle->PulseLevel;
			MCI_SetCurrentReferences(pHandle->pMCI, Iqdref);
		}
	}
}

void CurrentloopBWTestApp_SetIdRef10BitRes(CurrentloopBWTestApp_Handle_t* pHandle, uint16_t IdRef_10BitRes)
{
  pHandle->IdRef_10BitRes = IdRef_10BitRes;
  pHandle->IdRef = ((int32_t)IdRef_10BitRes * IQMAX) >> 10;
}
