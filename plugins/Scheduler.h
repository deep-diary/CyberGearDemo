/**
  ******************************************************************************
  * @file    Scheduler.h,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   Declaration to scheduler beyond the SDK one, the new scheduler can 
  * 		 synchronize the current loop, speed loop and position loop, so that 
  * 		 it can be more easily interfaced to time-sync-critical application 
  * 		 like EtherCAT 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Scheduler_H
#define __Scheduler_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/* Exported constants --------------------------------------------------------*/
#define LowFrequencyTaskISR 			        CRS_IRQHandler

/* Exported type -------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/**
 * @brief Initialization of the schedule just after the clock is initialized.
 * 
 */
// void Scheduler_PreInit(void);


/**
 * @brief Initialization of the schedular.
 */
void Scheduler_Init(void);

/**
 * @brief scheduler update at the medium frequency task, basically it means speed 
 * 		  loop.
 */
void Scheduler_MediumFrequencyUpdate(void);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __Scheduler_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
		
