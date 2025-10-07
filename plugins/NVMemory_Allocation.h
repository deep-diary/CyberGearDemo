/**
  ******************************************************************************
  * @file    NVMemory_Allocation.h,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   NV memory allocation
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
#ifndef __NVMemory_Allocation_H
#define __NVMemory_Allocation_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/

/* Exported type -------------------------------------------------------------*/
enum {
	NV_ADDR_RESERVED = 0,
  NV_ADDR_CAN_ID,
	NV_ADDR_ENCODER_ZEROANGLE_OFFSET,
	NV_ADDR_ENCODER_DIR,
};
/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __NVMemory_Allocation_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
		