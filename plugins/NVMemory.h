/**
 ******************************************************************************
 * @file    NVMemory.h,
 * @author  Motor Control Comptence Center, ST Microelectronics
 * @brief   Declaration of NV Memroy implementation
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
#ifndef __NVMemory_H
#define __NVMemory_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "eeprom_emul_types.h"


/* Exported constants --------------------------------------------------------*/

/* Exported type -------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

void NV_Init(void);
EE_Status NV_RequestWriteData(uint16_t address, uint32_t value);
EE_Status NV_ReadData(uint16_t address, uint32_t* pValue);
EE_Status NV_NMI_Handler(void);
void NV_Housekeeping(void);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __NVMemory_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
