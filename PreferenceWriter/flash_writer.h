/**
  ******************************************************************************
  * @file    flash_writer.h,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   Declaration of extra instances
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
#ifndef __FLASHWRITER_H
#define __FLASHWRITER_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_63   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (ADDR_FLASH_PAGE_63 + FLASH_PAGE_SIZE - 1)   /* End @ of user Flash area */

#define DATA_32                 ((uint32_t)0x12345678)
#define DATA_64                 ((uint64_t)0x1234567812345678)
/* Exported type -------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __mc_config_ext_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/