/**
 ******************************************************************************
 * @file    utils.h,
 * @author  Motor Control Comptence Center, ST Microelectronics
 * @brief   Declaration for the utilities
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
#ifndef __utils_H
#define __utils_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define TRIAG_THETA_Q          28     /*< Make sure mantissa x max gap less than 31 bit*/
#define TRIAG_TABLE_SIZE_SHIFT 8
#define TRIAG_MANTISSA_Q       (TRIAG_THETA_Q - TRIAG_TABLE_SIZE_SHIFT)
#define TRIAG_TABLE_INDEX_MASK ((1UL << TRIAG_TABLE_SIZE_SHIFT) - 1)

/* Exported type -------------------------------------------------------------*/
typedef struct {
  int16_t _sin;
  int16_t _cos;
} Triangle_t;
/* Exported variables --------------------------------------------------------*/
extern const int16_t SIN_TABLE[257];

/* Exported functions ------------------------------------------------------- */
Triangle_t SinCos(uint32_t theta);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __utils_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
