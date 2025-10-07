/**
 ******************************************************************************
 * @file    ChirpGenerator.h,
 * @author  Motor Control Comptence Center, ST Microelectronics
 * @brief   Declaration for chirp generator
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
#ifndef __SineGenerator_H
#define __SineGenerator_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "ramp_ext_mngr.h"

/* Exported constants --------------------------------------------------------*/
#define CPG_FREQUENCY_SCALE_BITS 10
/* Exported type -------------------------------------------------------------*/
typedef struct ChirpGenerator {
  RampExtMngr_Handle_t rampMngr;

  int16_t  hAmplitude;
  uint16_t hStartFrequency01Hz;
  uint16_t hEndFrequency01Hz;
  uint16_t hDurationms;
  uint32_t wAngle;
  uint32_t wAngleStep;
  uint32_t wCurrentFrequencyExt;
  int32_t  wInvUpdateFrequency;

  int16_t output;
  /* for debug */
  uint16_t hAngle; 
  uint16_t hPrevAngle;

  bool chirpMode;

} SineGenerator_Handle_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void SineGenerator_Reset(SineGenerator_Handle_t* pHandle);

int16_t SineGenerator_Update(SineGenerator_Handle_t* pHandle);

void SineGenerator_SetFrequency(SineGenerator_Handle_t* pHandle, uint16_t hStartFrequency01Hz, uint16_t hEndFrequency01Hz);

__STATIC_INLINE bool SineGenerator_IsCompleted(SineGenerator_Handle_t* pHandle)
{
  return REMNG_RampCompleted(&pHandle->rampMngr);
}

__STATIC_INLINE void SineGenerator_SetUpdateFrequency(SineGenerator_Handle_t* pHandle, uint16_t frequencyHz)
{
  pHandle->rampMngr.FrequencyHz = frequencyHz;
}

__STATIC_INLINE bool SineGenerator_IsCycleStart(SineGenerator_Handle_t* pHandle)
{
  return (pHandle->hPrevAngle > 32767 && pHandle->hAngle < 32767);
}

__STATIC_INLINE uint8_t SineGenerator_GetQuarter(SineGenerator_Handle_t* pHandle) {
  return (uint8_t)(((uint32_t)pHandle->hAngle * 4) >> 16);
}


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __SineGenerator_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
