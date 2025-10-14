/**
  ******************************************************************************
  * @file    PositionProfileGenerator.h,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   Declaration of position profile generator
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
#ifndef __PositionProfileGenerator_H
#define __PositionProfileGenerator_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/
#define PPG_FILTER_BUFFER_SIZE  1024

typedef enum {
  PPG_RELATIVE,
  PPG_ABSOLUTE,
} PPG_POS_REF_TYPE;

/* Exported type -------------------------------------------------------------*/


typedef struct {
  int32_t target;
  int32_t preFilterOutput;
  int32_t output;
  int32_t filterBuffer[PPG_FILTER_BUFFER_SIZE];
  int64_t filterAcc;
  int32_t setVelocity;  // always positive
  int32_t actualVelocity;

  int16_t bufferIndex;
  int16_t filterConst;

} PositionProfileGenerator_Handle_t;


/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void PositionProfileGenerator_Reset(PositionProfileGenerator_Handle_t* pHandle);
int32_t PositionProfileGenerator_Update(PositionProfileGenerator_Handle_t* pHandle);
void PositionProfileGenerator_Replan(PositionProfileGenerator_Handle_t* pHandle, int32_t target, PPG_POS_REF_TYPE refType);
void PositionProfileGenerator_SetFilterConst(PositionProfileGenerator_Handle_t* pHandle, uint32_t filterConst);
void PositionProfileGenerator_SetVelocity(PositionProfileGenerator_Handle_t* pHandle, int32_t velocity);
int32_t PositionProfileGenerator_GetProfilePosition(PositionProfileGenerator_Handle_t* pHandle);
int32_t PositionProfileGenerator_GetRemainDistance(PositionProfileGenerator_Handle_t* pHandle);
bool PositionProfileGenerator_IsProfileFinished(PositionProfileGenerator_Handle_t* pHandle);
void PositionProfileGenerator_PresetPosition(PositionProfileGenerator_Handle_t* pHandle, int32_t currentPosition);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __PositionProfileGenerator_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
    
