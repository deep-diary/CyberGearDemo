/**
  ******************************************************************************
  * @file    PositionControl.h,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   Interface to position control
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
#ifndef __PosCtrl_H
#define __PosCtrl_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "speed_pos_fdbk.h"
#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"

/* Exported type -------------------------------------------------------------*/

typedef enum {
  MRT_ROTARY,
  MRT_LINEAR
} MOTOR_ROTATE_TYPE;

typedef enum {
  POSCTR_PROFILE_MODE,
  POSCTR_MOTION_MODE
} POSCTRL_MODE;

typedef struct 
{
	SpeednPosFdbk_Handle_t* 	pSPD;

	int32_t wPositionAccumulator;
	int32_t	wError;			/*!< save error for observation */
	int32_t	wOutput;		/*!< save output for obvervation */
	int32_t wSpeedFeedforward;
  int32_t wPreveDeltaPosition;
  int32_t wAccelerationReference;
	int32_t wPositionRegularOutput;
	int16_t KpGain;
	int16_t feedForwardGain;
  uint32_t maxSpeedDpp;
  uint8_t EncoderResolutionBits;
  MOTOR_ROTATE_TYPE motorType;
  POSCTRL_MODE mode;

} PosCtrl_Handle_t;

/* Exported constants --------------------------------------------------------*/
#ifdef ABS_ENCODER
#define POSITION_RESOLUTION_BITS      17
#else
#define POSITION_RESOLUTION_BITS      16
#endif
// #define POS_DPP_2_SPEED_UNIT_SCALE    12
/*< for linear motor, the following conversion will convert delta pos to speed of 0.1mm/s or 
    0.01mm/s depending on SPEED_UNIT */
// #define POS_DPP_2_SPEED_UNIT_RATIO   (((uint32_t)POSITION_LOOP_FREQUENCY_HZ * SPEED_UNIT) >> POSITION_RESOLUTION_BITS)

#define SPEED_FF_GAIN_DIV_POW2        10
#define POSITION_KP_GAIN_DIV_POW2     8
/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
// void PosCtrl_Init(PosCtrl_Handle_t* pHandle, SpeednPosFdbk_Handle_t* pSTC);

void PosCtrl_Reset(PosCtrl_Handle_t* pHandle);

int32_t PosCtrl_Update(PosCtrl_Handle_t* pHandle, int32_t deltaPosition);

// int32_t PosCtrl_GetAbsReference(PosCtrl_Handle_t* pHandle);

// int32_t PosCtrl_GetCurrentPosition(PosCtrl_Handle_t* pHandle);

void PosCtrl_ForcePositionFeedback(PosCtrl_Handle_t* pHandle, int32_t forcedPosition);

__INLINE int32_t PosCtrl_GetAbsReference(PosCtrl_Handle_t* pHandle) { return pHandle->wPositionAccumulator; }

__INLINE int32_t PosCtrl_GetCurrentPosition(PosCtrl_Handle_t* pHandle) { return pHandle->pSPD->wMecAngle; }


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __PosCtrl_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
		