/**
  ******************************************************************************
  * @file    EncoderAlignmentApp.h,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   Declaration of encoder alignment application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
  
#ifndef __EncoderAlignmentApp_h
#define __EncoderAlignmentApp_h



#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_interface.h"
#include "user_application.h"
#include "encoder_speed_pos_fdbk.h"
#include "PositionProfileGenerator.h"
/* Exported constants --------------------------------------------------------*/
typedef enum {
    EAA_IDLE,
    EAA_ALIGNMENT,
    EAA_DIR_ID,
    EAA_COLLECT_OFFSET_FWD,
    EAA_COLLECT_OFFSET_BWD,
    EAA_CALC_n_SAVE,
    EAA_FINISHED,
} EAA_STATE_t;


/* Exported Variables --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct {
  UserApplication_Handle_t _Super;
  ENCODER_Handle_t*        pEncoder;
  PositionProfileGenerator_Handle_t* pPosGen;

  uint32_t TimeStamp;
  uint32_t AlignmentDuration;
  uint32_t SpinningDuration;
  uint16_t AlignedMecAngle;
  int32_t IdrefStep;
  int32_t PosAccumulator;
  int16_t Idref;
  int16_t error;

  union {
    uint16_t all;
    struct {
      uint16_t Start         : 1;
      uint16_t EnableSave2EE : 1;
      uint16_t Rsvd          : 14;
    } bits;

  } flags;

  EAA_STATE_t state;
} EncoderAlignmentApp_Handle_t;

void EncoderAlignmentApp_Init(UserApplication_Handle_t* pSuper);
void EncoderAlignmentApp_Reset(UserApplication_Handle_t* pSuper);
void EncoderAlignmentApp_OnLowFrequencyUpdate(UserApplication_Handle_t* pSuper);
void EncoderAlignmentApp_OnBackground(UserApplication_Handle_t* pSuper);


#ifdef __cplusplus
}
#endif /* __cpluplus */



#endif