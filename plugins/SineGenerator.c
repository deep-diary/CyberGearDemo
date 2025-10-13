/**
 ******************************************************************************
 * @file    ChirpGenerator.c
 * @author  Motor Control Competence Center, ST Microelectronics
 * @brief   Implementation of chirp generator
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
#include "SineGenerator.h"

/* Extra Includes -------------------------------------------------------------*/
#ifdef CPG_USE_MC_MATH
#include "mc_math.h"
#else
#include "utils.h"
#endif
/* Private constants --------------------------------------------------------*/

/* Private type -------------------------------------------------------------*/

/* Private variables --------------------------------------------------------*/

/* Private functions ------------------------------------------------------- */

/* Global functions ------------------------------------------------------- */
void SineGenerator_Reset(SineGenerator_Handle_t* pHandle)
{
  pHandle->wInvUpdateFrequency = (1UL << 31) / (pHandle->rampMngr.FrequencyHz * 10);
  if (CPG_SINGLEPOLAR_MODE == pHandle->OutputMode) {
    pHandle->wAngle              = -(1L << (TRIAG_THETA_Q - 2)); /* -90 degree, so that start speed = 0*/
    /* Set amplitude to half to avoid overflow */
    pHandle->hAmplitude >>= 1;
  } else {
    pHandle->wAngle              = 0;
  }
  pHandle->output              = 0;
  #if CPG_USE_MC_MATH
  pHandle->hAngle              = pHandle->wAngle;
  #else
  pHandle->hAngle              = (pHandle->wAngle >> (TRIAG_THETA_Q - 16));
  #endif
  pHandle->hPrevAngle          = pHandle->hAngle;
  if (pHandle->hStartFrequency01Hz != pHandle->hEndFrequency01Hz) {
    REMNG_Init(&pHandle->rampMngr);
    pHandle->wAngleStep = 0;
    REMNG_ExecRamp(&pHandle->rampMngr, (int32_t)pHandle->hStartFrequency01Hz << CPG_FREQUENCY_SCALE_BITS, 0);
    REMNG_ExecRamp(&pHandle->rampMngr, (int32_t)pHandle->hEndFrequency01Hz << CPG_FREQUENCY_SCALE_BITS,
                   pHandle->hDurationms);
    pHandle->chirpMode = true;
  } else {
#ifdef CPG_USE_MC_MATH
    pHandle->wAngleStep = ((int64_t)pHandle->hStartFrequency01Hz * pHandle->rwInvUpdateFrequency) >> (31 - 16);
#else
    pHandle->wAngleStep = ((int64_t)pHandle->hStartFrequency01Hz * pHandle->wInvUpdateFrequency) >> (31 - TRIAG_THETA_Q);
#endif
    pHandle->chirpMode = false;
  }
}

int16_t SineGenerator_Update(SineGenerator_Handle_t* pHandle)
{
  pHandle->hPrevAngle = pHandle->hAngle;
  if (pHandle->chirpMode) {
    int32_t frequencyExt = REMNG_Calc(&pHandle->rampMngr);
#ifdef CPG_USE_MC_MATH
    int32_t angleStep =
        ((int64_t)frequencyExt * pHandle->rwInvUpdateFrequency) >> (31 - (16 - CPG_FREQUENCY_SCALE_BITS));
    pHandle->wAngle += angleStep;
    pHandle->hAngle =  pHandle->wAngle;
    Trig_Components sincos        = MCM_Trig_Functions(pHandle->wAngle);
    pHandle->wCurrentFrequencyExt = frequencyExt;
    if (CPG_SINGLEPOLAR_MODE == pHandle->OutputMode) {
      pHandle->output             = ((int32_t)pHandle->hAmplitude * sincos.hSin >> 15) + pHandle->hAmplitude;
    } else {
      pHandle->output             = (int32_t)pHandle->hAmplitude * sincos.hSin >> 15;
    }
#else
    uint32_t angleStep =
        ((int64_t)frequencyExt * pHandle->wInvUpdateFrequency) >> (31 - (TRIAG_THETA_Q - CPG_FREQUENCY_SCALE_BITS));
    pHandle->wAngle += angleStep;
    pHandle->hAngle  = (pHandle->wAngle >> (TRIAG_THETA_Q - 16));
    Triangle_t sincos = SinCos(pHandle->wAngle);
    if (CPG_SINGLEPOLAR_MODE == pHandle->OutputMode) {
      pHandle->output             = ((int32_t)pHandle->hAmplitude * sincos._sin >> 15) + pHandle->hAmplitude;
    } else {
      pHandle->output             = (int32_t)pHandle->hAmplitude * sincos._sin >> 15;
    }
#endif

  } else {
    pHandle->wAngle += pHandle->wAngleStep;
#ifdef CPG_USE_MC_MATH
    Trig_Components sincos = MCM_Trig_Functions(pHandle->wAngle);
    if (CPG_SINGLEPOLAR_MODE == pHandle->OutputMode) {
      pHandle->output             = ((int32_t)pHandle->hAmplitude * sincos.hSin >> 15) + pHandle->hAmplitude;
    } else {
      pHandle->output             = (int32_t)pHandle->hAmplitude * sincos.hSin >> 15;
    }
    pHandle->hAngle        = pHandle->wAngle;
#else
    Triangle_t sincos = SinCos(pHandle->wAngle);
    if (CPG_SINGLEPOLAR_MODE == pHandle->OutputMode) {
      pHandle->output             = ((int32_t)pHandle->hAmplitude * sincos._sin >> 15) + pHandle->hAmplitude;
    } else {
      pHandle->output             = (int32_t)pHandle->hAmplitude * sincos._sin >> 15;
    }
    pHandle->hAngle   = (pHandle->wAngle >> (TRIAG_THETA_Q - 16));
#endif
  }
  return pHandle->output;
}

void SineGenerator_SetFrequency(SineGenerator_Handle_t* pHandle, uint16_t hStartFrequency01Hz, uint16_t hEndFrequency01Hz)
{
  pHandle->hStartFrequency01Hz = hStartFrequency01Hz;
  pHandle->hEndFrequency01Hz = hEndFrequency01Hz;
  if (hStartFrequency01Hz != hEndFrequency01Hz) {
    REMNG_Init(&pHandle->rampMngr);
    pHandle->wAngleStep = 0;
    REMNG_ExecRamp(&pHandle->rampMngr, (int32_t)hStartFrequency01Hz << CPG_FREQUENCY_SCALE_BITS, 0);
    REMNG_ExecRamp(&pHandle->rampMngr, (int32_t)hEndFrequency01Hz << CPG_FREQUENCY_SCALE_BITS,
                   pHandle->hDurationms);
    pHandle->chirpMode = true;
  } else {
#ifdef CPG_USE_MC_MATH
    pHandle->wAngleStep = ((int64_t)hStartFrequency01Hz * pHandle->rwInvUpdateFrequency) >> (31 - 16);
#else
    pHandle->wAngleStep = ((int64_t)hStartFrequency01Hz * pHandle->wInvUpdateFrequency) >> (31 - TRIAG_THETA_Q);
#endif
    pHandle->chirpMode = false;
  }
}

