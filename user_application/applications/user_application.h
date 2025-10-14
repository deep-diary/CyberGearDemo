/**
  ******************************************************************************
  * @file    user_application.h,
  * @author  Motor Control Comptence Center, ST Microelectronics
  * @brief   super class for user tasks
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
#ifndef __user_application_H
#define __user_application_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_interface.h"

/* Exported constants --------------------------------------------------------*/

/* Exported type -------------------------------------------------------------*/
typedef struct UserApplication_Handle UserApplication_Handle_t;


struct UserApplication_Handle
{
	MCI_Handle_t* pMCI;

	void (*pFctInit)(UserApplication_Handle_t* pHandle);
	void (*pFctReset)(UserApplication_Handle_t* pHandle);
	void (*pFctOnStart)(UserApplication_Handle_t* pHandle);
	void (*pFctOnExit)(UserApplication_Handle_t* pHandle);
	void (*pFctPreLowFreqUpdate)(UserApplication_Handle_t* pHandle);
	void (*pFctPostLowFreqUpdate)(UserApplication_Handle_t* pHandle);
	void (*pFctPreMediumFreqUpdate)(UserApplication_Handle_t* pHandle);
	void (*pFctPostMediumFreqUpdate)(UserApplication_Handle_t* pHandle);
	void (*pFctPreHighFreqUpdate)(UserApplication_Handle_t* pHandle);
	void (*pFctPostHighFreqUpdate)(UserApplication_Handle_t* pHandle);
	void (*pFctBackgroundUpdate)(UserApplication_Handle_t* pHandle);

	bool Activated;
	bool OneShootTask;
	bool OneShootTaskFinished;

};


/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

__STATIC_INLINE void UserApplication_Init(UserApplication_Handle_t* pHandle) {
	if (NULL != pHandle->pFctInit) {
		pHandle->pFctInit(pHandle);
	}
}

__STATIC_INLINE void UserApplication_Reset(UserApplication_Handle_t* pHandle) {
	if (NULL != pHandle->pFctReset/* && pHandle->Activated*/) {
		pHandle->pFctReset(pHandle);
	}
}

__STATIC_INLINE void UserApplication_OnStart(UserApplication_Handle_t* pHandle) {
	if (NULL != pHandle->pFctOnStart && pHandle->Activated) {
		pHandle->pFctOnStart(pHandle);
	}
}

__STATIC_INLINE void UserApplication_OnExit(UserApplication_Handle_t* pHandle) {
	if (NULL != pHandle->pFctOnExit /* && pHandle->Activated*/) {
		pHandle->pFctOnExit(pHandle);
	}

	if (RUN == MCI_GetSTMState(pHandle->pMCI)) {
		MCI_StopMotor(pHandle->pMCI);
	}	
}

__STATIC_INLINE void UserApplication_PreLowFrequencyUpdate(UserApplication_Handle_t* pHandle) {
	if (NULL != pHandle->pFctPreLowFreqUpdate && pHandle->Activated) {
		pHandle->pFctPreLowFreqUpdate(pHandle);
	}
}

__STATIC_INLINE void UserApplication_PostLowFrequencyUpdate(UserApplication_Handle_t* pHandle) {
	if (NULL != pHandle->pFctPostLowFreqUpdate && pHandle->Activated) {
		pHandle->pFctPostLowFreqUpdate(pHandle);
	}
}

__STATIC_INLINE void UserApplication_PreMediumFrequencyUpdate(UserApplication_Handle_t* pHandle) {
	if (NULL != pHandle->pFctPreMediumFreqUpdate && pHandle->Activated) {
		pHandle->pFctPreMediumFreqUpdate(pHandle);
	}
}

__STATIC_INLINE void UserApplication_PostMediumFrequencyUpdate(UserApplication_Handle_t* pHandle) {
	if (NULL != pHandle->pFctPostMediumFreqUpdate && pHandle->Activated) {
		pHandle->pFctPostMediumFreqUpdate(pHandle);
	}
}

__STATIC_INLINE void UserApplication_PreHighFrequencyUpdate(UserApplication_Handle_t* pHandle) {
	if (NULL != pHandle->pFctPreHighFreqUpdate && pHandle->Activated) {
		pHandle->pFctPreHighFreqUpdate(pHandle);
	}
}

__STATIC_INLINE void UserApplication_PostHighFrequencyUpdate(UserApplication_Handle_t* pHandle) {
	if (NULL != pHandle->pFctPostHighFreqUpdate && pHandle->Activated) {
		pHandle->pFctPostHighFreqUpdate(pHandle);
	}
}

__STATIC_INLINE void UserApplication_BackgroundUpdate(UserApplication_Handle_t* pHandle) {
	if (NULL != pHandle->pFctBackgroundUpdate && pHandle->Activated) {
		pHandle->pFctBackgroundUpdate(pHandle);
	}
}

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __user_application_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
		