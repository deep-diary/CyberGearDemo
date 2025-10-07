/**
 ******************************************************************************
 * @file    NVMemory.c
 * @author  Motor Control Competence Center, ST Microelectronics
 * @brief   Implementation of NV memory function
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
#include "NVMemory.h"

/* Extra Includes -------------------------------------------------------------*/
#include <stdbool.h>

#include "eeprom_emul.h"
#include "stm32g4xx_hal.h"

/* Private constants --------------------------------------------------------*/
#define WRITE_REQUEST_FIFO_DEPTH 8
/* Private type -------------------------------------------------------------*/
typedef struct {
  uint32_t data;
  uint16_t address;
  bool     ongoing;
} NVWriteRequest_Handle_t;

/* Private variables --------------------------------------------------------*/
static NVWriteRequest_Handle_t writeRequest[WRITE_REQUEST_FIFO_DEPTH];
/* Private functions ------------------------------------------------------- */
// #define HAL_FLASH_IsLocked()	(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U)
/* Global functions ------------------------------------------------------- */
extern void                    Error_Handler(void);

/* During the cleanup phase in EE_Init, AddressRead is the address being read */
extern __IO uint32_t AddressRead;
/* Flag equal to 1 when the cleanup phase is in progress, 0 if not */
extern __IO uint8_t  CleanupPhase;

/**
 * @brief  Programmable Voltage Detector (PVD) Configuration
 *         PVD set to level 6 for a threshold around 2.9V.
 * @param  None
 * @retval None
 */
static void PVD_Config(void)
{
  PWR_PVDTypeDef sConfigPVD;
  sConfigPVD.PVDLevel = PWR_PVDLEVEL_6;
  sConfigPVD.Mode     = PWR_PVD_MODE_IT_RISING;
  if (HAL_PWR_ConfigPVD(&sConfigPVD) != HAL_OK) {
    Error_Handler();
  }

  /* Enable PVD */
  HAL_PWR_EnablePVD();

  /* Enable and set PVD Interrupt priority */
  HAL_NVIC_SetPriority(PVD_PVM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);
}

void NV_Init(void)
{
  EE_Status status = EE_OK;

  /* Enable and set FLASH Interrupt priority */
  /* FLASH interrupt is used for the purpose of pages clean up under interrupt */
  HAL_NVIC_SetPriority(FLASH_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FLASH_IRQn);

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);

  /* Configure Programmable Voltage Detector (PVD) (optional) */
  /* PVD interrupt is used to suspend the current application flow in case
     a power-down is detected, allowing the flash interface to finish any
     ongoing operation before a reset is triggered. */
  PVD_Config();

  /* Set EEPROM emulation firmware to erase all potentially incompletely erased
     pages if the system came from an asynchronous reset. Conditional erase is
     safe to use if all Flash operations where completed before the system reset */

  /* Unlock the Flash Program Erase controller */
  HAL_FLASH_Unlock();

  /* System reset comes from a power-on reset: Forced Erase */
  /* Initialize EEPROM emulation driver (mandatory) */
  status = EE_Init(EE_CONDITIONAL_ERASE);

  HAL_FLASH_Lock();

  if (status != EE_OK) {
    Error_Handler();
  }

  for (int i = 0; i < WRITE_REQUEST_FIFO_DEPTH; i++) {
    writeRequest[i].ongoing = false;
  }
}

EE_Status NV_RequestWriteData(uint16_t address, uint32_t value)
{
  for (int i = 0; i < WRITE_REQUEST_FIFO_DEPTH; i++) {
    if (!writeRequest[i].ongoing) {
      writeRequest[i].ongoing = true;
      writeRequest[i].address = address;
      writeRequest[i].data    = value;
      return EE_OK;
    }
  }
  return EE_WRITE_ERROR;
}

EE_Status NV_ReadData(uint16_t address, uint32_t* pValue) { return EE_ReadVariable32bits(address, pValue); }

EE_Status NV_NMI_Handler(void)
{
  if (CleanupPhase == 1) {
    if ((AddressRead >= START_PAGE_ADDRESS) && (AddressRead <= END_EEPROM_ADDRESS)) {
      /* Delete the corrupted flash address */
      if (EE_DeleteCorruptedFlashAddress((uint32_t)AddressRead) == EE_OK) {
        /* Resume execution if deletion succeeds */
        return EE_OK;
      }
      /* If we do not succeed to delete the corrupted flash address */
      /* This might be because we try to write 0 at a line already considered at 0 which is a forbidden operation */
      /* This problem triggers PROGERR, PGAERR and PGSERR flags */
      else {
        /* We check if the flags concerned have been triggered */
        if ((__HAL_FLASH_GET_FLAG(FLASH_FLAG_PROGERR)) && (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR)) &&
            (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR))) {
          /* If yes, we clear them */
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PROGERR);
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);

          /* And we exit from NMI without doing anything */
          /* We do not invalidate that line because it is not programmable at 0 till the next page erase */
          /* The only consequence is that this line will trigger a new NMI later */
          return EE_OK;
        }
      }
    }
  } else {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);
    return EE_OK;
  }

  return EE_UNHANDLED_NMI_ERROR;
}

void NV_Housekeeping(void)
{
  for (int i = 0; i < WRITE_REQUEST_FIFO_DEPTH; i++) {
    if (writeRequest[i].ongoing) {
      EE_Status status = EE_OK;
      /* Unlock the Flash Program Erase controller */
      HAL_FLASH_Unlock();

      status = EE_WriteVariable32bits(writeRequest[i].address, writeRequest[i].data);
      /* Start cleanup IT mode, if cleanup is needed */
      if ((status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {
        status |= EE_CleanUp();
      }

      HAL_FLASH_Lock();

      writeRequest[i].ongoing = false;
    }
  }
}
