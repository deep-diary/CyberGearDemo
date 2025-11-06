
/**
 ******************************************************************************
 * @file    mc_app_hooks.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file implements default motor control app hooks.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 * @ingroup MCAppHooks
 */

/* Includes ------------------------------------------------------------------*/
#define DATALOGGER_NONE         0
#define DATALOGGER_JSCOPE       1
#define DATALOGGER_STMSTUDIO    2
#define DATALOGGER_TYPE         DATALOGGER_NONE

#define INCREMENTAL_ENCODER     0
#define TAMAGAWA_ENCODER        1
#define MAGNET_ENCODER          2
#define ENCODER_TYPE            MAGNET_ENCODER

#include "mc_app_hooks_servo.h"
#include "outAngleCalculate.h"
#if DATALOGGER_TYPE == DATALOGGER_JSCOPE
#include "JSInterface.h"
#endif
#ifdef ENABLE_IQREF_LPF
#include "LowPassFilter.h"
#endif
#include "Scheduler.h"
#include "SineGenerator.h"
#ifdef ENABLE_STALL_DETECTION
#include "StallDetection.h"
#endif
#if DATALOGGER_TYPE == DATALOGGER_STMSTUDIO
#include "dataAcq.h"
#endif
#include "mc_api.h"
#include "mc_app_hooks.h"
#include "mc_config.h"
#include "parameters_conversion.h"
#include "user_application.h"

#if ENCODER_TYPE == TAMAGAWA_ENCODER
#include "TamagawaAbsEncoder.h"
#elif ENCODER_TYPE == INCREMENTAL_ENCODER
#include "encoder_speed_pos_fdbk.h"
#endif
#ifdef ENABLE_MECH_ID_APP
#include "mech_param_id_app.h"
#endif
#include "can_interface.h"
#include "param_manager.h"
#include "JogApp.h"
#include "BasicApp.h"


SineGenerator_Handle_t sineGenerator;

/* 角度计算模块全局变量 */
 AngleCalcHandle_t s_angleCalcHandle;
static uint8_t s_angleCalcInitialized = 0;

/* 回零和零点设置功能结构体变量定义 */
HomingControl_t s_HomingControl = {
    .homingModeFlag = 0,     // 回零模式标志位
    .homingStartFlag = 0,    // 回零开始标志位
    .homingCounter = 0,      // 回零计数器
    .posRefSetSuccessFlag = 0, // PosRef设置成功标志位
    .posRefSetExecuted = 0,  // PosRef设置已执行标志位，防止重复执行
    .setZeroFlag = 0,        // 设置零点标志位
};

/* 默认角度计算配置 */
static AngleCalcConfig_t s_defaultAngleConfig = {
    .Va_max = 39520,       // 假设12位ADC，最大值4095
    .Va_min = 24320,       // ADC最小值0
    .Vb_max = 41488,       // 假设12位ADC，最大值4095
    .Vb_min = 23664,       // ADC最小值0
    .bias_angle = 0,
    .sector_bounds = {0, 3640, 10922, 18203, 25485, 32766, 40048, 47329, 54611, 61892, 65535}
};

BasicApp_Handle_t BasicApp = {
  ._Super =
      {
      .pMCI                     = &Mci[M1],
      .pFctInit                 = NULL,
      .pFctReset                = BasicApp_OnReset,
      .pFctOnStart              = NULL,
      .pFctOnExit               = BasicApp_OnExit,
      .pFctPreLowFreqUpdate     = NULL,
      .pFctPostLowFreqUpdate    = NULL,
      .pFctPreMediumFreqUpdate  = NULL,
      .pFctPostMediumFreqUpdate = NULL,
      .pFctPreHighFreqUpdate    = NULL,
      .pFctPostHighFreqUpdate   = NULL,
      .pFctBackgroundUpdate     = BasicApp_OnBackground,
      .Activated                = false,
      .OneShootTask             = false,
      .OneShootTaskFinished     = false,
      },
  .Iref        = 0,
  .SpeedRef    = 0,
  .PrevSpeedRef= 0,
  .Durationms  = 1000,
  .Flags       = {0}
        
};


PositionCtrlApp_Handle_t PositionCtrolApp = {
  ._Super =
      {
          .pMCI                     = &Mci[M1],
          .pFctInit                 = NULL,
          .pFctReset                = PositionCtrlApp_OnReset,
          .pFctOnStart              = PositionCtrlApp_OnStart,
          .pFctOnExit               = NULL,
          .pFctPreLowFreqUpdate     = PositionCtrlApp_OnLowFrequencyUpdate,
          .pFctPostLowFreqUpdate    = NULL,
          .pFctPreMediumFreqUpdate  = NULL,
          .pFctPostMediumFreqUpdate = NULL,
          .pFctPreHighFreqUpdate    = NULL,
          .pFctPostHighFreqUpdate   = NULL,
          .pFctBackgroundUpdate     = PositionCtrlApp_OnBackground,
          .Activated                = false,
          .OneShootTask             = false,
          .OneShootTaskFinished     = false,
      },
  .PosRef     = 0,
  .PrevPosRef = 0,
  .pPosGen   = &PositionProfileGeneratorM1,
  .pSin                   = &sineGenerator,
  .Fs                     = POSITION_LOOP_FREQUENCY_HZ,
  .RefSinStartFreq01Hz = 2,
  .RefSinEndFreq01Hz   = 2,
  .RefSinAmp           = 65536L * 5,  /* 5 rounds */
  .SweepDuration_ms    = 5000,
  .flags      = {0}

};

EncoderAlignmentApp_Handle_t EncoderAlignmentApp = {
    ._Super =
        {
            .pMCI                     = &Mci[M1],
            .pFctInit                 = EncoderAlignmentApp_Init,
            .pFctReset                = EncoderAlignmentApp_Reset,
            .pFctOnStart              = NULL,
            .pFctOnExit               = NULL,
            .pFctPreLowFreqUpdate     = EncoderAlignmentApp_OnLowFrequencyUpdate,
            .pFctPostLowFreqUpdate    = NULL,
            .pFctPreMediumFreqUpdate  = NULL,
            .pFctPostMediumFreqUpdate = NULL,
            .pFctPreHighFreqUpdate    = NULL,
            .pFctPostHighFreqUpdate   = NULL,
            .pFctBackgroundUpdate     = EncoderAlignmentApp_OnBackground,
            .Activated                = false,
            .OneShootTask             = true,
            .OneShootTaskFinished     = false,
        },

    .pEncoder          = &ENCODER_M1,
    .pPosGen           = &PositionProfileGeneratorM1,

    .TimeStamp         = 0,
    .AlignmentDuration = 2000,
    .SpinningDuration  = 10000,
    .AlignedMecAngle   = 0,
    .Idref             = IQMAX * 80 / 100,

};

JogApp_Handle_t JogApp = {
    ._Super =
        {
            .pMCI                     = &Mci[M1],
            .pFctInit                 = NULL,
            .pFctReset                = JogApp_OnReset,
            .pFctOnStart              = NULL,
            .pFctOnExit               = NULL,
            .pFctPreLowFreqUpdate     = NULL,
            .pFctPostLowFreqUpdate    = NULL,
            .pFctPreMediumFreqUpdate  = NULL,
            .pFctPostMediumFreqUpdate = NULL,
            .pFctPreHighFreqUpdate    = NULL,
            .pFctPostHighFreqUpdate   = NULL,
            .pFctBackgroundUpdate     = JogApp_OnBackground,
            .Activated                = false,
            .OneShootTask             = false,
            .OneShootTaskFinished     = false,
        },

    .flags             = {0},
    .Acc               = 200,   /* 2s/Hz*/
    .Idref             = IQMAX * 25 / 100,
    .JogSpeed          = 1 * SPEED_UNIT,

};

CurrentloopBWTestApp_Handle_t CurrentLoopBWTest = {
    ._Super =
        {
            .pMCI                     = &Mci[M1],
            .pFctInit                 = NULL,
            .pFctReset                = NULL,
            .pFctOnStart              = CurrentloopBWTestApp_OnStart,
            .pFctOnExit               = NULL,
            .pFctPreLowFreqUpdate     = NULL,
            .pFctPostLowFreqUpdate    = NULL,
            .pFctPreMediumFreqUpdate  = CurrentloopBWTestApp_MediumFreqUpdate,
            .pFctPostMediumFreqUpdate = NULL,
            .pFctPreHighFreqUpdate    = NULL,
            .pFctPostHighFreqUpdate   = NULL,
            .pFctBackgroundUpdate     = NULL,
            .Activated                = false,
            .OneShootTask             = false,
            .OneShootTaskFinished     = false,
        },
    .IdRef_10BitRes = 300,
    .IdRef          = IQMAX * 30 / 100,
    .PulseWidth_ms  = 100,
    .PulseLevel     = false,
};

SpeedloopBWTestApp_Handle_t SpeedLoopBWTest = {
    ._Super =
        {
            .pMCI                     = &Mci[M1],
            .pFctInit               = NULL,
            .pFctReset              = NULL,
            .pFctOnStart            = SpeedloopBWTestApp_OnStart,
            .pFctOnExit               = NULL,
            .pFctPreLowFreqUpdate     = NULL,
            .pFctPostLowFreqUpdate    = NULL,
            .pFctPreMediumFreqUpdate  = SpeedloopBWTestApp_PreMediumFrequencyUpdate,
            .pFctPostMediumFreqUpdate = NULL,
            .pFctPreHighFreqUpdate    = NULL,
            .pFctPostHighFreqUpdate   = NULL,
            .pFctBackgroundUpdate     = NULL,
            .Activated              = false,
            .OneShootTask           = false,
            .OneShootTaskFinished   = false,
        },
    .pSin                   = &sineGenerator,
    .Fs                     = MEDIUM_FREQUENCY_TASK_RATE,
    .SpdRefSinStartFreq01Hz = 2,
    .SpdRefSinEndFreq01Hz   = 2,
    .SpdRefSinAmp_SpeedUnit = 1 * SPEED_UNIT,
    .SpdSweepDuration_ms    = 5000,
};

#ifdef ENABLE_MECH_ID_APP
MechParamIDApp_Handle_t MechParamIDApp = {
    ._Super =
        {
            .pFctInit                 = NULL,
            .pFctReset                = NULL,
            .pFctOnStart              = MechParamIDApp_Reset,
            .pFctOnExit               = NULL,
            .pFctPreLowFreqUpdate     = NULL,
            .pFctPostLowFreqUpdate    = NULL,
            .pFctPreMediumFreqUpdate  = MechParamIDApp_PreMediumFreqUpdate,
            .pFctPostMediumFreqUpdate = NULL,
            .pFctPreHighFreqUpdate    = NULL,
            .pFctPostHighFreqUpdate   = NULL,
            .pFctBackgroundUpdate     = MechParamIDApp_BackgroundUpdate,
            .Activated                = false,
            .OneShootTask             = false,
            .OneShootTaskFinished     = false,
        },
    .MechParamID  = {
        .pSin              = &sineGenerator,
        .Fs                = SPEED_LOOP_FREQUENCY_HZ,
        .Kc                = CURRENT_CONV_FACTOR_INV,
        .SpeedRefFrequency = 2.0,
        .SpeedRefSinAmp    = {10.0, 15.0}
    },
    .pMCI         = &Mci[M1],
};
#endif

#define DEFAULT_USER_APP_ID   USER_APP_NONE


UserApplication_Handle_t* const USER_TASKS_ARRAY[USER_APP_COUNT] = {
    &BasicApp._Super,
    &PositionCtrolApp._Super,
    &EncoderAlignmentApp._Super,
    &CurrentLoopBWTest._Super,
    &SpeedLoopBWTest._Super,
#ifdef ENABLE_MECH_ID_APP
    &MechParamIDApp._Super,
#endif
    &JogApp._Super,
};

UserApplication_Handle_t* pCurrentTask = &BasicApp._Super;

USER_APP_ID UserAppID          = USER_APP_NONE;
USER_APP_ID RequestedUserAppID = USER_APP_NORMAL_POS_CTRL;


/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup MCTasks
 * @{
 */
#define DISABLE_HF_TASK()  LL_TIM_DisableIT_UPDATE(TIM1)
#define ENABLE_HF_TASK()  do {\
    LL_TIM_ClearFlag_UPDATE(TIM1);\
    LL_TIM_EnableIT_UPDATE(TIM1);\
} while (0)

/**
 * @defgroup MCAppHooks Motor Control Applicative hooks
 * @brief User defined functions that are called in the Motor Control tasks.
 *
 *
 * @{
 */


/**
 * @brief Hook function called right before the end of the MCboot function.
 *
 *
 *
 */
void MC_APP_BootHook(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed at the end of MCboot().
   */

  /* USER CODE BEGIN BootHook */
  Scheduler_Init();

#ifdef ENABLE_RESONANCE_SUPPRESSION
  ResonanceSuppression_Init(&ResonanceSuppressor_M1, SPEED_LOOP_FREQUENCY_HZ, SPEED_LOOP_FREQUENCY_HZ, IQMAX / 20);
#endif

#if DATALOGGER_TYPE == DATALOGGER_JSCOPE
  JSInterface_Init();
#endif

  /* Update current pi gains as the rshunt and motor parameters may be simply changed
  but the gains are not updated accordingly */
  // MCI_UpdateCurrentKpGain(&Mci[M1]);
  // MCI_UpdateCurrentKiGain(&Mci[M1]);

  for (uint8_t id = USER_APP_NONE + 1; id < USER_APP_COUNT; id++) {
    UserApplication_Init(USER_TASKS_ARRAY[id]);
  }

  ParamManager_Init();

  /* 初始化角度计算模块 */
  if (OutAngleCalc_Init(&s_angleCalcHandle, &s_defaultAngleConfig) == 0) {
    s_angleCalcInitialized = 1;
  }

  /* USER CODE END BootHook */
}

/**
  * @brief Hook function called right after the Medium Frequency Task for Motor 1.
  *
  */
void MC_APP_PostHighFrequencyHook_M1(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed right after the Medium
   * Frequency Task of Motor 1
   */

  /* USER SECTION BEGIN PostMediumFrequencyHookM1 */
  UserApplication_PostHighFrequencyUpdate(pCurrentTask);

  /* 角度计算功能 */
  if (s_angleCalcInitialized) {
    /* 获取霍尔传感器电压值 */
    uint16_t LineHALLA = RCM_GetRegularConv(&LineHALL_A);
    uint16_t LineHALLB = RCM_GetRegularConv(&LineHALL_B);
    
    /* 获取机械角度 */
    int16_t hMecAngle = MC_GetCurrentPosition1();
    
    /* 计算输出角度 */
    if (OutAngleCalc_Compute(&s_angleCalcHandle, LineHALLA, LineHALLB, hMecAngle) == 0) {
      /* 角度计算成功，可以在这里使用计算结果 */
      /* 例如：可以将结果用于其他控制算法或数据记录 */
    }
  }

  /* USER SECTION END PostMediumFrequencyHookM1 */
}

/**
 * @brief Hook function called right before the Medium Frequency Task for Motor 1.
 *
 */
void MC_APP_PrevMediumFrequencyHook_M1(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed right after the Medium
   * Frequency Task of Motor 1
   */

  /* USER SECTION BEGIN PostMediumFrequencyHookM1 */
  UserApplication_PreMediumFrequencyUpdate(pCurrentTask);
  /* USER SECTION END PostMediumFrequencyHookM1 */
}

/**
 * @brief Hook function called right after the Medium Frequency Task for Motor 1.
 *
 *
 *
 */
void MC_APP_PostMediumFrequencyHook_M1(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed right after the Medium
   * Frequency Task of Motor 1
   */

  /* USER SECTION BEGIN PostMediumFrequencyHookM1 */
  Scheduler_MediumFrequencyUpdate();
  UserApplication_PostMediumFrequencyUpdate(pCurrentTask);

#if DATALOGGER_TYPE == DATALOGGER_STMSTUDIO
  DumpTrace();
#endif

#if DATALOGGER_TYPE == DATALOGGER_JSCOPE
  JSInterface_Sample();
#endif
  /* USER SECTION END PostMediumFrequencyHookM1 */
}

/**
 * @brief Hook function called right after the Medium Frequency Task for Motor 1.
 *
 *
 *
 */
void MC_APP_LowFrequencyHook_M1(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed right after the Medium
   * Frequency Task of Motor 1
   */

  /* USER SECTION BEGIN PostMediumFrequencyHookM1 */
#ifdef ENABLE_STALL_DETECTION
  if (MC_GetSTMStateMotor1() == RUN) {
    StallDetection_Update(&StallDetection_M1, MC_GetMecPositionMotor1(), SpeednTorqCtrlM1.TorqueRef >> 16);
  }
#endif

  /* 设置零点功能 */
  if (s_HomingControl.setZeroFlag == 1) {
    /* 将Encoder._super.wmecangle设为0 */
    ENCODER_M1._Super.wMecAngle = 0;
    
    /* 将PositionCtrolApp的PosRef设为0 */
    PositionCtrolApp.PosRef = 0;
    PositionCtrolApp.PrevPosRef = 0;
    PositionCtrolApp.SinRefOffset = 0;
    PositionProfileGenerator_PresetPosition(PositionCtrolApp.pPosGen, 0);
    PosCtrl_Reset(PositionCtrolApp._Super.pMCI->pPosCtrl);

    /* 清除设置零点标志位 */
    s_HomingControl.setZeroFlag = 0;
  }



  /* 回零功能 */
  if (s_HomingControl.homingModeFlag == 1) {
    /* 切换到位置控制模式 */
    RequestedUserAppID = USER_APP_NORMAL_POS_CTRL;
    
    /* 如果回零开始标志位为1，开始回零过程 */
    if (s_HomingControl.homingStartFlag == 1) {
      /* 使能PositionCtrolApp的flags */
      PositionCtrolApp.flags.bits.MotorOn = 1;

        s_HomingControl.homingCounter++;

      
      /* 等程序执行100次后，将PosRef置0（只执行一次） */
      if (s_HomingControl.homingCounter == 100 && s_HomingControl.posRefSetExecuted == 0) {
        PositionCtrolApp.PosRef = 0;
        /* 标记PosRef设置已执行，防止重复执行 */
        s_HomingControl.posRefSetExecuted = 1;
        
      }
    }
    else{
      PositionCtrolApp.flags.bits.MotorOn = 0;
      s_HomingControl.homingCounter =0;
      s_HomingControl.posRefSetExecuted = 0;

    }
  }


  UserApplication_PreLowFrequencyUpdate(pCurrentTask);
  /* USER SECTION END PostMediumFrequencyHookM1 */
}

/**
 * @brief perform preparation before run
 *
 */
void MC_APP_StartRunHook_M1(void)
{
#ifdef ENABLE_STALL_DETECTION
  StallDetection_Reset(&StallDetection_M1);
#endif
#ifdef ENABLE_RESONANCE_SUPPRESSION
  NotchFilter_Clear(&ResonanceSuppressor_M1.notchFilter);
#endif
#ifdef ENABLE_IQREF_LPF
  LowPassFilter_Clear(&TorqueRefLPF_M1);
#endif

  UserApplication_OnStart(pCurrentTask);
}
bool firstJudge =false;
/**
 * @brief background tasks
 *
 */
void MC_APP_BackgroundHook_M1(void)
{
#ifdef ENABLE_RESONANCE_SUPPRESSION
  ResonanceSuppression_StateUpdate(&ResonanceSuppressor_M1);
#endif
#ifdef ENABLE_IQREF_LPF
  LowPassFilter_ParamUpdate(&TorqueRefLPF_M1);
#endif
  UserApplication_BackgroundUpdate(pCurrentTask);

  /* Only do the task switch during inactive*/
  MCI_State_t state = MC_GetSTMStateMotor1();

  if (IDLE == state || FAULT_NOW == state || FAULT_OVER == state) {
    if (pCurrentTask->OneShootTask && pCurrentTask->OneShootTaskFinished) {
      RequestedUserAppID = DEFAULT_USER_APP_ID;
    }

    if (UserAppID != RequestedUserAppID) {
      pCurrentTask->Activated = false;
      UserApplication_OnExit(pCurrentTask);
      UserApplication_Handle_t* pNewTask = USER_TASKS_ARRAY[RequestedUserAppID];
      UserApplication_Reset(pNewTask);
      pCurrentTask            = pNewTask;
      pCurrentTask->Activated = true;
      UserAppID               = RequestedUserAppID;
    }
  }

  CAN_ProcessMessages();
  if ((EncoderAlignmentApp.pEncoder->iSCalibrationCompletedFlag != 0xA0A0) && (firstJudge == false))
  //if ((EncoderAlignmentApp.pEncoder->iSCalibrationCompletedFlag != 0x0A0A) && (firstJudge == false))
  {
    RequestedUserAppID = USER_APP_ENCODER_ALIGNMENT;
    EncoderAlignmentApp.flags.all = 3;
    firstJudge = true;
  }

  if (ParamManager_IsParamSavePending() && MC_GetSTMStateMotor1() == IDLE) {
    EncoderERR_SaveToFlash();
    ParamManager_SaveToNV();
  }

}

/**
 * @brief user handler to torque calculation, can implement low pass filter or notch filter here
 *
 * @param hTref torque ref get from speed regulator
 * @return new torque ref after processing
 */
int16_t MC_APP_CalcTorqueReferenceHook(int16_t hTref)
{
#ifdef ENABLE_RESONANCE_SUPPRESSION
  ResonanceSuppression_SampleData(&ResonanceSuppressor_M1, hTref);
  hTref = ResonanceSuppression_NotchFilterUpdate(&ResonanceSuppressor_M1, hTref);
#endif
#ifdef ENABLE_IQREF_LPF
  hTref = LowPassFilter_Update(&TorqueRefLPF_M1, hTref);
#endif
  return hTref;
}


/** @} */

/** @} */

/** @} */

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
