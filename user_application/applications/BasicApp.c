#include "BasicApp.h"



void BasicApp_OnReset(UserApplication_Handle_t* pSuper)
{
    BasicApp_Handle_t* pHandle = (BasicApp_Handle_t*)pSuper;
    pSuper->OneShootTaskFinished = false;   
    pHandle->Flags.all = 0;
}

void BasicApp_OnExit(UserApplication_Handle_t* pSuper)
{
    BasicApp_Handle_t* pHandle = (BasicApp_Handle_t*)pSuper;
    pHandle->Flags.all = 0;
    MCI_StopMotor(pSuper->pMCI);
}

void BasicApp_OnBackground(UserApplication_Handle_t* pSuper)
{
    BasicApp_Handle_t* pHandle = (BasicApp_Handle_t*)pSuper;

    switch (MCI_GetSTMState(pSuper->pMCI))
    {
    case IDLE:
        if (pHandle->Flags.bits.MotorOn) {
            switch (MCI_GetControlMode(pSuper->pMCI))
            {
            case MCM_OPEN_LOOP_CURRENT_MODE:
                // MCI_ExecTorqueRamp(pSuper->pMCI, pHandle->Iref, pHandle->Durationms);
                VSS_SetMecAcceleration(pSuper->pMCI->pVSS, 0, 0);
                pHandle->PrevSpeedRef = 0;
                break;

            case MCM_SPEED_MODE:
                MCI_ExecSpeedRamp(pSuper->pMCI, pHandle->SpeedRef, pHandle->Durationms);
                pHandle->PrevSpeedRef = pHandle->SpeedRef;
                break;

            case MCM_TORQUE_MODE:
                MCI_ExecTorqueRamp(pSuper->pMCI, pHandle->Iref, pHandle->Durationms);
            break;
            
            default:
                break;
            }
            MCI_StartMotor(pSuper->pMCI);
        }

        break;

    case RUN:
        if (!pHandle->Flags.bits.MotorOn) { 
            MCI_StopMotor(pSuper->pMCI);
        } else {
            switch (MCI_GetControlMode(pSuper->pMCI))
            {
            case MCM_OPEN_LOOP_CURRENT_MODE:
                // if (pHandle->Iref != pSuper->pMCI->hFinalTorque) {
                //     MCI_ExecTorqueRamp(pSuper->pMCI, pHandle->Iref, pHandle->Durationms);
                // }
                if (pHandle->SpeedRef != pHandle->PrevSpeedRef) {
                    VSS_SetMecAcceleration(pSuper->pMCI->pVSS, pHandle->SpeedRef, pHandle->Durationms);
                    pHandle->PrevSpeedRef = pHandle->SpeedRef;
                }
                break;

            case MCM_SPEED_MODE:
                if (pHandle->SpeedRef != pHandle->PrevSpeedRef) {
                    MCI_ExecSpeedRamp(pSuper->pMCI, pHandle->SpeedRef, pHandle->Durationms);
                    pHandle->PrevSpeedRef = pHandle->SpeedRef;
                }
                break;
            case MCM_TORQUE_MODE:
                if (pHandle->Iref != pSuper->pMCI->hFinalTorque) {
                    MCI_ExecTorqueRamp(pSuper->pMCI, pHandle->Iref, pHandle->Durationms);
                }
                break;

            default:
                break;
            }
        }   
        break;

    case FAULT_OVER:
        if (pHandle->Flags.bits.FaultReset) {
            pHandle->Flags.bits.FaultReset = 0;
            MCI_FaultAcknowledged(pSuper->pMCI);
        }
        break;
    
    default:
        break;
    }


}
