/************************************************************************
** Includes
*************************************************************************/
#include <string.h>

#include "cfe.h"

#include "mpc_app.h"
#include "mpc_msg.h"
#include "mpc_version.h"
#include "math/Matrix3F3.hpp"
#include "math/Quaternion.hpp"
#include "math/Euler.hpp"
#include "math/Dcm.hpp"
#include <float.h>
#include <math.h>
#include "px4lib.h"
#include "geo/geo.h"
#include "math/Expo.hpp"
#include "math/Limits.hpp"
#include "math/Functions.hpp"

#define nan FP_NAN

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Local definitions                                               */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define MPC_CONSTANTS_ONE_G    9.80665f   /* m/s^2		*/
#define SIGMA_SINGLE_OP        0.000001f
#define SIGMA_NORM             0.001f

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Instantiate the application object.                             */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
MPC oMPC;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Default constructor.                                            */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
MPC::MPC() :
		_filter_manual_pitch(50.0f, 10.0f),
		_filter_manual_roll(50.0f, 10.0f),
		ManualDirectionChangeHysteresis(false)
{

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Destructor constructor.                                         */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
MPC::~MPC()
{

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialize event tables.                                        */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 MPC::InitEvent()
{
    int32  iStatus=CFE_SUCCESS;

    /* Register the table with CFE */
    iStatus = CFE_EVS_Register(0, 0, CFE_EVS_BINARY_FILTER);
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_ES_WriteToSysLog("MPC - Failed to register with EVS (0x%08lX)\n", iStatus);
    }

    return iStatus;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialize Message Pipes                                        */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 MPC::InitPipe()
{
    int32  iStatus=CFE_SUCCESS;

    /* Init schedule pipe and subscribe to wakeup messages */
    iStatus = CFE_SB_CreatePipe(&SchPipeId,
    		MPC_SCH_PIPE_DEPTH,
			MPC_SCH_PIPE_NAME);
    if (iStatus == CFE_SUCCESS)
    {
        iStatus = CFE_SB_SubscribeEx(MPC_WAKEUP_MID, SchPipeId, CFE_SB_Default_Qos, MPC_WAKEUP_MID_MAX_MSG_COUNT);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MPC_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
            		"Sch Pipe failed to subscribe to MPC_WAKEUP_MID. (0x%08lX)",
                    iStatus);
            goto MPC_InitPipe_Exit_Tag;
        }

        iStatus = CFE_SB_SubscribeEx(MPC_SEND_HK_MID, SchPipeId, CFE_SB_Default_Qos, MPC_SEND_HK_MID_MAX_MSG_COUNT);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MPC_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
					 "CMD Pipe failed to subscribe to MPC_SEND_HK_MID. (0x%08X)",
					 (unsigned int)iStatus);
            goto MPC_InitPipe_Exit_Tag;
        }

        iStatus = CFE_SB_SubscribeEx(PX4_CONTROL_STATE_MID, SchPipeId, CFE_SB_Default_Qos, 1);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MPC_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
					 "CMD Pipe failed to subscribe to PX4_CONTROL_STATE_MID. (0x%08lX)",
					 iStatus);
            goto MPC_InitPipe_Exit_Tag;
        }

        iStatus = CFE_SB_SubscribeEx(PX4_MANUAL_CONTROL_SETPOINT_MID, SchPipeId, CFE_SB_Default_Qos, 1);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MPC_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
					 "CMD Pipe failed to subscribe to PX4_MANUAL_CONTROL_SETPOINT_MID. (0x%08lX)",
					 iStatus);
            goto MPC_InitPipe_Exit_Tag;
        }

        iStatus = CFE_SB_SubscribeEx(PX4_HOME_POSITION_MID, SchPipeId, CFE_SB_Default_Qos, 1);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MPC_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
					 "CMD Pipe failed to subscribe to PX4_HOME_POSITION_MID. (0x%08lX)",
					 iStatus);
            goto MPC_InitPipe_Exit_Tag;
        }

        iStatus = CFE_SB_SubscribeEx(PX4_VEHICLE_CONTROL_MODE_MID, SchPipeId, CFE_SB_Default_Qos, 1);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MPC_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
					 "CMD Pipe failed to subscribe to PX4_VEHICLE_CONTROL_MODE_MID. (0x%08lX)",
					 iStatus);
            goto MPC_InitPipe_Exit_Tag;
        }

        iStatus = CFE_SB_SubscribeEx(PX4_POSITION_SETPOINT_TRIPLET_MID, SchPipeId, CFE_SB_Default_Qos, 1);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MPC_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
					 "CMD Pipe failed to subscribe to PX4_POSITION_SETPOINT_TRIPLET_MID. (0x%08lX)",
					 iStatus);
            goto MPC_InitPipe_Exit_Tag;
        }

        iStatus = CFE_SB_SubscribeEx(PX4_VEHICLE_STATUS_MID, SchPipeId, CFE_SB_Default_Qos, 1);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MPC_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
					 "CMD Pipe failed to subscribe to PX4_VEHICLE_STATUS_MID. (0x%08lX)",
					 iStatus);
            goto MPC_InitPipe_Exit_Tag;
        }

        iStatus = CFE_SB_SubscribeEx(PX4_VEHICLE_LAND_DETECTED_MID, SchPipeId, CFE_SB_Default_Qos, 1);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MPC_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
					 "CMD Pipe failed to subscribe to PX4_VEHICLE_LAND_DETECTED_MID. (0x%08lX)",
					 iStatus);
            goto MPC_InitPipe_Exit_Tag;
        }

        iStatus = CFE_SB_SubscribeEx(PX4_VEHICLE_LOCAL_POSITION_MID, SchPipeId, CFE_SB_Default_Qos, 1);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MPC_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
					 "CMD Pipe failed to subscribe to PX4_VEHICLE_LOCAL_POSITION_MID. (0x%08lX)",
					 iStatus);
            goto MPC_InitPipe_Exit_Tag;
        }
    }
    else
    {
        (void) CFE_EVS_SendEvent(MPC_PIPE_INIT_ERR_EID, CFE_EVS_ERROR,
			 "Failed to create SCH pipe (0x%08lX)",
			 iStatus);
        goto MPC_InitPipe_Exit_Tag;
    }

    /* Init command pipe and subscribe to command messages */
    iStatus = CFE_SB_CreatePipe(&CmdPipeId,
    		MPC_CMD_PIPE_DEPTH,
			MPC_CMD_PIPE_NAME);
    if (iStatus == CFE_SUCCESS)
    {
        /* Subscribe to command messages */
        iStatus = CFE_SB_Subscribe(MPC_CMD_MID, CmdPipeId);

        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(MPC_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
				 "CMD Pipe failed to subscribe to MPC_CMD_MID. (0x%08lX)",
				 iStatus);
            goto MPC_InitPipe_Exit_Tag;
        }
    }
    else
    {
        (void) CFE_EVS_SendEvent(MPC_PIPE_INIT_ERR_EID, CFE_EVS_ERROR,
			 "Failed to create CMD pipe (0x%08lX)",
			 iStatus);
        goto MPC_InitPipe_Exit_Tag;
    }

MPC_InitPipe_Exit_Tag:
    return iStatus;
}
    

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialize Global Variables                                     */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void MPC::InitData()
{
	/* Init housekeeping message. */
	CFE_SB_InitMsg(&HkTlm,
		MPC_HK_TLM_MID, sizeof(HkTlm), TRUE);

	/* Init output messages */
	CFE_SB_InitMsg(&VehicleAttitudeSetpointMsg,
		PX4_VEHICLE_ATTITUDE_SETPOINT_MID, sizeof(PX4_VehicleAttitudeSetpointMsg_t), TRUE);

	//CFE_SB_InitMsg(&VehicleLocalVelocitySetpointMsg,
	//	PX4_VEHICLE_LOCAL_VELOCITY_SETPOINT_MID, sizeof(PX4_VehicleLocalVelocitySetpointMsg_t), TRUE);

	CFE_SB_InitMsg(&VehicleLocalPositionSetpointMsg,
		PX4_VEHICLE_LOCAL_POSITION_SETPOINT_MID, sizeof(PX4_VehicleLocalPositionSetpointMsg_t), TRUE);

	CFE_SB_InitMsg(&VehicleGlobalVelocitySetpointMsg,
		PX4_VEHICLE_GLOBAL_VELOCITY_SETPOINT_MID, sizeof(PX4_VehicleGlobalVelocitySetpointMsg_t), TRUE);

	Rotation.Identity();
	Yaw = 0.0f;
	YawTakeoff = 0.0f;
	InLanding = false;
	InTakeoff = false;
	LndReachedGround = false;
	VelZLp = 0;
	AccZLp = 0;
	VelMaxXY = 0.0f;
	HeadingResetCounter = 0;

	RefPos = {};

	memset(&ControlStateMsg, 0, sizeof(ControlStateMsg));
	memset(&ManualControlSetpointMsg, 0, sizeof(ManualControlSetpointMsg));
	memset(&HomePositionMsg, 0, sizeof(HomePositionMsg));
	memset(&VehicleControlModeMsg, 0, sizeof(VehicleControlModeMsg));
	memset(&PositionSetpointTripletMsg, 0, sizeof(PositionSetpointTripletMsg));
	memset(&VehicleStatusMsg, 0, sizeof(VehicleStatusMsg));
	memset(&VehicleLandDetectedMsg, 0, sizeof(VehicleLandDetectedMsg));
	memset(&VehicleLocalPositionMsg, 0, sizeof(VehicleLocalPositionMsg));
	memset(&VehicleLocalPositionSetpointMsg, 0, sizeof(VehicleLocalPositionSetpointMsg));

	Z_ResetCounter = 0;
	XY_ResetCounter = 0;
	VZ_ResetCounter = 0;
	VXY_ResetCounter = 0;
	HeadingResetCounter = 0;
	TakeoffVelLimit = 0.0f;

	RefTimestamp = 0;
	RefAlt = 0.0f;

	Position.Zero();
	PositionSetpoint.Zero();
	Velocity.Zero();
	VelocitySetpoint.Zero();
	VelocityPrevious.Zero();
	VelocityFF.Zero();
	VelocitySetpointPrevious.Zero();
	VelocityErrD.Zero();
	CurrentPositionSetpoint.Zero();
	ThrustInt.Zero();
	PosP.Zero();

	RSetpoint.Identity();

	ResetPositionSetpoint = true;
	ResetAltitudeSetpoint = true;
	DoResetAltPos = true;
	ModeAuto = false;
	PositionHoldEngaged = false;
	AltitudeHoldEngaged = false;
	RunPosControl = true;
	RunAltControl = true;

	ResetIntZ = true;
	ResetIntXY = true;
	ResetIntZManual = false;
	ResetYawSetpoint = true;

	HoldOffboardXY = false;
	HoldOffboardZ = false;
	LimitVelXY = false;

	GearStateInitialized = false;

	VehicleAttitudeSetpointMsg.Q_D[0] = 1.0f;

	WasArmed = false;
	WasLanded = true;

	// NEW

	RefAltIsGlobal = false;
	UserIntentionXY = BRAKE; // NOTE: This needs to be initialized to brake to work.
	UserIntentionZ = NONE;
	StickInputXyPrev.Zero();
	m_VelMaxXy = 0.0f;
	AccelerationStateDependentXY = 0.0f;
	AccelerationStateDependentZ = 0.0f;
	ManualJerkLimitXY = ConfigTblPtr->MPC_JERK_MAX;
	ManualJerkLimitZ = (ConfigTblPtr->MPC_JERK_MAX > ConfigTblPtr->MPC_JERK_MIN) ? ConfigTblPtr->MPC_JERK_MAX: 1000000.0f;
	_z_derivative = 0.0f;
	//ManualDirectionChangeHysteresis(false);

	/* set trigger time for manual direction change detection */
	ManualDirectionChangeHysteresis.set_hysteresis_time_from(false, DIRECTION_CHANGE_TRIGGER_TIME_US);
	TripletLatLonFinite = false;
	PreviousPositionSetpoint.Zero();
	_man_yaw_offset = 0.0f;

	UpdateParamsFromTable();
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* MPC initialization                                              */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 MPC::InitApp()
{
    int32  iStatus   = CFE_SUCCESS;
    int8   hasEvents = 0;

    iStatus = InitEvent();
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_ES_WriteToSysLog("MPC - Failed to init events (0x%08lX)\n", iStatus);
        goto MPC_InitApp_Exit_Tag;
    }
    else
    {
        hasEvents = 1;
    }

    iStatus = InitPipe();
    if (iStatus != CFE_SUCCESS)
    {
        goto MPC_InitApp_Exit_Tag;
    }

    iStatus = InitConfigTbl();
    if (iStatus != CFE_SUCCESS)
    {
        goto MPC_InitApp_Exit_Tag;
    }

    InitData();

MPC_InitApp_Exit_Tag:
    if (iStatus == CFE_SUCCESS)
    {
        (void) CFE_EVS_SendEvent(MPC_INIT_INF_EID, CFE_EVS_INFORMATION,
                                 "Initialized.  Version %d.%d.%d.%d",
								 MPC_MAJOR_VERSION,
								 MPC_MINOR_VERSION,
								 MPC_REVISION,
								 MPC_MISSION_REV);
    }
    else
    {
        if (hasEvents == 1)
        {
            (void) CFE_ES_WriteToSysLog("MPC - Application failed to initialize\n");
        }
    }

    return iStatus;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Receive and Process Messages                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int32 MPC::RcvSchPipeMsg(int32 iBlocking)
{
    int32           iStatus=CFE_SUCCESS;
    CFE_SB_Msg_t*   MsgPtr=NULL;
    CFE_SB_MsgId_t  MsgId;

    /* Stop Performance Log entry */
    CFE_ES_PerfLogExit(MPC_MAIN_TASK_PERF_ID);

    /* Wait for WakeUp messages from scheduler */
    iStatus = CFE_SB_RcvMsg(&MsgPtr, SchPipeId, iBlocking);

    /* Start Performance Log entry */
    CFE_ES_PerfLogEntry(MPC_MAIN_TASK_PERF_ID);

    if (iStatus == CFE_SUCCESS)
    {
        MsgId = CFE_SB_GetMsgId(MsgPtr);
        switch (MsgId)
        {
            case MPC_WAKEUP_MID:
            	Execute();
                break;

            case MPC_SEND_HK_MID:
                ProcessCmdPipe();
                ReportHousekeeping();
                break;

            case PX4_CONTROL_STATE_MID:
            	ProcessControlStateMsg();
                memcpy(&ControlStateMsg, MsgPtr, sizeof(ControlStateMsg));
                break;

            case PX4_MANUAL_CONTROL_SETPOINT_MID:
                memcpy(&ManualControlSetpointMsg, MsgPtr, sizeof(ManualControlSetpointMsg));
                break;

            case PX4_HOME_POSITION_MID:
                memcpy(&HomePositionMsg, MsgPtr, sizeof(HomePositionMsg));
                break;

            case PX4_VEHICLE_CONTROL_MODE_MID:
                memcpy(&VehicleControlModeMsg, MsgPtr, sizeof(VehicleControlModeMsg));
                break;

            case PX4_POSITION_SETPOINT_TRIPLET_MID:
                memcpy(&PositionSetpointTripletMsg, MsgPtr, sizeof(PositionSetpointTripletMsg));
                ProcessPositionSetpointTripletMsg();
                break;

            case PX4_VEHICLE_STATUS_MID:
                memcpy(&VehicleStatusMsg, MsgPtr, sizeof(VehicleStatusMsg));

                break;

            case PX4_VEHICLE_LAND_DETECTED_MID:
                memcpy(&VehicleLandDetectedMsg, MsgPtr, sizeof(VehicleLandDetectedMsg));
                break;

            case PX4_VEHICLE_LOCAL_POSITION_MID:
                memcpy(&VehicleLocalPositionMsg, MsgPtr, sizeof(VehicleLocalPositionMsg));
                ProcessVehicleLocalPositionMsg();
                break;

            default:
                (void) CFE_EVS_SendEvent(MPC_MSGID_ERR_EID, CFE_EVS_ERROR,
                     "Recvd invalid SCH msgId (0x%04X)", MsgId);
        }
    }
    else if (iStatus == CFE_SB_NO_MESSAGE)
    {
        /* If there's no incoming message, you can do something here, or
         * nothing.  Note, this section is dead code only if the iBlocking arg
         * is CFE_SB_PEND_FOREVER. */
        iStatus = CFE_SUCCESS;
    }
    else if (iStatus == CFE_SB_TIME_OUT)
    {
        /* If there's no incoming message within a specified time (via the
         * iBlocking arg, you can do something here, or nothing.  
         * Note, this section is dead code only if the iBlocking arg
         * is CFE_SB_PEND_FOREVER. */
        iStatus = CFE_SUCCESS;
    }
    else
    {
        (void) CFE_EVS_SendEvent(MPC_RCVMSG_ERR_EID, CFE_EVS_ERROR,
			  "SCH pipe read error (0x%08lX).", iStatus);
    }

    return iStatus;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Process Incoming Commands                                       */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void MPC::ProcessCmdPipe()
{
    int32 iStatus = CFE_SUCCESS;
    CFE_SB_Msg_t*   CmdMsgPtr=NULL;
    CFE_SB_MsgId_t  CmdMsgId;

    /* Process command messages until the pipe is empty */
    while (1)
    {
        iStatus = CFE_SB_RcvMsg(&CmdMsgPtr, CmdPipeId, CFE_SB_POLL);
        if(iStatus == CFE_SUCCESS)
        {
            CmdMsgId = CFE_SB_GetMsgId(CmdMsgPtr);
            switch (CmdMsgId)
            {
                case MPC_CMD_MID:
                    ProcessAppCmds(CmdMsgPtr);
                    break;

                default:
                    /* Bump the command error counter for an unknown command.
                     * (This should only occur if it was subscribed to with this
                     *  pipe, but not handled in this switch-case.) */
                    HkTlm.usCmdErrCnt++;
                    (void) CFE_EVS_SendEvent(MPC_MSGID_ERR_EID, CFE_EVS_ERROR,
                                      "Recvd invalid CMD msgId (0x%04X)", (unsigned short)CmdMsgId);
                    break;
            }
        }
        else if (iStatus == CFE_SB_NO_MESSAGE)
        {
            break;
        }
        else
        {
            (void) CFE_EVS_SendEvent(MPC_RCVMSG_ERR_EID, CFE_EVS_ERROR,
                  "CMD pipe read error (0x%08lX)", iStatus);
            break;
        }
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Process MPC Commands                                            */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void MPC::ProcessAppCmds(CFE_SB_Msg_t* MsgPtr)
{
    uint32  uiCmdCode=0;

    if (MsgPtr != NULL)
    {
        uiCmdCode = CFE_SB_GetCmdCode(MsgPtr);
        switch (uiCmdCode)
        {
            case MPC_NOOP_CC:
                HkTlm.usCmdCnt++;
                (void) CFE_EVS_SendEvent(MPC_CMD_NOOP_EID, CFE_EVS_INFORMATION,
					"Recvd NOOP. Version %d.%d.%d.%d",
					MPC_MAJOR_VERSION,
					MPC_MINOR_VERSION,
					MPC_REVISION,
					MPC_MISSION_REV);
                break;

            case MPC_RESET_CC:
                HkTlm.usCmdCnt = 0;
                HkTlm.usCmdErrCnt = 0;
                break;

            case MPC_SET_XY_PID_CC:
            	UpdateXyPids((MPC_SetPidCmd_t *) MsgPtr);
				HkTlm.usCmdCnt++;
				(void) CFE_EVS_SendEvent(MPC_PID_UPDATE_EID, CFE_EVS_INFORMATION,
						"Updating XY PID values. Gain: %f, P: %f, I: %f, D: %f",
						PosP[0],
						VelP[0],
						VelI[0],
						VelD[0]);
				break;

            case MPC_SET_Z_PID_CC:
            	UpdateZPids((MPC_SetPidCmd_t *) MsgPtr);
				HkTlm.usCmdCnt++;
				(void) CFE_EVS_SendEvent(MPC_PID_UPDATE_EID, CFE_EVS_INFORMATION,
						"Updating Z PID values. Gain: %f, P: %f, I: %f, D: %f",
						PosP[2],
						VelP[2],
						VelI[2],
						VelD[2]);
				break;

            default:
                HkTlm.usCmdErrCnt++;
                (void) CFE_EVS_SendEvent(MPC_CC_ERR_EID, CFE_EVS_ERROR,
                                  "Recvd invalid command code (%u)", (unsigned int)uiCmdCode);
                break;
        }
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Send MPC Housekeeping                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void MPC::ReportHousekeeping()
{
    CFE_SB_TimeStampMsg((CFE_SB_Msg_t*)&HkTlm);
    CFE_SB_SendMsg((CFE_SB_Msg_t*)&HkTlm);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Publish Output Data                                             */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void MPC::SendVehicleAttitudeSetpointMsg()
{
    CFE_SB_TimeStampMsg((CFE_SB_Msg_t*)&VehicleAttitudeSetpointMsg);
    CFE_SB_SendMsg((CFE_SB_Msg_t*)&VehicleAttitudeSetpointMsg);
}

//void MPC::SendVehicleLocalVelocitySetpointMsg()
//{
//    CFE_SB_TimeStampMsg((CFE_SB_Msg_t*)&VehicleLocalVelocitySetpointMsg);
//    CFE_SB_SendMsg((CFE_SB_Msg_t*)&VehicleLocalVelocitySetpointMsg);
//}

void MPC::SendVehicleLocalPositionSetpointMsg()
{
    CFE_SB_TimeStampMsg((CFE_SB_Msg_t*)&VehicleLocalPositionSetpointMsg);
    CFE_SB_SendMsg((CFE_SB_Msg_t*)&VehicleLocalPositionSetpointMsg);
}

void MPC::SendVehicleGlobalVelocitySetpointMsg()
{
    CFE_SB_TimeStampMsg((CFE_SB_Msg_t*)&VehicleGlobalVelocitySetpointMsg);
    CFE_SB_SendMsg((CFE_SB_Msg_t*)&VehicleGlobalVelocitySetpointMsg);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Verify Command Length                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
boolean MPC::VerifyCmdLength(CFE_SB_Msg_t* MsgPtr,
                           uint16 usExpectedLen)
{
    boolean bResult  = TRUE;
    uint16  usMsgLen = 0;

    if (MsgPtr != NULL)
    {
        usMsgLen = CFE_SB_GetTotalMsgLength(MsgPtr);

        if (usExpectedLen != usMsgLen)
        {
            bResult = FALSE;
            CFE_SB_MsgId_t MsgId = CFE_SB_GetMsgId(MsgPtr);
            uint16 usCmdCode = CFE_SB_GetCmdCode(MsgPtr);

            (void) CFE_EVS_SendEvent(MPC_MSGLEN_ERR_EID, CFE_EVS_ERROR,
                              "Rcvd invalid msgLen: msgId=0x%08X, cmdCode=%d, "
                              "msgLen=%d, expectedLen=%d",
                              MsgId, usCmdCode, usMsgLen, usExpectedLen);
            HkTlm.usCmdErrCnt++;
        }
    }

    return bResult;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* MPC Application C style main entry point.                       */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern "C" void MPC_AppMain()
{
    oMPC.AppMain();
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* MPC Application C++ style main entry point.                     */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void MPC::AppMain()
{
    /* Register the application with Executive Services */
    uiRunStatus = CFE_ES_APP_RUN;

    int32 iStatus = CFE_ES_RegisterApp();
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_ES_WriteToSysLog("MPC - Failed to register the app (0x%08lX)\n", iStatus);
    }

    /* Start Performance Log entry */
    CFE_ES_PerfLogEntry(MPC_MAIN_TASK_PERF_ID);

    /* Perform application initializations */
    if (iStatus == CFE_SUCCESS)
    {
        iStatus = InitApp();
    }

    if (iStatus == CFE_SUCCESS)
    {
        /* Do not perform performance monitoring on startup sync */
        CFE_ES_PerfLogExit(MPC_MAIN_TASK_PERF_ID);
        CFE_ES_WaitForStartupSync(MPC_STARTUP_TIMEOUT_MSEC);
        CFE_ES_PerfLogEntry(MPC_MAIN_TASK_PERF_ID);
    }
    else
    {
        uiRunStatus = CFE_ES_APP_ERROR;
    }

    /* Application main loop */
    while (CFE_ES_RunLoop(&uiRunStatus) == TRUE)
    {
        RcvSchPipeMsg(MPC_SCH_PIPE_PEND_TIME);

        iStatus = AcquireConfigPointers();
        if(iStatus != CFE_SUCCESS)
        {
            /* We apparently tried to load a new table but failed.  Terminate the application. */
            uiRunStatus = CFE_ES_APP_ERROR;
        }
    }

    /* Stop Performance Log entry */
    CFE_ES_PerfLogExit(MPC_MAIN_TASK_PERF_ID);

    /* Exit the application */
    CFE_ES_ExitApp(uiRunStatus);
}

void MPC::ProcessVehicleLocalPositionMsg(void)// Updated
{
	/* Check if a reset event has happened if the vehicle is in manual mode
	 * we will shift the setpoints of the states which were reset. In auto
	 * mode we do not shift the setpoints since we want the vehicle to track
	 * the original state.
	 */
	// TODO: PE doesn't increment the reset counters. This may be dead code.
	if (VehicleControlModeMsg.ControlManualEnabled)
	{
		if (Z_ResetCounter != VehicleLocalPositionMsg.Z_ResetCounter)
		{
			PositionSetpoint[2] = VehicleLocalPositionMsg.Z;
		}

		if (XY_ResetCounter != VehicleLocalPositionMsg.XY_ResetCounter)
		{
			PositionSetpoint[0] = VehicleLocalPositionMsg.X;
			PositionSetpoint[1] = VehicleLocalPositionMsg.Y;
		}
	}

	/* Update the reset counters in any case. */
	Z_ResetCounter = VehicleLocalPositionMsg.Z_ResetCounter;
	XY_ResetCounter = VehicleLocalPositionMsg.XY_ResetCounter;
}

void MPC::ProcessControlStateMsg(void)
{
	math::Quaternion q_att(
			ControlStateMsg.Q[0],
			ControlStateMsg.Q[1],
			ControlStateMsg.Q[2],
			ControlStateMsg.Q[3]);

	Rotation = q_att.RotationMatrix();
	math::Vector3F euler_angles;

	euler_angles = Rotation.ToEuler();

	Yaw = euler_angles[2];

	if(VehicleControlModeMsg.ControlManualEnabled)
	{
		if (HeadingResetCounter != ControlStateMsg.QuatResetCounter)
		{
			HeadingResetCounter = ControlStateMsg.QuatResetCounter;
			math::Quaternion delta_q(ControlStateMsg.DeltaQReset[0],
					ControlStateMsg.DeltaQReset[1],
					ControlStateMsg.DeltaQReset[2],
					ControlStateMsg.DeltaQReset[3]);

			// we only extract the heading change from the delta quaternion
			math::Vector3F delta_euler = delta_q.ToEuler();
			VehicleAttitudeSetpointMsg.YawBody += delta_euler[2];
		}
	}
}

void MPC::ProcessPositionSetpointTripletMsg(void)// Updated
{
	/* Set current position setpoint invalid if alt is infinite. */
	if (!isfinite(PositionSetpointTripletMsg.Current.Alt))
	{
		PositionSetpointTripletMsg.Current.Valid = false;
	}

	/* To be a valid previous triplet, lat/lon/alt has to be finite */
	if (!isfinite(PositionSetpointTripletMsg.Previous.Lat) &&
	    !isfinite(PositionSetpointTripletMsg.Previous.Lon) &&
	    !isfinite(PositionSetpointTripletMsg.Previous.Alt))
	{
		PositionSetpointTripletMsg.Previous.Valid = false;
	}
}


void MPC::Execute(void) // NEW DONE
{
	static uint64 t_prev = 0;

	uint64 t = PX4LIB_GetPX4TimeUs();
	float dt = t_prev != 0 ? (t - t_prev) / 1e6f : 0.004f;
	t_prev = t;

	/* Set default max velocity in xy to vel_max */
	VelMaxXY = ConfigTblPtr->XY_VEL_MAX;

	/* Reset flags when landed */
	if (VehicleLandDetectedMsg.Landed)
	{
		ResetPositionSetpoint = true;
		ResetAltitudeSetpoint = true;
		DoResetAltPos = true;
		ModeAuto = false;
		PositionHoldEngaged = false;
		AltitudeHoldEngaged = false;
		RunPosControl = true;
		RunAltControl = true;
		ResetIntZ = true;
		ResetIntXY = true;
		ResetYawSetpoint = true;
		HoldOffboardXY = false;
		HoldOffboardZ = false;
		InLanding = false;
		LndReachedGround = false;

		/* Also reset previous setpoints */
		YawTakeoff = Yaw;
		VelocitySetpointPrevious.Zero();
		VelocityPrevious.Zero();

		/* Make sure attitude setpoint output "disables" attitude control
		 * TODO: we need a defined setpoint to do this properly especially when adjusting the mixer */
		VehicleAttitudeSetpointMsg.Thrust = 0.0f;
		VehicleAttitudeSetpointMsg.Timestamp = PX4LIB_GetPX4TimeUs();
	}

	if (!InTakeoff && VehicleLandDetectedMsg.Landed && VehicleControlModeMsg.Armed &&
		(InAutoTakeoff() || ManualWantsTakeoff()))
	{
		InTakeoff = true;
		/* This ramp starts negative and goes to positive later because we want to
		*  be as smooth as possible. If we start at 0, we alrady jump to hover throttle. */
		TakeoffVelLimit = -0.5f;
	}

	else if (!VehicleControlModeMsg.Armed) {
		/* If we're disarmed and for some reason were in a smooth takeoff, we reset that. */
		InTakeoff = false;
	}

	/* Set triplets to invalid if we just landed */
	if (VehicleLandDetectedMsg.Landed && !WasLanded)
	{
		PositionSetpointTripletMsg.Current.Valid = false;
	}

	WasLanded = VehicleLandDetectedMsg.Landed;
	UpdateRef();
	UpdateVelocityDerivative(dt);

	/* Reset the horizontal and vertical position hold flags for non-manual modes
	 * or if position / altitude is not controlled. */
	if (!VehicleControlModeMsg.ControlPositionEnabled || !VehicleControlModeMsg.ControlManualEnabled)
	{
		PositionHoldEngaged = false;
	}

	if (!VehicleControlModeMsg.ControlAltitudeEnabled || !VehicleControlModeMsg.ControlManualEnabled)
	{
		AltitudeHoldEngaged = false;
	}

	if (VehicleControlModeMsg.ControlAltitudeEnabled ||
		VehicleControlModeMsg.ControlPositionEnabled ||
		VehicleControlModeMsg.ControlClimbRateEnabled ||
		VehicleControlModeMsg.ControlVelocityEnabled ||
		VehicleControlModeMsg.ControlAccelerationEnabled)
	{
		DoControl(dt);

		/* Fill local position, velocity and thrust setpoint */
		VehicleLocalPositionSetpointMsg.Timestamp = PX4LIB_GetPX4TimeUs();
		VehicleLocalPositionSetpointMsg.X = PositionSetpoint[0];
		VehicleLocalPositionSetpointMsg.Y = PositionSetpoint[1];
		VehicleLocalPositionSetpointMsg.Z = PositionSetpoint[2];
		VehicleLocalPositionSetpointMsg.Yaw = VehicleAttitudeSetpointMsg.YawBody;
		VehicleLocalPositionSetpointMsg.VX = VelocitySetpoint[0];
		VehicleLocalPositionSetpointMsg.VY = VelocitySetpoint[1];
		VehicleLocalPositionSetpointMsg.VZ = VelocitySetpoint[2];

		/* Publish local position setpoint */
		SendVehicleLocalPositionSetpointMsg();
	}
	else
	{
		/* Position controller disabled, reset setpoints */
		ResetPositionSetpoint = true;
		ResetAltitudeSetpoint = true;
		DoResetAltPos = true;
		ModeAuto = false;
		ResetIntZ = true;
		ResetIntXY = true;

		/* Store last velocity in case a mode switch to position control occurs */
		VelocitySetpointPrevious = Velocity;
	}

	/* Generate attitude setpoint from manual controls */
	if (VehicleControlModeMsg.ControlManualEnabled && VehicleControlModeMsg.ControlAttitudeEnabled)
	{
		GenerateAttitudeSetpoint(dt);
	}
	else
	{
		ResetYawSetpoint = true;
		VehicleAttitudeSetpointMsg.YawSpMoveRate = 0.0f;
	}

	/* Update previous velocity for velocity controller D part */
	VelocityPrevious = Velocity;

	/* publish attitude setpoint
	 * Do not publish if
	 * - offboard is enabled but position/velocity/accel control is disabled,
	 * in this case the attitude setpoint is published by the mavlink app.
	 * - if the vehicle is a VTOL and it's just doing a transition (the VTOL attitude control module will generate
	 * attitude setpoints for the transition).
	 * - if not armed
	 */
	if (VehicleControlModeMsg.Armed &&
		(!(VehicleControlModeMsg.ControlOffboardEnabled &&
	      !(VehicleControlModeMsg.ControlPositionEnabled ||
		    VehicleControlModeMsg.ControlVelocityEnabled ||
			VehicleControlModeMsg.ControlAccelerationEnabled))))
	{
		SendVehicleAttitudeSetpointMsg();
	}

//	OS_printf("Position = [%f, %f, %f]\n", Position[0], Position[1], Position[2]);
//	OS_printf("PositionSetpoint = [%f, %f, %f]\n", PositionSetpoint[0], PositionSetpoint[1], PositionSetpoint[2]);
//	OS_printf("CurrentPositionSetpoint = [%f, %f, %f]\n", CurrentPositionSetpoint[0], CurrentPositionSetpoint[1], CurrentPositionSetpoint[2]);
//	OS_printf("Velocity = [%f, %f, %f]\n", Velocity[0], Velocity[1], Velocity[2]);
//	OS_printf("VelocitySetpoint = [%f, %f, %f]\n", VelocitySetpoint[0], VelocitySetpoint[1], VelocitySetpoint[2]);
//	OS_printf("AccelerationStateDependentXY = %f\n", AccelerationStateDependentXY);
//	OS_printf("AccelerationStateDependentZ = %f\n", AccelerationStateDependentZ);


}

void MPC::UpdateRef(void) // NEW DONE
{
	/* The reference point is only allowed to change when the vehicle is in standby state which is the
	normal state when the estimator origin is set. Changing reference point in flight causes large controller
	setpoint changes. Changing reference point in other arming states is untested and shoud not be performed. */
	if ((VehicleLocalPositionMsg.RefTimestamp != RefTimestamp)
	    && ((VehicleStatusMsg.ArmingState == PX4_ARMING_STATE_STANDBY)
		|| (!RefAltIsGlobal && VehicleLocalPositionMsg.Z_Global)))
	{
		double LatitudeSetpoint = 0.0f;
		double LongitudeSetpoint = 0.0f;
		float AltitudeSetpoint = 0.0f;
		uint64 CurrentTime = 0;

		if(RefTimestamp != 0)
		{
			/* Calculate current position setpoint in global frame. */
			map_projection_reproject(&RefPos, PositionSetpoint[0], PositionSetpoint[1], &LatitudeSetpoint, &LongitudeSetpoint);

			/* The altitude setpoint is the reference altitude (Z up) plus the (Z down)
			 * NED setpoint, multiplied out to minus*/
			AltitudeSetpoint = RefAlt - PositionSetpoint[2];
		}

		/* Update local projection reference including altitude. */
		CurrentTime = PX4LIB_GetPX4TimeUs();
		map_projection_init(&RefPos, VehicleLocalPositionMsg.RefLat, VehicleLocalPositionMsg.RefLon, CurrentTime);
		RefAlt = VehicleLocalPositionMsg.RefAlt;

		if(VehicleLocalPositionMsg.Z_Global)
		{
			RefAltIsGlobal = true;
		}

		if (RefTimestamp != 0)
		{
			/* Reproject position setpoint to new reference this effectively
			 * adjusts the position setpoint to keep the vehicle in its
			 * current local position. It would only change its global
			 * position on the next setpoint update. */
			map_projection_project(&RefPos, LatitudeSetpoint, LongitudeSetpoint, &PositionSetpoint[0], &PositionSetpoint[1]);
			PositionSetpoint[2] = -(AltitudeSetpoint - RefAlt);
		}

		RefTimestamp = VehicleLocalPositionMsg.RefTimestamp;
	}
}



void MPC::UpdateVelocityDerivative(float dt) // NEW DONE
{
	/* Update velocity derivative independent of the current flight mode */
	if (VehicleLocalPositionMsg.Timestamp == 0)
	{
		return;
	}

	/* TODO: this logic should be in the estimator, not the controller! */
	if (isfinite(VehicleLocalPositionMsg.X) &&
		isfinite(VehicleLocalPositionMsg.Y) &&
		isfinite(VehicleLocalPositionMsg.Z))
	{
		Position[0] = VehicleLocalPositionMsg.X;
		Position[1] = VehicleLocalPositionMsg.Y;

		if (ConfigTblPtr->ALT_MODE == 1 && VehicleLocalPositionMsg.DistBottomValid)
		{
			Position[2] = -VehicleLocalPositionMsg.DistBottom;
		}
		else
		{
			Position[2] = VehicleLocalPositionMsg.Z;
		}
	}

	if (isfinite(VehicleLocalPositionMsg.VX) &&
		isfinite(VehicleLocalPositionMsg.VY) &&
		isfinite(VehicleLocalPositionMsg.VZ))
	{
		Velocity[0] = VehicleLocalPositionMsg.VX;
		Velocity[1] = VehicleLocalPositionMsg.VY;

		if (ConfigTblPtr->ALT_MODE == 1 && VehicleLocalPositionMsg.DistBottomValid)
		{
			Velocity[2] = -VehicleLocalPositionMsg.DistBottomRate;
		}
		else
		{
			Velocity[2] = VehicleLocalPositionMsg.VZ;
		}

		if (!RunAltControl)
		{
			/* Set velocity to the derivative of position
			 * because it has less bias but blend it in across the landing speed range*/
			float weighting = fminf(fabsf(VelocitySetpoint[2]) / ConfigTblPtr->LAND_SPEED, 1.0f);
			Velocity[2] = _z_derivative * weighting + Velocity[2] * (1.0f - weighting);

		}
	}

	if (isfinite(VehicleLocalPositionMsg.VZ)) {
		_z_derivative = VehicleLocalPositionMsg.VZ;
	};

	VelocityErrD[0] = VelXDeriv.Update(-Velocity[0], dt, ConfigTblPtr->VELD_LP);
	VelocityErrD[1] = VelYDeriv.Update(-Velocity[1], dt, ConfigTblPtr->VELD_LP);
	VelocityErrD[2] = VelZDeriv.Update(-Velocity[2], dt, ConfigTblPtr->VELD_LP);
}



void MPC::DoControl(float dt) // NEW DONE
{
	/* By default, run position/altitude controller. the control_* functions
	 * can disable this and run velocity controllers directly in this cycle */
	RunPosControl = true;
	RunAltControl = true;
    
	if (VehicleControlModeMsg.ControlManualEnabled)
	{
		/* Manual control */
		ControlManual(dt);
		ModeAuto = false;

		/* We set tiplets to false.  This ensures that when switching to auto,
		 * the position controller will not use the old triplets but waits
		 * until triplets have been updated. */
		PositionSetpointTripletMsg.Current.Valid = false;
		PositionSetpointTripletMsg.Previous.Valid = false;
		CurrentPositionSetpoint = math::Vector3F(NAN, NAN, NAN);

		HoldOffboardXY = false;
		HoldOffboardZ = false;
	}
	else
	{
		/* reset acceleration to default */
		AccelerationStateDependentXY = ConfigTblPtr->ACC_HOR_MAX;
		AccelerationStateDependentZ = ConfigTblPtr->ACC_UP_MAX;
		ControlNonManual(dt);
	}
}



void MPC::GenerateAttitudeSetpoint(float dt) // NEW DONE
{
	/* Yaw setpoint is integrated over time, but we don't want to integrate the offset's */
	VehicleAttitudeSetpointMsg.YawBody -= _man_yaw_offset;
	_man_yaw_offset = 0.0f;

	/* Reset yaw setpoint to current position if needed. */
	if (ResetYawSetpoint)
	{
		ResetYawSetpoint = false;
		VehicleAttitudeSetpointMsg.YawBody = Yaw;
	}
	else if (!VehicleLandDetectedMsg.Landed &&
		   !(!VehicleControlModeMsg.ControlAltitudeEnabled && ManualControlSetpointMsg.Z < 0.1f))
	{
		/* Do not move yaw while sitting on the ground. */

		/* We want to know the real constraint, and global overrides manual. */
		const float YawRateMax = (math::radians(ConfigTblPtr->MAN_Y_MAX) < math::radians(ConfigTblPtr->MC_YAWRATE_MAX)) ? math::radians(ConfigTblPtr->MAN_Y_MAX) :
				math::radians(ConfigTblPtr->MC_YAWRATE_MAX);
		const float YawOffsetMax = YawRateMax / math::radians(ConfigTblPtr->MC_YAW_P);

		VehicleAttitudeSetpointMsg.YawSpMoveRate = ManualControlSetpointMsg.R * YawRateMax;
		float YawTarget = _wrap_pi(VehicleAttitudeSetpointMsg.YawBody + VehicleAttitudeSetpointMsg.YawSpMoveRate * dt);
		float YawOffs = _wrap_pi(YawTarget - Yaw);

		/* If the yaw offset became too big for the system to track stop
         * shifting it, only allow if it would make the offset smaller again. */
		if (fabsf(YawOffs) < YawOffsetMax ||
		    (VehicleAttitudeSetpointMsg.YawSpMoveRate > 0 && YawOffs < 0) ||
		    (VehicleAttitudeSetpointMsg.YawSpMoveRate < 0 && YawOffs > 0))
		{
			VehicleAttitudeSetpointMsg.YawBody = YawTarget;
		}
	}

	/* Control throttle directly if no climb rate controller is active */
	if (!VehicleControlModeMsg.ControlClimbRateEnabled)
	{
		float ThrVal = ThrottleCurve(ManualControlSetpointMsg.Z, ConfigTblPtr->THR_HOVER);
	    VehicleAttitudeSetpointMsg.Thrust = fmin(ThrVal, ConfigTblPtr->MANTHR_MAX);

		/* Enforce minimum throttle if not landed */
		if (!VehicleLandDetectedMsg.Landed)
		{
			VehicleAttitudeSetpointMsg.Thrust = fmax(VehicleAttitudeSetpointMsg.Thrust, ConfigTblPtr->MANTHR_MIN);
		}
	}

	/* Control roll and pitch directly if no aiding velocity controller is active. */
	if (!VehicleControlModeMsg.ControlVelocityEnabled)
	{
        VehicleAttitudeSetpointMsg.RollBody = ManualControlSetpointMsg.Y * math::radians(ConfigTblPtr->MAN_TILT_MAX);
		VehicleAttitudeSetpointMsg.PitchBody = -ManualControlSetpointMsg.X * math::radians(ConfigTblPtr->MAN_TILT_MAX);

		/* Only if optimal recovery is not used, modify roll/pitch. */
		if (ConfigTblPtr->VT_OPT_RECOV_EN <= 0)
		{
			/* Construct attitude setpoint rotation matrix. modify the setpoints for roll
			 * and pitch such that they reflect the user's intention even if a yaw error
			 * (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
			 * from the pure euler angle setpoints will lead to unexpected attitude behaviour from
			 * the user's view as the euler angle sequence uses the  yaw setpoint and not the current
			 * heading of the vehicle.
			 */

			/* Calculate our current yaw error. */
			float yaw_error = _wrap_pi(VehicleAttitudeSetpointMsg.YawBody - Yaw);

			// Compute the vector obtained by rotating a z unit vector by the rotation
			// given by the roll and pitch commands of the user
			math::Vector3F zB = {0, 0, 1};
			math::Matrix3F3 R_sp_roll_pitch = math::Matrix3F3::FromEuler(VehicleAttitudeSetpointMsg.RollBody, VehicleAttitudeSetpointMsg.PitchBody, 0);
			math::Vector3F z_roll_pitch_sp = R_sp_roll_pitch * zB;

			/* Transform the vector into a new frame which is rotated around the z axis
			 * by the current yaw error. this vector defines the desired tilt when we look
			 * into the direction of the desired heading.
			 */
			math::Matrix3F3 R_yaw_correction = math::Matrix3F3::FromEuler(0.0f, 0.0f, -yaw_error);
			z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

			/* Use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
			 * R_tilt is computed from_euler; only true if cos(roll) not equal zero
			 * -> valid if roll is not +-pi/2;
			 */
			VehicleAttitudeSetpointMsg.RollBody = -asinf(z_roll_pitch_sp[1]);
			VehicleAttitudeSetpointMsg.PitchBody = atan2f(z_roll_pitch_sp[0], z_roll_pitch_sp[2]);
		}

		/* Copy quaternion setpoint to attitude setpoint topic. */
		math::Quaternion q_sp(VehicleAttitudeSetpointMsg.RollBody, VehicleAttitudeSetpointMsg.PitchBody, VehicleAttitudeSetpointMsg.YawBody);
		q_sp.copyTo(VehicleAttitudeSetpointMsg.Q_D);
		VehicleAttitudeSetpointMsg.Q_D_Valid = true;
	}

	VehicleAttitudeSetpointMsg.Timestamp = PX4LIB_GetPX4TimeUs();
}



void MPC::ControlManual(float dt) // NEW DONE
{
	/* Entering manual control from non-manual control mode, reset alt/pos setpoints */
	if(ModeAuto == true)
	{
		ModeAuto = false;

		/* Reset alt pos flags if resetting is enabled. */
		if(DoResetAltPos == true)
		{
			ResetPositionSetpoint = true;
			ResetAltitudeSetpoint = true;
		}
	}

	/*
	 * Map from stick input to velocity setpoint.
	 */

	/* Velocity setpoint commanded by user stick input. */
	math::Vector3F ManVelSp(0.0f, 0.0f, 0.0f);

	if(VehicleControlModeMsg.ControlAltitudeEnabled)
	{
		/* Set vertical velocity setpoint with throttle stick, remapping of
		 * manual.z [0,1] to up and down command [-1,1] */
		ManVelSp[2] = -math::expof_deadzone(
				(ManualControlSetpointMsg.Z - 0.5f) * 2.0f,
				ConfigTblPtr->Z_MAN_EXPO, ConfigTblPtr->HOLD_DZ);

		/* Reset alt setpoint to current altitude if needed. */
		ResetAltSetpoint();
	}

	if (VehicleControlModeMsg.ControlPositionEnabled)
	{
		/* Set horizontal velocity setpoint with roll/pitch stick */
		ManVelSp[0] = math::expof_deadzone(
				ManualControlSetpointMsg.X,
				ConfigTblPtr->XY_MAN_EXPO, ConfigTblPtr->HOLD_DZ);
		ManVelSp[1] = math::expof_deadzone(
				ManualControlSetpointMsg.Y,
				ConfigTblPtr->XY_MAN_EXPO, ConfigTblPtr->HOLD_DZ);

		const float ManVelHorLength = math::Vector2F(ManVelSp[0], ManVelSp[1]).Length();

		/* Saturate such that magnitude is never larger than 1 */
		if (ManVelHorLength > 1.0f) {
			ManVelSp[0] /= ManVelHorLength;
			ManVelSp[1] /= ManVelHorLength;
		}

		/* Reset position setpoint to current position if needed */
		ResetPosSetpoint();
	}
	
	/* Prepare yaw to rotate into NED frame */
	float YawInputFrame = VehicleControlModeMsg.ControlFixedHdgEnabled ? YawTakeoff : VehicleAttitudeSetpointMsg.YawBody;

	/* Setpoint in NED frame and scaled to cruise velocity */
	ManVelSp = math::Matrix3F3::FromEuler(0.0f, 0.0f, YawInputFrame) * ManVelSp;

	/* Adjust acceleration based on stick input */
	math::Vector2F StickXy(ManVelSp[0], ManVelSp[1]);
	SetManualAccelerationXY(StickXy, dt);
	float StickZ = ManVelSp[2];
	float MaxAccZ = 0.0f;
	SetManualAccelerationZ(MaxAccZ, StickZ, dt);

	/* Prepare cruise speed (m/s) vector to scale the velocity setpoint */
	float VelMag = (ConfigTblPtr->MPC_VEL_MANUAL < VelMaxXY) ? ConfigTblPtr->MPC_VEL_MANUAL : VelMaxXY;
	math::Vector3F VelCruiseScale(VelMag, VelMag, (ManVelSp[2] > 0.0f) ? ConfigTblPtr->Z_VEL_MAX_DN : ConfigTblPtr->Z_VEL_MAX_UP);

	/* Setpoint scaled to cruise speed */
	ManVelSp = ManVelSp.EMult(VelCruiseScale);

	/*
	 * Assisted velocity mode: User controls velocity, but if velocity is small enough, position
	 * hold is activated for the corresponding axis.
	 */

	/* Want to get/stay in altitude hold if user has z stick in the middle (accounted for deadzone already) */
	const bool AltHoldDesired = VehicleControlModeMsg.ControlAltitudeEnabled && (UserIntentionZ == BRAKE);

	/* Want to get/stay in position hold if user has xy stick in the middle (accounted for deadzone already) */
	const bool PosHoldDesired = VehicleControlModeMsg.ControlPositionEnabled && (UserIntentionXY ==  BRAKE);

	/* Check vertical hold engaged flag. */
	if (AltitudeHoldEngaged)
	{
		AltitudeHoldEngaged = AltHoldDesired;
	}
	else
	{
		/* Check if we switch to alt_hold_engaged. */
		bool SmoothAltTransition = AltHoldDesired && ((MaxAccZ - AccelerationStateDependentZ) < FLT_EPSILON) &&
							     (ConfigTblPtr->HOLD_MAX_Z < FLT_EPSILON || fabsf(Velocity[2]) < ConfigTblPtr->HOLD_MAX_Z);

		/* During transition predict setpoint forward. */
		if (SmoothAltTransition)
		{
			/* Time to travel from current velocity to zero velocity. */
			float DeltaT = fabsf(Velocity[2] / MaxAccZ);

			/* Set desired position setpoint assuming max acceleraiton. */
			PositionSetpoint[2] = Position[2] + Velocity[2] * DeltaT + 0.5f * MaxAccZ * DeltaT * DeltaT;

			AltitudeHoldEngaged = true;
		}
	}

	/* Check horizontal hold engaged flag. */
	if (PositionHoldEngaged)
	{
		PositionHoldEngaged = PosHoldDesired;

		/* Use max acceleration */
		if (PositionHoldEngaged)
		{
			AccelerationStateDependentXY = ConfigTblPtr->ACC_HOR_MAX;
		}
	}
	else
	{
		/* Check if we switch to pos_hold_engaged. */
		float VelXyMag = sqrtf(Velocity[0] * Velocity[0] + Velocity[1] * Velocity[1]);
		bool SmoothPosTransition = PosHoldDesired
							     && (fabsf(ConfigTblPtr->ACC_HOR_MAX - AccelerationStateDependentXY) < FLT_EPSILON) &&
							     (ConfigTblPtr->HOLD_MAX_XY < FLT_EPSILON || VelXyMag < ConfigTblPtr->HOLD_MAX_XY);

		/* During transition predict setpoint forward. */
		if (SmoothPosTransition)
		{
			/* Time to travel from current velocity to zero velocity. */
			float DeltaT = sqrtf(Velocity[0] * Velocity[0] + Velocity[1] * Velocity[1]) / ConfigTblPtr->ACC_HOR_MAX;

			/* Position setpoint in xy from max acceleration and current velocity */
			math::Vector2F Pos(Position[0], Position[1]);
			math::Vector2F Vel(Velocity[0], Velocity[1]);
			math::Vector2F PosSp = Pos + Vel * DeltaT - Vel.Normalized() * 0.5f * ConfigTblPtr->ACC_HOR_MAX * DeltaT * DeltaT;
			PositionSetpoint[0] = PosSp[0];
			PositionSetpoint[1] = PosSp[1];

			PositionHoldEngaged = true;
		}
	}

	/* Set requested velocity setpoints */
	if (!AltitudeHoldEngaged)
	{
		PositionSetpoint[2] = Position[2];
		/* Request velocity setpoint to be used, instead of altitude setpoint */
		RunAltControl = false;
		VelocitySetpoint[2] = ManVelSp[2];
	}

	if (!PositionHoldEngaged)
	{
		PositionSetpoint[0] = Position[0];
		PositionSetpoint[1] = Position[1];
		/* Request velocity setpoint to be used, instead of position setpoint */
		RunPosControl = false;
		VelocitySetpoint[0] = ManVelSp[0];
		VelocitySetpoint[1] = ManVelSp[1];
	}

	ControlPosition(dt);
}



void MPC::ControlNonManual(float dt) // NEW DONE
{
	/* Select control source. */
	if(VehicleControlModeMsg.ControlOffboardEnabled)
	{
		/* Offboard control */
		ControlOffboard(dt);
		ModeAuto = false;
	}
	else
	{
		HoldOffboardXY = false;
		HoldOffboardZ = false;

		/* AUTO */
		ControlAuto(dt);
	}

	/* Guard against any bad velocity values. */
	bool VelocityValid = isfinite(PositionSetpointTripletMsg.Current.VX) &&
			isfinite(PositionSetpointTripletMsg.Current.VY) &&
			PositionSetpointTripletMsg.Current.VelocityValid;

	/* Do not go slower than the follow target velocity when position tracking
	 * is active (set to valid)	 */
	if (PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_FOLLOW_TARGET &&
	    VelocityValid &&
		PositionSetpointTripletMsg.Current.PositionValid)
	{
		math::Vector3F FtVel(PositionSetpointTripletMsg.Current.VX, PositionSetpointTripletMsg.Current.VY, 0);
		float CosRatio = (FtVel * VelocitySetpoint) / (FtVel.Length() * VelocitySetpoint.Length());

		/* Only override velocity set points when uav is traveling in same
		 * direction as target and vector component is greater than
		 * calculated position set point velocity component. */
		if (CosRatio > 0)
		{
			FtVel = FtVel * CosRatio;
			/* Min speed a little faster than target vel. */
			FtVel = FtVel + FtVel.Normalized() * 1.5f;

		}
		else
		{
			FtVel.Zero();
		}

		/* Track target using velocity only. */
		VelocitySetpoint[0] = fabsf(FtVel[0]) > fabsf(VelocitySetpoint[0]) ? FtVel[0] : VelocitySetpoint[0];
		VelocitySetpoint[1] = fabsf(FtVel[1]) > fabsf(VelocitySetpoint[1]) ? FtVel[1] : VelocitySetpoint[1];
	}
	else if(PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_FOLLOW_TARGET &&
		   VelocityValid)
	{
		VelocitySetpoint[0] = PositionSetpointTripletMsg.Current.VX;
		VelocitySetpoint[1] = PositionSetpointTripletMsg.Current.VY;
	}

	/* Use constant descend rate when landing, ignore altitude setpoint. */
	if (PositionSetpointTripletMsg.Current.Valid
	    && PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LAND)
	{
		VelocitySetpoint[2] = ConfigTblPtr->LAND_SPEED;
		RunAltControl = false;
	}

	if (PositionSetpointTripletMsg.Current.Valid
	    && PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_IDLE)
	{
		/* Idle state, don't run controller and set zero thrust. */
		RSetpoint.Identity();
		math::Quaternion Qd(RSetpoint);
		Qd.copyTo(VehicleAttitudeSetpointMsg.Q_D);
		VehicleAttitudeSetpointMsg.Q_D_Valid = true;

		VehicleAttitudeSetpointMsg.RollBody = 0.0f;
		VehicleAttitudeSetpointMsg.PitchBody = 0.0f;
		VehicleAttitudeSetpointMsg.YawBody = Yaw;
		VehicleAttitudeSetpointMsg.Thrust = 0.0f;

		VehicleAttitudeSetpointMsg.Timestamp = PX4LIB_GetPX4TimeUs();
	}
	else
	{
		ControlPosition(dt);
	}
}



float MPC::ThrottleCurve(float ctl, float ctr) // NEW DONE
{
	float result;

	/* Piecewise linear mapping: 0:ctr -> 0:0.5
	 * and ctr:1 -> 0.5:1 */
	if (ctl < 0.5f)
	{
		result = 2.0f * ctl * ctr;
	}
	else
	{
		result = ctr + 2.0f * (ctl - 0.5f) * (1.0f - ctr);
	}

	return result;
}



void MPC::ResetPosSetpoint(void) // NEW DONE
{
	if (ResetPositionSetpoint)
	{
		ResetPositionSetpoint = false;

		/* We have logic in the main function which chooses the velocity setpoint such that the attitude setpoint is
		 * continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		 * altitude in a special way. */
		PositionSetpoint[0] = Position[0];
		PositionSetpoint[1] = Position[1];
	}
}



void MPC::ResetAltSetpoint(void) // NEW DONE
{
	if (ResetAltitudeSetpoint)
	{
		ResetAltitudeSetpoint = false;

		/* We have logic in the main function which chooses the velocity setpoint such that the attitude setpoint is
		 * continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		 * position in a special way. In position control mode the position will be reset anyway until the vehicle has reduced speed.
		 */
		PositionSetpoint[2] = Position[2];
	}
}



void MPC::ControlPosition(float dt) // NEW DONE
{
	CalculateVelocitySetpoint(dt);

	if (VehicleControlModeMsg.ControlClimbRateEnabled ||
		VehicleControlModeMsg.ControlVelocityEnabled ||
		VehicleControlModeMsg.ControlAccelerationEnabled)
	{
		CalculateThrustSetpoint(dt);
	}
	else
	{
		ResetIntZ = true;
	}
}



void MPC::ControlOffboard(float dt) // NEW DONE
{
	if(PositionSetpointTripletMsg.Current.Valid)
	{
		if (VehicleControlModeMsg.ControlPositionEnabled && PositionSetpointTripletMsg.Current.PositionValid)
		{
			/* Control position */
			PositionSetpoint[0] = PositionSetpointTripletMsg.Current.X;
			PositionSetpoint[1] = PositionSetpointTripletMsg.Current.Y;
			RunPosControl = true;

			HoldOffboardXY = false;
		}
		else if(VehicleControlModeMsg.ControlVelocityEnabled &&
				PositionSetpointTripletMsg.Current.VelocityValid)
		{
			/* Control velocity */

			/* Reset position setpoint to current position if needed */
			ResetPosSetpoint();

			if (fabsf(PositionSetpointTripletMsg.Current.VX) <= FLT_EPSILON &&
			    fabsf(PositionSetpointTripletMsg.Current.VY) <= FLT_EPSILON &&
			    VehicleLocalPositionMsg.XY_Valid)
			{
				if (!HoldOffboardXY)
				{
					PositionSetpoint[0] = Position[0];
					PositionSetpoint[1] = Position[1];
					HoldOffboardXY = true;
				}

				RunPosControl = true;
			}
			else
			{
				if (PositionSetpointTripletMsg.Current.VelocityFrame == PX4_VELOCITY_FRAME_LOCAL_NED)
				{
					/* Set position setpoint move rate */
					VelocitySetpoint[0] = PositionSetpointTripletMsg.Current.VX;
					VelocitySetpoint[1] = PositionSetpointTripletMsg.Current.VY;
				}
				else if (PositionSetpointTripletMsg.Current.VelocityFrame == PX4_VELOCITY_FRAME_BODY_NED) {
					/* Transform velocity command from body frame to NED frame */
					VelocitySetpoint[0] = cosf(Yaw) * PositionSetpointTripletMsg.Current.VX - sinf(Yaw) * PositionSetpointTripletMsg.Current.VY;
					VelocitySetpoint[1] = sinf(Yaw) * PositionSetpointTripletMsg.Current.VX + cosf(Yaw) * PositionSetpointTripletMsg.Current.VY;
				}
				else
				{
		            (void) CFE_EVS_SendEvent(MPC_UNK_VEL_FRM_ERR_EID, CFE_EVS_ERROR,
		            		"Unknown velocity offboard coordinate frame. (%u)",
							PositionSetpointTripletMsg.Current.VelocityFrame);
				}

				RunPosControl = false;
				HoldOffboardXY = false;
			}
		}

		if (VehicleControlModeMsg.ControlAltitudeEnabled && PositionSetpointTripletMsg.Current.AltValid)
		{
			/* Control altitude as it is enabled. */
			PositionSetpoint[2] = PositionSetpointTripletMsg.Current.Z;
			RunAltControl = true;
			HoldOffboardZ = false;
		}
		else if (VehicleControlModeMsg.ControlClimbRateEnabled && PositionSetpointTripletMsg.Current.VelocityValid)
		{
			/* Reset alt setpoint to current altitude if needed */
			ResetAltSetpoint();

			if (fabsf(PositionSetpointTripletMsg.Current.VZ) <= FLT_EPSILON &&
			    VehicleLocalPositionMsg.Z_Valid)
			{
				if (!HoldOffboardZ)
				{
					PositionSetpoint[2] = Position[2];
					HoldOffboardZ = true;
				}

				RunAltControl = true;
			}
			else
			{
				/* Set position setpoint move rate */
				VelocitySetpoint[2] = PositionSetpointTripletMsg.Current.VZ;
				RunAltControl = false;
				HoldOffboardZ = false;
			}
		}

		if (PositionSetpointTripletMsg.Current.YawValid)
		{
			VehicleAttitudeSetpointMsg.YawBody = PositionSetpointTripletMsg.Current.Yaw;
		}
		else if (PositionSetpointTripletMsg.Current.YawspeedValid)
		{
			float YawTarget = _wrap_pi(VehicleAttitudeSetpointMsg.YawBody + PositionSetpointTripletMsg.Current.Yawspeed * dt);
			float YawOffs = _wrap_pi(YawTarget - Yaw);
			// TODO: Check if these need to be radians
			const float YawRateMax = (math::radians(ConfigTblPtr->MAN_Y_MAX) < math::radians(ConfigTblPtr->MC_YAWRATE_MAX)) ? math::radians(ConfigTblPtr->MAN_Y_MAX) :
					math::radians(ConfigTblPtr->MC_YAWRATE_MAX);
			const float YawOffsetMax = YawRateMax / math::radians(ConfigTblPtr->MC_YAW_P);

			// If the yaw offset became too big for the system to track stop
			// shifting it, only allow if it would make the offset smaller again.
			if (fabsf(YawOffs) < YawOffsetMax ||
				(PositionSetpointTripletMsg.Current.Yawspeed > 0 && YawOffs < 0) ||
				(PositionSetpointTripletMsg.Current.Yawspeed < 0 && YawOffs > 0))
			{
				VehicleAttitudeSetpointMsg.YawBody = YawTarget;
			}
		}
	}
	else
	{
		HoldOffboardXY = false;
		HoldOffboardZ = false;
		ResetPosSetpoint();
		ResetAltSetpoint();
	}
}

void MPC::ControlAuto(float dt) // NEW DONE
{

	/* Reset position setpoint on AUTO mode activation or if we are not in MC mode */
	if (!ModeAuto || !VehicleStatusMsg.IsRotaryWing)
	{
		if (!ModeAuto)
		{
			ModeAuto = true;
			TripletLatLonFinite = true;
		}

		ResetPositionSetpoint = true;
		ResetAltitudeSetpoint = true;
	}

	/* Always check reset state of altitude and position control flags in auto. */
	ResetPosSetpoint();
	ResetAltSetpoint();

	bool CurrentSetpointValid = false;
	bool PreviousSetpointValid = false;
	bool NextSetpointValid = false;
	bool TripletUpdated = false;

	math::Vector3F PrevSp(0.0f, 0.0f, 0.0f);
	math::Vector3F NextSp(0.0f, 0.0f, 0.0f);

	if (PositionSetpointTripletMsg.Current.Valid)
	{
		math::Vector3F CurrPosSp = CurrentPositionSetpoint;

		/* Only project setpoints if they are finite, else use current
		 * position. */
		if (isfinite(PositionSetpointTripletMsg.Current.Lat) &&
		    isfinite(PositionSetpointTripletMsg.Current.Lon))
		{
			/* Project setpoint to local frame. */
			map_projection_project(&RefPos,
					PositionSetpointTripletMsg.Current.Lat, PositionSetpointTripletMsg.Current.Lon,
					       &CurrPosSp[0], &CurrPosSp[1]);

			TripletLatLonFinite = true;
		}
		/* Use current position if NAN -> e.g. land */
		else
		{
			if (TripletLatLonFinite)
			{
				CurrPosSp[0] = Position[0];
				CurrPosSp[1] = Position[1];
				TripletLatLonFinite = false;
			}
		}

		/* Only project setpoints if they are finite, else use current position. */
		if (isfinite(PositionSetpointTripletMsg.Current.Alt))
		{
			CurrPosSp[2] = -(PositionSetpointTripletMsg.Current.Alt - RefAlt);
		}

		/* Sanity check */
		if (isfinite(CurrentPositionSetpoint[0]) &&
			isfinite(CurrentPositionSetpoint[1]) &&
			isfinite(CurrentPositionSetpoint[2]))
		{
			CurrentSetpointValid = true;
		}

		/* Check if triplets have been updated
		 * note: we only can look at xy since navigator applies slewrate to z */
		float  diff = 0.0f;

		if (TripletLatLonFinite)
		{
			diff = math::Vector2F((CurrentPositionSetpoint[0] - CurrPosSp[0]), (CurrentPositionSetpoint[1] - CurrPosSp[1])).Length();
		}
		else
		{
			diff = fabsf(CurrentPositionSetpoint[2] - CurrPosSp[2]);
		}

		if (diff > FLT_EPSILON || !isfinite(diff)) {
			TripletUpdated = true;
		}

		/* We need to update CurrentPositionSetpoint always since navigator applies slew rate on z */
		CurrentPositionSetpoint = CurrPosSp;
	}

	if (PositionSetpointTripletMsg.Previous.Valid)
	{
		map_projection_project(&RefPos,
				PositionSetpointTripletMsg.Previous.Lat, PositionSetpointTripletMsg.Previous.Lon,
				       &PrevSp[0], &PrevSp[1]);
		PrevSp[2] = -(PositionSetpointTripletMsg.Previous.Alt - RefAlt);

		if (isfinite(PrevSp[0]) &&
			isfinite(PrevSp[1]) &&
			isfinite(PrevSp[2]))
		{
			PreviousPositionSetpoint = PrevSp;
			PreviousSetpointValid = true;
		}
	}

	/* Set previous setpoint to current position if no previous setpoint available */
	if (!PreviousSetpointValid && TripletUpdated) {
		PreviousPositionSetpoint = Position;
		PreviousSetpointValid = true; /* currrently not necessary to set to true since not used*/
	}

	if (PositionSetpointTripletMsg.Next.Valid)
	{
		map_projection_project(&RefPos,
				PositionSetpointTripletMsg.Next.Lat, PositionSetpointTripletMsg.Next.Lon,
				       &NextSp[0], &NextSp[1]);
		NextSp[2] = -(PositionSetpointTripletMsg.Next.Alt - RefAlt);

		if (isfinite(NextSp[0]) &&
			isfinite(NextSp[1]) &&
			isfinite(NextSp[2]))
		{
			NextSetpointValid = true;
		}
	}

	/* Auto logic:
	 * The vehicle should follow the line previous-current.
	 * - if there is no next setpoint or the current is a loiter point, then slowly approach the current along the line
	 * - if there is a next setpoint, then the velocity is adjusted depending on the angle of the corner prev-current-next.
	 * When following the line, the PosSp is computed from the orthogonal distance to the closest point on line and the desired cruise speed along the track.
	 */

	/* create new _PosSp from triplets */
	if (CurrentSetpointValid &&
	    (PositionSetpointTripletMsg.Current.Type != PX4_SETPOINT_TYPE_IDLE))
	{
		/* update yaw setpoint if needed */
		if (PositionSetpointTripletMsg.Current.YawspeedValid
			&& PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_FOLLOW_TARGET)
		{
			VehicleAttitudeSetpointMsg.YawBody = VehicleAttitudeSetpointMsg.YawBody + PositionSetpointTripletMsg.Current.Yawspeed * dt;
		}
		else if (isfinite(PositionSetpointTripletMsg.Current.Yaw))
		{
			VehicleAttitudeSetpointMsg.YawBody = PositionSetpointTripletMsg.Current.Yaw;
		}

		float YawDiff = _wrap_pi(VehicleAttitudeSetpointMsg.YawBody - Yaw);

		/* Only follow previous-current-line for specific triplet type */
		if (PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_POSITION  ||
			PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER ||
			PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_FOLLOW_TARGET)
		{
			/* By default, use current setpoint as is. */
			math::Vector3F PosSp = CurrentPositionSetpoint;

			/*
			 * Z-DIRECTION
			 */

			/* Get various distances */
			float TotalDistZ = fabsf(CurrentPositionSetpoint[2] - PreviousPositionSetpoint[2]);
			float DistToPrevZ = fabsf(Position[2] - PreviousPositionSetpoint[2]);
			float DistToCurrentZ = fabsf(CurrentPositionSetpoint[2] - Position[2]);

			/* If PosSp has not reached target setpoint (=CurrentPositionSetpoint[2]),
			 * then compute setpoint depending on vel_max */
			if ((TotalDistZ >  SIGMA_NORM) && (fabsf(PositionSetpoint[2] - CurrentPositionSetpoint[2]) > SIGMA_NORM)) {

				/* Check sign */
				bool FlyingUpward = CurrentPositionSetpoint[2] < Position[2];

				/* Final_vel_z is the max velocity which depends on the distance of TotalDistZ
				 * with default params.vel_max_up/down */
				float FinalVelZ = (FlyingUpward) ? ConfigTblPtr->Z_VEL_MAX_UP : ConfigTblPtr->Z_VEL_MAX_DN;

				/* Target threshold defines the distance to CurrentPositionSetpoint[2] at which
				 * the vehicle starts to slow down to approach the target smoothly */
				float TargetThresholdZ = FinalVelZ * 1.5f;

				/* If the total distance in z is NOT 2x distance of target_threshold, we
				 * will need to adjust the final_vel_z */
				bool Is2TargetThresholdZ = TotalDistZ >= 2.0f * TargetThresholdZ;
				float Slope = (FinalVelZ) / (TargetThresholdZ); /* defines the the acceleration when slowing down */
				float MinVelZ = 0.2f; // minimum velocity: this is needed since estimation is not perfect

				if (!Is2TargetThresholdZ)
				{
					/* Adjust final_vel_z since we are already very close
					 * to current and therefore it is not necessary to accelerate
					 * up to full speed (=final_vel_z) */
					TargetThresholdZ = TotalDistZ * 0.5f;

					/* Get the velocity at TargetThresholdZ */
					float FinalVelZTmp = Slope * (TargetThresholdZ) + MinVelZ;

					/* Make sure that final_vel_z is never smaller than 0.5 of the default final_vel_z
					 * this is mainly done because the estimation in z is not perfect and therefore
					 * it is necessary to have a minimum speed */
					FinalVelZ = math::constrain(FinalVelZTmp, FinalVelZ * 0.5f, FinalVelZ);
				}

				float VelSpZ = FinalVelZ;

				/* We want to slow down */
				if (DistToCurrentZ < TargetThresholdZ)
				{
					VelSpZ = Slope * DistToCurrentZ + MinVelZ;
				}
				else if (DistToPrevZ < TargetThresholdZ)
				{
					/* We want to accelerate */
					float AccZ = (VelSpZ - fabsf(VelocitySetpoint[2])) / dt;
					float AccMax = (FlyingUpward) ? (ConfigTblPtr->ACC_UP_MAX * 0.5f) : (ConfigTblPtr->ACC_DOWN_MAX * 0.5f);

					if (AccZ > AccMax)
					{
						VelSpZ = ConfigTblPtr->ACC_UP_MAX * dt + fabsf(VelocitySetpoint[2]);
					}

				}

				/* If we already close to current, then just take over the velocity that
				 * we would have computed if going directly to the current setpoint
				 */
				if (VelSpZ >= (DistToCurrentZ * PosP[2]))
				{
					VelSpZ = DistToCurrentZ * PosP[2];
				}

				/* Make sure VelSpZ is always positive */
				VelSpZ = math::constrain(VelSpZ, 0.0f, FinalVelZ);
				/* Get the sign of VelSpZ */
				VelSpZ = (FlyingUpward) ? -VelSpZ : VelSpZ;
				/* Compute PosSp[2] */
				PosSp[2] = Position[2] + VelSpZ / PosP[2];
			}

			/*
			 * XY-DIRECTION
			 */

			/* Line from previous to current and from pos to current */
			math::Vector2F VecPrevToCurrent((CurrentPositionSetpoint[0] - PreviousPositionSetpoint[0]), (CurrentPositionSetpoint[1] - PreviousPositionSetpoint[1]));
			math::Vector2F VecPosToCurrent((CurrentPositionSetpoint[0] - Position[0]), (CurrentPositionSetpoint[1] - Position[1]));


			/* Check if we just want to stay at current position */
			math::Vector2F PosSpDiff((CurrentPositionSetpoint[0] - PositionSetpoint[0]), (CurrentPositionSetpoint[1] - PositionSetpoint[1]));
			bool StayAtCurrentPos = (PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER
							|| !NextSetpointValid)
						   && ((PosSpDiff.Length()) < SIGMA_NORM);

			/* Only follow line if previous to current has a minimum distance */
			if ((VecPrevToCurrent.Length()  > ConfigTblPtr->NAV_ACC_RAD) && !StayAtCurrentPos)
			{
				/* Normalize prev-current line (always > nav_rad) */
				math::Vector2F UnitPrevToCurrent = VecPrevToCurrent.Normalized();

				/* Unit vector from current to next */
				math::Vector2F UnitCurrentToNext(0.0f, 0.0f);

				if (NextSetpointValid)
				{
					UnitCurrentToNext = math::Vector2F((NextSp[0] - PosSp[0]), (NextSp[1] - PosSp[1]));
					UnitCurrentToNext = (UnitCurrentToNext.Length() > SIGMA_NORM) ? UnitCurrentToNext.Normalized() :
								   UnitCurrentToNext;
				}

				/* Point on line closest to pos */
				math::Vector2F ClosestPoint = math::Vector2F(PreviousPositionSetpoint[0], PreviousPositionSetpoint[1]) + UnitPrevToCurrent *
								 (math::Vector2F((Position[0] - PreviousPositionSetpoint[0]), (Position[1] - PreviousPositionSetpoint[1])) * UnitPrevToCurrent);

				math::Vector2F VecClosestToCurrent((CurrentPositionSetpoint[0] - ClosestPoint[0]), (CurrentPositionSetpoint[1] - ClosestPoint[1]));

				/* Compute vector from position-current and previous-position */
				math::Vector2F VecPrevToPos((Position[0] - PreviousPositionSetpoint[0]), (Position[1] - PreviousPositionSetpoint[1]));

				/* Current velocity along track */
				float VelSpAlongTrackPrev = math::Vector2F(VelocitySetpoint[0], VelocitySetpoint[1]) * UnitPrevToCurrent;

				/* Distance to target when brake should occur */
				float TargetThresholdXy = 1.5f * GetCruisingSpeedXY();

				bool CloseToCurrent = VecPosToCurrent.Length() < TargetThresholdXy;
				bool CloseToPrev = (VecPrevToPos.Length() < TargetThresholdXy) &&
							 (VecPrevToPos.Length() < VecPosToCurrent.Length());

				/* Indicates if we are at least half the distance from previous to current close to previous */
				bool Is2TargetThreshold = VecPrevToCurrent.Length() >= 2.0f * TargetThresholdXy;

				/* Check if the current setpoint is behind */
				bool CurrentBehind = ((VecPosToCurrent * -1.0f) * UnitPrevToCurrent) > 0.0f;

				/* Check if the previous is in front */
				bool PreviousInFront = (VecPrevToPos * UnitPrevToCurrent) < 0.0f;

				/* Default velocity along line prev-current */
				float VelSpAlongTrack = GetCruisingSpeedXY();

				/*
				 * Compute velocity setpoint along track
				 */

				/* Only go directly to previous setpoint if more than 5m away and previous in front*/
				if (PreviousInFront && (VecPrevToPos.Length() > 5.0f))
				{
					/* Just use the default velocity along track */
					VelSpAlongTrack = VecPrevToPos.Length() * PosP[0];

					if (VelSpAlongTrack > GetCruisingSpeedXY())
					{
						VelSpAlongTrack = GetCruisingSpeedXY();
					}

				}
				else if (CurrentBehind)
				{
					/* Go directly to current setpoint */
					VelSpAlongTrack = VecPosToCurrent.Length() * PosP[0];
					VelSpAlongTrack = (VelSpAlongTrack < GetCruisingSpeedXY()) ? VelSpAlongTrack : GetCruisingSpeedXY();

				}
				else if (CloseToPrev)
				{
					/* Accelerate from previous setpoint towards current setpoint */

					/* We are close to previous and current setpoint
					 * we first compute the start velocity when close to current septoint and use
					 * this velocity as final velocity when transition occurs from acceleration to deceleration.
					 * This ensures smooth transition */
					float FinalCruiseSpeed = GetCruisingSpeedXY();

					if (!Is2TargetThreshold)
					{
						/* Set target threshold to half dist pre-current */
						float TargetThresholdTmp = TargetThresholdXy;
						TargetThresholdXy = VecPrevToCurrent.Length() * 0.5f;

						if ((TargetThresholdXy - ConfigTblPtr->NAV_ACC_RAD) < SIGMA_NORM)
						{
							TargetThresholdXy = ConfigTblPtr->NAV_ACC_RAD;
						}

						/* Velocity close to current setpoint with default zero if no next setpoint is available */
						float vel_close = 0.0f;
						float acceptance_radius = 0.0f;

						/* We want to pass and need to compute the desired velocity close to current setpoint */
						if (NextSetpointValid &&  !(PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER))
						{
							/* Get velocity close to current that depends on angle between prev-current and current-next line */
							vel_close = GetVelClose(UnitPrevToCurrent, UnitCurrentToNext);
							acceptance_radius = ConfigTblPtr->NAV_ACC_RAD;
						}

						/* Compute velocity at transition where vehicle switches from acceleration to deceleration */
						if ((TargetThresholdTmp - acceptance_radius) < SIGMA_NORM)
						{
							FinalCruiseSpeed = vel_close;
						}
						else
						{
							float slope = (GetCruisingSpeedXY() - vel_close) / (TargetThresholdTmp - acceptance_radius);
							FinalCruiseSpeed = slope  * (TargetThresholdXy - acceptance_radius) + vel_close;
							FinalCruiseSpeed = (FinalCruiseSpeed > vel_close) ? FinalCruiseSpeed : vel_close;
						}
					}

					/* Make sure final cruise speed is larger than 0*/
					FinalCruiseSpeed = (FinalCruiseSpeed > SIGMA_NORM) ? FinalCruiseSpeed : SIGMA_NORM;
					VelSpAlongTrack = FinalCruiseSpeed;

					/* We want to accelerate not too fast
					* TODO: change the name acceleration_hor_man to something that can
					* be used by auto and manual */
					float acc_track = (FinalCruiseSpeed - VelSpAlongTrackPrev) / dt;

					/* If yaw offset is large, only accelerate with 0.5m/s^2 */
					float acc = (fabsf(YawDiff) >  math::radians(ConfigTblPtr->NAV_MIS_YAW_ERR)) ? 0.5f : ConfigTblPtr->MPC_ACC_HOR;

					if (acc_track > acc)
					{
						VelSpAlongTrack = acc * dt + VelSpAlongTrackPrev;
					}

					/* Enforce minimum cruise speed */
					VelSpAlongTrack  = math::constrain(VelSpAlongTrack, SIGMA_NORM, FinalCruiseSpeed);

				}
				else if (CloseToCurrent)
				{
					/* Slow down when close to current setpoint */

					/* Check if altidue is within acceptance radius */
					bool reached_altitude = (DistToCurrentZ < ConfigTblPtr->NAV_ACC_RAD) ? true : false;

					if (reached_altitude && NextSetpointValid
						&& !(PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER))
					{
						/* Since we have a next setpoint use the angle prev-current-next to compute velocity setpoint limit */

						/* Get velocity close to current that depends on angle between prev-current and current-next line */
						float VelClose = GetVelClose(UnitPrevToCurrent, UnitCurrentToNext);

						/* Compute velocity along line which depends on distance to current setpoint */
						if (VecClosestToCurrent.Length() < ConfigTblPtr->NAV_ACC_RAD)
						{
							VelSpAlongTrack = VelClose;
						}
						else
						{
							if (TargetThresholdXy - ConfigTblPtr->NAV_ACC_RAD < SIGMA_NORM)
							{
								VelSpAlongTrack = VelClose;
							}
							else
							{
								float slope = (GetCruisingSpeedXY() - VelClose) / (TargetThresholdXy - ConfigTblPtr->NAV_ACC_RAD) ;
								VelSpAlongTrack = slope  * (VecClosestToCurrent.Length() - ConfigTblPtr->NAV_ACC_RAD) + VelClose;
							}
						}

						/* Since we want to slow down take over previous velocity setpoint along track if it was lower */
						if ((VelSpAlongTrackPrev < VelSpAlongTrack) && (VelSpAlongTrack * VelSpAlongTrackPrev > 0.0f))
						{
							VelSpAlongTrack = VelSpAlongTrackPrev;
						}

						/* If we are close to target and the previous velocity setpoints was smaller than
						 * VelSpAlongTrack, then take over the previous one
						 * this ensures smoothness since we anyway want to slow down
						 */
						if ((VelSpAlongTrackPrev < VelSpAlongTrack) && (VelSpAlongTrack * VelSpAlongTrackPrev > 0.0f)
							&& (VelSpAlongTrackPrev > VelClose))
						{
							VelSpAlongTrack = VelSpAlongTrackPrev;
						}

						/* Make sure that vel_sp_along track is at least min */
						VelSpAlongTrack = (VelSpAlongTrack < VelClose) ? VelClose : VelSpAlongTrack;


					}
					else
					{
						/* We want to stop at current setpoint */
						float slope = (GetCruisingSpeedXY())  / TargetThresholdXy;
						VelSpAlongTrack =  slope * (VecClosestToCurrent.Length());

						/* Since we want to slow down take over previous velocity setpoint along track if it was lower but ensure its not zero */
						if ((VelSpAlongTrackPrev < VelSpAlongTrack) && (VelSpAlongTrack * VelSpAlongTrackPrev > 0.0f)
							&& (VelSpAlongTrackPrev > 0.5f))
						{
							VelSpAlongTrack = VelSpAlongTrackPrev;
						}
					}
				}

				/* Compute velocity orthogonal to prev-current-line to position*/
				math::Vector2F VecPosToClosest = ClosestPoint - math::Vector2F(Position[0], Position[1]);
				float VelSpOrthogonal = VecPosToClosest.Length() * PosP[0];

				/* Compute the cruise speed from velocity along line and orthogonal velocity setpoint */
				float CruiseSpMag = sqrtf(VelSpOrthogonal * VelSpOrthogonal + VelSpAlongTrack * VelSpAlongTrack);

				/* Sanity check */
				CruiseSpMag = (isfinite(CruiseSpMag)) ? CruiseSpMag : VelSpOrthogonal;

				/* Orthogonal velocity setpoint is smaller than cruise speed */
				if (VelSpOrthogonal < GetCruisingSpeedXY() && !CurrentBehind)
				{
					/* We need to limit VelSpAlongTrack such that cruise speed  is never exceeded but still can keep velocity orthogonal to track */
					if (CruiseSpMag > GetCruisingSpeedXY())
					{
						VelSpAlongTrack = sqrtf(GetCruisingSpeedXY() * GetCruisingSpeedXY() - VelSpOrthogonal * VelSpOrthogonal);
					}

					PosSp[0] = ClosestPoint[0] + UnitPrevToCurrent[0] * VelSpAlongTrack / PosP[0];
					PosSp[1] = ClosestPoint[1] + UnitPrevToCurrent[1] * VelSpAlongTrack / PosP[1];

				}
				else if (CurrentBehind)
				{
					/* Current is behind */
					if (VecPosToCurrent.Length()  > 0.01f)
					{
						PosSp[0] = Position[0] + VecPosToCurrent[0] / VecPosToCurrent.Length() * VelSpAlongTrack / PosP[0];
						PosSp[1] = Position[1] + VecPosToCurrent[1] / VecPosToCurrent.Length() * VelSpAlongTrack / PosP[1];
					}
					else
					{
						PosSp[0] = CurrentPositionSetpoint[0];
						PosSp[1] = CurrentPositionSetpoint[1];
					}

				}
				else
				{
					/* We are more than cruise_speed away from track */

					/* If previous is in front just go directly to previous point */
					if (PreviousInFront)
					{
						VecPosToClosest[0] = PreviousPositionSetpoint[0] - Position[0];
						VecPosToClosest[1] = PreviousPositionSetpoint[1] - Position[1];
					}

					/* Make sure that we never exceed maximum cruise speed */
					float cruise_sp = VecPosToClosest.Length() * PosP[0];

					if (cruise_sp > GetCruisingSpeedXY())
					{
						cruise_sp = GetCruisingSpeedXY();
					}

					/* Sanity check: don't divide by zero */
					if (VecPosToClosest.Length() > SIGMA_NORM)
					{
						PosSp[0] = Position[0] + VecPosToClosest[0] / VecPosToClosest.Length() * cruise_sp / PosP[0];
						PosSp[1] = Position[1] + VecPosToClosest[1] / VecPosToClosest.Length() * cruise_sp / PosP[1];
					}
					else
					{
						PosSp[0] = ClosestPoint[0];
						PosSp[1] = ClosestPoint[1];
					}
				}
			}

			PositionSetpoint = PosSp;

		}
		else if (PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_VELOCITY)
		{
			float VelXyMag = sqrtf(Velocity[0] * Velocity[0] + Velocity[1] * Velocity[1]);

			if (VelXyMag > SIGMA_NORM)
			{
				VelocitySetpoint[0] = Velocity[0] / VelXyMag * GetCruisingSpeedXY();
				VelocitySetpoint[1] = Velocity[1] / VelXyMag * GetCruisingSpeedXY();
			}
			else
			{
				/* TODO: We should go in the direction we are heading
				 * if current velocity is zero
				 */
				VelocitySetpoint[0] = 0.0f;
				VelocitySetpoint[1] = 0.0f;
			}

			RunPosControl = false;

		}
		else
		{
			/* Just go to the target point */;
			PositionSetpoint = CurrentPositionSetpoint;

			/* Set max velocity to cruise */
			m_VelMaxXy = GetCruisingSpeedXY();
		}

		/* Sanity check */
		if (!(isfinite(PositionSetpoint[0]) && isfinite(PositionSetpoint[1]) &&
			  isfinite(PositionSetpoint[2])))
		{
			//warn_rate_limited("Auto: Position setpoint not finite");
			PositionSetpoint = CurrentPositionSetpoint;
		}


		/* If we're already near the current takeoff setpoint don't reset in case we switch back to posctl.
		 * this makes the takeoff finish smoothly. */
		if ((PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_TAKEOFF
			 || PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER)
			&& PositionSetpointTripletMsg.Current.AcceptanceRadius > 0.0f
			/* Need to detect we're close a bit before the navigator switches from takeoff to next waypoint */
			&& (Position - PositionSetpoint).Length() < PositionSetpointTripletMsg.Current.AcceptanceRadius * 1.2f)
		{
			DoResetAltPos = false;
		}
		else
		{
			/* Otherwise: in case of interrupted mission don't go to waypoint but stay at current position */
			DoResetAltPos = true;
		}

//		// Handle the landing gear based on the manual landing alt
//		const bool high_enough_for_landing_gear = (-Position[2] + _home_pos.z > 2.0f);
//
//		// During a mission or in loiter it's safe to retract the landing gear.
//		if ((PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_POSITION ||
//			 PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER) &&
//			!_vehicle_land_detected.landed &&
//			high_enough_for_landing_gear) {
//
//			VehicleAttitudeSetpointMsg.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_UP;
//
//		} else if (PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_TAKEOFF ||
//			   PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LAND ||
//			   !high_enough_for_landing_gear) {
//
//			// During takeoff and landing, we better put it down again.
//			VehicleAttitudeSetpointMsg.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN;
//
//			// For the rest of the setpoint types, just leave it as is.
//		}

	}
	else
	{
		/* Idle or triplet not valid, set velocity setpoint to zero */
		VelocitySetpoint.Zero();
		RunPosControl = false;
		RunAltControl = false;
	}
}

void MPC::CalculateVelocitySetpoint(float dt) // NEW DONE
{
	/* Run position & altitude controllers if enabled (otherwise use already
	 * computed velocity setpoints) */
	if(RunPosControl)
	{
		/* If for any reason, we get a NaN position setpoint, we better just
		 * stay where we are. */
		if(isfinite(PositionSetpoint[0]) && isfinite(PositionSetpoint[1]))
		{
			VelocitySetpoint[0] = (PositionSetpoint[0] - Position[0]) * PosP[0];
			VelocitySetpoint[1] = (PositionSetpoint[1] - Position[1]) * PosP[1];
		}
		else
		{
			VelocitySetpoint[0] = 0.0f;
			VelocitySetpoint[1] = 0.0f;
		}
	}

	/* In auto the setpoint is already limited by the navigator */
	if (!VehicleControlModeMsg.ControlAutoEnabled)
	{
		LimitAltitude();
	}

	if (RunAltControl)
	{
		if(isfinite(PositionSetpoint[2]))
		{
			VelocitySetpoint[2] = (PositionSetpoint[2] - Position[2]) * PosP[2];
		}
		else
		{
			VelocitySetpoint[2] = 0.0f;
		}
	}

	if (!VehicleControlModeMsg.ControlPositionEnabled)
	{
		ResetPositionSetpoint = true;
	}

	if (!VehicleControlModeMsg.ControlAltitudeEnabled)
	{
		ResetAltitudeSetpoint = true;
	}

	if (!VehicleControlModeMsg.ControlVelocityEnabled)
	{
		VelocitySetpointPrevious[0] = Velocity[0];
		VelocitySetpointPrevious[1] = Velocity[1];
		VelocitySetpoint[0] = 0.0f;
		VelocitySetpoint[1] = 0.0f;
	}

	if (!VehicleControlModeMsg.ControlClimbRateEnabled)
	{
		VelocitySetpoint[2] = 0.0f;
	}

	/* Limit vertical upwards speed in auto takeoff and close to ground */
	float AltitudeAboveHome = -Position[2] + HomePositionMsg.Z;

	if (PositionSetpointTripletMsg.Current.Valid
	    && PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_TAKEOFF
	    && !VehicleControlModeMsg.ControlManualEnabled)
	{
		float TkVelLimit = math::gradual(AltitudeAboveHome,
										ConfigTblPtr->LAND_ALT2, ConfigTblPtr->LAND_ALT1,
										ConfigTblPtr->TKO_SPEED, ConfigTblPtr->Z_VEL_MAX_UP);
		VelocitySetpoint[2] = math::max(VelocitySetpoint[2], -TkVelLimit);
	}

	/* Limit vertical downwards speed (positive z) close to ground. For now we use the
	 * altitude above home and assume that we want to land at same height as we took off */
	float LandVelLimit = math::gradual(AltitudeAboveHome,
			ConfigTblPtr->LAND_ALT2, ConfigTblPtr->LAND_ALT1,
			ConfigTblPtr->LAND_SPEED, ConfigTblPtr->Z_VEL_MAX_DN);

	VelocitySetpoint[2] = math::min(VelocitySetpoint[2], LandVelLimit);

	/* Apply slew rate (aka acceleration limit) for smooth flying. */
	if (!VehicleControlModeMsg.ControlAutoEnabled && !InTakeoff)
	{
		ApplyVelocitySetpointSlewRate(dt);
	}

	/* Special velocity setpoint limitation for smooth takeoff. */
	if (InTakeoff)
	{
		InTakeoff = TakeoffVelLimit < -VelocitySetpoint[2];
		/* Ramp vertical velocity limit up to takeoff speed. */
		TakeoffVelLimit += -VelocitySetpoint[2] * dt / ConfigTblPtr->TKO_RAMP_T;
		/* Limit vertical velocity to the current ramp value. */
		VelocitySetpoint[2] = math::max(VelocitySetpoint[2], -TakeoffVelLimit);
	}

	/* Make sure velocity setpoint is constrained in all directions. */
	float VelNormXy = sqrtf(VelocitySetpoint[0] * VelocitySetpoint[0] + VelocitySetpoint[1] * VelocitySetpoint[1]);
	if (VelNormXy > ConfigTblPtr->XY_VEL_MAX)
	{
		VelocitySetpoint[0] = VelocitySetpoint[0] * ConfigTblPtr->XY_VEL_MAX / VelNormXy;
		VelocitySetpoint[1] = VelocitySetpoint[1] * ConfigTblPtr->XY_VEL_MAX / VelNormXy;
	}

	VelocitySetpoint[2] = math::constrain(VelocitySetpoint[2], -ConfigTblPtr->Z_VEL_MAX_UP, ConfigTblPtr->Z_VEL_MAX_DN);

	VelocitySetpointPrevious = VelocitySetpoint;
}

void MPC::CalculateThrustSetpoint(float dt) // NEW DONE
{
	/* Reset integrals if needed. */
	if (VehicleControlModeMsg.ControlClimbRateEnabled)
	{
		if (ResetIntZ)
		{
			ResetIntZ = false;
			ThrustInt[2] = 0.0f;
		}
	}
	else
	{
		ResetIntZ = true;
	}

	if (VehicleControlModeMsg.ControlVelocityEnabled)
	{
		if (ResetIntXY)
		{
			ResetIntXY = false;
			ThrustInt[0] = 0.0f;
			ThrustInt[1] = 0.0f;
		}
	}
	else
	{
		ResetIntXY = true;
	}

	/* If any of the velocity setpoint is bogus, it's probably safest to command no velocity at all. */
	for (int i = 0; i < 3; ++i)
	{
		if (!isfinite(VelocitySetpoint[i]))
		{
			VelocitySetpoint[i] = 0.0f;
		}
	}

	/* Velocity error */
	math::Vector3F VelErr = VelocitySetpoint - Velocity;

	/* Thrust vector in NED frame. */
	math::Vector3F ThrustSp(0.0f, 0.0f, 0.0f);

	if (VehicleControlModeMsg.ControlAccelerationEnabled && PositionSetpointTripletMsg.Current.AccelerationValid)
	{
		ThrustSp = math::Vector3F(PositionSetpointTripletMsg.Current.AX, PositionSetpointTripletMsg.Current.AY, PositionSetpointTripletMsg.Current.AZ);
	}
	else
	{
		ThrustSp = VelErr.EMult(VelP) + VelocityErrD.EMult(VelD)
			    + ThrustInt - math::Vector3F(0.0f, 0.0f, ConfigTblPtr->THR_HOVER);
	}

	if (!VehicleControlModeMsg.ControlVelocityEnabled && !VehicleControlModeMsg.ControlAccelerationEnabled)
	{
		ThrustSp[0] = 0.0f;
		ThrustSp[1] = 0.0f;
	}

	/* If still or already on ground command zero xy velocity and zero xy
	 * ThrustSp in body frame to consider uneven ground. */
	if (VehicleLandDetectedMsg.GroundContact && !InAutoTakeoff() && !ManualWantsTakeoff())
	{
		/* Thrust setpoint in body frame*/
		math::Vector3F ThrustSpBody = Rotation.Transpose() * ThrustSp;

		/* We dont want to make any correction in body x and y*/
		ThrustSpBody[0] = 0.0f;
		ThrustSpBody[1] = 0.0f;

		/* Make sure z component of ThrustSpBody is larger than 0 (positive thrust is downward) */
		ThrustSpBody[2] = ThrustSp[2] > 0.0f ? ThrustSp[2] : 0.0f;

		/* Convert back to local frame (NED) */
		ThrustSp = Rotation * ThrustSpBody;

		//TODO or ThrustSp.Zero();
	}

	if (!VehicleControlModeMsg.ControlClimbRateEnabled && !VehicleControlModeMsg.ControlAccelerationEnabled)
	{
		ThrustSp[2] = 0.0f;
	}

	/* Limit thrust vector and check for saturation. */
	bool SaturationXy = false;
	bool SaturationZ = false;

	/* Limit min lift */
	float ThrMin = ConfigTblPtr->THR_MIN;

	if (!VehicleControlModeMsg.ControlVelocityEnabled && ThrMin < 0.0f)
	{
		/* Don't allow downside thrust direction in manual attitude mode. */
		ThrMin = 0.0f;
	}

	float TiltMax = math::radians(ConfigTblPtr->TILTMAX_AIR);
	float ThrMax = ConfigTblPtr->THR_MAX;

	/* We can only run the control if we're already in-air, have a takeoff setpoint,
	 * or if we're in offboard control.  Otherwise, we should just bail out. */
	if (VehicleLandDetectedMsg.Landed && !InAutoTakeoff() && !ManualWantsTakeoff())
	{
		/* Keep throttle low while still on ground. */
		ThrMax = 0.0f;
	}
	else if (!VehicleControlModeMsg.ControlManualEnabled && PositionSetpointTripletMsg.Current.Valid &&
			PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LAND)
	{
		/* Adjust limits for landing mode.  Limit max tilt and min lift when landing. */
		TiltMax = math::radians(ConfigTblPtr->TILTMAX_LND);
	}

	/* Limit min lift */
	if (-ThrustSp[2] < ThrMin)
	{
		ThrustSp[2] = -ThrMin;

		/* Don't freeze altitude integral if it wants to throttle up. */
		SaturationZ = VelErr[2] > 0.0f ? true : SaturationZ;
	}

	if (VehicleControlModeMsg.ControlVelocityEnabled || VehicleControlModeMsg.ControlAccelerationEnabled)
	{
		/* Limit max tilt */
		if (ThrMin >= 0.0f && TiltMax < M_PI / 2 - 0.05f)
		{
			/* Absolute horizontal thrust */
			float ThrustSpXyLen = math::Vector2F(ThrustSp[0], ThrustSp[1]).Length();
			if (ThrustSpXyLen > 0.01f)
			{
				/* Max horizontal thrust for given vertical thrust. */
				float ThrustXyMax = -ThrustSp[2] * tanf(TiltMax);
				if (ThrustSpXyLen > ThrustXyMax)
				{
					float k = ThrustXyMax / ThrustSpXyLen;
					ThrustSp[0] *= k;
					ThrustSp[1] *= k;

					/* Don't freeze x,y integrals if they both want to throttle down. */
					SaturationXy = ((VelErr[0] * VelocitySetpoint[0] < 0.0f) && (VelErr[1] * VelocitySetpoint[1] < 0.0f)) ? SaturationXy : true;
				}
			}
		}
	}

	if (VehicleControlModeMsg.ControlClimbRateEnabled && !VehicleControlModeMsg.ControlVelocityEnabled)
	{
		/* Thrust compensation when vertical velocity but not horizontal velocity is controlled. */
		float AttComp;

		const float TiltCosMax = 0.7f;

		if (Rotation[2][2] > TiltCosMax)
		{
			AttComp = 1.0f / Rotation[2][2];
		}
		else if (Rotation[2][2] > 0.0f)
		{
			AttComp = ((1.0f / TiltCosMax - 1.0f) / TiltCosMax) * Rotation[2][2] + 1.0f;
			SaturationZ = true;
		}
		else
		{
			AttComp = 1.0f;
			SaturationZ = true;
		}

		ThrustSp[2] *= AttComp;
	}

	/* Calculate desired total thrust amount in body z direction. */
	/* To compensate for excess thrust during attitude tracking errors we
	 * project the desired thrust force vector F onto the real vehicle's thrust axis in NED:
	 * body thrust axis [0,0,-1]' rotated by R is: R*[0,0,-1]' = -R_z */
	math::Vector3F R_z(Rotation[0][2], Rotation[1][2], Rotation[2][2]);
	math::Vector3F F(ThrustSp);

	/* Recalculate because it might have changed. */
	float ThrustBodyZ = F * -R_z;

	/* Limit max thrust. */
	if (fabsf(ThrustBodyZ) > ThrMax)
	{
		if (ThrustSp[2] < 0.0f)
		{
			if (-ThrustSp[2] > ThrMax)
			{
				/* Thrust Z component is too large, limit it. */
				ThrustSp[0] = 0.0f;
				ThrustSp[1] = 0.0f;
				ThrustSp[2] = -ThrMax;
				SaturationXy = true;

				/* Don't freeze altitude integral if it wants to throttle down. */
				SaturationZ = VelErr[2] < 0.0f ? true : SaturationZ;
			}
			else
			{
				/* Preserve thrust Z component and lower XY, keeping altitude is more important than position. */
				float ThrustXyMax = sqrtf(ThrMax * ThrMax - ThrustSp[2] * ThrustSp[2]);
				float ThrustXyAbs = math::Vector2F(ThrustSp[0], ThrustSp[1]).Length();
				float k = ThrustXyMax / ThrustXyAbs;
				ThrustSp[0] *= k;
				ThrustSp[1] *= k;
				/* Don't freeze x,y integrals if they both want to throttle down */
				SaturationXy = ((VelErr[0] * VelocitySetpoint[0] < 0.0f) && (VelErr[1] * VelocitySetpoint[1] < 0.0f)) ? SaturationXy : true;
			}
		}
		else
		{
			/* Z component is positive, going down (Z is positive down in NED), simply limit thrust vector. */
			float k = ThrMax / fabsf(ThrustBodyZ);
			ThrustSp = ThrustSp * k;
			SaturationXy = true;
			SaturationZ = true;
		}

		ThrustBodyZ = ThrMax;
	}

	VehicleAttitudeSetpointMsg.Thrust = math::max(ThrustBodyZ, ThrMin);

	/* Update integrals */
	if (VehicleControlModeMsg.ControlVelocityEnabled && !SaturationXy)
	{
		ThrustInt[0] += VelErr[0] * VelI[0] * dt;
		ThrustInt[1] += VelErr[1] * VelI[1] * dt;
	}

	if (VehicleControlModeMsg.ControlClimbRateEnabled && !SaturationZ)
	{
		ThrustInt[2] += VelErr[2] * VelI[2] * dt;
	}

	/* Calculate attitude setpoint from thrust vector. */
	if (VehicleControlModeMsg.ControlVelocityEnabled || VehicleControlModeMsg.ControlAccelerationEnabled)
	{
		/* Desired BodyZ axis = -normalize(thrust_vector) */
		math::Vector3F BodyX(0.0f, 0.0f, 0.0f);
		math::Vector3F BodyY(0.0f, 0.0f, 0.0f);
		math::Vector3F BodyZ(0.0f, 0.0f, 0.0f);

		if (ThrustSp.Length() > SIGMA_NORM)
		{
			BodyZ = -ThrustSp.Normalized();
		}
		else
		{
			/* No thrust, set Z axis to safe value. */
			BodyZ.Zero();
			BodyZ[2] = 1.0f;
		}

		/* Vector of desired yaw direction in XY plane, rotated by PI/2. */
		math::Vector3F y_C(-sinf(VehicleAttitudeSetpointMsg.YawBody), cosf(VehicleAttitudeSetpointMsg.YawBody), 0.0f);

		if (fabsf(BodyZ[2]) > SIGMA_SINGLE_OP)
		{
			/* Desired BodyX axis, orthogonal to BodyZ. */
			BodyX = y_C % BodyZ;

			/* Keep nose to front while inverted upside down. */
			if (BodyZ[2] < 0.0f)
			{
				BodyX = -BodyX;
			}

			BodyX.Normalize();
		}
		else
		{
			/* Desired thrust is in XY plane, set X downside to construct
			 * correct matrix, but yaw component will not be used actually */
			BodyX.Zero();
			BodyX[2] = 1.0f;
		}

		/* Desired BodyY axis */
		BodyY = BodyZ % BodyX;

		/* Fill rotation matrix */
		for (uint32 i = 0; i < 3; i++)
		{
			RSetpoint[i][0] = BodyX[i];
			RSetpoint[i][1] = BodyY[i];
			RSetpoint[i][2] = BodyZ[i];
		}

		/* Copy quaternion setpoint to attitude setpoint topic. */
		math::Quaternion q_sp(RSetpoint);
		q_sp.copyTo(VehicleAttitudeSetpointMsg.Q_D);
		VehicleAttitudeSetpointMsg.Q_D_Valid = true;

		/* Calculate euler angles, for logging only.  Must not be used for
		 * control. */
		math::Vector3F euler = RSetpoint.ToEuler();
		VehicleAttitudeSetpointMsg.RollBody = euler[0];
		VehicleAttitudeSetpointMsg.PitchBody = euler[1];
		/* Yaw already used to construct rot matrix, but actual rotation
		 * matrix can have different yaw near singularity. */
	}
	else if (!VehicleControlModeMsg.ControlManualEnabled)
	{
		/* Autonomous altitude control without position control (failsafe
		 * landing).  Force level attitude, don't change yaw. */
		RSetpoint = math::Matrix3F3::FromEuler(0.0f, 0.0f, VehicleAttitudeSetpointMsg.YawBody);

		/* Copy quaternion setpoint to attitude setpoint topic. */
		math::Quaternion q_sp(RSetpoint);
		q_sp.copyTo(VehicleAttitudeSetpointMsg.Q_D);
		VehicleAttitudeSetpointMsg.Q_D_Valid = true;

		VehicleAttitudeSetpointMsg.RollBody = 0.0f;
		VehicleAttitudeSetpointMsg.PitchBody = 0.0f;
	}

	/* Save thrust setpoint for logging. */
	VehicleLocalPositionSetpointMsg.AccX = ThrustSp[0] * MPC_CONSTANTS_ONE_G;
	VehicleLocalPositionSetpointMsg.AccY = ThrustSp[1] * MPC_CONSTANTS_ONE_G;
	VehicleLocalPositionSetpointMsg.AccZ = ThrustSp[2] * MPC_CONSTANTS_ONE_G;

	VehicleAttitudeSetpointMsg.Timestamp = PX4LIB_GetPX4TimeUs();
}



float MPC::GetCruisingSpeedXY(void) // NEW DONE
{
	float Result = 0.0f;

	/* In missions, the user can choose cruising speed different to default. */
	Result = ((isfinite(PositionSetpointTripletMsg.Current.CruisingSpeed) &&
			!(PositionSetpointTripletMsg.Current.CruisingSpeed < 0.0f)) ?
					PositionSetpointTripletMsg.Current.CruisingSpeed : ConfigTblPtr->XY_CRUISE);

	return Result;
}



bool MPC::CrossSphereLine(const math::Vector3F &SphereC, const float SphereR,
		const math::Vector3F &LineA, const math::Vector3F &LineB, math::Vector3F &Res) // NEW DONE
{
	bool Result = false;
	math::Vector3F d;
	float CdLen = 0.0f;
	float DxLen = 0.0f;

	/* Project center of sphere on line normalized AB. */
	math::Vector3F AbNorm = LineB - LineA;

	if(AbNorm.Length() < 0.01)
	{
		Result = true;
		goto MPC_CrossSphereLine_Exit_Tag;
	}

	AbNorm.Normalize();
	d = LineA + AbNorm * ((SphereC - LineA) * AbNorm);
	CdLen = (SphereC - d).Length();

	if (SphereR > CdLen)
	{
		/* We have triangle CDX with known CD and CX = R, find DX. */
		DxLen = sqrtf(SphereR * SphereR - CdLen * CdLen);

		if ((SphereC - LineB) * AbNorm > 0.0f)
		{
			/* Target waypoint is already behind us. */
			Res = LineB;
		}
		else
		{
			/* Target is in front of us. */
			/* vector A->B on line */
			Res = d + AbNorm * DxLen;
		}

		Result = true;
	}
	else
	{
		/* Have no roots. Return D */
		/* Go directly to line */
		Res = d;

		/* Previous waypoint is still in front of us. */
		if ((SphereC - LineA) * AbNorm < 0.0f)
		{
			Res = LineA;
		}

		/* Target waypoint is already behind us. */
		if ((SphereC - LineB) * AbNorm > 0.0f)
		{
			Res = LineB;
		}

		Result = false;
	}

MPC_CrossSphereLine_Exit_Tag:
	return Result;
}

void MPC::UpdateXyPids(MPC_SetPidCmd_t* PidMsg)
{
	PosP[0] = PidMsg->PidGain;
	PosP[1] = PidMsg->PidGain;

	VelP[0] = PidMsg->PidVelP;
	VelP[1] = PidMsg->PidVelP;

	VelI[0] = PidMsg->PidVelI;
	VelI[1] = PidMsg->PidVelI;

	VelD[0] = PidMsg->PidVelD;
	VelD[1] = PidMsg->PidVelD;;
}

void MPC::UpdateZPids(MPC_SetPidCmd_t* PidMsg)
{
	PosP[2] = PidMsg->PidGain;
	VelP[2] = PidMsg->PidVelP;
	VelI[2] = PidMsg->PidVelI;
	VelD[2] = PidMsg->PidVelD;
}

void MPC::UpdateParamsFromTable(void)
{
	if(ConfigTblPtr != 0)
	{
		PosP[0] = ConfigTblPtr->XY_P;
		PosP[1] = ConfigTblPtr->XY_P;
		PosP[2] = ConfigTblPtr->Z_P;

		VelP[0] = ConfigTblPtr->XY_VEL_P;
		VelP[1] = ConfigTblPtr->XY_VEL_P;
		VelP[2] = ConfigTblPtr->Z_VEL_P;

		VelI[0] = ConfigTblPtr->XY_VEL_I;
		VelI[1] = ConfigTblPtr->XY_VEL_I;
		VelI[2] = ConfigTblPtr->Z_VEL_I;

		VelD[0] = ConfigTblPtr->XY_VEL_D;
		VelD[1] = ConfigTblPtr->XY_VEL_D;
		VelD[2] = ConfigTblPtr->Z_VEL_D;
	}
}

/**
 * Limit altitude based on several conditions
 */
void MPC::LimitAltitude(void) // NEW DONE
{
	float AltitudeAboveHome = 0.0f;
	float DeltaT = 0.0f;
	float PosZNext = 0.0f;

	if (VehicleLandDetectedMsg.AltMax < 0.0f)
	{
		/* There is no altitude limitation present */
		goto MPC_LimitAltitude_ExitTag;
	}

	AltitudeAboveHome = -(Position[2] - HomePositionMsg.Z);

	if (RunAltControl && (AltitudeAboveHome > VehicleLandDetectedMsg.AltMax))
	{
		/* We are above maximum altitude */
		PositionSetpoint[2] = -VehicleLandDetectedMsg.AltMax +  HomePositionMsg.Z;
	}
	else if (!RunAltControl && VelocitySetpoint[2] <= 0.0f)
	{
		/* We want to fly upwards: check if vehicle does not exceed altitude */

		/* Time to reach zero velocity */
		DeltaT = -Velocity[2] / ConfigTblPtr->ACC_DOWN_MAX;

		/* Predict next position based on current position, velocity, max acceleration downwards and time to reach zero velocity */
		PosZNext = Position[2] + Velocity[2] * DeltaT + 0.5f * ConfigTblPtr->ACC_DOWN_MAX * DeltaT * DeltaT;

		if (-(PosZNext - HomePositionMsg.Z) > VehicleLandDetectedMsg.AltMax) {
			/* Prevent vehicle from exceeding maximum altitude by switching back to altitude control with maximum altitude as setpoint */
			PositionSetpoint[2] = -VehicleLandDetectedMsg.AltMax + HomePositionMsg.Z;
			RunAltControl = true;
		}
	}

MPC_LimitAltitude_ExitTag:
	return;
}

void MPC::ApplyVelocitySetpointSlewRate(float dt) // NEW DONE
{
	math::Vector2F VelSpXy(VelocitySetpoint[0], VelocitySetpoint[1]);
	math::Vector2F VelSpPrevXy(VelocitySetpointPrevious[0], VelocitySetpointPrevious[1]);
	math::Vector2F AccXy = (VelSpXy - VelSpPrevXy) / dt;

	/* limit total horizontal acceleration */
	if (AccXy.Length() > AccelerationStateDependentXY)
	{
		VelSpXy = AccXy.Normalized() * AccelerationStateDependentXY * dt + VelSpPrevXy;
		VelocitySetpoint[0] = VelSpXy[0];
		VelocitySetpoint[1] = VelSpXy[1];
	}

	/* limit vertical acceleration */
	float AccZ = (VelocitySetpoint[2] - VelocitySetpointPrevious[2]) / dt;
	float MaxAccZ;

	if (VehicleControlModeMsg.ControlManualEnabled)
	{
		MaxAccZ = (AccZ < 0.0f) ? -AccelerationStateDependentZ : AccelerationStateDependentZ;
	}
	else
	{
		MaxAccZ = (AccZ < 0.0f) ? -ConfigTblPtr->ACC_UP_MAX : ConfigTblPtr->ACC_DOWN_MAX;
	}

	if (fabsf(AccZ) > fabsf(MaxAccZ))
	{
		VelocitySetpoint[2] = MaxAccZ * dt + VelocitySetpointPrevious[2];
	}
}



bool MPC::InAutoTakeoff(void) // NEW DONE
{
	bool Result = false;

	/*
	 * In auto mode, check if we do a takeoff
	 */
	if(PositionSetpointTripletMsg.Current.Valid &&
			PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_TAKEOFF)
	{
	    /* We are in takeoff mode. */
		Result = true;
	}
	else if( VehicleControlModeMsg.ControlOffboardEnabled)
	{
		/* Assume true in offboard mode. */
		Result = true;
	}

	return Result;
}

float MPC::GetVelClose(const math::Vector2F &UnitPrevToCurrent, const math::Vector2F &UnitCurrentToNext) // NEW DONE
{
	/* Minimum cruise speed when passing waypoint */
	float MinCruiseSpeed = 1.0f;
	float MiddleCruiseSpeed = ConfigTblPtr->MPC_CRUISE_90;
	bool UseLinearApproach = false;
	float Angle = 2.0f;
	float Slope = 0.0f;
	float VelClose = 0.0f;
	float a = 0.0f;
	float b = 0.0f;
	float c = 0.0f;

	/* Make sure that cruise speed is larger than minimum*/
	if ((GetCruisingSpeedXY() - MinCruiseSpeed) < SIGMA_NORM)
	{
		VelClose = GetCruisingSpeedXY();
		goto MPC_GetVelClose_ExitTag;
	}

	/* Middle cruise speed is a number between maximum cruising speed and minimum cruising speed and corresponds to speed at angle of 90degrees
	 * it needs to be always larger than minimum cruise speed */
	if ((MiddleCruiseSpeed - MinCruiseSpeed) < SIGMA_NORM)
	{
		MiddleCruiseSpeed = MinCruiseSpeed + SIGMA_NORM;
	}

	if ((GetCruisingSpeedXY() - MiddleCruiseSpeed) < SIGMA_NORM)
	{
		MiddleCruiseSpeed = (GetCruisingSpeedXY() + MinCruiseSpeed) * 0.5f;
	}

	/* If middle cruise speed is exactly in the middle, then compute
	 * vel_close linearly */
	if (((GetCruisingSpeedXY() + MinCruiseSpeed) * 0.5f) - MiddleCruiseSpeed < SIGMA_NORM)
	{
		UseLinearApproach = true;
	}

	/* Angle = cos(x) + 1.0
	 * Angle goes from 0 to 2 with 0 = large angle, 2 = small angle:   0 = PI ; 2 = PI*0 */
	if (UnitCurrentToNext.Length() > SIGMA_NORM)
	{
		Angle = UnitCurrentToNext * (UnitPrevToCurrent * -1.0f) + 1.0f;
	}

	/* Compute velocity target close to waypoint */
	if (UseLinearApproach)
	{
		/* Velocity close to target adjusted to angle vel_close = m*x+q */
		Slope = -(GetCruisingSpeedXY() - MinCruiseSpeed) / 2.0f;
		VelClose = Slope * Angle + GetCruisingSpeedXY();

	}
	else
	{
		/* Velocity close to target adjusted to angle
		 * vel_close = a *b ^x + c; where at angle = 0 -> vel_close = vel_cruise; angle = 1 -> vel_close = MiddleCruiseSpeed (this means that at 90degrees
		 * the velocity at target is MiddleCruiseSpeed);
		 * angle = 2 -> vel_close = min_cruising_speed */

		/* From maximum cruise speed, minimum cruise speed and middle cruise speed compute constants a, b and c */
		a = -((MiddleCruiseSpeed -  GetCruisingSpeedXY()) * (MiddleCruiseSpeed -  GetCruisingSpeedXY())) /
			  (2.0f * MiddleCruiseSpeed - GetCruisingSpeedXY() - MinCruiseSpeed);
		c =  GetCruisingSpeedXY() - a;
		b = (MiddleCruiseSpeed - c) / a;
		VelClose = a * powf(b, Angle) + c;
	}

MPC_GetVelClose_ExitTag:
	/* vel_close needs to be in between max and min */
	return math::constrain(VelClose, MinCruiseSpeed, GetCruisingSpeedXY());
}

void MPC::SetManualAccelerationZ(float &MaxAcceleration, const float StickZ, const float Dt) // NEW DONE
{
	/* In manual altitude control apply acceleration limit based on stick input
	 * we consider two states
	 * 1.) brake
	 * 2.) accelerate */

	/* Check if zero input stick */
	const bool IsCurrentZero = (fabsf(StickZ) <= FLT_EPSILON);

	/* Default is acceleration */
	manual_stick_input Intention = ACCELERATION;

	float Jerk = 0.0f;

	/* Check zero input stick */
	if (IsCurrentZero) {
		Intention = BRAKE;
	}

	/* Get max and min acceleration where min acceleration is just 1/5 of max acceleration */
	MaxAcceleration = (StickZ <= 0.0f) ? ConfigTblPtr->ACC_UP_MAX : ConfigTblPtr->ACC_DOWN_MAX;

	/* Update user input */
	if ((UserIntentionZ != BRAKE) && (Intention  == BRAKE)) {

		/* We start with lowest acceleration */
		AccelerationStateDependentZ = ConfigTblPtr->ACC_DOWN_MAX;

		/* Reset slew rate */
		VelocitySetpointPrevious[2] = Velocity[2];
		UserIntentionZ = BRAKE;
	}

	UserIntentionZ = Intention;

	/* Apply acceleration depending on state */
	if (UserIntentionZ == BRAKE) {

		/* Limit jerk when braking to zero */
		Jerk = (ConfigTblPtr->ACC_UP_MAX - AccelerationStateDependentZ) / Dt;

		if (Jerk > ManualJerkLimitZ) {
			AccelerationStateDependentZ = ManualJerkLimitZ * Dt + AccelerationStateDependentZ;

		} else {
			AccelerationStateDependentZ = ConfigTblPtr->ACC_UP_MAX;
		}
	}
	else if (UserIntentionZ == ACCELERATION) {
		AccelerationStateDependentZ = (MaxAcceleration - ConfigTblPtr->ACC_DOWN_MAX) * fabsf(
				StickZ) + ConfigTblPtr->ACC_DOWN_MAX;
	}
}

void MPC::SetManualAccelerationXY(math::Vector2F &StickXy, const float Dt) // NEW DONE
{

	/*
	 * In manual mode we consider four states with different acceleration handling:
	 * 1. user wants to stop
	 * 2. user wants to quickly change direction
	 * 3. user wants to accelerate
	 * 4. user wants to decelerate
	 */

	/* Get normalized stick input vector */
	math::Vector2F StickXyNorm = (StickXy.Length() > 0.0f) ? StickXy.Normalized() : StickXy;
	math::Vector2F StickXyPrevNorm = (StickInputXyPrev.Length() > 0.0f) ? StickInputXyPrev.Normalized() :
					      StickInputXyPrev;

	/* Check if stick direction and current velocity are within 60angle */
	const bool IsAligned = (StickXyNorm * StickXyPrevNorm) > 0.5f;

	/* Check if zero input stick */
	const bool IsPrevZero = (fabsf(StickInputXyPrev.Length()) <= FLT_EPSILON);
	const bool IsCurrentZero = (fabsf(StickXy.Length()) <= FLT_EPSILON);

	/* Check intentions */
	// TODO: Should this be IsPrevZero &&?
	const bool DoAcceleration = IsPrevZero || (IsAligned &&
				     ((StickXy.Length() > StickInputXyPrev.Length()) || (fabsf(StickXy.Length() - 1.0f) < FLT_EPSILON)));
	const bool DoDeceleration = (IsAligned && (StickXy.Length() <= StickInputXyPrev.Length()));
	const bool DoDirectionChange = !IsAligned;

	manual_stick_input Intention;

	if (IsCurrentZero)
	{
		/* We want to stop */
		Intention = BRAKE;
	}
	else if (DoAcceleration)
	{
		/* We do manual acceleration */
		Intention = ACCELERATION;
	}
	else if (DoDeceleration)
	{
		/* We do manual deceleration */
		Intention = DECELERATION;
	}
	else if (DoDirectionChange)
	{
		/* We have a direction change */
		Intention = DIRECTION_CHANGE;
	}
	else
	{
		/* Catch all: acceleration */
		Intention = ACCELERATION;
	}

	/* Update user intention */

	/* We always want to brake starting with slow deceleration */
	if ((UserIntentionXY != BRAKE) && (Intention  == BRAKE))
	{
		if (ConfigTblPtr->MPC_JERK_MAX > ConfigTblPtr->MPC_JERK_MIN)
		{
			ManualJerkLimitXY = (ConfigTblPtr->MPC_JERK_MAX - ConfigTblPtr->MPC_JERK_MIN) / ConfigTblPtr->MPC_VEL_MANUAL *
						sqrtf(Velocity[0] * Velocity[0] + Velocity[1] * Velocity[1]) + ConfigTblPtr->MPC_JERK_MIN;

			/* We start braking with lowest accleration */
			AccelerationStateDependentXY = ConfigTblPtr->MPC_DEC_HOR_SLOW;
		}
		else
		{
			/* Set the jerk limit large since we don't know it better*/
			ManualJerkLimitXY = 1000000.0f;

			/* At brake we use max acceleration */
			AccelerationStateDependentXY = ConfigTblPtr->ACC_HOR_MAX;
		}

		/* Reset slew rate */
		VelocitySetpointPrevious[0] = Velocity[0];
		VelocitySetpointPrevious[1] = Velocity[1];
	}

	switch (UserIntentionXY)
	{
		case BRAKE:
		{
				if (Intention != BRAKE)
				{
					UserIntentionXY = ACCELERATION;
					/* We initialize with lowest acceleration */
					AccelerationStateDependentXY = ConfigTblPtr->MPC_DEC_HOR_SLOW;
				}

				break;
		}

		case DIRECTION_CHANGE:
		{
				/* Only exit direction change if brake or aligned */
				math::Vector2F VelXy(Velocity[0], Velocity[1]);
				math::Vector2F VelXyNorm = (VelXy.Length() > 0.0f) ? VelXy.Normalized() : VelXy;
				bool StickVelAligned = (VelXyNorm * StickXyNorm > 0.0f);

				/* Update manual direction change hysteresis */
				ManualDirectionChangeHysteresis.set_state_and_update(!StickVelAligned, PX4LIB_GetPX4TimeUs());

				/* Exit direction change if one of the condition is met */
				if (Intention == BRAKE)
				{
					UserIntentionXY = Intention;
				}
				else if (StickVelAligned)
				{
					UserIntentionXY = ACCELERATION;
				}
				else if (ManualDirectionChangeHysteresis.get_state())
				{
					/* TODO: find conditions which are always continuous
					 * only if stick input is large*/
					if (StickXy.Length() > 0.6f)
					{
						AccelerationStateDependentXY = ConfigTblPtr->ACC_HOR_MAX;
					}
				}

				break;
		}

		case ACCELERATION:
		{
				UserIntentionXY = Intention;

				if (UserIntentionXY == DIRECTION_CHANGE)
				{
					VelocitySetpointPrevious[0] = Velocity[0];
					VelocitySetpointPrevious[1] = Velocity[1];
				}

				break;
		}

		case DECELERATION:
		{
				UserIntentionXY = Intention;

				if (UserIntentionXY == DIRECTION_CHANGE)
				{
					VelocitySetpointPrevious[0] = Velocity[0];
					VelocitySetpointPrevious[1] = Velocity[1];
				}

				break;
		}
	}

	/* Apply acceleration based on state */
	switch (UserIntentionXY)
	{
		case BRAKE:
		{

				/* Limit jerk when braking to zero */
				float jerk = (ConfigTblPtr->ACC_HOR_MAX - AccelerationStateDependentXY) / Dt;

				if (jerk > ManualJerkLimitXY)
				{
					AccelerationStateDependentXY = ManualJerkLimitXY * Dt + AccelerationStateDependentXY;
				}
				else
				{
					AccelerationStateDependentXY = ConfigTblPtr->ACC_HOR_MAX;
				}

				break;
		}

		case DIRECTION_CHANGE:
		{

				/* Limit acceleration linearly on stick input*/
				AccelerationStateDependentXY = (ConfigTblPtr->MPC_ACC_HOR - ConfigTblPtr->MPC_DEC_HOR_SLOW) * StickXy.Length() +
								   ConfigTblPtr->MPC_DEC_HOR_SLOW;
				break;
		}

		case ACCELERATION:
		{
				/* Limit acceleration linearly on stick input*/
				float acc_limit  = (ConfigTblPtr->MPC_ACC_HOR - ConfigTblPtr->MPC_DEC_HOR_SLOW) * StickXy.Length()
						   + ConfigTblPtr->MPC_DEC_HOR_SLOW;

				if (AccelerationStateDependentXY > acc_limit)
				{
					acc_limit = AccelerationStateDependentXY;
				}

				AccelerationStateDependentXY = acc_limit;
				break;
		}

		case DECELERATION:
		{
				AccelerationStateDependentXY = ConfigTblPtr->MPC_DEC_HOR_SLOW;
				break;
		}

		default :
			//warn_rate_limited("User intention not recognized"); TODO
			AccelerationStateDependentXY = ConfigTblPtr->ACC_HOR_MAX;
		}

	/* Update previous stick input */
	StickInputXyPrev = math::Vector2F(_filter_manual_pitch.apply(StickXy[0]),
						_filter_manual_roll.apply(StickXy[1]));


	if (StickInputXyPrev.Length() > 1.0f)
	{
		StickInputXyPrev = StickInputXyPrev.Normalized();
	}
}

bool MPC::ManualWantsTakeoff() // NEW DONE
{
	const bool ManualControlPresent = VehicleControlModeMsg.ControlManualEnabled && ManualControlSetpointMsg.Timestamp > 0;

	// Manual takeoff is triggered if the throttle stick is above 65%.
	return (ManualControlPresent && ManualControlSetpointMsg.Z > 0.65f);
}

/************************/
/*  End of File Comment */
/************************/
