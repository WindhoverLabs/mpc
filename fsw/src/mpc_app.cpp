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
		_manual_direction_change_hysteresis(false)
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

	/* Let's be safe and have the landing gear down by default. */
	VehicleAttitudeSetpointMsg.LandingGear = -1.0f;

	WasArmed = false;
	WasLanded = true;

	// NEW

	RefAltIsGlobal = false;
	_user_intention_xy = none;
	_user_intention_z = none;
	_stick_input_xy_prev.Zero();
	_vel_max_xy = 0.0f;
	_vel_max_xy = 0.0f;
	_acceleration_state_dependent_xy = 0.0f;
	_acceleration_state_dependent_z = 0.0f;
	_manual_jerk_limit_xy = 0.0f;
	_manual_jerk_limit_z = 0.0f;
	_z_derivative = 0.0f;
	//_manual_direction_change_hysteresis(false);
	_triplet_lat_lon_finite = false;
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
        /* TODO: If there's no incoming message, you can do something here, or 
         * nothing.  Note, this section is dead code only if the iBlocking arg
         * is CFE_SB_PEND_FOREVER. */
        iStatus = CFE_SUCCESS;
    }
    else if (iStatus == CFE_SB_TIME_OUT)
    {
        /* TODO: If there's no incoming message within a specified time (via the
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


void MPC::Execute(void) // Updated
{
	static uint64 t_prev = 0;

	uint64 t = PX4LIB_GetPX4TimeUs();
	float dt = t_prev != 0 ? (t - t_prev) / 1e6f : 0.004f;
	t_prev = t;

	/* Set default max velocity in xy to vel_max */
	VelMaxXY = ConfigTblPtr->XY_VEL_MAX;

	/* reset flags when landed */
	if (VehicleLandDetectedMsg.Landed) {
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

		/* also reset previous setpoints */
		YawTakeoff = Yaw;
		VelocitySetpointPrevious.Zero();
		VelocityPrevious.Zero();

		/* make sure attitude setpoint output "disables" attitude control
		 * TODO: we need a defined setpoint to do this properly especially when adjusting the mixer */
		VehicleAttitudeSetpointMsg.Thrust = 0.0f;
		VehicleAttitudeSetpointMsg.Timestamp = PX4LIB_GetPX4TimeUs();
	}

	if (!InTakeoff && VehicleLandDetectedMsg.Landed && VehicleControlModeMsg.Armed &&
		(InAutoTakeoff() || ManualWantsTakeoff())) {
		InTakeoff = true;
		// This ramp starts negative and goes to positive later because we want to
		// be as smooth as possible. If we start at 0, we alrady jump to hover throttle.
		TakeoffVelLimit = -0.5f;
	}

	else if (!VehicleControlModeMsg.Armed) {
		// If we're disarmed and for some reason were in a smooth takeoff, we reset that.
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

	if(VehicleControlModeMsg.ControlAltitudeEnabled ||
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
}

void MPC::UpdateRef(void) // Updated
{
	/* The reference point is only allowed to change when the vehicle is in standby state which is the
	normal state when the estimator origin is set. Changing reference point in flight causes large controller
	setpoint changes. Changing reference point in other arming states is untested and shoud not be performed. */
	if ((VehicleLocalPositionMsg.RefTimestamp != RefTimestamp)
	    && ((VehicleStatusMsg.ArmingState == PX4_ARMING_STATE_STANDBY)
		|| (!RefAltIsGlobal && VehicleLocalPositionMsg.Z_Global)))
	{
		double LatitudeSetpoint;
		double LongitudeSetpoint;
		float AltitudeSetpoint = 0.0f;
		uint64 currentTime;

		if(RefTimestamp != 0)
		{
			/* Calculate current position setpoint in global frame. */
			map_projection_reproject(&RefPos, PositionSetpoint[0], PositionSetpoint[1], &LatitudeSetpoint, &LongitudeSetpoint);

			/* The altitude setpoint is the reference altitude (Z up) plus the (Z down)
			 * NED setpoint, multiplied out to minus*/
			AltitudeSetpoint = RefAlt - PositionSetpoint[2];
		}

		/* Update local projection reference including altitude. */
		currentTime = PX4LIB_GetPX4TimeUs();
		map_projection_init(&RefPos, VehicleLocalPositionMsg.RefLat, VehicleLocalPositionMsg.RefLon, currentTime);
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



void MPC::UpdateVelocityDerivative(float dt) // UPDATED
{
	/* Update velocity derivative,
	 * independent of the current flight mode
	 */
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
			/* set velocity to the derivative of position
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



void MPC::DoControl(float dt) // Updated
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
		_acceleration_state_dependent_xy = ConfigTblPtr->ACC_HOR_MAX;
		_acceleration_state_dependent_z = ConfigTblPtr->ACC_UP_MAX;
		ControlNonManual(dt);
	}
}



void MPC::GenerateAttitudeSetpoint(float dt) // UPDATED
{
	// yaw setpoint is integrated over time, but we don't want to integrate the offset's
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
		const float yaw_rate_max = (ConfigTblPtr->MAN_Y_MAX < ConfigTblPtr->MC_YAWRATE_MAX) ? ConfigTblPtr->MAN_Y_MAX :
				ConfigTblPtr->MC_YAWRATE_MAX;
		const float yaw_offset_max = yaw_rate_max / ConfigTblPtr->MC_YAW_P;

		VehicleAttitudeSetpointMsg.YawSpMoveRate = ManualControlSetpointMsg.R * yaw_rate_max;

		float yaw_target = _wrap_pi(VehicleAttitudeSetpointMsg.YawBody + VehicleAttitudeSetpointMsg.YawSpMoveRate * dt);
		float yaw_offs = _wrap_pi(yaw_target - Yaw);

		/* If the yaw offset became too big for the system to track stop
         * shifting it, only allow if it would make the offset smaller again. */
		if (fabsf(yaw_offs) < yaw_offset_max ||
		    (VehicleAttitudeSetpointMsg.YawSpMoveRate > 0 && yaw_offs < 0) ||
		    (VehicleAttitudeSetpointMsg.YawSpMoveRate < 0 && yaw_offs > 0))
		{
			VehicleAttitudeSetpointMsg.YawBody = yaw_target;
		}
	}

	/* Control throttle directly if no climb rate controller is active */
	if (!VehicleControlModeMsg.ControlClimbRateEnabled)
	{
		float thr_val = ThrottleCurve(ManualControlSetpointMsg.Z, ConfigTblPtr->THR_HOVER);

	    VehicleAttitudeSetpointMsg.Thrust = fmin(thr_val, ConfigTblPtr->MANTHR_MAX);

		/* Enforce minimum throttle if not landed */
		if (!VehicleLandDetectedMsg.Landed)
		{
			VehicleAttitudeSetpointMsg.Thrust = fmax(VehicleAttitudeSetpointMsg.Thrust, ConfigTblPtr->MANTHR_MIN);
		}
	}

	/* Control roll and pitch directly if no aiding velocity controller is active. */
	if (!VehicleControlModeMsg.ControlVelocityEnabled)
	{
		/*
		 * Input mapping for roll & pitch setpoints
		 * ----------------------------------------
		 * This simplest thing to do is map the y & x inputs directly to roll and pitch, and scale according to the max
		 * tilt angle.
		 * But this has several issues:
		 * - The maximum tilt angle cannot easily be restricted. By limiting the roll and pitch separately,
		 *   it would be possible to get to a higher tilt angle by combining roll and pitch (the difference is
		 *   around 15 degrees maximum, so quite noticeable). Limiting this angle is not simple in roll-pitch-space,
		 *   it requires to limit the tilt angle = acos(cos(roll) * cos(pitch)) in a meaningful way (eg. scaling both
		 *   roll and pitch).
		 * - Moving the stick diagonally, such that |x| = |y|, does not move the vehicle towards a 45 degrees angle.
		 *   The direction angle towards the max tilt in the XY-plane is atan(1/cos(x)). Which means it even depends
		 *   on the tilt angle (for a tilt angle of 35 degrees, it's off by about 5 degrees).
		 *
		 * So instead we control the following 2 angles:
		 * - tilt angle, given by sqrt(x*x + y*y)
		 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
		 *
		 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
		 * points to, and changes of the stick input are linear.
		 */
		const float x = ManualControlSetpointMsg.X * ConfigTblPtr->MAN_TILT_MAX;
		const float y = ManualControlSetpointMsg.Y * ConfigTblPtr->MAN_TILT_MAX;

		// we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
		math::Vector2F v = math::Vector2F(y, -x);
		float v_norm = v.Length();// the norm of v defines the tilt angle. Same as length()

		if (v_norm > ConfigTblPtr->MAN_TILT_MAX) { // limit to the configured maximum tilt angle
			v = v * ConfigTblPtr->MAN_TILT_MAX / v_norm;
		}

		math::Quaternion q_sp_rpy = math::Vector3F(v[0], v[1], 0.0f);// = AxisAngle(v[0], v[1], 0.0f);
		// The axis angle can change the yaw as well (but only at higher tilt angles. Note: we're talking
		// about the world frame here, in terms of body frame the yaw rate will be unaffected).
		// This the the formula by how much the yaw changes:
		//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
		//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
		math::Euler euler_sp = math::Dcm(q_sp_rpy);
		// Since the yaw setpoint is integrated, we extract the offset here,
		// so that we can remove it before the next iteration
		_man_yaw_offset = euler_sp[2];

		// update the setpoints
		VehicleAttitudeSetpointMsg.RollBody = euler_sp[0];
		VehicleAttitudeSetpointMsg.PitchBody = euler_sp[1];
		VehicleAttitudeSetpointMsg.YawBody += euler_sp[2];

		/* only if optimal recovery is not used, modify roll/pitch */
		if (!(VehicleStatusMsg.IsVtol && ConfigTblPtr->VT_OPT_RECOV_EN)) {
			// construct attitude setpoint rotation matrix. modify the setpoints for roll
			// and pitch such that they reflect the user's intention even if a yaw error
			// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
			// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
			// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
			// heading of the vehicle.
			// The effect of that can be seen with:
			// - roll/pitch into one direction, keep it fixed (at high angle)
			// - apply a fast yaw rotation
			// - look at the roll and pitch angles: they should stay pretty much the same as when not yawing

			// calculate our current yaw error
			float yaw_error = _wrap_pi(VehicleAttitudeSetpointMsg.YawBody - Yaw);

			// compute the vector obtained by rotating a z unit vector by the rotation
			// given by the roll and pitch commands of the user
			math::Vector3F zB = {0, 0, 1};
			math::Matrix3F3 R_sp_roll_pitch;
			R_sp_roll_pitch.FromEuler(VehicleAttitudeSetpointMsg.RollBody, VehicleAttitudeSetpointMsg.PitchBody, 0);
			math::Vector3F z_roll_pitch_sp = R_sp_roll_pitch * zB;


			// transform the vector into a new frame which is rotated around the z axis
			// by the current yaw error. this vector defines the desired tilt when we look
			// into the direction of the desired heading
			math::Matrix3F3 R_yaw_correction;
			R_yaw_correction.FromEuler(0.0f, 0.0f, -yaw_error);
			z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

			// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
			// R_tilt is computed from_euler; only true if cos(roll) not equal zero
			// -> valid if roll is not +-pi/2;
			VehicleAttitudeSetpointMsg.RollBody = -asinf(z_roll_pitch_sp[1]);
			VehicleAttitudeSetpointMsg.PitchBody = atan2f(z_roll_pitch_sp[0], z_roll_pitch_sp[2]);
		}

		/* copy quaternion setpoint to attitude setpoint topic */
		math::Quaternion q_sp(VehicleAttitudeSetpointMsg.RollBody, VehicleAttitudeSetpointMsg.PitchBody, VehicleAttitudeSetpointMsg.YawBody);
		q_sp.copyTo(VehicleAttitudeSetpointMsg.Q_D);
		VehicleAttitudeSetpointMsg.Q_D_Valid = true;
	}

	VehicleAttitudeSetpointMsg.Timestamp = PX4LIB_GetPX4TimeUs();
}



void MPC::ControlManual(float dt) //UPDATED
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
	math::Vector3F man_vel_sp(0.0f, 0.0f, 0.0f);

	if(VehicleControlModeMsg.ControlAltitudeEnabled)
	{
		/* Set vertical velocity setpoint with throttle stick, remapping of
		 * manual.z [0,1] to up and down command [-1,1] */
		man_vel_sp[2] = -math::expof_deadzone(
				(ManualControlSetpointMsg.Z - 0.5f) * 2.0f,
				ConfigTblPtr->Z_MAN_EXPO, ConfigTblPtr->HOLD_DZ);

		/* Reset alt setpoint to current altitude if needed. */
		ResetAltSetpoint();
	}

	if (VehicleControlModeMsg.ControlPositionEnabled)
	{
		/* Set horizontal velocity setpoint with roll/pitch stick */
		man_vel_sp[0] = math::expof_deadzone(
				ManualControlSetpointMsg.X,
				ConfigTblPtr->XY_MAN_EXPO, ConfigTblPtr->HOLD_DZ);
		man_vel_sp[1] = math::expof_deadzone(
				ManualControlSetpointMsg.Y,
				ConfigTblPtr->XY_MAN_EXPO, ConfigTblPtr->HOLD_DZ);

		const float man_vel_hor_length = math::Vector2F(man_vel_sp[0], man_vel_sp[1]).Length();

		/* saturate such that magnitude is never larger than 1 */
		if (man_vel_hor_length > 1.0f) {
			man_vel_sp[0] /= man_vel_hor_length;
			man_vel_sp[1] /= man_vel_hor_length;
		}

		/* Reset position setpoint to current position if needed */
		ResetPosSetpoint();
	}

	/* Prepare yaw to rotate into NED frame */
	float yaw_input_frame = VehicleControlModeMsg.ControlFixedHdgEnabled ? YawTakeoff : VehicleAttitudeSetpointMsg.YawBody;

	/* Setpoint in NED frame and scaled to cruise velocity */
	man_vel_sp = math::Matrix3F3::FromEuler(0.0f, 0.0f, yaw_input_frame) * man_vel_sp;

	/* adjust acceleration based on stick input */
	math::Vector2F stick_xy(man_vel_sp[0], man_vel_sp[1]);
	SetManualAccelerationXY(stick_xy, dt);
	float stick_z = man_vel_sp[2];
	float max_acc_z;
	SetManualAccelerationZ(max_acc_z, stick_z, dt);

	/* Prepare cruise speed (m/s) vector to scale the velocity setpoint */
	float vel_mag = (ConfigTblPtr->MPC_VEL_MANUAL < VelMaxXY) ? ConfigTblPtr->MPC_VEL_MANUAL : VelMaxXY;
	math::Vector3F vel_cruise_scale(vel_mag, vel_mag, (man_vel_sp[2] > 0.0f) ? ConfigTblPtr->Z_VEL_MAX_DN : ConfigTblPtr->Z_VEL_MAX_UP);

	/* Setpoint scaled to cruise speed */
	man_vel_sp = man_vel_sp.EMult(vel_cruise_scale);

	/*
	 * Assisted velocity mode: User controls velocity, but if velocity is small enough, position
	 * hold is activated for the corresponding axis.
	 */

	/* Want to get/stay in altitude hold if user has z stick in the middle (accounted for deadzone already) */
	const bool alt_hold_desired = VehicleControlModeMsg.ControlAltitudeEnabled && (_user_intention_z == brake);

	/* Want to get/stay in position hold if user has xy stick in the middle (accounted for deadzone already) */
	const bool pos_hold_desired = VehicleControlModeMsg.ControlPositionEnabled && (_user_intention_xy ==  brake);

	/* Check vertical hold engaged flag. */
	if (AltitudeHoldEngaged)
	{
		AltitudeHoldEngaged = alt_hold_desired;
	}
	else
	{
		/* Check if we switch to alt_hold_engaged. */
		bool smooth_alt_transition = alt_hold_desired && ((max_acc_z - _acceleration_state_dependent_z) < FLT_EPSILON) &&
							     (ConfigTblPtr->HOLD_MAX_Z < FLT_EPSILON || fabsf(Velocity[2]) < ConfigTblPtr->HOLD_MAX_Z);

		/* During transition predict setpoint forward. */
		if (smooth_alt_transition)
		{
			/* Time to travel from current velocity to zero velocity. */
			float delta_t = fabsf(Velocity[2] / max_acc_z);

			/* Set desired position setpoint assuming max acceleraiton. */
			PositionSetpoint[2] = Position[2] + Velocity[2] * delta_t + 0.5f * max_acc_z * delta_t * delta_t;

			AltitudeHoldEngaged = true;
		}
	}

	/* Check horizontal hold engaged flag. */
	if (PositionHoldEngaged)
	{
		PositionHoldEngaged = pos_hold_desired;

		/* use max acceleration */
		if (PositionHoldEngaged) {
			_acceleration_state_dependent_xy = ConfigTblPtr->ACC_HOR_MAX;
		}
	}
	else
	{
		/* Check if we switch to pos_hold_engaged. */
		float vel_xy_mag = sqrtf(Velocity[0] * Velocity[0] + Velocity[1] * Velocity[1]);
		bool smooth_pos_transition = pos_hold_desired
							     && (fabsf(ConfigTblPtr->ACC_HOR_MAX - _acceleration_state_dependent_xy) < FLT_EPSILON) &&
							     (ConfigTblPtr->HOLD_MAX_XY < FLT_EPSILON || vel_xy_mag < ConfigTblPtr->HOLD_MAX_XY);

		/* During transition predict setpoint forward. */
		if (smooth_pos_transition)
		{
			/* Time to travel from current velocity to zero velocity. */
			float delta_t = sqrtf(Velocity[0] * Velocity[0] + Velocity[1] * Velocity[1]) / ConfigTblPtr->ACC_HOR_MAX;

			/* p pos_sp in xy from max acceleration and current velocity */
			math::Vector2F pos(Position[0], Position[1]);
			math::Vector2F vel(Velocity[0], Velocity[1]);
			math::Vector2F pos_sp = pos + vel * delta_t - vel.Normalized() * 0.5f * ConfigTblPtr->ACC_HOR_MAX * delta_t * delta_t;
			PositionSetpoint[0] = pos_sp[0];
			PositionSetpoint[1] = pos_sp[1];

			PositionHoldEngaged = true;
		}
	}

	/* Set requested velocity setpoints */
	if (!AltitudeHoldEngaged)
	{
		PositionSetpoint[2] = Position[2];
		/* Request velocity setpoint to be used, instead of altitude setpoint */
		RunAltControl = false;
		VelocitySetpoint[2] = man_vel_sp[2];
	}

	if (!PositionHoldEngaged)
	{
		PositionSetpoint[0] = Position[0];
		PositionSetpoint[1] = Position[1];
		/* Request velocity setpoint to be used, instead of position setpoint */
		RunPosControl = false;
		VelocitySetpoint[0] = man_vel_sp[0];
		VelocitySetpoint[1] = man_vel_sp[1];
	}

	ControlPosition(dt);
}



void MPC::ControlNonManual(float dt) // UPDATED
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
	bool velocity_valid = isfinite(PositionSetpointTripletMsg.Current.VX) &&
			isfinite(PositionSetpointTripletMsg.Current.VY) &&
			PositionSetpointTripletMsg.Current.VelocityValid;

	/* Do not go slower than the follow target velocity when position tracking
	 * is active (set to valid)
	 */
	if (PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_FOLLOW_TARGET &&
	    velocity_valid &&
		PositionSetpointTripletMsg.Current.PositionValid)
	{
		math::Vector3F ft_vel(PositionSetpointTripletMsg.Current.VX, PositionSetpointTripletMsg.Current.VY, 0);
		float cos_ratio = (ft_vel * VelocitySetpoint) / (ft_vel.Length() * VelocitySetpoint.Length());

		/* Only override velocity set points when uav is traveling in same
		 * direction as target and vector component is greater than
		 * calculated position set point velocity component.
		 */
		if (cos_ratio > 0)
		{
			ft_vel = ft_vel * cos_ratio;
			/* Min speed a little faster than target vel. */
			ft_vel = ft_vel + ft_vel.Normalized() * 1.5f;

		}
		else
		{
			ft_vel.Zero();
		}

		VelocitySetpoint[0] = fabsf(ft_vel[0]) > fabsf(VelocitySetpoint[0]) ? ft_vel[0] : VelocitySetpoint[0];
		VelocitySetpoint[1] = fabsf(ft_vel[1]) > fabsf(VelocitySetpoint[1]) ? ft_vel[1] : VelocitySetpoint[1];

		/* Track target using velocity only. */

	}
	else if(PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_FOLLOW_TARGET &&
		   velocity_valid)
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
		math::Quaternion qd(RSetpoint);
		qd.copyTo(VehicleAttitudeSetpointMsg.Q_D);
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



float MPC::ThrottleCurve(float ctl, float ctr) // GOOD
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



void MPC::ResetPosSetpoint(void) // DONE
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



void MPC::ResetAltSetpoint(void) // DONE
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



void MPC::ControlPosition(float dt) // Updated
{
	CalculateVelocitySetpoint(dt);

	if (VehicleControlModeMsg.ControlClimbRateEnabled || VehicleControlModeMsg.ControlVelocityEnabled ||
			VehicleControlModeMsg.ControlAccelerationEnabled)
	{
		CalculateThrustSetpoint(dt);
	}
	else
	{
		ResetIntZ = true;
	}
}



void MPC::ControlOffboard(float dt) // UPDATED
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
			float yaw_target = _wrap_pi(VehicleAttitudeSetpointMsg.YawBody + PositionSetpointTripletMsg.Current.Yawspeed * dt);
						float yaw_offs = _wrap_pi(yaw_target - Yaw);
						const float yaw_rate_max = (ConfigTblPtr->MAN_Y_MAX < ConfigTblPtr->MC_YAWRATE_MAX) ? ConfigTblPtr->MAN_Y_MAX :
								ConfigTblPtr->MC_YAWRATE_MAX;
						const float yaw_offset_max = yaw_rate_max / ConfigTblPtr->MC_YAW_P;

						// If the yaw offset became too big for the system to track stop
						// shifting it, only allow if it would make the offset smaller again.
						if (fabsf(yaw_offs) < yaw_offset_max ||
						    (PositionSetpointTripletMsg.Current.Yawspeed > 0 && yaw_offs < 0) ||
						    (PositionSetpointTripletMsg.Current.Yawspeed < 0 && yaw_offs > 0)) {
							VehicleAttitudeSetpointMsg.YawBody = yaw_target;
						}

			VehicleAttitudeSetpointMsg.YawBody = VehicleAttitudeSetpointMsg.YawBody + PositionSetpointTripletMsg.Current.Yawspeed * dt;
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

void MPC::ControlAuto(float dt) // DONE
{

	/* Reset position setpoint on AUTO mode activation or if we are not in
	 * MC mode */
	if (!ModeAuto || !VehicleStatusMsg.IsRotaryWing)
	{
		if (!ModeAuto)
		{
			ModeAuto = true;
			_triplet_lat_lon_finite = true;
		}

		ResetPositionSetpoint = true;
		ResetAltitudeSetpoint = true;
	}

	/* Always check reset state of altitude and position control flags in auto. */
	ResetPosSetpoint();
	ResetAltSetpoint();

	bool current_setpoint_valid = false;
	bool previous_setpoint_valid = false;
	bool next_setpoint_valid = false;
	bool triplet_updated = false;

	math::Vector3F prev_sp;
	math::Vector3F next_sp;

	if (PositionSetpointTripletMsg.Current.Valid)
	{
		math::Vector3F curr_pos_sp = CurrentPositionSetpoint;

		/* Only project setpoints if they are finite, else use current
		 * position. */
		if (isfinite(PositionSetpointTripletMsg.Current.Lat) &&
		    isfinite(PositionSetpointTripletMsg.Current.Lon))
		{
			/* Project setpoint to local frame. */
			map_projection_project(&RefPos,
					PositionSetpointTripletMsg.Current.Lat, PositionSetpointTripletMsg.Current.Lon,
					       &CurrentPositionSetpoint[0], &CurrentPositionSetpoint[1]);

			_triplet_lat_lon_finite = true;
		}
		/* use current position if NAN -> e.g. land */
		else
		{
			if (_triplet_lat_lon_finite)
			{
				CurrentPositionSetpoint[0] = Position[0];
				CurrentPositionSetpoint[1] = Position[1];
				_triplet_lat_lon_finite = false;
			}
		}

		/* Only project setpoints if they are finite, else use current position. */
		if (isfinite(PositionSetpointTripletMsg.Current.Alt))
		{
			CurrentPositionSetpoint[2] = -(PositionSetpointTripletMsg.Current.Alt - RefAlt);
		}

		/* sanity check */
		if (isfinite(CurrentPositionSetpoint[0]) &&
			isfinite(CurrentPositionSetpoint[1]) &&
			isfinite(CurrentPositionSetpoint[2]))
		{
			current_setpoint_valid = true;
		}

		/* check if triplets have been updated
		 * note: we only can look at xy since navigator applies slewrate to z */
		float  diff;

		if (_triplet_lat_lon_finite) {
			diff = math::Vector2F((CurrentPositionSetpoint[0] - curr_pos_sp[0]), (CurrentPositionSetpoint[1] - curr_pos_sp[1])).Length();

		} else {
			diff = fabsf(CurrentPositionSetpoint[2] - curr_pos_sp[2]);
		}

		if (diff > FLT_EPSILON || !isfinite(diff)) {
			triplet_updated = true;
		}

		/* we need to update _curr_pos_sp always since navigator applies slew rate on z */
		CurrentPositionSetpoint = curr_pos_sp;
	}

	if (PositionSetpointTripletMsg.Previous.Valid)
	{
		map_projection_project(&RefPos,
				PositionSetpointTripletMsg.Previous.Lat, PositionSetpointTripletMsg.Previous.Lon,
				       &prev_sp[0], &prev_sp[1]);
		prev_sp[2] = -(PositionSetpointTripletMsg.Previous.Alt - RefAlt);

		if (isfinite(prev_sp[0]) &&
			isfinite(prev_sp[1]) &&
			isfinite(prev_sp[2]))
		{
			PreviousPositionSetpoint = prev_sp;
			previous_setpoint_valid = true;
		}
	}

	/* set previous setpoint to current position if no previous setpoint available */
	if (!previous_setpoint_valid && triplet_updated) {
		PreviousPositionSetpoint = Position;
		previous_setpoint_valid = true; /* currrently not necessary to set to true since not used*/
	}

	if (PositionSetpointTripletMsg.Next.Valid)
	{
		map_projection_project(&RefPos,
				PositionSetpointTripletMsg.Next.Lat, PositionSetpointTripletMsg.Next.Lon,
				       &next_sp[0], &next_sp[1]);
		next_sp[2] = -(PositionSetpointTripletMsg.Next.Alt - RefAlt);

		if (isfinite(next_sp[0]) &&
				isfinite(next_sp[1]) &&
				isfinite(next_sp[2]))
		{
			next_setpoint_valid = true;
		}
	}

	/* Auto logic:
	 * The vehicle should follow the line previous-current.
	 * - if there is no next setpoint or the current is a loiter point, then slowly approach the current along the line
	 * - if there is a next setpoint, then the velocity is adjusted depending on the angle of the corner prev-current-next.
	 * When following the line, the pos_sp is computed from the orthogonal distance to the closest point on line and the desired cruise speed along the track.
	 */

	/* create new _pos_sp from triplets */
	if (current_setpoint_valid &&
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

		float yaw_diff = _wrap_pi(VehicleAttitudeSetpointMsg.YawBody - Yaw);

		/* only follow previous-current-line for specific triplet type */
		if (PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_POSITION  ||
			PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER ||
			PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_FOLLOW_TARGET)
		{
			/* By default, use current setpoint as is. */
			math::Vector3F pos_sp = CurrentPositionSetpoint;

			/*
			 * Z-DIRECTION
			 */

			/* get various distances */
			float total_dist_z = fabsf(CurrentPositionSetpoint[2] - PreviousPositionSetpoint[2]);
			float dist_to_prev_z = fabsf(Position[2] - PreviousPositionSetpoint[2]);
			float dist_to_current_z = fabsf(CurrentPositionSetpoint[2] - Position[2]);

			/* if pos_sp has not reached target setpoint (=currPositionSetpoint[2]),
			 * then compute setpoint depending on vel_max */
			if ((total_dist_z >  SIGMA_NORM) && (fabsf(PositionSetpoint[2] - CurrentPositionSetpoint[2]) > SIGMA_NORM)) {

				/* check sign */
				bool flying_upward = CurrentPositionSetpoint[2] < Position[2];

				/* final_vel_z is the max velocity which depends on the distance of total_dist_z
				 * with default params.vel_max_up/down
				 */
				float final_vel_z = (flying_upward) ? ConfigTblPtr->Z_VEL_MAX_UP : ConfigTblPtr->Z_VEL_MAX_DN;

				/* target threshold defines the distance to CurrentPositionSetpoint[2] at which
				 * the vehicle starts to slow down to approach the target smoothly
				 */
				float target_threshold_z = final_vel_z * 1.5f;

				/* if the total distance in z is NOT 2x distance of target_threshold, we
				 * will need to adjust the final_vel_z
				 */
				bool is_2_target_threshold_z = total_dist_z >= 2.0f * target_threshold_z;
				float slope = (final_vel_z) / (target_threshold_z); /* defines the the acceleration when slowing down */
				float min_vel_z = 0.2f; // minimum velocity: this is needed since estimation is not perfect

				if (!is_2_target_threshold_z) {
					/* adjust final_vel_z since we are already very close
					 * to current and therefore it is not necessary to accelerate
					 * up to full speed (=final_vel_z)
					 */
					target_threshold_z = total_dist_z * 0.5f;
					/* get the velocity at target_threshold_z */
					float final_vel_z_tmp = slope * (target_threshold_z) + min_vel_z;

					/* make sure that final_vel_z is never smaller than 0.5 of the default final_vel_z
					 * this is mainly done because the estimation in z is not perfect and therefore
					 * it is necessary to have a minimum speed
					 */
					final_vel_z = math::constrain(final_vel_z_tmp, final_vel_z * 0.5f, final_vel_z);
				}

				float vel_sp_z = final_vel_z;

				/* we want to slow down */
				if (dist_to_current_z < target_threshold_z) {

					vel_sp_z = slope * dist_to_current_z + min_vel_z;

				} else if (dist_to_prev_z < target_threshold_z) {
					/* we want to accelerate */

					float acc_z = (vel_sp_z - fabsf(VelocitySetpoint[2])) / dt;
					float acc_max = (flying_upward) ? (ConfigTblPtr->ACC_UP_MAX * 0.5f) : (ConfigTblPtr->ACC_DOWN_MAX * 0.5f);

					if (acc_z > acc_max) {
						vel_sp_z = ConfigTblPtr->ACC_UP_MAX * dt + fabsf(VelocitySetpoint[2]);
					}

				}

				/* if we already close to current, then just take over the velocity that
				 * we would have computed if going directly to the current setpoint
				 */
				if (vel_sp_z >= (dist_to_current_z * PosP[2])) {
					vel_sp_z = dist_to_current_z * PosP[2];
				}

				/* make sure vel_sp_z is always positive */
				vel_sp_z = math::constrain(vel_sp_z, 0.0f, final_vel_z);
				/* get the sign of vel_sp_z */
				vel_sp_z = (flying_upward) ? -vel_sp_z : vel_sp_z;
				/* compute pos_sp[2] */
				pos_sp[2] = Position[2] + vel_sp_z / PosP[2];
			}

			/*
			 * XY-DIRECTION
			 */

			/* line from previous to current and from pos to current */
			math::Vector2F vec_prev_to_current((CurrentPositionSetpoint[0] - PreviousPositionSetpoint[0]), (CurrentPositionSetpoint[1] - PreviousPositionSetpoint[1]));
			math::Vector2F vec_pos_to_current((CurrentPositionSetpoint[0] - Position[0]), (CurrentPositionSetpoint[1] - Position[1]));


			/* check if we just want to stay at current position */
			math::Vector2F pos_sp_diff((CurrentPositionSetpoint[0] - PositionSetpoint[0]), (CurrentPositionSetpoint[1] - PositionSetpoint[1]));
			bool stay_at_current_pos = (PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER
							|| !next_setpoint_valid)
						   && ((pos_sp_diff.Length()) < SIGMA_NORM);

			/* only follow line if previous to current has a minimum distance */
			if ((vec_prev_to_current.Length()  > ConfigTblPtr->NAV_ACC_RAD) && !stay_at_current_pos) {

				/* normalize prev-current line (always > nav_rad) */
				math::Vector2F unit_prev_to_current = vec_prev_to_current.Normalized();

				/* unit vector from current to next */
				math::Vector2F unit_current_to_next(0.0f, 0.0f);

				if (next_setpoint_valid) {
					unit_current_to_next = math::Vector2F((next_sp[0] - pos_sp[0]), (next_sp[1] - pos_sp[1]));
					unit_current_to_next = (unit_current_to_next.Length() > SIGMA_NORM) ? unit_current_to_next.Normalized() :
								   unit_current_to_next;
				}

				/* point on line closest to pos */
				math::Vector2F closest_point = math::Vector2F(PreviousPositionSetpoint[0], PreviousPositionSetpoint[1]) + unit_prev_to_current *
								 (math::Vector2F((Position[0] - PreviousPositionSetpoint[0]), (Position[1] - PreviousPositionSetpoint[1])) * unit_prev_to_current);

				math::Vector2F vec_closest_to_current((CurrentPositionSetpoint[0] - closest_point[0]), (CurrentPositionSetpoint[1] - closest_point[1]));

				/* compute vector from position-current and previous-position */
				math::Vector2F vec_prev_to_pos((Position[0] - PreviousPositionSetpoint[0]), (Position[1] - PreviousPositionSetpoint[1]));

				/* current velocity along track */
				float vel_sp_along_track_prev = math::Vector2F(VelocitySetpoint[0], VelocitySetpoint[1]) * unit_prev_to_current;

				/* distance to target when brake should occur */
				float target_threshold_xy = 1.5f * GetCruisingSpeedXY();

				bool close_to_current = vec_pos_to_current.Length() < target_threshold_xy;
				bool close_to_prev = (vec_prev_to_pos.Length() < target_threshold_xy) &&
							 (vec_prev_to_pos.Length() < vec_pos_to_current.Length());

				/* indicates if we are at least half the distance from previous to current close to previous */
				bool is_2_target_threshold = vec_prev_to_current.Length() >= 2.0f * target_threshold_xy;

				/* check if the current setpoint is behind */
				bool current_behind = ((vec_pos_to_current * -1.0f) * unit_prev_to_current) > 0.0f;

				/* check if the previous is in front */
				bool previous_in_front = (vec_prev_to_pos * unit_prev_to_current) < 0.0f;

				/* default velocity along line prev-current */
				float vel_sp_along_track = GetCruisingSpeedXY();

				/*
				 * compute velocity setpoint along track
				 */

				/* only go directly to previous setpoint if more than 5m away and previous in front*/
				if (previous_in_front && (vec_prev_to_pos.Length() > 5.0f)) {

					/* just use the default velocity along track */
					vel_sp_along_track = vec_prev_to_pos.Length() * PosP[0];

					if (vel_sp_along_track > GetCruisingSpeedXY()) {
						vel_sp_along_track = GetCruisingSpeedXY();
					}

				} else if (current_behind) {
					/* go directly to current setpoint */
					vel_sp_along_track = vec_pos_to_current.Length() * PosP[0];
					vel_sp_along_track = (vel_sp_along_track < GetCruisingSpeedXY()) ? vel_sp_along_track : GetCruisingSpeedXY();

				} else if (close_to_prev) {
					/* accelerate from previous setpoint towards current setpoint */

					/* we are close to previous and current setpoint
					 * we first compute the start velocity when close to current septoint and use
					 * this velocity as final velocity when transition occurs from acceleration to deceleration.
					 * This ensures smooth transition */
					float final_cruise_speed = GetCruisingSpeedXY();

					if (!is_2_target_threshold) {

						/* set target threshold to half dist pre-current */
						float target_threshold_tmp = target_threshold_xy;
						target_threshold_xy = vec_prev_to_current.Length() * 0.5f;

						if ((target_threshold_xy - ConfigTblPtr->NAV_ACC_RAD) < SIGMA_NORM) {
							target_threshold_xy = ConfigTblPtr->NAV_ACC_RAD;
						}

						/* velocity close to current setpoint with default zero if no next setpoint is available */
						float vel_close = 0.0f;
						float acceptance_radius = 0.0f;

						/* we want to pass and need to compute the desired velocity close to current setpoint */
						if (next_setpoint_valid &&  !(PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER)) {
							/* get velocity close to current that depends on angle between prev-current and current-next line */
							vel_close = GetVelClose(unit_prev_to_current, unit_current_to_next);
							acceptance_radius = ConfigTblPtr->NAV_ACC_RAD;
						}

						/* compute velocity at transition where vehicle switches from acceleration to deceleration */
						if ((target_threshold_tmp - acceptance_radius) < SIGMA_NORM) {
							final_cruise_speed = vel_close;

						} else {
							float slope = (GetCruisingSpeedXY() - vel_close) / (target_threshold_tmp - acceptance_radius);
							final_cruise_speed = slope  * (target_threshold_xy - acceptance_radius) + vel_close;
							final_cruise_speed = (final_cruise_speed > vel_close) ? final_cruise_speed : vel_close;
						}
					}

					/* make sure final cruise speed is larger than 0*/
					final_cruise_speed = (final_cruise_speed > SIGMA_NORM) ? final_cruise_speed : SIGMA_NORM;
					vel_sp_along_track = final_cruise_speed;

					/* we want to accelerate not too fast
					* TODO: change the name acceleration_hor_man to something that can
					* be used by auto and manual */
					float acc_track = (final_cruise_speed - vel_sp_along_track_prev) / dt;

					/* if yaw offset is large, only accelerate with 0.5m/s^2 */
					float acc = (fabsf(yaw_diff) >  math::radians(ConfigTblPtr->NAV_MIS_YAW_ERR)) ? 0.5f : ConfigTblPtr->MPC_ACC_HOR;

					if (acc_track > acc) {
						vel_sp_along_track = acc * dt + vel_sp_along_track_prev;
					}

					/* enforce minimum cruise speed */
					vel_sp_along_track  = math::constrain(vel_sp_along_track, SIGMA_NORM, final_cruise_speed);

				} else if (close_to_current) {
					/* slow down when close to current setpoint */

					/* check if altidue is within acceptance radius */
					bool reached_altitude = (dist_to_current_z < ConfigTblPtr->NAV_ACC_RAD) ? true : false;

					if (reached_altitude && next_setpoint_valid
						&& !(PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER)) {
						/* since we have a next setpoint use the angle prev-current-next to compute velocity setpoint limit */

						/* get velocity close to current that depends on angle between prev-current and current-next line */
						float vel_close = GetVelClose(unit_prev_to_current, unit_current_to_next);

						/* compute velocity along line which depends on distance to current setpoint */
						if (vec_closest_to_current.Length() < ConfigTblPtr->NAV_ACC_RAD) {
							vel_sp_along_track = vel_close;

						} else {

							if (target_threshold_xy - ConfigTblPtr->NAV_ACC_RAD < SIGMA_NORM) {
								vel_sp_along_track = vel_close;

							} else {
								float slope = (GetCruisingSpeedXY() - vel_close) / (target_threshold_xy - ConfigTblPtr->NAV_ACC_RAD) ;
								vel_sp_along_track = slope  * (vec_closest_to_current.Length() - ConfigTblPtr->NAV_ACC_RAD) + vel_close;
							}
						}

						/* since we want to slow down take over previous velocity setpoint along track if it was lower */
						if ((vel_sp_along_track_prev < vel_sp_along_track) && (vel_sp_along_track * vel_sp_along_track_prev > 0.0f)) {
							vel_sp_along_track = vel_sp_along_track_prev;
						}

						/* if we are close to target and the previous velocity setpoints was smaller than
						 * vel_sp_along_track, then take over the previous one
						 * this ensures smoothness since we anyway want to slow down
						 */
						if ((vel_sp_along_track_prev < vel_sp_along_track) && (vel_sp_along_track * vel_sp_along_track_prev > 0.0f)
							&& (vel_sp_along_track_prev > vel_close)) {
							vel_sp_along_track = vel_sp_along_track_prev;
						}

						/* make sure that vel_sp_along track is at least min */
						vel_sp_along_track = (vel_sp_along_track < vel_close) ? vel_close : vel_sp_along_track;


					} else {

						/* we want to stop at current setpoint */
						float slope = (GetCruisingSpeedXY())  / target_threshold_xy;
						vel_sp_along_track =  slope * (vec_closest_to_current.Length());

						/* since we want to slow down take over previous velocity setpoint along track if it was lower but ensure its not zero */
						if ((vel_sp_along_track_prev < vel_sp_along_track) && (vel_sp_along_track * vel_sp_along_track_prev > 0.0f)
							&& (vel_sp_along_track_prev > 0.5f)) {
							vel_sp_along_track = vel_sp_along_track_prev;
						}
					}
				}

				/* compute velocity orthogonal to prev-current-line to position*/
				math::Vector2F vec_pos_to_closest = closest_point - math::Vector2F(Position[0], Position[1]);
				float vel_sp_orthogonal = vec_pos_to_closest.Length() * PosP[0];

				/* compute the cruise speed from velocity along line and orthogonal velocity setpoint */
				float cruise_sp_mag = sqrtf(vel_sp_orthogonal * vel_sp_orthogonal + vel_sp_along_track * vel_sp_along_track);

				/* sanity check */
				cruise_sp_mag = (isfinite(cruise_sp_mag)) ? cruise_sp_mag : vel_sp_orthogonal;

				/* orthogonal velocity setpoint is smaller than cruise speed */
				if (vel_sp_orthogonal < GetCruisingSpeedXY() && !current_behind) {

					/* we need to limit vel_sp_along_track such that cruise speed  is never exceeded but still can keep velocity orthogonal to track */
					if (cruise_sp_mag > GetCruisingSpeedXY()) {
						vel_sp_along_track = sqrtf(GetCruisingSpeedXY() * GetCruisingSpeedXY() - vel_sp_orthogonal * vel_sp_orthogonal);
					}

					pos_sp[0] = closest_point[0] + unit_prev_to_current[0] * vel_sp_along_track / PosP[0];
					pos_sp[1] = closest_point[1] + unit_prev_to_current[1] * vel_sp_along_track / PosP[1];

				} else if (current_behind) {
					/* current is behind */

					if (vec_pos_to_current.Length()  > 0.01f) {
						pos_sp[0] = Position[0] + vec_pos_to_current[0] / vec_pos_to_current.Length() * vel_sp_along_track / PosP[0];
						pos_sp[1] = Position[1] + vec_pos_to_current[1] / vec_pos_to_current.Length() * vel_sp_along_track / PosP[1];

					} else {
						pos_sp[0] = CurrentPositionSetpoint[0];
						pos_sp[1] = CurrentPositionSetpoint[1];
					}

				} else {
					/* we are more than cruise_speed away from track */

					/* if previous is in front just go directly to previous point */
					if (previous_in_front) {
						vec_pos_to_closest[0] = PreviousPositionSetpoint[0] - Position[0];
						vec_pos_to_closest[1] = PreviousPositionSetpoint[1] - Position[1];
					}

					/* make sure that we never exceed maximum cruise speed */
					float cruise_sp = vec_pos_to_closest.Length() * PosP[0];

					if (cruise_sp > GetCruisingSpeedXY()) {
						cruise_sp = GetCruisingSpeedXY();
					}

					/* sanity check: don't divide by zero */
					if (vec_pos_to_closest.Length() > SIGMA_NORM) {
						pos_sp[0] = Position[0] + vec_pos_to_closest[0] / vec_pos_to_closest.Length() * cruise_sp / PosP[0];
						pos_sp[1] = Position[1] + vec_pos_to_closest[1] / vec_pos_to_closest.Length() * cruise_sp / PosP[1];

					} else {
						pos_sp[0] = closest_point[0];
						pos_sp[1] = closest_point[1];
					}
				}
			}

			PositionSetpoint = pos_sp;

		} else if (PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_VELOCITY) {

			float vel_xy_mag = sqrtf(Velocity[0] * Velocity[0] + Velocity[1] * Velocity[1]);

			if (vel_xy_mag > SIGMA_NORM) {
				VelocitySetpoint[0] = Velocity[0] / vel_xy_mag * GetCruisingSpeedXY();
				VelocitySetpoint[1] = Velocity[1] / vel_xy_mag * GetCruisingSpeedXY();

			} else {
				/* TODO: we should go in the direction we are heading
				 * if current velocity is zero
				 */
				VelocitySetpoint[0] = 0.0f;
				VelocitySetpoint[1] = 0.0f;
			}

			RunPosControl = false;

		} else {
			/* just go to the target point */;
			PositionSetpoint = CurrentPositionSetpoint;

			/* set max velocity to cruise */
			_vel_max_xy = GetCruisingSpeedXY();
		}

		/* sanity check */
		if (!(isfinite(PositionSetpoint[0]) && isfinite(PositionSetpoint[1]) &&
			  isfinite(PositionSetpoint[2]))) {

			//warn_rate_limited("Auto: Position setpoint not finite");
			PositionSetpoint = CurrentPositionSetpoint;
		}


		/*
		 * if we're already near the current takeoff setpoint don't reset in case we switch back to posctl.
		 * this makes the takeoff finish smoothly.
		 */
		if ((PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_TAKEOFF
			 || PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER)
			&& PositionSetpointTripletMsg.Current.AcceptanceRadius > 0.0f
			/* need to detect we're close a bit before the navigator switches from takeoff to next waypoint */
			&& (Position - PositionSetpoint).Length() < PositionSetpointTripletMsg.Current.AcceptanceRadius * 1.2f) {
			DoResetAltPos = false;

		} else {
			/* otherwise: in case of interrupted mission don't go to waypoint but stay at current position */
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
		/* idle or triplet not valid, set velocity setpoint to zero */
		VelocitySetpoint.Zero();
		RunPosControl = false;
		RunAltControl = false;
	}
}

void MPC::CalculateVelocitySetpoint(float dt) // DONE
{
	/* Run position & altitude controllers, if enabled (otherwise use already
	 * computed velocity setpoints) */
	if(RunPosControl)
	{
		/* If for any reason, we get a NaN position setpoint, we better just
		 * stay where we are.
		 */
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

	/* in auto the setpoint is already limited by the navigator */
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

	/* limit vertical upwards speed in auto takeoff and close to ground */
	float altitude_above_home = -Position[2] + HomePositionMsg.Z;

	if (PositionSetpointTripletMsg.Current.Valid
	    && PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_TAKEOFF
	    && !VehicleControlModeMsg.ControlManualEnabled)
	{
		float vel_limit = math::gradual(altitude_above_home,
										ConfigTblPtr->LAND_ALT2, ConfigTblPtr->LAND_ALT1,
										ConfigTblPtr->TKO_SPEED, ConfigTblPtr->Z_VEL_MAX_UP);
		VelocitySetpoint[2] = math::max(VelocitySetpoint[2], -vel_limit);
	}

	/* limit vertical downwards speed (positive z) close to ground
	 * for now we use the altitude above home and assume that we want to land at same height as we took off */
	float vel_limit = math::gradual(altitude_above_home,
			ConfigTblPtr->LAND_ALT2, ConfigTblPtr->LAND_ALT1,
			ConfigTblPtr->LAND_SPEED, ConfigTblPtr->Z_VEL_MAX_DN);

	VelocitySetpoint[2] = math::min(VelocitySetpoint[2], vel_limit);

	/* Apply slew rate (aka acceleration limit) for smooth flying. */
	if (!VehicleControlModeMsg.ControlAutoEnabled)
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
	float vel_norm_xy = sqrtf(VelocitySetpoint[0] * VelocitySetpoint[0] + VelocitySetpoint[1] * VelocitySetpoint[1]);
	if (vel_norm_xy > ConfigTblPtr->XY_VEL_MAX)
	{
		VelocitySetpoint[0] = VelocitySetpoint[0] * ConfigTblPtr->XY_VEL_MAX / vel_norm_xy;
		VelocitySetpoint[1] = VelocitySetpoint[1] * ConfigTblPtr->XY_VEL_MAX / vel_norm_xy;
	}

	VelocitySetpoint[2] = math::constrain(VelocitySetpoint[2], -ConfigTblPtr->Z_VEL_MAX_UP, ConfigTblPtr->Z_VEL_MAX_DN);

	VelocitySetpointPrevious = VelocitySetpoint;
	/* Publish velocity setpoint */
//	VehicleGlobalVelocitySetpointMsg.Timestamp = PX4LIB_GetPX4TimeUs();
//	VehicleGlobalVelocitySetpointMsg.VX = VelocitySetpoint[0];
//	VehicleGlobalVelocitySetpointMsg.VY = VelocitySetpoint[1];
//	VehicleGlobalVelocitySetpointMsg.VZ = VelocitySetpoint[2];
//
//	SendVehicleGlobalVelocitySetpointMsg(); TODO
}

void MPC::CalculateThrustSetpoint(float dt) // UPDATED
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

	/* if any of the velocity setpoint is bogus, it's probably safest to command no velocity at all. */
	for (int i = 0; i < 3; ++i) {
		if (!isfinite(VelocitySetpoint[i])) {
			VelocitySetpoint[i] = 0.0f;
		}
	}

	/* Velocity error */
	math::Vector3F vel_err = VelocitySetpoint - Velocity;

	/* Thrust vector in NED frame. */
	math::Vector3F thrust_sp;

	if (VehicleControlModeMsg.ControlAccelerationEnabled && PositionSetpointTripletMsg.Current.AccelerationValid)
	{
		thrust_sp = math::Vector3F(PositionSetpointTripletMsg.Current.AX, PositionSetpointTripletMsg.Current.AY, PositionSetpointTripletMsg.Current.AZ);
	}
	else
	{
		thrust_sp = vel_err.EMult(VelP) + VelocityErrD.EMult(VelD)
			    + ThrustInt - math::Vector3F(0.0f, 0.0f, ConfigTblPtr->THR_HOVER);
	}

	if (!VehicleControlModeMsg.ControlVelocityEnabled && !VehicleControlModeMsg.ControlAccelerationEnabled)
	{
		thrust_sp[0] = 0.0f;
		thrust_sp[1] = 0.0f;
	}

	/* If still or already on ground command zero xy velocity and zero xy
	 * thrust_sp in body frame to consider uneven ground. */
	if (VehicleLandDetectedMsg.GroundContact && !InAutoTakeoff() && !ManualWantsTakeoff())
	{
		/* Thrust setpoint in body frame*/
		math::Vector3F thrust_sp_body = Rotation.Transpose() * thrust_sp;

		/* We dont want to make any correction in body x and y*/
		thrust_sp_body[0] = 0.0f;
		thrust_sp_body[1] = 0.0f;

		/* Make sure z component of thrust_sp_body is larger than 0 (positive thrust is downward) */
		thrust_sp_body[2] = thrust_sp[2] > 0.0f ? thrust_sp[2] : 0.0f;

		/* Convert back to local frame (NED) */
		thrust_sp = Rotation * thrust_sp_body;

		//TODO or thrust_sp.Zero();
	}

	if (!VehicleControlModeMsg.ControlClimbRateEnabled && !VehicleControlModeMsg.ControlAccelerationEnabled)
	{
		thrust_sp[2] = 0.0f;
	}

	/* Limit thrust vector and check for saturation. */
	bool saturation_xy = false;
	bool saturation_z = false;

	/* Limit min lift */
	float thr_min = ConfigTblPtr->THR_MIN;

	if (!VehicleControlModeMsg.ControlVelocityEnabled && thr_min < 0.0f)
	{
		/* Don't allow downside thrust direction in manual attitude mode. */
		thr_min = 0.0f;
	}

	float tilt_max = ConfigTblPtr->TILTMAX_AIR;
	float thr_max = ConfigTblPtr->THR_MAX;

	/* We can only run the control if we're already in-air, have a takeoff setpoint,
	 * or if we're in offboard control.  Otherwise, we should just bail out. */
	if (VehicleLandDetectedMsg.Landed && !InAutoTakeoff() && !ManualWantsTakeoff())
	{
		/* Keep throttle low while still on ground. */
		thr_max = 0.0f;
	}
	else if (!VehicleControlModeMsg.ControlManualEnabled && PositionSetpointTripletMsg.Current.Valid &&
			PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LAND)
	{
		/* Adjust limits for landing mode.  Limit max tilt and min lift when landing. */
		tilt_max = ConfigTblPtr->TILTMAX_LND;
	}

	/* Limit min lift */
	if (-thrust_sp[2] < thr_min)
	{
		thrust_sp[2] = -thr_min;

		/* Don't freeze altitude integral if it wants to throttle up. */
		saturation_z = vel_err[2] > 0.0f ? true : saturation_z;
	}

	if (VehicleControlModeMsg.ControlVelocityEnabled || VehicleControlModeMsg.ControlAccelerationEnabled)
	{
		/* Limit max tilt */
		if (thr_min >= 0.0f && tilt_max < M_PI / 2 - 0.05f)
		{
			/* Absolute horizontal thrust */
			float thrust_sp_xy_len = math::Vector2F(thrust_sp[0], thrust_sp[1]).Length();

			if (thrust_sp_xy_len > 0.01f)
			{
				/* Max horizontal thrust for given vertical thrust. */
				float thrust_xy_max = -thrust_sp[2] * tanf(tilt_max);

				if (thrust_sp_xy_len > thrust_xy_max)
				{
					float k = thrust_xy_max / thrust_sp_xy_len;
					thrust_sp[0] *= k;
					thrust_sp[1] *= k;

					/* Don't freeze x,y integrals if they both want to throttle down. */
					saturation_xy = ((vel_err[0] * VelocitySetpoint[0] < 0.0f) && (vel_err[1] * VelocitySetpoint[1] < 0.0f)) ? saturation_xy : true;
				}
			}
		}
	}

	if (VehicleControlModeMsg.ControlClimbRateEnabled && !VehicleControlModeMsg.ControlVelocityEnabled)
	{
		/* Thrust compensation when vertical velocity but not horizontal velocity is controlled. */
		float att_comp;

		const float tilt_cos_max = 0.7f;

		if (Rotation[2][2] > tilt_cos_max)
		{
			att_comp = 1.0f / Rotation[2][2];
		}
		else if (Rotation[2][2] > 0.0f)
		{
			att_comp = ((1.0f / tilt_cos_max - 1.0f) / tilt_cos_max) * Rotation[2][2] + 1.0f;
			saturation_z = true;
		}
		else
		{
			att_comp = 1.0f;
			saturation_z = true;
		}

		thrust_sp[2] *= att_comp;
	}

	/* Calculate desired total thrust amount in body z direction. */
	/* To compensate for excess thrust during attitude tracking errors we
	 * project the desired thrust force vector F onto the real vehicle's thrust axis in NED:
	 * body thrust axis [0,0,-1]' rotated by R is: R*[0,0,-1]' = -R_z */
	math::Vector3F R_z(Rotation[0][2], Rotation[1][2], Rotation[2][2]);
	math::Vector3F F(thrust_sp);

	/* Recalculate because it might have changed. */
	float thrust_body_z = F * -R_z;

	/* Limit max thrust. */
	if (fabsf(thrust_body_z) > thr_max)
	{
		if (thrust_sp[2] < 0.0f)
		{
			if (-thrust_sp[2] > thr_max)
			{
				/* Thrust Z component is too large, limit it. */
				thrust_sp[0] = 0.0f;
				thrust_sp[1] = 0.0f;
				thrust_sp[2] = -thr_max;
				saturation_xy = true;

				/* Don't freeze altitude integral if it wants to throttle down. */
				saturation_z = vel_err[2] < 0.0f ? true : saturation_z;
			}
			else
			{
				/* Preserve thrust Z component and lower XY, keeping altitude is more important than position. */
				float thrust_xy_max = sqrtf(thr_max * thr_max - thrust_sp[2] * thrust_sp[2]);
				float thrust_xy_abs = math::Vector2F(thrust_sp[0], thrust_sp[1]).Length();
				float k = thrust_xy_max / thrust_xy_abs;
				thrust_sp[0] *= k;
				thrust_sp[1] *= k;
				/* Don't freeze x,y integrals if they both want to throttle down */
				saturation_xy = ((vel_err[0] * VelocitySetpoint[0] < 0.0f) && (vel_err[1] * VelocitySetpoint[1] < 0.0f)) ? saturation_xy : true;
			}
		}
		else
		{
			/* Z component is positive, going down (Z is positive down in NED), simply limit thrust vector. */
			float k = thr_max / fabsf(thrust_body_z);
			thrust_sp = thrust_sp * k;
			saturation_xy = true;
			saturation_z = true;
		}

		thrust_body_z = thr_max;
	}

	VehicleAttitudeSetpointMsg.Thrust = math::max(thrust_body_z, thr_min);

	/* Update integrals */
	if (VehicleControlModeMsg.ControlVelocityEnabled && !saturation_xy)
	{
		ThrustInt[0] += vel_err[0] * VelI[0] * dt;
		ThrustInt[1] += vel_err[1] * VelI[1] * dt;
	}

	if (VehicleControlModeMsg.ControlClimbRateEnabled && !saturation_z)
	{
		ThrustInt[2] += vel_err[2] * VelI[2] * dt;
	}

	/* Calculate attitude setpoint from thrust vector. */
	if (VehicleControlModeMsg.ControlVelocityEnabled || VehicleControlModeMsg.ControlAccelerationEnabled)
	{
		/* Desired body_z axis = -normalize(thrust_vector) */
		math::Vector3F body_x;
		math::Vector3F body_y;
		math::Vector3F body_z;

		if (thrust_sp.Length() > SIGMA_NORM)
		{
			body_z = -thrust_sp.Normalized();
		}
		else
		{
			/* No thrust, set Z axis to safe value. */
			body_z.Zero();
			body_z[2] = 1.0f;
		}

		/* Vector of desired yaw direction in XY plane, rotated by PI/2. */
		math::Vector3F y_C(-sinf(VehicleAttitudeSetpointMsg.YawBody), cosf(VehicleAttitudeSetpointMsg.YawBody), 0.0f);

		if (fabsf(body_z[2]) > SIGMA_SINGLE_OP)
		{
			/* Desired body_x axis, orthogonal to body_z. */
			body_x = y_C % body_z;

			/* Keep nose to front while inverted upside down. */
			if (body_z[2] < 0.0f)
			{
				body_x = -body_x;
			}

			body_x.Normalize();
		}
		else
		{
			/* Desired thrust is in XY plane, set X downside to construct
			 * correct matrix, but yaw component will not be used actually */
			body_x.Zero();
			body_x[2] = 1.0f;
		}

		/* Desired body_y axis */
		body_y = body_z % body_x;

		/* Fill rotation matrix */
		for (uint32 i = 0; i < 3; i++)
		{
			RSetpoint[i][0] = body_x[i];
			RSetpoint[i][1] = body_y[i];
			RSetpoint[i][2] = body_z[i];
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
	VehicleLocalPositionSetpointMsg.AccX = thrust_sp[0] * MPC_CONSTANTS_ONE_G;
	VehicleLocalPositionSetpointMsg.AccY = thrust_sp[1] * MPC_CONSTANTS_ONE_G;
	VehicleLocalPositionSetpointMsg.AccZ = thrust_sp[2] * MPC_CONSTANTS_ONE_G;

	VehicleAttitudeSetpointMsg.Timestamp = PX4LIB_GetPX4TimeUs();
}



float MPC::GetCruisingSpeedXY(void) // Updated
{
	float res;

	/*
	 * In missions, the user can choose cruising speed different to default.
	 */
	res = ((isfinite(PositionSetpointTripletMsg.Current.CruisingSpeed) &&
			!(PositionSetpointTripletMsg.Current.CruisingSpeed < 0.0f)) ?
					PositionSetpointTripletMsg.Current.CruisingSpeed : ConfigTblPtr->XY_CRUISE);

	return res;
}



bool MPC::CrossSphereLine(const math::Vector3F &sphere_c, const float sphere_r,
		const math::Vector3F &line_a, const math::Vector3F &line_b, math::Vector3F &res) // UPDATED
{
	bool result = false;
	math::Vector3F d;
	float cd_len = 0.0f;
	float dx_len = 0.0f;

	/* Project center of sphere on line normalized AB. */
	math::Vector3F ab_norm = line_b - line_a;

	if(ab_norm.Length() < 0.01)
	{
		result = true;
		goto MPC_CrossSphereLine_Exit_Tag;
	}

	ab_norm.Normalize();
	d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	cd_len = (sphere_c - d).Length();

	if (sphere_r > cd_len)
	{
		/* We have triangle CDX with known CD and CX = R, find DX. */
		dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

		if ((sphere_c - line_b) * ab_norm > 0.0f)
		{
			/* Target waypoint is already behind us. */
			res = line_b;
		}
		else
		{
			/* Target is in front of us. */
			/* vector A->B on line */
			res = d + ab_norm * dx_len;
		}

		result = true;
	}
	else
	{
		/* Have no roots. Return D */
		/* Go directly to line */
		res = d;

		/* Previous waypoint is still in front of us. */
		if ((sphere_c - line_a) * ab_norm < 0.0f) {
			res = line_a;
		}

		/* Target waypoint is already behind us. */
		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			res = line_b;
		}

		result = false;
	}

MPC_CrossSphereLine_Exit_Tag:
	return result;
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
void MPC::LimitAltitude(void)
{
	if (VehicleLandDetectedMsg.AltMax < 0.0f)
	{
		// there is no altitude limitation present
		//goto MPC_LimitAltitude_ExitTag;
		return;
	}

	float AltitudeAboveHome = -(Position[2] - HomePositionMsg.Z);

	if (RunAltControl && (AltitudeAboveHome > VehicleLandDetectedMsg.AltMax))
	{
		// we are above maximum altitude
		PositionSetpoint[2] = -VehicleLandDetectedMsg.AltMax +  HomePositionMsg.Z;
	}
	else if (!RunAltControl && VelocitySetpoint[2] <= 0.0f)
	{
		// we want to fly upwards: check if vehicle does not exceed altitude

		// time to reach zero velocity
		float delta_t = -Velocity[2] / ConfigTblPtr->ACC_DOWN_MAX;

		// predict next position based on current position, velocity, max acceleration downwards and time to reach zero velocity
		float pos_z_next = Position[2] + Velocity[2] * delta_t + 0.5f * ConfigTblPtr->ACC_DOWN_MAX * delta_t * delta_t;

		if (-(pos_z_next - HomePositionMsg.Z) > VehicleLandDetectedMsg.AltMax) {
			// prevent the vehicle from exceeding maximum altitude by switching back to altitude control with maximum altitude as setpoint
			PositionSetpoint[2] = -VehicleLandDetectedMsg.AltMax + HomePositionMsg.Z;
			RunAltControl = true;
		}
	}

	return;
}

void MPC::ApplyVelocitySetpointSlewRate(float dt) // UPDATED
{
	math::Vector2F vel_sp_xy(VelocitySetpoint[0], VelocitySetpoint[1]);
	math::Vector2F vel_sp_prev_xy(VelocitySetpointPrevious[0], VelocitySetpointPrevious[1]);
	math::Vector2F vel_xy(Velocity[0], Velocity[1]);
	math::Vector2F acc_xy = (vel_sp_xy - vel_sp_prev_xy) / dt;

	/* limit total horizontal acceleration */
	if (acc_xy.Length() > _acceleration_state_dependent_xy) {
		vel_sp_xy = acc_xy.Normalized() * _acceleration_state_dependent_xy * dt + vel_sp_prev_xy;
		VelocitySetpoint[0] = vel_sp_xy[0];
		VelocitySetpoint[1] = vel_sp_xy[1];
	}

	/* limit vertical acceleration */
	float acc_z = (VelocitySetpoint[2] - VelocitySetpointPrevious[2]) / dt;
	float max_acc_z;

	if (VehicleControlModeMsg.ControlManualEnabled) {
		max_acc_z = (acc_z < 0.0f) ? -_acceleration_state_dependent_z : _acceleration_state_dependent_z;

	} else {
		max_acc_z = (acc_z < 0.0f) ? -ConfigTblPtr->ACC_UP_MAX : ConfigTblPtr->ACC_DOWN_MAX;
	}

	if (fabsf(acc_z) > fabsf(max_acc_z)) {
		VelocitySetpoint[2] = max_acc_z * dt + VelocitySetpointPrevious[2];
	}
}



bool MPC::InAutoTakeoff(void) // Updated
{
	bool res = false;

	/*
	 * In auto mode, check if we do a takeoff
	 */
	if(PositionSetpointTripletMsg.Current.Valid &&
			PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_TAKEOFF)
	{
	    /* We are in takeoff mode. */
	    res = true;
	}
	else if( VehicleControlModeMsg.ControlOffboardEnabled)
	{
		/* We are in takeoff mode. */
		res = true;
	}

	return res;
}

float MPC::GetVelClose(const math::Vector2F &unit_prev_to_current, const math::Vector2F &unit_current_to_next) // Updated
{
	/* minimum cruise speed when passing waypoint */
	float min_cruise_speed = 1.0f;

	/* make sure that cruise speed is larger than minimum*/
	if ((GetCruisingSpeedXY() - min_cruise_speed) < SIGMA_NORM)
	{
		return GetCruisingSpeedXY();
	}

	/* middle cruise speed is a number between maximum cruising speed and minimum cruising speed and corresponds to speed at angle of 90degrees
	 * it needs to be always larger than minimum cruise speed */
	float middle_cruise_speed = ConfigTblPtr->MPC_CRUISE_90;

	if ((middle_cruise_speed - min_cruise_speed) < SIGMA_NORM)
	{
		middle_cruise_speed = min_cruise_speed + SIGMA_NORM;
	}

	if ((GetCruisingSpeedXY() - middle_cruise_speed) < SIGMA_NORM)
	{
		middle_cruise_speed = (GetCruisingSpeedXY() + min_cruise_speed) * 0.5f;
	}

	/* if middle cruise speed is exactly in the middle, then compute
	 * vel_close linearly
	 */
	bool use_linear_approach = false;

	if (((GetCruisingSpeedXY() + min_cruise_speed) * 0.5f) - middle_cruise_speed < SIGMA_NORM)
	{
		use_linear_approach = true;
	}

	/* angle = cos(x) + 1.0
	 * angle goes from 0 to 2 with 0 = large angle, 2 = small angle:   0 = PI ; 2 = PI*0 */
	float angle = 2.0f;

	if (unit_current_to_next.Length() > SIGMA_NORM)
	{
		angle = unit_current_to_next * (unit_prev_to_current * -1.0f) + 1.0f;
	}

	/* compute velocity target close to waypoint */
	float vel_close;

	if (use_linear_approach)
	{

		/* velocity close to target adjusted to angle
		 * vel_close =  m*x+q
		 */
		float slope = -(GetCruisingSpeedXY() - min_cruise_speed) / 2.0f;
		vel_close = slope * angle + GetCruisingSpeedXY();

	}
	else
	{
		/* velocity close to target adjusted to angle
		 * vel_close = a *b ^x + c; where at angle = 0 -> vel_close = vel_cruise; angle = 1 -> vel_close = middle_cruise_speed (this means that at 90degrees
		 * the velocity at target is middle_cruise_speed);
		 * angle = 2 -> vel_close = min_cruising_speed */

		/* from maximum cruise speed, minimum cruise speed and middle cruise speed compute constants a, b and c */
		float a = -((middle_cruise_speed -  GetCruisingSpeedXY()) * (middle_cruise_speed -  GetCruisingSpeedXY())) /
			  (2.0f * middle_cruise_speed - GetCruisingSpeedXY() - min_cruise_speed);
		float c =  GetCruisingSpeedXY() - a;
		float b = (middle_cruise_speed - c) / a;
		vel_close = a * powf(b, angle) + c;
	}

	/* vel_close needs to be in between max and min */
	return math::constrain(vel_close, min_cruise_speed, GetCruisingSpeedXY());
}

void MPC::SetManualAccelerationZ(float &max_acceleration, const float stick_z, const float dt) // Updated
{
	/* in manual altitude control apply acceleration limit based on stick input
	 * we consider two states
	 * 1.) brake
	 * 2.) accelerate */

	/* check if zero input stick */
	const bool is_current_zero = (fabsf(stick_z) <= FLT_EPSILON);

	/* default is acceleration */
	manual_stick_input intention = acceleration;

	/* check zero input stick */
	if (is_current_zero) {
		intention = brake;
	}

	/* get max and min acceleration where min acceleration is just 1/5 of max acceleration */
	max_acceleration = (stick_z <= 0.0f) ? ConfigTblPtr->ACC_UP_MAX : ConfigTblPtr->ACC_DOWN_MAX;

	/*
	 * update user input
	 */
	if ((_user_intention_z != brake) && (intention  == brake)) {

		/* we start with lowest acceleration */
		_acceleration_state_dependent_z = ConfigTblPtr->ACC_DOWN_MAX;

		/* reset slew rate */
		VelocitySetpointPrevious[2] = Velocity[2];
		_user_intention_z = brake;
	}

	_user_intention_z = intention;

	/*
	 * apply acceleration depending on state
	 */
	if (_user_intention_z == brake) {

		/* limit jerk when braking to zero */
		float jerk = (ConfigTblPtr->ACC_UP_MAX - _acceleration_state_dependent_z) / dt;

		if (jerk > _manual_jerk_limit_z) {
			_acceleration_state_dependent_z = _manual_jerk_limit_z * dt + _acceleration_state_dependent_z;

		} else {
			_acceleration_state_dependent_z = ConfigTblPtr->ACC_UP_MAX;
		}
	}

	if (_user_intention_z == acceleration) {
		_acceleration_state_dependent_z = (max_acceleration - ConfigTblPtr->ACC_DOWN_MAX) * fabsf(
				stick_z) + ConfigTblPtr->ACC_DOWN_MAX;
	}
}

void MPC::SetManualAccelerationXY(math::Vector2F &stick_xy, const float dt) // Updated
{

	/*
	 * In manual mode we consider four states with different acceleration handling:
	 * 1. user wants to stop
	 * 2. user wants to quickly change direction
	 * 3. user wants to accelerate
	 * 4. user wants to decelerate
	 */

	/* get normalized stick input vector */
	math::Vector2F stick_xy_norm = (stick_xy.Length() > 0.0f) ? stick_xy.Normalized() : stick_xy;
	math::Vector2F stick_xy_prev_norm = (_stick_input_xy_prev.Length() > 0.0f) ? _stick_input_xy_prev.Normalized() :
					      _stick_input_xy_prev;

	/* check if stick direction and current velocity are within 60angle */
	const bool is_aligned = (stick_xy_norm * stick_xy_prev_norm) > 0.5f;

	/* check if zero input stick */
	const bool is_prev_zero = (fabsf(_stick_input_xy_prev.Length()) <= FLT_EPSILON);
	const bool is_current_zero = (fabsf(stick_xy.Length()) <= FLT_EPSILON);

	/* check acceleration */
	const bool do_acceleration = is_prev_zero || (is_aligned &&
				     ((stick_xy.Length() > _stick_input_xy_prev.Length()) || (fabsf(stick_xy.Length() - 1.0f) < FLT_EPSILON)));

	const bool do_deceleration = (is_aligned && (stick_xy.Length() <= _stick_input_xy_prev.Length()));

	const bool do_direction_change = !is_aligned;

	manual_stick_input intention;

	if (is_current_zero) {
		/* we want to stop */
		intention = brake;

	} else if (do_acceleration) {
		/* we do manual acceleration */
		intention = acceleration;

	} else if (do_deceleration) {
		/* we do manual deceleration */
		intention = deceleration;

	} else if (do_direction_change) {
		/* we have a direction change */
		intention = direction_change;

	} else {
		/* catchall: acceleration */
		intention = acceleration;
	}


	/*
	 * update user intention
	 */

	/* we always want to break starting with slow deceleration */
	if ((_user_intention_xy != brake) && (intention  == brake)) {

		if (ConfigTblPtr->MPC_JERK_MAX > ConfigTblPtr->MPC_JERK_MIN) {
			_manual_jerk_limit_xy = (ConfigTblPtr->MPC_JERK_MAX - ConfigTblPtr->MPC_JERK_MIN) / ConfigTblPtr->MPC_VEL_MANUAL *
						sqrtf(Velocity[0] * Velocity[0] + Velocity[1] * Velocity[1]) + ConfigTblPtr->MPC_JERK_MIN;

			/* we start braking with lowest accleration */
			_acceleration_state_dependent_xy = ConfigTblPtr->MPC_DEC_HOR_SLOW;

		} else {

			/* set the jerk limit large since we don't know it better*/
			_manual_jerk_limit_xy = 1000000.f;

			/* at brake we use max acceleration */
			_acceleration_state_dependent_xy = ConfigTblPtr->ACC_HOR_MAX;

		}

		/* reset slew rate */
		VelocitySetpointPrevious[0] = Velocity[0];
		VelocitySetpointPrevious[1] = Velocity[1];

	}

	switch (_user_intention_xy) {
	case brake: {
			if (intention != brake) {
				_user_intention_xy = acceleration;
				/* we initialize with lowest acceleration */
				_acceleration_state_dependent_xy = ConfigTblPtr->MPC_DEC_HOR_SLOW;
			}

			break;
		}

	case direction_change: {
			/* only exit direction change if brake or aligned */
			math::Vector2F vel_xy(Velocity[0], Velocity[1]);
			math::Vector2F vel_xy_norm = (vel_xy.Length() > 0.0f) ? vel_xy.Normalized() : vel_xy;
			bool stick_vel_aligned = (vel_xy_norm * stick_xy_norm > 0.0f);

			/* update manual direction change hysteresis */
			_manual_direction_change_hysteresis.set_state_and_update(!stick_vel_aligned, PX4LIB_GetPX4TimeUs());


			/* exit direction change if one of the condition is met */
			if (intention == brake) {
				_user_intention_xy = intention;

			} else if (stick_vel_aligned) {
				_user_intention_xy = acceleration;

			} else if (_manual_direction_change_hysteresis.get_state()) {

				/* TODO: find conditions which are always continuous
				 * only if stick input is large*/
				if (stick_xy.Length() > 0.6f) {
					_acceleration_state_dependent_xy = ConfigTblPtr->ACC_HOR_MAX;
				}
			}

			break;
		}

	case acceleration: {
			_user_intention_xy = intention;

			if (_user_intention_xy == direction_change) {
				VelocitySetpointPrevious[0] = Velocity[0];
				VelocitySetpointPrevious[1] = Velocity[1];
			}

			break;
		}

	case deceleration: {
			_user_intention_xy = intention;

			if (_user_intention_xy == direction_change) {
				VelocitySetpointPrevious[0] = Velocity[0];
				VelocitySetpointPrevious[1] = Velocity[1];
			}

			break;
		}
	}

	/*
	 * apply acceleration based on state
	*/
	switch (_user_intention_xy) {
	case brake: {

			/* limit jerk when braking to zero */
			float jerk = (ConfigTblPtr->ACC_HOR_MAX - _acceleration_state_dependent_xy) / dt;

			if (jerk > _manual_jerk_limit_xy) {
				_acceleration_state_dependent_xy = _manual_jerk_limit_xy * dt + _acceleration_state_dependent_xy;

			} else {
				_acceleration_state_dependent_xy = ConfigTblPtr->ACC_HOR_MAX;
			}

			break;
		}

	case direction_change: {

			/* limit acceleration linearly on stick input*/
			_acceleration_state_dependent_xy = (ConfigTblPtr->MPC_ACC_HOR - ConfigTblPtr->MPC_DEC_HOR_SLOW) * stick_xy.Length() +
							   ConfigTblPtr->MPC_DEC_HOR_SLOW;
			break;
		}

	case acceleration: {
			/* limit acceleration linearly on stick input*/
			float acc_limit  = (ConfigTblPtr->MPC_ACC_HOR - ConfigTblPtr->MPC_DEC_HOR_SLOW) * stick_xy.Length()
					   + ConfigTblPtr->MPC_DEC_HOR_SLOW;

			if (_acceleration_state_dependent_xy > acc_limit) {
				acc_limit = _acceleration_state_dependent_xy;
			}

			_acceleration_state_dependent_xy = acc_limit;
			break;
		}

	case deceleration: {
			_acceleration_state_dependent_xy = ConfigTblPtr->MPC_DEC_HOR_SLOW;
			break;
		}

	default :
		//warn_rate_limited("User intention not recognized"); TODO
		_acceleration_state_dependent_xy = ConfigTblPtr->ACC_HOR_MAX;

	}

	/* update previous stick input */
	_stick_input_xy_prev = math::Vector2F(_filter_manual_pitch.apply(stick_xy[0]),
						_filter_manual_roll.apply(stick_xy[1]));


	if (_stick_input_xy_prev.Length() > 1.0f) {
		_stick_input_xy_prev = _stick_input_xy_prev.Normalized();
	}
}

bool MPC::ManualWantsTakeoff() // Updated
{
	const bool has_manual_control_present = VehicleControlModeMsg.ControlManualEnabled && ManualControlSetpointMsg.Timestamp > 0;

	// Manual takeoff is triggered if the throttle stick is above 65%.
	return (has_manual_control_present && ManualControlSetpointMsg.Z > 0.65f);
}

/************************/
/*  End of File Comment */
/************************/
