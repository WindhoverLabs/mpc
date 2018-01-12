/************************************************************************
** Includes
*************************************************************************/
#include <string.h>

#include "cfe.h"

#include "mpc_app.h"
#include "mpc_msg.h"
#include "mpc_version.h"
#include "Quaternion.hpp"
#include "Matrix3F3.hpp"
#include "Vector2F.hpp"
#include "Quaternion.hpp"
#include <float.h>
#include <math.h>
#include "lib/px4lib.h"
#include "geo/geo.h"
#include "lib/mathlib/math/Expo.hpp"


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
MPC::MPC()
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
	TrustInt.Zero();

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

    InitData();

    iStatus = InitConfigTbl();
    if (iStatus != CFE_SUCCESS)
    {
        goto MPC_InitApp_Exit_Tag;
    }

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
//                OS_printf("************************\n");
//                OS_printf("Timestamp = %llu\n", ControlStateMsg.Timestamp);
//                OS_printf("AccX = %f\n", (double)ControlStateMsg.AccX);
//                OS_printf("AccY = %f\n", (double)ControlStateMsg.AccY);
//                OS_printf("AccZ = %f\n", (double)ControlStateMsg.AccZ);
//                OS_printf("VelX = %f\n", (double)ControlStateMsg.VelX);
//                OS_printf("VelY = %f\n", (double)ControlStateMsg.VelY);
//                OS_printf("VelZ = %f\n", (double)ControlStateMsg.VelZ);
//                OS_printf("PosX = %f\n", (double)ControlStateMsg.PosX);
//                OS_printf("PosY = %f\n", (double)ControlStateMsg.PosY);
//                OS_printf("PosZ = %f\n", (double)ControlStateMsg.PosZ);
//                OS_printf("Airspeed = %f\n", (double)ControlStateMsg.Airspeed);
//                OS_printf("Airspeed = %f\n", (double)ControlStateMsg.Airspeed);
//                OS_printf("Airspeed = %f\n", (double)ControlStateMsg.Airspeed);
//                OS_printf("Airspeed = %f\n", (double)ControlStateMsg.Airspeed);
//                OS_printf("Airspeed = %f\n", (double)ControlStateMsg.Airspeed);
//                OS_printf("VelVariance[0] = %f\n", (double)ControlStateMsg.VelVariance[0]);
//                OS_printf("VelVariance[1] = %f\n", (double)ControlStateMsg.VelVariance[1]);
//                OS_printf("VelVariance[2] = %f\n", (double)ControlStateMsg.VelVariance[2]);
//                OS_printf("PosVariance[0] = %f\n", (double)ControlStateMsg.PosVariance[0]);
//                OS_printf("PosVariance[1] = %f\n", (double)ControlStateMsg.PosVariance[1]);
//                OS_printf("PosVariance[2] = %f\n", (double)ControlStateMsg.PosVariance[2]);
//                OS_printf("Q[0] = %f\n", (double)ControlStateMsg.Q[0]);
//                OS_printf("Q[1] = %f\n", (double)ControlStateMsg.Q[1]);
//                OS_printf("Q[2] = %f\n", (double)ControlStateMsg.Q[2]);
//                OS_printf("Q[3] = %f\n", (double)ControlStateMsg.Q[3]);
//                OS_printf("DeltaQReset[0] = %f\n", (double)ControlStateMsg.DeltaQReset[0]);
//                OS_printf("DeltaQReset[1] = %f\n", (double)ControlStateMsg.DeltaQReset[1]);
//                OS_printf("DeltaQReset[2] = %f\n", (double)ControlStateMsg.DeltaQReset[2]);
//                OS_printf("DeltaQReset[3] = %f\n", (double)ControlStateMsg.DeltaQReset[3]);
//                OS_printf("RollRate = %f\n", (double)ControlStateMsg.RollRate);
//                OS_printf("PitchRate = %f\n", (double)ControlStateMsg.PitchRate);
//                OS_printf("YawRate = %f\n", (double)ControlStateMsg.YawRate);
//                OS_printf("HorzAccMag = %f\n", (double)ControlStateMsg.HorzAccMag);
//                OS_printf("RollRateBias = %f\n", (double)ControlStateMsg.RollRateBias);
//                OS_printf("PitchRateBias = %f\n", (double)ControlStateMsg.PitchRateBias);
//                OS_printf("YawRateBias = %f\n", (double)ControlStateMsg.YawRateBias);
//                OS_printf("AirspeedValid = %u\n", ControlStateMsg.AirspeedValid);
//                OS_printf("QuatResetCounter = %u\n", ControlStateMsg.QuatResetCounter);
                ProcessControlStateMsg();
                break;

            case PX4_MANUAL_CONTROL_SETPOINT_MID:
                memcpy(&ManualControlSetpointMsg, MsgPtr, sizeof(ManualControlSetpointMsg));
                OS_printf("************************\n");
                OS_printf("Timestamp = %llu\n", ManualControlSetpointMsg.Timestamp);
                OS_printf("X = %f\n", (double)ManualControlSetpointMsg.X);
                OS_printf("Y = %f\n", (double)ManualControlSetpointMsg.Y);
                OS_printf("Z = %f\n", (double)ManualControlSetpointMsg.Z);
                OS_printf("R = %f\n", (double)ManualControlSetpointMsg.R);
                OS_printf("Flaps = %f\n", (double)ManualControlSetpointMsg.Flaps);
                OS_printf("Aux1 = %f\n", (double)ManualControlSetpointMsg.Aux1);
                OS_printf("Aux2 = %f\n", (double)ManualControlSetpointMsg.Aux2);
                OS_printf("Aux3 = %f\n", (double)ManualControlSetpointMsg.Aux3);
                OS_printf("Aux4 = %f\n", (double)ManualControlSetpointMsg.Aux4);
                OS_printf("Aux5 = %f\n", (double)ManualControlSetpointMsg.Aux5);
                OS_printf("ModeSwitch = %u\n", ManualControlSetpointMsg.ModeSwitch);
                OS_printf("ReturnSwitch = %u\n", ManualControlSetpointMsg.ReturnSwitch);
                OS_printf("RattitudeSwitch = %u\n", ManualControlSetpointMsg.RattitudeSwitch);
                OS_printf("PosctlSwitch = %u\n", ManualControlSetpointMsg.PosctlSwitch);
                OS_printf("LoiterSwitch = %u\n", ManualControlSetpointMsg.LoiterSwitch);
                OS_printf("AcroSwitch = %u\n", ManualControlSetpointMsg.AcroSwitch);
                OS_printf("OffboardSwitch = %u\n", ManualControlSetpointMsg.OffboardSwitch);
                OS_printf("KillSwitch = %u\n", ManualControlSetpointMsg.KillSwitch);
                OS_printf("TransitionSwitch = %u\n", ManualControlSetpointMsg.TransitionSwitch);
                OS_printf("GearSwitch = %u\n", ManualControlSetpointMsg.GearSwitch);
                OS_printf("ArmSwitch = %u\n", ManualControlSetpointMsg.ArmSwitch);
                OS_printf("StabSwitch = %u\n", ManualControlSetpointMsg.StabSwitch);
                OS_printf("ManSwitch = %u\n", ManualControlSetpointMsg.ManSwitch);
                OS_printf("ModeSlot = %i\n", ManualControlSetpointMsg.ModeSlot);
                OS_printf("DataSource = %u\n", ManualControlSetpointMsg.DataSource);
                break;

            case PX4_HOME_POSITION_MID:
                memcpy(&HomePositionMsg, MsgPtr, sizeof(HomePositionMsg));
//                OS_printf("************************\n");
//                OS_printf("Timestamp = %llu\n", HomePositionMsg.Timestamp);
//                OS_printf("Lat = %f\n", HomePositionMsg.Lat);
//                OS_printf("Lon = %f\n", HomePositionMsg.Lon);
//                OS_printf("Alt = %f\n", (double)HomePositionMsg.Alt);
//                OS_printf("X = %f\n", (double)HomePositionMsg.X);
//                OS_printf("Y = %f\n", (double)HomePositionMsg.Y);
//                OS_printf("Z = %f\n", (double)HomePositionMsg.Z);
//                OS_printf("Yaw = %f\n", (double)HomePositionMsg.Yaw);
//                OS_printf("DirectionX = %f\n", (double)HomePositionMsg.DirectionX);
//                OS_printf("DirectionY = %f\n", (double)HomePositionMsg.DirectionY);
//                OS_printf("DirectionZ = %f\n", (double)HomePositionMsg.DirectionZ);
                break;

            case PX4_VEHICLE_CONTROL_MODE_MID:
                memcpy(&VehicleControlModeMsg, MsgPtr, sizeof(VehicleControlModeMsg));
//                OS_printf("VehicleControlModeMsg ************************\n");
//                OS_printf("Timestamp = %llu\n", VehicleControlModeMsg.Timestamp);
//                OS_printf("Armed = %u\n", VehicleControlModeMsg.Armed);
//                OS_printf("ExternalManualOverrideOk = %u\n", VehicleControlModeMsg.ExternalManualOverrideOk);
//                OS_printf("SystemHilEnabled = %u\n", VehicleControlModeMsg.SystemHilEnabled);
//                OS_printf("ControlManualEnabled = %u\n", VehicleControlModeMsg.ControlManualEnabled);
//                OS_printf("ControlAutoEnabled = %u\n", VehicleControlModeMsg.ControlAutoEnabled);
//                OS_printf("ControlOffboardEnabled = %u\n", VehicleControlModeMsg.ControlOffboardEnabled);
//                OS_printf("ControlRatesEnabled = %u\n", VehicleControlModeMsg.ControlRatesEnabled);
//                OS_printf("ControlAttitudeEnabled = %u\n", VehicleControlModeMsg.ControlAttitudeEnabled);
//                OS_printf("ControlRattitudeEnabled = %u\n", VehicleControlModeMsg.ControlRattitudeEnabled);
//                OS_printf("ControlForceEnabled = %u\n", VehicleControlModeMsg.ControlForceEnabled);
//                OS_printf("ControlAccelerationEnabled = %u\n", VehicleControlModeMsg.ControlAccelerationEnabled);
//                OS_printf("ControlVelocityEnabled = %u\n", VehicleControlModeMsg.ControlVelocityEnabled);
//                OS_printf("ControlPositionEnabled = %u\n", VehicleControlModeMsg.ControlPositionEnabled);
//                OS_printf("ControlAltitudeEnabled = %u\n", VehicleControlModeMsg.ControlAltitudeEnabled);
//                OS_printf("ControlClimbRateEnabled = %u\n", VehicleControlModeMsg.ControlClimbRateEnabled);
//                OS_printf("ControlTerminationEnabled = %u\n", VehicleControlModeMsg.ControlTerminationEnabled);
                break;

            case PX4_POSITION_SETPOINT_TRIPLET_MID:
                memcpy(&PositionSetpointTripletMsg, MsgPtr, sizeof(PositionSetpointTripletMsg));
//                OS_printf("PositionSetpointTripletMsg ************************\n");
//                OS_printf("Timestamp = %llu\n", PositionSetpointTripletMsg.Timestamp);
//                OS_printf("Previous.Timestamp = %llu\n", PositionSetpointTripletMsg.Previous.Timestamp);
//                OS_printf("Previous.Lat = %f\n", PositionSetpointTripletMsg.Previous.Lat);
//                OS_printf("Previous.Lon = %f\n", PositionSetpointTripletMsg.Previous.Lon);
//                OS_printf("Previous.X = %f\n", (double)PositionSetpointTripletMsg.Previous.X);
//                OS_printf("Previous.Y = %f\n", (double)PositionSetpointTripletMsg.Previous.Y);
//                OS_printf("Previous.Z = %f\n", (double)PositionSetpointTripletMsg.Previous.Z);
//                OS_printf("Previous.VX = %f\n", (double)PositionSetpointTripletMsg.Previous.VX);
//                OS_printf("Previous.VY = %f\n", (double)PositionSetpointTripletMsg.Previous.VY);
//                OS_printf("Previous.VZ = %f\n", (double)PositionSetpointTripletMsg.Previous.VZ);
//                OS_printf("Previous.Alt = %f\n", (double)PositionSetpointTripletMsg.Previous.Alt);
//                OS_printf("Previous.Yaw = %f\n", (double)PositionSetpointTripletMsg.Previous.Yaw);
//                OS_printf("Previous.Yawspeed = %f\n", (double)PositionSetpointTripletMsg.Previous.Yawspeed);
//                OS_printf("Previous.LoiterRadius = %f\n", (double)PositionSetpointTripletMsg.Previous.LoiterRadius);
//                OS_printf("Previous.PitchMin = %f\n", (double)PositionSetpointTripletMsg.Previous.PitchMin);
//                OS_printf("Previous.A_X = %f\n", (double)PositionSetpointTripletMsg.Previous.A_X);
//                OS_printf("Previous.A_Y = %f\n", (double)PositionSetpointTripletMsg.Previous.A_Y);
//                OS_printf("Previous.A_Z = %f\n", (double)PositionSetpointTripletMsg.Previous.A_Z);
//                OS_printf("Previous.AcceptanceRadius = %f\n", (double)PositionSetpointTripletMsg.Previous.AcceptanceRadius);
//                OS_printf("Previous.CruisingSpeed = %f\n", (double)PositionSetpointTripletMsg.Previous.CruisingSpeed);
//                OS_printf("Previous.CruisingThrottle = %f\n", (double)PositionSetpointTripletMsg.Previous.CruisingThrottle);
//                OS_printf("Previous.Valid = %u\n", PositionSetpointTripletMsg.Previous.Valid);
//                OS_printf("Previous.Type = %u\n", PositionSetpointTripletMsg.Previous.Type);
//                OS_printf("Previous.PositionValid = %u\n", PositionSetpointTripletMsg.Previous.PositionValid);
//                OS_printf("Previous.VelocityValid = %u\n", PositionSetpointTripletMsg.Previous.VelocityValid);
//                OS_printf("Previous.YawValid = %u\n", PositionSetpointTripletMsg.Previous.YawValid);
//                OS_printf("Previous.DisableMcYawControl = %u\n", PositionSetpointTripletMsg.Previous.DisableMcYawControl);
//                OS_printf("Previous.YawspeedValid = %u\n", PositionSetpointTripletMsg.Previous.YawspeedValid);
//                OS_printf("Previous.LoiterDirection = %i\n", PositionSetpointTripletMsg.Previous.LoiterDirection);
//                OS_printf("Previous.AccelerationValid = %u\n", PositionSetpointTripletMsg.Previous.AccelerationValid);
//                OS_printf("Previous.AccelerationIsForce = %u\n", PositionSetpointTripletMsg.Previous.AccelerationIsForce);
//
//                OS_printf("Current.Timestamp = %llu\n", PositionSetpointTripletMsg.Current.Timestamp);
//                OS_printf("Current.Lat = %f\n", PositionSetpointTripletMsg.Current.Lat);
//                OS_printf("Current.Lon = %f\n", PositionSetpointTripletMsg.Current.Lon);
//                OS_printf("Current.X = %f\n", (double)PositionSetpointTripletMsg.Current.X);
//                OS_printf("Current.Y = %f\n", (double)PositionSetpointTripletMsg.Current.Y);
//                OS_printf("Current.Z = %f\n", (double)PositionSetpointTripletMsg.Current.Z);
//                OS_printf("Current.VX = %f\n", (double)PositionSetpointTripletMsg.Current.VX);
//                OS_printf("Current.VY = %f\n", (double)PositionSetpointTripletMsg.Current.VY);
//                OS_printf("Current.VZ = %f\n", (double)PositionSetpointTripletMsg.Current.VZ);
//                OS_printf("Current.Alt = %f\n", (double)PositionSetpointTripletMsg.Current.Alt);
//                OS_printf("Current.Yaw = %f\n", (double)PositionSetpointTripletMsg.Current.Yaw);
//                OS_printf("Current.Yawspeed = %f\n", (double)PositionSetpointTripletMsg.Current.Yawspeed);
//                OS_printf("Current.LoiterRadius = %f\n", (double)PositionSetpointTripletMsg.Current.LoiterRadius);
//                OS_printf("Current.PitchMin = %f\n", (double)PositionSetpointTripletMsg.Current.PitchMin);
//                OS_printf("Current.A_X = %f\n", (double)PositionSetpointTripletMsg.Current.A_X);
//                OS_printf("Current.A_Y = %f\n", (double)PositionSetpointTripletMsg.Current.A_Y);
//                OS_printf("Current.A_Z = %f\n", (double)PositionSetpointTripletMsg.Current.A_Z);
//                OS_printf("Current.AcceptanceRadius = %f\n", (double)PositionSetpointTripletMsg.Current.AcceptanceRadius);
//                OS_printf("Current.CruisingSpeed = %f\n", (double)PositionSetpointTripletMsg.Current.CruisingSpeed);
//                OS_printf("Current.CruisingThrottle = %f\n", (double)PositionSetpointTripletMsg.Current.CruisingThrottle);
//                OS_printf("Current.Valid = %u\n", PositionSetpointTripletMsg.Current.Valid);
//                OS_printf("Current.Type = %u\n", PositionSetpointTripletMsg.Current.Type);
//                OS_printf("Current.PositionValid = %u\n", PositionSetpointTripletMsg.Current.PositionValid);
//                OS_printf("Current.VelocityValid = %u\n", PositionSetpointTripletMsg.Current.VelocityValid);
//                OS_printf("Current.YawValid = %u\n", PositionSetpointTripletMsg.Current.YawValid);
//                OS_printf("Current.DisableMcYawControl = %u\n", PositionSetpointTripletMsg.Current.DisableMcYawControl);
//                OS_printf("Current.YawspeedValid = %u\n", PositionSetpointTripletMsg.Current.YawspeedValid);
//                OS_printf("Current.LoiterDirection = %i\n", PositionSetpointTripletMsg.Current.LoiterDirection);
//                OS_printf("Current.AccelerationValid = %u\n", PositionSetpointTripletMsg.Current.AccelerationValid);
//                OS_printf("Current.AccelerationIsForce = %u\n", PositionSetpointTripletMsg.Current.AccelerationIsForce);
//
//                OS_printf("Next.Timestamp = %llu\n", PositionSetpointTripletMsg.Next.Timestamp);
//                OS_printf("Next.Lat = %f\n", PositionSetpointTripletMsg.Next.Lat);
//                OS_printf("Next.Lon = %f\n", PositionSetpointTripletMsg.Next.Lon);
//                OS_printf("Next.X = %f\n", (double)PositionSetpointTripletMsg.Next.X);
//                OS_printf("Next.Y = %f\n", (double)PositionSetpointTripletMsg.Next.Y);
//                OS_printf("Next.Z = %f\n", (double)PositionSetpointTripletMsg.Next.Z);
//                OS_printf("Next.VX = %f\n", (double)PositionSetpointTripletMsg.Next.VX);
//                OS_printf("Next.VY = %f\n", (double)PositionSetpointTripletMsg.Next.VY);
//                OS_printf("Next.VZ = %f\n", (double)PositionSetpointTripletMsg.Next.VZ);
//                OS_printf("Next.Alt = %f\n", (double)PositionSetpointTripletMsg.Next.Alt);
//                OS_printf("Next.Yaw = %f\n", (double)PositionSetpointTripletMsg.Next.Yaw);
//                OS_printf("Next.Yawspeed = %f\n", (double)PositionSetpointTripletMsg.Next.Yawspeed);
//                OS_printf("Next.LoiterRadius = %f\n", (double)PositionSetpointTripletMsg.Next.LoiterRadius);
//                OS_printf("Next.PitchMin = %f\n", (double)PositionSetpointTripletMsg.Next.PitchMin);
//                OS_printf("Next.A_X = %f\n", (double)PositionSetpointTripletMsg.Next.A_X);
//                OS_printf("Next.A_Y = %f\n", (double)PositionSetpointTripletMsg.Next.A_Y);
//                OS_printf("Next.A_Z = %f\n", (double)PositionSetpointTripletMsg.Next.A_Z);
//                OS_printf("Next.AcceptanceRadius = %f\n", (double)PositionSetpointTripletMsg.Next.AcceptanceRadius);
//                OS_printf("Next.CruisingSpeed = %f\n", (double)PositionSetpointTripletMsg.Next.CruisingSpeed);
//                OS_printf("Next.CruisingThrottle = %f\n", (double)PositionSetpointTripletMsg.Next.CruisingThrottle);
//                OS_printf("Next.Valid = %u\n", PositionSetpointTripletMsg.Next.Valid);
//                OS_printf("Next.Type = %u\n", PositionSetpointTripletMsg.Next.Type);
//                OS_printf("Next.PositionValid = %u\n", PositionSetpointTripletMsg.Next.PositionValid);
//                OS_printf("Next.VelocityValid = %u\n", PositionSetpointTripletMsg.Next.VelocityValid);
//                OS_printf("Next.YawValid = %u\n", PositionSetpointTripletMsg.Next.YawValid);
//                OS_printf("Next.DisableMcYawControl = %u\n", PositionSetpointTripletMsg.Next.DisableMcYawControl);
//                OS_printf("Next.YawspeedValid = %u\n", PositionSetpointTripletMsg.Next.YawspeedValid);
//                OS_printf("Next.LoiterDirection = %i\n", PositionSetpointTripletMsg.Next.LoiterDirection);
//                OS_printf("Next.AccelerationValid = %u\n", PositionSetpointTripletMsg.Next.AccelerationValid);
//                OS_printf("Next.AccelerationIsForce = %u\n", PositionSetpointTripletMsg.Next.AccelerationIsForce);
                ProcessPositionSetpointTripletMsg();
                break;

            case PX4_VEHICLE_STATUS_MID:
                memcpy(&VehicleStatusMsg, MsgPtr, sizeof(VehicleStatusMsg));
//                OS_printf("VehicleStatusMsg ************************\n");
//                OS_printf("Timestamp = %llu\n", VehicleStatusMsg.Timestamp);
//                OS_printf("SystemID = %u\n", VehicleStatusMsg.SystemID);
//                OS_printf("ComponentID = %u\n", VehicleStatusMsg.ComponentID);
//                OS_printf("OnboardControlSensorsPresent = %u\n", VehicleStatusMsg.OnboardControlSensorsPresent);
//                OS_printf("OnboardControlSensorsEnabled = %u\n", VehicleStatusMsg.OnboardControlSensorsEnabled);
//                OS_printf("OnboardControlSensorsHealth = %u\n", VehicleStatusMsg.OnboardControlSensorsHealth);
//                OS_printf("NavState = %u\n", VehicleStatusMsg.NavState);
//                OS_printf("ArmingState = %u\n", VehicleStatusMsg.ArmingState);
//                OS_printf("HilState = %u\n", VehicleStatusMsg.HilState);
//                OS_printf("Failsafe = %u\n", VehicleStatusMsg.Failsafe);
//                OS_printf("SystemType = %u\n", VehicleStatusMsg.SystemType);
//                OS_printf("IsRotaryWing = %u\n", VehicleStatusMsg.IsRotaryWing);
//                OS_printf("IsVtol = %u\n", VehicleStatusMsg.IsVtol);
//                OS_printf("VtolFwPermanentStab = %u\n", VehicleStatusMsg.VtolFwPermanentStab);
//                OS_printf("InTransitionMode = %u\n", VehicleStatusMsg.InTransitionMode);
//                OS_printf("RcSignalLost = %u\n", VehicleStatusMsg.RcSignalLost);
//                OS_printf("RcInputMode = %u\n", VehicleStatusMsg.RcInputMode);
//                OS_printf("DataLinkLost = %u\n", VehicleStatusMsg.DataLinkLost);
//                OS_printf("DataLinkLostCounter = %u\n", VehicleStatusMsg.DataLinkLostCounter);
//                OS_printf("EngineFailure = %u\n", VehicleStatusMsg.EngineFailure);
//                OS_printf("EngineFailureCmd = %u\n", VehicleStatusMsg.EngineFailureCmd);
//                OS_printf("MissionFailure = %u\n", VehicleStatusMsg.MissionFailure);

                break;

            case PX4_VEHICLE_LAND_DETECTED_MID:
                memcpy(&VehicleLandDetectedMsg, MsgPtr, sizeof(VehicleLandDetectedMsg));
//                OS_printf("VehicleStatusMsg ************************\n");
//                OS_printf("Timestamp = %llu\n", VehicleLandDetectedMsg.Timestamp);
//                OS_printf("AltMax = %f\n", (double)VehicleLandDetectedMsg.AltMax);
//                OS_printf("Landed = %u\n", VehicleLandDetectedMsg.Landed);
//                OS_printf("Freefall = %u\n", VehicleLandDetectedMsg.Freefall);
//                OS_printf("GroundContact = %u\n", VehicleLandDetectedMsg.GroundContact);
                break;

            case PX4_VEHICLE_LOCAL_POSITION_MID:
                memcpy(&VehicleLocalPositionMsg, MsgPtr, sizeof(VehicleLocalPositionMsg));
//                OS_printf("VehicleLocalPositionMsg ************************\n");
//                OS_printf("Timestamp = %llu\n", VehicleLocalPositionMsg.Timestamp);
//                OS_printf("RefTimestamp = %llu\n", VehicleLocalPositionMsg.RefTimestamp);
//                OS_printf("RefLat = %f\n", (double)VehicleLocalPositionMsg.RefLat);
//                OS_printf("RefLon = %f\n", (double)VehicleLocalPositionMsg.RefLon);
//                OS_printf("SurfaceBottomTimestamp = %llu\n", VehicleLocalPositionMsg.SurfaceBottomTimestamp);
//                OS_printf("X = %f\n", (double)VehicleLocalPositionMsg.X);
//                OS_printf("Y = %f\n", (double)VehicleLocalPositionMsg.Y);
//                OS_printf("Z = %f\n", (double)VehicleLocalPositionMsg.Z);
//                OS_printf("VX = %f\n", (double)VehicleLocalPositionMsg.VX);
//                OS_printf("VY = %f\n", (double)VehicleLocalPositionMsg.VY);
//                OS_printf("VZ = %f\n", (double)VehicleLocalPositionMsg.VZ);
//                OS_printf("Yaw = %f\n", (double)VehicleLocalPositionMsg.Yaw);
//                OS_printf("RefAlt = %f\n", (double)VehicleLocalPositionMsg.RefAlt);
//                OS_printf("DistBottom = %f\n", (double)VehicleLocalPositionMsg.DistBottom);
//                OS_printf("DistBottomRate = %f\n", (double)VehicleLocalPositionMsg.DistBottomRate);
//                OS_printf("EpH = %f\n", (double)VehicleLocalPositionMsg.EpH);
//                OS_printf("EpV = %f\n", (double)VehicleLocalPositionMsg.EpV);
//                OS_printf("XY_Valid = %u\n", VehicleLocalPositionMsg.XY_Valid);
//                OS_printf("Z_Valid = %u\n", VehicleLocalPositionMsg.Z_Valid);
//                OS_printf("V_XY_Valid = %u\n", VehicleLocalPositionMsg.V_XY_Valid);
//                OS_printf("V_Z_Valid = %u\n", VehicleLocalPositionMsg.V_Z_Valid);
//                OS_printf("XY_Global = %u\n", VehicleLocalPositionMsg.XY_Global);
//                OS_printf("Z_Global = %u\n", VehicleLocalPositionMsg.Z_Global);
//                OS_printf("DistBottomValid = %u\n", VehicleLocalPositionMsg.DistBottomValid);
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



void MPC::ProcessVehicleLocalPositionMsg(void)
{
	/* Check if a reset event has happened if the vehicle is in manual mode
	 * we will shift the setpoints of the states which were reset. In auto
	 * mode we do not shift the setpoints since we want the vehicle to track
	 * the original state.
	 */
	if (VehicleControlModeMsg.ControlManualEnabled)
	{
		if (Z_ResetCounter != VehicleLocalPositionMsg.Z_ResetCounter)
		{
			PositionSetpoint[2] += VehicleLocalPositionMsg.Delta_Z;
		}

		if (XY_ResetCounter != VehicleLocalPositionMsg.XY_ResetCounter) {
			PositionSetpoint[0] += VehicleLocalPositionMsg.Delta_XY[0];
			PositionSetpoint[1] += VehicleLocalPositionMsg.Delta_XY[1];
		}

		if (VZ_ResetCounter != VehicleLocalPositionMsg.VZ_ResetCounter) {
			VelocitySetpoint[2] += VehicleLocalPositionMsg.Delta_VZ;
			VelocitySetpointPrevious[2] +=  VehicleLocalPositionMsg.Delta_VZ;
		}

		if (VXY_ResetCounter != VehicleLocalPositionMsg.VXY_ResetCounter) {
			VelocitySetpoint[0] += VehicleLocalPositionMsg.Delta_VXY[0];
			VelocitySetpoint[1] += VehicleLocalPositionMsg.Delta_VXY[1];
			VelocitySetpointPrevious[0] += VehicleLocalPositionMsg.Delta_VXY[0];
			VelocitySetpointPrevious[1] += VehicleLocalPositionMsg.Delta_VXY[1];
		}
	}

	/* Update the reset counters in any case. */
	Z_ResetCounter = VehicleLocalPositionMsg.Z_ResetCounter;
	XY_ResetCounter = VehicleLocalPositionMsg.XY_ResetCounter;
	VZ_ResetCounter = VehicleLocalPositionMsg.VZ_ResetCounter;
	VXY_ResetCounter = VehicleLocalPositionMsg.VXY_ResetCounter;
}



void MPC::ProcessPositionSetpointTripletMsg(void)
{
	/* Set current position setpoint invalid if none of them (lat, lon and
	 * alt) is finite. */
	if (!isfinite(PositionSetpointTripletMsg.Current.Lat) &&
	    !isfinite(PositionSetpointTripletMsg.Current.Lon) &&
	    !isfinite(PositionSetpointTripletMsg.Current.Alt))
	{
		PositionSetpointTripletMsg.Current.Valid = false;
	}
}


void MPC::Execute(void)
{
	static uint64 t_prev = 0;
	bool was_armed = false;
	bool was_landed = true;

	uint64 t = PX4LIB_GetPX4TimeUs();
	float dt = t_prev != 0 ? (t - t_prev) / 1e6f : 0.004f;
	t_prev = t;

	/* Set default max velocity in xy to vel_max */
	VelMaxXY = ConfigTblPtr->XY_VEL_MAX;

	if (VehicleControlModeMsg.Armed && !was_armed) {
		/* reset setpoints and integrals on arming */
		ResetPositionSetpoint = true;
		ResetAltitudeSetpoint = true;
		DoResetAltPos = true;
		VelocitySetpointPrevious.Zero();
		ResetIntZ = true;
		ResetIntXY = true;
		ResetYawSetpoint = true;
		YawTakeoff = Yaw;
	}

	/* Switch to smooth takeoff if we got out of landed state */
	if (!VehicleLandDetectedMsg.Landed && was_landed)
	{
		InTakeoff = true;
		TakeoffVelLimit = -0.5f;
	}

	/* Set triplets to invalid if we just landed */
	if (VehicleLandDetectedMsg.Landed && !was_landed)
	{
		PositionSetpointTripletMsg.Current.Valid = false;
	}

	was_landed = VehicleLandDetectedMsg.Landed;

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
		LimitVelXY = false;

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

	/* Publish attitude setpoint
	 * Do not publish if offboard is enabled but position/velocity/accel
	 * control is disabled, in this case the attitude setpoint is
	 * published by the mavlink app. Also do not publish if the vehicle is a
	 * VTOL and it's just doing a transition (the VTOL attitude control
	 * module will generate attitude setpoints for the transition).
	 */
	if (!(VehicleControlModeMsg.ControlOffboardEnabled &&
	      !(VehicleControlModeMsg.ControlPositionEnabled ||
	    		  VehicleControlModeMsg.ControlVelocityEnabled ||
				  VehicleControlModeMsg.ControlAccelerationEnabled)))
	{

		SendVehicleAttitudeSetpointMsg();
	}

	/* Reset altitude controller integral (hovering throttle) to manual
	 * throttle after manual throttle control */
	ResetIntZManual = VehicleControlModeMsg.Armed && VehicleControlModeMsg.ControlManualEnabled
			      && !VehicleControlModeMsg.ControlClimbRateEnabled;

}



void MPC::UpdateRef(void)
{
	if (VehicleLocalPositionMsg.RefTimestamp != RefTimestamp)
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



void MPC::UpdateVelocityDerivative(float dt)
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
	}

	VelocityErrD[0] = VelXDeriv.Update(-Velocity[0], dt, ConfigTblPtr->VELD_LP);
	VelocityErrD[1] = VelYDeriv.Update(-Velocity[1], dt, ConfigTblPtr->VELD_LP);
	VelocityErrD[2] = VelZDeriv.Update(-Velocity[2], dt, ConfigTblPtr->VELD_LP);
}



void MPC::DoControl(float dt)
{
	VelocityFF.Zero();

	/* By default, run position/altitude controller. the control_* functions
	 * can disable this and run velocity controllers directly in this cycle */
	RunPosControl = true;
	RunAltControl = true;

	/* If not in auto mode, we reset limit_vel_xy flag. */
	if(VehicleControlModeMsg.ControlManualEnabled || VehicleControlModeMsg.ControlOffboardEnabled)
	{
		LimitVelXY = false;
	}

	if (VehicleControlModeMsg.ControlManualEnabled)
	{
		/* Manual control */
		ControlManual(dt);
		ModeAuto = false;

		/* We set tiplets to false.  This ensures that when switching to auto,
		 * the position controller will not use the old triplets but waits
		 * until triplets have been updated. */
		PositionSetpointTripletMsg.Current.Valid = false;

		HoldOffboardXY = false;
		HoldOffboardZ = false;
	}
	else
	{
		ControlNonManual(dt);
	}
}



void MPC::GenerateAttitudeSetpoint(float dt)
{
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
		VehicleAttitudeSetpointMsg.RollBody = ManualControlSetpointMsg.Y * ConfigTblPtr->MAN_TILT_MAX;
		VehicleAttitudeSetpointMsg.PitchBody = -ManualControlSetpointMsg.X * ConfigTblPtr->MAN_TILT_MAX;

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
			math::Matrix3F3 R_sp_roll_pitch;
			R_sp_roll_pitch.FromEuler(VehicleAttitudeSetpointMsg.RollBody, VehicleAttitudeSetpointMsg.PitchBody, 0);
			math::Vector3F z_roll_pitch_sp = R_sp_roll_pitch * zB;

			/* Transform the vector into a new frame which is rotated around the z axis
			 * by the current yaw error. this vector defines the desired tilt when we look
			 * into the direction of the desired heading.
			 */
			math::Matrix3F3 R_yaw_correction;
			R_yaw_correction.FromEuler(0.0f, 0.0f, -yaw_error);
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

	/* Only switch the landing gear up if we are not landed and if
	 * the user switched from gear down to gear up.
	 * If the user had the switch in the gear up position and took off ignore it
     * until he toggles the switch to avoid retracting the gear immediately on takeoff.
     */
	if (ManualControlSetpointMsg.GearSwitch == PX4_SWITCH_POS_ON && GearStateInitialized &&
	    !VehicleLandDetectedMsg.Landed)
	{
		VehicleAttitudeSetpointMsg.LandingGear = 1.0f;

	}
	else if(ManualControlSetpointMsg.GearSwitch == PX4_SWITCH_POS_ON)
	{
		VehicleAttitudeSetpointMsg.LandingGear = -1.0f;
		/* Switching the gear off does put it into a safe defined state. */
		GearStateInitialized = true;
	}

	VehicleAttitudeSetpointMsg.Timestamp = PX4LIB_GetPX4TimeUs();
}



void MPC::ControlManual(float dt)
{
	/* Velocity setpoint commanded by user stick input. */
	math::Vector3F man_vel_sp;

	/* Entering manual control from non-manual control mode, reset alt/pos setpoints */
	if (ModeAuto)
	{
		ModeAuto = false;

		/* Reset alt pos flags if resetting is enabled. */
		if (DoResetAltPos)
		{
			ResetPositionSetpoint = true;
			ResetAltitudeSetpoint = true;
		}
	}

	/*
	 * Map from stick input to velocity setpoint.
	 */

	if(VehicleControlModeMsg.ControlAltitudeEnabled)
	{
		/* Set vertical velocity setpoint with throttle stick, remapping of
		 * manual.z [0,1] to up and down command [-1,1] */
		man_vel_sp[2] = -math::expof_deadzone(
				(ManualControlSetpointMsg.Z - 0.5f) * 2.0f,
				ConfigTblPtr->XY_MAN_EXPO, ConfigTblPtr->HOLD_DZ);

		/* Reset alt setpoint to current altitude if needed. */
		ResetAltSetpoint();
	}

	if (VehicleControlModeMsg.ControlPositionEnabled)
	{
		float man_vel_hor_length;
		math::Vector2F man_vel_hor;

		/* Set horizontal velocity setpoint with roll/pitch stick */
		man_vel_sp[0] = math::expof_deadzone(
				ManualControlSetpointMsg.X,
				ConfigTblPtr->XY_MAN_EXPO, ConfigTblPtr->HOLD_DZ);
		man_vel_sp[1] = math::expof_deadzone(
				ManualControlSetpointMsg.Y,
				ConfigTblPtr->XY_MAN_EXPO, ConfigTblPtr->HOLD_DZ);

		/* Get the horizontal component of the velocity vector. */
		man_vel_hor[0] = man_vel_sp[0];
		man_vel_hor[1] = man_vel_sp[1];

		/* Get the magnitude of the horizontal component. */
		man_vel_hor_length = man_vel_hor.Length();

		/* Saturate such that magnitude is never larger than 1 */
		if (man_vel_hor_length > 1.0f)
		{
			man_vel_sp[0] /= man_vel_hor_length;
			man_vel_sp[1] /= man_vel_hor_length;
		}

		/* Reset position setpoint to current position if needed */
		ResetPosSetpoint();
	}

	/* Prepare yaw to rotate into NED frame */
	float yaw_input_frame = VehicleControlModeMsg.ControlFixedHdgEnabled ? YawTakeoff : VehicleAttitudeSetpointMsg.YawBody;

	/* Prepare cruise speed (m/s) vector to scale the velocity setpoint */
	float vel_mag = (ConfigTblPtr->VEL_MAN_MAX < VelMaxXY) ? ConfigTblPtr->VEL_MAN_MAX : VelMaxXY;
	math::Vector3F vel_cruise_scale(vel_mag, vel_mag, (man_vel_sp[2] > 0.0f) ? ConfigTblPtr->Z_VEL_MAX_DN : ConfigTblPtr->Z_VEL_MAX_UP);

	/* Setpoint in NED frame and scaled to cruise velocity */
	man_vel_sp = math::Matrix3F3::FromEuler(0.0f, 0.0f, yaw_input_frame) * man_vel_sp.EMult(vel_cruise_scale);

	/*
	 * Assisted velocity mode: User controls velocity, but if velocity is small enough, position
	 * hold is activated for the corresponding axis.
	 */

	/* Want to get/stay in altitude hold if user has z stick in the middle (accounted for deadzone already) */
	const bool alt_hold_desired = VehicleControlModeMsg.ControlAltitudeEnabled && fabsf(man_vel_sp[2]) < FLT_EPSILON;

	/* Want to get/stay in position hold if user has xy stick in the middle (accounted for deadzone already) */
	const bool pos_hold_desired = VehicleControlModeMsg.ControlPositionEnabled &&
				      fabsf(man_vel_sp[0]) < FLT_EPSILON && fabsf(man_vel_sp[1]) < FLT_EPSILON;

	/* Check vertical hold engaged flag. */
	if (AltitudeHoldEngaged)
	{
		AltitudeHoldEngaged = alt_hold_desired;
	}
	else
	{
		/* Check if we switch to alt_hold_engaged. */
		bool smooth_alt_transition = AltitudeHoldEngaged &&
					     (ConfigTblPtr->HOLD_MAX_Z < FLT_EPSILON || fabsf(Velocity[2]) < ConfigTblPtr->HOLD_MAX_Z);

		/* During transition predict setpoint forward. */
		if (smooth_alt_transition)
		{
			/* Get max acceleration. */
			float max_acc_z = (Velocity[2] < 0.0f ? ConfigTblPtr->ACC_DOWN_MAX : -ConfigTblPtr->ACC_UP_MAX);

			/* Time to travel from current velocity to zero velocity. */
			float delta_t = fabsf(Velocity[2] / max_acc_z);

			/* Set desired position setpoint assuming max acceleraiton. */
			PositionSetpoint[2] = PositionSetpoint[2] + Velocity[2] * delta_t + 0.5f * max_acc_z * delta_t * delta_t;

			AltitudeHoldEngaged = true;
		}
	}

	/* Check horizontal hold engaged flag. */
	if (PositionHoldEngaged)
	{
		PositionHoldEngaged = pos_hold_desired;
	}
	else
	{
		/* Check if we switch to pos_hold_engaged. */
		float vel_xy_mag = sqrtf(Velocity[0] * Velocity[0] + Velocity[1] * Velocity[1]);
		bool smooth_pos_transition = pos_hold_desired &&
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

	if (VehicleLandDetectedMsg.Landed)
	{
		/* Don't run controller when landed */
		ResetPositionSetpoint = true;
		ResetAltitudeSetpoint = true;
		ModeAuto = false;
		ResetIntZ = true;
		ResetIntXY = true;

		RSetpoint.Identity();

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



void MPC::ControlNonManual(float dt)
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

	/* Weather-vane mode for vtol: disable yaw control */
	if (VehicleStatusMsg.IsVtol)
	{
		VehicleAttitudeSetpointMsg.DisableMcYawControl = PositionSetpointTripletMsg.Current.DisableMcYawControl;
	}
	else
	{
		VehicleAttitudeSetpointMsg.DisableMcYawControl = false;
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



float MPC::ThrottleCurve(float ctl, float ctr)
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



void MPC::ResetAltSetpoint(void)
{
	if (ResetAltitudeSetpoint)
	{
		ResetAltitudeSetpoint = false;

		/* We have logic in the main function which chooses the velocity setpoint such that the attitude setpoint is
		 * continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		 * altitude in a special way. */
		PositionSetpoint[0] = Position[0];
		PositionSetpoint[1] = Position[1];
	}
}



void MPC::ResetPosSetpoint(void)
{
	if (ResetPositionSetpoint)
	{
		ResetAltitudeSetpoint = false;

		/* We have logic in the main function which chooses the velocity setpoint such that the attitude setpoint is
		 * continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		 * position in a special way. In position control mode the position will be reset anyway until the vehicle has reduced speed.
		 */
		PositionSetpoint[2] = Position[2];
	}
}



void MPC::ControlPosition(float dt)
{
	/* TODO */
	//calculate_velocity_setpoint(dt);

	if (VehicleControlModeMsg.ControlClimbRateEnabled || VehicleControlModeMsg.ControlVelocityEnabled ||
			VehicleControlModeMsg.ControlAccelerationEnabled)
	{
		/* TODO */
		//calculate_thrust_setpoint(dt);
	}
	else
	{
		ResetIntZ = true;
	}
}



void MPC::ControlOffboard(float dt)
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
					/* TODO:  Replace with a CFE Event. */
					OS_printf("Unknown velocity offboard coordinate frame");
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



void MPC::ControlAuto(float dt)
{
	/* Reset position setpoint on AUTO mode activation or if we are not in
	 * MC mode */
	if (!ModeAuto || !VehicleStatusMsg.IsRotaryWing)
	{
		if (!ModeAuto)
		{
			ModeAuto = true;
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

	math::Vector3F prev_sp;
	math::Vector3F next_sp;

	if (PositionSetpointTripletMsg.Current.Valid)
	{
		/* Only project setpoints if they are finite, else use current
		 * position. */
		if (isfinite(PositionSetpointTripletMsg.Current.Lat) &&
		    isfinite(PositionSetpointTripletMsg.Current.Lon))
		{
			/* Project setpoint to local frame. */
			map_projection_project(&RefPos,
					PositionSetpointTripletMsg.Current.Lat, PositionSetpointTripletMsg.Current.Lon,
					       &CurrentPositionSetpoint[0], &CurrentPositionSetpoint[1]);
		}
		else
		{
			CurrentPositionSetpoint[0] = Position[0];
			CurrentPositionSetpoint[1] = Position[1];
		}

		/* Only project setpoints if they are finite, else use current position. */
		if (isfinite(PositionSetpointTripletMsg.Current.Alt))
		{
			CurrentPositionSetpoint[2] = -(PositionSetpointTripletMsg.Current.Alt - RefAlt);
		}
		else
		{
			CurrentPositionSetpoint[2] = Position[2];
		}

		if (isfinite(CurrentPositionSetpoint[0]) &&
				isfinite(CurrentPositionSetpoint[1]) &&
				isfinite(CurrentPositionSetpoint[2]))
		{
			current_setpoint_valid = true;
		}
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
			previous_setpoint_valid = true;
		}
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

	/* Set velocity limit if close to current setpoint and no next setpoint available. */
	math::Vector3F dist = CurrentPositionSetpoint - Position;
	LimitVelXY = (!next_setpoint_valid || (PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER))
			&& (sqrtf(dist[0] * dist[0] + dist[1] * dist[1]) <= ConfigTblPtr->TARGET_THRE);

	if (current_setpoint_valid &&
	    (PositionSetpointTripletMsg.Current.Type != PX4_SETPOINT_TYPE_IDLE))
	{
		/* TODO:  Implement get_cruising_speed_xy() and remove temporary stub. */
		float cruising_speed_xy = 0.0f;
		//float cruising_speed_xy = get_cruising_speed_xy();
		float cruising_speed_z = (CurrentPositionSetpoint[2] > Position[2]) ? ConfigTblPtr->Z_VEL_MAX_DN : ConfigTblPtr->Z_VEL_MAX_UP;

		/* Scaled space: 1 == position error resulting max allowed speed. */
		math::Vector3F cruising_speed(cruising_speed_xy, cruising_speed_xy, cruising_speed_z);

		/* If previous is valid, we want to follow line. */
		if (previous_setpoint_valid
		    && (PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_POSITION  ||
		    		PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER ||
					PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_FOLLOW_TARGET))
		{
			/* TODO - Implement the line below and remove stub. */
			//math::Vector3F scale = _params.pos_p.edivide(cruising_speed);
            math::Vector3F scale(0.0f, 0.0f, 0.0f);

			/* Convert current setpoint to scaled space. */
			math::Vector3F curr_sp_s = CurrentPositionSetpoint.EMult(scale);

			/* By default, use current setpoint as is. */
			math::Vector3F pos_sp_s = curr_sp_s;

			const float minimum_dist = 0.01f;

			if ((CurrentPositionSetpoint - prev_sp).Length() > minimum_dist)
			{
				/* find X - cross point of unit sphere and trajectory */
				math::Vector3F pos_s = Position.EMult(scale);
				math::Vector3F prev_sp_s = prev_sp.EMult(scale);
				math::Vector3F prev_curr_s = curr_sp_s - prev_sp_s;
				math::Vector3F curr_pos_s = pos_s - curr_sp_s;
				float curr_pos_s_len = curr_pos_s.Length();

				/* We are close to current setpoint. */
				if (curr_pos_s_len < 1.0f)
				{
					/* If next is valid, we want to have smooth transition. */
					if (next_setpoint_valid && (next_sp - CurrentPositionSetpoint).Length() > minimum_dist)
					{
						math::Vector3F next_sp_s = next_sp.EMult(scale);

						/* Calculate angle prev - curr - next */
						math::Vector3F curr_next_s = next_sp_s - curr_sp_s;
						math::Vector3F prev_curr_s_norm = prev_curr_s.Normalized();

						/* cos(a) * curr_next, a = angle between current and next trajectory segments */
						float cos_a_curr_next = prev_curr_s_norm * curr_next_s;

						/* cos(b), b = angle pos - _curr_pos_sp - prev_sp */
						float cos_b = -curr_pos_s * prev_curr_s_norm / curr_pos_s_len;

						if (cos_a_curr_next > 0.0f && cos_b > 0.0f)
						{
							float curr_next_s_len = curr_next_s.Length();

							/* If curr - next distance is larger than unit radius, limit it. */
							if (curr_next_s_len > 1.0f)
							{
								cos_a_curr_next /= curr_next_s_len;
							}

							/* Feed forward position setpoint offset. */
							math::Vector3F pos_ff = prev_curr_s_norm *
										 cos_a_curr_next * cos_b * cos_b * (1.0f - curr_pos_s_len) *
										 (1.0f - expf(-curr_pos_s_len * curr_pos_s_len * 20.0f));
							pos_sp_s = pos_sp_s + pos_ff;
						}
					}
				}
				else
				{
					/* If not close to current setpoint, check if we are
					 * within cross_sphere_line. */
					/* TODO - Implement cross_sphere_line and replace stub. */
					//bool near = cross_sphere_line(pos_s, 1.0f, prev_sp_s, curr_sp_s, pos_sp_s);
					bool near = false;

					if (!near)
					{
						/* We're far away from trajectory, pos_sp_s is set to
						 * the nearest point on the trajectory */
						pos_sp_s = pos_s + (pos_sp_s - pos_s).Normalized();
					}
				}
			}

			/* Move setpoint not faster than max allowed speed. */
			math::Vector3F pos_sp_old_s = PositionSetpoint.EMult(scale);

			/* Difference between current and desired position setpoints, 1 = max speed. */
			/* TODO:  Implement edivide and replace stub. */
			//math::Vector3F d_pos_m = (pos_sp_s - pos_sp_old_s).edivide(_params.pos_p);
			math::Vector3F d_pos_m(0.0f, 0.0f, 0.0f);
			float d_pos_m_len = d_pos_m.Length();

			if (d_pos_m_len > dt)
			{
				/* TODO:  Implement _params.pos_p */
				//pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt).EMult(_params.pos_p);
			}

			/* Scale back */
			/* TODO:  Implement edivide. */
			//PositionSetpoint = pos_sp_s.edivide(scale);
		}
		else
		{
			/* We just have a current setpoint that we want to go to. */
			PositionSetpoint = CurrentPositionSetpoint;

			/* Set max velocity to cruise. */
			VelMaxXY = cruising_speed[0];
		}

		/* Update yaw setpoint if needed. */
		if (PositionSetpointTripletMsg.Current.YawspeedValid
		    && PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_FOLLOW_TARGET)
		{
			VehicleAttitudeSetpointMsg.YawBody = VehicleAttitudeSetpointMsg.YawBody + PositionSetpointTripletMsg.Current.Yawspeed * dt;
		}
		else if (isfinite(PositionSetpointTripletMsg.Current.Yaw))
		{
			VehicleAttitudeSetpointMsg.YawBody = PositionSetpointTripletMsg.Current.Yaw;
		}

		/*
		 * If we're already near the current takeoff setpoint don't reset in case we switch back to posctl.
		 * this makes the takeoff finish smoothly.
		 */
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
			/* Otherwise: in case of interrupted mission don't go to waypoint but stay at current position. */
			DoResetAltPos = true;
		}

		/* Handle the landing gear based on the manual landing alt. */
		const bool high_enough_for_landing_gear = (Position[2] < ConfigTblPtr->MIS_LTRMIN_ALT * 2.0f);

		/* During a mission or in loiter it's safe to retract the landing gear. */
		if ((PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_POSITION ||
				PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER) &&
		    !VehicleLandDetectedMsg.Landed &&
		    high_enough_for_landing_gear)
		{
			VehicleAttitudeSetpointMsg.LandingGear = 1.0f;
		}
		else if (PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_TAKEOFF ||
				PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LAND ||
			   !high_enough_for_landing_gear)
		{
			/* During takeoff and landing, we better put it down again. */
			VehicleAttitudeSetpointMsg.LandingGear = -1.0f;
		}
		else
		{
			/* For the rest of the setpoint types, just leave it as is. */
		}
	}
	else
	{
		/* Idle or triplet not valid, set velocity setpoint to zero */
		VelocitySetpoint.Zero();
		RunPosControl = false;
		RunAltControl = false;
	}
}


/************************/
/*  End of File Comment */
/************************/
