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
#include "lib/mathlib/math/Limits.hpp"

#define nan FP_NAN


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Local definitions                                               */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define MPC_CONSTANTS_ONE_G    9.80665f   /* m/s^2		*/


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
                OS_printf("PositionSetpointTripletMsg ************************\n");
                OS_printf("Timestamp = %llu\n", PositionSetpointTripletMsg.Timestamp);
                OS_printf("Previous.Timestamp = %llu\n", PositionSetpointTripletMsg.Previous.Timestamp);
                OS_printf("Previous.Lat = %f\n", PositionSetpointTripletMsg.Previous.Lat);
                OS_printf("Previous.Lon = %f\n", PositionSetpointTripletMsg.Previous.Lon);
                OS_printf("Previous.X = %f\n", (double)PositionSetpointTripletMsg.Previous.X);
                OS_printf("Previous.Y = %f\n", (double)PositionSetpointTripletMsg.Previous.Y);
                OS_printf("Previous.Z = %f\n", (double)PositionSetpointTripletMsg.Previous.Z);
                OS_printf("Previous.VX = %f\n", (double)PositionSetpointTripletMsg.Previous.VX);
                OS_printf("Previous.VY = %f\n", (double)PositionSetpointTripletMsg.Previous.VY);
                OS_printf("Previous.VZ = %f\n", (double)PositionSetpointTripletMsg.Previous.VZ);
                OS_printf("Previous.Alt = %f\n", (double)PositionSetpointTripletMsg.Previous.Alt);
                OS_printf("Previous.Yaw = %f\n", (double)PositionSetpointTripletMsg.Previous.Yaw);
                OS_printf("Previous.Yawspeed = %f\n", (double)PositionSetpointTripletMsg.Previous.Yawspeed);
                OS_printf("Previous.LoiterRadius = %f\n", (double)PositionSetpointTripletMsg.Previous.LoiterRadius);
                OS_printf("Previous.PitchMin = %f\n", (double)PositionSetpointTripletMsg.Previous.PitchMin);
                OS_printf("Previous.AX = %f\n", (double)PositionSetpointTripletMsg.Previous.AX);
                OS_printf("Previous.AY = %f\n", (double)PositionSetpointTripletMsg.Previous.AY);
                OS_printf("Previous.AZ = %f\n", (double)PositionSetpointTripletMsg.Previous.AZ);
                OS_printf("Previous.AcceptanceRadius = %f\n", (double)PositionSetpointTripletMsg.Previous.AcceptanceRadius);
                OS_printf("Previous.CruisingSpeed = %f\n", (double)PositionSetpointTripletMsg.Previous.CruisingSpeed);
                OS_printf("Previous.CruisingThrottle = %f\n", (double)PositionSetpointTripletMsg.Previous.CruisingThrottle);
                OS_printf("Previous.Valid = %u\n", PositionSetpointTripletMsg.Previous.Valid);
                OS_printf("Previous.Type = %u\n", PositionSetpointTripletMsg.Previous.Type);
                OS_printf("Previous.PositionValid = %u\n", PositionSetpointTripletMsg.Previous.PositionValid);
                OS_printf("Previous.VelocityValid = %u\n", PositionSetpointTripletMsg.Previous.VelocityValid);
                OS_printf("Previous.YawValid = %u\n", PositionSetpointTripletMsg.Previous.YawValid);
                OS_printf("Previous.DisableMcYawControl = %u\n", PositionSetpointTripletMsg.Previous.DisableMcYawControl);
                OS_printf("Previous.YawspeedValid = %u\n", PositionSetpointTripletMsg.Previous.YawspeedValid);
                OS_printf("Previous.LoiterDirection = %i\n", PositionSetpointTripletMsg.Previous.LoiterDirection);
                OS_printf("Previous.AccelerationValid = %u\n", PositionSetpointTripletMsg.Previous.AccelerationValid);
                OS_printf("Previous.AccelerationIsForce = %u\n", PositionSetpointTripletMsg.Previous.AccelerationIsForce);

                OS_printf("Current.Timestamp = %llu\n", PositionSetpointTripletMsg.Current.Timestamp);
                OS_printf("Current.Lat = %f\n", PositionSetpointTripletMsg.Current.Lat);
                OS_printf("Current.Lon = %f\n", PositionSetpointTripletMsg.Current.Lon);
                OS_printf("Current.X = %f\n", (double)PositionSetpointTripletMsg.Current.X);
                OS_printf("Current.Y = %f\n", (double)PositionSetpointTripletMsg.Current.Y);
                OS_printf("Current.Z = %f\n", (double)PositionSetpointTripletMsg.Current.Z);
                OS_printf("Current.VX = %f\n", (double)PositionSetpointTripletMsg.Current.VX);
                OS_printf("Current.VY = %f\n", (double)PositionSetpointTripletMsg.Current.VY);
                OS_printf("Current.VZ = %f\n", (double)PositionSetpointTripletMsg.Current.VZ);
                OS_printf("Current.Alt = %f\n", (double)PositionSetpointTripletMsg.Current.Alt);
                OS_printf("Current.Yaw = %f\n", (double)PositionSetpointTripletMsg.Current.Yaw);
                OS_printf("Current.Yawspeed = %f\n", (double)PositionSetpointTripletMsg.Current.Yawspeed);
                OS_printf("Current.LoiterRadius = %f\n", (double)PositionSetpointTripletMsg.Current.LoiterRadius);
                OS_printf("Current.PitchMin = %f\n", (double)PositionSetpointTripletMsg.Current.PitchMin);
                OS_printf("Current.AX = %f\n", (double)PositionSetpointTripletMsg.Current.AX);
                OS_printf("Current.AY = %f\n", (double)PositionSetpointTripletMsg.Current.AY);
                OS_printf("Current.AZ = %f\n", (double)PositionSetpointTripletMsg.Current.AZ);
                OS_printf("Current.AcceptanceRadius = %f\n", (double)PositionSetpointTripletMsg.Current.AcceptanceRadius);
                OS_printf("Current.CruisingSpeed = %f\n", (double)PositionSetpointTripletMsg.Current.CruisingSpeed);
                OS_printf("Current.CruisingThrottle = %f\n", (double)PositionSetpointTripletMsg.Current.CruisingThrottle);
                OS_printf("Current.Valid = %u\n", PositionSetpointTripletMsg.Current.Valid);
                OS_printf("Current.Type = %u\n", PositionSetpointTripletMsg.Current.Type);
                OS_printf("Current.PositionValid = %u\n", PositionSetpointTripletMsg.Current.PositionValid);
                OS_printf("Current.VelocityValid = %u\n", PositionSetpointTripletMsg.Current.VelocityValid);
                OS_printf("Current.YawValid = %u\n", PositionSetpointTripletMsg.Current.YawValid);
                OS_printf("Current.DisableMcYawControl = %u\n", PositionSetpointTripletMsg.Current.DisableMcYawControl);
                OS_printf("Current.YawspeedValid = %u\n", PositionSetpointTripletMsg.Current.YawspeedValid);
                OS_printf("Current.LoiterDirection = %i\n", PositionSetpointTripletMsg.Current.LoiterDirection);
                OS_printf("Current.AccelerationValid = %u\n", PositionSetpointTripletMsg.Current.AccelerationValid);
                OS_printf("Current.AccelerationIsForce = %u\n", PositionSetpointTripletMsg.Current.AccelerationIsForce);

                OS_printf("Next.Timestamp = %llu\n", PositionSetpointTripletMsg.Next.Timestamp);
                OS_printf("Next.Lat = %f\n", PositionSetpointTripletMsg.Next.Lat);
                OS_printf("Next.Lon = %f\n", PositionSetpointTripletMsg.Next.Lon);
                OS_printf("Next.X = %f\n", (double)PositionSetpointTripletMsg.Next.X);
                OS_printf("Next.Y = %f\n", (double)PositionSetpointTripletMsg.Next.Y);
                OS_printf("Next.Z = %f\n", (double)PositionSetpointTripletMsg.Next.Z);
                OS_printf("Next.VX = %f\n", (double)PositionSetpointTripletMsg.Next.VX);
                OS_printf("Next.VY = %f\n", (double)PositionSetpointTripletMsg.Next.VY);
                OS_printf("Next.VZ = %f\n", (double)PositionSetpointTripletMsg.Next.VZ);
                OS_printf("Next.Alt = %f\n", (double)PositionSetpointTripletMsg.Next.Alt);
                OS_printf("Next.Yaw = %f\n", (double)PositionSetpointTripletMsg.Next.Yaw);
                OS_printf("Next.Yawspeed = %f\n", (double)PositionSetpointTripletMsg.Next.Yawspeed);
                OS_printf("Next.LoiterRadius = %f\n", (double)PositionSetpointTripletMsg.Next.LoiterRadius);
                OS_printf("Next.PitchMin = %f\n", (double)PositionSetpointTripletMsg.Next.PitchMin);
                OS_printf("Next.AX = %f\n", (double)PositionSetpointTripletMsg.Next.AX);
                OS_printf("Next.AY = %f\n", (double)PositionSetpointTripletMsg.Next.AY);
                OS_printf("Next.AZ = %f\n", (double)PositionSetpointTripletMsg.Next.AZ);
                OS_printf("Next.AcceptanceRadius = %f\n", (double)PositionSetpointTripletMsg.Next.AcceptanceRadius);
                OS_printf("Next.CruisingSpeed = %f\n", (double)PositionSetpointTripletMsg.Next.CruisingSpeed);
                OS_printf("Next.CruisingThrottle = %f\n", (double)PositionSetpointTripletMsg.Next.CruisingThrottle);
                OS_printf("Next.Valid = %u\n", PositionSetpointTripletMsg.Next.Valid);
                OS_printf("Next.Type = %u\n", PositionSetpointTripletMsg.Next.Type);
                OS_printf("Next.PositionValid = %u\n", PositionSetpointTripletMsg.Next.PositionValid);
                OS_printf("Next.VelocityValid = %u\n", PositionSetpointTripletMsg.Next.VelocityValid);
                OS_printf("Next.YawValid = %u\n", PositionSetpointTripletMsg.Next.YawValid);
                OS_printf("Next.DisableMcYawControl = %u\n", PositionSetpointTripletMsg.Next.DisableMcYawControl);
                OS_printf("Next.YawspeedValid = %u\n", PositionSetpointTripletMsg.Next.YawspeedValid);
                OS_printf("Next.LoiterDirection = %i\n", PositionSetpointTripletMsg.Next.LoiterDirection);
                OS_printf("Next.AccelerationValid = %u\n", PositionSetpointTripletMsg.Next.AccelerationValid);
                OS_printf("Next.AccelerationIsForce = %u\n", PositionSetpointTripletMsg.Next.AccelerationIsForce);
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

	uint64 t = PX4LIB_GetPX4TimeUs();
	float dt = t_prev != 0 ? (t - t_prev) / 1e6f : 0.004f;
	t_prev = t;

	/* Set default max velocity in xy to vel_max */
	VelMaxXY = ConfigTblPtr->XY_VEL_MAX;

	if (VehicleControlModeMsg.Armed && !WasArmed) {
		/* Reset setpoints and integrals on arming. */
		ResetPositionSetpoint = true;
		ResetAltitudeSetpoint = true;
		DoResetAltPos = true;
		VelocitySetpointPrevious.Zero();
		ResetIntZ = true;
		ResetIntXY = true;
		ResetYawSetpoint = true;
		YawTakeoff = Yaw;
	}

	WasArmed = VehicleControlModeMsg.Armed;

	/* Switch to smooth takeoff if we got out of landed state */
	if (!VehicleLandDetectedMsg.Landed && WasLanded)
	{
		InTakeoff = true;
		TakeoffVelLimit = -0.5f;
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
		const float yaw_rate_max = (math::radians(ConfigTblPtr->MAN_Y_MAX) < math::radians(ConfigTblPtr->MC_YAWRATE_MAX)) ? math::radians(ConfigTblPtr->MAN_Y_MAX) :
				math::radians(ConfigTblPtr->MC_YAWRATE_MAX);
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



void MPC::ResetPosSetpoint(void)
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



void MPC::ResetAltSetpoint(void)
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



void MPC::ControlPosition(float dt)
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
		float cruising_speed_xy = GetCruisingSpeedXY();
		float cruising_speed_z = (CurrentPositionSetpoint[2] > Position[2]) ? ConfigTblPtr->Z_VEL_MAX_DN : ConfigTblPtr->Z_VEL_MAX_UP;

		/* Scaled space: 1 == position error resulting max allowed speed. */
		math::Vector3F cruising_speed(cruising_speed_xy, cruising_speed_xy, cruising_speed_z);

		/* If previous is valid, we want to follow line. */
		if (previous_setpoint_valid
		    && (PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_POSITION  ||
		    		PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LOITER ||
					PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_FOLLOW_TARGET))
		{
			math::Vector3F scale = PosP.EDivide(cruising_speed);

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
					bool near = CrossSphereLine(pos_s, 1.0f, prev_sp_s, curr_sp_s, pos_sp_s);

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
			math::Vector3F d_pos_m = (pos_sp_s - pos_sp_old_s).EDivide(PosP);
			float d_pos_m_len = d_pos_m.Length();

			if (d_pos_m_len > dt)
			{
				pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt).EMult(PosP);
			}

			/* Scale back */
			PositionSetpoint = pos_sp_s.EDivide(scale);
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



void MPC::CalculateVelocitySetpoint(float dt)
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

	LimitAltitude();

	if (RunAltControl)
	{
		VelocitySetpoint[2] = (PositionSetpoint[2] - Position[2]) * PosP[2];
	}

	/* Make sure velocity setpoint is saturated in xy. */
	float vel_norm_xy = sqrtf(VelocitySetpoint[0] * VelocitySetpoint[0] +
			VelocitySetpoint[1] * VelocitySetpoint[1]);

	SlowLandGradualVelocityLimit();

	/* We are close to target and want to limit velocity in xy */
	if (LimitVelXY)
	{
		LimitVelXYGradually();
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

	/* Limit vertical takeoff speed if we are in auto takeoff. */
	if (PositionSetpointTripletMsg.Current.Valid
	    && PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_TAKEOFF
	    && !VehicleControlModeMsg.ControlManualEnabled)
	{
		VelocitySetpoint[2] = math::max(VelocitySetpoint[2], -ConfigTblPtr->TKO_SPEED);
	}

	/* Apply slew rate (aka acceleration limit) for smooth flying. */
	ApplyVelocitySetpointSlewRate(dt);
	VelocitySetpointPrevious = VelocitySetpoint;

	/* Make sure velocity setpoint is constrained in all directions. */
	if (vel_norm_xy > ConfigTblPtr->XY_VEL_MAX)
	{
		VelocitySetpoint[0] = VelocitySetpoint[0] * ConfigTblPtr->XY_VEL_MAX / vel_norm_xy;
		VelocitySetpoint[1] = VelocitySetpoint[1] * ConfigTblPtr->XY_VEL_MAX / vel_norm_xy;
	}

	VelocitySetpoint[2] = math::max(VelocitySetpoint[2], -ConfigTblPtr->Z_VEL_MAX_UP);

	/* Special velocity setpoint limitation for smooth takeoff. */
	if (InTakeoff)
	{
		InTakeoff = TakeoffVelLimit < -VelocitySetpoint[2];
		/* Ramp vertical velocity limit up to takeoff speed. */
		TakeoffVelLimit += ConfigTblPtr->TKO_SPEED * dt / ConfigTblPtr->TKO_RAMP_T;
		/* Limit vertical velocity to the current ramp value. */
		VelocitySetpoint[2] = math::max(VelocitySetpoint[2], -TakeoffVelLimit);
	}

	/* Publish velocity setpoint */
	VehicleGlobalVelocitySetpointMsg.Timestamp = PX4LIB_GetPX4TimeUs();
	VehicleGlobalVelocitySetpointMsg.VX = VelocitySetpoint[0];
	VehicleGlobalVelocitySetpointMsg.VY = VelocitySetpoint[1];
	VehicleGlobalVelocitySetpointMsg.VZ = VelocitySetpoint[2];

	SendVehicleGlobalVelocitySetpointMsg();
}



void MPC::CalculateThrustSetpoint(float dt)
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
	if (VehicleLandDetectedMsg.GroundContact && !InAutoTakeoff())
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

		/* Set velocity setpoint to zero and reset position. */
		VelocitySetpoint[0] = 0.0f;
		VelocitySetpoint[1] = 0.0f;
		PositionSetpoint[0] = Position[0];
		PositionSetpoint[1] = Position[1];
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

	float tilt_max = math::radians(ConfigTblPtr->TILTMAX_AIR);
	float thr_max = ConfigTblPtr->THR_MAX;

	/* Filter vel_z over 1/8sec */
	VelZLp = VelZLp * (1.0f - dt * 8.0f) + dt * 8.0f * Velocity[2];

	/* Filter vel_z change over 1/8sec */
	float vel_z_change = (Velocity[2] - VelocityPrevious[2]) / dt;
	AccZLp = AccZLp * (1.0f - dt * 8.0f) + dt * 8.0f * vel_z_change;

	/* We can only run the control if we're already in-air, have a takeoff setpoint,
	 * or if we're in offboard control.  Otherwise, we should just bail out. */
	if (VehicleLandDetectedMsg.Landed && !InAutoTakeoff())
	{
		/* Keep throttle low while still on ground. */
		thr_max = 0.0f;
	}
	else if (!VehicleControlModeMsg.ControlManualEnabled && PositionSetpointTripletMsg.Current.Valid &&
			PositionSetpointTripletMsg.Current.Type == PX4_SETPOINT_TYPE_LAND)
	{
		/* Adjust limits for landing mode.  Limit max tilt and min lift when landing. */
		tilt_max = math::radians(ConfigTblPtr->TILTMAX_LND);

		if (thr_min < 0.0f)
		{
			thr_min = 0.0f;
		}

		/* Descent stabilized.  We are landing. */
		if (!InLanding && !LndReachedGround
		    && (float)fabsf(AccZLp) < 0.1f
		    && VelZLp > 0.6f * ConfigTblPtr->LAND_SPEED)
		{
			InLanding = true;
		}

		float land_z_threshold = 0.1f;

		/* Assume ground.  cut thrust */
		if (InLanding
		    && VelZLp < land_z_threshold)
		{
			thr_max = 0.0f;
			InLanding = false;
			LndReachedGround = true;
		}
		else if(InLanding && VelZLp < math::min(0.3f * ConfigTblPtr->LAND_SPEED, 2.5f * land_z_threshold))
        {
			/* Not on ground but with ground contact, stop position and velocity control. */
			thrust_sp[0] = 0.0f;
			thrust_sp[1] = 0.0f;
			VelocitySetpoint[0] = Velocity[0];
			VelocitySetpoint[1] = Velocity[1];
			PositionSetpoint[0] = Position[0];
			PositionSetpoint[1] = Position[1];
		}

		/* Once we assumed to have reached the ground, always cut the thrust.
		 * Only free fall detection below can revoke this.
		 */
		if (!InLanding && LndReachedGround)
		{
			thr_max = 0.0f;
		}

		/* If we suddenly fall, reset landing logic and remove thrust limit. */
		if (LndReachedGround
		    /* XXX: magic value, assuming free fall above 4m/s2 acceleration */
		    && (AccZLp > 4.0f
			|| VelZLp > 2.0f * ConfigTblPtr->LAND_SPEED))
		{
			thr_max = ConfigTblPtr->THR_MAX;
			InLanding = true;
			LndReachedGround = false;
		}
	}
	else
	{
		InLanding = false;
		LndReachedGround = false;
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

		if (thrust_sp.Length() > FLT_EPSILON)
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

		if (fabsf(body_z[2]) > FLT_EPSILON)
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



float MPC::GetCruisingSpeedXY(void)
{
	float res;

	/*
	 * In missions, the user can choose cruising speed different to default.
	 */
	res = ((isfinite(PositionSetpointTripletMsg.Current.CruisingSpeed) &&
			(PositionSetpointTripletMsg.Current.CruisingSpeed > 0.1f)) ?
					PositionSetpointTripletMsg.Current.CruisingSpeed : ConfigTblPtr->XY_CRUISE);

	return res;
}



bool MPC::CrossSphereLine(const math::Vector3F &sphere_c, const float sphere_r,
		const math::Vector3F &line_a, const math::Vector3F &line_b, math::Vector3F &res)
{
	bool result;

	/* Project center of sphere on line normalized AB. */
	math::Vector3F ab_norm = line_b - line_a;
	ab_norm.Normalize();
	math::Vector3F d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).Length();

	if (sphere_r > cd_len)
	{
		/* We have triangle CDX with known CD and CX = R, find DX. */
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

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
	/* In altitude control, limit setpoint. */
	if (RunAltControl && PositionSetpoint[2] <= -VehicleLandDetectedMsg.AltMax)
	{
		PositionSetpoint[2] = -VehicleLandDetectedMsg.AltMax;
	}
	else
	{
		/* In velocity control mode and want to fly upwards. */
		if (!RunAltControl && VelocitySetpoint[2] <= 0.0f)
		{
			/* Time to travel to reach zero velocity. */
			float delta_t = -Velocity[2] / ConfigTblPtr->ACC_DOWN_MAX;

			/* Predicted position */
			float pos_z_next = Position[2] + Velocity[2] * delta_t + 0.5f *
					ConfigTblPtr->ACC_DOWN_MAX * delta_t * delta_t;

			if (pos_z_next <= -VehicleLandDetectedMsg.AltMax)
			{
				PositionSetpoint[2] = -VehicleLandDetectedMsg.AltMax;
				RunAltControl = true;
				return;
			}
		}
	}
}



void MPC::SlowLandGradualVelocityLimit(void)
{
	/*
	 * Make sure downward velocity (positive Z) is limited close to ground.
	 * for now we use the home altitude and assume that we want to land on a similar absolute altitude.
	 */
	float altitude_above_home = -Position[2] + HomePositionMsg.Z;
	float vel_limit = ConfigTblPtr->Z_VEL_MAX_DN;

	if (altitude_above_home < ConfigTblPtr->LAND_ALT2)
	{
		vel_limit = ConfigTblPtr->LAND_SPEED;
	}
	else if(altitude_above_home < ConfigTblPtr->LAND_ALT1)
	{
		/* Linear function between the two altitudes. */
		float a = (ConfigTblPtr->Z_VEL_MAX_DN - ConfigTblPtr->LAND_SPEED) / (ConfigTblPtr->LAND_ALT1 - ConfigTblPtr->LAND_ALT2);
		float b = ConfigTblPtr->LAND_SPEED - a * ConfigTblPtr->LAND_ALT2;
		vel_limit =  a * altitude_above_home + b;
	}

	VelocitySetpoint[2] = math::min(VelocitySetpoint[2], vel_limit);
}



void MPC::LimitVelXYGradually(void)
{
	/*
	 * The max velocity is defined by the linear line
	 * with x= (curr_sp - pos) and y = VelocitySetpoint with min limit of 0.01
	 */
	math::Vector3F dist = CurrentPositionSetpoint - Position;
	float slope = (GetCruisingSpeedXY() - 0.01f)  / ConfigTblPtr->TARGET_THRE;
	float vel_limit =  slope * sqrtf(dist[0] * dist[0] + dist[1] * dist[1]) + 0.01f;
	float vel_mag_xy = sqrtf(VelocitySetpoint[0] * VelocitySetpoint[0] + VelocitySetpoint[1] * VelocitySetpoint[1]);

	if (vel_mag_xy <= vel_limit)
	{
		return;
	}

	VelocitySetpoint[0] = VelocitySetpoint[0] / vel_mag_xy * vel_limit;
	VelocitySetpoint[1] = VelocitySetpoint[1] / vel_mag_xy * vel_limit;
}



void MPC::ApplyVelocitySetpointSlewRate(float dt)
{
	math::Vector3F acc = (VelocitySetpoint - VelocitySetpointPrevious) / dt;
	float acc_xy_mag = sqrtf(acc[0] * acc[0] + acc[1] * acc[1]);

	float acc_limit = ConfigTblPtr->ACC_HOR_MAX;

	/* Adapt slew rate if we are decelerating */
	if (Velocity * acc < 0)
	{
		acc_limit = ConfigTblPtr->DEC_HOR_MAX;
	}

	/* Limit total horizontal acceleration */
	if (acc_xy_mag > acc_limit)
	{
		VelocitySetpoint[0] = acc_limit * acc[0] / acc_xy_mag * dt + VelocitySetpointPrevious[0];
		VelocitySetpoint[1] = acc_limit * acc[1] / acc_xy_mag * dt + VelocitySetpointPrevious[1];
	}

	/* Limit vertical acceleration */
	float max_acc_z = acc[2] < 0.0f ? -ConfigTblPtr->ACC_UP_MAX : ConfigTblPtr->ACC_DOWN_MAX;

	if (fabsf(acc[2]) > fabsf(max_acc_z))
	{
		VelocitySetpoint[2] = max_acc_z * dt + VelocitySetpointPrevious[2];
	}
}



bool MPC::InAutoTakeoff(void)
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



void MPC::SetInputs(void)
{
    /***** INPUTS **********/
    VehicleStatusMsg.Timestamp = 64702172655;
    VehicleStatusMsg.SystemID = 1;
    VehicleStatusMsg.ComponentID = 1;
    VehicleStatusMsg.OnboardControlSensorsPresent = 0;
    VehicleStatusMsg.OnboardControlSensorsEnabled = 0;
    VehicleStatusMsg.OnboardControlSensorsHealth = 0;
    VehicleStatusMsg.NavState = (PX4_NavigationState_t)17;
    VehicleStatusMsg.ArmingState = (PX4_ArmingState_t)2;
    VehicleStatusMsg.HilState = (PX4_HilState_t)0;
    VehicleStatusMsg.Failsafe = 0;
    VehicleStatusMsg.SystemType = (PX4_SystemType_t)13;
    VehicleStatusMsg.IsRotaryWing = 1;
    VehicleStatusMsg.IsVtol = 0;
    VehicleStatusMsg.VtolFwPermanentStab = 0;
    VehicleStatusMsg.InTransitionMode = 0;
    VehicleStatusMsg.RcInputMode = (PX4_RcInMode_t)0;
    VehicleStatusMsg.DataLinkLost = 1;
    VehicleStatusMsg.DataLinkLostCounter = 0;
    VehicleStatusMsg.EngineFailure = 0;
    VehicleStatusMsg.EngineFailureCmd = 0;
    VehicleStatusMsg.MissionFailure = 0;
    VehicleLandDetectedMsg.Timestamp = 64698787480;
    VehicleLandDetectedMsg.AltMax = 10000.000000;
    VehicleLandDetectedMsg.Landed = 0;
    VehicleLandDetectedMsg.Freefall = 0;
    VehicleLandDetectedMsg.GroundContact = 0;
    ControlStateMsg.Timestamp = 64702297451;
    ControlStateMsg.AccX = 0.080502;
    ControlStateMsg.AccY = 0.303675;
    ControlStateMsg.AccZ = -9.636983;
    ControlStateMsg.VelX = 0.007697;
    ControlStateMsg.VelY = -0.040782;
    ControlStateMsg.VelZ = -0.125753;
    ControlStateMsg.PosX = -0.077892;
    ControlStateMsg.PosY = -0.053739;
    ControlStateMsg.PosZ = -2.412573;
    ControlStateMsg.Airspeed = 0.000000;
    ControlStateMsg.VelVariance[0] = 0.000000;
    ControlStateMsg.VelVariance[1] = 0.000000;
    ControlStateMsg.VelVariance[2] = 0.000000;
    ControlStateMsg.PosVariance[0] = 0.000000;
    ControlStateMsg.PosVariance[1] = 0.000000;
    ControlStateMsg.PosVariance[2] = 0.000000;
    ControlStateMsg.Q[0] = 0.713724;
    ControlStateMsg.Q[1] = 0.001765;
    ControlStateMsg.Q[2] = -0.014972;
    ControlStateMsg.Q[3] = 0.700265;
    ControlStateMsg.DeltaQReset[0] = 0.999362;
    ControlStateMsg.DeltaQReset[1] = -0.000642;
    ControlStateMsg.DeltaQReset[2] = 0.000736;
    ControlStateMsg.DeltaQReset[3] = -0.035702;
    ControlStateMsg.RollRate = 0.038124;
    ControlStateMsg.PitchRate = 0.020477;
    ControlStateMsg.YawRate = -0.015867;
    ControlStateMsg.HorzAccMag = 0.226660;
    ControlStateMsg.RollRateBias = -0.001481;
    ControlStateMsg.PitchRateBias = -0.005733;
    ControlStateMsg.YawRateBias = -0.004682;
    ControlStateMsg.AirspeedValid = 1;
    ControlStateMsg.QuatResetCounter = 1;
    ManualControlSetpointMsg.Timestamp = 0;
    ManualControlSetpointMsg.X = 0.000000;
    ManualControlSetpointMsg.Y = 0.000000;
    ManualControlSetpointMsg.Z = 0.000000;
    ManualControlSetpointMsg.R = 0.000000;
    ManualControlSetpointMsg.Flaps = 0.000000;
    ManualControlSetpointMsg.Aux1 = 0.000000;
    ManualControlSetpointMsg.Aux2 = 0.000000;
    ManualControlSetpointMsg.Aux3 = 0.000000;
    ManualControlSetpointMsg.Aux4 = 0.000000;
    ManualControlSetpointMsg.Aux5 = 0.000000;
    ManualControlSetpointMsg.ModeSwitch = (PX4_SwitchPos_t)0;
    ManualControlSetpointMsg.ReturnSwitch = (PX4_SwitchPos_t)0;
    ManualControlSetpointMsg.RattitudeSwitch = (PX4_SwitchPos_t)0;
    ManualControlSetpointMsg.PosctlSwitch = (PX4_SwitchPos_t)0;
    ManualControlSetpointMsg.LoiterSwitch = (PX4_SwitchPos_t)0;
    ManualControlSetpointMsg.AcroSwitch = (PX4_SwitchPos_t)0;
    ManualControlSetpointMsg.OffboardSwitch = (PX4_SwitchPos_t)0;
    ManualControlSetpointMsg.KillSwitch = (PX4_SwitchPos_t)0;
    ManualControlSetpointMsg.ArmSwitch = (PX4_SwitchPos_t)0;
    ManualControlSetpointMsg.TransitionSwitch = (PX4_SwitchPos_t)0;
    ManualControlSetpointMsg.GearSwitch = (PX4_SwitchPos_t)0;
    ManualControlSetpointMsg.ModeSlot = (PX4_ModeSlot_t)0;
    ManualControlSetpointMsg.DataSource = (PX4_ManualControlDataSource_t)0;
    ManualControlSetpointMsg.StabSwitch = (PX4_SwitchPos_t)0;
    ManualControlSetpointMsg.ManSwitch = (PX4_SwitchPos_t)0;
    PositionSetpointTripletMsg.Timestamp = 64700447907;
    PositionSetpointTripletMsg.Previous.Timestamp = 0;
    PositionSetpointTripletMsg.Previous.Lat = 0.000000;
    PositionSetpointTripletMsg.Previous.Lon = 0.000000;
    PositionSetpointTripletMsg.Previous.X = 0.000000;
    PositionSetpointTripletMsg.Previous.Y = 0.000000;
    PositionSetpointTripletMsg.Previous.Z = 0.000000;
    PositionSetpointTripletMsg.Previous.VX = 0.000000;
    PositionSetpointTripletMsg.Previous.VY = 0.000000;
    PositionSetpointTripletMsg.Previous.VZ = 0.000000;
    PositionSetpointTripletMsg.Previous.Alt = 0.000000;
    PositionSetpointTripletMsg.Previous.Yaw = 0.000000;
    PositionSetpointTripletMsg.Previous.Yawspeed = 0.000000;
    PositionSetpointTripletMsg.Previous.LoiterRadius = 0.000000;
    PositionSetpointTripletMsg.Previous.PitchMin = 0.000000;
    PositionSetpointTripletMsg.Previous.AX = 0.000000;
    PositionSetpointTripletMsg.Previous.AY = 0.000000;
    PositionSetpointTripletMsg.Previous.AZ = 0.000000;
    PositionSetpointTripletMsg.Previous.AcceptanceRadius = 0.000000;
    PositionSetpointTripletMsg.Previous.CruisingSpeed = 0.000000;
    PositionSetpointTripletMsg.Previous.CruisingThrottle = 0.000000;
    PositionSetpointTripletMsg.Previous.Valid = 0;
    PositionSetpointTripletMsg.Previous.Type = (PX4_SetpointType_t)0;
    PositionSetpointTripletMsg.Previous.PositionValid = 0;
    PositionSetpointTripletMsg.Previous.VelocityValid = 0;
    PositionSetpointTripletMsg.Previous.YawValid = 0;
    PositionSetpointTripletMsg.Previous.DisableMcYawControl = 0;
    PositionSetpointTripletMsg.Previous.YawspeedValid = 0;
    PositionSetpointTripletMsg.Previous.LoiterDirection = 0;
    PositionSetpointTripletMsg.Previous.AccelerationValid = 0;
    PositionSetpointTripletMsg.Previous.AccelerationIsForce = 0;
    PositionSetpointTripletMsg.Current.Timestamp = 0;
    PositionSetpointTripletMsg.Current.Lat = 47.397742;
    PositionSetpointTripletMsg.Current.Lon = 8.545594;
    PositionSetpointTripletMsg.Current.X = 0.000000;
    PositionSetpointTripletMsg.Current.Y = 0.000000;
    PositionSetpointTripletMsg.Current.Z = 0.000000;
    PositionSetpointTripletMsg.Current.VX = 0.000000;
    PositionSetpointTripletMsg.Current.VY = 0.000000;
    PositionSetpointTripletMsg.Current.VZ = 0.000000;
    PositionSetpointTripletMsg.Current.Alt = 490.709473;
    PositionSetpointTripletMsg.Current.Yaw = FP_NAN;
    PositionSetpointTripletMsg.Current.Yawspeed = 0.000000;
    PositionSetpointTripletMsg.Current.LoiterRadius = 50.000000;
    PositionSetpointTripletMsg.Current.PitchMin = 0.000000;
    PositionSetpointTripletMsg.Current.AX = 0.000000;
    PositionSetpointTripletMsg.Current.AY = 0.000000;
    PositionSetpointTripletMsg.Current.AZ = 0.000000;
    PositionSetpointTripletMsg.Current.AcceptanceRadius = 2.000000;
    PositionSetpointTripletMsg.Current.CruisingSpeed = -1.000000;
    PositionSetpointTripletMsg.Current.CruisingThrottle = -1.000000;
    PositionSetpointTripletMsg.Current.Valid = 1;
    PositionSetpointTripletMsg.Current.Type = (PX4_SetpointType_t)2;
    PositionSetpointTripletMsg.Current.PositionValid = 0;
    PositionSetpointTripletMsg.Current.VelocityValid = 0;
    PositionSetpointTripletMsg.Current.YawValid = 1;
    PositionSetpointTripletMsg.Current.DisableMcYawControl = 0;
    PositionSetpointTripletMsg.Current.YawspeedValid = 0;
    PositionSetpointTripletMsg.Current.LoiterDirection = 1;
    PositionSetpointTripletMsg.Current.AccelerationValid = 0;
    PositionSetpointTripletMsg.Current.AccelerationIsForce = 0;
    PositionSetpointTripletMsg.Next.Timestamp = 0;
    PositionSetpointTripletMsg.Next.Lat = 0.000000;
    PositionSetpointTripletMsg.Next.Lon = 0.000000;
    PositionSetpointTripletMsg.Next.X = 0.000000;
    PositionSetpointTripletMsg.Next.Y = 0.000000;
    PositionSetpointTripletMsg.Next.Z = 0.000000;
    PositionSetpointTripletMsg.Next.VX = 0.000000;
    PositionSetpointTripletMsg.Next.VY = 0.000000;
    PositionSetpointTripletMsg.Next.VZ = 0.000000;
    PositionSetpointTripletMsg.Next.Alt = 0.000000;
    PositionSetpointTripletMsg.Next.Yaw = 0.000000;
    PositionSetpointTripletMsg.Next.Yawspeed = 0.000000;
    PositionSetpointTripletMsg.Next.LoiterRadius = 0.000000;
    PositionSetpointTripletMsg.Next.PitchMin = 0.000000;
    PositionSetpointTripletMsg.Next.AX = 0.000000;
    PositionSetpointTripletMsg.Next.AY = 0.000000;
    PositionSetpointTripletMsg.Next.AZ = 0.000000;
    PositionSetpointTripletMsg.Next.AcceptanceRadius = 0.000000;
    PositionSetpointTripletMsg.Next.CruisingSpeed = 0.000000;
    PositionSetpointTripletMsg.Next.CruisingThrottle = 0.000000;
    PositionSetpointTripletMsg.Next.Valid = 0;
    PositionSetpointTripletMsg.Next.Type = (PX4_SetpointType_t)0;
    PositionSetpointTripletMsg.Next.PositionValid = 0;
    PositionSetpointTripletMsg.Next.VelocityValid = 0;
    PositionSetpointTripletMsg.Next.YawValid = 0;
    PositionSetpointTripletMsg.Next.DisableMcYawControl = 0;
    PositionSetpointTripletMsg.Next.YawspeedValid = 0;
    PositionSetpointTripletMsg.Next.LoiterDirection = 0;
    PositionSetpointTripletMsg.Next.AccelerationValid = 0;
    PositionSetpointTripletMsg.Next.AccelerationIsForce = 0;
    VehicleControlModeMsg.Timestamp = 64702172655;
    VehicleControlModeMsg.Armed = 1;
    VehicleControlModeMsg.ExternalManualOverrideOk = 0;
    VehicleControlModeMsg.SystemHilEnabled = 0;
    VehicleControlModeMsg.ControlManualEnabled = 0;
    VehicleControlModeMsg.ControlAutoEnabled = 1;
    VehicleControlModeMsg.ControlOffboardEnabled = 0;
    VehicleControlModeMsg.ControlRatesEnabled = 1;
    VehicleControlModeMsg.ControlAttitudeEnabled = 1;
    VehicleControlModeMsg.ControlRattitudeEnabled = 0;
    VehicleControlModeMsg.ControlForceEnabled = 0;
    VehicleControlModeMsg.ControlAccelerationEnabled = 0;
    VehicleControlModeMsg.ControlVelocityEnabled = 1;
    VehicleControlModeMsg.ControlPositionEnabled = 1;
    VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    VehicleControlModeMsg.ControlTerminationEnabled = 0;
    VehicleControlModeMsg.ControlFixedHdgEnabled = 0;
    VehicleLocalPositionMsg.Timestamp = 64702297451;
    VehicleLocalPositionMsg.RefTimestamp = 64697227396;
    VehicleLocalPositionMsg.RefLat = 47.397742;
    VehicleLocalPositionMsg.RefLon = 8.545594;
    VehicleLocalPositionMsg.SurfaceBottomTimestamp = 64702297451;
    VehicleLocalPositionMsg.X = -0.077892;
    VehicleLocalPositionMsg.Y = -0.053739;
    VehicleLocalPositionMsg.Z = -2.412573;
    VehicleLocalPositionMsg.Delta_XY[0] = -0.001637;
    VehicleLocalPositionMsg.Delta_XY[1] = -0.011751;
    VehicleLocalPositionMsg.Delta_Z = 0.000000;
    VehicleLocalPositionMsg.VX = 0.043289;
    VehicleLocalPositionMsg.VY = 0.009862;
    VehicleLocalPositionMsg.VZ = -0.124760;
    VehicleLocalPositionMsg.Delta_VXY[0] = -0.001951;
    VehicleLocalPositionMsg.Delta_VXY[1] = -0.017590;
    VehicleLocalPositionMsg.Delta_VZ = 0.010634;
    VehicleLocalPositionMsg.AX = 0.000000;
    VehicleLocalPositionMsg.AY = 0.000000;
    VehicleLocalPositionMsg.AZ = 0.000000;
    VehicleLocalPositionMsg.Yaw = 1.551980;
    VehicleLocalPositionMsg.RefAlt = 488.228607;
    VehicleLocalPositionMsg.DistBottom = 5.047842;
    VehicleLocalPositionMsg.DistBottomRate = 0.124760;
    VehicleLocalPositionMsg.EpH = 0.409998;
    VehicleLocalPositionMsg.EpV = 0.270683;
    VehicleLocalPositionMsg.EvV = 0.106474;
    VehicleLocalPositionMsg.EstimatorType = 0;
    VehicleLocalPositionMsg.XY_Valid = 1;
    VehicleLocalPositionMsg.Z_Valid = 1;
    VehicleLocalPositionMsg.V_XY_Valid = 1;
    VehicleLocalPositionMsg.V_Z_Valid = 1;
    VehicleLocalPositionMsg.XY_ResetCounter = 1;
    VehicleLocalPositionMsg.Z_ResetCounter = 0;
    VehicleLocalPositionMsg.VXY_ResetCounter = 1;
    VehicleLocalPositionMsg.VZ_ResetCounter = 1;
    VehicleLocalPositionMsg.XY_Global = 1;
    VehicleLocalPositionMsg.Z_Global = 1;
    VehicleLocalPositionMsg.DistBottomValid = 1;
    VehicleLocalPositionSetpointMsg.Timestamp = 64702293407;
    VehicleLocalPositionSetpointMsg.X = -0.001493;
    VehicleLocalPositionSetpointMsg.Y = 0.000167;
    VehicleLocalPositionSetpointMsg.Z = -2.480865;
    VehicleLocalPositionSetpointMsg.Yaw = 1.553056;
    VehicleLocalPositionSetpointMsg.VX = 0.033659;
    VehicleLocalPositionSetpointMsg.VY = 0.023705;
    VehicleLocalPositionSetpointMsg.VZ = -0.068879;
    VehicleLocalPositionSetpointMsg.AccX = 0.086912;
    VehicleLocalPositionSetpointMsg.AccY = 0.082852;
    VehicleLocalPositionSetpointMsg.AccZ = -4.143278;
    HomePositionMsg.Timestamp = 64698759848;
    HomePositionMsg.Lat = 47.397742;
    HomePositionMsg.Lon = 8.545594;
    HomePositionMsg.Alt = 488.209412;
    HomePositionMsg.X = -0.001485;
    HomePositionMsg.Y = 0.000027;
    HomePositionMsg.Z = 0.019192;
    HomePositionMsg.Yaw = 1.552912;
    HomePositionMsg.DirectionX = 0.000000;
    HomePositionMsg.DirectionY = 0.000000;
    HomePositionMsg.DirectionZ = 0.000000;
}



void MPC::SetMembers(void)
{
    /***** MEMBERS **********/
    GearStateInitialized = 0;
    RefAlt = 488.228607;
    RefPos.timestamp = 64697227436;
    RefPos.lat_rad = 0.827247;
    RefPos.lon_rad = 0.149149;
    RefPos.sin_lat = 0.736070;
    RefPos.cos_lat = 0.676905;
    RefPos.init_done = 1;
    RefTimestamp = 64697227396;
    ResetPositionSetpoint = 0;
    ResetAltitudeSetpoint = 0;
    DoResetAltPos = 0;
    ModeAuto = 1;
    PositionHoldEngaged = 0;
    AltitudeHoldEngaged = 0;
    RunPosControl = 1;
    RunAltControl = 1;
    ResetIntZ = 0;
    ResetIntXY = 0;
    ResetIntZManual = 0;
    ResetYawSetpoint = 1;
    HoldOffboardXY = 0;
    HoldOffboardZ = 0;
    LimitVelXY = 1;
    ThrustInt[0] = 0.009756;
    ThrustInt[1] = 0.006899;
    ThrustInt[2] = 0.043560;
    Position[0] = -0.078096;
    Position[1] = -0.053782;
    Position[2] = -2.411986;
    PositionSetpoint[0] = -0.001493;
    PositionSetpoint[1] = 0.000167;
    PositionSetpoint[2] = -2.480865;
    Velocity[0] = 0.043850;
    Velocity[1] = 0.008404;
    Velocity[2] = -0.125496;
    VelocitySetpoint[0] = 0.033659;
    VelocitySetpoint[1] = 0.023705;
    VelocitySetpoint[2] = -0.068879;
    VelocityPrevious[0] = 0.043850;
    VelocityPrevious[1] = 0.008404;
    VelocityPrevious[2] = -0.125496;
    VelocityFF[0] = 0.000000;
    VelocityFF[1] = 0.000000;
    VelocityFF[2] = 0.000000;
    VelocitySetpointPrevious[0] = 0.033659;
    VelocitySetpointPrevious[1] = 0.023705;
    VelocitySetpointPrevious[2] = -0.068879;
    VelocityErrD[0] = 0.062922;
    VelocityErrD[1] = -0.073553;
    VelocityErrD[2] = -0.219707;
    CurrentPositionSetpoint[0] = -0.001493;
    CurrentPositionSetpoint[1] = 0.000167;
    CurrentPositionSetpoint[2] = -2.480865;
    Rotation[0][0] = 0.018810;
    Rotation[0][1] = -0.999645;
    Rotation[0][2] = -0.018900;
    Rotation[1][0] = 0.999539;
    Rotation[1][1] = 0.019253;
    Rotation[1][2] = -0.023488;
    Rotation[2][0] = 0.023844;
    Rotation[2][1] = -0.018449;
    Rotation[2][2] = 0.999546;
    Yaw = 1.551980;
    YawTakeoff = 1.552942;
    InLanding = 0;
    LndReachedGround = 0;
    VelZLp = -0.149661;
    AccZLp = 0.201767;
    VelMaxXY = 8.000000;
    InTakeoff = 0;
    TakeoffVelLimit = 1.524719;
    Z_ResetCounter = 0;
    XY_ResetCounter = 1;
    VZ_ResetCounter = 1;
    VXY_ResetCounter = 1;
    HeadingResetCounter = 2;
    RSetpoint[0][0] = 0.017736;
    RSetpoint[0][1] = -0.999623;
    RSetpoint[0][2] = -0.020968;
    RSetpoint[1][0] = 0.999635;
    RSetpoint[1][1] = 0.018156;
    RSetpoint[1][2] = -0.019988;
    RSetpoint[2][0] = 0.020361;
    RSetpoint[2][1] = -0.020606;
    RSetpoint[2][2] = 0.999580;
}



void MPC::DisplayInputs(void)
{
	OS_printf("***** INPUTS **********");
	OS_printf("VehicleStatusMsg.Timestamp = %llu\n", VehicleStatusMsg.Timestamp);
	OS_printf("VehicleStatusMsg.SystemID = %u\n", VehicleStatusMsg.SystemID);
	OS_printf("VehicleStatusMsg.ComponentID = %u\n", VehicleStatusMsg.ComponentID);
	OS_printf("VehicleStatusMsg.OnboardControlSensorsPresent = %u\n", VehicleStatusMsg.OnboardControlSensorsPresent);
	OS_printf("VehicleStatusMsg.OnboardControlSensorsEnabled = %u\n", VehicleStatusMsg.OnboardControlSensorsEnabled);
	OS_printf("VehicleStatusMsg.OnboardControlSensorsHealth = %u\n", VehicleStatusMsg.OnboardControlSensorsHealth);
	OS_printf("VehicleStatusMsg.NavState = %u\n", VehicleStatusMsg.NavState);
	OS_printf("VehicleStatusMsg.ArmingState = %u\n", VehicleStatusMsg.ArmingState);
	OS_printf("VehicleStatusMsg.HilState = %u\n", VehicleStatusMsg.HilState);
	OS_printf("VehicleStatusMsg.Failsafe = %u\n", VehicleStatusMsg.Failsafe);
	OS_printf("VehicleStatusMsg.SystemType = %u\n", VehicleStatusMsg.SystemType);
	OS_printf("VehicleStatusMsg.IsRotaryWing = %u\n", VehicleStatusMsg.IsRotaryWing);
	OS_printf("VehicleStatusMsg.IsVtol = %u\n", VehicleStatusMsg.IsVtol);
	OS_printf("VehicleStatusMsg.VtolFwPermanentStab = %u\n", VehicleStatusMsg.VtolFwPermanentStab);
	OS_printf("VehicleStatusMsg.InTransitionMode = %u\n", VehicleStatusMsg.InTransitionMode);
	OS_printf("VehicleStatusMsg.RcSignalLost = %u\n", VehicleStatusMsg.RcSignalLost);
	OS_printf("VehicleStatusMsg.RcInputMode = %u\n", VehicleStatusMsg.RcInputMode);
	OS_printf("VehicleStatusMsg.DataLinkLost = %u\n", VehicleStatusMsg.DataLinkLost);
	OS_printf("VehicleStatusMsg.DataLinkLostCounter = %u\n", VehicleStatusMsg.DataLinkLostCounter);
	OS_printf("VehicleStatusMsg.EngineFailure = %u\n", VehicleStatusMsg.EngineFailure);
	OS_printf("VehicleStatusMsg.EngineFailureCmd = %u\n", VehicleStatusMsg.EngineFailureCmd);
	OS_printf("VehicleStatusMsg.MissionFailure = %u\n", VehicleStatusMsg.MissionFailure);

	OS_printf("VehicleLandDetectedMsg.Timestamp = %llu\n", VehicleLandDetectedMsg.Timestamp);
	OS_printf("VehicleLandDetectedMsg.AltMax = %f\n", (double)VehicleLandDetectedMsg.AltMax);
	OS_printf("VehicleLandDetectedMsg.Landed = %u\n", VehicleLandDetectedMsg.Landed);
	OS_printf("VehicleLandDetectedMsg.Freefall = %u\n", VehicleLandDetectedMsg.Freefall);
	OS_printf("VehicleLandDetectedMsg.GroundContact = %u\n", VehicleLandDetectedMsg.GroundContact);

	OS_printf("ControlStateMsg.Timestamp = %llu\n", ControlStateMsg.Timestamp);
	OS_printf("ControlStateMsg.AccX = %f\n", (double)ControlStateMsg.AccX);
	OS_printf("ControlStateMsg.AccY = %f\n", (double)ControlStateMsg.AccY);
	OS_printf("ControlStateMsg.AccZ = %f\n", (double)ControlStateMsg.AccZ);
	OS_printf("ControlStateMsg.VelX = %f\n", (double)ControlStateMsg.VelX);
	OS_printf("ControlStateMsg.VelY = %f\n", (double)ControlStateMsg.VelY);
	OS_printf("ControlStateMsg.VelZ = %f\n", (double)ControlStateMsg.VelZ);
	OS_printf("ControlStateMsg.PosX = %f\n", (double)ControlStateMsg.PosX);
	OS_printf("ControlStateMsg.PosY = %f\n", (double)ControlStateMsg.PosY);
	OS_printf("ControlStateMsg.PosZ = %f\n", (double)ControlStateMsg.PosZ);
	OS_printf("ControlStateMsg.Airspeed = %f\n", (double)ControlStateMsg.Airspeed);
	OS_printf("ControlStateMsg.VelVariance[0] = %f\n", (double)ControlStateMsg.VelVariance[0]);
	OS_printf("ControlStateMsg.VelVariance[1] = %f\n", (double)ControlStateMsg.VelVariance[1]);
	OS_printf("ControlStateMsg.VelVariance[2] = %f\n", (double)ControlStateMsg.VelVariance[2]);
	OS_printf("ControlStateMsg.PosVariance[0] = %f\n", (double)ControlStateMsg.PosVariance[0]);
	OS_printf("ControlStateMsg.PosVariance[1] = %f\n", (double)ControlStateMsg.PosVariance[1]);
	OS_printf("ControlStateMsg.PosVariance[2] = %f\n", (double)ControlStateMsg.PosVariance[2]);
	OS_printf("ControlStateMsg.Q[0] = %f\n", (double)ControlStateMsg.Q[0]);
	OS_printf("ControlStateMsg.Q[1] = %f\n", (double)ControlStateMsg.Q[1]);
	OS_printf("ControlStateMsg.Q[2] = %f\n", (double)ControlStateMsg.Q[2]);
	OS_printf("ControlStateMsg.Q[3] = %f\n", (double)ControlStateMsg.Q[3]);
	OS_printf("ControlStateMsg.DeltaQReset[0] = %f\n", (double)ControlStateMsg.DeltaQReset[0]);
	OS_printf("ControlStateMsg.DeltaQReset[1] = %f\n", (double)ControlStateMsg.DeltaQReset[1]);
	OS_printf("ControlStateMsg.DeltaQReset[2] = %f\n", (double)ControlStateMsg.DeltaQReset[2]);
	OS_printf("ControlStateMsg.DeltaQReset[3] = %f\n", (double)ControlStateMsg.DeltaQReset[3]);
	OS_printf("ControlStateMsg.RollRate = %f\n", (double)ControlStateMsg.RollRate);
	OS_printf("ControlStateMsg.PitchRate = %f\n", (double)ControlStateMsg.PitchRate);
	OS_printf("ControlStateMsg.YawRate = %f\n", (double)ControlStateMsg.YawRate);
	OS_printf("ControlStateMsg.HorzAccMag = %f\n", (double)ControlStateMsg.HorzAccMag);
	OS_printf("ControlStateMsg.RollRateBias = %f\n", (double)ControlStateMsg.RollRateBias);
	OS_printf("ControlStateMsg.PitchRateBias = %f\n", (double)ControlStateMsg.PitchRateBias);
	OS_printf("ControlStateMsg.YawRateBias = %f\n", (double)ControlStateMsg.YawRateBias);
	OS_printf("ControlStateMsg.AirspeedValid = %u\n", ControlStateMsg.AirspeedValid);
	OS_printf("ControlStateMsg.QuatResetCounter = %u\n", ControlStateMsg.QuatResetCounter);

    OS_printf("ManualControlSetpointMsg.Timestamp = %llu\n", ManualControlSetpointMsg.Timestamp);
    OS_printf("ManualControlSetpointMsg.X = %f\n", (double)ManualControlSetpointMsg.X);
    OS_printf("ManualControlSetpointMsg.Y = %f\n", (double)ManualControlSetpointMsg.Y);
    OS_printf("ManualControlSetpointMsg.Z = %f\n", (double)ManualControlSetpointMsg.Z);
    OS_printf("ManualControlSetpointMsg.R = %f\n", (double)ManualControlSetpointMsg.R);
    OS_printf("ManualControlSetpointMsg.Flaps = %f\n", (double)ManualControlSetpointMsg.Flaps);
    OS_printf("ManualControlSetpointMsg.Aux1 = %f\n", (double)ManualControlSetpointMsg.Aux1);
    OS_printf("ManualControlSetpointMsg.Aux2 = %f\n", (double)ManualControlSetpointMsg.Aux2);
    OS_printf("ManualControlSetpointMsg.Aux3 = %f\n", (double)ManualControlSetpointMsg.Aux3);
    OS_printf("ManualControlSetpointMsg.Aux4 = %f\n", (double)ManualControlSetpointMsg.Aux4);
    OS_printf("ManualControlSetpointMsg.Aux5 = %f\n", (double)ManualControlSetpointMsg.Aux5);
    OS_printf("ManualControlSetpointMsg.ModeSwitch = %u\n", ManualControlSetpointMsg.ModeSwitch);
    OS_printf("ManualControlSetpointMsg.ReturnSwitch = %u\n", ManualControlSetpointMsg.ReturnSwitch);
    OS_printf("ManualControlSetpointMsg.RattitudeSwitch = %u\n", ManualControlSetpointMsg.RattitudeSwitch);
    OS_printf("ManualControlSetpointMsg.PosctlSwitch = %u\n", ManualControlSetpointMsg.PosctlSwitch);
    OS_printf("ManualControlSetpointMsg.LoiterSwitch = %u\n", ManualControlSetpointMsg.LoiterSwitch);
    OS_printf("ManualControlSetpointMsg.AcroSwitch = %u\n", ManualControlSetpointMsg.AcroSwitch);
    OS_printf("ManualControlSetpointMsg.OffboardSwitch = %u\n", ManualControlSetpointMsg.OffboardSwitch);
    OS_printf("ManualControlSetpointMsg.KillSwitch = %u\n", ManualControlSetpointMsg.KillSwitch);
    OS_printf("ManualControlSetpointMsg.TransitionSwitch = %u\n", ManualControlSetpointMsg.TransitionSwitch);
    OS_printf("ManualControlSetpointMsg.GearSwitch = %u\n", ManualControlSetpointMsg.GearSwitch);
    OS_printf("ManualControlSetpointMsg.ArmSwitch = %u\n", ManualControlSetpointMsg.ArmSwitch);
    OS_printf("ManualControlSetpointMsg.StabSwitch = %u\n", ManualControlSetpointMsg.StabSwitch);
    OS_printf("ManualControlSetpointMsg.ManSwitch = %u\n", ManualControlSetpointMsg.ManSwitch);
    OS_printf("ManualControlSetpointMsg.ModeSlot = %i\n", ManualControlSetpointMsg.ModeSlot);
    OS_printf("ManualControlSetpointMsg.DataSource = %u\n", ManualControlSetpointMsg.DataSource);


    OS_printf("PositionSetpointTripletMsg.Timestamp = %llu\n", PositionSetpointTripletMsg.Timestamp);
    OS_printf("PositionSetpointTripletMsg.Previous.Timestamp = %llu\n", PositionSetpointTripletMsg.Previous.Timestamp);
    OS_printf("PositionSetpointTripletMsg.Previous.Lat = %f\n", PositionSetpointTripletMsg.Previous.Lat);
    OS_printf("PositionSetpointTripletMsg.Previous.Lon = %f\n", PositionSetpointTripletMsg.Previous.Lon);
    OS_printf("PositionSetpointTripletMsg.Previous.X = %f\n", (double)PositionSetpointTripletMsg.Previous.X);
    OS_printf("PositionSetpointTripletMsg.Previous.Y = %f\n", (double)PositionSetpointTripletMsg.Previous.Y);
    OS_printf("PositionSetpointTripletMsg.Previous.Z = %f\n", (double)PositionSetpointTripletMsg.Previous.Z);
    OS_printf("PositionSetpointTripletMsg.Previous.VX = %f\n", (double)PositionSetpointTripletMsg.Previous.VX);
    OS_printf("PositionSetpointTripletMsg.Previous.VY = %f\n", (double)PositionSetpointTripletMsg.Previous.VY);
    OS_printf("PositionSetpointTripletMsg.Previous.VZ = %f\n", (double)PositionSetpointTripletMsg.Previous.VZ);
    OS_printf("PositionSetpointTripletMsg.Previous.Alt = %f\n", (double)PositionSetpointTripletMsg.Previous.Alt);
    OS_printf("PositionSetpointTripletMsg.Previous.Yaw = %f\n", (double)PositionSetpointTripletMsg.Previous.Yaw);
    OS_printf("PositionSetpointTripletMsg.Previous.Yawspeed = %f\n", (double)PositionSetpointTripletMsg.Previous.Yawspeed);
    OS_printf("PositionSetpointTripletMsg.Previous.LoiterRadius = %f\n", (double)PositionSetpointTripletMsg.Previous.LoiterRadius);
    OS_printf("PositionSetpointTripletMsg.Previous.PitchMin = %f\n", (double)PositionSetpointTripletMsg.Previous.PitchMin);
    OS_printf("PositionSetpointTripletMsg.Previous.AX = %f\n", (double)PositionSetpointTripletMsg.Previous.AX);
    OS_printf("PositionSetpointTripletMsg.Previous.AY = %f\n", (double)PositionSetpointTripletMsg.Previous.AY);
    OS_printf("PositionSetpointTripletMsg.Previous.AZ = %f\n", (double)PositionSetpointTripletMsg.Previous.AZ);
    OS_printf("PositionSetpointTripletMsg.Previous.AcceptanceRadius = %f\n", (double)PositionSetpointTripletMsg.Previous.AcceptanceRadius);
    OS_printf("PositionSetpointTripletMsg.Previous.CruisingSpeed = %f\n", (double)PositionSetpointTripletMsg.Previous.CruisingSpeed);
    OS_printf("PositionSetpointTripletMsg.Previous.CruisingThrottle = %f\n", (double)PositionSetpointTripletMsg.Previous.CruisingThrottle);
    OS_printf("PositionSetpointTripletMsg.Previous.Valid = %u\n", PositionSetpointTripletMsg.Previous.Valid);
    OS_printf("PositionSetpointTripletMsg.Previous.Type = %u\n", PositionSetpointTripletMsg.Previous.Type);
    OS_printf("PositionSetpointTripletMsg.Previous.PositionValid = %u\n", PositionSetpointTripletMsg.Previous.PositionValid);
    OS_printf("PositionSetpointTripletMsg.Previous.VelocityValid = %u\n", PositionSetpointTripletMsg.Previous.VelocityValid);
    OS_printf("PositionSetpointTripletMsg.Previous.YawValid = %u\n", PositionSetpointTripletMsg.Previous.YawValid);
    OS_printf("PositionSetpointTripletMsg.Previous.DisableMcYawControl = %u\n", PositionSetpointTripletMsg.Previous.DisableMcYawControl);
    OS_printf("PositionSetpointTripletMsg.Previous.YawspeedValid = %u\n", PositionSetpointTripletMsg.Previous.YawspeedValid);
    OS_printf("PositionSetpointTripletMsg.Previous.LoiterDirection = %i\n", PositionSetpointTripletMsg.Previous.LoiterDirection);
    OS_printf("PositionSetpointTripletMsg.Previous.AccelerationValid = %u\n", PositionSetpointTripletMsg.Previous.AccelerationValid);
    OS_printf("PositionSetpointTripletMsg.Previous.AccelerationIsForce = %u\n", PositionSetpointTripletMsg.Previous.AccelerationIsForce);

    OS_printf("PositionSetpointTripletMsg.Current.Timestamp = %llu\n", PositionSetpointTripletMsg.Current.Timestamp);
    OS_printf("PositionSetpointTripletMsg.Current.Lat = %f\n", PositionSetpointTripletMsg.Current.Lat);
    OS_printf("PositionSetpointTripletMsg.Current.Lon = %f\n", PositionSetpointTripletMsg.Current.Lon);
    OS_printf("PositionSetpointTripletMsg.Current.X = %f\n", (double)PositionSetpointTripletMsg.Current.X);
    OS_printf("PositionSetpointTripletMsg.Current.Y = %f\n", (double)PositionSetpointTripletMsg.Current.Y);
    OS_printf("PositionSetpointTripletMsg.Current.Z = %f\n", (double)PositionSetpointTripletMsg.Current.Z);
    OS_printf("PositionSetpointTripletMsg.Current.VX = %f\n", (double)PositionSetpointTripletMsg.Current.VX);
    OS_printf("PositionSetpointTripletMsg.Current.VY = %f\n", (double)PositionSetpointTripletMsg.Current.VY);
    OS_printf("PositionSetpointTripletMsg.Current.VZ = %f\n", (double)PositionSetpointTripletMsg.Current.VZ);
    OS_printf("PositionSetpointTripletMsg.Current.Alt = %f\n", (double)PositionSetpointTripletMsg.Current.Alt);
    OS_printf("PositionSetpointTripletMsg.Current.Yaw = %f\n", (double)PositionSetpointTripletMsg.Current.Yaw);
    OS_printf("PositionSetpointTripletMsg.Current.Yawspeed = %f\n", (double)PositionSetpointTripletMsg.Current.Yawspeed);
    OS_printf("PositionSetpointTripletMsg.Current.LoiterRadius = %f\n", (double)PositionSetpointTripletMsg.Current.LoiterRadius);
    OS_printf("PositionSetpointTripletMsg.Current.PitchMin = %f\n", (double)PositionSetpointTripletMsg.Current.PitchMin);
    OS_printf("PositionSetpointTripletMsg.Current.AX = %f\n", (double)PositionSetpointTripletMsg.Current.AX);
    OS_printf("PositionSetpointTripletMsg.Current.AY = %f\n", (double)PositionSetpointTripletMsg.Current.AY);
    OS_printf("PositionSetpointTripletMsg.Current.AZ = %f\n", (double)PositionSetpointTripletMsg.Current.AZ);
    OS_printf("PositionSetpointTripletMsg.Current.AcceptanceRadius = %f\n", (double)PositionSetpointTripletMsg.Current.AcceptanceRadius);
    OS_printf("PositionSetpointTripletMsg.Current.CruisingSpeed = %f\n", (double)PositionSetpointTripletMsg.Current.CruisingSpeed);
    OS_printf("PositionSetpointTripletMsg.Current.CruisingThrottle = %f\n", (double)PositionSetpointTripletMsg.Current.CruisingThrottle);
    OS_printf("PositionSetpointTripletMsg.Current.Valid = %u\n", PositionSetpointTripletMsg.Current.Valid);
    OS_printf("PositionSetpointTripletMsg.Current.Type = %u\n", PositionSetpointTripletMsg.Current.Type);
    OS_printf("PositionSetpointTripletMsg.Current.PositionValid = %u\n", PositionSetpointTripletMsg.Current.PositionValid);
    OS_printf("PositionSetpointTripletMsg.Current.VelocityValid = %u\n", PositionSetpointTripletMsg.Current.VelocityValid);
    OS_printf("PositionSetpointTripletMsg.Current.YawValid = %u\n", PositionSetpointTripletMsg.Current.YawValid);
    OS_printf("PositionSetpointTripletMsg.Current.DisableMcYawControl = %u\n", PositionSetpointTripletMsg.Current.DisableMcYawControl);
    OS_printf("PositionSetpointTripletMsg.Current.YawspeedValid = %u\n", PositionSetpointTripletMsg.Current.YawspeedValid);
    OS_printf("PositionSetpointTripletMsg.Current.LoiterDirection = %i\n", PositionSetpointTripletMsg.Current.LoiterDirection);
    OS_printf("PositionSetpointTripletMsg.Current.AccelerationValid = %u\n", PositionSetpointTripletMsg.Current.AccelerationValid);
    OS_printf("PositionSetpointTripletMsg.Current.AccelerationIsForce = %u\n", PositionSetpointTripletMsg.Current.AccelerationIsForce);

    OS_printf("PositionSetpointTripletMsg.Next.Timestamp = %llu\n", PositionSetpointTripletMsg.Next.Timestamp);
    OS_printf("PositionSetpointTripletMsg.Next.Lat = %f\n", PositionSetpointTripletMsg.Next.Lat);
    OS_printf("PositionSetpointTripletMsg.Next.Lon = %f\n", PositionSetpointTripletMsg.Next.Lon);
    OS_printf("PositionSetpointTripletMsg.Next.X = %f\n", (double)PositionSetpointTripletMsg.Next.X);
    OS_printf("PositionSetpointTripletMsg.Next.Y = %f\n", (double)PositionSetpointTripletMsg.Next.Y);
    OS_printf("PositionSetpointTripletMsg.Next.Z = %f\n", (double)PositionSetpointTripletMsg.Next.Z);
    OS_printf("PositionSetpointTripletMsg.Next.VX = %f\n", (double)PositionSetpointTripletMsg.Next.VX);
    OS_printf("PositionSetpointTripletMsg.Next.VY = %f\n", (double)PositionSetpointTripletMsg.Next.VY);
    OS_printf("PositionSetpointTripletMsg.Next.VZ = %f\n", (double)PositionSetpointTripletMsg.Next.VZ);
    OS_printf("PositionSetpointTripletMsg.Next.Alt = %f\n", (double)PositionSetpointTripletMsg.Next.Alt);
    OS_printf("PositionSetpointTripletMsg.Next.Yaw = %f\n", (double)PositionSetpointTripletMsg.Next.Yaw);
    OS_printf("PositionSetpointTripletMsg.Next.Yawspeed = %f\n", (double)PositionSetpointTripletMsg.Next.Yawspeed);
    OS_printf("PositionSetpointTripletMsg.Next.LoiterRadius = %f\n", (double)PositionSetpointTripletMsg.Next.LoiterRadius);
    OS_printf("PositionSetpointTripletMsg.Next.PitchMin = %f\n", (double)PositionSetpointTripletMsg.Next.PitchMin);
    OS_printf("PositionSetpointTripletMsg.Next.AX = %f\n", (double)PositionSetpointTripletMsg.Next.AX);
    OS_printf("PositionSetpointTripletMsg.Next.AY = %f\n", (double)PositionSetpointTripletMsg.Next.AY);
    OS_printf("PositionSetpointTripletMsg.Next.AZ = %f\n", (double)PositionSetpointTripletMsg.Next.AZ);
    OS_printf("PositionSetpointTripletMsg.Next.AcceptanceRadius = %f\n", (double)PositionSetpointTripletMsg.Next.AcceptanceRadius);
    OS_printf("PositionSetpointTripletMsg.Next.CruisingSpeed = %f\n", (double)PositionSetpointTripletMsg.Next.CruisingSpeed);
    OS_printf("PositionSetpointTripletMsg.Next.CruisingThrottle = %f\n", (double)PositionSetpointTripletMsg.Next.CruisingThrottle);
    OS_printf("PositionSetpointTripletMsg.Next.Valid = %u\n", PositionSetpointTripletMsg.Next.Valid);
    OS_printf("PositionSetpointTripletMsg.Next.Type = %u\n", PositionSetpointTripletMsg.Next.Type);
    OS_printf("PositionSetpointTripletMsg.Next.PositionValid = %u\n", PositionSetpointTripletMsg.Next.PositionValid);
    OS_printf("PositionSetpointTripletMsg.Next.VelocityValid = %u\n", PositionSetpointTripletMsg.Next.VelocityValid);
    OS_printf("PositionSetpointTripletMsg.Next.YawValid = %u\n", PositionSetpointTripletMsg.Next.YawValid);
    OS_printf("PositionSetpointTripletMsg.Next.DisableMcYawControl = %u\n", PositionSetpointTripletMsg.Next.DisableMcYawControl);
    OS_printf("PositionSetpointTripletMsg.Next.YawspeedValid = %u\n", PositionSetpointTripletMsg.Next.YawspeedValid);
    OS_printf("PositionSetpointTripletMsg.Next.LoiterDirection = %i\n", PositionSetpointTripletMsg.Next.LoiterDirection);
    OS_printf("PositionSetpointTripletMsg.Next.AccelerationValid = %u\n", PositionSetpointTripletMsg.Next.AccelerationValid);
    OS_printf("PositionSetpointTripletMsg.Next.AccelerationIsForce = %u\n", PositionSetpointTripletMsg.Next.AccelerationIsForce);

	OS_printf("VehicleControlModeMsg.Timestamp = %llu\n", VehicleControlModeMsg.Timestamp);
	OS_printf("VehicleControlModeMsg.Armed = %u\n", VehicleControlModeMsg.Armed);
	OS_printf("VehicleControlModeMsg.ExternalManualOverrideOk = %u\n", VehicleControlModeMsg.ExternalManualOverrideOk);
	OS_printf("VehicleControlModeMsg.SystemHilEnabled = %u\n", VehicleControlModeMsg.SystemHilEnabled);
	OS_printf("VehicleControlModeMsg.ControlManualEnabled = %u\n", VehicleControlModeMsg.ControlManualEnabled);
	OS_printf("VehicleControlModeMsg.ControlAutoEnabled = %u\n", VehicleControlModeMsg.ControlAutoEnabled);
	OS_printf("VehicleControlModeMsg.ControlOffboardEnabled = %u\n", VehicleControlModeMsg.ControlOffboardEnabled);
	OS_printf("VehicleControlModeMsg.ControlRatesEnabled = %u\n", VehicleControlModeMsg.ControlRatesEnabled);
	OS_printf("VehicleControlModeMsg.ControlAttitudeEnabled = %u\n", VehicleControlModeMsg.ControlAttitudeEnabled);
	OS_printf("VehicleControlModeMsg.ControlRattitudeEnabled = %u\n", VehicleControlModeMsg.ControlRattitudeEnabled);
	OS_printf("VehicleControlModeMsg.ControlForceEnabled = %u\n", VehicleControlModeMsg.ControlForceEnabled);
	OS_printf("VehicleControlModeMsg.ControlAccelerationEnabled = %u\n", VehicleControlModeMsg.ControlAccelerationEnabled);
	OS_printf("VehicleControlModeMsg.ControlVelocityEnabled = %u\n", VehicleControlModeMsg.ControlVelocityEnabled);
	OS_printf("VehicleControlModeMsg.ControlPositionEnabled = %u\n", VehicleControlModeMsg.ControlPositionEnabled);
	OS_printf("VehicleControlModeMsg.ControlAltitudeEnabled = %u\n", VehicleControlModeMsg.ControlAltitudeEnabled);
	OS_printf("VehicleControlModeMsg.ControlClimbRateEnabled = %u\n", VehicleControlModeMsg.ControlClimbRateEnabled);
	OS_printf("VehicleControlModeMsg.ControlTerminationEnabled = %u\n", VehicleControlModeMsg.ControlTerminationEnabled);
	OS_printf("VehicleControlModeMsg.ControlFixedHdgEnabled = %u\n", VehicleControlModeMsg.ControlFixedHdgEnabled);

	OS_printf("VehicleLocalPositionMsg.Timestamp = %llu\n", VehicleLocalPositionMsg.Timestamp);
	OS_printf("VehicleLocalPositionMsg.RefTimestamp = %llu\n", VehicleLocalPositionMsg.RefTimestamp);
	OS_printf("VehicleLocalPositionMsg.RefLat = %f\n", (double)VehicleLocalPositionMsg.RefLat);
	OS_printf("VehicleLocalPositionMsg.RefLon = %f\n", (double)VehicleLocalPositionMsg.RefLon);
	OS_printf("VehicleLocalPositionMsg.SurfaceBottomTimestamp = %llu\n", VehicleLocalPositionMsg.SurfaceBottomTimestamp);
	OS_printf("VehicleLocalPositionMsg.X = %f\n", (double)VehicleLocalPositionMsg.X);
	OS_printf("VehicleLocalPositionMsg.Y = %f\n", (double)VehicleLocalPositionMsg.Y);
	OS_printf("VehicleLocalPositionMsg.Z = %f\n", (double)VehicleLocalPositionMsg.Z);
	OS_printf("VehicleLocalPositionMsg.Delta_XY[0] = %f\n", (double)VehicleLocalPositionMsg.Delta_XY[0]);
	OS_printf("VehicleLocalPositionMsg.Delta_XY[1] = %f\n", (double)VehicleLocalPositionMsg.Delta_XY[1]);
	OS_printf("VehicleLocalPositionMsg.Delta_Z = %f\n", (double)VehicleLocalPositionMsg.Delta_Z);
	OS_printf("VehicleLocalPositionMsg.VX = %f\n", (double)VehicleLocalPositionMsg.VX);
	OS_printf("VehicleLocalPositionMsg.VY = %f\n", (double)VehicleLocalPositionMsg.VY);
	OS_printf("VehicleLocalPositionMsg.VZ = %f\n", (double)VehicleLocalPositionMsg.VZ);
	OS_printf("VehicleLocalPositionMsg.Delta_VXY[0] = %f\n", (double)VehicleLocalPositionMsg.Delta_VXY[0]);
	OS_printf("VehicleLocalPositionMsg.Delta_VXY[1] = %f\n", (double)VehicleLocalPositionMsg.Delta_VXY[1]);
	OS_printf("VehicleLocalPositionMsg.Delta_VZ = %f\n", (double)VehicleLocalPositionMsg.Delta_VZ);
	OS_printf("VehicleLocalPositionMsg.AX = %f\n", (double)VehicleLocalPositionMsg.AX);
	OS_printf("VehicleLocalPositionMsg.AY = %f\n", (double)VehicleLocalPositionMsg.AY);
	OS_printf("VehicleLocalPositionMsg.AZ = %f\n", (double)VehicleLocalPositionMsg.AZ);
	OS_printf("VehicleLocalPositionMsg.Yaw = %f\n", (double)VehicleLocalPositionMsg.Yaw);
	OS_printf("VehicleLocalPositionMsg.RefAlt = %f\n", (double)VehicleLocalPositionMsg.RefAlt);
	OS_printf("VehicleLocalPositionMsg.DistBottom = %f\n", (double)VehicleLocalPositionMsg.DistBottom);
	OS_printf("VehicleLocalPositionMsg.DistBottomRate = %f\n", (double)VehicleLocalPositionMsg.DistBottomRate);
	OS_printf("VehicleLocalPositionMsg.EpH = %f\n", (double)VehicleLocalPositionMsg.EpH);
	OS_printf("VehicleLocalPositionMsg.EpV = %f\n", (double)VehicleLocalPositionMsg.EpV);
	OS_printf("VehicleLocalPositionMsg.EvV = %f\n", (double)VehicleLocalPositionMsg.EvV);
	OS_printf("VehicleLocalPositionMsg.EstimatorType = %f\n", (double)VehicleLocalPositionMsg.EstimatorType);
	OS_printf("VehicleLocalPositionMsg.XY_Valid = %u\n", VehicleLocalPositionMsg.XY_Valid);
	OS_printf("VehicleLocalPositionMsg.Z_Valid = %u\n", VehicleLocalPositionMsg.Z_Valid);
	OS_printf("VehicleLocalPositionMsg.V_XY_Valid = %u\n", VehicleLocalPositionMsg.V_XY_Valid);
	OS_printf("VehicleLocalPositionMsg.V_Z_Valid = %u\n", VehicleLocalPositionMsg.V_Z_Valid);
	OS_printf("VehicleLocalPositionMsg.XY_ResetCounter = %u\n", VehicleLocalPositionMsg.XY_ResetCounter);
	OS_printf("VehicleLocalPositionMsg.Z_ResetCounter = %u\n", VehicleLocalPositionMsg.Z_ResetCounter);
	OS_printf("VehicleLocalPositionMsg.VXY_ResetCounter = %u\n", VehicleLocalPositionMsg.VXY_ResetCounter);
	OS_printf("VehicleLocalPositionMsg.VZ_ResetCounter = %u\n", VehicleLocalPositionMsg.VZ_ResetCounter);
	OS_printf("VehicleLocalPositionMsg.XY_Global = %u\n", VehicleLocalPositionMsg.XY_Global);
	OS_printf("VehicleLocalPositionMsg.Z_Global = %u\n", VehicleLocalPositionMsg.Z_Global);
	OS_printf("VehicleLocalPositionMsg.DistBottomValid = %u\n", VehicleLocalPositionMsg.DistBottomValid);

	OS_printf("VehicleLocalPositionSetpointMsg.Timestamp = %llu\n", VehicleLocalPositionSetpointMsg.Timestamp);
	OS_printf("VehicleLocalPositionSetpointMsg.X = %f\n", VehicleLocalPositionSetpointMsg.X);
	OS_printf("VehicleLocalPositionSetpointMsg.Y = %f\n", VehicleLocalPositionSetpointMsg.Y);
	OS_printf("VehicleLocalPositionSetpointMsg.Z = %f\n", VehicleLocalPositionSetpointMsg.Z);
	OS_printf("VehicleLocalPositionSetpointMsg.Yaw = %f\n", VehicleLocalPositionSetpointMsg.Yaw);
	OS_printf("VehicleLocalPositionSetpointMsg.VX = %f\n", VehicleLocalPositionSetpointMsg.VX);
	OS_printf("VehicleLocalPositionSetpointMsg.VY = %f\n", VehicleLocalPositionSetpointMsg.VY);
	OS_printf("VehicleLocalPositionSetpointMsg.VZ = %f\n", VehicleLocalPositionSetpointMsg.VZ);
	OS_printf("VehicleLocalPositionSetpointMsg.AccX = %f\n", VehicleLocalPositionSetpointMsg.AccX);
	OS_printf("VehicleLocalPositionSetpointMsg.AccY = %f\n", VehicleLocalPositionSetpointMsg.AccY);
	OS_printf("VehicleLocalPositionSetpointMsg.AccZ = %f\n", VehicleLocalPositionSetpointMsg.AccZ);

	OS_printf("HomePositionMsg.Timestamp = %llu\n", HomePositionMsg.Timestamp);
	OS_printf("HomePositionMsg.Lat = %f\n", HomePositionMsg.Lat);
	OS_printf("HomePositionMsg.Lon = %f\n", HomePositionMsg.Lon);
	OS_printf("HomePositionMsg.Alt = %f\n", (double)HomePositionMsg.Alt);
	OS_printf("HomePositionMsg.X = %f\n", (double)HomePositionMsg.X);
	OS_printf("HomePositionMsg.Y = %f\n", (double)HomePositionMsg.Y);
	OS_printf("HomePositionMsg.Z = %f\n", (double)HomePositionMsg.Z);
	OS_printf("HomePositionMsg.Yaw = %f\n", (double)HomePositionMsg.Yaw);
	OS_printf("HomePositionMsg.DirectionX = %f\n", (double)HomePositionMsg.DirectionX);
	OS_printf("HomePositionMsg.DirectionY = %f\n", (double)HomePositionMsg.DirectionY);
	OS_printf("HomePositionMsg.DirectionZ = %f\n", (double)HomePositionMsg.DirectionZ);
}



void MPC::DisplayOutputs(void)
{
	OS_printf("***** OUTPUTS **********");
	OS_printf("VehicleAttitudeSetpointMsg.Timestamp = %llu\n", VehicleAttitudeSetpointMsg.Timestamp);
	OS_printf("VehicleAttitudeSetpointMsg.RollBody = %f\n", VehicleAttitudeSetpointMsg.RollBody);
	OS_printf("VehicleAttitudeSetpointMsg.PitchBody = %f\n", VehicleAttitudeSetpointMsg.PitchBody);
	OS_printf("VehicleAttitudeSetpointMsg.YawBody = %f\n", VehicleAttitudeSetpointMsg.YawBody);
	OS_printf("VehicleAttitudeSetpointMsg.YawSpMoveRate = %f\n", VehicleAttitudeSetpointMsg.YawSpMoveRate);
	for(uint32 i = 0; i < 4; ++i)
	{
		OS_printf("VehicleAttitudeSetpointMsg.Q_D[%u] = %f\n", i, VehicleAttitudeSetpointMsg.Q_D[i]);
	}
	OS_printf("VehicleAttitudeSetpointMsg.Q_D_Valid = %u\n", VehicleAttitudeSetpointMsg.Q_D_Valid);
	OS_printf("VehicleAttitudeSetpointMsg.Thrust = %f\n", VehicleAttitudeSetpointMsg.Thrust);
	OS_printf("VehicleAttitudeSetpointMsg.RollResetIntegral = %u\n", VehicleAttitudeSetpointMsg.RollResetIntegral);
	OS_printf("VehicleAttitudeSetpointMsg.PitchResetIntegral = %u\n", VehicleAttitudeSetpointMsg.PitchResetIntegral);
	OS_printf("VehicleAttitudeSetpointMsg.YawResetIntegral = %u\n", VehicleAttitudeSetpointMsg.YawResetIntegral);
	OS_printf("VehicleAttitudeSetpointMsg.FwControlYaw = %u\n", VehicleAttitudeSetpointMsg.FwControlYaw);
	OS_printf("VehicleAttitudeSetpointMsg.DisableMcYawControl = %u\n", VehicleAttitudeSetpointMsg.DisableMcYawControl);
	OS_printf("VehicleAttitudeSetpointMsg.ApplyFlaps = %u\n", VehicleAttitudeSetpointMsg.ApplyFlaps);
	OS_printf("VehicleAttitudeSetpointMsg.LandingGear = %f\n", VehicleAttitudeSetpointMsg.LandingGear);

	OS_printf("VehicleLocalPositionSetpointMsg.Timestamp = %llu\n", VehicleLocalPositionSetpointMsg.Timestamp);
	OS_printf("VehicleLocalPositionSetpointMsg.X = %f\n", VehicleLocalPositionSetpointMsg.X);
	OS_printf("VehicleLocalPositionSetpointMsg.Y = %f\n", VehicleLocalPositionSetpointMsg.Y);
	OS_printf("VehicleLocalPositionSetpointMsg.Z = %f\n", VehicleLocalPositionSetpointMsg.Z);
	OS_printf("VehicleLocalPositionSetpointMsg.Yaw = %f\n", VehicleLocalPositionSetpointMsg.Yaw);
	OS_printf("VehicleLocalPositionSetpointMsg.VX = %f\n", VehicleLocalPositionSetpointMsg.VX);
	OS_printf("VehicleLocalPositionSetpointMsg.VY = %f\n", VehicleLocalPositionSetpointMsg.VY);
	OS_printf("VehicleLocalPositionSetpointMsg.VZ = %f\n", VehicleLocalPositionSetpointMsg.VZ);
	OS_printf("VehicleLocalPositionSetpointMsg.AccX = %f\n", VehicleLocalPositionSetpointMsg.AccX);
	OS_printf("VehicleLocalPositionSetpointMsg.AccY = %f\n", VehicleLocalPositionSetpointMsg.AccY);
	OS_printf("VehicleLocalPositionSetpointMsg.AccZ = %f\n", VehicleLocalPositionSetpointMsg.AccZ);

	OS_printf("VehicleGlobalVelocitySetpointMsg.Timestamp = %llu\n", VehicleGlobalVelocitySetpointMsg.Timestamp);
	OS_printf("VehicleGlobalVelocitySetpointMsg.VX = %f\n", VehicleGlobalVelocitySetpointMsg.VX);
	OS_printf("VehicleGlobalVelocitySetpointMsg.VY = %f\n", VehicleGlobalVelocitySetpointMsg.VY);
	OS_printf("VehicleGlobalVelocitySetpointMsg.VZ = %f\n", VehicleGlobalVelocitySetpointMsg.VZ);
}



void MPC::DisplayMembers(void)
{
	OS_printf("***** MEMBERS **********");
	OS_printf("GearStateInitialized = %u\n", GearStateInitialized);
	OS_printf("RefAlt = %f\n", (double)RefAlt);
	OS_printf("RefPos.timestamp = %llu\n", RefPos.timestamp);
	OS_printf("RefPos.lat_rad = %f\n", (double)RefPos.lat_rad);
	OS_printf("RefPos.lon_rad = %f\n", (double)RefPos.lon_rad);
	OS_printf("RefPos.sin_lat = %f\n", (double)RefPos.sin_lat);
	OS_printf("RefPos.cos_lat = %f\n", (double)RefPos.cos_lat);
	OS_printf("RefPos.init_done = %u\n", RefPos.init_done);
	OS_printf("RefTimestamp = %llu\n", RefTimestamp);
	OS_printf("ResetPositionSetpoint = %u\n", ResetPositionSetpoint);
	OS_printf("ResetAltitudeSetpoint = %u\n", ResetAltitudeSetpoint);
	OS_printf("RunPosControl = %u\n", RunPosControl);
	OS_printf("RunAltControl = %u\n", RunAltControl);
	OS_printf("DoResetAltPos = %u\n", DoResetAltPos);
	OS_printf("ModeAuto = %u\n", ModeAuto);
	OS_printf("PositionHoldEngaged = %u\n", PositionHoldEngaged);
	OS_printf("AltitudeHoldEngaged = %u\n", AltitudeHoldEngaged);
	OS_printf("RunPosControl = %u\n", RunPosControl);
	OS_printf("RunAltControl = %u\n", RunAltControl);
	OS_printf("ResetIntZ = %u\n", ResetIntZ);
	OS_printf("ResetIntXY = %u\n", ResetIntXY);
	OS_printf("ResetIntZManual = %u\n", ResetIntZManual);
	OS_printf("ResetYawSetpoint = %u\n", ResetYawSetpoint);
	OS_printf("HoldOffboardXY = %u\n", HoldOffboardXY);
	OS_printf("HoldOffboardZ = %u\n", HoldOffboardZ);
	OS_printf("LimitVelXY = %u\n", LimitVelXY);
	for(uint32 i = 0; i < 3; ++i)
	{
		OS_printf("ThrustInt[%u] = %f\n", i, (double)ThrustInt[i] );
	}
	for(uint32 i = 0; i < 3; ++i)
	{
		OS_printf("Position[%u] = %f\n", i, (double)Position[i] );
	}
	for(uint32 i = 0; i < 3; ++i)
	{
		OS_printf("PositionSetpoint[%u] = %f\n", i, (double)PositionSetpoint[i] );
	}
	for(uint32 i = 0; i < 3; ++i)
	{
		OS_printf("Velocity[%u] = %f\n", i, (double)Velocity[i] );
	}
	for(uint32 i = 0; i < 3; ++i)
	{
		OS_printf("VelocitySetpoint[%u] = %f\n", i, (double)VelocitySetpoint[i] );
	}
	for(uint32 i = 0; i < 3; ++i)
	{
		OS_printf("VelocityPrevious[%u] = %f\n", i, (double)VelocityPrevious[i] );
	}
	for(uint32 i = 0; i < 3; ++i)
	{
		OS_printf("VelocityFF[%u] = %f\n", i, (double)VelocityFF[i] );
	}
	for(uint32 i = 0; i < 3; ++i)
	{
		OS_printf("VelocitySetpointPrevious[%u] = %f\n", i, (double)VelocitySetpointPrevious[i] );
	}
	for(uint32 i = 0; i < 3; ++i)
	{
		OS_printf("VelocityErrD[%u] = %f\n", i, (double)VelocityErrD[i] );
	}
	for(uint32 i = 0; i < 3; ++i)
	{
		OS_printf("CurrentPositionSetpoint[%u] = %f\n", i, (double)CurrentPositionSetpoint[i] );
	}
	for(uint32 x = 0; x < 3; ++x)
	{
		for(uint32 y = 0; y < 3; ++y)
		{
			OS_printf("Rotation[%u][%u] = %f\n", x, y, (double)Rotation[x][y]);
		}
	}
	OS_printf("Yaw = %f\n", (double)Yaw);
	OS_printf("YawTakeoff = %f\n", (double)YawTakeoff);
	OS_printf("InLanding = %u\n", InLanding);
	OS_printf("LndReachedGround = %u\n", LndReachedGround);
	OS_printf("VelZLp = %f\n", (double)VelZLp);
	OS_printf("AccZLp = %f\n", (double)AccZLp);
	OS_printf("VelMaxXY = %f\n", (double)VelMaxXY);
	OS_printf("InTakeoff = %u\n", InTakeoff);
	OS_printf("TakeoffVelLimit = %f\n", (double)TakeoffVelLimit);
	OS_printf("Z_ResetCounter = %u\n", Z_ResetCounter);
	OS_printf("XY_ResetCounter = %u\n", XY_ResetCounter);
	OS_printf("VZ_ResetCounter = %u\n", VZ_ResetCounter);
	OS_printf("VXY_ResetCounter = %u\n", VXY_ResetCounter);
	OS_printf("HeadingResetCounter = %u\n", HeadingResetCounter);
	for(uint32 x = 0; x < 3; ++x)
	{
		for(uint32 y = 0; y < 3; ++y)
		{
			OS_printf("RSetpoint[%u][%u] = %f\n", x, y, (double)RSetpoint[x][y]);
		}
	}
}

/************************/
/*  End of File Comment */
/************************/
