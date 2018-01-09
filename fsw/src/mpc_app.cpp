/************************************************************************
** Includes
*************************************************************************/
#include <string.h>

#include "cfe.h"

#include "mpc_app.h"
#include "mpc_msg.h"
#include "mpc_version.h"


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

	CFE_SB_InitMsg(&VehicleGlobalVelocitySetpointMsg,
		PX4_VEHICLE_GLOBAL_VELOCITY_SETPOINT_MID, sizeof(PX4_VehicleGlobalVelocitySetpointMsg_t), TRUE);
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
                /* TODO:  Do something here. */
                break;

            case MPC_SEND_HK_MID:
                ProcessCmdPipe();
                ReportHousekeeping();
                break;

            case PX4_CONTROL_STATE_MID:
                memcpy(&CVT.ControlStateMsg, MsgPtr, sizeof(CVT.ControlStateMsg));
//                OS_printf("************************\n");
//                OS_printf("Timestamp = %llu\n", CVT.ControlStateMsg.Timestamp);
//                OS_printf("AccX = %f\n", (double)CVT.ControlStateMsg.AccX);
//                OS_printf("AccY = %f\n", (double)CVT.ControlStateMsg.AccY);
//                OS_printf("AccZ = %f\n", (double)CVT.ControlStateMsg.AccZ);
//                OS_printf("VelX = %f\n", (double)CVT.ControlStateMsg.VelX);
//                OS_printf("VelY = %f\n", (double)CVT.ControlStateMsg.VelY);
//                OS_printf("VelZ = %f\n", (double)CVT.ControlStateMsg.VelZ);
//                OS_printf("PosX = %f\n", (double)CVT.ControlStateMsg.PosX);
//                OS_printf("PosY = %f\n", (double)CVT.ControlStateMsg.PosY);
//                OS_printf("PosZ = %f\n", (double)CVT.ControlStateMsg.PosZ);
//                OS_printf("Airspeed = %f\n", (double)CVT.ControlStateMsg.Airspeed);
//                OS_printf("Airspeed = %f\n", (double)CVT.ControlStateMsg.Airspeed);
//                OS_printf("Airspeed = %f\n", (double)CVT.ControlStateMsg.Airspeed);
//                OS_printf("Airspeed = %f\n", (double)CVT.ControlStateMsg.Airspeed);
//                OS_printf("Airspeed = %f\n", (double)CVT.ControlStateMsg.Airspeed);
//                OS_printf("VelVariance[0] = %f\n", (double)CVT.ControlStateMsg.VelVariance[0]);
//                OS_printf("VelVariance[1] = %f\n", (double)CVT.ControlStateMsg.VelVariance[1]);
//                OS_printf("VelVariance[2] = %f\n", (double)CVT.ControlStateMsg.VelVariance[2]);
//                OS_printf("PosVariance[0] = %f\n", (double)CVT.ControlStateMsg.PosVariance[0]);
//                OS_printf("PosVariance[1] = %f\n", (double)CVT.ControlStateMsg.PosVariance[1]);
//                OS_printf("PosVariance[2] = %f\n", (double)CVT.ControlStateMsg.PosVariance[2]);
//                OS_printf("Q[0] = %f\n", (double)CVT.ControlStateMsg.Q[0]);
//                OS_printf("Q[1] = %f\n", (double)CVT.ControlStateMsg.Q[1]);
//                OS_printf("Q[2] = %f\n", (double)CVT.ControlStateMsg.Q[2]);
//                OS_printf("Q[3] = %f\n", (double)CVT.ControlStateMsg.Q[3]);
//                OS_printf("DeltaQReset[0] = %f\n", (double)CVT.ControlStateMsg.DeltaQReset[0]);
//                OS_printf("DeltaQReset[1] = %f\n", (double)CVT.ControlStateMsg.DeltaQReset[1]);
//                OS_printf("DeltaQReset[2] = %f\n", (double)CVT.ControlStateMsg.DeltaQReset[2]);
//                OS_printf("DeltaQReset[3] = %f\n", (double)CVT.ControlStateMsg.DeltaQReset[3]);
//                OS_printf("RollRate = %f\n", (double)CVT.ControlStateMsg.RollRate);
//                OS_printf("PitchRate = %f\n", (double)CVT.ControlStateMsg.PitchRate);
//                OS_printf("YawRate = %f\n", (double)CVT.ControlStateMsg.YawRate);
//                OS_printf("HorzAccMag = %f\n", (double)CVT.ControlStateMsg.HorzAccMag);
//                OS_printf("RollRateBias = %f\n", (double)CVT.ControlStateMsg.RollRateBias);
//                OS_printf("PitchRateBias = %f\n", (double)CVT.ControlStateMsg.PitchRateBias);
//                OS_printf("YawRateBias = %f\n", (double)CVT.ControlStateMsg.YawRateBias);
//                OS_printf("AirspeedValid = %u\n", CVT.ControlStateMsg.AirspeedValid);
//                OS_printf("QuatResetCounter = %u\n", CVT.ControlStateMsg.QuatResetCounter);
                break;

            case PX4_MANUAL_CONTROL_SETPOINT_MID:
                memcpy(&CVT.ManualControlSetpointMsg, MsgPtr, sizeof(CVT.ManualControlSetpointMsg));
                OS_printf("************************\n");
                OS_printf("Timestamp = %llu\n", CVT.ManualControlSetpointMsg.Timestamp);
                OS_printf("X = %f\n", (double)CVT.ManualControlSetpointMsg.X);
                OS_printf("Y = %f\n", (double)CVT.ManualControlSetpointMsg.Y);
                OS_printf("Z = %f\n", (double)CVT.ManualControlSetpointMsg.Z);
                OS_printf("R = %f\n", (double)CVT.ManualControlSetpointMsg.R);
                OS_printf("Flaps = %f\n", (double)CVT.ManualControlSetpointMsg.Flaps);
                OS_printf("Aux1 = %f\n", (double)CVT.ManualControlSetpointMsg.Aux1);
                OS_printf("Aux2 = %f\n", (double)CVT.ManualControlSetpointMsg.Aux2);
                OS_printf("Aux3 = %f\n", (double)CVT.ManualControlSetpointMsg.Aux3);
                OS_printf("Aux4 = %f\n", (double)CVT.ManualControlSetpointMsg.Aux4);
                OS_printf("Aux5 = %f\n", (double)CVT.ManualControlSetpointMsg.Aux5);
                OS_printf("ModeSwitch = %u\n", CVT.ManualControlSetpointMsg.ModeSwitch);
                OS_printf("ReturnSwitch = %u\n", CVT.ManualControlSetpointMsg.ReturnSwitch);
                OS_printf("RattitudeSwitch = %u\n", CVT.ManualControlSetpointMsg.RattitudeSwitch);
                OS_printf("PosctlSwitch = %u\n", CVT.ManualControlSetpointMsg.PosctlSwitch);
                OS_printf("LoiterSwitch = %u\n", CVT.ManualControlSetpointMsg.LoiterSwitch);
                OS_printf("AcroSwitch = %u\n", CVT.ManualControlSetpointMsg.AcroSwitch);
                OS_printf("OffboardSwitch = %u\n", CVT.ManualControlSetpointMsg.OffboardSwitch);
                OS_printf("KillSwitch = %u\n", CVT.ManualControlSetpointMsg.KillSwitch);
                OS_printf("TransitionSwitch = %u\n", CVT.ManualControlSetpointMsg.TransitionSwitch);
                OS_printf("GearSwitch = %u\n", CVT.ManualControlSetpointMsg.GearSwitch);
                OS_printf("ArmSwitch = %u\n", CVT.ManualControlSetpointMsg.ArmSwitch);
                OS_printf("StabSwitch = %u\n", CVT.ManualControlSetpointMsg.StabSwitch);
                OS_printf("ManSwitch = %u\n", CVT.ManualControlSetpointMsg.ManSwitch);
                OS_printf("ModeSlot = %i\n", CVT.ManualControlSetpointMsg.ModeSlot);
                OS_printf("DataSource = %u\n", CVT.ManualControlSetpointMsg.DataSource);
                break;

            case PX4_HOME_POSITION_MID:
                memcpy(&CVT.HomePositionMsg, MsgPtr, sizeof(CVT.HomePositionMsg));
//                OS_printf("************************\n");
//                OS_printf("Timestamp = %llu\n", CVT.HomePositionMsg.Timestamp);
//                OS_printf("Lat = %f\n", CVT.HomePositionMsg.Lat);
//                OS_printf("Lon = %f\n", CVT.HomePositionMsg.Lon);
//                OS_printf("Alt = %f\n", (double)CVT.HomePositionMsg.Alt);
//                OS_printf("X = %f\n", (double)CVT.HomePositionMsg.X);
//                OS_printf("Y = %f\n", (double)CVT.HomePositionMsg.Y);
//                OS_printf("Z = %f\n", (double)CVT.HomePositionMsg.Z);
//                OS_printf("Yaw = %f\n", (double)CVT.HomePositionMsg.Yaw);
//                OS_printf("DirectionX = %f\n", (double)CVT.HomePositionMsg.DirectionX);
//                OS_printf("DirectionY = %f\n", (double)CVT.HomePositionMsg.DirectionY);
//                OS_printf("DirectionZ = %f\n", (double)CVT.HomePositionMsg.DirectionZ);
                break;

            case PX4_VEHICLE_CONTROL_MODE_MID:
                memcpy(&CVT.VehicleControlModeMsg, MsgPtr, sizeof(CVT.VehicleControlModeMsg));
//                OS_printf("VehicleControlModeMsg ************************\n");
//                OS_printf("Timestamp = %llu\n", CVT.VehicleControlModeMsg.Timestamp);
//                OS_printf("Armed = %u\n", CVT.VehicleControlModeMsg.Armed);
//                OS_printf("ExternalManualOverrideOk = %u\n", CVT.VehicleControlModeMsg.ExternalManualOverrideOk);
//                OS_printf("SystemHilEnabled = %u\n", CVT.VehicleControlModeMsg.SystemHilEnabled);
//                OS_printf("ControlManualEnabled = %u\n", CVT.VehicleControlModeMsg.ControlManualEnabled);
//                OS_printf("ControlAutoEnabled = %u\n", CVT.VehicleControlModeMsg.ControlAutoEnabled);
//                OS_printf("ControlOffboardEnabled = %u\n", CVT.VehicleControlModeMsg.ControlOffboardEnabled);
//                OS_printf("ControlRatesEnabled = %u\n", CVT.VehicleControlModeMsg.ControlRatesEnabled);
//                OS_printf("ControlAttitudeEnabled = %u\n", CVT.VehicleControlModeMsg.ControlAttitudeEnabled);
//                OS_printf("ControlRattitudeEnabled = %u\n", CVT.VehicleControlModeMsg.ControlRattitudeEnabled);
//                OS_printf("ControlForceEnabled = %u\n", CVT.VehicleControlModeMsg.ControlForceEnabled);
//                OS_printf("ControlAccelerationEnabled = %u\n", CVT.VehicleControlModeMsg.ControlAccelerationEnabled);
//                OS_printf("ControlVelocityEnabled = %u\n", CVT.VehicleControlModeMsg.ControlVelocityEnabled);
//                OS_printf("ControlPositionEnabled = %u\n", CVT.VehicleControlModeMsg.ControlPositionEnabled);
//                OS_printf("ControlAltitudeEnabled = %u\n", CVT.VehicleControlModeMsg.ControlAltitudeEnabled);
//                OS_printf("ControlClimbRateEnabled = %u\n", CVT.VehicleControlModeMsg.ControlClimbRateEnabled);
//                OS_printf("ControlTerminationEnabled = %u\n", CVT.VehicleControlModeMsg.ControlTerminationEnabled);
                break;

            case PX4_POSITION_SETPOINT_TRIPLET_MID:
                memcpy(&CVT.PositionSetpointTripletMsg, MsgPtr, sizeof(CVT.PositionSetpointTripletMsg));
//                OS_printf("PositionSetpointTripletMsg ************************\n");
//                OS_printf("Timestamp = %llu\n", CVT.PositionSetpointTripletMsg.Timestamp);
//                OS_printf("Previous.Timestamp = %llu\n", CVT.PositionSetpointTripletMsg.Previous.Timestamp);
//                OS_printf("Previous.Lat = %f\n", CVT.PositionSetpointTripletMsg.Previous.Lat);
//                OS_printf("Previous.Lon = %f\n", CVT.PositionSetpointTripletMsg.Previous.Lon);
//                OS_printf("Previous.X = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.X);
//                OS_printf("Previous.Y = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.Y);
//                OS_printf("Previous.Z = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.Z);
//                OS_printf("Previous.VX = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.VX);
//                OS_printf("Previous.VY = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.VY);
//                OS_printf("Previous.VZ = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.VZ);
//                OS_printf("Previous.Alt = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.Alt);
//                OS_printf("Previous.Yaw = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.Yaw);
//                OS_printf("Previous.Yawspeed = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.Yawspeed);
//                OS_printf("Previous.LoiterRadius = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.LoiterRadius);
//                OS_printf("Previous.PitchMin = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.PitchMin);
//                OS_printf("Previous.A_X = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.A_X);
//                OS_printf("Previous.A_Y = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.A_Y);
//                OS_printf("Previous.A_Z = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.A_Z);
//                OS_printf("Previous.AcceptanceRadius = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.AcceptanceRadius);
//                OS_printf("Previous.CruisingSpeed = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.CruisingSpeed);
//                OS_printf("Previous.CruisingThrottle = %f\n", (double)CVT.PositionSetpointTripletMsg.Previous.CruisingThrottle);
//                OS_printf("Previous.Valid = %u\n", CVT.PositionSetpointTripletMsg.Previous.Valid);
//                OS_printf("Previous.Type = %u\n", CVT.PositionSetpointTripletMsg.Previous.Type);
//                OS_printf("Previous.PositionValid = %u\n", CVT.PositionSetpointTripletMsg.Previous.PositionValid);
//                OS_printf("Previous.VelocityValid = %u\n", CVT.PositionSetpointTripletMsg.Previous.VelocityValid);
//                OS_printf("Previous.YawValid = %u\n", CVT.PositionSetpointTripletMsg.Previous.YawValid);
//                OS_printf("Previous.DisableMcYawControl = %u\n", CVT.PositionSetpointTripletMsg.Previous.DisableMcYawControl);
//                OS_printf("Previous.YawspeedValid = %u\n", CVT.PositionSetpointTripletMsg.Previous.YawspeedValid);
//                OS_printf("Previous.LoiterDirection = %i\n", CVT.PositionSetpointTripletMsg.Previous.LoiterDirection);
//                OS_printf("Previous.AccelerationValid = %u\n", CVT.PositionSetpointTripletMsg.Previous.AccelerationValid);
//                OS_printf("Previous.AccelerationIsForce = %u\n", CVT.PositionSetpointTripletMsg.Previous.AccelerationIsForce);
//
//                OS_printf("Current.Timestamp = %llu\n", CVT.PositionSetpointTripletMsg.Current.Timestamp);
//                OS_printf("Current.Lat = %f\n", CVT.PositionSetpointTripletMsg.Current.Lat);
//                OS_printf("Current.Lon = %f\n", CVT.PositionSetpointTripletMsg.Current.Lon);
//                OS_printf("Current.X = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.X);
//                OS_printf("Current.Y = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.Y);
//                OS_printf("Current.Z = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.Z);
//                OS_printf("Current.VX = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.VX);
//                OS_printf("Current.VY = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.VY);
//                OS_printf("Current.VZ = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.VZ);
//                OS_printf("Current.Alt = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.Alt);
//                OS_printf("Current.Yaw = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.Yaw);
//                OS_printf("Current.Yawspeed = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.Yawspeed);
//                OS_printf("Current.LoiterRadius = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.LoiterRadius);
//                OS_printf("Current.PitchMin = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.PitchMin);
//                OS_printf("Current.A_X = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.A_X);
//                OS_printf("Current.A_Y = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.A_Y);
//                OS_printf("Current.A_Z = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.A_Z);
//                OS_printf("Current.AcceptanceRadius = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.AcceptanceRadius);
//                OS_printf("Current.CruisingSpeed = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.CruisingSpeed);
//                OS_printf("Current.CruisingThrottle = %f\n", (double)CVT.PositionSetpointTripletMsg.Current.CruisingThrottle);
//                OS_printf("Current.Valid = %u\n", CVT.PositionSetpointTripletMsg.Current.Valid);
//                OS_printf("Current.Type = %u\n", CVT.PositionSetpointTripletMsg.Current.Type);
//                OS_printf("Current.PositionValid = %u\n", CVT.PositionSetpointTripletMsg.Current.PositionValid);
//                OS_printf("Current.VelocityValid = %u\n", CVT.PositionSetpointTripletMsg.Current.VelocityValid);
//                OS_printf("Current.YawValid = %u\n", CVT.PositionSetpointTripletMsg.Current.YawValid);
//                OS_printf("Current.DisableMcYawControl = %u\n", CVT.PositionSetpointTripletMsg.Current.DisableMcYawControl);
//                OS_printf("Current.YawspeedValid = %u\n", CVT.PositionSetpointTripletMsg.Current.YawspeedValid);
//                OS_printf("Current.LoiterDirection = %i\n", CVT.PositionSetpointTripletMsg.Current.LoiterDirection);
//                OS_printf("Current.AccelerationValid = %u\n", CVT.PositionSetpointTripletMsg.Current.AccelerationValid);
//                OS_printf("Current.AccelerationIsForce = %u\n", CVT.PositionSetpointTripletMsg.Current.AccelerationIsForce);
//
//                OS_printf("Next.Timestamp = %llu\n", CVT.PositionSetpointTripletMsg.Next.Timestamp);
//                OS_printf("Next.Lat = %f\n", CVT.PositionSetpointTripletMsg.Next.Lat);
//                OS_printf("Next.Lon = %f\n", CVT.PositionSetpointTripletMsg.Next.Lon);
//                OS_printf("Next.X = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.X);
//                OS_printf("Next.Y = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.Y);
//                OS_printf("Next.Z = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.Z);
//                OS_printf("Next.VX = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.VX);
//                OS_printf("Next.VY = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.VY);
//                OS_printf("Next.VZ = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.VZ);
//                OS_printf("Next.Alt = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.Alt);
//                OS_printf("Next.Yaw = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.Yaw);
//                OS_printf("Next.Yawspeed = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.Yawspeed);
//                OS_printf("Next.LoiterRadius = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.LoiterRadius);
//                OS_printf("Next.PitchMin = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.PitchMin);
//                OS_printf("Next.A_X = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.A_X);
//                OS_printf("Next.A_Y = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.A_Y);
//                OS_printf("Next.A_Z = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.A_Z);
//                OS_printf("Next.AcceptanceRadius = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.AcceptanceRadius);
//                OS_printf("Next.CruisingSpeed = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.CruisingSpeed);
//                OS_printf("Next.CruisingThrottle = %f\n", (double)CVT.PositionSetpointTripletMsg.Next.CruisingThrottle);
//                OS_printf("Next.Valid = %u\n", CVT.PositionSetpointTripletMsg.Next.Valid);
//                OS_printf("Next.Type = %u\n", CVT.PositionSetpointTripletMsg.Next.Type);
//                OS_printf("Next.PositionValid = %u\n", CVT.PositionSetpointTripletMsg.Next.PositionValid);
//                OS_printf("Next.VelocityValid = %u\n", CVT.PositionSetpointTripletMsg.Next.VelocityValid);
//                OS_printf("Next.YawValid = %u\n", CVT.PositionSetpointTripletMsg.Next.YawValid);
//                OS_printf("Next.DisableMcYawControl = %u\n", CVT.PositionSetpointTripletMsg.Next.DisableMcYawControl);
//                OS_printf("Next.YawspeedValid = %u\n", CVT.PositionSetpointTripletMsg.Next.YawspeedValid);
//                OS_printf("Next.LoiterDirection = %i\n", CVT.PositionSetpointTripletMsg.Next.LoiterDirection);
//                OS_printf("Next.AccelerationValid = %u\n", CVT.PositionSetpointTripletMsg.Next.AccelerationValid);
//                OS_printf("Next.AccelerationIsForce = %u\n", CVT.PositionSetpointTripletMsg.Next.AccelerationIsForce);

                break;

            case PX4_VEHICLE_STATUS_MID:
                memcpy(&CVT.VehicleStatusMsg, MsgPtr, sizeof(CVT.VehicleStatusMsg));
//                OS_printf("VehicleStatusMsg ************************\n");
//                OS_printf("Timestamp = %llu\n", CVT.VehicleStatusMsg.Timestamp);
//                OS_printf("SystemID = %u\n", CVT.VehicleStatusMsg.SystemID);
//                OS_printf("ComponentID = %u\n", CVT.VehicleStatusMsg.ComponentID);
//                OS_printf("OnboardControlSensorsPresent = %u\n", CVT.VehicleStatusMsg.OnboardControlSensorsPresent);
//                OS_printf("OnboardControlSensorsEnabled = %u\n", CVT.VehicleStatusMsg.OnboardControlSensorsEnabled);
//                OS_printf("OnboardControlSensorsHealth = %u\n", CVT.VehicleStatusMsg.OnboardControlSensorsHealth);
//                OS_printf("NavState = %u\n", CVT.VehicleStatusMsg.NavState);
//                OS_printf("ArmingState = %u\n", CVT.VehicleStatusMsg.ArmingState);
//                OS_printf("HilState = %u\n", CVT.VehicleStatusMsg.HilState);
//                OS_printf("Failsafe = %u\n", CVT.VehicleStatusMsg.Failsafe);
//                OS_printf("SystemType = %u\n", CVT.VehicleStatusMsg.SystemType);
//                OS_printf("IsRotaryWing = %u\n", CVT.VehicleStatusMsg.IsRotaryWing);
//                OS_printf("IsVtol = %u\n", CVT.VehicleStatusMsg.IsVtol);
//                OS_printf("VtolFwPermanentStab = %u\n", CVT.VehicleStatusMsg.VtolFwPermanentStab);
//                OS_printf("InTransitionMode = %u\n", CVT.VehicleStatusMsg.InTransitionMode);
//                OS_printf("RcSignalLost = %u\n", CVT.VehicleStatusMsg.RcSignalLost);
//                OS_printf("RcInputMode = %u\n", CVT.VehicleStatusMsg.RcInputMode);
//                OS_printf("DataLinkLost = %u\n", CVT.VehicleStatusMsg.DataLinkLost);
//                OS_printf("DataLinkLostCounter = %u\n", CVT.VehicleStatusMsg.DataLinkLostCounter);
//                OS_printf("EngineFailure = %u\n", CVT.VehicleStatusMsg.EngineFailure);
//                OS_printf("EngineFailureCmd = %u\n", CVT.VehicleStatusMsg.EngineFailureCmd);
//                OS_printf("MissionFailure = %u\n", CVT.VehicleStatusMsg.MissionFailure);

                break;

            case PX4_VEHICLE_LAND_DETECTED_MID:
                memcpy(&CVT.VehicleLandDetectedMsg, MsgPtr, sizeof(CVT.VehicleLandDetectedMsg));
//                OS_printf("VehicleStatusMsg ************************\n");
//                OS_printf("Timestamp = %llu\n", CVT.VehicleLandDetectedMsg.Timestamp);
//                OS_printf("AltMax = %f\n", (double)CVT.VehicleLandDetectedMsg.AltMax);
//                OS_printf("Landed = %u\n", CVT.VehicleLandDetectedMsg.Landed);
//                OS_printf("Freefall = %u\n", CVT.VehicleLandDetectedMsg.Freefall);
//                OS_printf("GroundContact = %u\n", CVT.VehicleLandDetectedMsg.GroundContact);
                break;

            case PX4_VEHICLE_LOCAL_POSITION_MID:
                memcpy(&CVT.VehicleLocalPositionMsg, MsgPtr, sizeof(CVT.VehicleLocalPositionMsg));
//                OS_printf("VehicleLocalPositionMsg ************************\n");
//                OS_printf("Timestamp = %llu\n", CVT.VehicleLocalPositionMsg.Timestamp);
//                OS_printf("RefTimestamp = %llu\n", CVT.VehicleLocalPositionMsg.RefTimestamp);
//                OS_printf("RefLat = %f\n", (double)CVT.VehicleLocalPositionMsg.RefLat);
//                OS_printf("RefLon = %f\n", (double)CVT.VehicleLocalPositionMsg.RefLon);
//                OS_printf("SurfaceBottomTimestamp = %llu\n", CVT.VehicleLocalPositionMsg.SurfaceBottomTimestamp);
//                OS_printf("X = %f\n", (double)CVT.VehicleLocalPositionMsg.X);
//                OS_printf("Y = %f\n", (double)CVT.VehicleLocalPositionMsg.Y);
//                OS_printf("Z = %f\n", (double)CVT.VehicleLocalPositionMsg.Z);
//                OS_printf("VX = %f\n", (double)CVT.VehicleLocalPositionMsg.VX);
//                OS_printf("VY = %f\n", (double)CVT.VehicleLocalPositionMsg.VY);
//                OS_printf("VZ = %f\n", (double)CVT.VehicleLocalPositionMsg.VZ);
//                OS_printf("Yaw = %f\n", (double)CVT.VehicleLocalPositionMsg.Yaw);
//                OS_printf("RefAlt = %f\n", (double)CVT.VehicleLocalPositionMsg.RefAlt);
//                OS_printf("DistBottom = %f\n", (double)CVT.VehicleLocalPositionMsg.DistBottom);
//                OS_printf("DistBottomRate = %f\n", (double)CVT.VehicleLocalPositionMsg.DistBottomRate);
//                OS_printf("EpH = %f\n", (double)CVT.VehicleLocalPositionMsg.EpH);
//                OS_printf("EpV = %f\n", (double)CVT.VehicleLocalPositionMsg.EpV);
//                OS_printf("XY_Valid = %u\n", CVT.VehicleLocalPositionMsg.XY_Valid);
//                OS_printf("Z_Valid = %u\n", CVT.VehicleLocalPositionMsg.Z_Valid);
//                OS_printf("V_XY_Valid = %u\n", CVT.VehicleLocalPositionMsg.V_XY_Valid);
//                OS_printf("V_Z_Valid = %u\n", CVT.VehicleLocalPositionMsg.V_Z_Valid);
//                OS_printf("XY_Global = %u\n", CVT.VehicleLocalPositionMsg.XY_Global);
//                OS_printf("Z_Global = %u\n", CVT.VehicleLocalPositionMsg.Z_Global);
//                OS_printf("DistBottomValid = %u\n", CVT.VehicleLocalPositionMsg.DistBottomValid);

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


/************************/
/*  End of File Comment */
/************************/
