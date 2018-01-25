/****************************************************************************
*
*   Copyright (c) 2017 Windhover Labs, L.L.C. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name Windhover Labs nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/

#ifndef MPC_APP_H
#define MPC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************
 ** Pragmas
 *************************************************************************/

/************************************************************************
 ** Includes
 *************************************************************************/
#include "cfe.h"

#include "mpc_platform_cfg.h"
#include "mpc_mission_cfg.h"
#include "mpc_perfids.h"
#include "mpc_msgids.h"
#include "mpc_msg.h"
#include "mpc_events.h"
#include "mpc_tbldefs.h"
#include "px4_msgs.h"
#include "Matrix3F3.hpp"
#include "geo/geo.h"
#include "Derivative.hpp"

/************************************************************************
 ** Local Defines
 *************************************************************************/

/************************************************************************
 ** Local Structure Definitions
 *************************************************************************/

/**
 **  \brief MPC Application Class
 */
class MPC
{
public:
    MPC();
    ~MPC();

    /**\brief Scheduling Pipe ID */
    CFE_SB_PipeId_t SchPipeId;

    /** \brief Command Pipe ID */
    CFE_SB_PipeId_t CmdPipeId;

    /* Task-related */

    /** \brief Task Run Status */
    uint32 uiRunStatus;

    /* Config table-related */

    /** \brief Config Table Handle */
    CFE_TBL_Handle_t ConfigTblHdl;

    /** \brief Config Table Pointer */
    MPC_ConfigTbl_t* ConfigTblPtr;

    /* Output Messages */
    /** \brief Housekeeping Telemetry for downlink */
    MPC_HkTlm_t HkTlm;
    /** \brief Output Data published at the end of cycle */
    PX4_VehicleAttitudeSetpointMsg_t VehicleAttitudeSetpointMsg;
    //PX4_VehicleLocalVelocitySetpointMsg_t VehicleLocalVelocitySetpointMsg;
    PX4_VehicleLocalPositionSetpointMsg_t VehicleLocalPositionSetpointMsg;
    PX4_VehicleGlobalVelocitySetpointMsg_t VehicleGlobalVelocitySetpointMsg;

    /* Input Messages */
    PX4_ControlStateMsg_t ControlStateMsg;
    PX4_ManualControlSetpointMsg_t ManualControlSetpointMsg;
    PX4_HomePositionMsg_t HomePositionMsg;
    PX4_VehicleControlModeMsg_t VehicleControlModeMsg;
    PX4_PositionSetpointTripletMsg_t PositionSetpointTripletMsg;
    PX4_VehicleStatusMsg_t VehicleStatusMsg;
    PX4_VehicleLandDetectedMsg_t VehicleLandDetectedMsg;
    PX4_VehicleLocalPositionMsg_t VehicleLocalPositionMsg;

	math::Matrix3F3 Rotation; /**< rotation matrix from attitude quaternions */
	float Yaw;				  /**< yaw angle (euler) */
	float YawTakeoff;	      /**< home yaw angle present when vehicle was taking off (euler) */
	bool  InLanding;	      /**< the vehicle is in the landing descent */
	bool  LndReachedGround;   /**< controller assumes the vehicle has reached the ground after landing */
	float VelZLp;
	float AccZLp;
	float VelMaxXY;           /**< equal to vel_max except in auto mode when close to target */
	bool  InTakeoff;	      /**< flag for smooth velocity setpoint takeoff ramp */
	float TakeoffVelLimit;    /**< velocity limit value which gets ramped up */

	uint8 Z_ResetCounter;
	uint8 XY_ResetCounter;
	uint8 VZ_ResetCounter;
	uint8 VXY_ResetCounter;
	uint8 HeadingResetCounter;

	math::Vector3F ThrustInt;

	math::Vector3F Position;
	math::Vector3F PositionSetpoint;
	math::Vector3F Velocity;
	math::Vector3F VelocitySetpoint;
	math::Vector3F VelocityPrevious;			/**< velocity on previous step */
	math::Vector3F VelocityFF;
	math::Vector3F VelocitySetpointPrevious;
	math::Vector3F VelocityErrD;		     /**< derivative of current velocity */
	math::Vector3F CurrentPositionSetpoint;  /**< current setpoint of the triplets */

	math::Matrix3F3 RSetpoint;

	bool ResetPositionSetpoint;
	bool ResetAltitudeSetpoint;
	bool DoResetAltPos;
	bool ModeAuto;
	bool PositionHoldEngaged;
	bool AltitudeHoldEngaged;
	bool RunPosControl;
	bool RunAltControl;

	bool ResetIntZ;
	bool ResetIntXY;
	bool ResetIntZManual;
	bool ResetYawSetpoint;

	bool HoldOffboardXY;
	bool HoldOffboardZ;
	bool LimitVelXY;

	bool GearStateInitialized;

	uint64 RefTimestamp;
	struct map_projection_reference_s RefPos;
	float RefAlt;

	Derivative VelXDeriv;
	Derivative VelYDeriv;
	Derivative VelZDeriv;

	math::Vector3F PosP;
	math::Vector3F VelP;
	math::Vector3F VelI;
	math::Vector3F VelD;

	float VelocityMaxXY;  /**< Equal to vel_max except in auto mode when close to target. */

    /************************************************************************/
    /** \brief Multicopter Position Control (MPC) application entry point
     **
     **  \par Description
     **       Multicopter Position Control Task application entry point.  This function
     **       performs app initialization, then waits for the cFE ES Startup
     **       Sync, then executes the main processing loop.
     **
     **  \par Assumptions, External Events, and Notes:
     **       If there is an unrecoverable failure during initialization the
     **       main loop is never executed and the application will exit.
     **
     *************************************************************************/
    void AppMain(void);

    /************************************************************************/
    /** \brief Initialize the Multicopter Position Control (MPC) application
     **
     **  \par Description
     **       Multicopter Position Control application initialization routine. This
     **       function performs all the required startup steps to
     **       initialize (or restore from CDS) MPC data structures and get
     **       the application registered with the cFE services so it can
     **       begin to receive command messages and send events.
     **
     **  \par Assumptions, External Events, and Notes:
     **       None
     **
     **  \returns
     **  \retcode #CFE_SUCCESS  \retdesc \copydoc CFE_SUCCESS    \endcode
     **  \retstmt Return codes from #CFE_ES_RegisterApp          \endcode
     **  \retstmt Return codes from #MPC_InitEvent               \endcode
     **  \retstmt Return codes from #MPC_InitPipe                \endcode
     **  \retstmt Return codes from #MPC_InitData                \endcode
     **  \retstmt Return codes from #MPC_InitConfigTbl           \endcode
     **  \retstmt Return codes from #OS_TaskInstallDeleteHandler \endcode
     **  \endreturns
     **
     *************************************************************************/
    int32 InitApp(void);

    /************************************************************************/
    /** \brief Initialize Event Services and Event tables
     **
     **  \par Description
     **       This function performs the steps required to setup
     **       cFE Event Services for use by the MPC application.
     **
     **  \par Assumptions, External Events, and Notes:
     **       None
     **
     **  \returns
     **  \retcode #CFE_SUCCESS  \retdesc \copydoc CFE_SUCCESS \endcode
     **  \retstmt Return codes from #CFE_EVS_Register  \endcode
     **  \endreturns
     **
     *************************************************************************/
    int32 InitEvent(void);

    /************************************************************************/
    /** \brief Initialize global variables used by MPC application
     **
     **  \par Description
     **       This function performs the steps required to initialize
     **       the MPC application data.
     **
     **  \par Assumptions, External Events, and Notes:
     **       None
     **
     *************************************************************************/
    void InitData(void);

    /************************************************************************/
    /** \brief Initialize message pipes
     **
     **  \par Description
     **       This function performs the steps required to setup
     **       initialize the cFE Software Bus message pipes and subscribe to
     **       messages for the MPC application.
     **
     **  \par Assumptions, External Events, and Notes:
     **       None
     **
     **  \returns
     **  \retcode #CFE_SUCCESS  \retdesc \copydoc CFE_SUCCESS \endcode
     **  \retstmt Return codes from #CFE_SB_CreatePipe        \endcode
     **  \retstmt Return codes from #CFE_SB_SubscribeEx       \endcode
     **  \retstmt Return codes from #CFE_SB_Subscribe         \endcode
     **  \endreturns
     **
     *************************************************************************/
    int32 InitPipe(void);

    /************************************************************************/
    /** \brief Receive and process messages from the scheduler pipe.
     **
     **  \par Description
     **       This function receives and processes messages
     **       for the MPC application from the SCH pipe.  This function
     **       will pend for the type defined by iBlocking, allowing
     **       it to wait for messages, i.e. wakeup messages from scheduler.
     **
     **  \par Assumptions, External Events, and Notes:
     **       None
     **
     **  \param [in]   iBlocking    A #CFE_SB_PEND_FOREVER, #CFE_SB_POLL or
     **                             millisecond timeout
     **
     **  \returns
     **  \retcode #CFE_SUCCESS  \retdesc \copydoc CFE_SUCCESS \endcode
     **  \retstmt Return codes from #CFE_SB_RcvMsg            \endcode
     **  \endreturns
     **
     *************************************************************************/
    int32 RcvSchPipeMsg(int32 iBlocking);

    /************************************************************************/
    /** \brief Multicopter Position Control Task incoming command processing
     **
     **  \par Description
     **       This function processes incoming commands subscribed
     **       by MPC application
     **
     **  \par Assumptions, External Events, and Notes:
     **       None
     **
     *************************************************************************/
    void ProcessCmdPipe(void);

    /************************************************************************/
    /** \brief Multicopter Position Control Task application commands
     **
     **  \par Description
     **       This function processes command messages
     **       specific to the MPC application
     **
     **  \par Assumptions, External Events, and Notes:
     **       None
     **
     **  \param [in]   MsgPtr       A #CFE_SB_Msg_t pointer that
     **                             references the software bus message
     **
     *************************************************************************/
    void ProcessAppCmds(CFE_SB_Msg_t* MsgPtr);

    /************************************************************************/
    /** \brief Sends MPC housekeeping message
     **
     **  \par Description
     **       This function sends the housekeeping message
     **
     **  \par Assumptions, External Events, and Notes:
     **       None
     **
     *************************************************************************/
    void ReportHousekeeping(void);
    /************************************************************************/
    /** \brief Sends the VehicleAttitudeSetpointMsg message.
     **
     **  \par Description
     **       This function publishes the VehicleAttitudeSetpointMsg message containing
     **       <TODO>
     **
     **  \par Assumptions, External Events, and Notes:
     **       None
     **
     *************************************************************************/
    void SendVehicleAttitudeSetpointMsg(void);

//    /************************************************************************/
//    /** \brief Sends the VehicleLocalVelocitySetpointMsg message.
//     **
//     **  \par Description
//     **       This function publishes the VehicleLocalVelocitySetpointMsg message containing
//     **       <TODO>
//     **
//     **  \par Assumptions, External Events, and Notes:
//     **       None
//     **
//     *************************************************************************/
//    void SendVehicleLocalVelocitySetpointMsg(void);

    /************************************************************************/
    /** \brief Sends the VehicleGlobalVelocitySetpointMsg message.
     **
     **  \par Description
     **       This function publishes the VehicleGlobalVelocitySetpointMsg message containing
     **       <TODO>
     **
     **  \par Assumptions, External Events, and Notes:
     **       None
     **
     *************************************************************************/
    void SendVehicleGlobalVelocitySetpointMsg(void);

    /************************************************************************/
    /** \brief Verify Command Length
     **
     **  \par Description
     **       This function verifies the command message length.
     **
     **  \par Assumptions, External Events, and Notes:
     **       None
     **
     **  \param [in]   MsgPtr        A #CFE_SB_Msg_t pointer that
     **                              references the software bus message
     **  \param [in]   usExpectedLen The expected length of the message
     **
     **  \returns
     **  TRUE if the message length matches expectations, FALSE if it does not.
     **  \endreturns
     **
     *************************************************************************/
    boolean VerifyCmdLength(CFE_SB_Msg_t* MsgPtr, uint16 usExpectedLen);

    void SendVehicleLocalPositionSetpointMsg(void);


public:
    /************************************************************************/
    /** \brief Initialize the MPC configuration tables.
    **
    **  \par Description
    **       This function initializes MPC's configuration tables.  This
    **       includes <TODO>.
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    **  \returns
    **  \retcode #CFE_SUCCESS  \retdesc \copydoc CFE_SUCCESS  \endcode
    **  \retstmt Return codes from #CFE_TBL_Register          \endcode
    **  \retstmt Return codes from #CFE_TBL_Load              \endcode
    **  \retstmt Return codes from #MPC_AcquireConfigPointers \endcode
    **  \endreturns
    **
    *************************************************************************/
    int32  InitConfigTbl(void);

    /************************************************************************/
    /** \brief Obtain MPC configuration tables data pointers.
    **
    **  \par Description
    **       This function manages the configuration tables
    **       and obtains a pointer to their data.
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    **  \returns
    **  \retcode #CFE_SUCCESS  \retdesc \copydoc CFE_SUCCESS  \endcode
    **  \endreturns
    **
    *************************************************************************/
    int32  AcquireConfigPointers(void);

public:
    void ProcessControlStateMsg(void);
    void ProcessVehicleLocalPositionMsg(void);
    void ProcessPositionSetpointTripletMsg(void);
    void Execute(void);
    void UpdateRef(void);
    void UpdateVelocityDerivative(float dt);
    void DoControl(float dt);
    void GenerateAttitudeSetpoint(float dt);
    void ControlManual(float dt);
    void ControlNonManual(float dt);
    float ThrottleCurve(float ctl, float ctr);
    void ResetAltSetpoint(void);
    void ResetPosSetpoint(void);
    void ControlPosition(float dt);
    void ControlOffboard(float dt);
    void ControlAuto(float dt);
	void CalculateVelocitySetpoint(float dt);
	void CalculateThrustSetpoint(float dt);
	float GetCruisingSpeedXY(void);
	bool CrossSphereLine(const math::Vector3F &sphere_c, const float sphere_r,
			const math::Vector3F &line_a, const math::Vector3F &line_b, math::Vector3F &res);
	void UpdateParamsFromTable(void);
	void LimitAltitude(void);
	void SlowLandGradualVelocityLimit(void);
	bool InAutoTakeoff(void);


	/*
	 * Limit vel horizontally when close to target
	 */
	void LimitVelXYGradually(void);


	void ApplyVelocitySetpointSlewRate(float dt);

public:
    /************************************************************************/
    /** \brief Validate MPC configuration table
    **
    **  \par Description
    **       This function validates MPC's configuration table
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    **  \param [in]   ConfigTblPtr    A pointer to the table to validate.
    **
    **  \returns
    **  \retcode #CFE_SUCCESS  \retdesc \copydoc CFE_SUCCESS  \endcode
    **  \endreturns
    **
    *************************************************************************/
    static int32  ValidateConfigTbl(void*);



    void DisplayInputs(void);
    void DisplayOutputs(void);
    void DisplayMembers(void);
    void SetInputs(void);
    void SetMembers(void);
};

#ifdef __cplusplus
}
#endif 

#endif /* MPC_APP_H */

/************************/
/*  End of File Comment */
/************************/
