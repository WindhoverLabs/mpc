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
#include "math/Matrix3F3.hpp"
#include "geo/geo.h"
#include "math/Derivative.hpp"
#include "math/Vector2F.hpp"
#include "systemlib/hysteresis.h"
#include "math/filters/LowPassFilter2p.hpp"

/************************************************************************
 ** Local Defines
 *************************************************************************/
enum ManualStickInput {
	BRAKE = 0,
	DIRECTION_CHANGE = 1,
	ACCELERATION = 2,
	DECELERATION = 3,
	NONE = 4
};


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

    /** \brief Diagnositc Telemetry for downlink */
    MPC_DiagPacket_t DiagTlm;

    /** \brief Output Data published at the end of cycle */
    PX4_VehicleAttitudeSetpointMsg_t m_VehicleAttitudeSetpointMsg;
    PX4_VehicleLocalPositionSetpointMsg_t m_VehicleLocalPositionSetpointMsg;

    /* Input Messages */
    PX4_ControlStateMsg_t m_ControlStateMsg;
    PX4_ManualControlSetpointMsg_t m_ManualControlSetpointMsg;
    PX4_HomePositionMsg_t m_HomePositionMsg;
    PX4_VehicleControlModeMsg_t m_VehicleControlModeMsg;
    PX4_PositionSetpointTripletMsg_t m_PositionSetpointTripletMsg;
    PX4_VehicleStatusMsg_t m_VehicleStatusMsg;
    PX4_VehicleLandDetectedMsg_t m_VehicleLandDetectedMsg;
    PX4_VehicleLocalPositionMsg_t m_VehicleLocalPositionMsg;

	/* Reset counters */
	uint8 m_ResetCounterZ;
	uint8 m_ResetCounterXy;
	uint8 m_HeadingResetCounter;

	/* Control variables used for altitude, position, and yaw hold */
	math::Vector3F m_Position;
	math::Vector3F m_PositionSetpoint;
	math::Vector3F m_Velocity;
	math::Vector3F m_VelocityPrevious;			/**< velocity on previous step */
	math::Vector3F m_VelocitySetpoint;
	math::Vector3F m_VelocitySetpointPrevious;
	math::Vector3F m_VelocityErrD;		     /**< derivative of current velocity */
	math::Vector3F m_CurrentPositionSetpoint;  /**< current setpoint of the triplets */
	math::Vector3F m_PreviousPositionSetpoint;
	math::Matrix3F3 m_RSetpoint;
	math::Matrix3F3 m_Rotation; /**< rotation matrix from attitude quaternions */
	math::Vector3F m_ThrustInt;
	float m_DerivativeZ; /**< velocity in z that agrees with position rate */
	float m_Yaw;				  /**< yaw angle (euler) */
	float m_ManYawOffset; /**< current yaw offset in manual mode */

	/* State variables */
	boolean m_ModeAuto;
	boolean m_PositionHoldEngaged;
	boolean m_AltitudeHoldEngaged;
	boolean m_RunPosControl;
	boolean m_RunAltControl;
	boolean m_ResetPositionSetpoint;
	boolean m_ResetAltitudeSetpoint;
	boolean m_DoResetAltPos;
	boolean m_WasArmed;
	boolean m_WasLanded;
	boolean m_ResetIntZ;
	boolean m_ResetIntXY;
	boolean m_ResetIntZManual;
	boolean m_ResetYawSetpoint;
	boolean m_HoldOffboardXY;
	boolean m_HoldOffboardZ;
	boolean m_InTakeoff;	      /**< flag for smooth velocity setpoint takeoff ramp */
	boolean m_TripletLatLonFinite;

	/* Reference point */
	uint64 m_RefTimestamp;
	struct map_projection_reference_s m_RefPos;
	float m_RefAlt;
	boolean m_RefAltIsGlobal; /** true when the reference altitude is defined in a global reference frame */
	float m_YawTakeoff;	      /**< home yaw angle present when vehicle was taking off (euler) */

	/* Velocity controller PIDs */
	math::Vector3F m_PosP;
	math::Vector3F m_VelP;
	math::Vector3F m_VelI;
	math::Vector3F m_VelD;
	Derivative m_VelXDeriv;
	Derivative m_VelYDeriv;
	Derivative m_VelZDeriv;

	/* Limit variables */
	float m_AccelerationStateLimitXY; /**< acceleration limit applied in manual mode */
	float m_AccelerationStateLimitZ; /**< acceleration limit applied in manual mode in z */
	float m_ManualJerkLimitXY; /**< jerk limit in manual mode dependent on stick input */
	float m_ManualJerkLimitZ; /**< jerk limit in manual mode in z */
	float m_VelMaxXy;           /**< equal to vel_max except in auto mode when close to target */
	float m_TakeoffVelLimit;    /**< velocity limit value which gets ramped up */

	/* Stick input variables */
	math::LowPassFilter2p m_FilterManualPitch;
	math::LowPassFilter2p m_FilterManualRoll;
	math::Vector2F m_StickInputXyPrev; /**< for manual controlled mode to detect direction change */
	ManualStickInput m_UserIntentionXY; /**< defines what the user intends to do derived from the stick input */
	ManualStickInput m_UserIntentionZ; /**< defines what the user intends to do derived from the stick input in z direciton */
	systemlib::Hysteresis m_ManualDirectionChangeHysteresis;

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
    /** \brief Initialize the Multicopter m_Position Control (MPC) application
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
    /** \brief Sends MPC Diagnostic message
     **
     **  \par Description
     **       This function sends the diagnostic message
     **
     **  \par Assumptions, External Events, and Notes:
     **       None
     **
     *************************************************************************/
    void ReportDiagnostic(void);

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

    /************************************************************************/
    /** \brief Sends the VehicleLocalPosition message.
     **
     **  \par Description
     **       This function publishes the VehicleLocalPosition message containing
     **       <TODO>
     **
     **  \par Assumptions, External Events, and Notes:
     **       None
     **
     *************************************************************************/
    void SendVehicleLocalPositionSetpointMsg(void);

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

    /************************************************************************/
    /** \brief Process Control State Message
    **
    **  \par Description
    **       This function calculates yaw and Euler angles from the current
    **       control state message.
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void ProcessControlStateMsg(void);

    /************************************************************************/
    /** \brief Process Vehicle Local Position Message
    **
    **  \par Description
    **       This function verifies that the TODO
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void ProcessVehicleLocalPositionMsg(void);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void ProcessPositionSetpointTripletMsg(void);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void Execute(void);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void UpdateRef(void);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void UpdateVelocityDerivative(float dt);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void DoControl(float dt);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void GenerateAttitudeSetpoint(float dt);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void ControlManual(float dt);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void ControlNonManual(float dt);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    float ThrottleCurve(float ctl, float ctr);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void ResetAltSetpoint(void);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void ResetPosSetpoint(void);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void ControlPosition(float dt);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void ControlOffboard(float dt);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void ControlAuto(float dt);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void CalculateVelocitySetpoint(float dt);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void CalculateThrustSetpoint(float dt);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    float GetCruisingSpeedXY(void);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    boolean CrossSphereLine(const math::Vector3F &sphere_c, const float sphere_r,
			const math::Vector3F &line_a, const math::Vector3F &line_b, math::Vector3F &res);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void UpdateParamsFromTable(void);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void LimitAltitude(void);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    void SlowLandGradualVelocityLimit(void);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
    boolean InAutoTakeoff(void);

    /*
	 * Limit vel horizontally when close to target
	 */
    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
	void LimitVelXYGradually(void);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
	void ApplyVelocitySetpointSlewRate(float dt);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
	float GetVelClose(const math::Vector2F &UnitPrevToCurrent, const math::Vector2F &UnitCurrentToNext);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
	void SetManualAccelerationZ(float &max_acceleration, const float stick_z, const float dt);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
	void SetManualAccelerationXY(math::Vector2F &stick_xy, const float dt);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
	boolean ManualWantsTakeoff(void);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
	void UpdateXyPids(MPC_SetPidCmd_t* PidMsg);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
	void UpdateZPids(MPC_SetPidCmd_t* PidMsg);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/
	void UpdateHoldDz(MPC_SetDzCmd_t* DzMsg);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/	
	void UpdateStickExpo(MPC_SetStickExpoCmd_t* ExpoMsg);

    /************************************************************************/
    /** \brief
    **
    **  \par Description
    **       This function
    **
    **  \par Assumptions, External Events, and Notes:
    **       None
    **
    *************************************************************************/	
	void UpdateTakeoffTampTime(MPC_SetTkoRampCmd_t* TkoRampMsg);

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
};

#ifdef __cplusplus
}
#endif 

#endif /* MPC_APP_H */

/************************/
/*  End of File Comment */
/************************/
