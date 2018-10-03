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

#ifndef MPC_MSG_H
#define MPC_MSG_H

/************************************************************************
** Pragmas
*************************************************************************/

/************************************************************************
** Includes
*************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

#include "cfe.h"


/************************************************************************
** Local Defines
*************************************************************************/

/************************************************************************
** MPC Command Codes
*************************************************************************/

/** \mpccmd Noop 
**  
**  \par Description
**       Implements the Noop command that demonstrates the MPC task is alive
**
**  \mpccmdmnemonic \MPC_NOOP
**
**  \par Command Structure
**       #MPC_NoArgCmd_t
**
**  \par Command Verification
**       Successful execution of this command may be verified with
**       the following telemetry:
**       - \b \c \MPC_CMDACPTCNT - command counter will increment
**       - The #MPC_CMD_INF_EID informational event message will be 
**         generated when the command is received
** 
**  \par Error Conditions
**       This command may fail for the following reason(s):
**       - Command packet length not as expected
** 
**  \par Evidence of failure may be found in the following telemetry: 
**       - \b \c \MPC_CMDRJCTCNT - command error counter will increment
**       - Error specific event message #MPC_MSGID_ERR_EID
**
**  \par Criticality
**       None
**
**  \sa #MPC_RESET_CC
*/
#define MPC_NOOP_CC                 (0)

/** \mpccmd Reset Counters
**  
**  \par Description
**       Resets the MPC housekeeping counters
**
**  \mpccmdmnemonic \MPC_TLMRST
**
**  \par Command Structure
**       #MPC_NoArgCmd_t
**
**  \par Command Verification
**       Successful execution of this command may be verified with
**       the following telemetry:
**       - \b \c \MPC_CMDACTPCNT       - command counter will be cleared
**       - \b \c \MPC_CMDRJCTCNT       - command error counter will be cleared
**       - The #MPC_CMD_INF_EID debug event message will be 
**         generated when the command is executed
** 
**  \par Error Conditions
**       This command may fail for the following reason(s):
**       - Command packet length not as expected
** 
**  \par Evidence of failure may be found in the following telemetry: 
**       - \b \c \MPC_CMDRJCTCNT - command error counter will increment
**       - Error specific event message #MPC_MSGID_ERR_EID
**
**  \par Criticality
**       None
**
**  \sa #MPC_NOOP_CC
*/
#define MPC_RESET_CC                (1)

/** \mpccmd
**
**  \par Description
**
**
**  \par Command Structure
**       #MPC_NoArgCmd_t
**
**  \par Command Verification
**       Successful execution of this command may be verified with
**       the following telemetry:
**       - \b \c \MPC_CMDACPTCNT - command counter will increment
**       - The #TODO informational event message will be
**         generated when the command is received
**
**  \par Error Conditions
**       This command may fail for the following reason(s):
**       - Command packet length not as expected
**
**  \par Evidence of failure may be found in the following telemetry:
**       - \b \c \MPC_CMDRJCTCNT - command error counter will increment
**       - Error specific event message #TODO
**
**  \par Criticality
**       TODO
**
**  \sa #MPC_NOOP_CC
*/
#define MPC_SET_XY_PID_CC           (2)

/** \mpccmd
**
**  \par Description
**
**
**  \par Command Structure
**       #MPC_NoArgCmd_t
**
**  \par Command Verification
**       Successful execution of this command may be verified with
**       the following telemetry:
**       - \b \c \MPC_CMDACPTCNT - command counter will increment
**       - The #TODO informational event message will be
**         generated when the command is received
**
**  \par Error Conditions
**       This command may fail for the following reason(s):
**       - Command packet length not as expected
**
**  \par Evidence of failure may be found in the following telemetry:
**       - \b \c \MPC_CMDRJCTCNT - command error counter will increment
**       - Error specific event message #TODO
**
**  \par Criticality
**       TODO
**
**  \sa #MPC_NOOP_CC
*/
#define MPC_SET_Z_PID_CC            (3)

/** \mpccmd
**
**  \par Description
**
**
**  \par Command Structure
**       #MPC_NoArgCmd_t
**
**  \par Command Verification
**       Successful execution of this command may be verified with
**       the following telemetry:
**       - \b \c \MPC_CMDACPTCNT - command counter will increment
**       - The #TODO informational event message will be
**         generated when the command is received
**
**  \par Error Conditions
**       This command may fail for the following reason(s):
**       - Command packet length not as expected
**
**  \par Evidence of failure may be found in the following telemetry:
**       - \b \c \MPC_CMDRJCTCNT - command error counter will increment
**       - Error specific event message #TODO
**
**  \par Criticality
**       TODO
**
**  \sa #MPC_SEND_DIAG_CC
*/
#define MPC_SEND_DIAG_CC            (4)

/** \mpccmd
**
**  \par Description
**
**
**  \par Command Structure
**       #MPC_NoArgCmd_t
**
**  \par Command Verification
**       Successful execution of this command may be verified with
**       the following telemetry:
**       - \b \c \MPC_CMDACPTCNT - command counter will increment
**       - The #TODO informational event message will be
**         generated when the command is received
**
**  \par Error Conditions
**       This command may fail for the following reason(s):
**       - Command packet length not as expected
**
**  \par Evidence of failure may be found in the following telemetry:
**       - \b \c \MPC_CMDRJCTCNT - command error counter will increment
**       - Error specific event message #TODO
**
**  \par Criticality
**       TODO
**
**  \sa #MPC_SET_HOLD_DZ_CC
*/
#define MPC_SET_HOLD_DZ_CC            (5)

/** \mpccmd
**
**  \par Description
**
**
**  \par Command Structure
**       #MPC_NoArgCmd_t
**
**  \par Command Verification
**       Successful execution of this command may be verified with
**       the following telemetry:
**       - \b \c \MPC_CMDACPTCNT - command counter will increment
**       - The #TODO informational event message will be
**         generated when the command is received
**
**  \par Error Conditions
**       This command may fail for the following reason(s):
**       - Command packet length not as expected
**
**  \par Evidence of failure may be found in the following telemetry:
**       - \b \c \MPC_CMDRJCTCNT - command error counter will increment
**       - Error specific event message #TODO
**
**  \par Criticality
**       TODO
**
**  \sa #MPC_SET_STICK_EXPO
*/
#define MPC_SET_STICK_EXPO_CC            (6)

/** \mpccmd
**
**  \par Description
**
**
**  \par Command Structure
**       #MPC_NoArgCmd_t
**
**  \par Command Verification
**       Successful execution of this command may be verified with
**       the following telemetry:
**       - \b \c \MPC_CMDACPTCNT - command counter will increment
**       - The #TODO informational event message will be
**         generated when the command is received
**
**  \par Error Conditions
**       This command may fail for the following reason(s):
**       - Command packet length not as expected
**
**  \par Evidence of failure may be found in the following telemetry:
**       - \b \c \MPC_CMDRJCTCNT - command error counter will increment
**       - Error specific event message #TODO
**
**  \par Criticality
**       TODO
**
**  \sa #MPC_SET_STICK_EXPO
*/
#define MPC_SET_TKO_RAMP_CC              (7)

/************************************************************************
** Local Structure Declarations
*************************************************************************/

/** 
**  \brief No Arguments Command
**  For command details see #MPC_NOOP_CC, #MPC_RESET_CC
**  Also see #MPC_SEND_HK_MID
*/
typedef struct
{
    uint8  ucCmdHeader[CFE_SB_CMD_HDR_SIZE];
} MPC_NoArgCmd_t;

/** 
**  \brief MPC application housekeeping data
*/
typedef struct
{
	/** \brief cFE SB Tlm Msg Hdr */
    uint8              TlmHeader[CFE_SB_TLM_HDR_SIZE];

    /** \mpctlmmnemonic \MPC_CMDACPTCNT
        \brief Count of accepted commands */
    uint8              usCmdCnt;   

    /** \mpctlmmnemonic \MPC_CMDRJCTCNT
		\brief Count of failed commands */
    uint8              usCmdErrCnt;

    /** \brief  */
	float              AccelerationStateLimitXY;

	/** \brief  */
	float              AccelerationStateLimitZ;

	/** \brief  */
	float              ManualJerkLimitXY;

	/** \brief  */
	float              ManualJerkLimitZ;

	/** \brief  */
	float              TakeoffVelLimit;

	/** \brief  */
	float              VelMaxXy;

	/** \brief  */
	float              YawTakeoff;

	/** \brief  */
	float              Yaw;

	/** \brief Defines what the user intends to do derived from the stick input in xy axis */
	uint8   UserIntentionXY;

	/** \brief Defines what the user intends to do derived from the stick input in z axis */
	uint8   UserIntentionZ;

	/** \brief  */
	boolean            ModeAuto;

	/** \brief  */
	boolean            PositionHoldEngaged;

	/** \brief  */
	boolean            AltitudeHoldEngaged;

	/** \brief  */
	boolean            RunPosControl;

	/** \brief  */
	boolean            RunAltControl;

	/** \brief  */
	boolean            InTakeoff;

	/** \brief  */
	boolean            TripletLatLonFinite;

	/** \brief  */
	boolean            WasLanded;

	/** \brief  */
	boolean            WasArmed;

} MPC_HkTlm_t;

/**
**  \brief MPC diagnostic data
*/
typedef struct
{
    uint8           TlmHeader[CFE_SB_TLM_HDR_SIZE];

    /** \brief  */
	float            Z_P;

    /** \brief  */
	float            Z_VEL_P;

    /** \brief  */
	float            Z_VEL_I;

    /** \brief  */
	float            Z_VEL_D;

    /** \brief  */
	float            Z_VEL_MAX_UP;

    /** \brief  */
	float            Z_VEL_MAX_DN;

    /** \brief  */
	float            Z_FF;

    /** \brief  */
	float            XY_P;

    /** \brief  */
	float            XY_VEL_P;

    /** \brief  */
	float            XY_VEL_I;

    /** \brief  */
	float            XY_VEL_D;

    /** \brief  */
	float            XY_CRUISE;

    /** \brief  */
	float            MPC_VEL_MANUAL;

    /** \brief  */
	float            XY_VEL_MAX;

    /** \brief  */
	float            XY_FF;

    /** \brief  */
	float            TILTMAX_AIR;

    /** \brief  */
	float            ACC_HOR_MAX;

    /** \brief  */
	float            ACC_UP_MAX;

    /** \brief  */
	float            ACC_DOWN_MAX;

    /** \brief  */
	float            MPC_DEC_HOR_SLOW;

    /** \brief  */
	float            MPC_HOLD_DZ;
	
    /** \brief  */
	float            XY_MAN_EXPO;
	
    /** \brief  */
	float            Z_MAN_EXPO;
	
	/** \brief  */
	float            TKO_RAMP_T;

} MPC_DiagPacket_t;

/**
**  \brief MPC set PID cmd
*/
typedef struct
{
	/** \brief cFE SB Cmd Msg Hdr */
	uint8  ucCmdHeader[CFE_SB_CMD_HDR_SIZE];

	/** \brief PID Gain */
	float  PidGain;

	/** \brief PID P value */
	float  PidVelP;

	/** \brief PID I value */
	float  PidVelI;

	/** \brief PID D value */
	float  PidVelD;

} MPC_SetPidCmd_t;

/**
**  \brief MPC set Hold DZ
*/
typedef struct
{
	/** \brief cFE SB Cmd Msg Hdr */
	uint8  ucCmdHeader[CFE_SB_CMD_HDR_SIZE];

	/** \brief PID Gain */
	float  Deadzone;

} MPC_SetDzCmd_t;

/**
**  \brief MPC set stick exponential curve values
*/
typedef struct
{
	/** \brief cFE SB Cmd Msg Hdr */
	uint8  ucCmdHeader[CFE_SB_CMD_HDR_SIZE];

	/** \brief XY Expo */
	float  XY;
	
	/** \brief Z Expo */
	float  Z;

} MPC_SetStickExpoCmd_t;

/**
**  \brief MPC set take ramp time
*/
typedef struct
{
	/** \brief cFE SB Cmd Msg Hdr */
	uint8  ucCmdHeader[CFE_SB_CMD_HDR_SIZE];

	/** \brief XY Expo */
	float  TKO_RAMP_T;
	
} MPC_SetTkoRampCmd_t;

#ifdef __cplusplus
}
#endif

#endif /* MPC_MSG_H */

/************************/
/*  End of File Comment */
/************************/
