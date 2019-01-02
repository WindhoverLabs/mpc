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

#ifndef MPC_TBLDEFS_H
#define MPC_TBLDEFS_H

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
#include "mpc_platform_cfg.h"

/************************************************************************
** Local Defines
*************************************************************************/

/**
 * \brief Defines the table identification name used for the
 * configuration table registration.
 */
#define MPC_CONFIG_TABLENAME ("CONFIG_TBL")


/** \brief Definition for a single config table entry */
typedef struct
{
	/**
	 * Minimum thrust in auto thrust control
	 *
	 * It's recommended to set it > 0 to avoid free fall with zero thrust.
	 *
	 * @unit norm
	 * @min 0.05
	 * @max 1.0
	 * @decimal 2
	 * @increment 0.01
	 * @group Multicopter Position Control
	 */
    float THR_MIN;

    /**
     * Hover thrust
     *
     * Vertical thrust required to hover.
     * This value is mapped to center stick for manual throttle control.
     * With this value set to the thrust required to hover, transition
     * from manual to ALTCTL mode while hovering will occur with the
     * throttle stick near center, which is then interpreted as (near)
     * zero demand for vertical speed.
     *
     * @unit norm
     * @min 0.2
     * @max 0.8
     * @decimal 2
     * @increment 0.01
     * @group Multicopter Position Control
     */
    float THR_HOVER;

    /**
     * Maximum thrust in auto thrust control
     *
     * Limit max allowed thrust. Setting a value of one can put
     * the system into actuator saturation as no spread between
     * the motors is possible any more. A value of 0.8 - 0.9
     * is recommended.
     *
     * @unit norm
     * @min 0.0
     * @max 0.95
     * @decimal 2
     * @increment 0.01
     * @group Multicopter Position Control
     */
    float THR_MAX;

    /**
     * Minimum manual thrust
     *
     * Minimum vertical thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
     *
     * @unit norm
     * @min 0.0
     * @max 1.0
     * @decimal 2
     * @increment 0.01
     * @group Multicopter Position Control
     */
    float MANTHR_MIN;

    /**
     * Maximum manual thrust
     *
     * Limit max allowed thrust. Setting a value of one can put
     * the system into actuator saturation as no spread between
     * the motors is possible any more. A value of 0.8 - 0.9
     * is recommended.
     *
     * @unit norm
     * @min 0.0
     * @max 1.0
     * @decimal 2
     * @increment 0.01
     * @group Multicopter Position Control
     */
    float MANTHR_MAX;

    /**
     * Proportional gain for vertical position error
     *
     * @min 0.0
     * @max 1.5
     * @decimal 2
     * @group Multicopter Position Control
     */
    float Z_P;

    /**
     * Proportional gain for vertical velocity error
     *
     * @min 0.1
     * @max 0.4
     * @decimal 2
     * @group Multicopter Position Control
     */
    float Z_VEL_P;

    /**
     * Integral gain for vertical velocity error
     *
     * Non zero value allows hovering thrust estimation on stabilized or autonomous takeoff.
     *
     * @min 0.01
     * @max 0.1
     * @decimal 3
     * @group Multicopter Position Control
     */
    float Z_VEL_I;

    /**
     * Differential gain for vertical velocity error
     *
     * @min 0.0
     * @max 0.1
     * @decimal 3
     * @group Multicopter Position Control
     */
    float Z_VEL_D;

    /**
     * Maximum vertical ascent velocity
     *
     * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
     *
     * @unit m/s
     * @min 0.5
     * @max 8.0
     * @decimal 1
     * @group Multicopter Position Control
     */
    float Z_VEL_MAX_UP;

    /**
     * Maximum vertical descent velocity
     *
     * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
     *
     * @unit m/s
     * @min 0.5
     * @max 4.0
     * @group Multicopter Position Control
     */
    float Z_VEL_MAX_DN;

    /**
     * Vertical velocity feed forward
     *
     * Feed forward weight for altitude control in stabilized modes (ALTCTRL, POSCTRL). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
     *
     * @min 0.0
     * @max 1.0
     * @decimal 2
     * @group Multicopter Position Control
     */
    float Z_FF;

    /**
     * Proportional gain for horizontal position error
     *
     * @min 0.0
     * @max 2.0
     * @decimal 2
     * @group Multicopter Position Control
     */
    float XY_P;

    /**
     * Proportional gain for horizontal velocity error
     *
     * @min 0.06
     * @max 0.15
     * @decimal 2
     * @group Multicopter Position Control
     */
    float XY_VEL_P;

    /**
     * Integral gain for horizontal velocity error
     *
     * Non-zero value allows to resist wind.
     *
     * @min 0.0
     * @max 0.1
     * @decimal 3
     * @group Multicopter Position Control
     */
    float XY_VEL_I;

    /**
     * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
     *
     * @min 0.005
     * @max 0.1
     * @decimal 3
     * @group Multicopter Position Control
     */
    float XY_VEL_D;

    /**
     * Nominal horizontal velocity in mission
     *
     * Normal horizontal velocity in AUTO modes (includes
     * also RTL / hold / etc.) and endpoint for
     * position stabilized mode (POSCTRL).
     *
     * @unit m/s
     * @min 3.0
     * @max 20.0
     * @increment 1
     * @decimal 2
     * @group Multicopter Position Control
     */
    float XY_CRUISE;

    /**
     * Distance Threshold Horizontal Auto
     *
     * The distance defines at which point the vehicle
     * has to slow down to reach target if no direct
     * passing to the next target is desired
     *
     * @unit m
     * @min 1.0
     * @max 50.0
     * @increment 1
     * @decimal 2
     * @group Multicopter Position Control
     */
    float TARGET_THRE;

    /**
     * Maximum horizontal velocity
     *
     * Maximum horizontal velocity in AUTO mode. If higher speeds
     * are commanded in a mission they will be capped to this velocity.
     *
     * @unit m/s
     * @min 0.0
     * @max 20.0
     * @increment 1
     * @decimal 2
     * @group Multicopter Position Control
     */
    float XY_VEL_MAX;

    /**
     * Horizontal velocity feed forward
     *
     * Feed forward weight for position control in position control mode (POSCTRL). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
     *
     * @min 0.0
     * @max 1.0
     * @decimal 2
     * @group Multicopter Position Control
     */
    float XY_FF;

    /**
     * Maximum tilt angle in air
     *
     * Limits maximum tilt in AUTO and POSCTRL modes during flight.
     *
     * @unit radians
     * @min 0.0
     * @max pi/2 (1.5708)
     * @decimal 1
     * @group Multicopter Position Control
     */
    float TILTMAX_AIR;

    /**
     * Maximum tilt during landing
     *
     * Limits maximum tilt angle on landing.
     *
     * @unit radians
     * @min 0.0
     * @max pi/2 (1.5708)
     * @decimal 1
     * @group Multicopter Position Control
     */
    float TILTMAX_LND;

    /**
     * Landing descend rate
     *
     * @unit m/s
     * @min 0.2
     * @decimal 1
     * @group Multicopter Position Control
     */
    float LAND_SPEED;

    /**
     * Takeoff climb rate
     *
     * @unit m/s
     * @min 1
     * @max 5
     * @decimal 2
     * @group Multicopter Position Control
     */
    float TKO_SPEED;

    /**
     * Maximal tilt angle in manual or altitude mode
     *
     * @unit deg
     * @min 0.0
     * @max 90.0
     * @decimal 1
     * @group Multicopter Position Control
     */
    float MAN_TILT_MAX;

    /**
     * Max manual yaw rate
     *
     * @unit deg/s
     * @min 0.0
     * @max 400
     * @decimal 1
     * @group Multicopter Position Control
     */
    float MAN_Y_MAX;

    /**
     * Deadzone of sticks where position hold is enabled
     *
     * @min 0.0
     * @max 1.0
     * @decimal 2
     * @group Multicopter Position Control
     */
    float HOLD_DZ;

    /**
     * Maximum horizontal velocity for which position hold is enabled (use 0 to disable check)
     *
     * @unit m/s
     * @min 0.0
     * @max 3.0
     * @decimal 2
     * @group Multicopter Position Control
     */
    float HOLD_MAX_XY;

    /**
     * Maximum vertical velocity for which position hold is enabled (use 0 to disable check)
     *
     * @unit m/s
     * @min 0.0
     * @max 3.0
     * @decimal 2
     * @group Multicopter Position Control
     */
    float HOLD_MAX_Z;

    /**
     * Low pass filter cut freq. for numerical velocity derivative
     *
     * @unit Hz
     * @min 0.0
     * @max 10
     * @decimal 2
     * @group Multicopter Position Control
     */
    float VELD_LP;

    /**
     * Maximum horizonal acceleration in velocity controlled modes
     *
     * @unit m/s/s
     * @min 2.0
     * @max 15.0
     * @increment 1
     * @decimal 2
     * @group Multicopter Position Control
     */
    float ACC_HOR_MAX;

    /**
     * Maximum vertical acceleration in velocity controlled modes upward
     *
     * @unit m/s/s
     * @min 2.0
     * @max 15.0
     * @increment 1
     * @decimal 2
     * @group Multicopter Position Control
     */
    float ACC_UP_MAX;

    /**
     * Maximum vertical acceleration in velocity controlled modes down
     *
     * @unit m/s/s
     * @min 2.0
     * @max 15.0
     * @increment 1
     * @decimal 2
     * @group Multicopter Position Control
     */
    float ACC_DOWN_MAX;

    /**
     * Altitude control mode, note mode 1 only tested with LPE
     *
     * @min 0
     * @max 1
     * @value 0 Altitude following
     * @value 1 Terrain following
     * @group Multicopter Position Control
     */
    uint32 ALT_MODE;

    /**
     * Manual control stick exponential curve sensitivity attenuation with small velocity setpoints
     *
     * The higher the value the less sensitivity the stick has around zero
     * while still reaching the maximum value with full stick deflection.
     *
     * 0 Purely linear input curve (default)
     * 1 Purely cubic input curve
     *
     * @min 0
     * @max 1
     * @decimal 2
     * @group Multicopter Position Control
     */
    float XY_MAN_EXPO;

    /**
     * Manual control stick vertical exponential curve
     *
     * The higher the value the less sensitivity the stick has around zero
     * while still reaching the maximum value with full stick deflection.
     *
     * 0 Purely linear input curve (default)
     * 1 Purely cubic input curve
     *
     * @min 0
     * @max 1
     * @decimal 2
     * @group Multicopter Position Control
     */
    float Z_MAN_EXPO;

    /**
     * Altitude for 1. step of slow landing (descend)
     *
     * Below this altitude descending velocity gets limited
     * to a value between "MPC_Z_VEL_MAX" and "MPC_LAND_SPEED"
     * to enable a smooth descent experience
     * Value needs to be higher than "MPC_LAND_ALT2"
     *
     * @unit m
     * @min 0
     * @max 122
     * @decimal 1
     * @group Multicopter Position Control
     */
    uint32 LAND_ALT1;

    /**
     * Altitude for 2. step of slow landing (landing)
     *
     * Below this altitude descending velocity gets limited to "MPC_LAND_SPEED"
     * Value needs to be lower than "MPC_LAND_ALT1"
     *
     * @unit m
     * @min 0
     * @max 122
     * @decimal 1
     * @group Multicopter Position Control
     */
    uint32 LAND_ALT2;

    /**
     * Position control smooth takeoff ramp time constant
     *
     * Increasing this value will make automatic and manual takeoff slower.
     * If it's too slow the drone might scratch the ground and tip over.
     *
     * @min 0.1
     * @max 1
     * @group Multicopter Position Control
     */
    float TKO_RAMP_T;

    float MC_YAWRATE_MAX;

    float MC_YAW_P;

    int32 VT_OPT_RECOV_EN;

    float MIS_LTRMIN_ALT;

    /**
     * Cruise speed when angle prev-current/current-next setpoint
     * is 90 degrees. It should be lower than MPC_XY_CRUISE.
     *
     * Applies only in AUTO modes (includes
     * also RTL / hold / etc.)
     *
     * @unit m/s
     * @min 1.0
     * @increment 1
     * @decimal 2
     * @group Multicopter Position Control
     */
    float MPC_CRUISE_90;

    /**
     * Maximum jerk in manual controlled mode for BRAKING to zero.
     * If this value is below MPC_JERK_MIN, the acceleration limit in xy and z
     * is MPC_ACC_HOR_MAX and MPC_ACC_UP_MAX respectively instantaneously when the
     * user demands brake (=zero stick input).
     * Otherwise the acceleration limit increases from current acceleration limit
     * towards MPC_ACC_HOR_MAX/MPC_ACC_UP_MAX with jerk limit
     *
     * @unit m/s/s/s
     * @min 0.0
     * @max 15.0
     * @increment 1
     * @decimal 2
     * @group Multicopter Position Control
     */
    float MPC_JERK_MAX;

    /**
     * Minimum jerk in manual controlled mode for BRAKING to zero
     *
     * @unit m/s/s/s
     * @min 0.5
     * @max 10.0
     * @increment 1
     * @decimal 2
     * @group Multicopter Position Control
     */
    float MPC_JERK_MIN;

    /**
     * Slow horizontal manual deceleration for manual mode
     *
     * @unit m/s/s
     * @min 0.5
     * @max 10.0
     * @increment 1
     * @decimal 2
     * @group Multicopter Position Control
     */
    float MPC_DEC_HOR_SLOW;

    /**
     * Maximum horizontal velocity setpoint for manual controlled mode
     * If velocity setpoint larger than MPC_XY_VEL_MAX is set, then
     * the setpoint will be capped to MPC_XY_VEL_MAX
     *
     * @unit m/s
     * @min 3.0
     * @max 20.0
     * @increment 1
     * @decimal 2
     * @group Multicopter Position Control
     */
    float MPC_VEL_MANUAL;

    /**
     * Acceptance Radius
     *
     * Default acceptance radius, overridden by acceptance radius of waypoint if set.
     * For fixed wing the L1 turning distance is used for horizontal acceptance.
     *
     * @unit m
     * @min 0.05
     * @max 200.0
     * @decimal 1
     * @increment 0.5
     * @group Mission
     */
    float NAV_ACC_RAD;

    /** NAV_MIS_YAW_ERR
     *
     *  \brief Max yaw error in degrees needed for waypoint heading acceptance.
     *
     *  \par Limits:
     *       Min > Max (incr.) 0 > 90 , default 12.0.
     */
    float NAV_MIS_YAW_ERR;


} MPC_ConfigTbl_t;


#ifdef __cplusplus
}
#endif

#endif /* MPC_TBLDEFS_H */

/************************/
/*  End of File Comment */
/************************/
