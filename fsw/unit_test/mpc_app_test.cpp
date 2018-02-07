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

#include "cfe.h"
#include "mpc_app.h"
#include "mpc_test_utils.h"
#include "mpc_app_test.h"
#include "uttest.h"
#include "ut_osapi_stubs.h"
#include "ut_cfe_sb_stubs.h"
#include "ut_cfe_sb_hooks.h"
#include "ut_cfe_es_stubs.h"
#include "ut_cfe_es_hooks.h"
#include "ut_cfe_evs_stubs.h"
#include "ut_cfe_evs_hooks.h"
#include "ut_cfe_time_stubs.h"
#include "ut_cfe_psp_memutils_stubs.h"
#include "ut_cfe_tbl_stubs.h"
#include "ut_cfe_fs_stubs.h"
#include "ut_cfe_time_stubs.h"
#include "math/Quaternion.hpp"
#include "math/Matrix3F3.hpp"
#include <float.h>
#include "stubs/ut_mpc_custom_stubs.h"

int32 hookCalledCount = 0;


/**************************************************************************
 * Tests for MPC_InitEvent()
 **************************************************************************/

/**
 * Test MPC_InitEvent() with failed CFE_EVS_Register
 */
void Test_MPC_InitEvent_Fail_Register(void)
{
    MPC oMPC;

    /* Set a fail result for EVS */
    int32 result = (CFE_SEVERITY_BITMASK & CFE_SEVERITY_ERROR)
                   | CFE_EVENTS_SERVICE | CFE_EVS_NOT_IMPLEMENTED;
    int32 expected = CFE_EVS_APP_NOT_REGISTERED;

    Ut_CFE_EVS_SetReturnCode(UT_CFE_EVS_REGISTER_INDEX, expected, 1);

    /* Execute the function being tested */
    result = oMPC.InitEvent();

    /* Verify results */
    UtAssert_True (result == expected, "InitEvent, failed EVS Register");
}


/**************************************************************************
 * Tests for MPC_InitPipe()
 **************************************************************************/
/**
 * Test MPC_InitPipe(), fail SCH CFE_SB_CreatePipe
 */
void Test_MPC_InitPipe_Fail_CreateSCHPipe(void)
{
    MPC oMPC;

    /* Set a fail result for SB */
    int32 result = (CFE_SEVERITY_BITMASK & CFE_SEVERITY_ERROR)
                   | CFE_SOFTWARE_BUS_SERVICE | CFE_SB_NOT_IMPLEMENTED;
    int32 expected = CFE_SB_BAD_ARGUMENT;

    Ut_CFE_SB_SetReturnCode(UT_CFE_SB_CREATEPIPE_INDEX, expected, 1);

    /* Execute the function being tested */
    result = oMPC.InitPipe();

    /* Verify results */
    UtAssert_True (result == expected, "InitPipe, fail SB create SCH pipe");
}


/**
 * Test MPC_InitPipe(), fail CFE_SB_SubscribeEx for wakeup
 */
void Test_MPC_InitPipe_Fail_SubscribeWakeup(void)
{
    MPC oMPC;

    /* Set a fail result for SB */
    int32 result = (CFE_SEVERITY_BITMASK & CFE_SEVERITY_ERROR)
                   | CFE_SOFTWARE_BUS_SERVICE | CFE_SB_NOT_IMPLEMENTED;
    int32 expected = CFE_SB_BAD_ARGUMENT;

    Ut_CFE_SB_SetReturnCode(UT_CFE_SB_SUBSCRIBEEX_INDEX, expected, 1);

    /* Execute the function being tested */
    result = oMPC.InitPipe();

    /* Verify results */
    UtAssert_True (result == expected, "InitPipe, fail CFE_SB_SubscribeEx for wakeup");
}


/**
 * Test MPC_InitPipe(), fail CFE_SB_SubscribeEx for sendhk
 */
void Test_MPC_InitPipe_Fail_SubscribeSendHK(void)
{
    MPC oMPC;

    /* Set a fail result for SB */
    int32 result = (CFE_SEVERITY_BITMASK & CFE_SEVERITY_ERROR)
                   | CFE_SOFTWARE_BUS_SERVICE | CFE_SB_NOT_IMPLEMENTED;
    int32 expected = CFE_SB_BAD_ARGUMENT;

    Ut_CFE_SB_SetReturnCode(UT_CFE_SB_SUBSCRIBEEX_INDEX, expected, 2);

    /* Execute the function being tested */
    result = oMPC.InitPipe();

    /* Verify results */
    UtAssert_True (result == expected, "InitPipe, fail CFE_SB_SubscribeEx for sendhk");
}


/**
 * Test MPC_InitPipe(), fail CMD CFE_SB_CreatePipe
 */
void Test_MPC_InitPipe_Fail_CreateCMDPipe(void)
{
    MPC oMPC;

    /* Set a fail result for SB */
    int32 result = (CFE_SEVERITY_BITMASK & CFE_SEVERITY_ERROR)
                   | CFE_SOFTWARE_BUS_SERVICE | CFE_SB_NOT_IMPLEMENTED;
    int32 expected = CFE_SB_BAD_ARGUMENT;

    Ut_CFE_SB_SetReturnCode(UT_CFE_SB_CREATEPIPE_INDEX, expected, 2);

    /* Execute the function being tested */
    result = oMPC.InitPipe();

    /* Verify results */
    UtAssert_True (result == expected, "InitPipe, fail SB create CMD pipe");
}


/**
 * Test MPC_InitPipe(), fail CFE_SB_Subscribe for CMD msg
 */
void Test_MPC_InitPipe_Fail_SubscribeCMD(void)
{
    MPC oMPC;

    /* Set a fail result for SB */
    int32 result = (CFE_SEVERITY_BITMASK & CFE_SEVERITY_ERROR)
                   | CFE_SOFTWARE_BUS_SERVICE | CFE_SB_NOT_IMPLEMENTED;
    int32 expected = CFE_SB_BAD_ARGUMENT;

    Ut_CFE_SB_SetReturnCode(UT_CFE_SB_SUBSCRIBE_INDEX, expected, 1);

    /* Execute the function being tested */
    result = oMPC.InitPipe();

    /* Verify results */
    UtAssert_True (result == expected, "InitPipe, fail CFE_SB_Subscribe for CMD");
}


/**
 * Test MPC_InitPipe(), fail DATA CFE_SB_CreatePipe
 */
void Test_MPC_InitPipe_Fail_CreateDATAPipe(void)
{
    MPC oMPC;

    /* Set a fail result for SB */
    int32 result = (CFE_SEVERITY_BITMASK & CFE_SEVERITY_ERROR)
                   | CFE_SOFTWARE_BUS_SERVICE | CFE_SB_NOT_IMPLEMENTED;
    int32 expected = CFE_SB_BAD_ARGUMENT;

    Ut_CFE_SB_SetReturnCode(UT_CFE_SB_CREATEPIPE_INDEX, expected, 2);

    /* Execute the function being tested */
    result = oMPC.InitPipe();

    /* Verify results */
    UtAssert_True (result == expected, "InitPipe, fail SB create DATA pipe");
}


/**************************************************************************
 * Tests for MPC_InitData()
 **************************************************************************/
/**
 * Test MPC_InitData()
 */
void Test_MPC_InitData(void)
{
    MPC oMPC;

    /* Set a fail result */
    int32 expected = CFE_SUCCESS;

    /* Execute the function being tested */
    oMPC.InitData();

    /* Verify results */
    //UtAssert_True (result == expected, "InitData");
}


/**************************************************************************
 * Tests for MPC_InitApp()
 **************************************************************************/
/**
 * Test MPC_InitApp(), fail init event
 */
void Test_MPC_InitApp_Fail_InitEvent(void)
{
    MPC oMPC;

    int32 result = CFE_SUCCESS;
    int32 expected = CFE_EVS_APP_NOT_REGISTERED;

    Ut_CFE_EVS_SetReturnCode(UT_CFE_EVS_REGISTER_INDEX, expected, 1);

    /* Execute the function being tested */
    result = oMPC.InitApp();

    /* Verify results */
    UtAssert_True (result == expected, "InitApp, fail init event");
}


/**
 * Test MPC_InitApp(), fail init pipe
 */
void Test_MPC_InitApp_Fail_InitPipe(void)
{
    MPC oMPC;

    int32 result = CFE_SUCCESS;
    int32 expected = CFE_SB_BAD_ARGUMENT;

    Ut_CFE_SB_SetReturnCode(UT_CFE_SB_CREATEPIPE_INDEX, expected, 1);

    /* Execute the function being tested */
    result = oMPC.InitApp();

    /* Verify results */
    UtAssert_True (result == expected, "InitApp, fail init pipe");
}


/**
 * Test MPC_InitApp(), fail init data.
 * NOTE: no current way to fail MPC_InitData() in default
 */
void Test_MPC_InitApp_Fail_InitData(void)
{
    MPC oMPC;

    int32 result = CFE_SUCCESS;
    int32 expected = CFE_SUCCESS;

    /* Execute the function being tested */
    result = oMPC.InitApp();

    /* Verify results */
    UtAssert_True (result == expected, "InitApp, fail init data");
}


/**
 * Test MPC_InitApp(), fail init config table
 */
void Test_MPC_InitApp_Fail_InitConfigTbl(void)
{
    MPC oMPC;

    int32 result = CFE_SUCCESS;
    int32 expected = CFE_TBL_ERR_INVALID_NAME;

    Ut_CFE_TBL_SetReturnCode(UT_CFE_TBL_REGISTER_INDEX, expected, 1);

    /* Execute the function being tested */
    result = oMPC.InitApp();

    /* Verify results */
    UtAssert_True (result == expected, "InitApp, fail init config table");
}


/**
 * Test MPC_InitApp(), Nominal
 */
void Test_MPC_InitApp_Nominal(void)
{
    MPC oMPC;

    /* Set a fail result for SB */
    int32 result = (CFE_SEVERITY_BITMASK & CFE_SEVERITY_ERROR)
                   | CFE_EXECUTIVE_SERVICE | CFE_ES_ERR_APP_REGISTER;
    int32 expected = CFE_SUCCESS;

    /* Execute the function being tested */
    result = oMPC.InitApp();

    /* Verify results */
    UtAssert_True (result == expected, "InitApp, nominal");
}

/**************************************************************************
 * Tests for MPC_AppMain()
 **************************************************************************/
/**
 * Test MPC_AppMain(), Fail RegisterApp
 */
void Test_MPC_AppMain_Fail_RegisterApp(void)
{
    MPC oMPC;

    /* fail the register app */
    Ut_CFE_ES_SetReturnCode(UT_CFE_ES_REGISTERAPP_INDEX, CFE_ES_ERR_APP_REGISTER, 1);

    /* Execute the function being tested */
    oMPC.AppMain();
}


/**
 * Test MPC_AppMain(), Fail InitApp
 */
void Test_MPC_AppMain_Fail_InitApp(void)
{
    MPC oMPC;

    /* fail the register app */
    Ut_CFE_EVS_SetReturnCode(UT_CFE_EVS_REGISTER_INDEX, CFE_EVS_APP_NOT_REGISTERED, 1);

    /* Execute the function being tested */
    oMPC.AppMain();
}


/**
 * Test MPC_AppMain(), Fail AcquireConfigPtrs
 */
void Test_MPC_AppMain_Fail_AcquireConfigPtrs(void)
{
    MPC oMPC;

    /* fail the register app */
    Ut_CFE_TBL_SetReturnCode(UT_CFE_TBL_GETADDRESS_INDEX, CFE_TBL_ERR_INVALID_HANDLE, 2);

    /* Execute the function being tested */
    oMPC.AppMain();
}


/**
 * Test MPC_AppMain(), Invalid Schedule Message
 */
void Test_MPC_AppMain_InvalidSchMessage(void)
{
    MPC oMPC;

    /* The following will emulate behavior of receiving a SCH message to send HK */
    Ut_CFE_SB_SetReturnCode(UT_CFE_SB_RCVMSG_INDEX, CFE_SUCCESS, 1);
    Ut_CFE_SB_SetReturnCode(UT_CFE_SB_GETMSGID_INDEX, 0, 1);

    Ut_CFE_ES_SetReturnCode(UT_CFE_ES_RUNLOOP_INDEX, FALSE, 2);

    /* Execute the function being tested */
    oMPC.AppMain();

}


/**
 * Hook to support: MPC_AppMain(), Nominal - SendHK
 */
int32 Test_MPC_AppMain_Nominal_SendHK_SendMsgHook(CFE_SB_Msg_t *MsgPtr)
{
    /* TODO:  Test the contents of your HK message here. */

    hookCalledCount++;

    return CFE_SUCCESS;
}

/**
 * Test MPC_AppMain(), Nominal - SendHK
 */
void Test_MPC_AppMain_Nominal_SendHK(void)
{
    MPC oMPC;

    /* The following will emulate behavior of receiving a SCH message to WAKEUP */
    Ut_CFE_SB_SetReturnCode(UT_CFE_SB_RCVMSG_INDEX, CFE_SUCCESS, 1);
    Ut_CFE_SB_SetReturnCode(UT_CFE_SB_GETMSGID_INDEX, MPC_SEND_HK_MID, 1);

    Ut_CFE_ES_SetReturnCode(UT_CFE_ES_RUNLOOP_INDEX, FALSE, 2);

    /* Used to verify HK was transmitted correctly. */
    hookCalledCount = 0;
    Ut_CFE_ES_SetFunctionHook(UT_CFE_SB_SENDMSG_INDEX, (void*)&Test_MPC_AppMain_Nominal_SendHK_SendMsgHook);

    /* Execute the function being tested */
    oMPC.AppMain();

    /* Verify results */
    UtAssert_True (hookCalledCount == 1, "AppMain_Nominal_SendHK");

}


/**
 * Test MPC_AppMain(), Nominal - Wakeup
 */
void Test_MPC_AppMain_Nominal_Wakeup(void)
{
    MPC oMPC;

    /* The following will emulate behavior of receiving a SCH message to WAKEUP */
    Ut_CFE_SB_SetReturnCode(UT_CFE_SB_RCVMSG_INDEX, CFE_SUCCESS, 1);
    Ut_CFE_SB_SetReturnCode(UT_CFE_SB_GETMSGID_INDEX, MPC_WAKEUP_MID, 1);

    Ut_CFE_ES_SetReturnCode(UT_CFE_ES_RUNLOOP_INDEX, FALSE, 2);

    /* Execute the function being tested */
    //oMPC.AppMain();

}


/**
 * Test MPC_AppMain(), Nominal - ProcessControlStateMsg
 */
void Test_MPC_AppMain_Nominal_ProcessControlStateMsg(void)
{
    MPC oMPC;
    PX4_ControlStateMsg_t controlState = {};
    int32 schPipe;

    schPipe = Ut_CFE_SB_CreatePipe("MPC_SCH_PIPE");

    CFE_SB_InitMsg (&controlState, PX4_CONTROL_STATE_MID, sizeof(controlState), TRUE);

    controlState.Timestamp = 86440828023;
    controlState.AccX = -0.236284539;
    controlState.AccY = 0.101594232;
    controlState.AccZ = -9.76688671;
    controlState.VelX = 0.000539890432;
    controlState.VelY = 0.0182482861;
    controlState.VelZ = 0.00519393012;
    controlState.PosX = -0.00807033759;
    controlState.PosY = -0.000570089556;
    controlState.PosZ = 0.0179239418;
    controlState.Airspeed = 0;
    controlState.VelVariance[0] = 0.0f;
    controlState.VelVariance[1] = 0.0f;
    controlState.VelVariance[2] = 0.0f;
    controlState.PosVariance[0] = 0.0f;
    controlState.PosVariance[1] = 0.0f;
    controlState.PosVariance[2] = 0.0f;
    controlState.Q[0] = 0.712293863;
    controlState.Q[1] = 0.00346049992;
    controlState.Q[2] = -0.0159233119;
    controlState.Q[3] = 0.701692224;
    controlState.DeltaQReset[0] = 0.999934673;
    controlState.DeltaQReset[1] = 0.000154913403;
    controlState.DeltaQReset[2] = -0.00022063707;
    controlState.DeltaQReset[3] = 0.0114225745;
    controlState.RollRate = 0.00590410549;
    controlState.PitchRate = 0.00479567284;
    controlState.YawRate = 0.000454715308;
    controlState.HorzAccMag = 0.158787385;
    controlState.RollRateBias = -0.00138732162;
    controlState.PitchRateBias = -0.00102514029;
    controlState.YawRateBias = -0.00102514029;
    controlState.AirspeedValid = false;
    controlState.QuatResetCounter = 2;

    Ut_CFE_SB_AddMsgToPipe(&controlState, schPipe);

    Ut_CFE_ES_SetReturnCode(UT_CFE_ES_RUNLOOP_INDEX, FALSE, 2);

    /* Execute the function being tested */
    oMPC.AppMain();

    /* Verify results */
    UtAssert_True(oMPC.HeadingResetCounter == 0,"HeadingResetCounter == 0");
    UtAssert_DoubleCmpAbs(oMPC.Rotation[0][0], 0.0147489905, FLT_EPSILON, "oMPC.Rotation[0][0] == 0.0147489905");
    UtAssert_DoubleCmpAbs(oMPC.Rotation[0][1], -0.999732316, FLT_EPSILON, "oMPC.Rotation[0][1] == -0.999732316");
    UtAssert_DoubleCmpAbs(oMPC.Rotation[0][2], -0.0178277437, FLT_EPSILON, "oMPC.Rotation[0][2] == -0.0178277437");
    UtAssert_DoubleCmpAbs(oMPC.Rotation[1][0], 0.999511898, FLT_EPSILON, "oMPC.Rotation[1][0] == 0.999511898");
    UtAssert_DoubleCmpAbs(oMPC.Rotation[1][1], 0.0152321458, FLT_EPSILON, "oMPC.Rotation[1][1] == 0.0152321458");
    UtAssert_DoubleCmpAbs(oMPC.Rotation[1][2], -0.0272763148, FLT_EPSILON, "oMPC.Rotation[1][2] == -0.0272763148");
    UtAssert_DoubleCmpAbs(oMPC.Rotation[2][0], 0.0275405664, FLT_EPSILON, "oMPC.Rotation[2][0] == 0.0275405664");
    UtAssert_DoubleCmpAbs(oMPC.Rotation[2][1], -0.0174167417, FLT_EPSILON, "oMPC.Rotation[2][1] == -0.0174167417");
    UtAssert_DoubleCmpAbs(oMPC.Rotation[2][2], 0.999468982, FLT_EPSILON, "oMPC.Rotation[2][1] == 0.999468982");
    UtAssert_DoubleCmpAbs(oMPC.Yaw, 1.55604124, FLT_EPSILON, "oMPC.Yaw == 1.55604124");
}


/**
 * Test MPC_AppMain(), Nominal - ProcessControlStateMsg
 */
void Test_MPC_AppMain_Nominal_ProcessVehicleLocalPositionMsg(void)
{
    MPC oMPC;
    PX4_VehicleLocalPositionMsg_t vehicleLocalPosition = {};
    PX4_ControlStateMsg_t controlState = {};
    int32 schPipe;

    schPipe = Ut_CFE_SB_CreatePipe("MPC_SCH_PIPE");

    CFE_SB_InitMsg(&vehicleLocalPosition, PX4_VEHICLE_LOCAL_POSITION_MID, sizeof(vehicleLocalPosition), TRUE);
    CFE_SB_InitMsg(&controlState, PX4_CONTROL_STATE_MID, sizeof(controlState), TRUE);


    vehicleLocalPosition.Timestamp = 88516625760;
    vehicleLocalPosition.RefTimestamp = 86416035453;
    vehicleLocalPosition.RefLat = 47.397742000000001;
    vehicleLocalPosition.RefLon = 8.5455939000000001;
    vehicleLocalPosition.SurfaceBottomTimestamp = 88516625760;
    vehicleLocalPosition.X = -0.0382288173;
    vehicleLocalPosition.Y = 0.0131685678;
    vehicleLocalPosition.Z = 0.405019075;
    vehicleLocalPosition.Delta_XY[0] = -0.00702110073;
    vehicleLocalPosition.Delta_XY[1] = -0.00230869721;
    vehicleLocalPosition.Delta_Z = 0;
    vehicleLocalPosition.VX = -0.035448242;
    vehicleLocalPosition.VY = 0.00551212905;
    vehicleLocalPosition.VZ = 0.20808062;
    vehicleLocalPosition.Delta_VXY[0] = -0.0166048259;
    vehicleLocalPosition.Delta_VXY[1] = -0.00429295097;
    vehicleLocalPosition.Delta_VZ = 0.0100669134;
    vehicleLocalPosition.AX = 0;
    vehicleLocalPosition.AY = 0;
    vehicleLocalPosition.AZ = 0;
    vehicleLocalPosition.Yaw = 1.51540768;
    vehicleLocalPosition.RefAlt = 488.201019;
    vehicleLocalPosition.DistBottom = 0.995466709;
    vehicleLocalPosition.DistBottomRate = -0.20808062;
    vehicleLocalPosition.EpH = 0.412961185;
    vehicleLocalPosition.EpV = 0.23175697;
    vehicleLocalPosition.EvH = 0.188474447;
    vehicleLocalPosition.EvV = 0.0783097148;
    vehicleLocalPosition.EstimatorType = 0;
    vehicleLocalPosition.XY_Valid = true;
    vehicleLocalPosition.Z_Valid = true;
    vehicleLocalPosition.V_XY_Valid = true;
    vehicleLocalPosition.V_Z_Valid = true;
    vehicleLocalPosition.XY_ResetCounter = 1;
    vehicleLocalPosition.Z_ResetCounter = 0;
    vehicleLocalPosition.VXY_ResetCounter = 1;
    vehicleLocalPosition.VZ_ResetCounter = 1;
    vehicleLocalPosition.XY_Global = true;
    vehicleLocalPosition.Z_Global = true;
    vehicleLocalPosition.DistBottomValid = true;

    controlState.Timestamp = 86440828023;
    controlState.AccX = -0.236284539;
    controlState.AccY = 0.101594232;
    controlState.AccZ = -9.76688671;
    controlState.VelX = 0.000539890432;
    controlState.VelY = 0.0182482861;
    controlState.VelZ = 0.00519393012;
    controlState.PosX = -0.00807033759;
    controlState.PosY = -0.000570089556;
    controlState.PosZ = 0.0179239418;
    controlState.Airspeed = 0;
    controlState.VelVariance[0] = 0.0f;
    controlState.VelVariance[1] = 0.0f;
    controlState.VelVariance[2] = 0.0f;
    controlState.PosVariance[0] = 0.0f;
    controlState.PosVariance[1] = 0.0f;
    controlState.PosVariance[2] = 0.0f;
    controlState.Q[0] = 0.712293863;
    controlState.Q[1] = 0.00346049992;
    controlState.Q[2] = -0.0159233119;
    controlState.Q[3] = 0.701692224;
    controlState.DeltaQReset[0] = 0.999934673;
    controlState.DeltaQReset[1] = 0.000154913403;
    controlState.DeltaQReset[2] = -0.00022063707;
    controlState.DeltaQReset[3] = 0.0114225745;
    controlState.RollRate = 0.00590410549;
    controlState.PitchRate = 0.00479567284;
    controlState.YawRate = 0.000454715308;
    controlState.HorzAccMag = 0.158787385;
    controlState.RollRateBias = -0.00138732162;
    controlState.PitchRateBias = -0.00102514029;
    controlState.YawRateBias = -0.00102514029;
    controlState.AirspeedValid = false;
    controlState.QuatResetCounter = 2;

    Ut_CFE_SB_AddMsgToPipe(&controlState, schPipe);
    Ut_CFE_SB_AddMsgToPipe(&vehicleLocalPosition, schPipe);

    Ut_CFE_ES_SetReturnCode(UT_CFE_ES_RUNLOOP_INDEX, FALSE, 3);

    /* Execute the function being tested */
    oMPC.AppMain();

    /* Verify results */
    UtAssert_True(oMPC.HeadingResetCounter == 0,"HeadingResetCounter == 0");
    UtAssert_True(oMPC.XY_ResetCounter == 1, "oMPC.XZ_ResetCounter == 0");
    UtAssert_True(oMPC.Z_ResetCounter == 0, "oMPC.Z_ResetCounter == 0");
    UtAssert_True(oMPC.VXY_ResetCounter == 1, "oMPC.VXZ_ResetCounter == 0");
    UtAssert_True(oMPC.VZ_ResetCounter == 1, "oMPC.VZ_ResetCounter == 0");
}


/**
 * Test MPC_AppMain(), Nominal - ProcessControlStateMsg
 */
void Test_MPC_AppMain_Nominal_UpdateParamsFromTable(void)
{
    MPC oMPC;
    int32 schPipe;

    Ut_CFE_ES_SetReturnCode(UT_CFE_ES_RUNLOOP_INDEX, FALSE, 1);

    /* Execute the function being tested */
    oMPC.AppMain();

    /* Verify results */
    UtAssert_DoubleCmpAbs(oMPC.PosP[0], 0.950000, FLT_EPSILON, "oMPC.PosP[0]");
    UtAssert_DoubleCmpAbs(oMPC.PosP[1], 0.950000, FLT_EPSILON, "oMPC.PosP[1]");
    UtAssert_DoubleCmpAbs(oMPC.PosP[2], 1.000000, FLT_EPSILON, "oMPC.PosP[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelP[0], 0.150000, FLT_EPSILON, "oMPC.VelP[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelP[1], 0.150000, FLT_EPSILON, "oMPC.VelP[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelP[2], 0.600000, FLT_EPSILON, "oMPC.VelP[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelI[0], 0.200000, FLT_EPSILON, "oMPC.VelI[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelI[1], 0.200000, FLT_EPSILON, "oMPC.VelI[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelI[2], 0.150000, FLT_EPSILON, "oMPC.VelI[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelD[0], 0.010000, FLT_EPSILON, "oMPC.VelI[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelD[1], 0.010000, FLT_EPSILON, "oMPC.VelI[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelD[2], 0.000000, FLT_EPSILON, "oMPC.VelI[2]");
}



/**
 * Test MPC_AppMain(), Nominal - UpdateVelocityDerivative
 */
void Test_MPC_AppMain_Nominal_UpdateVelocityDerivative(void)
{
    MPC oMPC;

    /* Run 2 passes so we can verify the Velocity Error Derivative
     * calculation.
     *
     * Pass 1
     */
    oMPC.VehicleLocalPositionMsg.Timestamp = 66561590577;
    oMPC.VehicleLocalPositionMsg.X = 0.000000;
    oMPC.VehicleLocalPositionMsg.Y = 0.000000;
    oMPC.VehicleLocalPositionMsg.Z = -0.000002;
    oMPC.VehicleLocalPositionMsg.DistBottom = 1.029581;
    oMPC.VehicleLocalPositionMsg.DistBottomRate = 0.000493;
    oMPC.VehicleLocalPositionMsg.DistBottomValid = 1;
    oMPC.VehicleLocalPositionMsg.VX = 0.000422;
    oMPC.VehicleLocalPositionMsg.VY = 0.001771;
    oMPC.VehicleLocalPositionMsg.VZ = -0.000493;
    oMPC.Velocity[0] = 0.000000;
    oMPC.Velocity[1] = 0.000000;
    oMPC.Velocity[2] = 0.000000;
    oMPC.Position[0] = 0.000000;
    oMPC.Position[1] = 0.000000;
    oMPC.Position[2] = 0.000000;

    oMPC.UpdateVelocityDerivative(0.016292);

    UtAssert_DoubleCmpAbs(oMPC.Position[0],  0.000000, FLT_EPSILON, "oMPC.Position[0]");
    UtAssert_DoubleCmpAbs(oMPC.Position[1],  0.000000, FLT_EPSILON, "oMPC.Position[1]");
    UtAssert_DoubleCmpAbs(oMPC.Position[2], -0.000002, FLT_EPSILON, "oMPC.Position[2]");
    UtAssert_DoubleCmpAbs(oMPC.Velocity[0],  0.000422, FLT_EPSILON, "oMPC.Velocity[0]");
    UtAssert_DoubleCmpAbs(oMPC.Velocity[1],  0.001771, FLT_EPSILON, "oMPC.Velocity[1]");
    UtAssert_DoubleCmpAbs(oMPC.Velocity[2], -0.000493, FLT_EPSILON, "oMPC.Velocity[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityErrD[0],  0.000000, FLT_EPSILON, "oMPC.VelocityErrD[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityErrD[1],  0.000000, FLT_EPSILON, "oMPC.VelocityErrD[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityErrD[2],  0.000000, FLT_EPSILON, "oMPC.VelocityErrD[2]");

    /*
     * Pass 2
     */
    oMPC.VehicleLocalPositionMsg.Timestamp = 66561595349;
    oMPC.VehicleLocalPositionMsg.X = 0.000000;
    oMPC.VehicleLocalPositionMsg.Y = 0.000000;
    oMPC.VehicleLocalPositionMsg.Z = -0.000005;
    oMPC.VehicleLocalPositionMsg.DistBottom = 1.029584;
    oMPC.VehicleLocalPositionMsg.DistBottomRate = 0.000810;
    oMPC.VehicleLocalPositionMsg.DistBottomValid = 1;
    oMPC.VehicleLocalPositionMsg.VX = 0.000195;
    oMPC.VehicleLocalPositionMsg.VY = 0.003512;
    oMPC.VehicleLocalPositionMsg.VZ = -0.000810;

    oMPC.UpdateVelocityDerivative(0.004689);

    UtAssert_DoubleCmpAbs(oMPC.Position[0],  0.000000, FLT_EPSILON, "oMPC.Position[0]");
    UtAssert_DoubleCmpAbs(oMPC.Position[1],  0.000000, FLT_EPSILON, "oMPC.Position[1]");
    UtAssert_DoubleCmpAbs(oMPC.Position[2], -0.000005, FLT_EPSILON, "oMPC.Position[2]");
    UtAssert_DoubleCmpAbs(oMPC.Velocity[0],  0.000195, FLT_EPSILON, "oMPC.Velocity[0]");
    UtAssert_DoubleCmpAbs(oMPC.Velocity[1],  0.003512, FLT_EPSILON, "oMPC.Velocity[1]");
    UtAssert_DoubleCmpAbs(oMPC.Velocity[2], -0.000810, FLT_EPSILON, "oMPC.Velocity[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityErrD[0],  0.00023751, FLT_EPSILON, "oMPC.VelocityErrD[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityErrD[1], -0.00182158, FLT_EPSILON, "oMPC.VelocityErrD[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityErrD[2],  0.00033167, FLT_EPSILON, "oMPC.VelocityErrD[2]");
}



/**
 * Test MPC_AppMain(), Nominal - SlowLandGradualVelocityLimit
 */
void Test_MPC_AppMain_Nominal_SlowLandGradualVelocityLimit(void)
{
    MPC oMPC;

    /* PASS 1 - Landed and idle. */
    oMPC.Position[2] = 0.021387;
    oMPC.HomePositionMsg.Z = 0.020611;
    oMPC.VelocitySetpoint[2] = 0.000000;

    oMPC.SlowLandGradualVelocityLimit();

    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2],  0.000000, FLT_EPSILON, " oMPC.VelocitySetpoint[2]");

    /* PASS 2 - Takeoff */
    oMPC.Position[2] = -1.565828;
    oMPC.HomePositionMsg.Z = -0.007575;
    oMPC.VelocitySetpoint[2] = -0.941252;

    oMPC.SlowLandGradualVelocityLimit();

    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2],  -0.941252, FLT_EPSILON, " oMPC.VelocitySetpoint[2]");
}



/**
 * Test MPC_AppMain(), Nominal - LimitAltitude
 */
void Test_MPC_AppMain_Nominal_LimitAltitude(void)
{
    MPC oMPC;

    /* PASS 1 - Landed and idle. */
    oMPC.RunAltControl = 0;
    oMPC.PositionSetpoint[2] = 0.005629;
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000;
    oMPC.VelocitySetpoint[2] = 0.000000;
    oMPC.Velocity[2] = 0.010930;
    oMPC.Position[2] = 0.005629;

    oMPC.LimitAltitude();

    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2],  0.005629, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_True(oMPC.RunAltControl == false,  "oMPC.RunAltControl");

    /* PASS 2 - Landed and idle. */
    oMPC.RunAltControl = 0;
    oMPC.PositionSetpoint[2] = 0.005629;
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000;
    oMPC.VelocitySetpoint[2] = 0.000000;
    oMPC.Velocity[2] = 0.010245;
    oMPC.Position[2] = 0.005350;

    oMPC.LimitAltitude();

    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2],  0.005629, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_True(oMPC.RunAltControl == false,  "oMPC.RunAltControl");

    /* PASS 3 - Takeoff. */
    oMPC.RunAltControl = 1;
    oMPC.PositionSetpoint[2] = -2.729340;
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000;
    oMPC.VelocitySetpoint[2] = -0.687779;
    oMPC.Velocity[2] = -0.752254;
    oMPC.Position[2] = -0.287284;

    oMPC.LimitAltitude();

    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2],  -2.729340, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_True(oMPC.RunAltControl == true,  "oMPC.RunAltControl");
}



/**
 * Test MPC_AppMain(), Nominal - ApplyVelocitySetpointSlewRate
 */
void Test_MPC_AppMain_Nominal_ApplyVelocitySetpointSlewRate(void)
{
    MPC oMPC;

    /* PASS 1 - Landed and idle. */
    oMPC.VelocitySetpoint[0] = 0.000000;
    oMPC.VelocitySetpoint[1] = 0.000000;
    oMPC.VelocitySetpoint[2] = 0.000000;
    oMPC.VelocitySetpointPrevious[0] = -0.004032;
    oMPC.VelocitySetpointPrevious[1] = -0.010259;
    oMPC.VelocitySetpointPrevious[2] = 0.016675;

    oMPC.ApplyVelocitySetpointSlewRate(0.005820);

    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");

    /* PASS 2 - Takeoff. */
    oMPC.VelocitySetpoint[0] = 0.000719;
    oMPC.VelocitySetpoint[1] = -0.001065;
    oMPC.VelocitySetpoint[2] = -1.500000;
    oMPC.VelocitySetpointPrevious[0] = 0.000306;
    oMPC.VelocitySetpointPrevious[1] = -0.000646;
    oMPC.VelocitySetpointPrevious[2] = -0.727830;

    oMPC.ApplyVelocitySetpointSlewRate(0.068840);

    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0], 0.000719, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1], -0.001065, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2], -1.072030, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
}



/**
 * Test MPC_AppMain(), Nominal - CalculateVelocitySetpoint
 */
void Test_MPC_AppMain_Nominal_CalculateVelocitySetpoint(void)
{
    MPC oMPC;

    oMPC.InitData();
    oMPC.InitConfigTbl();

    /* PASS 1 - Landed and idle. */
    oMPC.RunPosControl = 0;
    oMPC.RunAltControl = 0;
    oMPC.Position[0] = 0.005878;
    oMPC.Position[1] = -0.002398;
    oMPC.Position[2] = 0.027536;
    oMPC.PositionSetpoint[0] = 0.005878;
    oMPC.PositionSetpoint[1] = -0.002398;
    oMPC.PositionSetpoint[2] = 0.027536;
    oMPC.Velocity[0] = 0.005205;
    oMPC.Velocity[1] = -0.006782;
    oMPC.Velocity[2] = 0.011264;
    oMPC.VelocitySetpoint[0] = 0.000000;
    oMPC.VelocitySetpoint[1] = 0.000000;
    oMPC.VelocitySetpoint[2] = 0.000000;
    oMPC.LimitVelXY = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Current.Type = PX4_SETPOINT_TYPE_POSITION;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 0;
    oMPC.InTakeoff = 0;
    oMPC.TakeoffVelLimit = 0.000000;
    oMPC.VehicleGlobalVelocitySetpointMsg.Timestamp = 0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VX = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VY = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VZ = 0.0;
    oMPC.ResetPositionSetpoint = false;
    oMPC.ResetAltitudeSetpoint = false;

    Ut_MPC_Custom_SetReturnCode(UT_MPC_PX4LIB_GETPX4TIMEUS_INDEX, 1, 1);
    oMPC.CalculateVelocitySetpoint(0.009890);

    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpointPrevious[0], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpointPrevious[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpointPrevious[1], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpointPrevious[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpointPrevious[2], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpointPrevious[2]");
    UtAssert_True(oMPC.ResetPositionSetpoint == 0, "oMPC.ResetPositionSetpoint");
    UtAssert_True(oMPC.ResetAltitudeSetpoint == 0, "oMPC.ResetAltitudeSetpoint");
    UtAssert_True(oMPC.InTakeoff == 0, "oMPC.InTakeoff");
    UtAssert_DoubleCmpAbs(oMPC.TakeoffVelLimit, 0.000000, FLT_EPSILON, "oMPC.TakeoffVelLimit");
    UtAssert_True(oMPC.VehicleGlobalVelocitySetpointMsg.Timestamp != 0, "oMPC.VehicleGlobalVelocitySetpointMsg.Timestamp");
    UtAssert_DoubleCmpAbs(oMPC.VehicleGlobalVelocitySetpointMsg.VX, 0.000000, FLT_EPSILON, "oMPC.VehicleGlobalVelocitySetpointMsg.VX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleGlobalVelocitySetpointMsg.VY, 0.000000, FLT_EPSILON, "oMPC.VehicleGlobalVelocitySetpointMsg.VY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleGlobalVelocitySetpointMsg.VZ, 0.000000, FLT_EPSILON, "oMPC.VehicleGlobalVelocitySetpointMsg.VZ");

    /* PASS 2 - Landed and idle. */
    oMPC.RunPosControl = 0;
    oMPC.RunAltControl = 0;
    oMPC.Position[0] = 0.005625;
    oMPC.Position[1] = -0.002856;
    oMPC.Position[2] = 0.027540;
    oMPC.PositionSetpoint[0] = 0.005878;
    oMPC.PositionSetpoint[1] = -0.002398;
    oMPC.PositionSetpoint[2] = 0.027536;
    oMPC.Velocity[0] = 0.004707;
    oMPC.Velocity[1] = -0.005651;
    oMPC.Velocity[2] = 0.010354;
    oMPC.VelocitySetpoint[0] = 0.000000;
    oMPC.VelocitySetpoint[1] = 0.000000;
    oMPC.VelocitySetpoint[2] = 0.000000;
    oMPC.LimitVelXY = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Current.Type = PX4_SETPOINT_TYPE_TAKEOFF;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 0;
    oMPC.InTakeoff = 0;
    oMPC.TakeoffVelLimit = 0.000000;
    oMPC.VehicleGlobalVelocitySetpointMsg.Timestamp = 0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VX = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VY = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VZ = 0.0;
    oMPC.ResetPositionSetpoint = false;
    oMPC.ResetAltitudeSetpoint = false;

    Ut_MPC_Custom_SetReturnCode(UT_MPC_PX4LIB_GETPX4TIMEUS_INDEX, 1, 1);
    oMPC.CalculateVelocitySetpoint(0.025312);

    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpointPrevious[0], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpointPrevious[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpointPrevious[1], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpointPrevious[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpointPrevious[2], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpointPrevious[2]");
    UtAssert_True(oMPC.ResetPositionSetpoint == 0, "oMPC.ResetPositionSetpoint");
    UtAssert_True(oMPC.ResetAltitudeSetpoint == 0, "oMPC.ResetAltitudeSetpoint");
    UtAssert_True(oMPC.InTakeoff == 0, "oMPC.InTakeoff");
    UtAssert_DoubleCmpAbs(oMPC.TakeoffVelLimit, 0.000000, FLT_EPSILON, "oMPC.TakeoffVelLimit");
    UtAssert_True(oMPC.VehicleGlobalVelocitySetpointMsg.Timestamp != 0, "oMPC.VehicleGlobalVelocitySetpointMsg.Timestamp");
    UtAssert_DoubleCmpAbs(oMPC.VehicleGlobalVelocitySetpointMsg.VX, 0.000000, FLT_EPSILON, "oMPC.VehicleGlobalVelocitySetpointMsg.VX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleGlobalVelocitySetpointMsg.VY, 0.000000, FLT_EPSILON, "oMPC.VehicleGlobalVelocitySetpointMsg.VY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleGlobalVelocitySetpointMsg.VZ, 0.000000, FLT_EPSILON, "oMPC.VehicleGlobalVelocitySetpointMsg.VZ");

    /* PASS 3 - Taking off */
    oMPC.RunPosControl = 1;
    oMPC.RunAltControl = 1;
    oMPC.Position[0] =  0.008560;
    oMPC.Position[1] = -0.021263;
    oMPC.Position[2] =  0.166312;
    oMPC.PositionSetpoint[0] =  0.008954;
    oMPC.PositionSetpoint[1] = -0.021870;
    oMPC.PositionSetpoint[2] = -2.334442;
    oMPC.Velocity[0] = -0.008084;
    oMPC.Velocity[1] = -0.017554;
    oMPC.Velocity[2] = -0.042712;
    oMPC.VelocitySetpoint[0] =  0.000038;
    oMPC.VelocitySetpoint[1] =  0.000105;
    oMPC.VelocitySetpoint[2] = -0.049970;
    oMPC.LimitVelXY = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 1;
    oMPC.PositionSetpointTripletMsg.Current.Type = PX4_SETPOINT_TYPE_TAKEOFF;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 0;
    oMPC.InTakeoff = 1;
    oMPC.TakeoffVelLimit = -0.500000;
    oMPC.VehicleGlobalVelocitySetpointMsg.Timestamp = 0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VX = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VY = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VZ = 0.0;
    oMPC.ResetPositionSetpoint = false;
    oMPC.ResetAltitudeSetpoint = false;
    /* For LimitAltitude() */
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000;
    /* For SlowLandGradualVelocityLimit() */
    oMPC.HomePositionMsg.Z = 0.165064;
    /* For LimitVelXYGradually() */
    oMPC.CurrentPositionSetpoint[0] =  0.008954;
    oMPC.CurrentPositionSetpoint[1] = -0.021870;
    oMPC.CurrentPositionSetpoint[2] = -2.334442;
    oMPC.Position[0] =  0.008560;
    oMPC.Position[1] = -0.021263;
    oMPC.Position[2] =  0.166312;
    /* For ApplyVelocitySetpointSlewRate() */
    oMPC.VelocitySetpointPrevious[0] =  0.000038;
    oMPC.VelocitySetpointPrevious[1] =  0.000105;
    oMPC.VelocitySetpointPrevious[2] = -0.049970;
    oMPC.Velocity[0] =  -0.008084;
    oMPC.Velocity[1] =  -0.017554;
    oMPC.Velocity[2] =  -0.042712;
    /* For GetCruisingSpeedXY() */
    oMPC.PositionSetpointTripletMsg.Current.CruisingSpeed = -1.0;

    Ut_MPC_Custom_SetReturnCode(UT_MPC_PX4LIB_GETPX4TIMEUS_INDEX, 1, 1);
    oMPC.CalculateVelocitySetpoint(0.065570);

    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0],  0.000374399736, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1], -0.000576650607, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2],  0.254112512, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpointPrevious[0],  0.000374299735, FLT_EPSILON, "oMPC.VelocitySetpointPrevious[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpointPrevious[1], -0.000576650607, FLT_EPSILON, "oMPC.VelocitySetpointPrevious[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpointPrevious[2], -0.377819985, FLT_EPSILON, "oMPC.VelocitySetpointPrevious[2]");
    UtAssert_True(oMPC.ResetPositionSetpoint == 0, "oMPC.ResetPositionSetpoint");
    UtAssert_True(oMPC.ResetAltitudeSetpoint == 0, "oMPC.ResetAltitudeSetpoint");
    UtAssert_True(oMPC.InTakeoff == 1, "oMPC.InTakeoff");
    UtAssert_DoubleCmpAbs(oMPC.TakeoffVelLimit, -0.254112512, FLT_EPSILON, "oMPC.TakeoffVelLimit");
    UtAssert_True(oMPC.VehicleGlobalVelocitySetpointMsg.Timestamp != 0, "oMPC.VehicleGlobalVelocitySetpointMsg.Timestamp");
    UtAssert_DoubleCmpAbs(oMPC.VehicleGlobalVelocitySetpointMsg.VX,  0.000374299736, FLT_EPSILON, "oMPC.VehicleGlobalVelocitySetpointMsg.VX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleGlobalVelocitySetpointMsg.VY, -0.000576650607, FLT_EPSILON, "oMPC.VehicleGlobalVelocitySetpointMsg.VY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleGlobalVelocitySetpointMsg.VZ,  0.254112512, FLT_EPSILON, "oMPC.VehicleGlobalVelocitySetpointMsg.VZ");

}



/**
 * Test MPC_AppMain(), Nominal - CalculateThrustSetpoint
 */
void Test_MPC_AppMain_Nominal_CalculateThrustSetpoint(void)
{
    MPC oMPC;

    oMPC.InitData();
    oMPC.InitConfigTbl();

    /* PASS 1 - Landed and idle. */
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.ResetIntXY = 1;
    oMPC.ResetIntZ = 1;
    oMPC.ThrustInt[0] = 0.0;
    oMPC.ThrustInt[1] = 0.0;
    oMPC.ThrustInt[2] = 0.0;
    oMPC.VehicleControlModeMsg.ControlAccelerationEnabled = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.AX = 0.0;
    oMPC.PositionSetpointTripletMsg.Current.AY = 0.0;
    oMPC.PositionSetpointTripletMsg.Current.AZ = 0.0;
    oMPC.VelocityErrD[0] = 0.01287147;
    oMPC.VelocityErrD[1] = 0.22019185;
    oMPC.VelocityErrD[2] = -0.33453843;
    oMPC.VehicleLandDetectedMsg.GroundContact = 0;
    oMPC.Rotation[0][0] = 0.03877932;
    oMPC.Rotation[0][1] = -0.99902272;
    oMPC.Rotation[0][2] = -0.02121111;
    oMPC.Rotation[1][0] = 0.99905360;
    oMPC.Rotation[1][1] = 0.03918153;
    oMPC.Rotation[1][2] = -0.01888950;
    oMPC.Rotation[2][0] = 0.01970213;
    oMPC.Rotation[2][1] = -0.02045851;
    oMPC.Rotation[2][2] = 0.99959666;
    oMPC.VelZLp = 0.0;
    oMPC.AccZLp = 0.0;
    oMPC.VehicleLandDetectedMsg.Landed = 1;
    oMPC.LndReachedGround = 0;
    oMPC.InLanding = 0;
    oMPC.VehicleAttitudeSetpointMsg.YawBody = 0.03080329;
    oMPC.RSetpoint[0][0] = 1.00000000;
    oMPC.RSetpoint[0][1] = 0.00000000;
    oMPC.RSetpoint[0][2] = 0.00000000;
    oMPC.RSetpoint[1][0] = 0.00000000;
    oMPC.RSetpoint[1][1] = 1.00000000;
    oMPC.RSetpoint[1][2] = 0.00000000;
    oMPC.RSetpoint[2][0] = 0.00000000;
    oMPC.RSetpoint[2][1] = 0.00000000;
    oMPC.RSetpoint[2][2] = 1.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[0] = 0.99988139;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[1] = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[2] = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[3] = 0.01540103;
    oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid = 1;
    oMPC.VehicleAttitudeSetpointMsg.RollBody = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.PitchBody = 0.00000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccX = 0.00000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccY = 0.00000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccZ = -0.00000000;
    /* For InAutoTakeoff() */
    oMPC.VehicleControlModeMsg.ControlOffboardEnabled = 0;
    /* For CalculateVelocitySetpoint() */
    oMPC.RunPosControl = 0;
    oMPC.RunAltControl = 0;
    oMPC.Position[0] = -0.00207413;
    oMPC.Position[1] = -0.00082683;
    oMPC.Position[2] = -0.02151199;
    oMPC.PositionSetpoint[0] = -0.00207413;
    oMPC.PositionSetpoint[1] = -0.00082683;
    oMPC.PositionSetpoint[2] = -0.02151199;
    oMPC.Velocity[0] = -0.00085380;
    oMPC.Velocity[1] = -0.00069751;
    oMPC.Velocity[2] = 0.00590463;
    oMPC.VelocitySetpoint[0] = 0.00000000;
    oMPC.VelocitySetpoint[1] = 0.00000000;
    oMPC.VelocitySetpoint[2] = 0.00000000;
    oMPC.LimitVelXY = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Current.Type = PX4_SETPOINT_TYPE_POSITION;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 0;
    oMPC.InTakeoff = 0;
    oMPC.TakeoffVelLimit = 0.00000000;
    oMPC.VehicleGlobalVelocitySetpointMsg.Timestamp = 0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VX = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VY = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VZ = 0.0;
    oMPC.ResetPositionSetpoint = false;
    oMPC.ResetAltitudeSetpoint = false;
    /* For LimitAltitude() */
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000;
    /* For SlowLandGradualVelocityLimit() */
    oMPC.HomePositionMsg.Z = -0.02179659;
    /* For LimitVelXYGradually() */
    oMPC.CurrentPositionSetpoint[0] = 0.00000000;
    oMPC.CurrentPositionSetpoint[1] = 0.00000000;
    oMPC.CurrentPositionSetpoint[2] = 0.00000000;
    oMPC.Position[0] = -0.00207413;
    oMPC.Position[1] = -0.00082683;
    oMPC.Position[2] = -0.02151199;
    /* For ApplyVelocitySetpointSlewRate() */
    oMPC.VelocityPrevious[0] = -0.00135698;
    oMPC.VelocityPrevious[1] = -0.00031078;
    oMPC.VelocityPrevious[2] = 0.00620359;
    oMPC.Velocity[0] = -0.00085380;
    oMPC.Velocity[1] = -0.00069751;
    oMPC.Velocity[2] = 0.00590463;
    /* For GetCruisingSpeedXY() */
    oMPC.PositionSetpointTripletMsg.Current.CruisingSpeed = 0.00000000;

    Ut_MPC_Custom_SetReturnCode(UT_MPC_PX4LIB_GETPX4TIMEUS_INDEX, 1, 1);
    oMPC.CalculateThrustSetpoint(0.00549900);

    UtAssert_True(oMPC.ResetIntXY == false, "oMPC.ResetIntXY");
    UtAssert_True(oMPC.ResetIntZ == false, "oMPC.ResetIntZ");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[0], 0.000000, FLT_EPSILON, "oMPC.ThrustInt[0]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[1], 0.000000, FLT_EPSILON, "oMPC.ThrustInt[1]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[2], 0.000000, FLT_EPSILON, "oMPC.ThrustInt[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2], 0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[0], -0.00207413, FLT_EPSILON, "oMPC.PositionSetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[1], -0.00082683, FLT_EPSILON, "oMPC.PositionSetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2], -0.02151199, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelZLp,  0.00025976, FLT_EPSILON, "oMPC.VelZLp");
    UtAssert_DoubleCmpAbs(oMPC.AccZLp, -0.00239170, FLT_EPSILON, "oMPC.AccZLp");
    UtAssert_True(oMPC.InLanding == false, "oMPC.InLanding");
    UtAssert_True(oMPC.LndReachedGround == false, "oMPC.LndReachedGround");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Thrust, 0.12000000, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Thrust");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][0],  0.99952561, FLT_EPSILON, "oMPC.RSetpoint[0][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][1], -0.03079842, FLT_EPSILON, "oMPC.RSetpoint[0][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][2],  0.00000000, FLT_EPSILON, "oMPC.RSetpoint[0][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][0],  0.03079842, FLT_EPSILON, "oMPC.RSetpoint[1][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][1],  0.99952561, FLT_EPSILON, "oMPC.RSetpoint[1][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][2],  0.00000000, FLT_EPSILON, "oMPC.RSetpoint[1][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][0], -0.00000000, FLT_EPSILON, "oMPC.RSetpoint[2][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][1],  0.00000000, FLT_EPSILON, "oMPC.RSetpoint[2][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][2],  1.00000000, FLT_EPSILON, "oMPC.RSetpoint[2][2]");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid == true, "oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.RollBody, 0.000000, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.RollBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.PitchBody, 0.000000, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.PitchBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccX, 0.000000, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccY, 0.000000, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccZ, 0.000000, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccZ");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Timestamp != 0, "oMPC.VehicleAttitudeSetpointMsg.Timestamp");

    /* PASS 2 - Transition to takeoff. */
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.ResetIntXY = 1;
    oMPC.ResetIntZ = 1;
    oMPC.ThrustInt[0] = 0.0;
    oMPC.ThrustInt[1] = 0.0;
    oMPC.ThrustInt[2] = 0.0;
    oMPC.VehicleControlModeMsg.ControlAccelerationEnabled = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.AX = 0.0;
    oMPC.PositionSetpointTripletMsg.Current.AY = 0.0;
    oMPC.PositionSetpointTripletMsg.Current.AZ = 0.0;
    oMPC.VelocityErrD[0] =  0.00865284;
    oMPC.VelocityErrD[1] = -0.01698322;
    oMPC.VelocityErrD[2] =  0.01355007;
    oMPC.VehicleLandDetectedMsg.GroundContact = 0;
    oMPC.Rotation[0][0] = -0.01877549;
    oMPC.Rotation[0][1] = -0.99964637;
    oMPC.Rotation[0][2] = -0.01883845;
    oMPC.Rotation[1][0] = 0.99973601;
    oMPC.Rotation[1][1] = -0.01852074;
    oMPC.Rotation[1][2] = -0.01360697;
    oMPC.Rotation[2][0] = 0.01325325;
    oMPC.Rotation[2][1] = -0.01908895;
    oMPC.Rotation[2][2] = 0.99973011;
    oMPC.VelZLp = 0.00051300;
    oMPC.AccZLp = -0.01061635;
    oMPC.VehicleLandDetectedMsg.Landed = 1;
    oMPC.LndReachedGround = 0;
    oMPC.InLanding = 0;
    oMPC.RSetpoint[0][0] = 1.00000000;
    oMPC.RSetpoint[0][1] = 0.00000000;
    oMPC.RSetpoint[0][2] = 0.00000000;
    oMPC.RSetpoint[1][0] = 0.00000000;
    oMPC.RSetpoint[1][1] = 1.00000000;
    oMPC.RSetpoint[1][2] = 0.00000000;
    oMPC.RSetpoint[2][0] = 0.00000000;
    oMPC.RSetpoint[2][1] = 0.00000000;
    oMPC.RSetpoint[2][2] = 1.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[0] = 1.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[1] = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[2] = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[3] = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid = 1;
    oMPC.VehicleAttitudeSetpointMsg.RollBody = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.PitchBody = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.YawBody = 1.58957458;
    oMPC.VehicleLocalPositionSetpointMsg.AccX = 0.00000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccY = 0.00000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccZ = -0.00000000;
    /* For InAutoTakeoff() */
    oMPC.VehicleControlModeMsg.ControlOffboardEnabled = 0;
    /* For CalculateVelocitySetpoint() */
    oMPC.RunPosControl = 1;
    oMPC.RunAltControl = 1;
    oMPC.Position[0] = 0.01195339;
    oMPC.Position[1] = -0.01815947;
    oMPC.Position[2] = 0.14472280;
    oMPC.PositionSetpoint[0] =  0.01195339;
    oMPC.PositionSetpoint[1] = -0.01815947;
    oMPC.PositionSetpoint[2] = -2.35528564;
    oMPC.Velocity[0] = -0.00035331;
    oMPC.Velocity[1] = -0.03173279;
    oMPC.Velocity[2] =  0.12474953;
    oMPC.VelocitySetpoint[0] = -0.00000000;
    oMPC.VelocitySetpoint[1] =  0.00000000;
    oMPC.VelocitySetpoint[2] = -0.06405000;
    oMPC.LimitVelXY = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 1;
    oMPC.PositionSetpointTripletMsg.Current.Type = PX4_SETPOINT_TYPE_TAKEOFF;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 0;
    oMPC.InTakeoff = 0;
    oMPC.TakeoffVelLimit = 0.00000000;
    oMPC.VehicleGlobalVelocitySetpointMsg.Timestamp = 0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VX = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VY = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VZ = 0.0;
    oMPC.ResetPositionSetpoint = false;
    oMPC.ResetAltitudeSetpoint = false;
    /* For LimitAltitude() */
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000;
    /* For SlowLandGradualVelocityLimit() */
    oMPC.HomePositionMsg.Z = 0.14212561;
    /* For LimitVelXYGradually() */
    oMPC.CurrentPositionSetpoint[0] = 0.01195339;
    oMPC.CurrentPositionSetpoint[1] = -0.01815947;
    oMPC.CurrentPositionSetpoint[2] = -2.35528564;
    oMPC.Position[0] = 0.01195339;
    oMPC.Position[1] = -0.01815947;
    oMPC.Position[2] = 0.14472280;
    /* For ApplyVelocitySetpointSlewRate() */
    oMPC.VelocityPrevious[0] = -0.00012040;
    oMPC.VelocityPrevious[1] = -0.03237132;
    oMPC.VelocityPrevious[2] = 0.12617928;
    oMPC.Velocity[0] = -0.00035331;
    oMPC.Velocity[1] = -0.03173279;
    oMPC.Velocity[2] = 0.12474953;
    /* For GetCruisingSpeedXY() */
    oMPC.PositionSetpointTripletMsg.Current.CruisingSpeed = -1.00000000;

    Ut_MPC_Custom_SetReturnCode(UT_MPC_PX4LIB_GETPX4TIMEUS_INDEX, 1, 1);
    oMPC.CalculateThrustSetpoint(0.01281000);

    UtAssert_True(oMPC.ResetIntXY == false, "oMPC.ResetIntXY");
    UtAssert_True(oMPC.ResetIntZ == false, "oMPC.ResetIntZ");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[0],  0.00000091, FLT_EPSILON, "oMPC.ThrustInt[0]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[1],  0.00008130, FLT_EPSILON, "oMPC.ThrustInt[1]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[2], -0.00036278, FLT_EPSILON, "oMPC.ThrustInt[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0], -0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1],  0.000000, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2], -0.06405000, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[0], 0.01195339, FLT_EPSILON, "oMPC.PositionSetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[1], -0.01815947, FLT_EPSILON, "oMPC.PositionSetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2], -2.35528564, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelZLp,  0.01324476, FLT_EPSILON, "oMPC.VelZLp");
    UtAssert_DoubleCmpAbs(oMPC.AccZLp, -0.02096640, FLT_EPSILON, "oMPC.AccZLp");
    UtAssert_True(oMPC.InLanding == false, "oMPC.InLanding");
    UtAssert_True(oMPC.LndReachedGround == false, "oMPC.LndReachedGround");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Thrust, 0.61317927, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Thrust");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][0], -0.01877662, FLT_EPSILON, "oMPC.RSetpoint[0][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][1], -0.99982363, FLT_EPSILON, "oMPC.RSetpoint[0][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][2], -0.00022750, FLT_EPSILON, "oMPC.RSetpoint[0][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][0],  0.99979573, FLT_EPSILON, "oMPC.RSetpoint[1][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][1], -0.01877439, FLT_EPSILON, "oMPC.RSetpoint[1][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][2], -0.00748428, FLT_EPSILON, "oMPC.RSetpoint[1][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][0],  0.00747869, FLT_EPSILON, "oMPC.RSetpoint[2][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][1], -0.00036798, FLT_EPSILON, "oMPC.RSetpoint[2][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][2],  0.99997193, FLT_EPSILON, "oMPC.RSetpoint[2][2]");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid == true, "oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.RollBody, -0.00036799, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.RollBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.PitchBody, -0.00747876, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.PitchBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccX,  0.00136826, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccY,  0.04501338, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccZ, -6.01421928, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccZ");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Timestamp != 0, "oMPC.VehicleAttitudeSetpointMsg.Timestamp");

    /* PASS 3 - Taking off. */
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.ResetIntXY = 0;
    oMPC.ResetIntZ = 0;
    oMPC.ThrustInt[0] = -0.000015456446;
    oMPC.ThrustInt[1] = -0.000006746221;
    oMPC.ThrustInt[2] = 0.000065119777;
    oMPC.VehicleControlModeMsg.ControlAccelerationEnabled = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.AX = 0.0;
    oMPC.PositionSetpointTripletMsg.Current.AY = 0.0;
    oMPC.PositionSetpointTripletMsg.Current.AZ = 0.0;
    oMPC.VelocityErrD[0] = -0.037555422634;
    oMPC.VelocityErrD[1] = -0.007769315038;
    oMPC.VelocityErrD[2] = 0.097660362720;
    oMPC.VehicleLandDetectedMsg.GroundContact = 0;
    oMPC.Rotation[0][0] = 0.793960154057;
    oMPC.Rotation[0][1] = -0.606549143791;
    oMPC.Rotation[0][2] = -0.041537456214;
    oMPC.Rotation[1][0] = 0.606583714485;
    oMPC.Rotation[1][1] = 0.794909775257;
    oMPC.Rotation[1][2] = -0.013206949458;
    oMPC.Rotation[2][0] = 0.041029199958;
    oMPC.Rotation[2][1] = -0.014710154384;
    oMPC.Rotation[2][2] = 0.999049603939;
    oMPC.VelZLp = -0.097704738379;
    oMPC.AccZLp = -0.005671607330;
    oMPC.VehicleLandDetectedMsg.Landed = 0;
    oMPC.LndReachedGround = 0;
    oMPC.InLanding = 0;
    oMPC.RSetpoint[0][0] =  0.794545173645;
    oMPC.RSetpoint[0][1] = -0.607062637806;
    oMPC.RSetpoint[0][2] =  0.013155251741;
    oMPC.RSetpoint[1][0] =  0.607051432133;
    oMPC.RSetpoint[1][1] =  0.794645130634;
    oMPC.RSetpoint[1][2] =  0.005279467907;
    oMPC.RSetpoint[2][0] = -0.013658720069;
    oMPC.RSetpoint[2][1] =  0.003791138530;
    oMPC.RSetpoint[2][2] =  0.999899625778;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[0] = 0.94724464;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[1] = -0.00039280;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[2] = 0.00707683;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[3] = 0.32043308;
    oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid = 1;
    oMPC.VehicleAttitudeSetpointMsg.RollBody = 0.00379150;
    oMPC.VehicleAttitudeSetpointMsg.PitchBody = 0.01365914;
    oMPC.VehicleAttitudeSetpointMsg.YawBody = 0.65241611;
	oMPC.VehicleLocalPositionSetpointMsg.AccX = -0.050282537937;
	oMPC.VehicleLocalPositionSetpointMsg.AccY = -0.020179396495;
	oMPC.VehicleLocalPositionSetpointMsg.AccZ = -3.821856975555;
    /* For InAutoTakeoff() */
    oMPC.VehicleControlModeMsg.ControlOffboardEnabled = 0;
    /* For CalculateVelocitySetpoint() */
    oMPC.RunPosControl = 1;
    oMPC.RunAltControl = 1;
    oMPC.Position[0] = 0.008880654350;
    oMPC.Position[1] = 0.006610852201;
    oMPC.Position[2] = -0.043556511402;
    oMPC.PositionSetpoint[0] = 0.008687019348;
    oMPC.PositionSetpoint[1] = 0.006540667731;
    oMPC.PositionSetpoint[2] = -2.542480468750;
    oMPC.Velocity[0] = 0.033547148108;
    oMPC.Velocity[1] = 0.014865242876;
    oMPC.Velocity[2] = -0.198812872171;
    oMPC.VelocitySetpoint[0] = -0.000183953249;
    oMPC.VelocitySetpoint[1] = -0.000066675246;
    oMPC.VelocitySetpoint[2] = -0.055955003947;
    oMPC.LimitVelXY = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 1;
    oMPC.PositionSetpointTripletMsg.Current.Type = PX4_SETPOINT_TYPE_TAKEOFF;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 0;
    oMPC.InTakeoff = 1;
    oMPC.TakeoffVelLimit = 0.00000000;
    oMPC.VehicleGlobalVelocitySetpointMsg.Timestamp = 0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VX = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VY = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VZ = 0.0;
    oMPC.ResetPositionSetpoint = false;
    oMPC.ResetAltitudeSetpoint = false;
    /* For LimitAltitude() */
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000;
    /* For SlowLandGradualVelocityLimit() */
    oMPC.HomePositionMsg.Z = -0.044796019793;
    /* For LimitVelXYGradually() */
    oMPC.CurrentPositionSetpoint[0] = 0.008687019348;
    oMPC.CurrentPositionSetpoint[1] = 0.006540667731;
    oMPC.CurrentPositionSetpoint[2] = -2.542480468750;
    /* For ApplyVelocitySetpointSlewRate() */
    oMPC.VelocityPrevious[0] = 0.032718978822;
    oMPC.VelocityPrevious[1] = 0.014280737378;
    oMPC.VelocityPrevious[2] = -0.195608407259;
    /* For GetCruisingSpeedXY() */
    oMPC.PositionSetpointTripletMsg.Current.CruisingSpeed = -1.00000000;

    Ut_MPC_Custom_SetReturnCode(UT_MPC_PX4LIB_GETPX4TIMEUS_INDEX, 1, 1);
    oMPC.CalculateThrustSetpoint(0.008829000406);

    UtAssert_True(oMPC.ResetIntXY == false, "oMPC.ResetIntXY");
    UtAssert_True(oMPC.ResetIntZ == false, "oMPC.ResetIntZ");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[0], -0.000075018834, FLT_EPSILON, "oMPC.ThrustInt[0]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[1], -0.000033113003, FLT_EPSILON, "oMPC.ThrustInt[1]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[2],  0.000254313607, FLT_EPSILON, "oMPC.ThrustInt[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0], -0.000183953249, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1], -0.000066675246, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2], -0.055955003947, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[0],  0.008687019348, FLT_EPSILON, "oMPC.PositionSetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[1],  0.006540667731, FLT_EPSILON, "oMPC.PositionSetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2], -2.542480468750, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelZLp, -0.104846209288, FLT_EPSILON, "oMPC.VelZLp");
    UtAssert_DoubleCmpAbs(oMPC.AccZLp, -0.030906729400, FLT_EPSILON, "oMPC.AccZLp");
    UtAssert_True(oMPC.InLanding == false, "oMPC.InLanding");
    UtAssert_True(oMPC.LndReachedGround == false, "oMPC.LndReachedGround");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Thrust, 0.413569390774, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Thrust");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][0],  0.794542908669, FLT_EPSILON, "oMPC.RSetpoint[0][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][1], -0.607065439224, FLT_EPSILON, "oMPC.RSetpoint[0][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][2],  0.013157540932, FLT_EPSILON, "oMPC.RSetpoint[0][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][0],  0.607049763203, FLT_EPSILON, "oMPC.RSetpoint[1][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][1],  0.794644057751, FLT_EPSILON, "oMPC.RSetpoint[1][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][2],  0.005610517692, FLT_EPSILON, "oMPC.RSetpoint[1][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][0], -0.013861512765, FLT_EPSILON, "oMPC.RSetpoint[2][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][1],  0.003529485315, FLT_EPSILON, "oMPC.RSetpoint[2][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][2],  0.999897718430, FLT_EPSILON, "oMPC.RSetpoint[2][2]");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid == true, "oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.RollBody, 0.003529831767, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.RollBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.PitchBody, 0.013861957006, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.PitchBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccX, -0.053452875465, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccY, -0.022792883217, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccZ, -4.062112331390, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccZ");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Timestamp != 0, "oMPC.VehicleAttitudeSetpointMsg.Timestamp");
}



/**
 * Test MPC_AppMain(), Nominal - ControlAuto
 */
void Test_MPC_AppMain_Nominal_ControlAuto(void)
{
    MPC oMPC;

    oMPC.InitData();
    oMPC.InitConfigTbl();

    /* PASS 1 - Initializing. */
    oMPC.VehicleStatusMsg.Timestamp = 20280576066;
    oMPC.VehicleStatusMsg.SystemID = 1;
    oMPC.VehicleStatusMsg.ComponentID = 1;
    oMPC.VehicleStatusMsg.OnboardControlSensorsPresent = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsEnabled = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsHealth = 0;
    oMPC.VehicleStatusMsg.NavState = (PX4_NavigationState_t)4;
    oMPC.VehicleStatusMsg.ArmingState = (PX4_ArmingState_t)1;
    oMPC.VehicleStatusMsg.HilState = (PX4_HilState_t)0;
    oMPC.VehicleStatusMsg.Failsafe = 0;
    oMPC.VehicleStatusMsg.SystemType = (PX4_SystemType_t)13;
    oMPC.VehicleStatusMsg.IsRotaryWing = 1;
    oMPC.VehicleStatusMsg.IsVtol = 0;
    oMPC.VehicleStatusMsg.VtolFwPermanentStab = 0;
    oMPC.VehicleStatusMsg.InTransitionMode = 0;
    oMPC.VehicleStatusMsg.RcInputMode = (PX4_RcInMode_t)1;
    oMPC.VehicleStatusMsg.DataLinkLost = 1;
    oMPC.VehicleStatusMsg.DataLinkLostCounter = 0;
    oMPC.VehicleStatusMsg.EngineFailure = 0;
    oMPC.VehicleStatusMsg.EngineFailureCmd = 0;
    oMPC.VehicleStatusMsg.MissionFailure = 0;
    oMPC.VehicleLandDetectedMsg.Timestamp = 20278631900;
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000000000;
    oMPC.VehicleLandDetectedMsg.Landed = 1;
    oMPC.VehicleLandDetectedMsg.Freefall = 0;
    oMPC.VehicleLandDetectedMsg.GroundContact = 0;
    oMPC.ControlStateMsg.Timestamp = 20280585087;
    oMPC.ControlStateMsg.AccX = -0.174654871225;
    oMPC.ControlStateMsg.AccY = 0.096246533096;
    oMPC.ControlStateMsg.AccZ = -9.875363349915;
    oMPC.ControlStateMsg.VelX = 0.000000000000;
    oMPC.ControlStateMsg.VelY = 0.000000000000;
    oMPC.ControlStateMsg.VelZ = 0.000000000000;
    oMPC.ControlStateMsg.PosX = 0.000000000000;
    oMPC.ControlStateMsg.PosY = 0.000000000000;
    oMPC.ControlStateMsg.PosZ = 0.000000000000;
    oMPC.ControlStateMsg.Airspeed = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.Q[0] = 0.694662868977;
    oMPC.ControlStateMsg.Q[1] = 0.012822493911;
    oMPC.ControlStateMsg.Q[2] = -0.013447887264;
    oMPC.ControlStateMsg.Q[3] = 0.719095528126;
    oMPC.ControlStateMsg.DeltaQReset[0] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[1] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[2] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[3] = 0.000000000000;
    oMPC.ControlStateMsg.RollRate = -0.001191451796;
    oMPC.ControlStateMsg.PitchRate = -0.004856707528;
    oMPC.ControlStateMsg.YawRate = 0.000788278761;
    oMPC.ControlStateMsg.HorzAccMag = 0.000000000000;
    oMPC.ControlStateMsg.RollRateBias = 0.000000000000;
    oMPC.ControlStateMsg.PitchRateBias = 0.000000000000;
    oMPC.ControlStateMsg.YawRateBias = 0.000000000000;
    oMPC.ControlStateMsg.AirspeedValid = 1;
    oMPC.ControlStateMsg.QuatResetCounter = 1;
    oMPC.ManualControlSetpointMsg.Timestamp = 0;
    oMPC.ManualControlSetpointMsg.X = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Y = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Z = 0.000000000000;
    oMPC.ManualControlSetpointMsg.R = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Flaps = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux1 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux2 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux3 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux4 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux5 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.ModeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ReturnSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.RattitudeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.PosctlSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.LoiterSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.AcroSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.OffboardSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.KillSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ArmSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.TransitionSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.GearSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ModeSlot = (PX4_ModeSlot_t)0;
    oMPC.ManualControlSetpointMsg.DataSource = (PX4_ManualControlDataSource_t)0;
    oMPC.ManualControlSetpointMsg.StabSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ManSwitch = (PX4_SwitchPos_t)0;
    oMPC.PositionSetpointTripletMsg.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Previous.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Current.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Current.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Current.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Current.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Next.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Next.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Next.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Next.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationIsForce = 0;
    oMPC.VehicleAttitudeSetpointMsg.Timestamp = 20280574950;
    oMPC.VehicleAttitudeSetpointMsg.RollBody = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.PitchBody = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.YawBody = 1.57048452;
    oMPC.VehicleAttitudeSetpointMsg.YawSpMoveRate = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[0] = 0.70721704;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[1] = -0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[2] = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[3] = 0.70699656;
    oMPC.VehicleAttitudeSetpointMsg.Thrust = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.LandingGear = -1.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid = 1;
    oMPC.VehicleAttitudeSetpointMsg.RollResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.PitchResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.YawResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.FwControlYaw = 0;
    oMPC.VehicleAttitudeSetpointMsg.DisableMcYawControl = 0;
    oMPC.VehicleAttitudeSetpointMsg.ApplyFlaps = 0;
    oMPC.VehicleControlModeMsg.Timestamp = 20280576066;
    oMPC.VehicleControlModeMsg.Armed = 0;
    oMPC.VehicleControlModeMsg.ExternalManualOverrideOk = 0;
    oMPC.VehicleControlModeMsg.SystemHilEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlAutoEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlOffboardEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlRatesEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAttitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlRattitudeEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlForceEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlAccelerationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlTerminationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlFixedHdgEnabled = 0;
    oMPC.VehicleLocalPositionMsg.Timestamp = 20280585123;
    oMPC.VehicleLocalPositionMsg.RefTimestamp = 20280585123;
    oMPC.VehicleLocalPositionMsg.RefLat = 47.397743983469;
    oMPC.VehicleLocalPositionMsg.RefLon = 8.545593796482;
    oMPC.VehicleLocalPositionMsg.SurfaceBottomTimestamp = 20280585123;
    oMPC.VehicleLocalPositionMsg.X = 0.001803947147;
    oMPC.VehicleLocalPositionMsg.Y = 0.000300874759;
    oMPC.VehicleLocalPositionMsg.Z = -0.000000002900;
    oMPC.VehicleLocalPositionMsg.Delta_XY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_XY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_Z = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.VX = -0.013946888968;
    oMPC.VehicleLocalPositionMsg.VY = 0.019273743033;
    oMPC.VehicleLocalPositionMsg.VZ = -0.225940167904;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AX = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AY = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Yaw = 1.605385303497;
    oMPC.VehicleLocalPositionMsg.RefAlt = 488.269012451172;
    oMPC.VehicleLocalPositionMsg.DistBottom = -0.000000000137;
    oMPC.VehicleLocalPositionMsg.DistBottomRate = 0.225940167904;
    oMPC.VehicleLocalPositionMsg.EpH = 3.000000000000;
    oMPC.VehicleLocalPositionMsg.EpV = 2.000649929047;
    oMPC.VehicleLocalPositionMsg.EvV = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.EstimatorType = 0;
    oMPC.VehicleLocalPositionMsg.XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.Z_Valid = 0;
    oMPC.VehicleLocalPositionMsg.V_XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.V_Z_Valid = 0;
    oMPC.VehicleLocalPositionMsg.XY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.Z_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VXY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VZ_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.XY_Global = 1;
    oMPC.VehicleLocalPositionMsg.Z_Global = 0;
    oMPC.VehicleLocalPositionMsg.DistBottomValid = 0;
    oMPC.VehicleLocalPositionSetpointMsg.Timestamp = 0;
    oMPC.VehicleLocalPositionSetpointMsg.X = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.Y = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.Z = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.Yaw = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.VX = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.VY = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.VZ = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccX = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccY = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccZ = 0.000000000000;
    oMPC.HomePositionMsg.Timestamp = 20280565980;
    oMPC.HomePositionMsg.Lat = 47.397744000959;
    oMPC.HomePositionMsg.Lon = 8.545593797628;
    oMPC.HomePositionMsg.Alt = 488.269012451172;
    oMPC.HomePositionMsg.X = 0.001944838790;
    oMPC.HomePositionMsg.Y = 0.000086301727;
    oMPC.HomePositionMsg.Z = -0.000000002846;
    oMPC.HomePositionMsg.Yaw = 1.575607180595;
    oMPC.HomePositionMsg.DirectionX = 0.000000000000;
    oMPC.HomePositionMsg.DirectionY = 0.000000000000;
    oMPC.HomePositionMsg.DirectionZ = 0.000000000000;
    oMPC.GearStateInitialized = 0;
    oMPC.RefAlt = 488.269012451172;
    oMPC.RefPos.timestamp = 20280585168;
    oMPC.RefPos.lat_rad = 0.827246690529;
    oMPC.RefPos.lon_rad = 0.149148748287;
    oMPC.RefPos.sin_lat = 0.736070434614;
    oMPC.RefPos.cos_lat = 0.676904952919;
    oMPC.RefPos.init_done = 1;
    oMPC.RefTimestamp = 20280585123;
    oMPC.ResetPositionSetpoint = 1;
    oMPC.ResetAltitudeSetpoint = 1;
    oMPC.DoResetAltPos = 1;
    oMPC.ModeAuto = 0;
    oMPC.PositionHoldEngaged = 0;
    oMPC.AltitudeHoldEngaged = 0;
    oMPC.RunPosControl = 1;
    oMPC.RunAltControl = 1;
    oMPC.ResetIntZ = 1;
    oMPC.ResetIntXY = 1;
    oMPC.ResetIntZManual = 0;
    oMPC.ResetYawSetpoint = 0;
    oMPC.HoldOffboardXY = 0;
    oMPC.HoldOffboardZ = 0;
    oMPC.LimitVelXY = 0;
    oMPC.ThrustInt[0] = 0.000000000000;
    oMPC.ThrustInt[1] = 0.000000000000;
    oMPC.ThrustInt[2] = 0.000000000000;
    oMPC.Position[0] = 0.001803947147;
    oMPC.Position[1] = 0.000300874759;
    oMPC.Position[2] = -0.000000002900;
    oMPC.PositionSetpoint[0] = 0.000000000000;
    oMPC.PositionSetpoint[1] = 0.000000000000;
    oMPC.PositionSetpoint[2] = 0.000000000000;
    oMPC.Velocity[0] = -0.013946888968;
    oMPC.Velocity[1] = 0.019273743033;
    oMPC.Velocity[2] = -0.225940167904;
    oMPC.VelocitySetpoint[0] = 0.000000000000;
    oMPC.VelocitySetpoint[1] = 0.000000000000;
    oMPC.VelocitySetpoint[2] = 0.000000000000;
    oMPC.VelocityPrevious[0] = -0.013365665451;
    oMPC.VelocityPrevious[1] = 0.018462687731;
    oMPC.VelocityPrevious[2] = -0.226975008845;
    oMPC.VelocityFF[0] = 0.000000000000;
    oMPC.VelocityFF[1] = 0.000000000000;
    oMPC.VelocityFF[2] = 0.000000000000;
    oMPC.VelocitySetpointPrevious[0] = -0.013365665451;
    oMPC.VelocitySetpointPrevious[1] = 0.018462687731;
    oMPC.VelocitySetpointPrevious[2] = -0.226975008845;
    oMPC.VelocityErrD[0] = 0.019212024286;
    oMPC.VelocityErrD[1] = -0.031484425068;
    oMPC.VelocityErrD[2] = -0.046475097537;
    oMPC.CurrentPositionSetpoint[0] = 0.000000000000;
    oMPC.CurrentPositionSetpoint[1] = 0.000000000000;
    oMPC.CurrentPositionSetpoint[2] = 0.000000000000;
    oMPC.Rotation[0][0] = -0.034558296204;
    oMPC.Rotation[0][1] = -0.999402821064;
    oMPC.Rotation[0][2] = -0.000242298469;
    oMPC.Rotation[1][0] = 0.998713076115;
    oMPC.Rotation[1][1] = -0.034525454044;
    oMPC.Rotation[1][2] = -0.037155251950;
    oMPC.Rotation[2][0] = 0.037124693394;
    oMPC.Rotation[2][1] = -0.001526009291;
    oMPC.Rotation[2][2] = 0.999309599400;
    oMPC.Yaw = 1.605385303497;
    oMPC.YawTakeoff = 0.000000000000;
    oMPC.InLanding = 0;
    oMPC.LndReachedGround = 0;
    oMPC.VelZLp = 0.000000000000;
    oMPC.AccZLp = 0.000000000000;
    oMPC.VelMaxXY = 8.000000000000;
    oMPC.InTakeoff = 0;
    oMPC.TakeoffVelLimit = 0.000000000000;
    oMPC.Z_ResetCounter = 0;
    oMPC.XY_ResetCounter = 0;
    oMPC.VZ_ResetCounter = 0;
    oMPC.VXY_ResetCounter = 0;
    oMPC.HeadingResetCounter = 0;
    oMPC.RSetpoint[0][0] = 1.000000000000;
    oMPC.RSetpoint[0][1] = 0.000000000000;
    oMPC.RSetpoint[0][2] = 0.000000000000;
    oMPC.RSetpoint[1][0] = 0.000000000000;
    oMPC.RSetpoint[1][1] = 1.000000000000;
    oMPC.RSetpoint[1][2] = 0.000000000000;
    oMPC.RSetpoint[2][0] = 0.000000000000;
    oMPC.RSetpoint[2][1] = 0.000000000000;
    oMPC.RSetpoint[2][2] = 1.000000000000;

    Ut_MPC_Custom_SetReturnCode(UT_MPC_PX4LIB_GETPX4TIMEUS_INDEX, 1, 1);
    oMPC.ControlAuto(0.010219999589);

    UtAssert_True(oMPC.ResetIntXY == true, "oMPC.ResetIntXY");
    UtAssert_True(oMPC.ResetIntZ == true, "oMPC.ResetIntZ");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[0], 0.000000000000, FLT_EPSILON, "oMPC.ThrustInt[0]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[1], 0.000000000000, FLT_EPSILON, "oMPC.ThrustInt[1]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[2], 0.000000000000, FLT_EPSILON, "oMPC.ThrustInt[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0], 0.000000000000, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1], 0.000000000000, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2], 0.000000000000, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[0], 0.001803947147, FLT_EPSILON, "oMPC.PositionSetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[1], 0.000300874759, FLT_EPSILON, "oMPC.PositionSetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2], -0.000000002900, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelZLp, 0.000000000000, FLT_EPSILON, "oMPC.VelZLp");
    UtAssert_DoubleCmpAbs(oMPC.AccZLp, 0.000000000000, FLT_EPSILON, "oMPC.AccZLp");
    UtAssert_True(oMPC.InLanding == false, "oMPC.InLanding");
    UtAssert_True(oMPC.LndReachedGround == false, "oMPC.LndReachedGround");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Thrust, 0.000000000000, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Thrust");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][0],  1.000000000000, FLT_EPSILON, "oMPC.RSetpoint[0][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][1],  0.000000000000, FLT_EPSILON, "oMPC.RSetpoint[0][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][2],  0.000000000000, FLT_EPSILON, "oMPC.RSetpoint[0][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][0],  0.000000000000, FLT_EPSILON, "oMPC.RSetpoint[1][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][1],  1.000000000000, FLT_EPSILON, "oMPC.RSetpoint[1][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][2],  0.000000000000, FLT_EPSILON, "oMPC.RSetpoint[1][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][0],  0.000000000000, FLT_EPSILON, "oMPC.RSetpoint[2][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][1],  0.000000000000, FLT_EPSILON, "oMPC.RSetpoint[2][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][2],  1.000000000000, FLT_EPSILON, "oMPC.RSetpoint[2][2]");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid == true, "oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.RollBody, -0.000000000000, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.RollBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.PitchBody, 0.000000000000, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.PitchBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.YawBody, 1.570484519005, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.YawBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccX, 0.000000000000, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccY, 0.000000000000, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccZ, 0.000000000000, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccZ");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Timestamp != 0, "oMPC.VehicleAttitudeSetpointMsg.Timestamp");

    /* PASS 2 - Transition to takeoff. */
    oMPC.VehicleStatusMsg.Timestamp = 22718352358;
    oMPC.VehicleStatusMsg.SystemID = 1;
    oMPC.VehicleStatusMsg.ComponentID = 1;
    oMPC.VehicleStatusMsg.OnboardControlSensorsPresent = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsEnabled = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsHealth = 0;
    oMPC.VehicleStatusMsg.NavState = (PX4_NavigationState_t)17;
    oMPC.VehicleStatusMsg.ArmingState = (PX4_ArmingState_t)2;
    oMPC.VehicleStatusMsg.HilState = (PX4_HilState_t)0;
    oMPC.VehicleStatusMsg.Failsafe = 0;
    oMPC.VehicleStatusMsg.SystemType = (PX4_SystemType_t)13;
    oMPC.VehicleStatusMsg.IsRotaryWing = 1;
    oMPC.VehicleStatusMsg.IsVtol = 0;
    oMPC.VehicleStatusMsg.VtolFwPermanentStab = 0;
    oMPC.VehicleStatusMsg.InTransitionMode = 0;
    oMPC.VehicleStatusMsg.RcInputMode = (PX4_RcInMode_t)1;
    oMPC.VehicleStatusMsg.DataLinkLost = 1;
    oMPC.VehicleStatusMsg.DataLinkLostCounter = 0;
    oMPC.VehicleStatusMsg.EngineFailure = 0;
    oMPC.VehicleStatusMsg.EngineFailureCmd = 0;
    oMPC.VehicleStatusMsg.MissionFailure = 0;
    oMPC.VehicleLandDetectedMsg.Timestamp = 22718419546;
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000000000;
    oMPC.VehicleLandDetectedMsg.Landed = 0;
    oMPC.VehicleLandDetectedMsg.Freefall = 0;
    oMPC.VehicleLandDetectedMsg.GroundContact = 0;
    oMPC.ControlStateMsg.Timestamp = 22718419636;
    oMPC.ControlStateMsg.AccX = -0.178399145603;
    oMPC.ControlStateMsg.AccY = 0.143566161394;
    oMPC.ControlStateMsg.AccZ = -9.878698348999;
    oMPC.ControlStateMsg.VelX = 0.000000000000;
    oMPC.ControlStateMsg.VelY = 0.000000000000;
    oMPC.ControlStateMsg.VelZ = 0.000000000000;
    oMPC.ControlStateMsg.PosX = 0.000000000000;
    oMPC.ControlStateMsg.PosY = 0.000000000000;
    oMPC.ControlStateMsg.PosZ = 0.000000000000;
    oMPC.ControlStateMsg.Airspeed = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.Q[0] = 0.715221941471;
    oMPC.ControlStateMsg.Q[1] = 0.004807707854;
    oMPC.ControlStateMsg.Q[2] = -0.020368587226;
    oMPC.ControlStateMsg.Q[3] = 0.698583960533;
    oMPC.ControlStateMsg.DeltaQReset[0] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[1] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[2] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[3] = 0.000000000000;
    oMPC.ControlStateMsg.RollRate = 0.004654823337;
    oMPC.ControlStateMsg.PitchRate = 0.000240786583;
    oMPC.ControlStateMsg.YawRate = -0.001242246712;
    oMPC.ControlStateMsg.HorzAccMag = 0.000000000000;
    oMPC.ControlStateMsg.RollRateBias = 0.000000000000;
    oMPC.ControlStateMsg.PitchRateBias = 0.000000000000;
    oMPC.ControlStateMsg.YawRateBias = 0.000000000000;
    oMPC.ControlStateMsg.AirspeedValid = 1;
    oMPC.ControlStateMsg.QuatResetCounter = 1;
    oMPC.ManualControlSetpointMsg.Timestamp = 0;
    oMPC.ManualControlSetpointMsg.X = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Y = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Z = 0.000000000000;
    oMPC.ManualControlSetpointMsg.R = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Flaps = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux1 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux2 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux3 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux4 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux5 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.ModeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ReturnSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.RattitudeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.PosctlSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.LoiterSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.AcroSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.OffboardSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.KillSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ArmSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.TransitionSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.GearSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ModeSlot = (PX4_ModeSlot_t)0;
    oMPC.ManualControlSetpointMsg.DataSource = (PX4_ManualControlDataSource_t)0;
    oMPC.ManualControlSetpointMsg.StabSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ManSwitch = (PX4_SwitchPos_t)0;
    oMPC.PositionSetpointTripletMsg.Timestamp = 22718395507;
    oMPC.PositionSetpointTripletMsg.Previous.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Previous.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Current.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Current.Lat = 47.397741928975;
    oMPC.PositionSetpointTripletMsg.Current.Lon = 8.545593979817;
    oMPC.PositionSetpointTripletMsg.Current.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Alt = 490.751251220703;
    oMPC.PositionSetpointTripletMsg.Current.Yaw = 1.547718405724;
    oMPC.PositionSetpointTripletMsg.Current.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.LoiterRadius = 50.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingSpeed = -1.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingThrottle = -1.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 1;
    oMPC.PositionSetpointTripletMsg.Current.Type = (PX4_SetpointType_t)3;
    oMPC.PositionSetpointTripletMsg.Current.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawValid = 1;
    oMPC.PositionSetpointTripletMsg.Current.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.LoiterDirection = 1;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Next.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Next.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Next.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Next.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationIsForce = 0;
    oMPC.VehicleAttitudeSetpointMsg.Timestamp = 22718406228;
    oMPC.VehicleAttitudeSetpointMsg.RollBody = 0.00286550;
    oMPC.VehicleAttitudeSetpointMsg.PitchBody = 0.00668981;
    oMPC.VehicleAttitudeSetpointMsg.YawBody = 1.54771841;
    oMPC.VehicleAttitudeSetpointMsg.YawSpMoveRate = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[0] = 0.71521747;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[1] = -0.00131303;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[2] = 0.00339367;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[3] = 0.69889259;
    oMPC.VehicleAttitudeSetpointMsg.Thrust = 0.55761039;
    oMPC.VehicleAttitudeSetpointMsg.LandingGear = -1.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid = 1;
    oMPC.VehicleAttitudeSetpointMsg.RollResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.PitchResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.YawResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.FwControlYaw = 0;
    oMPC.VehicleAttitudeSetpointMsg.DisableMcYawControl = 0;
    oMPC.VehicleAttitudeSetpointMsg.ApplyFlaps = 0;
    oMPC.VehicleControlModeMsg.Timestamp = 22718352358;
    oMPC.VehicleControlModeMsg.Armed = 1;
    oMPC.VehicleControlModeMsg.ExternalManualOverrideOk = 0;
    oMPC.VehicleControlModeMsg.SystemHilEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlAutoEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlOffboardEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlRatesEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAttitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlRattitudeEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlForceEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlAccelerationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlTerminationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlFixedHdgEnabled = 0;
    oMPC.VehicleLocalPositionMsg.Timestamp = 22718420325;
    oMPC.VehicleLocalPositionMsg.RefTimestamp = 22718420325;
    oMPC.VehicleLocalPositionMsg.RefLat = 47.397741905620;
    oMPC.VehicleLocalPositionMsg.RefLon = 8.545593899402;
    oMPC.VehicleLocalPositionMsg.SurfaceBottomTimestamp = 22718420325;
    oMPC.VehicleLocalPositionMsg.X = 0.002443513833;
    oMPC.VehicleLocalPositionMsg.Y = 0.005807545502;
    oMPC.VehicleLocalPositionMsg.Z = 0.003159755142;
    oMPC.VehicleLocalPositionMsg.Delta_XY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_XY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_Z = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.VX = 0.012711226940;
    oMPC.VehicleLocalPositionMsg.VY = 0.027060005814;
    oMPC.VehicleLocalPositionMsg.VZ = -0.027509212494;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AX = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AY = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Yaw = 1.547648310661;
    oMPC.VehicleLocalPositionMsg.RefAlt = 488.253997802734;
    oMPC.VehicleLocalPositionMsg.DistBottom = 0.002536314772;
    oMPC.VehicleLocalPositionMsg.DistBottomRate = 0.027509212494;
    oMPC.VehicleLocalPositionMsg.EpH = 0.364578098059;
    oMPC.VehicleLocalPositionMsg.EpV = 0.234133124352;
    oMPC.VehicleLocalPositionMsg.EvV = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.EstimatorType = 0;
    oMPC.VehicleLocalPositionMsg.XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.Z_Valid = 1;
    oMPC.VehicleLocalPositionMsg.V_XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.V_Z_Valid = 1;
    oMPC.VehicleLocalPositionMsg.XY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.Z_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VXY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VZ_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.XY_Global = 1;
    oMPC.VehicleLocalPositionMsg.Z_Global = 1;
    oMPC.VehicleLocalPositionMsg.DistBottomValid = 1;
    oMPC.VehicleLocalPositionSetpointMsg.Timestamp = 22718406228;
    oMPC.VehicleLocalPositionSetpointMsg.X = 0.002596978797;
    oMPC.VehicleLocalPositionSetpointMsg.Y = 0.006052741315;
    oMPC.VehicleLocalPositionSetpointMsg.Z = -2.497253417969;
    oMPC.VehicleLocalPositionSetpointMsg.Yaw = 1.547718405724;
    oMPC.VehicleLocalPositionSetpointMsg.VX = 0.000100608748;
    oMPC.VehicleLocalPositionSetpointMsg.VY = 0.000175435256;
    oMPC.VehicleLocalPositionSetpointMsg.VZ = -0.123394995928;
    oMPC.VehicleLocalPositionSetpointMsg.AccX = -0.016529330984;
    oMPC.VehicleLocalPositionSetpointMsg.AccY = -0.036253951490;
    oMPC.VehicleLocalPositionSetpointMsg.AccZ = -5.474771976471;
    oMPC.HomePositionMsg.Timestamp = 22718352358;
    oMPC.HomePositionMsg.Lat = 47.397741928428;
    oMPC.HomePositionMsg.Lon = 8.545593976476;
    oMPC.HomePositionMsg.Alt = 488.252136230469;
    oMPC.HomePositionMsg.X = 0.002536065644;
    oMPC.HomePositionMsg.Y = 0.005801218562;
    oMPC.HomePositionMsg.Z = 0.001871463843;
    oMPC.HomePositionMsg.Yaw = 1.547679662704;
    oMPC.HomePositionMsg.DirectionX = 0.000000000000;
    oMPC.HomePositionMsg.DirectionY = 0.000000000000;
    oMPC.HomePositionMsg.DirectionZ = 0.000000000000;
    oMPC.GearStateInitialized = 0;
    oMPC.RefAlt = 488.253997802734;
    oMPC.RefPos.timestamp = 22718420398;
    oMPC.RefPos.lat_rad = 0.827246654264;
    oMPC.RefPos.lon_rad = 0.149148750083;
    oMPC.RefPos.sin_lat = 0.736070410066;
    oMPC.RefPos.cos_lat = 0.676904979613;
    oMPC.RefPos.init_done = 1;
    oMPC.RefTimestamp = 22718420325;
    oMPC.ResetPositionSetpoint = 0;
    oMPC.ResetAltitudeSetpoint = 0;
    oMPC.DoResetAltPos = 1;
    oMPC.ModeAuto = 1;
    oMPC.PositionHoldEngaged = 0;
    oMPC.AltitudeHoldEngaged = 0;
    oMPC.RunPosControl = 1;
    oMPC.RunAltControl = 1;
    oMPC.ResetIntZ = 0;
    oMPC.ResetIntXY = 0;
    oMPC.ResetIntZManual = 0;
    oMPC.ResetYawSetpoint = 1;
    oMPC.HoldOffboardXY = 0;
    oMPC.HoldOffboardZ = 0;
    oMPC.LimitVelXY = 1;
    oMPC.ThrustInt[0] = -0.000066571869;
    oMPC.ThrustInt[1] = -0.000139073905;
    oMPC.ThrustInt[2] = -0.000246876385;
    oMPC.Position[0] = 0.002443513833;
    oMPC.Position[1] = 0.005807545502;
    oMPC.Position[2] = 0.003159755142;
    oMPC.PositionSetpoint[0] = 0.002596978797;
    oMPC.PositionSetpoint[1] = 0.006052741315;
    oMPC.PositionSetpoint[2] = -2.497253417969;
    oMPC.Velocity[0] = 0.012711226940;
    oMPC.Velocity[1] = 0.027060005814;
    oMPC.Velocity[2] = -0.027509212494;
    oMPC.VelocitySetpoint[0] = 0.000100608748;
    oMPC.VelocitySetpoint[1] = 0.000175435256;
    oMPC.VelocitySetpoint[2] = -0.123394995928;
    oMPC.VelocityPrevious[0] = 0.013109490275;
    oMPC.VelocityPrevious[1] = 0.027552505955;
    oMPC.VelocityPrevious[2] = -0.026428043842;
    oMPC.VelocityFF[0] = 0.000000000000;
    oMPC.VelocityFF[1] = 0.000000000000;
    oMPC.VelocityFF[2] = 0.000000000000;
    oMPC.VelocitySetpointPrevious[0] = 0.000100608748;
    oMPC.VelocitySetpointPrevious[1] = 0.000175435256;
    oMPC.VelocitySetpointPrevious[2] = -0.123394995928;
    oMPC.VelocityErrD[0] = 0.029724396765;
    oMPC.VelocityErrD[1] = 0.044614251703;
    oMPC.VelocityErrD[2] = 0.014642260969;
    oMPC.CurrentPositionSetpoint[0] = 0.002596978797;
    oMPC.CurrentPositionSetpoint[1] = 0.006052741315;
    oMPC.CurrentPositionSetpoint[2] = -2.497253417969;
    oMPC.Rotation[0][0] = 0.023131102324;
    oMPC.Rotation[0][1] = -0.999481022358;
    oMPC.Rotation[0][2] = -0.022418946028;
    oMPC.Rotation[1][0] = 0.999089300632;
    oMPC.Rotation[1][1] = 0.023914664984;
    oMPC.Rotation[1][2] = -0.035335492343;
    oMPC.Rotation[2][0] = 0.035853296518;
    oMPC.Rotation[2][1] = -0.021581180394;
    oMPC.Rotation[2][2] = 0.999123930931;
    oMPC.Yaw = 1.547648310661;
    oMPC.YawTakeoff = 1.547740340233;
    oMPC.InLanding = 0;
    oMPC.LndReachedGround = 0;
    oMPC.VelZLp = -0.034922800958;
    oMPC.AccZLp = -0.017607331276;
    oMPC.VelMaxXY = 8.000000000000;
    oMPC.InTakeoff = 1;
    oMPC.TakeoffVelLimit = -0.500000000000;
    oMPC.Z_ResetCounter = 0;
    oMPC.XY_ResetCounter = 0;
    oMPC.VZ_ResetCounter = 0;
    oMPC.VXY_ResetCounter = 0;
    oMPC.HeadingResetCounter = 0;
    oMPC.RSetpoint[0][0] = 0.023075357080;
    oMPC.RSetpoint[0][1] = -0.999729216099;
    oMPC.RSetpoint[0][2] = 0.003019101685;
    oMPC.RSetpoint[1][0] = 0.999711394310;
    oMPC.RSetpoint[1][1] = 0.023094942793;
    oMPC.RSetpoint[1][2] = 0.006621827371;
    oMPC.RSetpoint[2][0] = -0.006689760368;
    oMPC.RSetpoint[2][1] = 0.002865429502;
    oMPC.RSetpoint[2][2] = 0.999973535538;

    Ut_MPC_Custom_SetReturnCode(UT_MPC_PX4LIB_GETPX4TIMEUS_INDEX, 1, 1);
    oMPC.ControlAuto(0.014178999700);

    UtAssert_True(oMPC.ResetIntXY == false, "oMPC.ResetIntXY");
    UtAssert_True(oMPC.ResetIntZ == false, "oMPC.ResetIntZ");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[0], -0.000066571869, FLT_EPSILON, "oMPC.ThrustInt[0]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[1], -0.000139073905, FLT_EPSILON, "oMPC.ThrustInt[1]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[2], -0.000246876385, FLT_EPSILON, "oMPC.ThrustInt[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0],  0.000100608748, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1],  0.000175435256, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2], -0.123394995928, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[0], -0.0382288098, FLT_EPSILON, "oMPC.PositionSetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[1],  0.00106973771, FLT_EPSILON, "oMPC.PositionSetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2], -2.49725342, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelZLp, -0.034922800958, FLT_EPSILON, "oMPC.VelZLp");
    UtAssert_DoubleCmpAbs(oMPC.AccZLp, -0.017607331276, FLT_EPSILON, "oMPC.AccZLp");
    UtAssert_True(oMPC.InLanding == false, "oMPC.InLanding");
    UtAssert_True(oMPC.LndReachedGround == false, "oMPC.LndReachedGround");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Thrust, 0.557610392570, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Thrust");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][0],  0.023075357080, FLT_EPSILON, "oMPC.RSetpoint[0][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][1], -0.999729216099, FLT_EPSILON, "oMPC.RSetpoint[0][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][2],  0.003019101685, FLT_EPSILON, "oMPC.RSetpoint[0][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][0],  0.999711394310, FLT_EPSILON, "oMPC.RSetpoint[1][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][1],  0.023094942793, FLT_EPSILON, "oMPC.RSetpoint[1][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][2],  0.006621827371, FLT_EPSILON, "oMPC.RSetpoint[1][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][0], -0.006689760368, FLT_EPSILON, "oMPC.RSetpoint[2][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][1],  0.002865429502, FLT_EPSILON, "oMPC.RSetpoint[2][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][2],  0.999973535538, FLT_EPSILON, "oMPC.RSetpoint[2][2]");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid == true, "oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.RollBody, 0.002865497489, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.RollBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.PitchBody, 0.006689810194, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.PitchBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.YawBody, 1.547718405724, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.YawBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccX, -0.016529330984, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccY, -0.036253951490, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccZ, -5.474771976471, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccZ");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Timestamp != 0, "oMPC.VehicleAttitudeSetpointMsg.Timestamp");
}



/**
 * Test MPC_AppMain(), Nominal - ControlNonManual
 */
void Test_MPC_AppMain_Nominal_ControlNonManual(void)
{
    MPC oMPC;

    oMPC.InitData();
    oMPC.InitConfigTbl();

    /* PASS 1 - Initializing. */
    oMPC.VehicleStatusMsg.Timestamp = 32043154862;
    oMPC.VehicleStatusMsg.SystemID = 1;
    oMPC.VehicleStatusMsg.ComponentID = 1;
    oMPC.VehicleStatusMsg.OnboardControlSensorsPresent = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsEnabled = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsHealth = 0;
    oMPC.VehicleStatusMsg.NavState = (PX4_NavigationState_t)4;
    oMPC.VehicleStatusMsg.ArmingState = (PX4_ArmingState_t)1;
    oMPC.VehicleStatusMsg.HilState = (PX4_HilState_t)0;
    oMPC.VehicleStatusMsg.Failsafe = 0;
    oMPC.VehicleStatusMsg.SystemType = (PX4_SystemType_t)13;
    oMPC.VehicleStatusMsg.IsRotaryWing = 1;
    oMPC.VehicleStatusMsg.IsVtol = 0;
    oMPC.VehicleStatusMsg.VtolFwPermanentStab = 0;
    oMPC.VehicleStatusMsg.InTransitionMode = 0;
    oMPC.VehicleStatusMsg.RcInputMode = (PX4_RcInMode_t)1;
    oMPC.VehicleStatusMsg.DataLinkLost = 1;
    oMPC.VehicleStatusMsg.DataLinkLostCounter = 0;
    oMPC.VehicleStatusMsg.EngineFailure = 0;
    oMPC.VehicleStatusMsg.EngineFailureCmd = 0;
    oMPC.VehicleStatusMsg.MissionFailure = 0;
    oMPC.VehicleLandDetectedMsg.Timestamp = 32041122164;
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000000000;
    oMPC.VehicleLandDetectedMsg.Landed = 1;
    oMPC.VehicleLandDetectedMsg.Freefall = 0;
    oMPC.VehicleLandDetectedMsg.GroundContact = 0;
    oMPC.ControlStateMsg.Timestamp = 32043154215;
    oMPC.ControlStateMsg.AccX = -0.234742432833;
    oMPC.ControlStateMsg.AccY = 0.068961113691;
    oMPC.ControlStateMsg.AccZ = -10.363160133362;
    oMPC.ControlStateMsg.VelX = 0.000000000000;
    oMPC.ControlStateMsg.VelY = 0.000000000000;
    oMPC.ControlStateMsg.VelZ = 0.000000000000;
    oMPC.ControlStateMsg.PosX = 0.000000000000;
    oMPC.ControlStateMsg.PosY = 0.000000000000;
    oMPC.ControlStateMsg.PosZ = 0.000000000000;
    oMPC.ControlStateMsg.Airspeed = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.Q[0] = 0.689731538296;
    oMPC.ControlStateMsg.Q[1] = 0.006344274618;
    oMPC.ControlStateMsg.Q[2] = -0.012429238297;
    oMPC.ControlStateMsg.Q[3] = 0.723930716515;
    oMPC.ControlStateMsg.DeltaQReset[0] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[1] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[2] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[3] = 0.000000000000;
    oMPC.ControlStateMsg.RollRate = 0.003042250173;
    oMPC.ControlStateMsg.PitchRate = -0.004442838021;
    oMPC.ControlStateMsg.YawRate = -0.004229306709;
    oMPC.ControlStateMsg.HorzAccMag = 0.000000000000;
    oMPC.ControlStateMsg.RollRateBias = 0.000000000000;
    oMPC.ControlStateMsg.PitchRateBias = 0.000000000000;
    oMPC.ControlStateMsg.YawRateBias = 0.000000000000;
    oMPC.ControlStateMsg.AirspeedValid = 1;
    oMPC.ControlStateMsg.QuatResetCounter = 1;
    oMPC.ManualControlSetpointMsg.Timestamp = 0;
    oMPC.ManualControlSetpointMsg.X = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Y = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Z = 0.000000000000;
    oMPC.ManualControlSetpointMsg.R = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Flaps = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux1 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux2 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux3 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux4 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux5 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.ModeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ReturnSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.RattitudeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.PosctlSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.LoiterSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.AcroSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.OffboardSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.KillSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ArmSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.TransitionSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.GearSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ModeSlot = (PX4_ModeSlot_t)0;
    oMPC.ManualControlSetpointMsg.DataSource = (PX4_ManualControlDataSource_t)0;
    oMPC.ManualControlSetpointMsg.StabSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ManSwitch = (PX4_SwitchPos_t)0;
    oMPC.PositionSetpointTripletMsg.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Previous.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Current.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Current.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Current.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Current.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Next.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Next.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Next.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Next.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationIsForce = 0;
    oMPC.VehicleAttitudeSetpointMsg.Timestamp = 32043147103;
    oMPC.VehicleAttitudeSetpointMsg.RollBody = -0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.PitchBody = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.YawBody = 1.58806074;
    oMPC.VehicleAttitudeSetpointMsg.YawSpMoveRate = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[0] = 0.70097661;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[1] = -0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[2] = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[3] = 0.71318424;
    oMPC.VehicleAttitudeSetpointMsg.Thrust = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.LandingGear = -1.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid = 1;
    oMPC.VehicleAttitudeSetpointMsg.RollResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.PitchResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.YawResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.FwControlYaw = 0;
    oMPC.VehicleAttitudeSetpointMsg.DisableMcYawControl = 0;
    oMPC.VehicleAttitudeSetpointMsg.ApplyFlaps = 0;
    oMPC.VehicleControlModeMsg.Timestamp = 32043154862;
    oMPC.VehicleControlModeMsg.Armed = 0;
    oMPC.VehicleControlModeMsg.ExternalManualOverrideOk = 0;
    oMPC.VehicleControlModeMsg.SystemHilEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlAutoEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlOffboardEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlRatesEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAttitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlRattitudeEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlForceEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlAccelerationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlTerminationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlFixedHdgEnabled = 0;
    oMPC.VehicleLocalPositionMsg.Timestamp = 32043159089;
    oMPC.VehicleLocalPositionMsg.RefTimestamp = 32043159089;
    oMPC.VehicleLocalPositionMsg.RefLat = 47.397743297475;
    oMPC.VehicleLocalPositionMsg.RefLon = 8.545593786945;
    oMPC.VehicleLocalPositionMsg.SurfaceBottomTimestamp = 32043159089;
    oMPC.VehicleLocalPositionMsg.X = 0.000201690331;
    oMPC.VehicleLocalPositionMsg.Y = 0.000447960949;
    oMPC.VehicleLocalPositionMsg.Z = -0.000000001442;
    oMPC.VehicleLocalPositionMsg.Delta_XY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_XY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_Z = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.VX = 0.001371126156;
    oMPC.VehicleLocalPositionMsg.VY = 0.009506213479;
    oMPC.VehicleLocalPositionMsg.VZ = -0.488032341003;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AX = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AY = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Yaw = 1.619292497635;
    oMPC.VehicleLocalPositionMsg.RefAlt = 488.269012451172;
    oMPC.VehicleLocalPositionMsg.DistBottom = -0.000000000128;
    oMPC.VehicleLocalPositionMsg.DistBottomRate = 0.488032341003;
    oMPC.VehicleLocalPositionMsg.EpH = 3.000000000000;
    oMPC.VehicleLocalPositionMsg.EpV = 2.000424146652;
    oMPC.VehicleLocalPositionMsg.EvV = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.EstimatorType = 0;
    oMPC.VehicleLocalPositionMsg.XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.Z_Valid = 0;
    oMPC.VehicleLocalPositionMsg.V_XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.V_Z_Valid = 0;
    oMPC.VehicleLocalPositionMsg.XY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.Z_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VXY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VZ_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.XY_Global = 1;
    oMPC.VehicleLocalPositionMsg.Z_Global = 0;
    oMPC.VehicleLocalPositionMsg.DistBottomValid = 0;
    oMPC.VehicleLocalPositionSetpointMsg.Timestamp = 0;
    oMPC.VehicleLocalPositionSetpointMsg.X = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.Y = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.Z = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.Yaw = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.VX = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.VY = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.VZ = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccX = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccY = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccZ = 0.000000000000;
    oMPC.HomePositionMsg.Timestamp = 32043143528;
    oMPC.HomePositionMsg.Lat = 47.397743299551;
    oMPC.HomePositionMsg.Lon = 8.545593796525;
    oMPC.HomePositionMsg.Alt = 488.269012451172;
    oMPC.HomePositionMsg.X = 0.000230828955;
    oMPC.HomePositionMsg.Y = 0.000721091288;
    oMPC.HomePositionMsg.Z = -0.000000001134;
    oMPC.HomePositionMsg.Yaw = 1.619436144829;
    oMPC.HomePositionMsg.DirectionX = 0.000000000000;
    oMPC.HomePositionMsg.DirectionY = 0.000000000000;
    oMPC.HomePositionMsg.DirectionZ = 0.000000000000;
    oMPC.GearStateInitialized = 0;
    oMPC.RefAlt = 488.269012451172;
    oMPC.RefPos.timestamp = 32043159139;
    oMPC.RefPos.lat_rad = 0.827246678556;
    oMPC.RefPos.lon_rad = 0.149148748120;
    oMPC.RefPos.sin_lat = 0.736070426510;
    oMPC.RefPos.cos_lat = 0.676904961732;
    oMPC.RefPos.init_done = 1;
    oMPC.RefTimestamp = 32043159089;
    oMPC.ResetPositionSetpoint = 1;
    oMPC.ResetAltitudeSetpoint = 1;
    oMPC.DoResetAltPos = 1;
    oMPC.ModeAuto = 0;
    oMPC.PositionHoldEngaged = 0;
    oMPC.AltitudeHoldEngaged = 0;
    oMPC.RunPosControl = 1;
    oMPC.RunAltControl = 1;
    oMPC.ResetIntZ = 1;
    oMPC.ResetIntXY = 1;
    oMPC.ResetIntZManual = 0;
    oMPC.ResetYawSetpoint = 0;
    oMPC.HoldOffboardXY = 0;
    oMPC.HoldOffboardZ = 0;
    oMPC.LimitVelXY = 0;
    oMPC.ThrustInt[0] = 0.000000000000;
    oMPC.ThrustInt[1] = 0.000000000000;
    oMPC.ThrustInt[2] = 0.000000000000;
    oMPC.Position[0] = 0.000201690331;
    oMPC.Position[1] = 0.000447960949;
    oMPC.Position[2] = -0.000000001442;
    oMPC.PositionSetpoint[0] = 0.000000000000;
    oMPC.PositionSetpoint[1] = 0.000000000000;
    oMPC.PositionSetpoint[2] = 0.000000000000;
    oMPC.Velocity[0] = 0.001371126156;
    oMPC.Velocity[1] = 0.009506213479;
    oMPC.Velocity[2] = -0.488032341003;
    oMPC.VelocitySetpoint[0] = 0.000000000000;
    oMPC.VelocitySetpoint[1] = 0.000000000000;
    oMPC.VelocitySetpoint[2] = 0.000000000000;
    oMPC.VelocityPrevious[0] = 0.001327236881;
    oMPC.VelocityPrevious[1] = 0.009677568451;
    oMPC.VelocityPrevious[2] = -0.482442229986;
    oMPC.VelocityFF[0] = 0.000000000000;
    oMPC.VelocityFF[1] = 0.000000000000;
    oMPC.VelocityFF[2] = 0.000000000000;
    oMPC.VelocitySetpointPrevious[0] = 0.001327236881;
    oMPC.VelocitySetpointPrevious[1] = 0.009677568451;
    oMPC.VelocitySetpointPrevious[2] = -0.482442229986;
    oMPC.VelocityErrD[0] = -0.001915354049;
    oMPC.VelocityErrD[1] = 0.015473826788;
    oMPC.VelocityErrD[2] = 0.239046335220;
    oMPC.CurrentPositionSetpoint[0] = 0.000000000000;
    oMPC.CurrentPositionSetpoint[1] = 0.000000000000;
    oMPC.CurrentPositionSetpoint[2] = 0.000000000000;
    oMPC.Rotation[0][0] = -0.048460334539;
    oMPC.Rotation[0][1] = -0.998793423176;
    oMPC.Rotation[0][2] = -0.007960043848;
    oMPC.Rotation[1][0] = 0.998477995396;
    oMPC.Rotation[1][1] = -0.048231869936;
    oMPC.Rotation[1][2] = -0.026747506112;
    oMPC.Rotation[2][0] = 0.026331305504;
    oMPC.Rotation[2][1] = -0.009244121611;
    oMPC.Rotation[2][2] = 0.999610543251;
    oMPC.Yaw = 1.619292497635;
    oMPC.YawTakeoff = 0.000000000000;
    oMPC.InLanding = 0;
    oMPC.LndReachedGround = 0;
    oMPC.VelZLp = 0.000000000000;
    oMPC.AccZLp = 0.000000000000;
    oMPC.VelMaxXY = 8.000000000000;
    oMPC.InTakeoff = 0;
    oMPC.TakeoffVelLimit = 0.000000000000;
    oMPC.Z_ResetCounter = 0;
    oMPC.XY_ResetCounter = 0;
    oMPC.VZ_ResetCounter = 0;
    oMPC.VXY_ResetCounter = 0;
    oMPC.HeadingResetCounter = 0;
    oMPC.RSetpoint[0][0] = 1.000000000000;
    oMPC.RSetpoint[0][1] = 0.000000000000;
    oMPC.RSetpoint[0][2] = 0.000000000000;
    oMPC.RSetpoint[1][0] = 0.000000000000;
    oMPC.RSetpoint[1][1] = 1.000000000000;
    oMPC.RSetpoint[1][2] = 0.000000000000;
    oMPC.RSetpoint[2][0] = 0.000000000000;
    oMPC.RSetpoint[2][1] = 0.000000000000;
    oMPC.RSetpoint[2][2] = 1.000000000000;

    oMPC.ControlNonManual(0.012037999928);

    UtAssert_True(oMPC.ResetIntXY == false, "oMPC.ResetIntXY");
    UtAssert_True(oMPC.ResetIntZ == false, "oMPC.ResetIntZ");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[0], 0.000000000000, FLT_EPSILON, "oMPC.ThrustInt[0]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[1], 0.000000000000, FLT_EPSILON, "oMPC.ThrustInt[1]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[2], 0.000118779142, FLT_EPSILON, "oMPC.ThrustInt[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0],  0.000000000000, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1],  0.000000000000, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2], -0.422252237797, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[0],  0.000201690331, FLT_EPSILON, "oMPC.PositionSetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[1],  0.000447960949, FLT_EPSILON, "oMPC.PositionSetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2], -0.000000001442, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelZLp, -0.046999465674, FLT_EPSILON, "oMPC.VelZLp");
    UtAssert_DoubleCmpAbs(oMPC.AccZLp, -0.044720888138, FLT_EPSILON, "oMPC.AccZLp");
    UtAssert_True(oMPC.InLanding == false, "oMPC.InLanding");
    UtAssert_True(oMPC.LndReachedGround == false, "oMPC.LndReachedGround");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Thrust, 0.119999997318, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Thrust");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][0], -0.017263552174, FLT_EPSILON, "oMPC.RSetpoint[0][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][1], -0.999850988388, FLT_EPSILON, "oMPC.RSetpoint[0][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][2],  0.000000000000, FLT_EPSILON, "oMPC.RSetpoint[0][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][0],  0.999850988388, FLT_EPSILON, "oMPC.RSetpoint[1][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][1], -0.017263552174, FLT_EPSILON, "oMPC.RSetpoint[1][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][2],  0.000000000000, FLT_EPSILON, "oMPC.RSetpoint[1][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][0],  0.000000000000, FLT_EPSILON, "oMPC.RSetpoint[2][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][1],  0.000000000000, FLT_EPSILON, "oMPC.RSetpoint[2][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][2],  1.000000000000, FLT_EPSILON, "oMPC.RSetpoint[2][2]");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid == true, "oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.RollBody, 0.000000000000, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.RollBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.PitchBody, -0.000000000000, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.PitchBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.YawBody, 1.588060736656, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.YawBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccX,  0.000000000000, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccY,  0.000000000000, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccZ, -0.000000000000, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccZ");
    //UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Timestamp != 0, "oMPC.VehicleAttitudeSetpointMsg.Timestamp");

    /* PASS 2 - Transition to takeoff. */
    oMPC.VehicleStatusMsg.Timestamp = 33671229639;
    oMPC.VehicleStatusMsg.SystemID = 1;
    oMPC.VehicleStatusMsg.ComponentID = 1;
    oMPC.VehicleStatusMsg.OnboardControlSensorsPresent = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsEnabled = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsHealth = 0;
    oMPC.VehicleStatusMsg.NavState = (PX4_NavigationState_t)17;
    oMPC.VehicleStatusMsg.ArmingState = (PX4_ArmingState_t)2;
    oMPC.VehicleStatusMsg.HilState = (PX4_HilState_t)0;
    oMPC.VehicleStatusMsg.Failsafe = 0;
    oMPC.VehicleStatusMsg.SystemType = (PX4_SystemType_t)13;
    oMPC.VehicleStatusMsg.IsRotaryWing = 1;
    oMPC.VehicleStatusMsg.IsVtol = 0;
    oMPC.VehicleStatusMsg.VtolFwPermanentStab = 0;
    oMPC.VehicleStatusMsg.InTransitionMode = 0;
    oMPC.VehicleStatusMsg.RcInputMode = (PX4_RcInMode_t)1;
    oMPC.VehicleStatusMsg.DataLinkLost = 1;
    oMPC.VehicleStatusMsg.DataLinkLostCounter = 0;
    oMPC.VehicleStatusMsg.EngineFailure = 0;
    oMPC.VehicleStatusMsg.EngineFailureCmd = 0;
    oMPC.VehicleStatusMsg.MissionFailure = 0;
    oMPC.VehicleLandDetectedMsg.Timestamp = 33671297774;
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000000000;
    oMPC.VehicleLandDetectedMsg.Landed = 0;
    oMPC.VehicleLandDetectedMsg.Freefall = 0;
    oMPC.VehicleLandDetectedMsg.GroundContact = 0;
    oMPC.ControlStateMsg.Timestamp = 33671300139;
    oMPC.ControlStateMsg.AccX = -0.023082222790;
    oMPC.ControlStateMsg.AccY = 0.250855356455;
    oMPC.ControlStateMsg.AccZ = -10.073497772217;
    oMPC.ControlStateMsg.VelX = 0.000000000000;
    oMPC.ControlStateMsg.VelY = 0.000000000000;
    oMPC.ControlStateMsg.VelZ = 0.000000000000;
    oMPC.ControlStateMsg.PosX = 0.000000000000;
    oMPC.ControlStateMsg.PosY = 0.000000000000;
    oMPC.ControlStateMsg.PosZ = 0.000000000000;
    oMPC.ControlStateMsg.Airspeed = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.Q[0] = 0.617908656597;
    oMPC.ControlStateMsg.Q[1] = -0.000546063937;
    oMPC.ControlStateMsg.Q[2] = -0.016150537878;
    oMPC.ControlStateMsg.Q[3] = 0.786083877087;
    oMPC.ControlStateMsg.DeltaQReset[0] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[1] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[2] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[3] = 0.000000000000;
    oMPC.ControlStateMsg.RollRate = -0.005168430042;
    oMPC.ControlStateMsg.PitchRate = -0.006302529015;
    oMPC.ControlStateMsg.YawRate = -0.001021062024;
    oMPC.ControlStateMsg.HorzAccMag = 0.000000000000;
    oMPC.ControlStateMsg.RollRateBias = 0.000000000000;
    oMPC.ControlStateMsg.PitchRateBias = 0.000000000000;
    oMPC.ControlStateMsg.YawRateBias = 0.000000000000;
    oMPC.ControlStateMsg.AirspeedValid = 1;
    oMPC.ControlStateMsg.QuatResetCounter = 1;
    oMPC.ManualControlSetpointMsg.Timestamp = 0;
    oMPC.ManualControlSetpointMsg.X = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Y = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Z = 0.000000000000;
    oMPC.ManualControlSetpointMsg.R = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Flaps = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux1 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux2 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux3 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux4 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux5 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.ModeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ReturnSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.RattitudeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.PosctlSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.LoiterSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.AcroSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.OffboardSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.KillSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ArmSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.TransitionSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.GearSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ModeSlot = (PX4_ModeSlot_t)0;
    oMPC.ManualControlSetpointMsg.DataSource = (PX4_ManualControlDataSource_t)0;
    oMPC.ManualControlSetpointMsg.StabSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ManSwitch = (PX4_SwitchPos_t)0;
    oMPC.PositionSetpointTripletMsg.Timestamp = 33671277064;
    oMPC.PositionSetpointTripletMsg.Previous.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Previous.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Current.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Current.Lat = 47.397743504482;
    oMPC.PositionSetpointTripletMsg.Current.Lon = 8.545593681335;
    oMPC.PositionSetpointTripletMsg.Current.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Alt = 490.789276123047;
    oMPC.PositionSetpointTripletMsg.Current.Yaw = 1.809637427330;
    oMPC.PositionSetpointTripletMsg.Current.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.LoiterRadius = 50.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingSpeed = -1.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingThrottle = -1.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 1;
    oMPC.PositionSetpointTripletMsg.Current.Type = (PX4_SetpointType_t)3;
    oMPC.PositionSetpointTripletMsg.Current.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawValid = 1;
    oMPC.PositionSetpointTripletMsg.Current.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.LoiterDirection = 1;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Next.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Next.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Next.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Next.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationIsForce = 0;
    oMPC.VehicleAttitudeSetpointMsg.Timestamp = 33671288749;
    oMPC.VehicleAttitudeSetpointMsg.RollBody = 0.00116860;
    oMPC.VehicleAttitudeSetpointMsg.PitchBody = 0.01192364;
    oMPC.VehicleAttitudeSetpointMsg.YawBody = 1.80963743;
    oMPC.VehicleAttitudeSetpointMsg.YawSpMoveRate = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[0] = 0.61781979;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[1] = -0.00432684;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[2] = 0.00414279;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[3] = 0.78629696;
    oMPC.VehicleAttitudeSetpointMsg.Thrust = 0.43548700;
    oMPC.VehicleAttitudeSetpointMsg.LandingGear = -1.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid = 1;
    oMPC.VehicleAttitudeSetpointMsg.RollResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.PitchResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.YawResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.FwControlYaw = 0;
    oMPC.VehicleAttitudeSetpointMsg.DisableMcYawControl = 0;
    oMPC.VehicleAttitudeSetpointMsg.ApplyFlaps = 0;
    oMPC.VehicleControlModeMsg.Timestamp = 33671229639;
    oMPC.VehicleControlModeMsg.Armed = 1;
    oMPC.VehicleControlModeMsg.ExternalManualOverrideOk = 0;
    oMPC.VehicleControlModeMsg.SystemHilEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlAutoEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlOffboardEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlRatesEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAttitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlRattitudeEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlForceEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlAccelerationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlTerminationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlFixedHdgEnabled = 0;
    oMPC.VehicleLocalPositionMsg.Timestamp = 33671300317;
    oMPC.VehicleLocalPositionMsg.RefTimestamp = 33671300317;
    oMPC.VehicleLocalPositionMsg.RefLat = 47.397743509334;
    oMPC.VehicleLocalPositionMsg.RefLon = 8.545593597490;
    oMPC.VehicleLocalPositionMsg.SurfaceBottomTimestamp = 33671300317;
    oMPC.VehicleLocalPositionMsg.X = -0.000593128381;
    oMPC.VehicleLocalPositionMsg.Y = 0.006940861698;
    oMPC.VehicleLocalPositionMsg.Z = -0.023656614125;
    oMPC.VehicleLocalPositionMsg.Delta_XY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_XY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_Z = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.VX = -0.003156758146;
    oMPC.VehicleLocalPositionMsg.VY = 0.031119752675;
    oMPC.VehicleLocalPositionMsg.VZ = -0.166829451919;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AX = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AY = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Yaw = 1.809476375580;
    oMPC.VehicleLocalPositionMsg.RefAlt = 488.269012451172;
    oMPC.VehicleLocalPositionMsg.DistBottom = 0.024133512750;
    oMPC.VehicleLocalPositionMsg.DistBottomRate = 0.166829451919;
    oMPC.VehicleLocalPositionMsg.EpH = 0.411934226751;
    oMPC.VehicleLocalPositionMsg.EpV = 0.275059789419;
    oMPC.VehicleLocalPositionMsg.EvV = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.EstimatorType = 0;
    oMPC.VehicleLocalPositionMsg.XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.Z_Valid = 1;
    oMPC.VehicleLocalPositionMsg.V_XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.V_Z_Valid = 1;
    oMPC.VehicleLocalPositionMsg.XY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.Z_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VXY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VZ_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.XY_Global = 1;
    oMPC.VehicleLocalPositionMsg.Z_Global = 1;
    oMPC.VehicleLocalPositionMsg.DistBottomValid = 1;
    oMPC.VehicleLocalPositionSetpointMsg.Timestamp = 33671288750;
    oMPC.VehicleLocalPositionSetpointMsg.X = -0.000539467554;
    oMPC.VehicleLocalPositionSetpointMsg.Y = 0.006310853176;
    oMPC.VehicleLocalPositionSetpointMsg.Z = -2.520263671875;
    oMPC.VehicleLocalPositionSetpointMsg.Yaw = 1.809637427330;
    oMPC.VehicleLocalPositionSetpointMsg.VX = 0.000020811351;
    oMPC.VehicleLocalPositionSetpointMsg.VY = -0.000289084419;
    oMPC.VehicleLocalPositionSetpointMsg.VZ = -0.058515001088;
    oMPC.VehicleLocalPositionSetpointMsg.AccX = 0.007203754503;
    oMPC.VehicleLocalPositionSetpointMsg.AccY = -0.050698895007;
    oMPC.VehicleLocalPositionSetpointMsg.AccZ = -4.273990154266;
    oMPC.HomePositionMsg.Timestamp = 33671219483;
    oMPC.HomePositionMsg.Lat = 47.397743504982;
    oMPC.HomePositionMsg.Lon = 8.545593667673;
    oMPC.HomePositionMsg.Alt = 488.283905029297;
    oMPC.HomePositionMsg.X = -0.000483906595;
    oMPC.HomePositionMsg.Y = 0.005282537080;
    oMPC.HomePositionMsg.Z = -0.014878900722;
    oMPC.HomePositionMsg.Yaw = 1.809811234474;
    oMPC.HomePositionMsg.DirectionX = 0.000000000000;
    oMPC.HomePositionMsg.DirectionY = 0.000000000000;
    oMPC.HomePositionMsg.DirectionZ = 0.000000000000;
    oMPC.GearStateInitialized = 0;
    oMPC.RefAlt = 488.269012451172;
    oMPC.RefPos.timestamp = 33671300423;
    oMPC.RefPos.lat_rad = 0.827246682254;
    oMPC.RefPos.lon_rad = 0.149148744814;
    oMPC.RefPos.sin_lat = 0.736070429013;
    oMPC.RefPos.cos_lat = 0.676904959010;
    oMPC.RefPos.init_done = 1;
    oMPC.RefTimestamp = 33671300317;
    oMPC.ResetPositionSetpoint = 0;
    oMPC.ResetAltitudeSetpoint = 0;
    oMPC.DoResetAltPos = 1;
    oMPC.ModeAuto = 1;
    oMPC.PositionHoldEngaged = 0;
    oMPC.AltitudeHoldEngaged = 0;
    oMPC.RunPosControl = 1;
    oMPC.RunAltControl = 1;
    oMPC.ResetIntZ = 0;
    oMPC.ResetIntXY = 0;
    oMPC.ResetIntZManual = 0;
    oMPC.ResetYawSetpoint = 1;
    oMPC.HoldOffboardXY = 0;
    oMPC.HoldOffboardZ = 0;
    oMPC.LimitVelXY = 1;
    oMPC.ThrustInt[0] = 0.000005650265;
    oMPC.ThrustInt[1] = -0.000070955997;
    oMPC.ThrustInt[2] = 0.000187757949;
    oMPC.Position[0] = -0.000593128381;
    oMPC.Position[1] = 0.006940861698;
    oMPC.Position[2] = -0.023656614125;
    oMPC.PositionSetpoint[0] = -0.000539467554;
    oMPC.PositionSetpoint[1] = 0.006310853176;
    oMPC.PositionSetpoint[2] = -2.520263671875;
    oMPC.Velocity[0] = -0.003156758146;
    oMPC.Velocity[1] = 0.031119752675;
    oMPC.Velocity[2] = -0.166829451919;
    oMPC.VelocitySetpoint[0] = 0.000020811351;
    oMPC.VelocitySetpoint[1] = -0.000289084419;
    oMPC.VelocitySetpoint[2] = -0.058515001088;
    oMPC.VelocityPrevious[0] = -0.002393212635;
    oMPC.VelocityPrevious[1] = 0.030026217923;
    oMPC.VelocityPrevious[2] = -0.165472149849;
    oMPC.VelocityFF[0] = 0.000000000000;
    oMPC.VelocityFF[1] = 0.000000000000;
    oMPC.VelocityFF[2] = 0.000000000000;
    oMPC.VelocitySetpointPrevious[0] = 0.000020811351;
    oMPC.VelocitySetpointPrevious[1] = -0.000289084419;
    oMPC.VelocitySetpointPrevious[2] = -0.058515001088;
    oMPC.VelocityErrD[0] = 0.044798217714;
    oMPC.VelocityErrD[1] = -0.070677615702;
    oMPC.VelocityErrD[2] = 0.132893085480;
    oMPC.CurrentPositionSetpoint[0] = -0.000539467554;
    oMPC.CurrentPositionSetpoint[1] = 0.006310853176;
    oMPC.CurrentPositionSetpoint[2] = -2.520263671875;
    oMPC.Rotation[0][0] = -0.236377269030;
    oMPC.Rotation[0][1] = -0.971438407898;
    oMPC.Rotation[0][2] = -0.020817618817;
    oMPC.Rotation[1][0] = 0.971473693848;
    oMPC.Rotation[1][1] = -0.235856205225;
    oMPC.Rotation[1][2] = -0.024716520682;
    oMPC.Rotation[2][0] = 0.019100610167;
    oMPC.Rotation[2][1] = -0.026066189632;
    oMPC.Rotation[2][2] = 0.999477863312;
    oMPC.Yaw = 1.809476375580;
    oMPC.YawTakeoff = 1.809818625450;
    oMPC.InLanding = 0;
    oMPC.LndReachedGround = 0;
    oMPC.VelZLp = -0.112574152648;
    oMPC.AccZLp = -0.069385573268;
    oMPC.VelMaxXY = 8.000000000000;
    oMPC.InTakeoff = 1;
    oMPC.TakeoffVelLimit = -0.500000000000;
    oMPC.Z_ResetCounter = 0;
    oMPC.XY_ResetCounter = 0;
    oMPC.VZ_ResetCounter = 0;
    oMPC.VXY_ResetCounter = 0;
    oMPC.HeadingResetCounter = 0;
    oMPC.RSetpoint[0][0] = -0.236559972167;
    oMPC.RSetpoint[0][1] = -0.971615433693;
    oMPC.RSetpoint[0][2] = -0.001685365918;
    oMPC.RSetpoint[1][0] = 0.971543788910;
    oMPC.RSetpoint[1][1] = -0.236563086510;
    oMPC.RSetpoint[1][2] = 0.011861341074;
    oMPC.RSetpoint[2][0] = -0.011923358776;
    oMPC.RSetpoint[2][1] = 0.001168511692;
    oMPC.RSetpoint[2][2] = 0.999928176403;

    oMPC.ControlNonManual(0.011679000221);

    UtAssert_True(oMPC.ResetIntXY == false, "oMPC.ResetIntXY");
    UtAssert_True(oMPC.ResetIntZ == false, "oMPC.ResetIntZ");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[0], -0.0000417999763, FLT_EPSILON, "oMPC.ThrustInt[0]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[1], -0.000151194414, FLT_EPSILON, "oMPC.ThrustInt[1]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[2],  0.00127921882, FLT_EPSILON, "oMPC.ThrustInt[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0], -0.0234711003, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1], -0.00323182391, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2],  0.456203759, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[0], -0.0413683914, FLT_EPSILON, "oMPC.PositionSetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[1],  0.00132636249, FLT_EPSILON, "oMPC.PositionSetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2], -2.52026367, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelZLp, -0.117643333972, FLT_EPSILON, "oMPC.VelZLp");
    UtAssert_DoubleCmpAbs(oMPC.AccZLp, -0.073761157691, FLT_EPSILON, "oMPC.AccZLp");
    UtAssert_True(oMPC.InLanding == false, "oMPC.InLanding");
    UtAssert_True(oMPC.LndReachedGround == false, "oMPC.LndReachedGround");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Thrust, 0.12572594, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Thrust");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][0], -0.236379489, FLT_EPSILON, "oMPC.RSetpoint[0][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][1], -0.971443236, FLT_EPSILON, "oMPC.RSetpoint[0][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][2],  0.0205576271, FLT_EPSILON, "oMPC.RSetpoint[0][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][0],  0.970802546, FLT_EPSILON, "oMPC.RSetpoint[1][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][1], -0.235228792, FLT_EPSILON, "oMPC.RSetpoint[1][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][2],  0.0470080934, FLT_EPSILON, "oMPC.RSetpoint[1][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][0], -0.0408299528, FLT_EPSILON, "oMPC.RSetpoint[2][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][1],  0.0310691465, FLT_EPSILON, "oMPC.RSetpoint[2][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][2],  0.998682916, FLT_EPSILON, "oMPC.RSetpoint[2][2]");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid == true, "oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Q_D[0],  0.617874324, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Q_D[0]");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Q_D[1], -0.00644910522, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Q_D[1]");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Q_D[2],  0.0248382147, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Q_D[2]");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Q_D[3],  0.785857916, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Q_D[3]");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.RollBody, 0.0311000906, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.RollBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.PitchBody, 0.0408413075, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.PitchBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.YawBody, 1.80963743, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.YawBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccX, -0.0254337303, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccY, -0.0581580326, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccZ, -1.23556244, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccZ");
    //UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Timestamp != 0, "oMPC.VehicleAttitudeSetpointMsg.Timestamp");
}



/**
 * Test MPC_AppMain(), Nominal - DoControl
 */
void Test_MPC_AppMain_Nominal_DoControl(void)
{
    MPC oMPC;

    oMPC.InitData();
    oMPC.InitConfigTbl();

    /* PASS 1 - Initializing. */
    oMPC.VehicleStatusMsg.Timestamp = 33671229639;
    oMPC.VehicleStatusMsg.SystemID = 1;
    oMPC.VehicleStatusMsg.ComponentID = 1;
    oMPC.VehicleStatusMsg.OnboardControlSensorsPresent = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsEnabled = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsHealth = 0;
    oMPC.VehicleStatusMsg.NavState = (PX4_NavigationState_t)17;
    oMPC.VehicleStatusMsg.ArmingState = (PX4_ArmingState_t)2;
    oMPC.VehicleStatusMsg.HilState = (PX4_HilState_t)0;
    oMPC.VehicleStatusMsg.Failsafe = 0;
    oMPC.VehicleStatusMsg.SystemType = (PX4_SystemType_t)13;
    oMPC.VehicleStatusMsg.IsRotaryWing = 1;
    oMPC.VehicleStatusMsg.IsVtol = 0;
    oMPC.VehicleStatusMsg.VtolFwPermanentStab = 0;
    oMPC.VehicleStatusMsg.InTransitionMode = 0;
    oMPC.VehicleStatusMsg.RcInputMode = (PX4_RcInMode_t)1;
    oMPC.VehicleStatusMsg.DataLinkLost = 1;
    oMPC.VehicleStatusMsg.DataLinkLostCounter = 0;
    oMPC.VehicleStatusMsg.EngineFailure = 0;
    oMPC.VehicleStatusMsg.EngineFailureCmd = 0;
    oMPC.VehicleStatusMsg.MissionFailure = 0;
    oMPC.VehicleLandDetectedMsg.Timestamp = 33671297774;
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000000000;
    oMPC.VehicleLandDetectedMsg.Landed = 0;
    oMPC.VehicleLandDetectedMsg.Freefall = 0;
    oMPC.VehicleLandDetectedMsg.GroundContact = 0;
    oMPC.ControlStateMsg.Timestamp = 33671300139;
    oMPC.ControlStateMsg.AccX = -0.023082222790;
    oMPC.ControlStateMsg.AccY = 0.250855356455;
    oMPC.ControlStateMsg.AccZ = -10.073497772217;
    oMPC.ControlStateMsg.VelX = 0.000000000000;
    oMPC.ControlStateMsg.VelY = 0.000000000000;
    oMPC.ControlStateMsg.VelZ = 0.000000000000;
    oMPC.ControlStateMsg.PosX = 0.000000000000;
    oMPC.ControlStateMsg.PosY = 0.000000000000;
    oMPC.ControlStateMsg.PosZ = 0.000000000000;
    oMPC.ControlStateMsg.Airspeed = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.Q[0] = 0.617908656597;
    oMPC.ControlStateMsg.Q[1] = -0.000546063937;
    oMPC.ControlStateMsg.Q[2] = -0.016150537878;
    oMPC.ControlStateMsg.Q[3] = 0.786083877087;
    oMPC.ControlStateMsg.DeltaQReset[0] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[1] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[2] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[3] = 0.000000000000;
    oMPC.ControlStateMsg.RollRate = -0.005168430042;
    oMPC.ControlStateMsg.PitchRate = -0.006302529015;
    oMPC.ControlStateMsg.YawRate = -0.001021062024;
    oMPC.ControlStateMsg.HorzAccMag = 0.000000000000;
    oMPC.ControlStateMsg.RollRateBias = 0.000000000000;
    oMPC.ControlStateMsg.PitchRateBias = 0.000000000000;
    oMPC.ControlStateMsg.YawRateBias = 0.000000000000;
    oMPC.ControlStateMsg.AirspeedValid = 1;
    oMPC.ControlStateMsg.QuatResetCounter = 1;
    oMPC.ManualControlSetpointMsg.Timestamp = 0;
    oMPC.ManualControlSetpointMsg.X = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Y = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Z = 0.000000000000;
    oMPC.ManualControlSetpointMsg.R = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Flaps = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux1 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux2 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux3 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux4 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux5 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.ModeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ReturnSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.RattitudeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.PosctlSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.LoiterSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.AcroSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.OffboardSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.KillSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ArmSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.TransitionSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.GearSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ModeSlot = (PX4_ModeSlot_t)0;
    oMPC.ManualControlSetpointMsg.DataSource = (PX4_ManualControlDataSource_t)0;
    oMPC.ManualControlSetpointMsg.StabSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ManSwitch = (PX4_SwitchPos_t)0;
    oMPC.PositionSetpointTripletMsg.Timestamp = 33671277064;
    oMPC.PositionSetpointTripletMsg.Previous.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Previous.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Current.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Current.Lat = 47.397743504482;
    oMPC.PositionSetpointTripletMsg.Current.Lon = 8.545593681335;
    oMPC.PositionSetpointTripletMsg.Current.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Alt = 490.789276123047;
    oMPC.PositionSetpointTripletMsg.Current.Yaw = 1.809637427330;
    oMPC.PositionSetpointTripletMsg.Current.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.LoiterRadius = 50.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingSpeed = -1.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingThrottle = -1.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 1;
    oMPC.PositionSetpointTripletMsg.Current.Type = (PX4_SetpointType_t)3;
    oMPC.PositionSetpointTripletMsg.Current.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawValid = 1;
    oMPC.PositionSetpointTripletMsg.Current.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.LoiterDirection = 1;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Next.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Next.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Next.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Next.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationIsForce = 0;
    oMPC.VehicleAttitudeSetpointMsg.Timestamp = 33671288749;
    oMPC.VehicleAttitudeSetpointMsg.RollBody = 0.00116860;
    oMPC.VehicleAttitudeSetpointMsg.PitchBody = 0.01192364;
    oMPC.VehicleAttitudeSetpointMsg.YawBody = 1.80963743;
    oMPC.VehicleAttitudeSetpointMsg.YawSpMoveRate = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[0] = 0.61781979;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[1] = -0.00432684;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[2] = 0.00414279;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[3] = 0.78629696;
    oMPC.VehicleAttitudeSetpointMsg.Thrust = 0.43548700;
    oMPC.VehicleAttitudeSetpointMsg.LandingGear = -1.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid = 1;
    oMPC.VehicleAttitudeSetpointMsg.RollResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.PitchResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.YawResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.FwControlYaw = 0;
    oMPC.VehicleAttitudeSetpointMsg.DisableMcYawControl = 0;
    oMPC.VehicleAttitudeSetpointMsg.ApplyFlaps = 0;
    oMPC.VehicleControlModeMsg.Timestamp = 33671229639;
    oMPC.VehicleControlModeMsg.Armed = 1;
    oMPC.VehicleControlModeMsg.ExternalManualOverrideOk = 0;
    oMPC.VehicleControlModeMsg.SystemHilEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlAutoEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlOffboardEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlRatesEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAttitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlRattitudeEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlForceEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlAccelerationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlTerminationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlFixedHdgEnabled = 0;
    oMPC.VehicleLocalPositionMsg.Timestamp = 33671300317;
    oMPC.VehicleLocalPositionMsg.RefTimestamp = 33671300317;
    oMPC.VehicleLocalPositionMsg.RefLat = 47.397743509334;
    oMPC.VehicleLocalPositionMsg.RefLon = 8.545593597490;
    oMPC.VehicleLocalPositionMsg.SurfaceBottomTimestamp = 33671300317;
    oMPC.VehicleLocalPositionMsg.X = -0.000593128381;
    oMPC.VehicleLocalPositionMsg.Y = 0.006940861698;
    oMPC.VehicleLocalPositionMsg.Z = -0.023656614125;
    oMPC.VehicleLocalPositionMsg.Delta_XY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_XY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_Z = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.VX = -0.003156758146;
    oMPC.VehicleLocalPositionMsg.VY = 0.031119752675;
    oMPC.VehicleLocalPositionMsg.VZ = -0.166829451919;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AX = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AY = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Yaw = 1.809476375580;
    oMPC.VehicleLocalPositionMsg.RefAlt = 488.269012451172;
    oMPC.VehicleLocalPositionMsg.DistBottom = 0.024133512750;
    oMPC.VehicleLocalPositionMsg.DistBottomRate = 0.166829451919;
    oMPC.VehicleLocalPositionMsg.EpH = 0.411934226751;
    oMPC.VehicleLocalPositionMsg.EpV = 0.275059789419;
    oMPC.VehicleLocalPositionMsg.EvV = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.EstimatorType = 0;
    oMPC.VehicleLocalPositionMsg.XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.Z_Valid = 1;
    oMPC.VehicleLocalPositionMsg.V_XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.V_Z_Valid = 1;
    oMPC.VehicleLocalPositionMsg.XY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.Z_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VXY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VZ_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.XY_Global = 1;
    oMPC.VehicleLocalPositionMsg.Z_Global = 1;
    oMPC.VehicleLocalPositionMsg.DistBottomValid = 1;
    oMPC.VehicleLocalPositionSetpointMsg.Timestamp = 33671288750;
    oMPC.VehicleLocalPositionSetpointMsg.X = -0.000539467554;
    oMPC.VehicleLocalPositionSetpointMsg.Y = 0.006310853176;
    oMPC.VehicleLocalPositionSetpointMsg.Z = -2.520263671875;
    oMPC.VehicleLocalPositionSetpointMsg.Yaw = 1.809637427330;
    oMPC.VehicleLocalPositionSetpointMsg.VX = 0.000020811351;
    oMPC.VehicleLocalPositionSetpointMsg.VY = -0.000289084419;
    oMPC.VehicleLocalPositionSetpointMsg.VZ = -0.058515001088;
    oMPC.VehicleLocalPositionSetpointMsg.AccX = 0.007203754503;
    oMPC.VehicleLocalPositionSetpointMsg.AccY = -0.050698895007;
    oMPC.VehicleLocalPositionSetpointMsg.AccZ = -4.273990154266;
    oMPC.HomePositionMsg.Timestamp = 33671219483;
    oMPC.HomePositionMsg.Lat = 47.397743504982;
    oMPC.HomePositionMsg.Lon = 8.545593667673;
    oMPC.HomePositionMsg.Alt = 488.283905029297;
    oMPC.HomePositionMsg.X = -0.000483906595;
    oMPC.HomePositionMsg.Y = 0.005282537080;
    oMPC.HomePositionMsg.Z = -0.014878900722;
    oMPC.HomePositionMsg.Yaw = 1.809811234474;
    oMPC.HomePositionMsg.DirectionX = 0.000000000000;
    oMPC.HomePositionMsg.DirectionY = 0.000000000000;
    oMPC.HomePositionMsg.DirectionZ = 0.000000000000;
    oMPC.GearStateInitialized = 0;
    oMPC.RefAlt = 488.269012451172;
    oMPC.RefPos.timestamp = 33671300423;
    oMPC.RefPos.lat_rad = 0.827246682254;
    oMPC.RefPos.lon_rad = 0.149148744814;
    oMPC.RefPos.sin_lat = 0.736070429013;
    oMPC.RefPos.cos_lat = 0.676904959010;
    oMPC.RefPos.init_done = 1;
    oMPC.RefTimestamp = 33671300317;
    oMPC.ResetPositionSetpoint = 0;
    oMPC.ResetAltitudeSetpoint = 0;
    oMPC.DoResetAltPos = 1;
    oMPC.ModeAuto = 1;
    oMPC.PositionHoldEngaged = 0;
    oMPC.AltitudeHoldEngaged = 0;
    oMPC.RunPosControl = 1;
    oMPC.RunAltControl = 1;
    oMPC.ResetIntZ = 0;
    oMPC.ResetIntXY = 0;
    oMPC.ResetIntZManual = 0;
    oMPC.ResetYawSetpoint = 1;
    oMPC.HoldOffboardXY = 0;
    oMPC.HoldOffboardZ = 0;
    oMPC.LimitVelXY = 1;
    oMPC.ThrustInt[0] = 0.000005650265;
    oMPC.ThrustInt[1] = -0.000070955997;
    oMPC.ThrustInt[2] = 0.000187757949;
    oMPC.Position[0] = -0.000593128381;
    oMPC.Position[1] = 0.006940861698;
    oMPC.Position[2] = -0.023656614125;
    oMPC.PositionSetpoint[0] = -0.000539467554;
    oMPC.PositionSetpoint[1] = 0.006310853176;
    oMPC.PositionSetpoint[2] = -2.520263671875;
    oMPC.Velocity[0] = -0.003156758146;
    oMPC.Velocity[1] = 0.031119752675;
    oMPC.Velocity[2] = -0.166829451919;
    oMPC.VelocitySetpoint[0] = 0.000020811351;
    oMPC.VelocitySetpoint[1] = -0.000289084419;
    oMPC.VelocitySetpoint[2] = -0.058515001088;
    oMPC.VelocityPrevious[0] = -0.002393212635;
    oMPC.VelocityPrevious[1] = 0.030026217923;
    oMPC.VelocityPrevious[2] = -0.165472149849;
    oMPC.VelocityFF[0] = 0.000000000000;
    oMPC.VelocityFF[1] = 0.000000000000;
    oMPC.VelocityFF[2] = 0.000000000000;
    oMPC.VelocitySetpointPrevious[0] = 0.000020811351;
    oMPC.VelocitySetpointPrevious[1] = -0.000289084419;
    oMPC.VelocitySetpointPrevious[2] = -0.058515001088;
    oMPC.VelocityErrD[0] = 0.044798217714;
    oMPC.VelocityErrD[1] = -0.070677615702;
    oMPC.VelocityErrD[2] = 0.132893085480;
    oMPC.CurrentPositionSetpoint[0] = -0.000539467554;
    oMPC.CurrentPositionSetpoint[1] = 0.006310853176;
    oMPC.CurrentPositionSetpoint[2] = -2.520263671875;
    oMPC.Rotation[0][0] = -0.236377269030;
    oMPC.Rotation[0][1] = -0.971438407898;
    oMPC.Rotation[0][2] = -0.020817618817;
    oMPC.Rotation[1][0] = 0.971473693848;
    oMPC.Rotation[1][1] = -0.235856205225;
    oMPC.Rotation[1][2] = -0.024716520682;
    oMPC.Rotation[2][0] = 0.019100610167;
    oMPC.Rotation[2][1] = -0.026066189632;
    oMPC.Rotation[2][2] = 0.999477863312;
    oMPC.Yaw = 1.809476375580;
    oMPC.YawTakeoff = 1.809818625450;
    oMPC.InLanding = 0;
    oMPC.LndReachedGround = 0;
    oMPC.VelZLp = -0.112574152648;
    oMPC.AccZLp = -0.069385573268;
    oMPC.VelMaxXY = 8.000000000000;
    oMPC.InTakeoff = 1;
    oMPC.TakeoffVelLimit = -0.500000000000;
    oMPC.Z_ResetCounter = 0;
    oMPC.XY_ResetCounter = 0;
    oMPC.VZ_ResetCounter = 0;
    oMPC.VXY_ResetCounter = 0;
    oMPC.HeadingResetCounter = 0;
    oMPC.RSetpoint[0][0] = -0.236559972167;
    oMPC.RSetpoint[0][1] = -0.971615433693;
    oMPC.RSetpoint[0][2] = -0.001685365918;
    oMPC.RSetpoint[1][0] = 0.971543788910;
    oMPC.RSetpoint[1][1] = -0.236563086510;
    oMPC.RSetpoint[1][2] = 0.011861341074;
    oMPC.RSetpoint[2][0] = -0.011923358776;
    oMPC.RSetpoint[2][1] = 0.001168511692;
    oMPC.RSetpoint[2][2] = 0.999928176403;

    oMPC.DoControl(0.011679000221);

    UtAssert_True(oMPC.ResetIntXY == false, "oMPC.ResetIntXY");
    UtAssert_True(oMPC.ResetIntZ == false, "oMPC.ResetIntZ");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[0], -0.0000417999763, FLT_EPSILON, "oMPC.ThrustInt[0]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[1], -0.000151194414, FLT_EPSILON, "oMPC.ThrustInt[1]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[2],  0.00127921882, FLT_EPSILON, "oMPC.ThrustInt[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0], -0.0234711003, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1], -0.00323182391, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2],  0.456203759, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[0], -0.0413683914, FLT_EPSILON, "oMPC.PositionSetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[1],  0.00132636249, FLT_EPSILON, "oMPC.PositionSetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2], -2.52026367, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelZLp, -0.117643334, FLT_EPSILON, "oMPC.VelZLp");
    UtAssert_DoubleCmpAbs(oMPC.AccZLp, -0.0737611577, FLT_EPSILON, "oMPC.AccZLp");
    UtAssert_True(oMPC.InLanding == false, "oMPC.InLanding");
    UtAssert_True(oMPC.LndReachedGround == false, "oMPC.LndReachedGround");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Thrust, 0.12572594, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Thrust");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][0], -0.236379489, FLT_EPSILON, "oMPC.RSetpoint[0][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][1], -0.971443236, FLT_EPSILON, "oMPC.RSetpoint[0][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][2],  0.0205576271, FLT_EPSILON, "oMPC.RSetpoint[0][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][0],  0.970802546, FLT_EPSILON, "oMPC.RSetpoint[1][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][1], -0.235228792, FLT_EPSILON, "oMPC.RSetpoint[1][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][2],  0.0470080934, FLT_EPSILON, "oMPC.RSetpoint[1][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][0], -0.0408299528, FLT_EPSILON, "oMPC.RSetpoint[2][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][1],  0.0310691465, FLT_EPSILON, "oMPC.RSetpoint[2][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][2],  0.998682916, FLT_EPSILON, "oMPC.RSetpoint[2][2]");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid == true, "oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.RollBody, 0.0311000906, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.RollBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.PitchBody, 0.0408413075, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.PitchBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.YawBody, 1.80963743, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.YawBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccX, -0.0254337303, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccY, -0.0581580326, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccZ, -1.23556244, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccZ");
    //UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Timestamp != 0, "oMPC.VehicleAttitudeSetpointMsg.Timestamp");

    /* PASS 2 - Received triplet to transition to takeoff. */
    oMPC.VehicleStatusMsg.Timestamp = 24032670706;
    oMPC.VehicleStatusMsg.SystemID = 1;
    oMPC.VehicleStatusMsg.ComponentID = 1;
    oMPC.VehicleStatusMsg.OnboardControlSensorsPresent = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsEnabled = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsHealth = 0;
    oMPC.VehicleStatusMsg.NavState = (PX4_NavigationState_t)17;
    oMPC.VehicleStatusMsg.ArmingState = (PX4_ArmingState_t)2;
    oMPC.VehicleStatusMsg.HilState = (PX4_HilState_t)0;
    oMPC.VehicleStatusMsg.Failsafe = 0;
    oMPC.VehicleStatusMsg.SystemType = (PX4_SystemType_t)13;
    oMPC.VehicleStatusMsg.IsRotaryWing = 1;
    oMPC.VehicleStatusMsg.IsVtol = 0;
    oMPC.VehicleStatusMsg.VtolFwPermanentStab = 0;
    oMPC.VehicleStatusMsg.InTransitionMode = 0;
    oMPC.VehicleStatusMsg.RcInputMode = (PX4_RcInMode_t)2;
    oMPC.VehicleStatusMsg.DataLinkLost = 1;
    oMPC.VehicleStatusMsg.DataLinkLostCounter = 0;
    oMPC.VehicleStatusMsg.EngineFailure = 0;
    oMPC.VehicleStatusMsg.EngineFailureCmd = 0;
    oMPC.VehicleStatusMsg.MissionFailure = 0;
    oMPC.VehicleLandDetectedMsg.Timestamp = 24027002927;
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000000000;
    oMPC.VehicleLandDetectedMsg.Landed = 1;
    oMPC.VehicleLandDetectedMsg.Freefall = 0;
    oMPC.VehicleLandDetectedMsg.GroundContact = 0;
    oMPC.ControlStateMsg.Timestamp = 24032731497;
    oMPC.ControlStateMsg.AccX = 0.309893906116;
    oMPC.ControlStateMsg.AccY = 2.341282367706;
    oMPC.ControlStateMsg.AccZ = -9.519306182861;
    oMPC.ControlStateMsg.VelX = 0.000000000000;
    oMPC.ControlStateMsg.VelY = 0.000000000000;
    oMPC.ControlStateMsg.VelZ = 0.000000000000;
    oMPC.ControlStateMsg.PosX = 0.000000000000;
    oMPC.ControlStateMsg.PosY = 0.000000000000;
    oMPC.ControlStateMsg.PosZ = 0.000000000000;
    oMPC.ControlStateMsg.Airspeed = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.Q[0] = 0.710987925529;
    oMPC.ControlStateMsg.Q[1] = -0.096762053668;
    oMPC.ControlStateMsg.Q[2] = -0.078669473529;
    oMPC.ControlStateMsg.Q[3] = 0.692058146000;
    oMPC.ControlStateMsg.DeltaQReset[0] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[1] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[2] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[3] = 0.000000000000;
    oMPC.ControlStateMsg.RollRate = -0.006228174083;
    oMPC.ControlStateMsg.PitchRate = -0.005193498451;
    oMPC.ControlStateMsg.YawRate = 0.003723319387;
    oMPC.ControlStateMsg.HorzAccMag = 0.000000000000;
    oMPC.ControlStateMsg.RollRateBias = 0.000000000000;
    oMPC.ControlStateMsg.PitchRateBias = 0.000000000000;
    oMPC.ControlStateMsg.YawRateBias = 0.000000000000;
    oMPC.ControlStateMsg.AirspeedValid = 1;
    oMPC.ControlStateMsg.QuatResetCounter = 1;
    oMPC.ManualControlSetpointMsg.Timestamp = 0;
    oMPC.ManualControlSetpointMsg.X = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Y = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Z = 0.000000000000;
    oMPC.ManualControlSetpointMsg.R = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Flaps = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux1 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux2 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux3 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux4 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux5 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.ModeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ReturnSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.RattitudeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.PosctlSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.LoiterSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.AcroSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.OffboardSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.KillSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ArmSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.TransitionSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.GearSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ModeSlot = (PX4_ModeSlot_t)0;
    oMPC.ManualControlSetpointMsg.DataSource = (PX4_ManualControlDataSource_t)0;
    oMPC.ManualControlSetpointMsg.StabSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ManSwitch = (PX4_SwitchPos_t)0;
    oMPC.PositionSetpointTripletMsg.Timestamp = 24032719721;
    oMPC.PositionSetpointTripletMsg.Previous.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Previous.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Current.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Current.Lat = 47.397742018286;
    oMPC.PositionSetpointTripletMsg.Current.Lon = 8.545593861439;
    oMPC.PositionSetpointTripletMsg.Current.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Alt = 491.345642089844;
    oMPC.PositionSetpointTripletMsg.Current.Yaw = 1.541023015976;
    oMPC.PositionSetpointTripletMsg.Current.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.LoiterRadius = 50.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingSpeed = -1.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingThrottle = -1.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 1;
    oMPC.PositionSetpointTripletMsg.Current.Type = (PX4_SetpointType_t)3;
    oMPC.PositionSetpointTripletMsg.Current.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawValid = 1;
    oMPC.PositionSetpointTripletMsg.Current.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.LoiterDirection = 1;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Next.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Next.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Next.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Next.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationIsForce = 0;
    oMPC.VehicleAttitudeSetpointMsg.Timestamp = 24032720693;
    oMPC.VehicleAttitudeSetpointMsg.RollBody = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.PitchBody = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.YawBody = 1.54103744;
    oMPC.VehicleAttitudeSetpointMsg.YawSpMoveRate = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[0] = 1.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[1] = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[2] = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[3] = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Thrust = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.LandingGear = -1.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid = 1;
    oMPC.VehicleAttitudeSetpointMsg.RollResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.PitchResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.YawResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.FwControlYaw = 0;
    oMPC.VehicleAttitudeSetpointMsg.DisableMcYawControl = 0;
    oMPC.VehicleAttitudeSetpointMsg.ApplyFlaps = 0;
    oMPC.VehicleControlModeMsg.Timestamp = 24032670706;
    oMPC.VehicleControlModeMsg.Armed = 1;
    oMPC.VehicleControlModeMsg.ExternalManualOverrideOk = 0;
    oMPC.VehicleControlModeMsg.SystemHilEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlAutoEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlOffboardEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlRatesEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAttitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlRattitudeEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlForceEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlAccelerationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlTerminationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlFixedHdgEnabled = 0;
    oMPC.VehicleLocalPositionMsg.Timestamp = 24032731662;
    oMPC.VehicleLocalPositionMsg.RefTimestamp = 24032731662;
    oMPC.VehicleLocalPositionMsg.RefLat = 47.397741988665;
    oMPC.VehicleLocalPositionMsg.RefLon = 8.545593887673;
    oMPC.VehicleLocalPositionMsg.SurfaceBottomTimestamp = 24032731662;
    oMPC.VehicleLocalPositionMsg.X = 0.003216056153;
    oMPC.VehicleLocalPositionMsg.Y = -0.002135221381;
    oMPC.VehicleLocalPositionMsg.Z = 0.037966083735;
    oMPC.VehicleLocalPositionMsg.Delta_XY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_XY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_Z = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.VX = 0.010284845717;
    oMPC.VehicleLocalPositionMsg.VY = 0.018262712285;
    oMPC.VehicleLocalPositionMsg.VZ = 0.002778425580;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AX = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AY = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Yaw = 1.541051983833;
    oMPC.VehicleLocalPositionMsg.RefAlt = 488.884002685547;
    oMPC.VehicleLocalPositionMsg.DistBottom = -0.001209149254;
    oMPC.VehicleLocalPositionMsg.DistBottomRate = -0.002778425580;
    oMPC.VehicleLocalPositionMsg.EpH = 0.426289767027;
    oMPC.VehicleLocalPositionMsg.EpV = 0.174988523126;
    oMPC.VehicleLocalPositionMsg.EvV = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.EstimatorType = 0;
    oMPC.VehicleLocalPositionMsg.XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.Z_Valid = 1;
    oMPC.VehicleLocalPositionMsg.V_XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.V_Z_Valid = 1;
    oMPC.VehicleLocalPositionMsg.XY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.Z_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VXY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VZ_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.XY_Global = 1;
    oMPC.VehicleLocalPositionMsg.Z_Global = 1;
    oMPC.VehicleLocalPositionMsg.DistBottomValid = 1;
    oMPC.VehicleLocalPositionSetpointMsg.Timestamp = 24032720694;
    oMPC.VehicleLocalPositionSetpointMsg.X = 0.003091249382;
    oMPC.VehicleLocalPositionSetpointMsg.Y = -0.002206754638;
    oMPC.VehicleLocalPositionSetpointMsg.Z = 0.038970947266;
    oMPC.VehicleLocalPositionSetpointMsg.Yaw = 1.541037440300;
    oMPC.VehicleLocalPositionSetpointMsg.VX = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.VY = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.VZ = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccX = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccY = 0.000000000000;
    oMPC.VehicleLocalPositionSetpointMsg.AccZ = -0.000000000000;
    oMPC.HomePositionMsg.Timestamp = 24032660299;
    oMPC.HomePositionMsg.Lat = 47.397742015757;
    oMPC.HomePositionMsg.Lon = 8.545593857299;
    oMPC.HomePositionMsg.Alt = 488.844818115234;
    oMPC.HomePositionMsg.X = 0.003012487665;
    oMPC.HomePositionMsg.Y = -0.002286192495;
    oMPC.HomePositionMsg.Z = 0.039181273431;
    oMPC.HomePositionMsg.Yaw = 1.541115880013;
    oMPC.HomePositionMsg.DirectionX = 0.000000000000;
    oMPC.HomePositionMsg.DirectionY = 0.000000000000;
    oMPC.HomePositionMsg.DirectionZ = 0.000000000000;
    oMPC.GearStateInitialized = 0;
    oMPC.RefAlt = 488.884002685547;
    oMPC.RefPos.timestamp = 24032731790;
    oMPC.RefPos.lat_rad = 0.827246655713;
    oMPC.RefPos.lon_rad = 0.149148749878;
    oMPC.RefPos.sin_lat = 0.736070411047;
    oMPC.RefPos.cos_lat = 0.676904978546;
    oMPC.RefPos.init_done = 1;
    oMPC.RefTimestamp = 24032731662;
    oMPC.ResetPositionSetpoint = 0;
    oMPC.ResetAltitudeSetpoint = 0;
    oMPC.DoResetAltPos = 1;
    oMPC.ModeAuto = 1;
    oMPC.PositionHoldEngaged = 0;
    oMPC.AltitudeHoldEngaged = 0;
    oMPC.RunPosControl = 0;
    oMPC.RunAltControl = 0;
    oMPC.ResetIntZ = 1;
    oMPC.ResetIntXY = 1;
    oMPC.ResetIntZManual = 0;
    oMPC.ResetYawSetpoint = 1;
    oMPC.HoldOffboardXY = 0;
    oMPC.HoldOffboardZ = 0;
    oMPC.LimitVelXY = 0;
    oMPC.ThrustInt[0] = 0.000000000000;
    oMPC.ThrustInt[1] = 0.000000000000;
    oMPC.ThrustInt[2] = 0.000535357744;
    oMPC.Position[0] = 0.003216056153;
    oMPC.Position[1] = -0.002135221381;
    oMPC.Position[2] = 0.037966083735;
    oMPC.PositionSetpoint[0] = 0.003091249382;
    oMPC.PositionSetpoint[1] = -0.002206754638;
    oMPC.PositionSetpoint[2] = 0.038970947266;
    oMPC.Velocity[0] = 0.010284845717;
    oMPC.Velocity[1] = 0.018262712285;
    oMPC.Velocity[2] = 0.002778425580;
    oMPC.VelocitySetpoint[0] = 0.000000000000;
    oMPC.VelocitySetpoint[1] = 0.000000000000;
    oMPC.VelocitySetpoint[2] = 0.000000000000;
    oMPC.VelocityPrevious[0] = 0.011032714508;
    oMPC.VelocityPrevious[1] = 0.019521865994;
    oMPC.VelocityPrevious[2] = 0.003654700937;
    oMPC.VelocityFF[0] = 0.000000000000;
    oMPC.VelocityFF[1] = 0.000000000000;
    oMPC.VelocityFF[2] = 0.000000000000;
    oMPC.VelocitySetpointPrevious[0] = 0.000000000000;
    oMPC.VelocitySetpointPrevious[1] = 0.000000000000;
    oMPC.VelocitySetpointPrevious[2] = 0.000000000000;
    oMPC.VelocityErrD[0] = 0.019077949226;
    oMPC.VelocityErrD[1] = 0.028009349480;
    oMPC.VelocityErrD[2] = 0.106526166201;
    oMPC.CurrentPositionSetpoint[0] = -5227320.500000000000;
    oMPC.CurrentPositionSetpoint[1] = -1067128.125000000000;
    oMPC.CurrentPositionSetpoint[2] = 488.884002685547;
    oMPC.Rotation[0][0] = 0.029733359814;
    oMPC.Rotation[0][1] = -0.968865513802;
    oMPC.Rotation[0][2] = -0.245796024799;
    oMPC.Rotation[1][0] = 0.999314427376;
    oMPC.Rotation[1][1] = 0.023385345936;
    oMPC.Rotation[1][2] = 0.028705596924;
    oMPC.Rotation[2][0] = -0.022063843906;
    oMPC.Rotation[2][1] = -0.246481001377;
    oMPC.Rotation[2][2] = 0.968896508217;
    oMPC.Yaw = 1.541051387787;
    oMPC.YawTakeoff = 1.541065335274;
    oMPC.InLanding = 0;
    oMPC.LndReachedGround = 0;
    oMPC.VelZLp = 1.018464684486;
    oMPC.AccZLp = -2.107974290848;
    oMPC.VelMaxXY = 8.000000000000;
    oMPC.InTakeoff = 0;
    oMPC.TakeoffVelLimit = 0.000000000000;
    oMPC.Z_ResetCounter = 0;
    oMPC.XY_ResetCounter = 0;
    oMPC.VZ_ResetCounter = 0;
    oMPC.VXY_ResetCounter = 0;
    oMPC.HeadingResetCounter = 0;
    oMPC.RSetpoint[0][0] = 1.000000000000;
    oMPC.RSetpoint[0][1] = 0.000000000000;
    oMPC.RSetpoint[0][2] = 0.000000000000;
    oMPC.RSetpoint[1][0] = 0.000000000000;
    oMPC.RSetpoint[1][1] = 1.000000000000;
    oMPC.RSetpoint[1][2] = 0.000000000000;
    oMPC.RSetpoint[2][0] = 0.000000000000;
    oMPC.RSetpoint[2][1] = 0.000000000000;
    oMPC.RSetpoint[2][2] = 1.000000000000;

    oMPC.DoControl(0.012089000084);

    UtAssert_True(oMPC.ResetIntXY == false, "oMPC.ResetIntXY");
    UtAssert_True(oMPC.ResetIntZ == false, "oMPC.ResetIntZ");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[0], -0.0000816519387, FLT_EPSILON, "oMPC.ThrustInt[0]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[1], -0.0000508740704, FLT_EPSILON, "oMPC.ThrustInt[1]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[2], -0.000114646195, FLT_EPSILON, "oMPC.ThrustInt[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0], -0.0234863255, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1], -0.00277875992, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2], -0.0604449995, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[0], -0.0375322029, FLT_EPSILON, "oMPC.PositionSetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[1], -0.00695630955, FLT_EPSILON, "oMPC.PositionSetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2], -2.4616394, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelZLp, 0.920235634, FLT_EPSILON, "oMPC.VelZLp");
    UtAssert_DoubleCmpAbs(oMPC.AccZLp, -1.91111803, FLT_EPSILON, "oMPC.AccZLp");
    UtAssert_True(oMPC.InLanding == false, "oMPC.InLanding");
    UtAssert_True(oMPC.LndReachedGround == false, "oMPC.LndReachedGround");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Thrust, 0.520086765, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Thrust");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][0],  0.0297684446, FLT_EPSILON, "oMPC.RSetpoint[0][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][1], -0.999515772, FLT_EPSILON, "oMPC.RSetpoint[0][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][2],  0.00906175375, FLT_EPSILON, "oMPC.RSetpoint[0][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][0],  0.999541104, FLT_EPSILON, "oMPC.RSetpoint[1][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][1],  0.0298176687, FLT_EPSILON, "oMPC.RSetpoint[1][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][2],  0.00534632057, FLT_EPSILON, "oMPC.RSetpoint[1][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][0], -0.00561393239, FLT_EPSILON, "oMPC.RSetpoint[2][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][1],  0.00889844354, FLT_EPSILON, "oMPC.RSetpoint[2][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][2],  0.999944627, FLT_EPSILON, "oMPC.RSetpoint[2][2]");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid == true, "oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.RollBody, 0.00889870152, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.RollBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.PitchBody, 0.00561396172, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.PitchBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.YawBody, 1.54102302, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.YawBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccX, -0.0478064008, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccY, -0.028205175, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccZ, -5.27533102, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccZ");
    //UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Timestamp != 0, "oMPC.VehicleAttitudeSetpointMsg.Timestamp");
}






/**
 * Test MPC_AppMain(), Nominal - ControlManual
 */
void Test_MPC_AppMain_Nominal_ControlManual(void)
{
    MPC oMPC;

    oMPC.InitData();
    oMPC.InitConfigTbl();

    /* Hovering stationary */
    oMPC.VehicleStatusMsg.Timestamp = 71387147216;
    oMPC.VehicleStatusMsg.SystemID = 1;
    oMPC.VehicleStatusMsg.ComponentID = 1;
    oMPC.VehicleStatusMsg.OnboardControlSensorsPresent = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsEnabled = 0;
    oMPC.VehicleStatusMsg.OnboardControlSensorsHealth = 0;
    oMPC.VehicleStatusMsg.NavState = (PX4_NavigationState_t)2;
    oMPC.VehicleStatusMsg.ArmingState = (PX4_ArmingState_t)2;
    oMPC.VehicleStatusMsg.HilState = (PX4_HilState_t)0;
    oMPC.VehicleStatusMsg.Failsafe = 0;
    oMPC.VehicleStatusMsg.SystemType = (PX4_SystemType_t)13;
    oMPC.VehicleStatusMsg.IsRotaryWing = 1;
    oMPC.VehicleStatusMsg.IsVtol = 0;
    oMPC.VehicleStatusMsg.VtolFwPermanentStab = 0;
    oMPC.VehicleStatusMsg.InTransitionMode = 0;
    oMPC.VehicleStatusMsg.RcInputMode = (PX4_RcInMode_t)2;
    oMPC.VehicleStatusMsg.DataLinkLost = 1;
    oMPC.VehicleStatusMsg.DataLinkLostCounter = 0;
    oMPC.VehicleStatusMsg.EngineFailure = 0;
    oMPC.VehicleStatusMsg.EngineFailureCmd = 0;
    oMPC.VehicleStatusMsg.MissionFailure = 0;
    oMPC.VehicleLandDetectedMsg.Timestamp = 71352650214;
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000000000;
    oMPC.VehicleLandDetectedMsg.Landed = 0;
    oMPC.VehicleLandDetectedMsg.Freefall = 0;
    oMPC.VehicleLandDetectedMsg.GroundContact = 0;
    oMPC.ControlStateMsg.Timestamp = 71387312526;
    oMPC.ControlStateMsg.AccX = -0.039567112923;
    oMPC.ControlStateMsg.AccY = 0.473200112581;
    oMPC.ControlStateMsg.AccZ = -10.268331527710;
    oMPC.ControlStateMsg.VelX = 0.000000000000;
    oMPC.ControlStateMsg.VelY = 0.000000000000;
    oMPC.ControlStateMsg.VelZ = 0.000000000000;
    oMPC.ControlStateMsg.PosX = 0.000000000000;
    oMPC.ControlStateMsg.PosY = 0.000000000000;
    oMPC.ControlStateMsg.PosZ = 0.000000000000;
    oMPC.ControlStateMsg.Airspeed = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.VelVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[0] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[1] = 0.000000000000;
    oMPC.ControlStateMsg.PosVariance[2] = 0.000000000000;
    oMPC.ControlStateMsg.Q[0] = 0.708361566067;
    oMPC.ControlStateMsg.Q[1] = 0.003428169293;
    oMPC.ControlStateMsg.Q[2] = -0.012681787834;
    oMPC.ControlStateMsg.Q[3] = 0.705727636814;
    oMPC.ControlStateMsg.DeltaQReset[0] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[1] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[2] = 0.000000000000;
    oMPC.ControlStateMsg.DeltaQReset[3] = 0.000000000000;
    oMPC.ControlStateMsg.RollRate = -0.144795238972;
    oMPC.ControlStateMsg.PitchRate = -0.034734643996;
    oMPC.ControlStateMsg.YawRate = 0.042580589652;
    oMPC.ControlStateMsg.HorzAccMag = 0.000000000000;
    oMPC.ControlStateMsg.RollRateBias = 0.000000000000;
    oMPC.ControlStateMsg.PitchRateBias = 0.000000000000;
    oMPC.ControlStateMsg.YawRateBias = 0.000000000000;
    oMPC.ControlStateMsg.AirspeedValid = 1;
    oMPC.ControlStateMsg.QuatResetCounter = 1;
    oMPC.ManualControlSetpointMsg.Timestamp = 71387286482;
    oMPC.ManualControlSetpointMsg.X = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Y = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Z = 0.557142853737;
    oMPC.ManualControlSetpointMsg.R = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Flaps = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux1 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux2 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux3 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux4 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.Aux5 = 0.000000000000;
    oMPC.ManualControlSetpointMsg.ModeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ReturnSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.RattitudeSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.PosctlSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.LoiterSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.AcroSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.OffboardSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.KillSwitch = (PX4_SwitchPos_t)3;
    oMPC.ManualControlSetpointMsg.ArmSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.TransitionSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.GearSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ModeSlot = (PX4_ModeSlot_t)-1;
    oMPC.ManualControlSetpointMsg.DataSource = (PX4_ManualControlDataSource_t)1;
    oMPC.ManualControlSetpointMsg.StabSwitch = (PX4_SwitchPos_t)0;
    oMPC.ManualControlSetpointMsg.ManSwitch = (PX4_SwitchPos_t)0;
    oMPC.PositionSetpointTripletMsg.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Previous.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Previous.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Previous.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Previous.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Current.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Current.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Current.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Current.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Current.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationIsForce = 0;
    oMPC.PositionSetpointTripletMsg.Next.Timestamp = 0;
    oMPC.PositionSetpointTripletMsg.Next.Lat = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Lon = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.X = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Y = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Z = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.VZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Alt = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yaw = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Yawspeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.LoiterRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.PitchMin = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AX = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AY = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AZ = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.AcceptanceRadius = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingSpeed = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.CruisingThrottle = 0.000000000000;
    oMPC.PositionSetpointTripletMsg.Next.Valid = 0;
    oMPC.PositionSetpointTripletMsg.Next.Type = (PX4_SetpointType_t)0;
    oMPC.PositionSetpointTripletMsg.Next.PositionValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.VelocityValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.DisableMcYawControl = 0;
    oMPC.PositionSetpointTripletMsg.Next.YawspeedValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.LoiterDirection = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Next.AccelerationIsForce = 0;
    oMPC.VehicleAttitudeSetpointMsg.Timestamp = 71387315002;
    oMPC.VehicleAttitudeSetpointMsg.RollBody = -0.01284447;
    oMPC.VehicleAttitudeSetpointMsg.PitchBody = -0.02443473;
    oMPC.VehicleAttitudeSetpointMsg.YawBody = 1.56718373;
    oMPC.VehicleAttitudeSetpointMsg.YawSpMoveRate = 0.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[0] = 0.70837075;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[1] = 0.00407394;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[2] = -0.01318680;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[3] = 0.70570564;
    oMPC.VehicleAttitudeSetpointMsg.Thrust = 0.43972412;
    oMPC.VehicleAttitudeSetpointMsg.LandingGear = -1.00000000;
    oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid = 1;
    oMPC.VehicleAttitudeSetpointMsg.RollResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.PitchResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.YawResetIntegral = 0;
    oMPC.VehicleAttitudeSetpointMsg.FwControlYaw = 0;
    oMPC.VehicleAttitudeSetpointMsg.DisableMcYawControl = 0;
    oMPC.VehicleAttitudeSetpointMsg.ApplyFlaps = 0;
    oMPC.VehicleControlModeMsg.Timestamp = 71387147216;
    oMPC.VehicleControlModeMsg.Armed = 1;
    oMPC.VehicleControlModeMsg.ExternalManualOverrideOk = 0;
    oMPC.VehicleControlModeMsg.SystemHilEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAutoEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlOffboardEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlRatesEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAttitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlRattitudeEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlForceEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlAccelerationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlTerminationEnabled = 0;
    oMPC.VehicleControlModeMsg.ControlFixedHdgEnabled = 0;
    oMPC.VehicleLocalPositionMsg.Timestamp = 71387313201;
    oMPC.VehicleLocalPositionMsg.RefTimestamp = 71387313201;
    oMPC.VehicleLocalPositionMsg.RefLat = 47.397741972655;
    oMPC.VehicleLocalPositionMsg.RefLon = 8.545593900131;
    oMPC.VehicleLocalPositionMsg.SurfaceBottomTimestamp = 71387313201;
    oMPC.VehicleLocalPositionMsg.X = -1.995731592178;
    oMPC.VehicleLocalPositionMsg.Y = 1.565559029579;
    oMPC.VehicleLocalPositionMsg.Z = -0.826584398746;
    oMPC.VehicleLocalPositionMsg.Delta_XY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_XY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_Z = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.VX = -0.027511324733;
    oMPC.VehicleLocalPositionMsg.VY = 0.006788865663;
    oMPC.VehicleLocalPositionMsg.VZ = -0.051438082010;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[0] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VXY[1] = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Delta_VZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AX = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AY = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.AZ = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.Yaw = 1.566952824593;
    oMPC.VehicleLocalPositionMsg.RefAlt = 504.262969970703;
    oMPC.VehicleLocalPositionMsg.DistBottom = 1.155246853828;
    oMPC.VehicleLocalPositionMsg.DistBottomRate = 0.051438082010;
    oMPC.VehicleLocalPositionMsg.EpH = 0.369742006063;
    oMPC.VehicleLocalPositionMsg.EpV = 0.216528818011;
    oMPC.VehicleLocalPositionMsg.EvV = 0.000000000000;
    oMPC.VehicleLocalPositionMsg.EstimatorType = 0;
    oMPC.VehicleLocalPositionMsg.XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.Z_Valid = 1;
    oMPC.VehicleLocalPositionMsg.V_XY_Valid = 1;
    oMPC.VehicleLocalPositionMsg.V_Z_Valid = 1;
    oMPC.VehicleLocalPositionMsg.XY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.Z_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VXY_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.VZ_ResetCounter = 0;
    oMPC.VehicleLocalPositionMsg.XY_Global = 1;
    oMPC.VehicleLocalPositionMsg.Z_Global = 1;
    oMPC.VehicleLocalPositionMsg.DistBottomValid = 1;
    oMPC.VehicleLocalPositionSetpointMsg.Timestamp = 71387315000;
    oMPC.VehicleLocalPositionSetpointMsg.X = -2.018968582153;
    oMPC.VehicleLocalPositionSetpointMsg.Y = 1.573781371117;
    oMPC.VehicleLocalPositionSetpointMsg.Z = -0.799546360970;
    oMPC.VehicleLocalPositionSetpointMsg.Yaw = 1.567183732986;
    oMPC.VehicleLocalPositionSetpointMsg.VX = -0.025572603568;
    oMPC.VehicleLocalPositionSetpointMsg.VY = 0.008065467700;
    oMPC.VehicleLocalPositionSetpointMsg.VZ = -0.047619022429;
    oMPC.VehicleLocalPositionSetpointMsg.AccX = 0.055766887963;
    oMPC.VehicleLocalPositionSetpointMsg.AccY = 0.105148039758;
    oMPC.VehicleLocalPositionSetpointMsg.AccZ = -4.310579776764;
    oMPC.HomePositionMsg.Timestamp = 71342533449;
    oMPC.HomePositionMsg.Lat = 47.397741945098;
    oMPC.HomePositionMsg.Lon = 8.545593960621;
    oMPC.HomePositionMsg.Alt = 504.170379638672;
    oMPC.HomePositionMsg.X = -0.003064178396;
    oMPC.HomePositionMsg.Y = 0.004552976228;
    oMPC.HomePositionMsg.Z = 0.092577040195;
    oMPC.HomePositionMsg.Yaw = 1.543434739113;
    oMPC.HomePositionMsg.DirectionX = 0.000000000000;
    oMPC.HomePositionMsg.DirectionY = 0.000000000000;
    oMPC.HomePositionMsg.DirectionZ = 0.000000000000;
    oMPC.GearStateInitialized = 0;
    oMPC.RefAlt = 504.262969970703;
    oMPC.RefPos.timestamp = 71387315018;
    oMPC.RefPos.lat_rad = 0.827246655434;
    oMPC.RefPos.lon_rad = 0.149148750096;
    oMPC.RefPos.sin_lat = 0.736070410858;
    oMPC.RefPos.cos_lat = 0.676904978752;
    oMPC.RefPos.init_done = 1;
    oMPC.RefTimestamp = 71387313201;
    oMPC.ResetPositionSetpoint = 0;
    oMPC.ResetAltitudeSetpoint = 0;
    oMPC.DoResetAltPos = 1;
    oMPC.ModeAuto = 0;
    oMPC.PositionHoldEngaged = 1;
    oMPC.AltitudeHoldEngaged = 0;
    oMPC.RunPosControl = 1;
    oMPC.RunAltControl = 1;
    oMPC.ResetIntZ = 0;
    oMPC.ResetIntXY = 0;
    oMPC.ResetIntZManual = 0;
    oMPC.ResetYawSetpoint = 0;
    oMPC.HoldOffboardXY = 0;
    oMPC.HoldOffboardZ = 0;
    oMPC.LimitVelXY = 0;
    oMPC.ThrustInt[0] = 0.005063694902;
    oMPC.ThrustInt[1] = 0.010720678605;
    oMPC.ThrustInt[2] = 0.077658712864;
    oMPC.Position[0] = -1.995731592178;
    oMPC.Position[1] = 1.565559029579;
    oMPC.Position[2] = -0.826584398746;
    oMPC.PositionSetpoint[0] = -2.018968582153;
    oMPC.PositionSetpoint[1] = 1.573781371117;
    oMPC.PositionSetpoint[2] = -0.799560546875;
    oMPC.Velocity[0] = -0.027511324733;
    oMPC.Velocity[1] = 0.006788865663;
    oMPC.Velocity[2] = -0.051438082010;
    oMPC.VelocitySetpoint[0] = -0.025572603568;
    oMPC.VelocitySetpoint[1] = 0.008065467700;
    oMPC.VelocitySetpoint[2] = -0.047619022429;
    oMPC.VelocityPrevious[0] = -0.030311975628;
    oMPC.VelocityPrevious[1] = 0.008536919951;
    oMPC.VelocityPrevious[2] = -0.017990816385;
    oMPC.VelocityFF[0] = 0.000000000000;
    oMPC.VelocityFF[1] = 0.000000000000;
    oMPC.VelocityFF[2] = 0.000000000000;
    oMPC.VelocitySetpointPrevious[0] = -0.025572603568;
    oMPC.VelocitySetpointPrevious[1] = 0.008065467700;
    oMPC.VelocitySetpointPrevious[2] = -0.047619022429;
    oMPC.VelocityErrD[0] = -0.014930430800;
    oMPC.VelocityErrD[1] = 0.010728724301;
    oMPC.VelocityErrD[2] = 0.159829705954;
    oMPC.CurrentPositionSetpoint[0] = 0.000000000000;
    oMPC.CurrentPositionSetpoint[1] = 0.000000000000;
    oMPC.CurrentPositionSetpoint[2] = 0.000000000000;
    oMPC.Rotation[0][0] = 0.003575533628;
    oMPC.Rotation[0][1] = -0.999907612801;
    oMPC.Rotation[0][2] = -0.013127873652;
    oMPC.Rotation[1][0] = 0.999733686447;
    oMPC.Rotation[1][1] = 0.003873676062;
    oMPC.Rotation[1][2] = -0.022756543010;
    oMPC.Rotation[2][0] = 0.022805288434;
    oMPC.Rotation[2][1] = -0.013043008745;
    oMPC.Rotation[2][2] = 0.999655008316;
    oMPC.Yaw = 1.567219853401;
    oMPC.YawTakeoff = 1.543403744698;
    oMPC.InLanding = 0;
    oMPC.LndReachedGround = 0;
    oMPC.VelZLp = -0.017763005570;
    oMPC.AccZLp = 0.145495340228;
    oMPC.VelMaxXY = 8.000000000000;
    oMPC.InTakeoff = 0;
    oMPC.TakeoffVelLimit = 1.180821299553;
    oMPC.Z_ResetCounter = 0;
    oMPC.XY_ResetCounter = 0;
    oMPC.VZ_ResetCounter = 0;
    oMPC.VXY_ResetCounter = 0;
    oMPC.HeadingResetCounter = 0;
    oMPC.RSetpoint[0][0] = 0.003611507826;
    oMPC.RSetpoint[0][1] = -0.999909877777;
    oMPC.RSetpoint[0][2] = -0.012932283804;
    oMPC.RSetpoint[1][0] = 0.999695003033;
    oMPC.RSetpoint[1][1] = 0.003926097415;
    oMPC.RSetpoint[1][2] = -0.024383721873;
    oMPC.RSetpoint[2][0] = 0.024432297796;
    oMPC.RSetpoint[2][1] = -0.012840277515;
    oMPC.RSetpoint[2][2] = 0.999619007111;

    oMPC.ControlManual(0.148974999785);

    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.RollBody, -0.013540800661, FLT_EPSILON, "oMPC.VehicleAttitudeSetpoint.RollBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.PitchBody, -0.026186019182, FLT_EPSILON, "oMPC.VehicleAttitudeSetpoint.PitchBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.YawBody, 1.567183732986, FLT_EPSILON, "oMPC.VehicleAttitudeSetpoint.YawBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.YawSpMoveRate, 0.000000000000, FLT_EPSILON, "oMPC.VehicleAttitudeSetpoint.YawSpMoveRate");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Q_D[0],  0.708368480206, FLT_EPSILON, "oMPC.VehicleAttitudeSetpoint.Q_D[0]");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Q_D[1],  0.004445353057, FLT_EPSILON, "oMPC.VehicleAttitudeSetpoint.Q_D[1]");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Q_D[2], -0.014052679762, FLT_EPSILON, "oMPC.VehicleAttitudeSetpoint.Q_D[2]");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Q_D[3],  0.705688893795, FLT_EPSILON, "oMPC.VehicleAttitudeSetpoint.Q_D[3]");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Thrust,  0.420230060816, FLT_EPSILON, "oMPC.VehicleAttitudeSetpoint.Thrust");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.LandingGear, -1.000000000000, FLT_EPSILON, "oMPC.VehicleAttitudeSetpoint.LandingGear");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid == 1, "oMPC.VehicleAttitudeSetpoint.Q_D_Valid");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.RollResetIntegral == 0, "oMPC.VehicleAttitudeSetpoint.RollResetIntegral");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.PitchResetIntegral == 0, "oMPC.VehicleAttitudeSetpoint.PitchResetIntegral");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.YawResetIntegral == 0, "oMPC.VehicleAttitudeSetpoint.YawResetIntegral");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.FwControlYaw == 0, "oMPC.VehicleAttitudeSetpoint.FwControlYaw");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.DisableMcYawControl == 0, "oMPC.VehicleAttitudeSetpoint.DisableMcYawControl");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.ApplyFlaps == 0, "oMPC.VehicleAttitudeSetpoint.ApplyFlaps");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.X, -2.018968582153, FLT_EPSILON, "MPC.VehicleLocalPositionSetpointMsg.X");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.Y,  1.573781371117, FLT_EPSILON, "MPC.VehicleLocalPositionSetpointMsg.Y");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.Z, -0.799546360970, FLT_EPSILON, "MPC.VehicleLocalPositionSetpointMsg.Z");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.Yaw, 1.567183732986, FLT_EPSILON, "MPC.VehicleLocalPositionSetpointMsg.Yaw");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.VX, -0.025572603568, FLT_EPSILON, "MPC.VehicleLocalPositionSetpointMsg.VX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.VY,  0.008065467700, FLT_EPSILON, "MPC.VehicleLocalPositionSetpointMsg.VY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.VZ, -0.047619022429, FLT_EPSILON, "MPC.VehicleLocalPositionSetpointMsg.VZ");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccX,  0.056190323085, FLT_EPSILON, "MPC.VehicleLocalPositionSetpointMsg.AccX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccY,  0.107689961791, FLT_EPSILON, "MPC.VehicleLocalPositionSetpointMsg.AccY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccZ, -4.119281768799, FLT_EPSILON, "MPC.VehicleLocalPositionSetpointMsg.AccZ");
    UtAssert_DoubleCmpAbs(oMPC.VehicleGlobalVelocitySetpointMsg.VX, -0.022075140849, FLT_EPSILON, "MPC.VehicleGlobalVelocitySetpointMsg.VX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleGlobalVelocitySetpointMsg.VY,  0.007811224554, FLT_EPSILON, "MPC.VehicleGlobalVelocitySetpointMsg.VY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleGlobalVelocitySetpointMsg.VZ, -0.047619022429, FLT_EPSILON, "MPC.VehicleGlobalVelocitySetpointMsg.VZ");
    UtAssert_True(oMPC.GearStateInitialized == 0, "oMPC.GearStateInitialized");
    UtAssert_DoubleCmpAbs(oMPC.RefAlt, 504.262969970703, FLT_EPSILON, "oMPC.RefAlt");
    UtAssert_DoubleCmpAbs(oMPC.RefPos.lat_rad, 0.827246655434, FLT_EPSILON, "oMPC.LatRad");
    UtAssert_DoubleCmpAbs(oMPC.RefPos.lon_rad, 0.149148750096, FLT_EPSILON, "oMPC.LonRad");
    UtAssert_DoubleCmpAbs(oMPC.RefPos.sin_lat, 0.736070410858, FLT_EPSILON, "oMPC.SinLat");
    UtAssert_DoubleCmpAbs(oMPC.RefPos.cos_lat, 0.676904978752, FLT_EPSILON, "oMPC.CosLat");
    UtAssert_True(oMPC.RefPos.init_done == 1, "oMPC.init_done");
    UtAssert_True(oMPC.ResetPositionSetpoint == 0, "oMPC.ResetPositionSetpoint");
    UtAssert_True(oMPC.ResetAltitudeSetpoint == 0, "oMPC.ResetAltitudeSetpoint");
    UtAssert_True(oMPC.DoResetAltPos == 1, "oMPC.DoResetAltPos");
    UtAssert_True(oMPC.ModeAuto == 0, "oMPC.ModeAuto");
    UtAssert_True(oMPC.PositionHoldEngaged == 1, "oMPC.PositionHoldEngaged");
    UtAssert_True(oMPC.AltitudeHoldEngaged == 0, "oMPC.AltitudeHoldEngaged");
    UtAssert_True(oMPC.RunPosControl == 1, "oMPC.RunPosControl");
    UtAssert_True(oMPC.RunAltControl == 0, "oMPC.RunAltControl");
    UtAssert_True(oMPC.ResetIntZ == 0, "oMPC.ResetIntZ");
    UtAssert_True(oMPC.ResetIntXY == 0, "oMPC.ResetIntXY");
    UtAssert_True(oMPC.ResetIntZManual == 0, "oMPC.ResetIntZManual");
    UtAssert_True(oMPC.ResetYawSetpoint == 0, "oMPC.ResetYawSetpoint");
    UtAssert_True(oMPC.HoldOffboardXY == 0, "oMPC.HoldOffboardXY");
    UtAssert_True(oMPC.HoldOffboardZ == 0, "oMPC.HoldOffboardZ");
    UtAssert_True(oMPC.LimitVelXY == 0, "oMPC.LimitVelXY");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[0], 0.005225665867, FLT_EPSILON, "oMPC.ThrustInt[0]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[1], 0.010751139373, FLT_EPSILON, "oMPC.ThrustInt[1]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[2], 0.077744051814, FLT_EPSILON, "oMPC.ThrustInt[2]");
    UtAssert_DoubleCmpAbs(oMPC.Position[0], -1.995731592178, FLT_EPSILON, "oMPC.Position[0]");
    UtAssert_DoubleCmpAbs(oMPC.Position[1],  1.565559029579, FLT_EPSILON, "oMPC.Position[1]");
    UtAssert_DoubleCmpAbs(oMPC.Position[2], -0.826584398746, FLT_EPSILON, "oMPC.Position[2]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[0], -2.0189685821534, FLT_EPSILON, "oMPC.PositionSetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[1],  1.573781371117, FLT_EPSILON, "oMPC.PositionSetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2], -0.826584398746, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.Velocity[0], -0.027511324733, FLT_EPSILON, "oMPC.Velocity[0]");
    UtAssert_DoubleCmpAbs(oMPC.Velocity[1],  0.006788865663, FLT_EPSILON, "oMPC.Velocity[1]");
    UtAssert_DoubleCmpAbs(oMPC.Velocity[2], -0.051438082010, FLT_EPSILON, "oMPC.Velocity[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0], -0.022075140849, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1],  0.007811224554, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2], -0.047619022429, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityPrevious[0], -0.030311975628, FLT_EPSILON, "oMPC.VelocityPrevious[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityPrevious[1],  0.008536919951, FLT_EPSILON, "oMPC.VelocityPrevious[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityPrevious[2], -0.017990816385, FLT_EPSILON, "oMPC.VelocityPrevious[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityFF[0], 0.000000000000, FLT_EPSILON, "oMPC.VelocityFF[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityFF[1], 0.000000000000, FLT_EPSILON, "oMPC.VelocityFF[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityFF[2], 0.000000000000, FLT_EPSILON, "oMPC.VelocityFF[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpointPrevious[0], -0.022075140849, FLT_EPSILON, "oMPC.VelocitySetpointPrevious[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpointPrevious[1],  0.007811224554, FLT_EPSILON, "oMPC.VelocitySetpointPrevious[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpointPrevious[2], -0.047619022429, FLT_EPSILON, "oMPC.VelocitySetpointPrevious[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityErrD[0], -0.014930430800, FLT_EPSILON, "oMPC.VelocityErrD[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityErrD[1],  0.010728724301, FLT_EPSILON, "oMPC.VelocityErrD[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocityErrD[2],  0.159829705954, FLT_EPSILON, "oMPC.VelocityErrD[2]");
    UtAssert_DoubleCmpAbs(oMPC.CurrentPositionSetpoint[0], 0.000000000000, FLT_EPSILON, "oMPC.CurrentPositionSetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.CurrentPositionSetpoint[1], 0.000000000000, FLT_EPSILON, "oMPC.CurrentPositionSetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.CurrentPositionSetpoint[2], 0.000000000000, FLT_EPSILON, "oMPC.CurrentPositionSetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][0],  0.003611347405, FLT_EPSILON, "oMPC.RSetpoint[0][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][1], -0.999900460243, FLT_EPSILON, "oMPC.RSetpoint[0][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][2], -0.013634878211, FLT_EPSILON, "oMPC.RSetpoint[0][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][0],  0.999650597572, FLT_EPSILON, "oMPC.RSetpoint[1][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][1],  0.003966780379, FLT_EPSILON, "oMPC.RSetpoint[1][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][2], -0.026131536812, FLT_EPSILON, "oMPC.RSetpoint[1][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][0],  0.026183025911, FLT_EPSILON, "oMPC.RSetpoint[2][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][1], -0.013535744511, FLT_EPSILON, "oMPC.RSetpoint[2][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][2],  0.999565482140, FLT_EPSILON, "oMPC.RSetpoint[2][2]");
}



/**************************************************************************
 * Rollup Test Cases
 **************************************************************************/
void MPC_App_Test_AddTestCases(void)
{
    UtTest_Add(Test_MPC_InitEvent_Fail_Register, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_InitEvent_Fail_Register");

    UtTest_Add(Test_MPC_InitPipe_Fail_CreateSCHPipe, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_InitPipe_Fail_CreateSCHPipe");
    UtTest_Add(Test_MPC_InitPipe_Fail_SubscribeWakeup, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_InitPipe_Fail_SubscribeWakeup");
    UtTest_Add(Test_MPC_InitPipe_Fail_SubscribeSendHK, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_InitPipe_Fail_SubscribeSendHK");
    UtTest_Add(Test_MPC_InitPipe_Fail_CreateCMDPipe, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_InitPipe_Fail_CreateCMDPipe");
    UtTest_Add(Test_MPC_InitPipe_Fail_SubscribeCMD, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_InitPipe_Fail_SubscribeCMD");
    UtTest_Add(Test_MPC_InitPipe_Fail_CreateDATAPipe, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_InitPipe_Fail_CreateDATAPipe");

    UtTest_Add(Test_MPC_InitData, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_InitData");

    UtTest_Add(Test_MPC_InitApp_Fail_InitEvent, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_InitApp_Fail_InitEvent");
    UtTest_Add(Test_MPC_InitApp_Fail_InitPipe, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_InitApp_Fail_InitPipe");
    UtTest_Add(Test_MPC_InitApp_Fail_InitData, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_InitApp_Fail_InitData");
    UtTest_Add(Test_MPC_InitApp_Fail_InitConfigTbl, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_InitApp_Fail_InitConfigTbl");
    UtTest_Add(Test_MPC_InitApp_Nominal, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_InitApp_Nominal");

    UtTest_Add(Test_MPC_AppMain_Fail_RegisterApp, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Fail_RegisterApp");
    UtTest_Add(Test_MPC_AppMain_Fail_InitApp, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Fail_InitApp");
    UtTest_Add(Test_MPC_AppMain_Fail_AcquireConfigPtrs, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Fail_AcquireConfigPtrs");
    UtTest_Add(Test_MPC_AppMain_InvalidSchMessage, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_InvalidSchMessage");
    UtTest_Add(Test_MPC_AppMain_Nominal_SendHK, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_SendHK");
    UtTest_Add(Test_MPC_AppMain_Nominal_Wakeup, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_Wakeup");

    UtTest_Add(Test_MPC_AppMain_Nominal_ProcessControlStateMsg, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_ProcessControlStateMsg");

    UtTest_Add(Test_MPC_AppMain_Nominal_ProcessVehicleLocalPositionMsg, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_ProcessVehicleLocalPositionMsg");

    UtTest_Add(Test_MPC_AppMain_Nominal_UpdateParamsFromTable, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_UpdateParamsFromTable");

    UtTest_Add(Test_MPC_AppMain_Nominal_UpdateVelocityDerivative, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_UpdateVelocityDerivative");

    UtTest_Add(Test_MPC_AppMain_Nominal_SlowLandGradualVelocityLimit, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_SlowLandGradualVelocityLimit");

    UtTest_Add(Test_MPC_AppMain_Nominal_LimitAltitude, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_LimitAltitude");

    UtTest_Add(Test_MPC_AppMain_Nominal_ApplyVelocitySetpointSlewRate, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_ApplyVelocitySetpointSlewRate");

    UtTest_Add(Test_MPC_AppMain_Nominal_CalculateVelocitySetpoint, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_CalculateVelocitySetpoint");

    UtTest_Add(Test_MPC_AppMain_Nominal_CalculateThrustSetpoint, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_CalculateThrustSetpoint");

    UtTest_Add(Test_MPC_AppMain_Nominal_ControlAuto, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_ControlAuto");

    UtTest_Add(Test_MPC_AppMain_Nominal_ControlNonManual, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_ControlNonManual");

    UtTest_Add(Test_MPC_AppMain_Nominal_DoControl, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_DoControl");

    UtTest_Add(Test_MPC_AppMain_Nominal_ControlManual, MPC_Test_Setup, MPC_Test_TearDown,
               "Test_MPC_AppMain_Nominal_ControlManual");
}


