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
#include "Quaternion.hpp"
#include "Matrix3F3.hpp"
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
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][1], -0.99982351, FLT_EPSILON, "oMPC.RSetpoint[0][1]");
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
    oMPC.ThrustInt[0] = 0.00030984;
    oMPC.ThrustInt[1] = -0.00008532;
    oMPC.ThrustInt[2] = -0.00359623;
    oMPC.VehicleControlModeMsg.ControlAccelerationEnabled = 0;
    oMPC.PositionSetpointTripletMsg.Current.AccelerationValid = 0;
    oMPC.PositionSetpointTripletMsg.Current.AX = 0.0;
    oMPC.PositionSetpointTripletMsg.Current.AY = 0.0;
    oMPC.PositionSetpointTripletMsg.Current.AZ = 0.0;
    oMPC.VelocityErrD[0] = -0.06768808;
    oMPC.VelocityErrD[1] = -0.09714489;
    oMPC.VelocityErrD[2] = 2.95152497;
    oMPC.VehicleLandDetectedMsg.GroundContact = 0;
    oMPC.Rotation[0][0] = -0.00171348;
    oMPC.Rotation[0][1] = -0.99986279;
    oMPC.Rotation[0][2] = -0.01647412;
    oMPC.Rotation[1][0] = 0.99964392;
    oMPC.Rotation[1][1] = -0.00127402;
    oMPC.Rotation[1][2] = -0.02665191;
    oMPC.Rotation[2][0] = 0.02662726;
    oMPC.Rotation[2][1] = -0.01651392;
    oMPC.Rotation[2][2] = 0.99950898;
    oMPC.VelZLp = 0.08065736;
    oMPC.AccZLp = 0.01490617;
    oMPC.VehicleLandDetectedMsg.Landed = 0;
    oMPC.LndReachedGround = 0;
    oMPC.InLanding = 0;
    oMPC.RSetpoint[0][0] = -0.00177507;
    oMPC.RSetpoint[0][1] = -0.99998456;
    oMPC.RSetpoint[0][2] = -0.00526018;
    oMPC.RSetpoint[1][0] = 0.99999738;
    oMPC.RSetpoint[1][1] = -0.00178276;
    oMPC.RSetpoint[1][2] = 0.00145798;
    oMPC.RSetpoint[2][0] = -0.00146734;
    oMPC.RSetpoint[2][1] = -0.00525758;
    oMPC.RSetpoint[2][2] = 0.99998504;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[0] = 0.70647490;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[1] = -0.00237643;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[2] = -0.00134217;
    oMPC.VehicleAttitudeSetpointMsg.Q_D[3] = 0.70773280;
    oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid = 1;
    oMPC.VehicleAttitudeSetpointMsg.RollBody = -0.00525761;
    oMPC.VehicleAttitudeSetpointMsg.PitchBody = 0.00146734;
    oMPC.VehicleAttitudeSetpointMsg.YawBody = 1.57257140;
    oMPC.VehicleLocalPositionSetpointMsg.AccX = 0.04007427;
    oMPC.VehicleLocalPositionSetpointMsg.AccY = -0.01110753;
    oMPC.VehicleLocalPositionSetpointMsg.AccZ = -7.61830711;
    /* For InAutoTakeoff() */
    oMPC.VehicleControlModeMsg.ControlOffboardEnabled = 0;
    /* For CalculateVelocitySetpoint() */
    oMPC.RunPosControl = 1;
    oMPC.RunAltControl = 1;
    oMPC.Position[0] = -0.01327894;
    oMPC.Position[1] = 0.00520686;
    oMPC.Position[2] = 0.29706430;
    oMPC.PositionSetpoint[0] = -0.01353991;
    oMPC.PositionSetpoint[1] = 0.00498144;
    oMPC.PositionSetpoint[2] = -2.20419312;
    oMPC.Velocity[0] = -0.02288467;
    oMPC.Velocity[1] = 0.01710499;
    oMPC.Velocity[2] = -0.08474080;
    oMPC.VelocitySetpoint[0] = -0.00024793;
    oMPC.VelocitySetpoint[1] = -0.00021415;
    oMPC.VelocitySetpoint[2] = 0.27136251;
    oMPC.LimitVelXY = 1;
    oMPC.VehicleControlModeMsg.ControlPositionEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlAltitudeEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlVelocityEnabled = 1;
    oMPC.VehicleControlModeMsg.ControlClimbRateEnabled = 1;
    oMPC.PositionSetpointTripletMsg.Current.Valid = 1;
    oMPC.PositionSetpointTripletMsg.Current.Type = PX4_SETPOINT_TYPE_TAKEOFF;
    oMPC.VehicleControlModeMsg.ControlManualEnabled = 0;
    oMPC.InTakeoff = 1;
    oMPC.TakeoffVelLimit = -0.27136251;
    oMPC.VehicleGlobalVelocitySetpointMsg.Timestamp = 0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VX = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VY = 0.0;
    oMPC.VehicleGlobalVelocitySetpointMsg.VZ = 0.0;
    oMPC.ResetPositionSetpoint = false;
    oMPC.ResetAltitudeSetpoint = false;
    /* For LimitAltitude() */
    oMPC.VehicleLandDetectedMsg.AltMax = 10000.000000;
    /* For SlowLandGradualVelocityLimit() */
    oMPC.HomePositionMsg.Z = 0.29691720;
    /* For LimitVelXYGradually() */
    oMPC.CurrentPositionSetpoint[0] = -0.01353991;
    oMPC.CurrentPositionSetpoint[1] = 0.00498144;
    oMPC.CurrentPositionSetpoint[2] = -2.20419312;
    /* For ApplyVelocitySetpointSlewRate() */
    oMPC.VelocityPrevious[0] = -0.02853232;
    oMPC.VelocityPrevious[1] = 0.00790535;
    oMPC.VelocityPrevious[2] = 0.19010030;
    /* For GetCruisingSpeedXY() */
    oMPC.PositionSetpointTripletMsg.Current.CruisingSpeed = -1.00000000;

    Ut_MPC_Custom_SetReturnCode(UT_MPC_PX4LIB_GETPX4TIMEUS_INDEX, 1, 1);
    oMPC.CalculateThrustSetpoint(0.06097000);

    UtAssert_True(oMPC.ResetIntXY == false, "oMPC.ResetIntXY");
    UtAssert_True(oMPC.ResetIntZ == false, "oMPC.ResetIntZ");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[0],  0.00058588, FLT_EPSILON, "oMPC.ThrustInt[0]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[1], -0.00029651, FLT_EPSILON, "oMPC.ThrustInt[1]");
    UtAssert_DoubleCmpAbs(oMPC.ThrustInt[2], -0.00033949, FLT_EPSILON, "oMPC.ThrustInt[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[0], -0.00024793, FLT_EPSILON, "oMPC.VelocitySetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[1], -0.00021415, FLT_EPSILON, "oMPC.VelocitySetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.VelocitySetpoint[2],  0.27136251, FLT_EPSILON, "oMPC.VelocitySetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[0], -0.01353991, FLT_EPSILON, "oMPC.PositionSetpoint[0]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[1],  0.00498144, FLT_EPSILON, "oMPC.PositionSetpoint[1]");
    UtAssert_DoubleCmpAbs(oMPC.PositionSetpoint[2], -2.20419312, FLT_EPSILON, "oMPC.PositionSetpoint[2]");
    UtAssert_DoubleCmpAbs(oMPC.VelZLp, -0.00001725, FLT_EPSILON, "oMPC.VelZLp");
    UtAssert_DoubleCmpAbs(oMPC.AccZLp, -2.19109321, FLT_EPSILON, "oMPC.AccZLp");
    UtAssert_True(oMPC.InLanding == false, "oMPC.InLanding");
    UtAssert_True(oMPC.LndReachedGround == false, "oMPC.LndReachedGround");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.Thrust, 0.28974438, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.Thrust");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][0], -0.00177493, FLT_EPSILON, "oMPC.RSetpoint[0][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][1], -0.99994385, FLT_EPSILON, "oMPC.RSetpoint[0][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[0][2], -0.01044398, FLT_EPSILON, "oMPC.RSetpoint[0][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][0],  0.99991876, FLT_EPSILON, "oMPC.RSetpoint[1][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][1], -0.00190652, FLT_EPSILON, "oMPC.RSetpoint[1][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[1][2],  0.01260337, FLT_EPSILON, "oMPC.RSetpoint[1][2]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][0], -0.01262258, FLT_EPSILON, "oMPC.RSetpoint[2][0]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][1], -0.01042076, FLT_EPSILON, "oMPC.RSetpoint[2][1]");
    UtAssert_DoubleCmpAbs(oMPC.RSetpoint[2][2],  0.99986601, FLT_EPSILON, "oMPC.RSetpoint[2][2]");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid == true, "oMPC.VehicleAttitudeSetpointMsg.Q_D_Valid");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.RollBody, -0.01042178, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.RollBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleAttitudeSetpointMsg.PitchBody, 0.01262291, FLT_EPSILON, "oMPC.VehicleAttitudeSetpointMsg.PitchBody");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccX,  0.04007427, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccX");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccY, -0.01110753, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccY");
    UtAssert_DoubleCmpAbs(oMPC.VehicleLocalPositionSetpointMsg.AccZ, -7.61830711, FLT_EPSILON, "oMPC.VehicleLocalPositionSetpointMsg.AccZ");
    UtAssert_True(oMPC.VehicleAttitudeSetpointMsg.Timestamp != 0, "oMPC.VehicleAttitudeSetpointMsg.Timestamp");
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
}


