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
}


