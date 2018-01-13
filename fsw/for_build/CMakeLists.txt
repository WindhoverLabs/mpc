
add_airliner_app_def(mpc
    FILE MPC
    SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_app.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_app.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_config_utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_events.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_msg.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_tbldefs.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_version.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Matrix3F3.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Matrix3F3.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Matrix4F4.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Matrix4F4.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Quaternion.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Quaternion.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Vector3F.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Vector3F.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Vector4F.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Vector4F.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/geo/geo.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/geo/geo.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/geo_lookup/geo_mag_declination.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/geo_lookup/geo_mag_declination.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Derivative.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Derivative.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/LowPassFilter2p.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/LowPassFilter2p.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/lib/mathlib/math/Expo.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/lib/mathlib/math/Expo.hpp
   
    INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/

    USER_DOCS   ${CMAKE_CURRENT_SOURCE_DIR}/user_doxy.in
    DESIGN_DOCS ${CMAKE_CURRENT_SOURCE_DIR}/detail_doxy.in
)

add_airliner_app_unit_test(mpc-ut
    UTASSERT
    SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_app_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_app_test.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_cmds_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_cmds_test.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_config_tbl_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_config_tbl_test.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_testrunner.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_test_utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_test_utils.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/stubs/ut_mpc_custom_stubs.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/stubs/ut_mpc_custom_stubs.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/stubs/ut_mpc_custom_hooks.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/stubs/ut_mpc_custom_hooks.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_test_tables.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_nominal_config.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_app.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_app.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_config_utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_events.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_msg.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_tbldefs.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_version.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Matrix3F3.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Matrix3F3.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Matrix4F4.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Matrix4F4.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Quaternion.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Quaternion.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Vector2F.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Vector2F.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Vector3F.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Vector3F.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Vector4F.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Vector4F.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/geo/geo.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/geo/geo.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/geo_lookup/geo_mag_declination.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/geo_lookup/geo_mag_declination.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Derivative.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/Derivative.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/LowPassFilter2p.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/LowPassFilter2p.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/lib/mathlib/math/Expo.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/lib/mathlib/math/Expo.hpp
                
    INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/
        
    VALGRIND_SUPPRESSION_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/valgrind.supp
)
