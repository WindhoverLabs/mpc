
add_airliner_app_def(mpc
    FILE MPC
    SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_app.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_app.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_config_utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_events.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_msg.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_symbols.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_tbldefs.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_version.h

    INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/

    USER_DOCS   ${CMAKE_CURRENT_SOURCE_DIR}/user_doxy.in
    DESIGN_DOCS ${CMAKE_CURRENT_SOURCE_DIR}/detail_doxy.in

    MSG_OVERRIDES ${CMAKE_CURRENT_SOURCE_DIR}/mpc_msg_overrides.json

    COMMANDER_PLUGIN
        ${CMAKE_CURRENT_SOURCE_DIR}/../../cdr
)

#add_airliner_app_unit_test(mpc-ut
#    UTASSERT
#    SOURCES
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_app_test.cpp
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_app_test.h
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_cmds_test.cpp
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_cmds_test.h
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_config_tbl_test.cpp
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_config_tbl_test.h
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_testrunner.cpp
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_test_utils.cpp
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_test_utils.h
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/stubs/ut_mpc_custom_stubs.cpp
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/stubs/ut_mpc_custom_stubs.h
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/stubs/ut_mpc_custom_hooks.cpp
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/stubs/ut_mpc_custom_hooks.h
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_test_tables.h
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/mpc_nominal_config.c
#        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_app.cpp
#        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_app.h
#        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_config_utils.cpp
#        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_events.h
#        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_msg.h
#        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_tbldefs.h
#        ${CMAKE_CURRENT_SOURCE_DIR}/../src/mpc_version.h
#        ${PX4LIB_LIB_SOURCES}
#
#    INCLUDES
#        ${CMAKE_CURRENT_SOURCE_DIR}/../src/
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/
#
#    VALGRIND_SUPPRESSION_FILE
#        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/valgrind.supp
#)
