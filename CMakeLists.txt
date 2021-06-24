####################################################
# CMakeLists.txt for project sick_scan_xd
####################################################
cmake_minimum_required(VERSION 2.8.3)

# build options: set OFF for relese version, ON for development and test
option(ENABLE_EMULATOR "Build emulator for offline and unittests" ON) # OFF (release) or ON (development)
option(BUILD_DEBUG_TARGET "Build debug target" ON) # OFF (release) or ON (development)

# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic")
#

# Added CMP0048 to avoid unstable warning of build process
#
if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(sick_scan)



macro(use_cxx11)
    if (CMAKE_VERSION VERSION_LESS "3.1")
        if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
        endif ()
    else ()
        set(CMAKE_CXX_STANDARD 11)
    endif ()
endmacro(use_cxx11)

if(BUILD_DEBUG_TARGET)
        set(CMAKE_BUILD_TYPE Debug) #uncomment to activate debug mode for lib_sick as well
        set(CMAKE_ENABLE_EXPORTS 1) #uncomment to activate debug mode for lib_sick as well
        # https://cmake.org/cmake/help/v3.5/prop_tgt/ENABLE_EXPORTS.html
        # set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++03")
endif(BUILD_DEBUG_TARGET)

# Switch on, if you use c11-specific commands
use_cxx11()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-format-overflow")

# By uncommenting the following lines you can list all variable names ...
# --- CUT ---
# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#	message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
# --- CUT ---

# For warning <catkin_workspace>/devel/include' should be placed in
# the devel space instead of the build space
# see: https://answers.ros.org/question/67244/qtcreator-with-catkin/


# message(${CMAKE_BINARY_DIR})

# set(CATKIN_DEVEL_PREFIX "${CMAKE_BINARY_DIR}/devel")
# set(CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}/devel;/opt/ros/melodic")

find_package(Boost REQUIRED COMPONENTS system serialization)

if(ENABLE_EMULATOR)
        find_package(jsoncpp REQUIRED) # install libjsoncpp by running "sudo apt-get install libjsoncpp-dev"
endif(ENABLE_EMULATOR)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib # needed ros::package::getPath()
        sensor_msgs
        diagnostic_updater
        dynamic_reconfigure
        geometry_msgs
        std_msgs
        sensor_msgs
        visualization_msgs
        message_generation
        tf
        tf2
        )


find_package(PkgConfig REQUIRED)



generate_dynamic_reconfigure_options(
        cfg/SickScan.cfg
        cfg/tf_dyn.cfg
)

# driver messages
add_message_files(
        DIRECTORY msg
        FILES
        SickImu.msg
        RadarObject.msg
        RadarPreHeaderDeviceBlock.msg
        RadarPreHeaderMeasurementParam1Block.msg
        RadarPreHeaderStatusBlock.msg
        RadarPreHeaderEncoderBlock.msg
        RadarPreHeader.msg
        RadarScan.msg
        Encoder.msg
        LFErecFieldMsg.msg
        LFErecMsg.msg
        LIDoutputstateMsg.msg
)

# driver services
add_service_files(
        DIRECTORY srv
        FILES
        ColaMsgSrv.srv
        ECRChangeArrSrv.srv
        LIDoutputstateSrv.srv
)

if(ENABLE_EMULATOR)
    # emulator messages
    add_message_files(
        DIRECTORY test/emulator/msg
        FILES
        SickLocColaTelegramMsg.msg
        SickLocDiagnosticMsg.msg
        SickLocResultPortCrcMsg.msg
        SickLocResultPortHeaderMsg.msg
        SickLocResultPortPayloadMsg.msg
        SickLocResultPortTelegramMsg.msg
        SickLocResultPortTestcaseMsg.msg
    )

    # emulator services
    add_service_files(
        DIRECTORY test/emulator/srv
        FILES
        SickLocColaTelegramSrv.srv
        SickLocRequestTimestampSrv.srv
        SickLocSetResultModeSrv.srv
        SickLocSetResultPoseIntervalSrv.srv
        SickLocIsSystemReadySrv.srv
        SickLocSetPoseSrv.srv
        SickLocSetResultPortSrv.srv
        SickLocStartLocalizingSrv.srv
        SickLocStopSrv.srv
        SickLocRequestResultDataSrv.srv
        SickLocSetResultEndiannessSrv.srv
        SickLocSetResultPoseEnabledSrv.srv
        SickLocStateSrv.srv
        SickLocTimeSyncSrv.srv
        SickDevGetLidarConfigSrv.srv
        SickDevGetLidarIdentSrv.srv
        SickDevGetLidarStateSrv.srv
        SickDevSetLidarConfigSrv.srv
        SickGetSoftwareVersionSrv.srv
        SickLocAutoStartActiveSrv.srv
        SickLocAutoStartSavePoseIntervalSrv.srv
        SickLocAutoStartSavePoseSrv.srv
        SickLocForceUpdateSrv.srv
        SickLocInitializePoseSrv.srv
        SickLocInitialPoseSrv.srv
        SickLocMapSrv.srv
        SickLocMapStateSrv.srv
        SickLocOdometryActiveSrv.srv
        SickLocOdometryPortSrv.srv
        SickLocOdometryRestrictYMotionSrv.srv
        SickLocReflectorsForSupportActiveSrv.srv
        SickLocResultEndiannessSrv.srv
        SickLocResultModeSrv.srv
        SickLocResultPortSrv.srv
        SickLocResultPoseIntervalSrv.srv
        SickLocResultStateSrv.srv
        SickLocRingBufferRecordingActiveSrv.srv
        SickLocSaveRingBufferRecordingSrv.srv
        SickLocSetAutoStartActiveSrv.srv
        SickLocSetAutoStartSavePoseIntervalSrv.srv
        SickLocSetMapSrv.srv
        SickLocSetOdometryActiveSrv.srv
        SickLocSetOdometryPortSrv.srv
        SickLocSetOdometryRestrictYMotionSrv.srv
        SickLocSetReflectorsForSupportActiveSrv.srv
        SickLocSetRingBufferRecordingActiveSrv.srv
        SickLocStartDemoMappingSrv.srv
        SickReportUserMessageSrv.srv
        SickSavePermanentSrv.srv
        SickDevSetIMUActiveSrv.srv
        SickDevIMUActiveSrv.srv
    ) 
endif(ENABLE_EMULATOR)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime roscpp sensor_msgs diagnostic_updater dynamic_reconfigure pcl_conversions pcl_ros tf tf2
        LIBRARIES sick_scan_lib
        INCLUDE_DIRS include
        DEPENDS Boost
)

include_directories(include include/tinyxml ${catkin_INCLUDE_DIRS}  include/sick_scan)

add_library(sick_scan_lib
        driver/src/dataDumper.cpp
        driver/src/sick_scan_common.cpp
        driver/src/abstract_parser.cpp
        driver/src/tcp/tcp.cpp
        driver/src/tcp/Mutex.cpp
        driver/src/tcp/SickThread.cpp
        driver/src/tcp/errorhandler.cpp
        driver/src/tcp/toolbox.cpp
        driver/src/tcp/Time.cpp
        driver/src/tcp/colaa.cpp
        driver/src/tcp/colab.cpp
        driver/src/binScanf.cpp
        driver/src/sick_scan_common_tcp.cpp
        driver/src/sick_generic_radar.cpp
        driver/src/sick_generic_imu.cpp
        driver/src/sick_generic_parser.cpp
        driver/src/sick_scan_common_nw.cpp
        include/sick_scan/sick_scan_config_internal.h
        driver/src/sick_scan_config_internal.cpp
        include/sick_scan/softwarePLL.h
        driver/src/softwarePLL.cpp
        driver/src/helper/angle_compensator.cpp
        driver/src/sick_generic_field_mon.cpp
        driver/src/sick_scan_marker.cpp
        driver/src/sick_scan_messages.cpp
        driver/src/sick_scan_services.cpp
        )

add_dependencies(sick_scan_lib ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(sick_scan_lib
        ${catkin_LIBRARIES})

add_executable(sick_generic_caller
        driver/src/sick_generic_laser.cpp
        driver/src/sick_generic_caller.cpp
        )

#
#  radar_object_marker (receives radar msg. and publishes marker array for rviz or similar
#
add_executable(radar_object_marker
        tools/radar_object_marker/src/radar_object_marker.cpp
        tools/pcl_converter/src/gnuplotPaletteReader.cpp
        include/radar_object_marker/radar_object_marker.h)

target_link_libraries(radar_object_marker sick_scan_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#
#
#


#
# pcl_converter disabled to avoid dependency to pcl
#
# add_executable(pcl_converter tools/pcl_converter/src/pcl_converter.cpp tools/pcl_converter/src/gnuplotPaletteReader.cpp)
#
# target_link_libraries(pcl_converter
#        ${catkin_LIBRARIES})


target_link_libraries(sick_generic_caller sick_scan_lib)

add_executable(sick_scan_test
        test/src/sick_scan_test.cpp
        driver/src/tinyxml/tinystr.cpp
        driver/src/tinyxml/tinyxml.cpp
        driver/src/tinyxml/tinyxmlerror.cpp
        driver/src/tinyxml/tinyxmlparser.cpp
        )
target_link_libraries(sick_scan_test
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        sick_scan_lib)

install(TARGETS sick_scan_lib
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
        TARGETS
        sick_generic_caller
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
        TARGETS
        sick_scan_test
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES include/${PROJECT_NAME}/abstract_parser.h
        include/${PROJECT_NAME}/sick_scan_common.h
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY test/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
        FILES_MATCHING PATTERN "*.xml"
        PATTERN "test/src" EXCLUDE)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY meshes/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY urdf/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

if(ENABLE_EMULATOR)
    #
    # emulator
    #
    # message("ENABLE_EMULATOR: build with emulator...")

    add_executable(sick_scan_emulator
        test/emulator/src/test_server.cpp
        test/emulator/src/test_server_thread.cpp
        test/emulator/src/client_socket.cpp
        test/emulator/src/cola_converter.cpp
        test/emulator/src/cola_encoder.cpp
        test/emulator/src/cola_parser.cpp
        test/emulator/src/cola_transmitter.cpp
        test/emulator/src/crc/crc16ccitt_false.cpp
        test/emulator/src/pcapng_json_parser.cpp
        test/emulator/src/random_generator.cpp
        test/emulator/src/result_port_parser.cpp
        test/emulator/src/ros_wrapper.cpp
        test/emulator/src/SoftwarePLL.cpp
        test/emulator/src/testcase_generator.cpp
        test/emulator/src/utils.cpp
        )

    target_link_libraries(sick_scan_emulator
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        jsoncpp_lib # ${jsoncpp_LIBRARIES}
        sick_scan_lib)

    target_compile_definitions(sick_scan_emulator PUBLIC __ROS_VERSION=1)

    target_include_directories(sick_scan_emulator PUBLIC test/emulator/include)

    install(TARGETS sick_scan_emulator
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

    install(DIRECTORY test/emulator/launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

    install(DIRECTORY test/emulator/yaml/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/yaml)

    install(DIRECTORY test/emulator/scandata/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scandata)

endif(ENABLE_EMULATOR)
