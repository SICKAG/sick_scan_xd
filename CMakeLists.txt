####################################################
# CMakeLists.txt for project sick_scan
####################################################
cmake_minimum_required(VERSION 3.5)

project(sick_scan_xd)

# Emulator build options
option(ENABLE_EMULATOR "Build emulator for offline and unittests (ON) or without emulator (OFF)" OFF)
if(CMAKE_ENABLE_EMULATOR EQUAL 1)
    set(ENABLE_EMULATOR ON CACHE BOOL "Build emulator for offline and unittests (ON) or without emulator (OFF)" FORCE) # OFF (release) or ON (development)
    message(STATUS "Option CMAKE_ENABLE_EMULATOR = ${CMAKE_ENABLE_EMULATOR}, building ${PROJECT_NAME} with emulator support, ENABLE_EMULATOR=${ENABLE_EMULATOR}")
else()
    set(ENABLE_EMULATOR OFF CACHE BOOL "Build emulator for offline and unittests (ON) or without emulator (OFF)" FORCE) # OFF (release) or ON (development)
    message(STATUS "Option CMAKE_ENABLE_EMULATOR = ${CMAKE_ENABLE_EMULATOR}, building ${PROJECT_NAME} without emulator support, ENABLE_EMULATOR=${ENABLE_EMULATOR}")
endif()

# LDMRS build options
option(BUILD_WITH_LDMRS_SUPPORT "Build ${PROJECT_NAME} with LDMRS support (ON, libsick_ldmrs from https://github.com/SICKAG/libsick_ldmrs required) or without LDMRS support (OFF)" OFF)
if(WIN32)
    set(BUILD_WITH_LDMRS_SUPPORT OFF CACHE BOOL "Build ${PROJECT_NAME} with LDMRS support (ON, libsick_ldmrs from https://github.com/SICKAG/libsick_ldmrs required) or without LDMRS support (OFF)" FORCE)
elseif(LDMRS EQUAL 0)
    set(BUILD_WITH_LDMRS_SUPPORT OFF CACHE BOOL "Build ${PROJECT_NAME} with LDMRS support (ON, libsick_ldmrs from https://github.com/SICKAG/libsick_ldmrs required) or without LDMRS support (OFF)" FORCE)
else()
    set(BUILD_WITH_LDMRS_SUPPORT ON CACHE BOOL "Build ${PROJECT_NAME} with LDMRS support (ON, libsick_ldmrs from https://github.com/SICKAG/libsick_ldmrs required) or without LDMRS support (OFF)" FORCE)
endif()

# SCANSEGMENT_XD/MULTISCAN136 build options
option(BUILD_WITH_SCANSEGMENT_XD_SUPPORT "Build ${PROJECT_NAME} with SCANSEGMENT_XD support (ON) or without SCANSEGMENT_XD support (OFF)" ON)
if(SCANSEGMENT_XD EQUAL 0)
    set(BUILD_WITH_SCANSEGMENT_XD_SUPPORT OFF CACHE BOOL "Build ${PROJECT_NAME} with SCANSEGMENT_XD support (ON) or without SCANSEGMENT_XD support (OFF)" FORCE)
else()
    set(BUILD_WITH_SCANSEGMENT_XD_SUPPORT ON CACHE BOOL "Build ${PROJECT_NAME} with SCANSEGMENT_XD support (ON) or without SCANSEGMENT_XD support (OFF)" FORCE)
endif()

# Linker option visibilty
option(BUILD_LIB_HIDE_FUNCTIONS "Build ${PROJECT_NAME} library with function hiding except for SickScanApi-functions (ON) or without function hiding (OFF)" ON)
if(VISIBILITY EQUAL 0)
    set(BUILD_LIB_HIDE_FUNCTIONS ON CACHE BOOL "Build ${PROJECT_NAME} library with function hiding except for SickScanApi-functions (ON) or without function hiding (OFF)" FORCE)
else()
    set(BUILD_LIB_HIDE_FUNCTIONS OFF CACHE BOOL "Build ${PROJECT_NAME} library with function hiding except for SickScanApi-functions (ON) or without function hiding (OFF)" FORCE)
endif()

# Debug or Release build options
option(BUILD_DEBUG_TARGET "Build debug target (ON) or release (OFF)" ON) # OFF (release) or ON (development)

# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic")
# if(NOT WIN32) # Note: compiler option -Wshadow generates lots of warnings in ros header files. Therefore it's deactivated by default, but can be usefull for development and testing.
#     add_compile_options(-Wall -Wshadow -Wno-reorder -Wno-sign-compare -Wno-unused-local-typedefs -Wno-unused-parameter -Wno-unused-function -Wno-unused-result -Wno-unused-variable -Wno-unused-but-set-variable)
# endif()

if(NOT WIN32)
    add_compile_options(-O3)
endif()
# Added CMP0022 and CMP0048 to avoid cmake warnings
if (POLICY CMP0022)
    cmake_policy(SET CMP0022 NEW)
endif ()
if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif()


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(${BUILD_DEBUG_TARGET} OR BUILD_DEBUG_TARGET EQUAL ON)
        set(CMAKE_BUILD_TYPE Debug) #uncomment to activate debug mode for lib_sick as well
        set(CMAKE_ENABLE_EXPORTS 1) #uncomment to activate debug mode for lib_sick as well
endif()

# Build sick_scan_xd library hiding all functions except for SickScanApi-functions, see https://gcc.gnu.org/wiki/Visibility for details
if(BUILD_LIB_HIDE_FUNCTIONS AND NOT WIN32)
    add_compile_options(-fvisibility=hidden)
    add_link_options(-fvisibility=hidden)
endif()

# Switch on, if you use c11-specific commands
if(NOT WIN32)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-format-overflow -fno-var-tracking-assignments")
endif()

# Option SICK_TARGET_ENDIANESS sets the endianess of the cmake target to build sick_scan_xd (default is little endian).
# The target endianess can be set either to little endian by cmake option "-DSICK_TARGET_ENDIANESS=1",
# or to big endian by cmake option "-DSICK_TARGET_ENDIANESS=2"
# Since LITTLE_ENDIAN and BIG_ENDIAN might already be defined differently on a system or in a 3rd party headerfile,
# SICK_TARGET_ENDIANESS, SICK_LITTLE_ENDIAN and SICK_BIG_ENDIAN are used to avoid conflicts
if(DEFINED ENV{SICK_TARGET_ENDIANESS})
    add_compile_options(-DSICK_TARGET_ENDIANESS=$ENV{SICK_TARGET_ENDIANESS})
elseif(DEFINED SICK_TARGET_ENDIANESS)
    add_compile_options(-DSICK_TARGET_ENDIANESS=${SICK_TARGET_ENDIANESS})
endif()

# ROS Version
message(STATUS "ROS_VERSION precheck: ENV{ROS_VERSION}=$ENV{ROS_VERSION} ENV{ROS_DISTRO}=$ENV{ROS_DISTRO} ROS_VERSION=${ROS_VERSION} CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}")
if(DEFINED CMAKE_PREFIX_PATH AND "${CMAKE_PREFIX_PATH}" STREQUAL "/opt/ros/humble")
    set(ROS_VERSION 2)  # required for ROS2 humble bloom releases with jenkins buildfarm
elseif(NOT DEFINED ROS_VERSION AND EXISTS "/opt/ros/humble/setup.sh")
    set(ROS_VERSION 2)  # required for ROS2 humble bloom releases with jenkins buildfarm
endif()
if(NOT DEFINED ROS_VERSION AND DEFINED ENV{ROS_VERSION})
    set(ROS_VERSION $ENV{ROS_VERSION})
endif()
message(STATUS "ROS_VERSION=${ROS_VERSION}")
add_compile_options(-D__ROS_VERSION=${ROS_VERSION})
if(ROS_VERSION EQUAL 0)
    add_compile_options(-DROSSIMU)
endif()

if(WIN32 AND EXISTS "c:/vcpkg/installed/x64-windows")
    include_directories(c:/vcpkg/installed/x64-windows/include)
    link_directories(c:/vcpkg/installed/x64-windows/lib)
endif()

if(WIN32)
    add_compile_options(-D_WINSOCK_DEPRECATED_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_WIN32_WINNT=0x0A00)
    link_libraries(ws2_32)    
endif()

if(RASPBERRY EQUAL 1)
    add_compile_options(-DRASPBERRY=1)
    message(STATUS "Building sick_scan_xd for target RASPBERRY")
else()
    add_compile_options(-DRASPBERRY=0)
endif()


# Checks and prints a message, if a header file does not exist
function(check_header_exists headerfilepath headerfilename packagename)
    if(NOT EXISTS ${headerfilepath})
        message(STATUS "${packagename} check: file ${headerfilepath} not found.")
        message(STATUS "If you encounter problems with missing ${headerfilename}, please checkout the following repositories:")
        message(STATUS "git clone https://github.com/SICKAG/libsick_ldmrs.git")
        message(STATUS "git clone https://github.com/SICKAG/msgpack11.git")
        message(STATUS "git clone -b master https://github.com/SICKAG/sick_scan_xd.git")
        message(STATUS "and rebuild. See the build instructions in INSTALL-GENERIC.md, INSTALL-ROS1.md and INSTALL-ROS2.md for further details.")
    endif()
endfunction()

# By uncommenting the following lines you can list all variable names ...
# --- CUT ---
# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#   message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
# --- CUT ---

# jsoncpp required for emulator and lidar3d
if(${ENABLE_EMULATOR} OR ENABLE_EMULATOR EQUAL ON)
    # emulator requires jsoncpp for pcapng and other files
    find_package(jsoncpp REQUIRED) # install libjsoncpp by running "sudo apt-get install libjsoncpp-dev" (Linux) resp. "vcpkg install jsoncpp:x64-windows" (Windows) 
    if(WIN32)
      set(LIB_JSONCPP jsoncpp_lib)
    else()
      set(LIB_JSONCPP jsoncpp_lib)
    endif()
endif()

if(ROS_VERSION EQUAL 1)
    find_package(catkin REQUIRED COMPONENTS
        diagnostic_updater
        dynamic_reconfigure
        geometry_msgs
        message_generation
        nav_msgs
        roscpp
        roslib # needed ros::package::getPath()
        sensor_msgs
        std_msgs
        tf
        tf2
        visualization_msgs
    )

    find_package(PkgConfig REQUIRED)

    generate_dynamic_reconfigure_options(
        cfg/SickLDMRSDriver.cfg
        cfg/SickScan.cfg
        cfg/tf_dyn.cfg
    )

    # driver messages
    add_message_files(
        DIRECTORY msg
        FILES
        Encoder.msg
        LFErecFieldMsg.msg
        LFErecMsg.msg
        LIDoutputstateMsg.msg
        RadarObject.msg
        RadarPreHeader.msg
        RadarPreHeaderDeviceBlock.msg
        RadarPreHeaderEncoderBlock.msg
        RadarPreHeaderMeasurementParam1Block.msg
        RadarPreHeaderStatusBlock.msg
        RadarScan.msg
        SickImu.msg
        NAVLandmarkData.msg
        NAVOdomVelocity.msg
        NAVPoseData.msg
        NAVReflectorData.msg
    )
    add_message_files(
        DIRECTORY msg/ldmrs
        FILES
        SickLdmrsObject.msg
        SickLdmrsObjectArray.msg
    )

    # driver services
    add_service_files(
        DIRECTORY srv
        FILES
        ColaMsgSrv.srv
        ECRChangeArrSrv.srv
        GetContaminationResultSrv.srv
        LIDoutputstateSrv.srv
        SCdevicestateSrv.srv
        SCrebootSrv.srv
        SCsoftresetSrv.srv
        SickScanExitSrv.srv
    )
    set(diagnostic_updater_pkg diagnostic_updater)
    add_compile_options(-DROS_DIAGNOSTICS_UPDATER_AVAILABLE)
endif()

if(ROS_VERSION EQUAL 2)
    #uncomment this lines and change path accordingly
    set(FastRTPS_INCLUDE_DIR /opt/ros/$ENV{ROS_DISTRO}/include)
    set(FastRTPS_LIBRARY_RELEASE /opt/ros/$ENV{ROS_DISTRO}/lib/libfastrtps.so)

    find_package(ament_cmake REQUIRED)
    if(NOT WIN32)
        # still check if diagnostics updater is available (depends on ros distribution)
        find_package(diagnostic_updater QUIET)
        if (${diagnostic_updater_FOUND})
            set(diagnostic_updater_pkg diagnostic_updater)
            add_compile_options(-DROS_DIAGNOSTICS_UPDATER_AVAILABLE)
        endif()
    endif()
    find_package(diagnostic_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(visualization_msgs REQUIRED)
    if(${ENABLE_EMULATOR} OR ENABLE_EMULATOR EQUAL ON)
        set(ROSIDL_EMULATOR_FILES
            # emulator messages
            "test/emulator/msg/SickLocColaTelegramMsg.msg"
            "test/emulator/msg/SickLocDiagnosticMsg.msg"
            "test/emulator/msg/SickLocResultPortCrcMsg.msg"
            "test/emulator/msg/SickLocResultPortHeaderMsg.msg"
            "test/emulator/msg/SickLocResultPortPayloadMsg.msg"
            "test/emulator/msg/SickLocResultPortTelegramMsg.msg"
            "test/emulator/msg/SickLocResultPortTestcaseMsg.msg"
            # emulator services
            "test/emulator/srv/SickLocColaTelegramSrv.srv"
            "test/emulator/srv/SickLocRequestTimestampSrv.srv"
            "test/emulator/srv/SickLocSetResultModeSrv.srv"
            "test/emulator/srv/SickLocSetResultPoseIntervalSrv.srv"
            "test/emulator/srv/SickLocIsSystemReadySrv.srv"
            "test/emulator/srv/SickLocSetPoseSrv.srv"
            "test/emulator/srv/SickLocSetResultPortSrv.srv"
            "test/emulator/srv/SickLocStartLocalizingSrv.srv"
            "test/emulator/srv/SickLocStopSrv.srv"
            "test/emulator/srv/SickLocRequestResultDataSrv.srv"
            "test/emulator/srv/SickLocSetResultEndiannessSrv.srv"
            "test/emulator/srv/SickLocSetResultPoseEnabledSrv.srv"
            "test/emulator/srv/SickLocStateSrv.srv"
            "test/emulator/srv/SickLocTimeSyncSrv.srv"
            "test/emulator/srv/SickDevGetLidarConfigSrv.srv"
            "test/emulator/srv/SickDevGetLidarIdentSrv.srv"
            "test/emulator/srv/SickDevGetLidarStateSrv.srv"
            "test/emulator/srv/SickDevSetLidarConfigSrv.srv"
            "test/emulator/srv/SickGetSoftwareVersionSrv.srv"
            "test/emulator/srv/SickLocAutoStartActiveSrv.srv"
            "test/emulator/srv/SickLocAutoStartSavePoseIntervalSrv.srv"
            "test/emulator/srv/SickLocAutoStartSavePoseSrv.srv"
            "test/emulator/srv/SickLocForceUpdateSrv.srv"
            "test/emulator/srv/SickLocInitializePoseSrv.srv"
            "test/emulator/srv/SickLocInitialPoseSrv.srv"
            "test/emulator/srv/SickLocMapSrv.srv"
            "test/emulator/srv/SickLocMapStateSrv.srv"
            "test/emulator/srv/SickLocOdometryActiveSrv.srv"
            "test/emulator/srv/SickLocOdometryPortSrv.srv"
            "test/emulator/srv/SickLocOdometryRestrictYMotionSrv.srv"
            "test/emulator/srv/SickLocReflectorsForSupportActiveSrv.srv"
            "test/emulator/srv/SickLocResultEndiannessSrv.srv"
            "test/emulator/srv/SickLocResultModeSrv.srv"
            "test/emulator/srv/SickLocResultPortSrv.srv"
            "test/emulator/srv/SickLocResultPoseIntervalSrv.srv"
            "test/emulator/srv/SickLocResultStateSrv.srv"
            "test/emulator/srv/SickLocRingBufferRecordingActiveSrv.srv"
            "test/emulator/srv/SickLocSaveRingBufferRecordingSrv.srv"
            "test/emulator/srv/SickLocSetAutoStartActiveSrv.srv"
            "test/emulator/srv/SickLocSetAutoStartSavePoseIntervalSrv.srv"
            "test/emulator/srv/SickLocSetMapSrv.srv"
            "test/emulator/srv/SickLocSetOdometryActiveSrv.srv"
            "test/emulator/srv/SickLocSetOdometryPortSrv.srv"
            "test/emulator/srv/SickLocSetOdometryRestrictYMotionSrv.srv"
            "test/emulator/srv/SickLocSetReflectorsForSupportActiveSrv.srv"
            "test/emulator/srv/SickLocSetRingBufferRecordingActiveSrv.srv"
            "test/emulator/srv/SickLocStartDemoMappingSrv.srv"
            "test/emulator/srv/SickReportUserMessageSrv.srv"
            "test/emulator/srv/SickSavePermanentSrv.srv"
            "test/emulator/srv/SickDevSetIMUActiveSrv.srv"
            "test/emulator/srv/SickDevIMUActiveSrv.srv"
        )
    endif()
    rosidl_generate_interfaces(${PROJECT_NAME}
        # message files
        "msg/Encoder.msg"
        "msg/ImuExtended.msg"
        "msg/LFErecFieldMsg.msg"
        "msg/LFErecMsg.msg"
        "msg/LIDoutputstateMsg.msg"
        "msg/NAVLandmarkData.msg"
        "msg/NAVOdomVelocity.msg"
        "msg/NAVPoseData.msg"
        "msg/NAVReflectorData.msg"        
        "msg/ros2/RadarObject.msg"
        "msg/RadarPreHeader.msg"
        "msg/RadarPreHeaderDeviceBlock.msg"
        "msg/RadarPreHeaderEncoderBlock.msg"
        "msg/RadarPreHeaderMeasurementParam1Block.msg"
        "msg/RadarPreHeaderStatusBlock.msg"
        "msg/RadarScan.msg"
        "msg/SickImu.msg"
        "msg/ros2/ldmrs/SickLdmrsObject.msg"
        "msg/ros2/ldmrs/SickLdmrsObjectArray.msg"
        # service files
        "srv/ColaMsgSrv.srv"
        "srv/ECRChangeArrSrv.srv"
        "srv/GetContaminationResultSrv.srv"
        "srv/LIDoutputstateSrv.srv"
        "srv/SCdevicestateSrv.srv"
        "srv/SCrebootSrv.srv"
        "srv/SCsoftresetSrv.srv"
        "srv/SickScanExitSrv.srv"
        ${ROSIDL_EMULATOR_FILES}
        DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs
    )
endif()

# Support for LDMRS
if(RASPBERRY EQUAL 1)
    message(STATUS "Building ${PROJECT_NAME} for raspberry without ldmrs support")
elseif(${BUILD_WITH_LDMRS_SUPPORT} OR BUILD_WITH_LDMRS_SUPPORT EQUAL ON)
    message(STATUS "Building ${PROJECT_NAME} with LDMRS support")
    find_package(SickLDMRS QUIET)    
    if (${SickLDMRS_FOUND})    
        add_compile_options(-DLDMRS_SUPPORT=1)
        set(LDMRS_INCLUDES ${SICK_LDMRS_INCLUDE_DIRS})
        set(LDMRS_SOURCES driver/src/ldmrs/sick_ldmrs_config.cpp driver/src/ldmrs/sick_ldmrs_driver.cpp driver/src/ldmrs/sick_ldmrs_node.cpp)
        set(LDMRS_TARGET_DEPENDENCIES SickLDMRS)
        message(STATUS "LDMRS_INCLUDES: ${LDMRS_INCLUDES}")
        message(STATUS "LDMRS_SOURCES: ${LDMRS_SOURCES}")
        message(STATUS "LDMRS_TARGET_DEPENDENCIES: ${LDMRS_TARGET_DEPENDENCIES}")
        message(STATUS "SICK_LDMRS_LIBRARIES: ${SICK_LDMRS_LIBRARIES}")
        check_header_exists(${LDMRS_INCLUDES}/sick_ldmrs/manager.hpp sick_ldmrs/manager.hpp sick_ldmrs)
    else()
        message(STATUS "package SickLDMR not found, ldmrs support disabled")
        add_compile_options(-DLDMRS_SUPPORT=0)
    endif()
else()
    message(STATUS "Building ${PROJECT_NAME} without ldmrs support")
endif()

# Support for SCANSEGMENT_XD/MULTISCAN136
if(${BUILD_WITH_SCANSEGMENT_XD_SUPPORT} OR BUILD_WITH_SCANSEGMENT_XD_SUPPORT EQUAL ON)
    message(STATUS "Building ${PROJECT_NAME} with SCANSEGMENT_XD support")
    add_compile_options(-DSCANSEGMENT_XD_SUPPORT=1)
    # msgpack11 sources and header required for SCANSEGMENT_XD support
    include_directories(include/sick_scansegment_xd/msgpack11)
    file(GLOB SCANSEGMENT_XD_SOURCES driver/src/sick_scansegment_xd/*.cpp driver/src/sick_scansegment_xd/msgpack11/*.cpp)
else()
    message(STATUS "Building ${PROJECT_NAME} without SCANSEGMENT_XD support")
endif()
 
if((${ENABLE_EMULATOR} OR ENABLE_EMULATOR EQUAL ON) AND ROS_VERSION EQUAL 1)
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
        SickDevGetLidarConfigSrv.srv
        SickDevGetLidarIdentSrv.srv
        SickDevGetLidarStateSrv.srv
        SickDevIMUActiveSrv.srv
        SickDevSetIMUActiveSrv.srv
        SickDevSetLidarConfigSrv.srv
        SickGetSoftwareVersionSrv.srv
        SickLocAutoStartActiveSrv.srv
        SickLocAutoStartSavePoseIntervalSrv.srv
        SickLocAutoStartSavePoseSrv.srv
        SickLocColaTelegramSrv.srv
        SickLocForceUpdateSrv.srv
        SickLocInitialPoseSrv.srv
        SickLocInitializePoseSrv.srv
        SickLocIsSystemReadySrv.srv
        SickLocMapSrv.srv
        SickLocMapStateSrv.srv
        SickLocOdometryActiveSrv.srv
        SickLocOdometryPortSrv.srv
        SickLocOdometryRestrictYMotionSrv.srv
        SickLocReflectorsForSupportActiveSrv.srv
        SickLocRequestResultDataSrv.srv
        SickLocRequestTimestampSrv.srv
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
        SickLocSetPoseSrv.srv
        SickLocSetReflectorsForSupportActiveSrv.srv
        SickLocSetResultEndiannessSrv.srv
        SickLocSetResultModeSrv.srv
        SickLocSetResultPortSrv.srv
        SickLocSetResultPoseEnabledSrv.srv
        SickLocSetResultPoseIntervalSrv.srv
        SickLocSetRingBufferRecordingActiveSrv.srv
        SickLocStartDemoMappingSrv.srv
        SickLocStartLocalizingSrv.srv
        SickLocStateSrv.srv
        SickLocStopSrv.srv
        SickLocTimeSyncSrv.srv
        SickReportUserMessageSrv.srv
        SickSavePermanentSrv.srv
    ) 
endif()

if(ROS_VERSION EQUAL 1)
    generate_messages(
        DEPENDENCIES
        geometry_msgs
        nav_msgs
        sensor_msgs
        std_msgs
    )

    catkin_package(
        CATKIN_DEPENDS
            diagnostic_msgs
            ${diagnostic_updater_pkg}
            dynamic_reconfigure
            geometry_msgs
            message_runtime
            nav_msgs
            # pcl_conversions
            # pcl_ros
            roscpp
            sensor_msgs
            std_msgs
            tf
            tf2
            # tf tf2
            visualization_msgs
        LIBRARIES ${PROJECT_NAME}_lib ${PROJECT_NAME}_shared_lib
        INCLUDE_DIRS include
    )
endif()

include_directories(include include/sick_scan_xd_api include/tinyxml ${catkin_INCLUDE_DIRS} ${LDMRS_INCLUDES} include/sick_scan tools/test_server/include roswrap/src/include/launchparser)
if(ROS_VERSION EQUAL 0)
    include_directories(roswrap/src/msg_header)
endif()

if(ROS_VERSION EQUAL 1)

    if(WIN32)
        include_directories(roswrap/helper_win)
        file(GLOB SRC_WIN_FILES roswrap/helper_win/usleep/usleep.c)
    endif()
    set(SICK_SCAN_LIB_SRC
        driver/src/abstract_parser.cpp
        driver/src/binPrintf.cpp
        driver/src/binScanf.cpp
        driver/src/dataDumper.cpp
        driver/src/helper/angle_compensator.cpp
        driver/src/sick_cloud_transform.cpp
        driver/src/sick_generic_callback.cpp
        driver/src/sick_generic_field_mon.cpp
        driver/src/sick_generic_imu.cpp
        driver/src/sick_generic_laser.cpp
        driver/src/sick_generic_monitoring.cpp
        driver/src/sick_generic_parser.cpp
        driver/src/sick_generic_radar.cpp
        driver/src/sick_lmd_scandata_parser.cpp
        driver/src/sick_scan_common.cpp
        driver/src/sick_scan_common_nw.cpp
        driver/src/sick_scan_common_tcp.cpp
        driver/src/sick_scan_config_internal.cpp
        driver/src/sick_scan_marker.cpp
        driver/src/sick_scan_messages.cpp
        driver/src/sick_scan_parse_util.cpp
        driver/src/sick_scan_services.cpp
        driver/src/sick_tf_publisher.cpp
        driver/src/sick_scan_xd_api/api_impl.cpp
        driver/src/sick_scan_xd_api/sick_scan_api_converter.cpp
        driver/src/sick_nav_scandata_parser.cpp
        driver/src/softwarePLL.cpp
        driver/src/tcp/Mutex.cpp
        driver/src/tcp/SickThread.cpp
        driver/src/tcp/Time.cpp
        driver/src/tcp/colaa.cpp
        driver/src/tcp/colab.cpp
        driver/src/tcp/errorhandler.cpp
        driver/src/tcp/tcp.cpp
        driver/src/tcp/toolbox.cpp
        driver/src/tcp/wsa_init.cpp
        driver/src/tinyxml/tinystr.cpp
        driver/src/tinyxml/tinyxml.cpp
        driver/src/tinyxml/tinyxmlerror.cpp
        driver/src/tinyxml/tinyxmlparser.cpp
        roswrap/src/launchparser/launchparser.cpp
        ${LDMRS_SOURCES}
        ${SCANSEGMENT_XD_SOURCES}
        ${SRC_WIN_FILES}
    )
    add_library(${PROJECT_NAME}_lib STATIC ${SICK_SCAN_LIB_SRC})
    # target_compile_options(sick_scan_lib PUBLIC "-fPIC")
    add_library(${PROJECT_NAME}_shared_lib SHARED ${SICK_SCAN_LIB_SRC})

    add_dependencies(${PROJECT_NAME}_lib
        ${PROJECT_NAME}_gencfg
        ${catkin_EXPORTED_TARGETS}
        ${${PROJECT_NAME}_EXPORTED_TARGETS})
    add_dependencies(${PROJECT_NAME}_shared_lib
        ${PROJECT_NAME}_gencfg
        ${catkin_EXPORTED_TARGETS}
        ${${PROJECT_NAME}_EXPORTED_TARGETS})

    target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})
    target_link_libraries(${PROJECT_NAME}_shared_lib ${catkin_LIBRARIES})

    add_executable(sick_generic_caller driver/src/sick_generic_caller.cpp)
    
    target_link_libraries(sick_generic_caller ${PROJECT_NAME}_lib ${SICK_LDMRS_LIBRARIES})
    
    #
    #  radar_object_marker (receives radar msg. and publishes marker array for rviz or similar
    #
    # add_executable(radar_object_marker
    #     tools/radar_object_marker/src/radar_object_marker.cpp
    #     tools/pcl_converter/src/gnuplotPaletteReader.cpp
    #     include/radar_object_marker/radar_object_marker.h)
    # target_link_libraries(radar_object_marker ${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

    #
    # pcl_converter disabled to avoid dependency to pcl
    #
    # add_executable(pcl_converter tools/pcl_converter/src/pcl_converter.cpp tools/pcl_converter/src/gnuplotPaletteReader.cpp)
    # target_link_libraries(pcl_converter ${catkin_LIBRARIES})

    # add_executable(${PROJECT_NAME}_test test/src/${PROJECT_NAME}_test.cpp)
    # target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${roslib_LIBRARIES} ${PROJECT_NAME}_lib ${Boost_LIBRARIES})

else() # i.e. (ROS_VERSION EQUAL 0 OR ROS_VERSION EQUAL 2)

    include_directories(roswrap/src/include)
    if(WIN32)
        include_directories(roswrap/helper_win)
        file(GLOB SRC_WIN_FILES roswrap/helper_win/usleep/usleep.c)
    endif()
    if(ROS_VERSION EQUAL 0)
        include_directories(roswrap/src/include roswrap/src/rossimu/melodic/include roswrap/src/rossimu/kinetic/include roswrap/src/cfgsimu roswrap/src/toojpeg roswrap/src/tools)
        file(GLOB SRC_ROSSIMU_FILES 
            roswrap/src/cfgsimu/sick_scan/time_modi.cpp
            roswrap/src/rossimu/kinetic/src/rossimu.cpp
            roswrap/src/rossimu/kinetic/src/duration.cpp
            roswrap/src/rossimu/kinetic/src/rate.cpp
            roswrap/src/tools/sick_scan/pointcloud_utils.cpp
        )
    endif()

    set(SICK_SCAN_LIB_SRC
        driver/src/abstract_parser.cpp
        driver/src/binPrintf.cpp
        driver/src/binScanf.cpp
        driver/src/dataDumper.cpp
        driver/src/helper/angle_compensator.cpp
        driver/src/sick_cloud_transform.cpp
        driver/src/sick_generic_callback.cpp
        driver/src/sick_generic_field_mon.cpp
        driver/src/sick_generic_imu.cpp
        driver/src/sick_nav_scandata_parser.cpp        
        driver/src/sick_generic_laser.cpp
        driver/src/sick_generic_monitoring.cpp
        driver/src/sick_generic_parser.cpp
        driver/src/sick_generic_radar.cpp
        driver/src/sick_lmd_scandata_parser.cpp
        driver/src/sick_scan_common.cpp
        driver/src/sick_scan_common_nw.cpp
        driver/src/sick_scan_common_tcp.cpp
        driver/src/sick_scan_marker.cpp
        driver/src/sick_scan_messages.cpp
        driver/src/sick_scan_parse_util.cpp
        driver/src/sick_scan_services.cpp
        driver/src/sick_tf_publisher.cpp
        driver/src/sick_scan_xd_api/api_impl.cpp
        driver/src/sick_scan_xd_api/sick_scan_api_converter.cpp
        driver/src/softwarePLL.cpp
        driver/src/tcp/Mutex.cpp
        driver/src/tcp/SickThread.cpp
        driver/src/tcp/Time.cpp
        driver/src/tcp/colaa.cpp
        driver/src/tcp/colab.cpp
        driver/src/tcp/errorhandler.cpp
        driver/src/tcp/tcp.cpp
        driver/src/tcp/toolbox.cpp
        driver/src/tcp/wsa_init.cpp
        driver/src/tinyxml/tinystr.cpp
        driver/src/tinyxml/tinyxml.cpp
        driver/src/tinyxml/tinyxmlerror.cpp
        driver/src/tinyxml/tinyxmlparser.cpp
        roswrap/src/getopt/getopt.c
        roswrap/src/launchparser/launchparser.cpp
        roswrap/src/toojpeg/toojpeg.cpp
        ${LDMRS_SOURCES}
        ${SCANSEGMENT_XD_SOURCES}
        ${SRC_ROSSIMU_FILES}
        ${SRC_WIN_FILES}
    )
    add_library(${PROJECT_NAME}_lib STATIC ${SICK_SCAN_LIB_SRC})
    # target_compile_options(sick_scan_lib PUBLIC "-fPIC")
    add_library(${PROJECT_NAME}_shared_lib SHARED ${SICK_SCAN_LIB_SRC})
    if(ROS_VERSION EQUAL 0)    
        # target_compile_options(${PROJECT_NAME}_shared_lib PUBLIC "-fPIC")
        # target_link_options(${PROJECT_NAME}_shared_lib PUBLIC "-fPIC")
        target_link_libraries(${PROJECT_NAME}_shared_lib ${SICK_LDMRS_LIBRARIES})
    endif()
    add_executable(sick_generic_caller driver/src/sick_generic_caller.cpp)
    if(ROS_VERSION EQUAL 0 AND NOT WIN32)
        target_link_libraries(sick_generic_caller ${PROJECT_NAME}_lib ${SICK_LDMRS_LIBRARIES} "pthread") # pthread required for std::thread
        target_link_options(sick_generic_caller PUBLIC "LINKER:--no-as-needed") # fixes exception "Enable multithreading to use std::thread: Operation not permitted" 
    else()    
        target_link_libraries(sick_generic_caller ${PROJECT_NAME}_lib ${SICK_LDMRS_LIBRARIES})
    endif()

endif()

if(ROS_VERSION EQUAL 2)

    ament_target_dependencies(
        ${PROJECT_NAME}_lib
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "geometry_msgs"
        "diagnostic_msgs" 
        "nav_msgs"
        "visualization_msgs"
        "tf2_ros"
        "${diagnostic_updater_pkg}"
        ${LDMRS_TARGET_DEPENDENCIES}
    )

    ament_target_dependencies(
        ${PROJECT_NAME}_shared_lib
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "geometry_msgs"
        "diagnostic_msgs" 
        "nav_msgs"
        "visualization_msgs"
        "tf2_ros"
        "${diagnostic_updater_pkg}"
        ${LDMRS_TARGET_DEPENDENCIES}
    )

    ament_target_dependencies(
        sick_generic_caller
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "geometry_msgs"
        "diagnostic_msgs"
        "nav_msgs"
        "visualization_msgs"
        "tf2_ros"
        "${diagnostic_updater_pkg}"
        ${LDMRS_TARGET_DEPENDENCIES}
    )

    if(WIN32 OR EXISTS "/opt/ros/eloquent" OR EXISTS "/opt/ros/foxy" OR EXISTS "/opt/ros/galactic") # rosidl_typesupport for ROS2 eloquent, foxy, galaxy
        rosidl_target_interfaces(${PROJECT_NAME}_lib ${PROJECT_NAME} "rosidl_typesupport_c")
        rosidl_target_interfaces(${PROJECT_NAME}_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
        rosidl_target_interfaces(${PROJECT_NAME}_shared_lib ${PROJECT_NAME} "rosidl_typesupport_c")
        rosidl_target_interfaces(${PROJECT_NAME}_shared_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
        rosidl_target_interfaces(sick_generic_caller ${PROJECT_NAME} "rosidl_typesupport_c") # required?
        rosidl_target_interfaces(sick_generic_caller ${PROJECT_NAME} "rosidl_typesupport_cpp") # required?
    else() # rosidl_typesupport for ROS2 humble or later
        rosidl_get_typesupport_target(c_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_c")
        rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
        target_link_libraries(${PROJECT_NAME}_lib "${c_typesupport_target}")
        target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")
        target_link_libraries(${PROJECT_NAME}_shared_lib "${p_typesupport_target}")
        target_link_libraries(${PROJECT_NAME}_shared_lib "${cpp_typesupport_target}")
        target_link_libraries(sick_generic_caller "${c_typesupport_target}")
        target_link_libraries(sick_generic_caller "${cpp_typesupport_target}")
    endif()

endif()

# Build test and usage example for sick_scan_xd_api
# For ROS-1 and ROS-2:
# - sick_scan_xd_api_test with is deactivated by default
#   to avoid phtread dependencies for bloom releases
# - sick_scan_xd_api_test for development and test only
# Native Windows and Linux (ROS_VERSION==0): 
# - build sick_scan_xd_api_test (development, test and usage example)
if(ROS_VERSION EQUAL 0) # build API test on native Windows or Linux
    set(BUILD_SICK_SCAN_XD_API_TEST 1)
elseif(${ENABLE_EMULATOR} OR ENABLE_EMULATOR EQUAL ON) # i.e. development
    set(BUILD_SICK_SCAN_XD_API_TEST 1)
else()
    set(BUILD_SICK_SCAN_XD_API_TEST 0)
endif()
if(BUILD_SICK_SCAN_XD_API_TEST EQUAL 1)
    add_executable(sick_scan_xd_api_test
        driver/src/sick_scan_xd_api/sick_scan_api_converter.cpp
        test/src/sick_scan_xd_api/sick_scan_xd_api_test.cpp
        test/src/sick_scan_xd_api/sick_scan_xd_api_wrapper.c
        test/src/sick_scan_xd_api/toojpeg/toojpeg.cpp
    )
    target_include_directories(sick_scan_xd_api_test PUBLIC test/src/sick_scan_xd_api/toojpeg)
    if(NOT WIN32)
        target_link_libraries(sick_scan_xd_api_test "dl") # link with dl for dynamic library loading
        if(ROS_VERSION EQUAL 0)
            target_link_libraries(sick_scan_xd_api_test "pthread") # pthread required for std::thread
            target_link_options(sick_scan_xd_api_test PUBLIC "LINKER:--no-as-needed") # fixes exception "Enable multithreading to use std::thread: Operation not permitted" 
        elseif(ROS_VERSION EQUAL 1)
            target_link_libraries(sick_scan_xd_api_test ${catkin_LIBRARIES})
            add_dependencies(sick_scan_xd_api_test ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
        elseif(ROS_VERSION EQUAL 2)
            target_link_libraries(sick_scan_xd_api_test "pthread") # pthread required for std::thread
        endif()
    endif()
endif()

# install sick_scan_xd_shared_lib incl. API headerfiles
if(ROS_VERSION EQUAL 0)
    include(GNUInstallDirs)
    set_target_properties(${PROJECT_NAME}_shared_lib PROPERTIES PUBLIC_HEADER "include/sick_scan_xd_api/sick_scan_api.h;python/api/sick_scan_api.py")
    install(TARGETS ${PROJECT_NAME}_shared_lib
        EXPORT sick_scan_xd_Export
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/sick_scan_xd"
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    # Export sick_scan_xd_shared_lib target to be used by other projects
    install(EXPORT sick_scan_xd_Export
        FILE sick_scan_xd-config.cmake
        NAMESPACE sick_scan_xd::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sick_scan_xd
    )
endif()

if(ROS_VERSION EQUAL 1)
    install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_shared_lib
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

    install(TARGETS sick_generic_caller
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

    if(BUILD_SICK_SCAN_XD_API_TEST EQUAL 1)
        install(TARGETS sick_scan_xd_api_test
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    endif()

    install(FILES include/sick_scan/abstract_parser.h
        include/sick_scan/sick_scan_common.h
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
endif()

if(ROS_VERSION EQUAL 2)

    if(BUILD_SICK_SCAN_XD_API_TEST EQUAL 1)
        install(TARGETS sick_scan_xd_api_test DESTINATION lib/${PROJECT_NAME})
    endif()
    install(TARGETS sick_generic_caller DESTINATION lib/${PROJECT_NAME})
    install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_shared_lib DESTINATION lib)
    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
        
    ament_package()        
endif()

#
# build and install emulator
#
if((${ENABLE_EMULATOR} OR ENABLE_EMULATOR EQUAL ON) AND (NOT WIN32 OR ROS_VERSION EQUAL 2)) # sick_scan_emulator not supported on native Windows
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
        test/emulator/src/server_socket.cpp
        test/emulator/src/test_server.cpp
        test/emulator/src/test_server_thread.cpp
        test/emulator/src/testcase_generator.cpp
        test/emulator/src/utils.cpp
        ${SRC_ROSSIMU_FILES}
        ${SRC_WIN_FILES}
        )
    target_link_libraries(sick_scan_emulator
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${PROJECT_NAME}_lib 
        ${SICK_LDMRS_LIBRARIES}
        ${LIB_JSONCPP})
    target_include_directories(sick_scan_emulator PUBLIC test test/emulator/include)
    if(NOT WIN32 AND NOT ROS_VERSION EQUAL 1)
        target_link_libraries(sick_scan_emulator "pthread") # pthread required for std::thread
    endif()    
    if(ROS_VERSION EQUAL 1)
        install(TARGETS sick_scan_emulator
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        install(DIRECTORY test/emulator/launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
        install(DIRECTORY test/emulator/yaml/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/yaml)
        install(DIRECTORY test/emulator/scandata/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scandata)
    else()
        install(TARGETS sick_scan_emulator
            RUNTIME DESTINATION lib/${PROJECT_NAME})
        install(DIRECTORY test/emulator/launch/
            DESTINATION share/${PROJECT_NAME})
        install(DIRECTORY test/emulator/yaml/
            DESTINATION share/${PROJECT_NAME})
        install(DIRECTORY test/emulator/scandata/
            DESTINATION share/${PROJECT_NAME})
    endif()
endif()

#
# build and install test_server
#
if(${ENABLE_EMULATOR} OR ENABLE_EMULATOR EQUAL ON)
    add_executable(test_server 
        test/emulator/src/server_socket.cpp
        tools/test_server/src/test_server.cpp
        tools/test_server/src/test_server_cola_msg.cpp
        tools/test_server/src/test_server_ldmrs_msg.cpp
        tools/test_server/src/test_server_thread.cpp)
    target_link_libraries(test_server ${PROJECT_NAME}_lib ${SICK_LDMRS_LIBRARIES} ${LIB_JSONCPP})
    target_include_directories(test_server PUBLIC test test/emulator/include)
    if(NOT WIN32 AND NOT ROS_VERSION EQUAL 1)
        target_link_libraries(test_server "pthread") # pthread required for std::thread
    endif()    
    if(ROS_VERSION EQUAL 2)
        ament_target_dependencies(test_server "rclcpp" )
    endif()
    install(TARGETS 
        test_server
        DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY 
        tools/test_server/config
        tools/test_server/launch
        DESTINATION share/${PROJECT_NAME})
endif()
