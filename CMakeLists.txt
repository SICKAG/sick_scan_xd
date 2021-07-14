####################################################
# CMakeLists.txt for project sick_scan_xd
####################################################
cmake_minimum_required(VERSION 3.5)

# build options: set OFF for relese version, ON for development and test
option(ENABLE_EMULATOR "Build emulator for offline and unittests" ON) # OFF (release) or ON (development)
if(WIN32)
    option(BUILD_WITH_LDMRS_SUPPORT "Build sick_scan_xd with LDMRS support (requires PCL and libsick_ldmrs from https://github.com/SICKAG/libsick_ldmrs" OFF) # OFF (release) or ON (development)
else()
    option(BUILD_WITH_LDMRS_SUPPORT "Build sick_scan_xd with LDMRS support (requires PCL and libsick_ldmrs from https://github.com/SICKAG/libsick_ldmrs" ON) # OFF (release) or ON (development)
endif()
option(BUILD_DEBUG_TARGET "Build debug target" ON) # OFF (release) or ON (development)

# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic")

# Added CMP0048 to avoid unstable warning of build process
if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(sick_scan)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(BUILD_DEBUG_TARGET)
        set(CMAKE_BUILD_TYPE Debug) #uncomment to activate debug mode for lib_sick as well
        set(CMAKE_ENABLE_EXPORTS 1) #uncomment to activate debug mode for lib_sick as well
endif(BUILD_DEBUG_TARGET)

# Switch on, if you use c11-specific commands
if(NOT WIN32)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-format-overflow")
endif()

if(WIN32 AND EXISTS "c:/vcpkg/installed/x64-windows")
    include_directories(c:/vcpkg/installed/x64-windows/include)
    link_directories(c:/vcpkg/installed/x64-windows/lib)
endif()

add_compile_options(-D__ROS_VERSION=${ROS_VERSION})
if(ROS_VERSION EQUAL 0)
    add_compile_options(-DROSSIMU)
endif()
if(WIN32)
    add_compile_options(-DBOOST_BIND_GLOBAL_PLACEHOLDERS -D_WINSOCK_DEPRECATED_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_WIN32_WINNT=0x0A00)
else()
    set (pthread_LIBRARIES "pthread")
endif()

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

if(ROS_VERSION LESS 2)
    find_package(Boost REQUIRED COMPONENTS filesystem system serialization thread)
endif(ROS_VERSION LESS 2)

if(ENABLE_EMULATOR)
        find_package(jsoncpp REQUIRED) # install libjsoncpp by running "sudo apt-get install libjsoncpp-dev" (Linux) resp. "vcpkg install jsoncpp:x64-windows" (Windows)
endif(ENABLE_EMULATOR)

if(ROS_VERSION EQUAL 1)
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
        cfg/SickLDMRSDriver.cfg
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
        LIDoutputstateSrv.srv
    )
endif(ROS_VERSION EQUAL 1)

if(ROS_VERSION EQUAL 2)
    #uncomment this lines and change path accordingly
    set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
    set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)
    #to fix
    #CMake Error at /snap/clion/126/bin/cmake/linux/share/cmake-3.17/Modules/FindPackageHandleStandardArgs.cmake:164 (message):
    #  Could NOT find FastRTPS (missing: FastRTPS_INCLUDE_DIR FastRTPS_LIBRARIES)

    find_package(ament_cmake REQUIRED)
    # find_package(diagnostic_updater REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(diagnostic_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    if(ENABLE_EMULATOR)
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
    endif(ENABLE_EMULATOR)
    rosidl_generate_interfaces(${PROJECT_NAME}
        # message files
        "msg/Encoder.msg"
        "msg/ImuExtended.msg"
        "msg/LFErecFieldMsg.msg"
        "msg/LFErecMsg.msg"
        "msg/LIDoutputstateMsg.msg"
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
        "srv/LIDoutputstateSrv.srv"
        ${ROSIDL_EMULATOR_FILES}
        DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
    )   
    find_package(Boost REQUIRED COMPONENTS filesystem system serialization thread) # rosidl_generate_interfaces overwrites ${Boost_LIBRARIES}. Workaround: Do find_package(Boost) after calling rosidl_generate_interfaces in CMakeLists.txt
endif(ROS_VERSION EQUAL 2)

if(BUILD_WITH_LDMRS_SUPPORT)
    message(STATUS "Building sick_scan with LDMRS support")
    add_compile_options(-DLDMRS_SUPPORT=1)
    find_package(SickLDMRS REQUIRED)
    find_package(PCL REQUIRED) # workaround https://github.com/ros2/rosidl/issues/402 : Do find_package(PCL) after calling rosidl_generate_interfaces in your CMakeLists.txt
    set(LDMRS_INCLUDES ${SICK_LDMRS_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
    set(LDMRS_SOURCES driver/src/ldmrs/sick_ldmrs_config.cpp driver/src/ldmrs/sick_ldmrs_driver.cpp driver/src/ldmrs/sick_ldmrs_node.cpp)
    set(LDMRS_TARGET_DEPENDENCIES SickLDMRS)
    message(STATUS "LDMRS_INCLUDES: ${LDMRS_INCLUDES}")
    message(STATUS "LDMRS_SOURCES: ${LDMRS_SOURCES}")
    message(STATUS "LDMRS_TARGET_DEPENDENCIES: ${LDMRS_TARGET_DEPENDENCIES}")
endif()
 
if(ENABLE_EMULATOR AND ROS_VERSION EQUAL 1)
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
endif(ENABLE_EMULATOR AND ROS_VERSION EQUAL 1)

if(ROS_VERSION EQUAL 1)
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
endif(ROS_VERSION EQUAL 1)

include_directories(include include/tinyxml ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${LDMRS_INCLUDES} include/sick_scan tools/test_server/include roswrap/src/include/launchparser)

if(ROS_VERSION EQUAL 1)
    add_library(sick_scan_lib STATIC
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
        driver/src/binPrintf.cpp
        driver/src/binScanf.cpp
        driver/src/sick_scan_common_tcp.cpp
        driver/src/sick_generic_radar.cpp
        driver/src/sick_generic_imu.cpp
        driver/src/sick_generic_parser.cpp
        driver/src/sick_scan_common_nw.cpp
        driver/src/sick_scan_config_internal.cpp
        driver/src/softwarePLL.cpp
        driver/src/helper/angle_compensator.cpp
        driver/src/sick_generic_field_mon.cpp
        driver/src/sick_scan_marker.cpp
        driver/src/sick_scan_messages.cpp
        driver/src/sick_scan_services.cpp
        driver/src/sick_generic_laser.cpp
        roswrap/src/launchparser/launchparser.cpp 
        driver/src/tinyxml/tinystr.cpp
        driver/src/tinyxml/tinyxml.cpp
        driver/src/tinyxml/tinyxmlerror.cpp
        driver/src/tinyxml/tinyxmlparser.cpp
        ${LDMRS_SOURCES} 
    )

    add_dependencies(sick_scan_lib ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}) # ${LDMRS_TARGET_DEPENDENCIES})

    target_link_libraries(sick_scan_lib
        ${catkin_LIBRARIES})

    add_executable(sick_generic_caller
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
    # pcl_converter disabled to avoid dependency to pcl
    #
    # add_executable(pcl_converter tools/pcl_converter/src/pcl_converter.cpp tools/pcl_converter/src/gnuplotPaletteReader.cpp)
    #
    # target_link_libraries(pcl_converter
    #        ${catkin_LIBRARIES})


    target_link_libraries(sick_generic_caller sick_scan_lib ${SICK_LDMRS_LIBRARIES})
    
    add_executable(sick_scan_test
        test/src/sick_scan_test.cpp
        )
    target_link_libraries(sick_scan_test
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        sick_scan_lib)

else() # i.e. (ROS_VERSION EQUAL 0 OR ROS_VERSION EQUAL 2)

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

    add_library(sick_scan_lib STATIC
	    roswrap/src/getopt/getopt.c
        roswrap/src/toojpeg/toojpeg.cpp
        roswrap/src/launchparser/launchparser.cpp 
        driver/src/binScanf.cpp
        driver/src/dataDumper.cpp
        driver/src/sick_generic_imu.cpp
        driver/src/sick_generic_parser.cpp
        driver/src/sick_scan_common.cpp
        driver/src/sick_scan_common_nw.cpp
        driver/src/sick_scan_common_tcp.cpp
        driver/src/abstract_parser.cpp
        driver/src/binPrintf.cpp
        driver/src/sick_generic_field_mon.cpp
        driver/src/sick_generic_laser.cpp
        driver/src/sick_generic_radar.cpp
        driver/src/sick_scan_marker.cpp
        driver/src/sick_scan_messages.cpp
        driver/src/sick_scan_services.cpp
        driver/src/tinyxml/tinystr.cpp
        driver/src/tinyxml/tinyxml.cpp
        driver/src/tinyxml/tinyxmlerror.cpp
        driver/src/tinyxml/tinyxmlparser.cpp
        driver/src/tcp/colaa.cpp
        driver/src/tcp/colab.cpp
        driver/src/tcp/errorhandler.cpp
        driver/src/tcp/Mutex.cpp
        driver/src/tcp/SickThread.cpp
        driver/src/tcp/tcp.cpp
        driver/src/tcp/Time.cpp
        driver/src/tcp/toolbox.cpp
        driver/src/softwarePLL.cpp
        driver/src/helper/angle_compensator.cpp
        ${LDMRS_SOURCES} 
        ${SRC_ROSSIMU_FILES}
        ${SRC_WIN_FILES}
    )

    add_executable(sick_generic_caller
        driver/src/sick_generic_caller.cpp
    )

    target_link_libraries(sick_generic_caller sick_scan_lib ${Boost_LIBRARIES} ${pthread_LIBRARIES} ${SICK_LDMRS_LIBRARIES})

endif()

if(ROS_VERSION EQUAL 2)

    ament_target_dependencies(
        sick_scan_lib
        "Boost"
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "geometry_msgs"
        "diagnostic_msgs" 
        "visualization_msgs"
        "tf2_ros" 
        ${LDMRS_TARGET_DEPENDENCIES}
    )

    ament_target_dependencies(
        sick_generic_caller
        "Boost"
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "geometry_msgs"
        "diagnostic_msgs" 
        "visualization_msgs"
        "tf2_ros" 
        ${LDMRS_TARGET_DEPENDENCIES}
    )

    rosidl_target_interfaces(sick_scan_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
        
endif(ROS_VERSION EQUAL 2)

if(ROS_VERSION EQUAL 1)
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
endif(ROS_VERSION EQUAL 1)

if(ROS_VERSION EQUAL 2)
    install(TARGETS
        sick_generic_caller
        DESTINATION lib/${PROJECT_NAME})

    install(TARGETS
        sick_scan_lib
        DESTINATION lib)

    install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME})
        
    ament_package()        
endif(ROS_VERSION EQUAL 2)

#
# build and install sick_scan_emulator
#
if(ENABLE_EMULATOR AND (NOT WIN32 OR ROS_VERSION EQUAL 2)) # sick_scan_emulator not supported on native Windows
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
        ${SRC_ROSSIMU_FILES}
        ${SRC_WIN_FILES}
        )
    target_link_libraries(sick_scan_emulator
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        jsoncpp_lib # ${jsoncpp_LIBRARIES}
        sick_scan_lib
        ${Boost_LIBRARIES})
    target_include_directories(sick_scan_emulator PUBLIC test test/emulator/include)
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
if(ENABLE_EMULATOR)
    add_executable(test_server 
        tools/test_server/src/test_server.cpp 
        tools/test_server/src/test_server_cola_msg.cpp 
        tools/test_server/src/test_server_ldmrs_msg.cpp 
        tools/test_server/src/test_server_thread.cpp)
    target_link_libraries(test_server sick_scan_lib ${SICK_LDMRS_LIBRARIES} ${Boost_LIBRARIES})
    if(ROS_VERSION EQUAL 2)
        ament_target_dependencies(test_server "rclcpp")
    endif(ROS_VERSION EQUAL 2)
    install(TARGETS 
        test_server
        DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY 
        tools/test_server/config
        tools/test_server/launch
        DESTINATION share/${PROJECT_NAME})
endif() 
