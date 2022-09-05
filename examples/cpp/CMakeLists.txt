cmake_minimum_required(VERSION 3.5)
project(minimum_sick_scan_api_client)
set(SICK_SCAN_XD_ROOT_DIR "${PROJECT_SOURCE_DIR}/../..")
set(SOURCES minimum_sick_scan_api_client.cpp ${SICK_SCAN_XD_ROOT_DIR}/test/src/sick_scan_xd_api/sick_scan_xd_api_wrapper.c)
add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC "${SICK_SCAN_XD_ROOT_DIR}/include") # include folder for sick_scan_api.h
if(NOT WIN32)
  target_link_libraries(${PROJECT_NAME} PUBLIC dl)  # dynamic library loading
endif()
