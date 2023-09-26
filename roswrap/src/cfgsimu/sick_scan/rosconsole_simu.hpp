#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
#ifndef ROSCONSOLE_SIMU_HPP
#define ROSCONSOLE_SIMU_HPP

#pragma warning(disable: 4996)
#pragma warning(disable: 4267)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
// #define ROSCPP_ROS_H
// #define ROSCPP_NODE_HANDLE_H
// #define ROSCPP_THIS_NODE_H
// #define ROSCPP_PUBLISHER_HANDLE_H
// #define ROSCPP_SUBSCRIPTION_H
#include <ros/ros.h>

//int fork();
void sleep(int secs);

namespace roswrap
{
	namespace console
	{
		//bool g_initialized = true;
#undef ROS_WARN
#undef ROS_INFO
#undef ROS_DEBUG
#undef ROS_FATAL
#undef ROS_ERROR
#undef ROS_ERROR_THROTTLE
#undef ROS_WARN_ONCE
#define ROS_WARN(...) do { fprintf(stderr,"ROS_WARN: "); fprintf (stderr, __VA_ARGS__); fprintf (stderr,"\n"); } while(0)
#define ROS_INFO(...) do { fprintf(stderr,"ROS_INFO: "); fprintf (stderr, __VA_ARGS__); fprintf (stderr,"\n"); } while(0)
#define ROS_DEBUG(...) do { fprintf(stderr,"ROS_DEBUG: "); fprintf (stderr, __VA_ARGS__); fprintf (stderr,"\n"); } while(0)
#define ROS_FATAL(...)
#define ROS_ERROR(...)  do { fprintf (stderr, __VA_ARGS__); fprintf (stderr,"\n"); } while(0) // #define eprintf(format, �) fprintf (stderr, format, __VA_ARGS__)

#define ROS_ERROR_THROTTLE(...)
#define ROS_WARN_ONCE(...)
		// #define ROS_WARN(...) ROS_LOG(::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

//#undef ROS_DEBUG_STREAM
//#define ROS_DEBUG_STREAM(message) { std::stringstream _msg; _msg << message; ROS_DEBUG(_msg.str().c_str()); }
	}

	// class NodeHandle
	// {
	// public:
	// 	template <typename T> bool getParam(const std::string& param_name, T& param_value) { return false; }
	// 	template <typename T> void setParam(const std::string& param_name, const T& param_value) { }
	// };
}

int rossimu_error(const char *format, ...);
int rossimu_settings(ros::NodeHandle& nhPriv);
#endif