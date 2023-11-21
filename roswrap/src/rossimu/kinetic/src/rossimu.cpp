#pragma warning(disable: 4996)
#pragma warning(disable: 4267)

#define _USE_MATH_DEFINES
#include <math.h>
#include <sick_scan/sick_scan_common.h>


#include "ros/forwards.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/service_server.h"
#include "ros/service_client.h"
#include "ros/timer.h"
#include "ros/rate.h"
#include "ros/wall_timer.h"
#include "ros/advertise_options.h"
#include "ros/advertise_service_options.h"
#include "ros/subscribe_options.h"
#include "ros/service_client_options.h"
#include "ros/timer_options.h"
#include "ros/wall_timer_options.h"
#include "ros/spinner.h"
#include "ros/init.h"


//#include <boost/bind.hpp>

#include <xmlrpcpp/XmlRpcValue.h>

#ifndef _MSC_VER
#define __cdecl
#endif
std::string unknownNode = "????";

class MapStringSingleton
{
public:
	// Some accessor functions for the class, itself
	bool hasTag(std::string tag)
	{
		std::map<std::string, std::string>::const_iterator   find = mMapString.find(tag);
		if (find != mMapString.end())
		{
			return(true);
		}
		else
		{
			return(false);
		}

	}
	std::string getVal(std::string tag) const
	{
		std::string s;
		std::map<std::string, std::string>::const_iterator   find = mMapString.find(tag);
		if (find != mMapString.end())
		{
			// If we find it return the value.
			return find->second;
		}
		return("");
	}

	void setVal(std::string tag, std::string val)
	{
		mMapString[tag] = val;
	}

	// The magic function, which allows access to the class from anywhere
	// To get the value of the instance of the class, call:
	//     StringSingleton::Instance().GetString();
	static MapStringSingleton &Instance()
	{
		// This line only runs once, thus creating the only instance in existence
		static MapStringSingleton *instance = new MapStringSingleton;
		// dereferencing the variable here, saves the caller from having to use
		// the arrow operator, and removes temptation to try and delete the
		// returned instance.
		return *instance; // always returns the same instance
	}

private:
	// We need to make some given functions private to finish the definition of the singleton
	MapStringSingleton() {} // default constructor available only to members or friends of this class

							// Note that the next two functions are not given bodies, thus any attempt
							// to call them implicitly will return as compiler errors. This prevents
							// accidental copying of the only instance of the class.
	MapStringSingleton(const MapStringSingleton &old); // disallow copy constructor
	const MapStringSingleton &operator=(const MapStringSingleton &old); //disallow assignment operator

																		// Note that although this should be allowed,
																		// some compilers may not implement private destructors
																		// This prevents others from deleting our one single instance, which was otherwise created on the heap
	~MapStringSingleton() {}
private: // private data for an instance of this class
	std::map<std::string, std::string> mMapString;
};

namespace roswrap
{
	namespace console
	{
	ROSCPP_DECL   bool set_logger_level(const std::string& name, console::levels::Level level)
	{
		return(true);
	}
	}

	ROSCPP_DECL void Subscriber::shutdown(void)
	{
		return;
	}
	ROSCPP_DECL  void NodeHandle::setParam(const std::string& key, int d) const
	{
		char szTmp[255];
		MapStringSingleton& single = MapStringSingleton::Instance();
		sprintf(szTmp, "%d", d);
		single.setVal(key, szTmp);
	}

	ROSCPP_DECL  void NodeHandle::setParam(const std::string& key, double d) const
	{
		char szTmp[255];
		MapStringSingleton& single = MapStringSingleton::Instance();
		sprintf(szTmp, "%le", d);
		single.setVal(key, szTmp);

		//	00530   return param::set(resolveName(key), d);
	}

	ROSCPP_DECL  void NodeHandle::setParam(const std::string& key, bool b) const
	{
		MapStringSingleton& single = MapStringSingleton::Instance();
		single.setVal(key, b ? "true" : "false");
	}
	void NodeHandle::setParam(const std::string& key, const std::string& s) const
	{
		MapStringSingleton& single = MapStringSingleton::Instance();
		single.setVal(key, s);

	}
	bool NodeHandle::getParam(const std::string& key, std::string& s) const
	{
		bool fnd = true;
		MapStringSingleton& single = MapStringSingleton::Instance();
		fnd = single.hasTag(key);
		s = single.getVal(key);
		return(fnd);
	}
	bool NodeHandle::getParam(const std::string& key, double& d) const
	{
		bool fnd = true;
		double dummy = 0.0;
		std::string s;
		MapStringSingleton& single = MapStringSingleton::Instance();
		fnd = single.hasTag(key);
		if (fnd)
		{
			s = single.getVal(key);
			sscanf(s.c_str(), "%le", &dummy);
			d = dummy;
		}
		return(fnd);
	}
	bool NodeHandle::getParam(const std::string& key, float& d) const
	{
		bool fnd = true;
		float dummy = 0.0;
		std::string s;
		MapStringSingleton& single = MapStringSingleton::Instance();
		fnd = single.hasTag(key);
		if (fnd)
		{
			s = single.getVal(key);
			sscanf(s.c_str(), "%e", &dummy);
			d = dummy;
		}
		return(fnd);
	}
	bool NodeHandle::getParam(const std::string& key, int& d) const
	{
		bool fnd = true;
		int dummy = -1;
		std::string s;
		MapStringSingleton& single = MapStringSingleton::Instance();
		fnd = single.hasTag(key);
		if (fnd)
		{
			s = single.getVal(key);
			sscanf(s.c_str(),  "%d", &dummy);
			d = dummy;
		}
		return(fnd);
	}
	bool NodeHandle::getParam(const std::string& key, bool& d) const
	{
		bool fnd = true;
		bool dummy = false;
		std::string s;
		MapStringSingleton& single = MapStringSingleton::Instance();
		fnd = single.hasTag(key);
		if (fnd)
		{
			s = single.getVal(key);
			if (s.length() > 0)
			{
				if (s[0] == '1' || s[0] == 't' || s[0] == 'T')
				dummy = true;
			}
			else
			{
				dummy = false;
			}
			d = dummy;
		}
		return(fnd);
	}
}

namespace roswrap
{
	namespace console
	{
	}

}

void ros::init(int &, char * *, class std::basic_string<char, struct std::char_traits<char>, class std::allocator<char> > const &, unsigned int)
{
  Time::init();
}

void ros::console::initialize(void)
{
}

void ros::console::print(class ros::console::FilterBase *, void *, enum ros::console::levels::Level, char const * src_file, int src_line, char const * fmt, char const *, ...)
{
}

void ros::console::setLogLocationLevel(struct ros::console::LogLocation * loc, enum ros::console::levels::Level level)
{
	if(loc)
	    loc->level_ = level;
}

void ros::console::checkLogLocationEnabled(struct ros::console::LogLocation * loc)
{
	if (loc && loc->level_ >= ros::console::levels::Level::Info)
		loc->logger_enabled_ = true;
}

void ros::spinOnce(void)
{
    usleep(100000); // sleep 100 mikroseconds
}

void ros::spin(void) // difference between spinOnce and spin?
{
	while(ros::ok())
	{
        ros::spinOnce();
        usleep(100000); // sleep 100 mikroseconds
	}
}

void ros::shutdown(void)
{

}
bool ros::ok(void)
{
	return(true);
}
bool ros::isShuttingDown(void)
{
	return(false);
}
ros::NodeHandle::~NodeHandle(void)
{
}

ros::Subscriber::~Subscriber(void)
{
}

ros::Subscriber ros::NodeHandle::subscribe(struct ros::SubscribeOptions &)
{
	ros::Subscriber s;
	return(s);
}



ros::Publisher::Publisher(class ros::Publisher const &)
{
	// printf("Publisher constructor called\n");
}

ros::Publisher::~Publisher(void)
{
}

ros::Subscriber::Subscriber(class ros::Subscriber const &)
{
}

bool ros::NodeHandle::getParamCached(const std::string& key, double& d) const
{
	return(true);
}
bool ros::Publisher::Impl::isValid(void)const
{
	return(true);
}

void ros::Publisher::publish(class std::function<class ros::SerializedMessage __cdecl(void)> const &, class ros::SerializedMessage &)const
{

}

void ros::serialization::throwStreamOverrun(void)
{

}

ros::ServiceServer::~ServiceServer(void)
{

}

ros::NodeHandle::NodeHandle(class ros::NodeHandle const &)
{
}

bool ros::NodeHandle::ok(void)const
{
	return(true);
}

std::string const & ros::this_node::getName(void)
{

	return(unknownNode);
}


class ros::Publisher ros::NodeHandle::advertise(struct ros::AdvertiseOptions &opt)
{
	ros::Publisher p;
	return(p);
}

void ros::console::print(class ros::console::FilterBase *, void *, enum ros::console::levels::Level msg_level, class std::basic_stringstream<char, struct std::char_traits<char>, class std::allocator<char> > const & msg, char const *, int, char const *)
{
	if (msg_level >= ros::console::levels::Level::Info)
	{
		std::string level_str;
		switch (msg_level)
		{
		case ros::console::levels::Debug:
			level_str = "[DEBUG]";
			break;
		case ros::console::levels::Info:
			level_str = "[Info]";
			break;
		case ros::console::levels::Warn:
			level_str = "[Warn]";
			break;
		case ros::console::levels::Error:
			level_str = "[Error]";
			break;
		case ros::console::levels::Fatal:
			level_str = "[Fatal]";
			break;
		default:
			level_str = "[]";
			break;
		}
		std::cout << level_str << ": " << msg.str() << std::endl;
	}
}

class ros::ServiceServer ros::NodeHandle::advertiseService(struct ros::AdvertiseServiceOptions &)
{
	ros::ServiceServer server;
	return(server);
}

//sick_scan_xd::AbstractParser::AbstractParser(void)
//{
//}

//sick_scan_xd::AbstractParser::~AbstractParser(void)
//{
//}

void ros::console::initializeLogLocation(struct ros::console::LogLocation * loc, class std::basic_string<char, struct std::char_traits<char>, class std::allocator<char> > const & msg, enum ros::console::levels::Level level)
{
	if (loc)
		loc->initialized_ = true;
}

bool ros::NodeHandle::hasParam(class std::basic_string<char, struct std::char_traits<char>, class std::allocator<char> > const &)const
{
	return(true);
}


ros::NodeHandle::NodeHandle(class std::basic_string<char, struct std::char_traits<char>, class std::allocator<char> > const &, class std::map<class std::basic_string<char, struct std::char_traits<char>, class std::allocator<char> >, class std::basic_string<char, struct std::char_traits<char>, class std::allocator<char> >, struct std::less<class std::basic_string<char, struct std::char_traits<char>, class std::allocator<char> > >, class std::allocator<struct std::pair<class std::basic_string<char, struct std::char_traits<char>, class std::allocator<char> > const, class std::basic_string<char, struct std::char_traits<char>, class std::allocator<char> > > > > const &)
{
}

std::string ros::Publisher::getTopic(void)const
{
	std::string s;
	return(s);
}

ros::ServiceServer::ServiceServer(class ros::ServiceServer const &)
{

}



// __printf(const char *format, ...)
int rossimu_error(const char *format, ...)
{
	va_list arg;
	int done;

	va_start(arg, format);
	done = vfprintf(stdout, format, arg);
	va_end(arg);

	return done;
}


int rossimu_settings(ros::NodeHandle& nhPriv)
{
	// set all parameters, which are necessary for debugging without using roslaunch. Just start roscore at the beginning of your debug session
	int tmpactive_echos = 1;
	int tmpEcho_filter = 2;
	bool tmpauto_reboot = true;
	std::string tmpframe_id = "cloud";
	std::string scannerName;
	nhPriv.getParam("name", scannerName);
	std::string tmphostname = "192.168.0.232";
	bool tmpintensity = false;
	if (scannerName.compare("sick_mrs_1xxx") == 0)
	{
		tmphostname = "192.168.0.4";
	}
	if (scannerName.compare("sick_mrs_6xxx") == 0)
	{
		tmpintensity = true;
		tmphostname = "192.168.0.24";
	}

	double tmpminang = -60 / 180.0 * M_PI;   // MRS6124-TEST with +/- 30�
	double tmpmaxang = +60 / 180.0 * M_PI;
	std::string tmpport = "2112";
	double tmprange_min = 0.01;
	double tmprange_max = 25.0;
	int tmpskip = 0;
	double tmptime_offset = -0.001;
	double tmptimelimit = 5;

	/////////////////////////////

	nhPriv.setParam("active_echos", tmpactive_echos); // obsolete???
	nhPriv.setParam("filter_echos", tmpEcho_filter);
	nhPriv.setParam("auto_reboot", tmpauto_reboot);
	nhPriv.setParam("hostname", tmphostname);
	nhPriv.setParam("frame_id", tmpframe_id);
	nhPriv.setParam("intensity", tmpintensity);
	nhPriv.setParam("min_ang", tmpminang);
	nhPriv.setParam("max_ang", tmpmaxang);
	nhPriv.setParam("port", tmpport);
	nhPriv.setParam("range_min", tmprange_min);
	nhPriv.setParam("range_max", tmprange_max);
	nhPriv.setParam("skip", tmpskip);
	nhPriv.setParam("time_offset", tmptime_offset);
	nhPriv.setParam("timelimit", tmptimelimit);
	return(0);
}

int fork()
{
	return(0);
}

#ifdef _MSC_VER
void sleep(int secs)
{
	Sleep(secs * 1000);
}
#endif