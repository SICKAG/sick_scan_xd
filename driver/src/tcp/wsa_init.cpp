
#ifdef _MSC_VER
#include <WinSock2.h>
#endif

class WSA_AUTO_INIT
{
public:
	WSA_AUTO_INIT() : m_wsastartup(0) {}
	~WSA_AUTO_INIT()
	{
#if defined _MSC_VER && __ROS_VERSION == 0
		if (m_wsastartup > 0)
		{
			WSACleanup();
			m_wsastartup = 0;
		}
#endif
	}
	void init()
	{
#if defined _MSC_VER && __ROS_VERSION == 0
		if (m_wsastartup == 0)
		{
			WSADATA wsaData;
			WSAStartup(MAKEWORD(2, 2), &wsaData);
		}
#endif
	}
protected:
	int m_wsastartup;
};

static WSA_AUTO_INIT s_wsa_auto_init_singleton;

void wsa_init(void)
{
	s_wsa_auto_init_singleton.init();
}

