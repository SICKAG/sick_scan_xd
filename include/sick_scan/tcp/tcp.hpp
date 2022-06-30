#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
//
// TCP.hpp
//
// Ethernet TCP data sender/receiver.
//
// Sick AG
//
// HISTORY
//
// 1.0.0	10.11.2011, VWi
//			Initial version.
//



#ifndef TCP_HPP
#define TCP_HPP

#include "sick_scan/sick_ros_wrapper.h"
#include "sick_scan/tcp/BasicDatatypes.hpp"
#ifdef _MSC_VER
#include <winsock2.h>
#else
#include <sys/socket.h> /* for socket(), bind(), and connect() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#endif
#include "sick_scan/tcp/Mutex.hpp"
#include "sick_scan/tcp/SickThread.hpp"
#include <list>


//
// Sender and receiver for data over a TCP connection. Client!
//
class Tcp
{
public:
	Tcp();
	~Tcp();

	// Opens the connection.
	bool open(std::string ipAddress, UINT16 port, bool enableVerboseDebugOutput = false);
	bool open(UINT32 ipAddress, UINT16 port, bool enableVerboseDebugOutput = false);
	void close();											// Closes the connection, if it was open.
	bool isOpen();	// "True" if a connection is currently open.
	bool write(UINT8* buffer, UINT32 numberOfBytes);		// Writes numberOfBytes bytes to the open connection.
	std::string readString(UINT8 delimiter);				// Reads a string, if available. Strings are separated with the delimiter char.

	// Buffer read function (for polling)
	UINT32 getNumReadableBytes();							// Returns the number of bytes currently available for reading.
	UINT32 read(UINT8* buffer, UINT32 bufferLen);			// Reads up to bufferLen bytes from the buffer.

	// Read callbacks (for being called when data is available)
	typedef void (*ReadFunction)(void* obj, UINT8* inputBuffer, UINT32& numBytes);	//  ReadFunction
	void setReadCallbackFunction(ReadFunction readFunction, void* obj);

	// Information if the connection is disconnected.
	typedef void (*DisconnectFunction)(void* obj);								//  Called on disconnect
	void setDisconnectCallbackFunction(DisconnectFunction discFunction, void* obj);

	uint64_t getNanosecTimestampLastTcpMessageReceived(void) { return m_last_tcp_msg_received_nsec; } // Returns a timestamp in nanoseconds of the last received tcp message (or 0 if no message received)

private:
	bool m_longStringWarningPrinted;
	std::string m_rxString;						// fuer readString()
	bool isClientConnected_unlocked();
	std::list<unsigned char> m_rxBuffer;		// Main input buffer
	void closeSocket();
	void stopReadThread();
	void startServerThread();
	void stopServerThread();
	
    struct sockaddr_in m_serverAddr;				// Local address
	bool m_beVerbose;
	Mutex m_socketMutex;
#ifndef _MSC_VER	
	INT32 m_connectionSocket;	// Socket, wenn wir der Client sind (z.B. Verbindung zum Scanner)
#else
	SOCKET m_connectionSocket;	// Socket, wenn wir der Client sind (z.B. Verbindung zum Scanner)
#endif
	void readThreadFunction(bool& endThread, UINT16& waitTimeMs);
	SickThread<Tcp, &Tcp::readThreadFunction>* m_readThread;
	INT32 readInputData();
	
	ReadFunction m_readFunction;		// Receive callback
	void* m_readFunctionObjPtr;			// Object of the Receive callback
	DisconnectFunction m_disconnectFunction;
	void* m_disconnectFunctionObjPtr;	// Object of the Disconect callback

	uint64_t m_last_tcp_msg_received_nsec; // timestamp in nanoseconds of the last received tcp message (or 0 if no message received)
};

#endif // TCP_HPP
