//
// Tcp.cpp
//
// TCP-Client.
//

#include "sick_scan/tcp/tcp.hpp"
#include "sick_scan/tcp/errorhandler.hpp"
#include "sick_scan/tcp/toolbox.hpp"
#include "sick_scan/tcp/wsa_init.hpp"
#include <stdio.h>      // for sprintf()

#ifdef _MSC_VER
#include <WinSock2.h>
#else
#include <sys/socket.h> // for socket(), bind(), and connect()
#include <arpa/inet.h>  // for sockaddr_in and inet_ntoa()
#endif
#include <string.h>     // for memset()
#include <netdb.h>      // for hostent
#include <iostream>     // for cout
#ifndef _MSC_VER
#include <sys/poll.h>
#include <poll.h>
#endif

Tcp::Tcp()
{
	m_beVerbose = false;
	m_connectionSocket = -1;
	
	m_readThread = 0; // m_readThread.m_threadShouldRun = false;
	
	m_longStringWarningPrinted = false;
	m_disconnectFunction = NULL;
	m_disconnectFunctionObjPtr = NULL;
	m_readFunction = NULL;
	m_readFunctionObjPtr = NULL;

    m_last_tcp_msg_received_nsec = 0; // no message received
}

//
// Destruktor.
//
Tcp::~Tcp(void)
{
	close();
}


//
// Schreibe eine Anzahl Bytes auf die Schnittstelle.
//
// Dieser Aufruf wartet, bis alle Bytes geschrieben wurden.
//
bool Tcp::write(UINT8* buffer, UINT32 numberOfBytes)
{
	// Ist die Verbindung offen?
	if (isOpen() == false)
	{
		ROS_ERROR("Tcp::write: Connection is not open");
		return false;
	}
	INT32 bytesSent;
	bool result;
#ifdef _MSC_VER
	SOCKET* socketPtr = &m_connectionSocket;
	bytesSent = ::send(*socketPtr, (const char*)buffer, numberOfBytes, 0);
#else
	INT32* socketPtr = &m_connectionSocket;
	bytesSent = ::send(*socketPtr, buffer, numberOfBytes, 0);
#endif	
	// Sende Daten an das Socket
	if (bytesSent != (INT32)numberOfBytes)
	{
		printWarning("Tcp::write: Failed to send data to socket.");
		result = false;
	}
	else
	{
		// Erfolg
		printInfoMessage("Tcp::write: Sent " + toString(numberOfBytes) + " bytes to client.", m_beVerbose);
		result = true;
	}

	return result;
}


//
// Setzt die Funktion, die bei einem Disconnect-Ereignis aufgerufen wird.
//
void Tcp::setDisconnectCallbackFunction(DisconnectFunction discFunction, void* obj)
{
	m_disconnectFunction = discFunction;
	m_disconnectFunctionObjPtr = obj;
}



/**
 * True = offen
 *
 * -- Wir sind der Client und mit einem Server verbunden --
 */
bool Tcp::isOpen()
{
	if (m_connectionSocket >= 0)
	{
//		printInfoMessage("Tcp::isOpen: Reporting open connection.", m_beVerbose);
		return true;
	}

//	printInfoMessage("Tcp::isOpen: Reporting no connection.", m_beVerbose);
	return false;
}

//
// Definiere die Lese-Callback-Funktion.
//
void Tcp::setReadCallbackFunction(Tcp::ReadFunction readFunction, void* obj)
{
	m_readFunction = readFunction;
	m_readFunctionObjPtr = obj;
}

//
// Alternative open-Funktion.
//
bool Tcp::open(UINT32 ipAddress, UINT16 port,  bool enableVerboseDebugOutput)
{
	std::string ipAdrStr;
	
	ipAdrStr = ipAdrToString(ipAddress);
				
	bool result = open(ipAdrStr, port, enableVerboseDebugOutput);
	
	return result;
}


//
// Oeffnet die Verbindung.
//
// -- Wir sind der Client, und wollen uns z.B. mit einem Scanner verbinden --
//
bool Tcp::open(std::string ipAddress, UINT16 port, bool enableVerboseDebugOutput)
{
	INT32 result;
	m_beVerbose = enableVerboseDebugOutput;
    m_last_tcp_msg_received_nsec = 0; // no message received
//	printInfoMessage("Tcp::open: Setting up input buffer with size=" + convertValueToString(requiredInputBufferSize) + " bytes.", m_beVerbose);
//	m_inBuffer.init(requiredInputBufferSize, m_beVerbose);
	
	printInfoMessage("Tcp::open: Opening connection.", m_beVerbose);
	wsa_init();

	// Socket erzeugen
	m_connectionSocket = -1;	// Keine Verbindung
	{
		ScopedLock lock(&m_socketMutex);		// Mutex setzen
		m_connectionSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	}
	if (m_connectionSocket  < 0)
	{
        ROS_ERROR("Tcp::open: socket() failed, aborting.");
		return false;
	}

	// Setting a timeout on the TCP connection (fix issue #424 picoScan recovery)
	struct timeval timeout;
	timeout.tv_sec = 2; // Set timeout seconds
	timeout.tv_usec = 0; // Set timeout microseconds
	if (setsockopt(m_connectionSocket, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout)) < 0) 
	{
		ROS_ERROR("Tcp::open: setsockopt failed to set timeout, aborting.");
		close();
		return false;
	}

	// Socket ist da. Nun die Verbindung oeffnen.
	ROS_INFO_STREAM("sick_scan_xd: Tcp::open: connecting to " << ipAddress << ":"  << port << " ...");
	printInfoMessage("Tcp::open: Connecting. Target address is " + ipAddress + ":" + toString(port) + ".", m_beVerbose);
	
	struct sockaddr_in addr;
	struct hostent *server = 0;
	server = gethostbyname(ipAddress.c_str());
	memset(&addr, 0, sizeof(addr));     		// Zero out structure
	addr.sin_family = AF_INET;
	if (server != 0 && server->h_addr != 0)
	{
#ifdef _MSC_VER
		memcpy((char*)&addr.sin_addr.s_addr, (char*)server->h_addr, server->h_length);
#else
		bcopy((char*)server->h_addr, (char*)&addr.sin_addr.s_addr, server->h_length);
#endif
	}
	else
	{
		addr.sin_addr.s_addr = inet_addr(ipAddress.c_str());
	}
	addr.sin_port = htons(port);				// Host-2-Network byte order
#ifdef _MSC_VER
	result = connect(m_connectionSocket, (SOCKADDR*)(&addr), sizeof(addr));
#else
	result = connect(m_connectionSocket, (sockaddr*)(&addr), sizeof(addr));
#endif
	if (result < 0)
	{
		// Verbindungsversuch ist fehlgeschlagen
		std::string text = "Tcp::open: Failed to open TCP connection to " + ipAddress + ":" + toString(port) + ", aborting.";
#ifdef _MSC_VER
		char msgbuf[256] = "";
		int err = WSAGetLastError();
		FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, err, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),	msgbuf,	sizeof(msgbuf), NULL);
		text = text + " Connect error " + toString(WSAGetLastError()) + std::string(msgbuf);
#endif
		ROS_ERROR_STREAM("" << text);
		close(); // close socket
		return false;
	}

	printInfoMessage("Tcp::open: Connection established. Now starting read thread.", m_beVerbose);

	// Empfangsthread starten
	m_readThread = new SickThread<Tcp, &Tcp::readThreadFunction>("TcpRecvThread");
	m_readThread->run(this);
	
	ROS_INFO_STREAM("sick_scan_xd Tcp::open: connected to " << ipAddress << ":"  << port);
	printInfoMessage("Tcp::open: Done, leaving now.", m_beVerbose);

	return true;
}


//
// Lese-Thread (Hauptfunktion).
//
void Tcp::readThreadFunction(bool& endThread, UINT16& waitTimeMs)
{
	INT32 result;

	// Lesen
	result = readInputData();

	// Ergebnis?
	if (result < 0)
	{
		// Verbindung wurde abgebrochen
		if (m_readThread && m_readThread->m_threadShouldRun == true)
		{
			// Wir sollten eigentlich noch laufen!
			printInfoMessage("Tcp::readThreadMain: Connection is lost! Read thread terminates now.", m_beVerbose);
			endThread = true; // interrupt thread
		}
		waitTimeMs = 0;
	}
	else if (result == 0)
	{
		// Wir haben nichts empfangen. Schlafen und dann weiter...
		waitTimeMs = 1;
	}
	else
	{
		// Wir haben etwas empfangen, also nicht schlafen
		waitTimeMs = 0;
	}
}

//
// Read some data from the TCP connection.
//
INT32 Tcp::readInputData()
{
	// Prepare the input buffer
	const UINT16 max_length = 8192;
	UINT8 inBuffer[max_length];
	INT32 recvMsgSize = 0;

	// Ist die Verbindung offen?
	if (isOpen() == false)
	{
    if (rosOk())
  		ROS_ERROR("Tcp::readInputData: Connection is not open, aborting!");
		else
  		ROS_INFO("Tcp::readInputData: Connection is not open, aborting");
		return -1;
	}
		
	// Read some data, if any
#ifdef _MSC_VER
	recvMsgSize = recv(m_connectionSocket, (char *)inBuffer, max_length, 0);
#else
	{
		int ret = -1;
		do {
			struct pollfd fd;

			fd.fd = m_connectionSocket; // your socket handler
			fd.events = POLLIN;
			ret = poll(&fd, 1, 1000); // 1 second for timeout
			switch (ret) {
				case -1:
					// Error
					break;
				case 0:
					// Timeout
					break;
				default:
					recvMsgSize = recv(m_connectionSocket, inBuffer, max_length, 0);
					break;
			}
			if (!m_readThread || m_readThread->m_threadShouldRun == false)
			{
				recvMsgSize = 0;
				break;
			}
		} while (ret == 0);
	}
#endif
	if (recvMsgSize < 0)
	{
		// Fehler
    if (rosOk())
  		ROS_ERROR("Tcp::readInputData: Failed to read data from socket, aborting!");
		else
  		ROS_INFO("Tcp::readInputData: Failed to read data from socket, aborting!");
		ScopedLock lock(&m_socketMutex);
		closeSocket(); // otherwise the driver can terminate with broken pipe in next call to Tcp::write()
	}
	else if (recvMsgSize > 0)
	{
		// Erfolg
		printInfoMessage("Tcp::readInputData: Read " + toString(recvMsgSize) + " bytes from the connection.", m_beVerbose);
		
		// Falls eine Callback-Funktion definiert ist, rufe sie auf mit den
		// empfangenen Daten.
		if (m_readFunction != NULL)
		{
			// Die Daten an die Callback-Funktion uebergeben
			UINT32 length_uint32 = (UINT32)recvMsgSize;
			m_readFunction(m_readFunctionObjPtr, inBuffer, length_uint32);
		}
		else
		{
			// Es ist keine Callback-Funktion definiert, also die Daten im
			// lokalen Puffer speichern.
			for (INT32 i = 0; i < recvMsgSize; i++)
			{
				m_rxBuffer.push_back(inBuffer[i]);
			}
		}
	    m_last_tcp_msg_received_nsec = rosNanosecTimestampNow(); // timestamp in nanoseconds of the last received tcp message (or 0 if no message received)
		
	}
	else if (recvMsgSize == 0)
	{
		// Verbindungsabbruch
    if (rosOk())
  		ROS_ERROR("Tcp::readInputData: Read 0 bytes, connection is lost!");
		else
  		ROS_INFO("Tcp::readInputData: Read 0 bytes, connection is lost");
		
		// Informieren?
		if (m_disconnectFunction != NULL)
		{
			m_disconnectFunction(m_disconnectFunctionObjPtr);
		}
		
		// Mutex setzen
		ScopedLock lock(&m_socketMutex);
		closeSocket(); // otherwise the driver can terminate with broken pipe in next call to Tcp::write()
		// m_connectionSocket = -1;	// Keine Verbindung mehr
	}
	
	return recvMsgSize;
}


//
// Close an open connection, if any, and stops the read thread (if running)
//
void Tcp::close()
{
	printInfoMessage("Tcp::close: Closing Tcp connection.", m_beVerbose);

	// Dem Lese-Thread ein Ende signalisieren
	if(m_readThread)
		m_readThread->m_threadShouldRun = false;
	// Close TCP socket
	if (isOpen() == true)
	{
		closeSocket();
	}
	else
	{
		printInfoMessage("Tcp::close: Nothing to do - no open connection? Aborting.", m_beVerbose);
	}
    // Thread stoppen
	if(m_readThread)
		stopReadThread();
	m_last_tcp_msg_received_nsec = 0; // no message received

	printInfoMessage("Tcp::close: Done - Connection is now closed.", m_beVerbose);
}

/**
 * Closes the tcp connection (if it's currently open)
 */
void Tcp::closeSocket()
{
	// Close TCP socket
	if (isOpen() == true)
	{
		// Verbindung schliessen
#ifdef _MSC_VER
		::shutdown(m_connectionSocket, SD_BOTH);
		closesocket(m_connectionSocket);  // waere evtl. auch fuer Linux korrekt
#else
		::shutdown(m_connectionSocket, SHUT_RDWR);
		::close(m_connectionSocket);
#endif
		m_connectionSocket = -1;	// Keine Verbindung mehr
	}
}

/**
 * Stoppe den Lese-Thread.
 */
void Tcp::stopReadThread()
{
	if(m_readThread)
	{
		printInfoMessage("Tcp::stopReadThread: Stopping thread.", m_beVerbose);
		
		m_readThread->m_threadShouldRun = false;
		m_readThread->join();
		delete m_readThread;
		m_readThread = 0;
		printInfoMessage("Tcp::stopReadThread: Done - Read thread is now closed.", m_beVerbose);
	}
}



/**
 * Auslesen der Anzahl zum Lesen verfuegbarer Bytes.
 *
 * Rueckgabe: Anzahl der Bytes im Lese-Puffer.
 */
UINT32 Tcp::getNumReadableBytes()
{
	return (UINT32)m_rxBuffer.size();
}


//
// Read function.
//
// 0..bufferLen bytes are returned.
// Return value is the number of returned bytes.
//
// DEPRECATED. Use the callback mechanism instead!
//
UINT32 Tcp::read(UINT8* buffer, UINT32 bufferLen)
{
	UINT32 bytesRead = 0;
	
	// Lesen
	while ((getNumReadableBytes() > 0) && (bufferLen > bytesRead))
	{
		buffer[bytesRead] = m_rxBuffer.front();
		m_rxBuffer.pop_front();
		bytesRead += 1;	// m_inBuffer.read(buffer, bufferLen);
	}
	
	return bytesRead;
}


/**
 * Lese einen String, bis zum Trennzeichen.
 *
 * ** DO NOT INTERMIX WITH CALLS TO READ() **
 */
std::string Tcp::readString(UINT8 delimiter)
{
	UINT8 c = delimiter;
	std::string outString;
	const UINT16 maxStringLength = 8192;

	// String fuellen
	while (m_rxBuffer.size() > 0)
	{
		// Es sind noch Daten im Puffer
		c = m_rxBuffer.front();
		m_rxBuffer.pop_front();
		if (c == delimiter)
		{
			// Trennzeichen gefunden - wir sind fertig!
			outString = m_rxString;
			m_rxString.clear();
			break;
		}
		m_rxString += c;
	}

	// Ueberlauf der Ausgabe?
	if (m_rxString.length() > maxStringLength)
	{
		if (m_longStringWarningPrinted == false)
		{
			// Die lange Version
			printWarning("Receive-String has excessive length (" + toString(m_rxString.length()) +" bytes). Clearing string. On serial devices, incorrect bitrate settings may cause this behaviour.");
			m_longStringWarningPrinted = true;
		}
		else
		{
			// Die Kurzfassung
			printWarning("Receive-String has excessive length (" + toString(m_rxString.length()) +" bytes). Clearing string.");
		}
		m_rxString.clear();
	}

	// Textmeldung
	if ((m_beVerbose == true) && (outString.length() > 0))
	{
		printInfoMessage("Tcp::readString: Returning string: " + outString, true);
	}

	return outString;
}


