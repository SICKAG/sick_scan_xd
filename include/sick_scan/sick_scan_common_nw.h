//
// Created by michael on 1/22/18.
//

#ifndef SICK_SCAN_SICK_SCAN_COMMON_NW_H
#define SICK_SCAN_SICK_SCAN_COMMON_NW_H

#include "sick_scan/tcp/BasicDatatypes.hpp"
#include "sick_scan/tcp/tcp.hpp"
#include <map>  // for std::map

//
// SickScanCommonNw.cpp
//
//  Created on: 18.07.2011
//      Author: sick
//
#include "sick_scan/tcp/tcp.hpp"
#include "sick_scan/tcp/errorhandler.hpp"
#include "sick_scan/tcp/toolbox.hpp"
#include "sick_scan/tcp/Mutex.hpp"


#include <string>

class SopasEventMessage;

class SopasAnswer;

enum SopasProtocol
{
  CoLa_A, ///< Command Language ASCI
  CoLa_B,  ///< Command Language binary
  CoLa_Unknown  ///< Unknown Command Language
};


class SickScanCommonNw
{
public:


  SickScanCommonNw();

  ~SickScanCommonNw();

  bool init(std::string ipAddress,
            unsigned short portNumber,
            Tcp::DisconnectFunction disconnectFunction,
            void *obj);

  bool setReadCallbackFunction(Tcp::ReadFunction readFunction,
                               void *obj);

  /// Connects to a sensor via tcp and reads the device name.
  bool connect();

  /// Returns true if the tcp connection is established.
  bool isConnected();

  /** \brief Closes the connection to the LMS. This is the opposite of init().
   *
   * Switches this device from the CONNECTED state back in the
   * CONSTRUCTED state.
   *
   * \return True if the device is now in the CONSTRUCTED state
   */
  bool disconnect();

  void sendCommandBuffer(UINT8 *buffer, UINT16 len);

private:
  // TCP
  bool openTcpConnection();

  void closeTcpConnection();

  /// Function that will be called on incomming data via tcp.
  static void readCallbackFunctionS(void *obj, UINT8 *buffer, UINT32 &numOfBytes);

  void readCallbackFunction(UINT8 *buffer, UINT32 &numOfBytes);

  SopasEventMessage findFrameInReceiveBuffer();

  void processFrame(SopasEventMessage &frame);

  bool m_beVerbose;


  // Response buffer
  UINT32 m_numberOfBytesInResponseBuffer; ///< Number of bytes in buffer
  UINT8 m_responseBuffer[1024]; ///< Receive buffer for everything except scan data and eval case data.
  Mutex m_receiveDataMutex; ///< Access mutex for buffer

  // Receive buffer
  UINT32 m_numberOfBytesInReceiveBuffer; ///< Number of bytes in buffer
  UINT8 m_receiveBuffer[25000]; ///< Low-Level receive buffer for all data (25000 should be enough for NAV300 Events)



  // TCP
  Tcp m_tcp;
  std::string m_ipAddress;
  UINT16 m_portNumber;
  SopasProtocol m_protocol;


  void copyFrameToResposeBuffer(UINT32 frameLength);

  void removeFrameFromReceiveBuffer(UINT32 frameLength);

protected:
  enum State
  {
    /// Object has been constructed. Use init() to go into CONNECTED state.
    CONSTRUCTED
    /// Object is now connected. Use run() to go into RUNNING
    /// state, or disconnect() to go back into CONSTRUCTED state.
    ,
    CONNECTED
    /// Object is connected and emitting data. Use stop() to go back into CONNECTED,
    /// or disconnect() to go back into CONSTRUCTED state.
    //		, RUNNING
  };

  State m_state;
};

/// Class that represents a message that was sent by a sensor. (Event message)
class SopasEventMessage
{
public:
  /// Default constructor
  SopasEventMessage();

  /// Destructor
  ~SopasEventMessage()
  {}

  /**
   * @brief Constructor. This class will only store a pointer to the byte buffer. It will not deallocate the
   *        memory. Please make sure that the buffer is not deallocated while you are working with this class.
   * @param buffer byte buffer with the message (Sopas frame)
   * @param protocol type of protocol (Cola-A, Cola-B)
   * @param frameLength length of the frame
   */
  SopasEventMessage(BYTE *buffer, SopasProtocol protocol, UINT32 frameLength);

  SopasProtocol getProtocolType() const
  {
    return m_protocol;
  }


  UINT32 size() const
  {
    return m_frameLength;
  }

  /// contains 's' + command string(2 byte) + content(payload length - 3)
  UINT32 getPayLoadLength() const;

  std::string getCommandString() const;

  /// contains 's' + command string(2 byte) + content(payload length - 3)
  BYTE *getPayLoad();

  BYTE *getRawData();

  /// Returns the index of a variable (answer to read variable by index). In case of error a negative value will be returned
  INT32 getVariableIndex();

  /// Returns the name of a variable (answer to read variable by name). In case of error an empty value will be returned
  std::string getVariableName();

  bool isValid() const
  { return (m_buffer != NULL); }

private:
  void detectEncoding();

  void detectMessageType();

private:
  BYTE *m_buffer;
  SopasProtocol m_protocol;
  UINT32 m_frameLength;
};

/// Class that encapsulates a buffer that was sent as return to a sync call. (variable / method)
class SopasAnswer
{
public:
  /// Constructor. Copies the content of the answer into the buffer of this object.
  SopasAnswer(const BYTE *answer, UINT32 answerLength);

  /// Destructor. Frees the memory for the copied buffer.
  ~SopasAnswer();

  BYTE *getBuffer()
  { return m_answerBuffer; }

  UINT32 size()
  { return m_answerLength; }

  bool isValid()
  { return (m_answerBuffer != NULL); }

private:
  UINT32 m_answerLength;
  BYTE *m_answerBuffer;
};

#endif //SICK_SCAN_SICK_SCAN_COMMON_NW_H
