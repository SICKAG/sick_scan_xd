/**
* \file
* \brief Laser Scanner network communication
 *
* Copyright (C) 2018,2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2018,2017, SICK AG, Waldkirch
*
* All rights reserved.
* \class SickScanCommonNw
*
* \brief Interface for TCP/IP 
*
* This class provides an interface for TCP/IP communication. 
* It also contains simple methods for accessing the essential contents of the SOPAS message 
* (for example, determining the payload and the SOPAS command used).
* It based on an example of SICK AG.
*
* Doxygen example: http://www-numi.fnal.gov/offline_software/srt_public_context/WebDocs/doxygen-howto.html
*
*/

#include "sick_scan/sick_scan_common_nw.h"
#include "sick_scan/tcp/colaa.hpp"
#include "sick_scan/tcp/colab.hpp"
#include "sick_scan/tcp/BasicDatatypes.hpp"
#include "sick_scan/tcp/tcp.hpp"
#include <map>  // for std::map

#include "sick_scan/tcp/tcp.hpp"
#include "sick_scan/tcp/errorhandler.hpp"
#include "sick_scan/tcp/toolbox.hpp"
#include "sick_scan/tcp/Mutex.hpp"
#include <assert.h>

SickScanCommonNw::SickScanCommonNw()
{
  m_state = CONSTRUCTED;
  m_beVerbose = false;

}

SickScanCommonNw::~SickScanCommonNw()
{
  // Disconnect and shut down receive thread.
  if (isConnected() == true)
  {
    // Change from CONNECTED to CONSTRUCTED
    disconnect();
  }
}


//
// Disconnect from the scanner, and close the interface.
//
bool SickScanCommonNw::disconnect()
{
  closeTcpConnection();

  // Change back to CONSTRUCTED
  m_state = CONSTRUCTED;
  return true;
}


//
// Initialisation from Scanner class:
// Parameter setup only. Afterwards, call connect() to connect to the scanner.
//
bool SickScanCommonNw::init(std::string ipAddress,
                            unsigned short portNumber,
                            Tcp::DisconnectFunction disconnectFunction,
                            void *obj)
{
  m_ipAddress = ipAddress;
  m_portNumber = portNumber;
  m_tcp.setDisconnectCallbackFunction(disconnectFunction, obj);
  return true;
}


bool SickScanCommonNw::setReadCallbackFunction(Tcp::ReadFunction readFunction,
                                               void *obj)
{
  m_tcp.setReadCallbackFunction(readFunction, obj);
  return (true);
}
//
// Verbinde mit dem unter init() eingestellten Geraet, und pruefe die Verbindung
// durch einen DeviceIdent-Aufruf.
//
// true = Erfolgreich.
//


bool SickScanCommonNw::connect()
{

  assert (m_state == CONSTRUCTED); // must not be opened or running already

  // Initialise buffer variables
  m_numberOfBytesInReceiveBuffer = 0; // Buffer is empty
  m_numberOfBytesInResponseBuffer = 0; // Buffer is empty

  // Establish connection here
  // Set the data input callback for our TCP connection
  // m_tcp.setReadCallbackFunction(&SickScanCommonNw::readCallbackFunctionS, this);	// , this, _1, _2));

  bool success = openTcpConnection();
  if (success == true)
  {
    // Check if scanner type matches
    m_state = CONNECTED;

  }
  return success;
}


//
// True, if state is CONNECTED, that is:
// - A TCP-connection exists
// - Read thread is running
//
bool SickScanCommonNw::isConnected()
{
  return (m_state == CONNECTED);
}


/**
 * Open TCP-connection to endpoint (usually IP-address and port)
 *
 * true = Connected, false = no connection
 */
bool SickScanCommonNw::openTcpConnection()
{
  //  printInfoMessage("SickScanCommonNw::openTcpConnection: Connecting TCP/IP connection to " + m_ipAddress + ":" + toString(m_portNumber) + " ...", m_beVerbose);

  bool success = m_tcp.open(m_ipAddress, m_portNumber, m_beVerbose);
  if (success == false)
  {
    // printError("SickScanCommonNw::openTcpConnection: ERROR: Failed to establish TCP connection, aborting!");
    return false;
  }

  return true;
}


//
// Close TCP-connection and shut down read thread
//
void SickScanCommonNw::closeTcpConnection()
{
  if (m_tcp.isOpen())
  {
    m_tcp.close();
  }
}

//
// Static entry point.
//
void SickScanCommonNw::readCallbackFunctionS(void *obj, UINT8 *buffer, UINT32 &numOfBytes)
{
  ((SickScanCommonNw *) obj)->readCallbackFunction(buffer, numOfBytes);
}


/**
 * Read callback. Diese Funktion wird aufgerufen, sobald Daten auf der Schnittstelle
 * hereingekommen sind.
 */
void SickScanCommonNw::readCallbackFunction(UINT8 *buffer, UINT32 &numOfBytes)
{
  bool beVerboseHere = false;
  printInfoMessage(
      "SickScanCommonNw::readCallbackFunction(): Called with " + toString(numOfBytes) + " available bytes.",
      beVerboseHere);

  ScopedLock lock(&m_receiveDataMutex); // Mutex for access to the input buffer
  UINT32 remainingSpace = sizeof(m_receiveBuffer) - m_numberOfBytesInReceiveBuffer;
  UINT32 bytesToBeTransferred = numOfBytes;
  if (remainingSpace < numOfBytes)
  {
    bytesToBeTransferred = remainingSpace;
    // printWarning("SickScanCommonNw::readCallbackFunction(): Input buffer space is to small, transferring only " +
    //              ::toString(bytesToBeTransferred) + " of " + ::toString(numOfBytes) + " bytes.");
  }
  else
  {
    // printInfoMessage("SickScanCommonNw::readCallbackFunction(): Transferring " + ::toString(bytesToBeTransferred) +
    //                   " bytes from TCP to input buffer.", beVerboseHere);
  }

  if (bytesToBeTransferred > 0)
  {
    // Data can be transferred into our input buffer
    memcpy(&(m_receiveBuffer[m_numberOfBytesInReceiveBuffer]), buffer, bytesToBeTransferred);
    m_numberOfBytesInReceiveBuffer += bytesToBeTransferred;

    UINT32 size = 0;

    while (1)
    {
      // Now work on the input buffer until all received datasets are processed
      SopasEventMessage frame = findFrameInReceiveBuffer();

      size = frame.size();
      if (size == 0)
      {
        // Framesize = 0: There is no valid frame in the buffer. The buffer is either empty or the frame
        // is incomplete, so leave the loop
        printInfoMessage("SickScanCommonNw::readCallbackFunction(): No complete frame in input buffer, we are done.",
                         beVerboseHere);

        // Leave the loop
        break;
      }
      else
      {
        // A frame was found in the buffer, so process it now.
        printInfoMessage(
            "SickScanCommonNw::readCallbackFunction(): Processing a frame of length " + ::toString(frame.size()) +
            " bytes.", beVerboseHere);
        processFrame(frame);
      }
    }
  }
  else
  {
    // There was input data from the TCP interface, but our input buffer was unable to hold a single byte.
    // Either we have not read data from our buffer for a long time, or something has gone wrong. To re-sync,
    // we clear the input buffer here.
    m_numberOfBytesInReceiveBuffer = 0;
  }

}


//
// Look for 23-frame (STX/ETX) in receive buffer.
// Move frame to start of buffer
//
// Return: 0 : No (complete) frame found
//        >0 : Frame length
//
SopasEventMessage SickScanCommonNw::findFrameInReceiveBuffer()
{
  UINT32 frameLen = 0;
  UINT32 i;

  // Depends on protocol...
  if (m_protocol == CoLa_A)
  {
    //
    // COLA-A
    //
    // Must start with STX (0x02)
    if (m_receiveBuffer[0] != 0x02)
    {
      // Look for starting STX (0x02)
      for (i = 1; i < m_numberOfBytesInReceiveBuffer; i++)
      {
        if (m_receiveBuffer[i] == 0x02)
        {
          break;
        }
      }

      // Found beginning of frame?
      if (i >= m_numberOfBytesInReceiveBuffer)
      {
        // No start found, everything can be discarded
        m_numberOfBytesInReceiveBuffer = 0; // Invalidate buffer
        return SopasEventMessage(); // No frame found
      }

      // Move frame start to index 0
      UINT32 newLen = m_numberOfBytesInReceiveBuffer - i;
      memmove(&(m_receiveBuffer[0]), &(m_receiveBuffer[i]), newLen);
      m_numberOfBytesInReceiveBuffer = newLen;
    }

    // Look for ending ETX (0x03)
    for (i = 1; i < m_numberOfBytesInReceiveBuffer; i++)
    {
      if (m_receiveBuffer[i] == 0x03)
      {
        break;
      }
    }

    // Found end?
    if (i >= m_numberOfBytesInReceiveBuffer)
    {
      // No end marker found, so it's not a complete frame (yet)
      return SopasEventMessage(); // No frame found
    }

    // Calculate frame length in byte
    frameLen = i + 1;

    return SopasEventMessage(m_receiveBuffer, CoLa_A, frameLen);
  }
  else if (m_protocol == CoLa_B)
  {
    UINT32 magicWord;
    UINT32 payloadlength;

    if (m_numberOfBytesInReceiveBuffer < 4)
    {
      return SopasEventMessage();
    }
    UINT16 pos = 0;
    magicWord = colab::getIntegerFromBuffer<UINT32>(m_receiveBuffer, pos);
    if (magicWord != 0x02020202)
    {
      // Look for starting STX (0x02020202)
      for (i = 1; i <= m_numberOfBytesInReceiveBuffer - 4; i++)
      {
        pos = i; // this is needed, as the position value is updated by getIntegerFromBuffer
        magicWord = colab::getIntegerFromBuffer<UINT32>(m_receiveBuffer, pos);
        if (magicWord == 0x02020202)
        {
          // found magic word
          break;
        }
      }

      // Found beginning of frame?
      if (i > m_numberOfBytesInReceiveBuffer - 4)
      {
        // No start found, everything can be discarded
        m_numberOfBytesInReceiveBuffer = 0; // Invalidate buffer
        return SopasEventMessage(); // No frame found
      }
      else
      {
        // Move frame start to index
        UINT32 bytesToMove = m_numberOfBytesInReceiveBuffer - i;
        memmove(&(m_receiveBuffer[0]), &(m_receiveBuffer[i]), bytesToMove); // payload+magic+length+s+checksum
        m_numberOfBytesInReceiveBuffer = bytesToMove;
      }
    }

    // Pruefe Laenge des Pufferinhalts
    if (m_numberOfBytesInReceiveBuffer < 9)
    {
      // Es sind nicht genug Daten fuer einen Frame
      printInfoMessage("SickScanCommonNw::findFrameInReceiveBuffer: Frame cannot be decoded yet, only " +
                       ::toString(m_numberOfBytesInReceiveBuffer) + " bytes in the buffer.", m_beVerbose);
      return SopasEventMessage();
    }

    // Read length of payload
    pos = 4;
    payloadlength = colab::getIntegerFromBuffer<UINT32>(m_receiveBuffer, pos);
    printInfoMessage(
        "SickScanCommonNw::findFrameInReceiveBuffer: Decoded payload length is " + ::toString(payloadlength) +
        " bytes.", m_beVerbose);

    // Ist die Datenlaenge plausibel und wuede in den Puffer passen?
    if (payloadlength > (sizeof(m_receiveBuffer) - 9))
    {
      // magic word + length + checksum = 9
      printWarning(
          "SickScanCommonNw::findFrameInReceiveBuffer: Frame too big for receive buffer. Frame discarded with length:"
          + ::toString(payloadlength) + ".");
      m_numberOfBytesInReceiveBuffer = 0;
      return SopasEventMessage();
    }
    if ((payloadlength + 9) > m_numberOfBytesInReceiveBuffer)
    {
      // magic word + length + s + checksum = 10
      printInfoMessage(
          "SickScanCommonNw::findFrameInReceiveBuffer: Frame not complete yet. Waiting for the rest of it (" +
          ::toString(payloadlength + 9 - m_numberOfBytesInReceiveBuffer) + " bytes missing).", m_beVerbose);
      return SopasEventMessage(); // frame not complete
    }

    // Calculate the total frame length in bytes: Len = Frame (9 bytes) + Payload
    frameLen = payloadlength + 9;

    //
    // test checksum of payload
    //
    UINT8 temp = 0;
    UINT8 temp_xor = 0;
    UINT8 checkSum;

    // Read original checksum
    pos = frameLen - 1;
    checkSum = colab::getIntegerFromBuffer<UINT8>(m_receiveBuffer, pos);

    // Erzeuge die Pruefsumme zum Vergleich
    for (UINT16 i = 8; i < (frameLen - 1); i++)
    {
      pos = i;
      temp = colab::getIntegerFromBuffer<UINT8>(m_receiveBuffer, pos);
      temp_xor = temp_xor ^ temp;
    }

    // Vergleiche die Pruefsummen
    if (temp_xor != checkSum)
    {
      printWarning("SickScanCommonNw::findFrameInReceiveBuffer: Wrong checksum, Frame discarded.");
      m_numberOfBytesInReceiveBuffer = 0;
      return SopasEventMessage();
    }

    return SopasEventMessage(m_receiveBuffer, CoLa_B, frameLen);
  }

  // Return empty frame
  return SopasEventMessage();
}


/**
 * Send contents of buffer to scanner using according framing.
 *
 * Send buffer is limited to 1024 byte!
 */
void SickScanCommonNw::sendCommandBuffer(UINT8 *buffer, UINT16 len)
{
  m_tcp.write(buffer, len);
}


/**
 * Reads one frame from receive buffer and decodes it.
 * Switches directly to the decoder of the protocol.
 *
 */
void SickScanCommonNw::processFrame(SopasEventMessage &frame)
{

  if (m_protocol == CoLa_A)
  {
    printInfoMessage(
        "SickScanCommonNw::processFrame: Calling processFrame_CoLa_A() with " + ::toString(frame.size()) + " bytes.",
        m_beVerbose);
    // processFrame_CoLa_A(frame);
  }
  else if (m_protocol == CoLa_B)
  {
    printInfoMessage(
        "SickScanCommonNw::processFrame: Calling processFrame_CoLa_B() with " + ::toString(frame.size()) + " bytes.",
        m_beVerbose);
    // processFrame_CoLa_B(frame);
  }
}


//
// Copies a complete frame - in any protocol - from the main input buffer to
// the response buffer.
// The frame is *not* removed from the main input buffer.
//
void SickScanCommonNw::copyFrameToResposeBuffer(UINT32 frameLength)
{
  printInfoMessage("SickScanCommonNw::copyFrameToResposeBuffer: Copying a frame of " + ::toString(frameLength) +
                   " bytes to response buffer.", m_beVerbose);

  if (frameLength <= sizeof(m_responseBuffer))
  {
    // Wir duerfen kopieren
    memcpy(m_responseBuffer, m_receiveBuffer, frameLength);
    m_numberOfBytesInResponseBuffer = frameLength;
  }
  else
  {
    // Der respose-Buffer ist zu klein
    printError("SickScanCommonNw::copyFrameToResposeBuffer: Failed to copy frame (Length=" + ::toString(frameLength) +
               " bytes) to response buffer because the response buffer is too small (buffer size=" +
               ::toString(sizeof(m_responseBuffer)) + " bytes).");
    m_numberOfBytesInResponseBuffer = 0;
  }
}


//
// Removes a complete frame - in any protocol - from the main input buffer.
//
void SickScanCommonNw::removeFrameFromReceiveBuffer(UINT32 frameLength)
{
  // Remove frame from receive buffer
  if (frameLength < m_numberOfBytesInReceiveBuffer)
  {
    // More data in buffer, move them to the buffer start
    UINT32 newLen = m_numberOfBytesInReceiveBuffer - frameLength;
    printInfoMessage("SickScanCommonNw::removeFrameFromReceiveBuffer: Removing " + ::toString(frameLength) +
                     " bytes from the input buffer. New length is " + ::toString(newLen) + " bytes.", m_beVerbose);
    memmove(m_receiveBuffer, &(m_receiveBuffer[frameLength]), newLen);
    m_numberOfBytesInReceiveBuffer = newLen;
  }
  else
  {
    // No other data in buffer, just mark as empty
    printInfoMessage("SickScanCommonNw::removeFrameFromReceiveBuffer: Done, no more data in input buffer.",
                     m_beVerbose);
    m_numberOfBytesInReceiveBuffer = 0;
  }
}


//
// ************************* SOPAS FRAME ************************************************** //
//
SopasEventMessage::SopasEventMessage() :
    m_buffer(NULL), m_protocol(CoLa_A), m_frameLength(0)
{
}


SopasEventMessage::SopasEventMessage(BYTE *buffer, SopasProtocol protocol, UINT32 frameLength) :
    m_buffer(buffer), m_protocol(protocol), m_frameLength(frameLength)
{
//      Constructor
}


UINT32 SopasEventMessage::getPayLoadLength() const
{
  UINT32 payLoadLength = 0;

  switch (m_protocol)
  {
    case CoLa_A:
      payLoadLength = m_frameLength - 2; // everything except the 0x02 0x03 frame
      break;
    case CoLa_B:
      payLoadLength =
          m_frameLength - 9; // everything except start 0x02020202(4byte), payloadLength(4byte) and checksum(1 byte)
  }

  return payLoadLength;
}


/** \brief Returns two character long command
* \return string container command
*
* Returns the core command of a sopas message (e.g. "WN") for ..sWN <whatever>
*
*/
std::string SopasEventMessage::getCommandString() const
{
  std::string commandString;

  switch (m_protocol)
  {
    case CoLa_A:
      commandString = std::string((char *) &m_buffer[2], 2);
      break;
    case CoLa_B:
      commandString = std::string((char *) &m_buffer[9], 2);
  }

  return commandString;
}


/** \brief Returns a pointer to the first payload byte.
* \return Pointer to payload part of message
*
* Returns a pointer to the first payload byte.
  * CoLa-A: Points beyond the leading "0x02" to the "s..." data.
  * CoLa-B: Points beyond the magic word and length bytes, to the "s..." data.
*/
BYTE *SopasEventMessage::getPayLoad()
{
  BYTE *bufferPos = NULL;

  switch (m_protocol)
  {
    case CoLa_A:
      bufferPos = &m_buffer[1];
      break;
    case CoLa_B:
      bufferPos = &m_buffer[8];
      break;
  }

  return bufferPos;
}


/** \brief get SOPAS raw data include header and CRC
* \return Pointer to raw data message
*
* The raw data is stored in m_buffer.
* This function returns a pointer to this buffer.
*/
BYTE *SopasEventMessage::getRawData()
{
  BYTE *bufferPos = NULL;
  bufferPos = &m_buffer[0];
  return bufferPos;
}


INT32 SopasEventMessage::getVariableIndex()
{
  INT32 index = -1;


  BYTE *bufferPos = &getPayLoad()[3];
  switch (m_protocol)
  {
    case CoLa_A:
      index = (INT32) (colaa::decodeUINT16(bufferPos));
      break;
    case CoLa_B:
      index = (INT32) (colab::decodeUINT16(bufferPos));
      break;
    default:
      printError("SopasEventMessage::getVariableIndex: Unknown protocol!");
  }

  return index;
}