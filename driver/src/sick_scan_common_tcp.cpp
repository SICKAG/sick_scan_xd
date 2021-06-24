/**
* \file
* \brief Laser Scanner communication (TCP Helper Class)
* Copyright (C) 2013, Osnabrück University
* Copyright (C) 2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017, SICK AG, Waldkirch
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Osnabrück University nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*     * Neither the name of SICK AG nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*  Last modified: 12th Dec 2017
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*         Jochen Sprickerhof <jochen@sprickerhof.de>
*         Martin Günther <mguenthe@uos.de>
*
* Based on the TiM communication example by SICK AG.
*
*/

#ifdef _MSC_VER
#pragma warning(disable: 4996)
#pragma warning(disable: 4267)
#pragma warning(disable: 4101)   // C4101: "e" : Unreferenzierte lokale Variable
#define _WIN32_WINNT 0x0501

#endif

#include <sick_scan/sick_scan_common_tcp.h>
#include <sick_scan/tcp/colaa.hpp>
#include <sick_scan/tcp/colab.hpp>

#include <boost/asio.hpp>
#include <boost/lambda/lambda.hpp>
#include <algorithm>
#include <iterator>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <sick_scan/sick_generic_radar.h>

#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif

std::vector<unsigned char> exampleData(65536);
std::vector<unsigned char> receivedData(65536);
static long receivedDataLen = 0;

static int getDiagnosticErrorCode()
{
#ifdef _MSC_VER
#undef ERROR
  return(2);
#else
  return (diagnostic_msgs::DiagnosticStatus::ERROR);
#endif
}

namespace sick_scan
{
  bool emulateReply(UINT8 *requestData, int requestLen, std::vector<unsigned char> *replyVector)
  {
    std::string request;
    std::string reply;
    std::vector<std::string> keyWordList;
    std::vector<std::string> answerList;

    keyWordList.push_back("sMN SetAccessMode");
    answerList.push_back("sAN SetAccessMode 1");

    keyWordList.push_back("sWN EIHstCola");
    answerList.push_back("sWA EIHstCola");

    keyWordList.push_back("sRN FirmwareVersion");
    answerList.push_back("sRA FirmwareVersion 8 1.0.0.0R");

    keyWordList.push_back("sRN OrdNum");
    answerList.push_back("sRA OrdNum 7 1234567");

    keyWordList.push_back("sWN TransmitTargets 1");
    answerList.push_back("sWA TransmitTargets");

    keyWordList.push_back("sWN TransmitObjects 1");
    answerList.push_back("sWA TransmitObjects");

    keyWordList.push_back("sWN TCTrackingMode 0");
    answerList.push_back("sWA TCTrackingMode");

    keyWordList.push_back("sRN SCdevicestate");
    answerList.push_back("sRA SCdevicestate 1");

    keyWordList.push_back("sRN DItype");
    answerList.push_back("sRA DItype D RMS3xx-xxxxxx");

    keyWordList.push_back("sRN ODoprh");
    answerList.push_back("sRA ODoprh 451");

    keyWordList.push_back("sMN mSCloadappdef");
    answerList.push_back("sAN mSCloadappdef");


    keyWordList.push_back("sRN SerialNumber");
    answerList.push_back("sRA SerialNumber 8 18020073");

    keyWordList.push_back("sMN Run");
    answerList.push_back("sAN Run 1s");

    keyWordList.push_back("sRN ODpwrc");
    answerList.push_back("sRA ODpwrc 20");

    keyWordList.push_back("sRN LocationName");
    answerList.push_back("sRA LocationName B not defined");

    keyWordList.push_back("sEN LMDradardata 1");
    answerList.push_back("sEA LMDradardata 1");

    for (int i = 0; i < requestLen; i++)
    {
      request += (char) requestData[i];
    }
    for (int i = 0; i < keyWordList.size(); i++)
    {
      if (request.find(keyWordList[i]) != std::string::npos)
      {
        reply = (char) 0x02;
        reply += answerList[i];
        reply += (char) 0x03;
      }
    }

    replyVector->clear();
    for (int i = 0; i < reply.length(); i++)
    {
      replyVector->push_back((unsigned char) reply[i]);
    }


    return (true);
  }


  SickScanCommonTcp::SickScanCommonTcp(const std::string &hostname, const std::string &port, int &timelimit,
                                       SickGenericParser *parser, char cola_dialect_id)
      :
      SickScanCommon(parser),
      socket_(io_service_),
      deadline_(io_service_),
      hostname_(hostname),
      port_(port),
      timelimit_(timelimit)
  {

    setEmulSensor(false);
    if ((cola_dialect_id == 'a') || (cola_dialect_id == 'A'))
    {
      this->setProtocolType(CoLa_A);
    }

    if ((cola_dialect_id == 'b') || (cola_dialect_id == 'B'))
    {
      this->setProtocolType(CoLa_B);
    }

    assert(this->getProtocolType() != CoLa_Unknown);

    m_numberOfBytesInReceiveBuffer = 0;
    m_alreadyReceivedBytes = 0;
    this->setReplyMode(0);
    // io_service_.setReadCallbackFunction(boost::bind(&SopasDevice::readCallbackFunction, this, _1, _2));

    // Set up the deadline actor to implement timeouts.
    // Based on blocking TCP example on:
    // http://www.boost.org/doc/libs/1_46_0/doc/html/boost_asio/example/timeouts/blocking_tcp_client.cpp
    deadline_.expires_at(boost::posix_time::pos_infin);
    checkDeadline();

  }

  SickScanCommonTcp::~SickScanCommonTcp()
  {
    // stop_scanner();
    close_device();
  }

  using boost::asio::ip::tcp;
  using boost::lambda::var;
  using boost::lambda::_1;


  void SickScanCommonTcp::disconnectFunction()
  {

  }

  void SickScanCommonTcp::disconnectFunctionS(void *obj)
  {
    if (obj != NULL)
    {
      ((SickScanCommonTcp *) (obj))->disconnectFunction();
    }
  }

  void SickScanCommonTcp::readCallbackFunctionS(void *obj, UINT8 *buffer, UINT32 &numOfBytes)
  {
    ((SickScanCommonTcp *) obj)->readCallbackFunction(buffer, numOfBytes);
  }


  void SickScanCommonTcp::setReplyMode(int _mode)
  {
    m_replyMode = _mode;
  }

  int SickScanCommonTcp::getReplyMode()
  {
    return (m_replyMode);
  }

#if 0
  void SickScanCommonTcp::setProtocolType(char cola_dialect_id)
  {
    if ((cola_dialect_id == 'a') || (cola_dialect_id == 'A'))
    {
      this->m_protocol = CoLa_A;
    }
    else
    {
      this->m_protocol = CoLa_B;
    }
  }
#endif

/*!
		\brief Set emulation flag (using emulation instead of "real" scanner - currently implemented for radar
		\param _emulFlag: Flag to switch emulation on or off
		\return
*/
  void SickScanCommonTcp::setEmulSensor(bool _emulFlag)
  {
    m_emulSensor = _emulFlag;
  }

/*!
		\brief get emulation flag (using emulation instead of "real" scanner - currently implemented for radar
		\param
		\return bool: Flag to switch emulation on or off
*/
  bool SickScanCommonTcp::getEmulSensor()
  {
    return (m_emulSensor);
  }

  //
  // Look for 23-frame (STX/ETX) in receive buffer.
  // Move frame to start of buffer
  //
  // Return: 0 : No (complete) frame found
  //        >0 : Frame length
  //
  SopasEventMessage SickScanCommonTcp::findFrameInReceiveBuffer()
  {
    UINT32 frameLen = 0;
    UINT32 i;

    // Depends on protocol...
    if (getProtocolType() == CoLa_A)
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
    else if (getProtocolType() == CoLa_B)
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
 * Read callback. Diese Funktion wird aufgerufen, sobald Daten auf der Schnittstelle
 * hereingekommen sind.
 */

  void SickScanCommonTcp::processFrame(ros::Time timeStamp, SopasEventMessage &frame)
  {

    if (getProtocolType() == CoLa_A)
    {
      printInfoMessage(
          "SickScanCommonNw::processFrame: Calling processFrame_CoLa_A() with " + ::toString(frame.size()) + " bytes.",
          m_beVerbose);
      // processFrame_CoLa_A(frame);
    }
    else if (getProtocolType() == CoLa_B)
    {
      printInfoMessage(
          "SickScanCommonNw::processFrame: Calling processFrame_CoLa_B() with " + ::toString(frame.size()) + " bytes.",
          m_beVerbose);
      // processFrame_CoLa_B(frame);
    }

    // Push frame to recvQueue

    DatagramWithTimeStamp dataGramWidthTimeStamp(timeStamp, std::vector<unsigned char>(frame.getRawData(),
                                                                                       frame.getRawData() +
                                                                                       frame.size()));
    // recvQueue.push(std::vector<unsigned char>(frame.getRawData(), frame.getRawData() + frame.size()));
    recvQueue.push(dataGramWidthTimeStamp);
  }

  void SickScanCommonTcp::readCallbackFunction(UINT8 *buffer, UINT32 &numOfBytes)
  {
    ros::Time rcvTimeStamp = ros::Time::now(); // stamp received datagram
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
          processFrame(rcvTimeStamp, frame);
          UINT32 bytesToMove = m_numberOfBytesInReceiveBuffer - size;
          memmove(&(m_receiveBuffer[0]), &(m_receiveBuffer[size]), bytesToMove); // payload+magic+length+s+checksum
          m_numberOfBytesInReceiveBuffer = bytesToMove;

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


  int SickScanCommonTcp::init_device()
  {
    int portInt;
    sscanf(port_.c_str(), "%d", &portInt);
    m_nw.init(hostname_, portInt, disconnectFunctionS, (void *) this);
    m_nw.setReadCallbackFunction(readCallbackFunctionS, (void *) this);
    if (this->getEmulSensor())
    {
      ROS_INFO("Sensor emulation is switched on - network traffic is switched off.");
    }
    else
    {
      m_nw.connect();
    }
    return ExitSuccess;
  }

  int SickScanCommonTcp::close_device()
  {
    ROS_WARN("Disconnecting TCP-Connection.");
    m_nw.disconnect();
    return 0;
  }


  bool SickScanCommonTcp::stopScanData()
  {
    stop_scanner();
    return (true);
  }

  void SickScanCommonTcp::handleRead(boost::system::error_code error, size_t bytes_transfered)
  {
    ec_ = error;
    bytes_transfered_ += bytes_transfered;
  }


  void SickScanCommonTcp::checkDeadline()
  {
    if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
    {
      // The reason the function is called is that the deadline expired. Close
      // the socket to return all IO operations and reset the deadline
      socket_.close();
      deadline_.expires_at(boost::posix_time::pos_infin);
    }

    // Nothing bad happened, go back to sleep
    deadline_.async_wait(boost::bind(&SickScanCommonTcp::checkDeadline, this));
  }


  int SickScanCommonTcp::numberOfDatagramInInputFifo()
  {
    int ret = 0;
    ret = this->recvQueue.getNumberOfEntriesInQueue();
    return(ret);
  }

  int SickScanCommonTcp::readWithTimeout(size_t timeout_ms, char *buffer, int buffer_size, int *bytes_read,
                                         bool *exception_occured, bool isBinary)
  {
    // Set up the deadline to the proper timeout, error and delimiters
    deadline_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
    const char end_delim = static_cast<char>(0x03);
    int dataLen = 0;
    ec_ = boost::asio::error::would_block;
    bytes_transfered_ = 0;

    size_t to_read;

    int numBytes = 0;
    // Polling - should be changed to condition variable in the future
    int waitingTimeInMs = 1; // try to lookup for new incoming packages
    int i;
    for (i = 0; i < timeout_ms; i += waitingTimeInMs)
    {
      if (false == this->recvQueue.isQueueEmpty())
      {
        break;
      }
      boost::this_thread::sleep(boost::posix_time::milliseconds(waitingTimeInMs));
    }
    if (i >= timeout_ms)
    {
      ROS_ERROR("no answer received after %zu ms. Maybe sopas mode is wrong.\n", timeout_ms);
      return (ExitError);
    }
    boost::condition_variable cond_;
    DatagramWithTimeStamp datagramWithTimeStamp = this->recvQueue.pop();

    *bytes_read = datagramWithTimeStamp.datagram.size();
    memcpy(buffer, &(datagramWithTimeStamp.datagram[0]), datagramWithTimeStamp.datagram.size());
    return (ExitSuccess);
  }

  /**
   * Send a SOPAS command to the device and print out the response to the console.
   */
  int SickScanCommonTcp::sendSOPASCommand(const char *request, std::vector<unsigned char> *reply, int cmdLen)
  {
#if 0
    if (!socket_.is_open()) {
      ROS_ERROR("sendSOPASCommand: socket not open");
      diagnostics_.broadcast(getDiagnosticErrorCode(), "sendSOPASCommand: socket not open.");
      return ExitError;
    }
#endif
    int sLen = 0;
    int preambelCnt = 0;
    bool cmdIsBinary = false;

    if (request != NULL)
    {
      sLen = cmdLen;
      preambelCnt = 0; // count 0x02 bytes to decide between ascii and binary command
      if (sLen >= 4)
      {
        for (int i = 0; i < 4; i++)
        {
          if (request[i] == 0x02)
          {
            preambelCnt++;
          }
        }
      }

      if (preambelCnt < 4)
      {
        cmdIsBinary = false;
      }
      else
      {
        cmdIsBinary = true;
      }
      int msgLen = 0;
      if (cmdIsBinary == false)
      {
        msgLen = strlen(request);
      }
      else
      {
        int dataLen = 0;
        for (int i = 4; i < 8; i++)
        {
          dataLen |= ((unsigned char) request[i] << (7 - i) * 8);
        }
        msgLen = 8 + dataLen + 1; // 8 Msg. Header + Packet +
      }
#if 1
      if (getEmulSensor())
      {
        emulateReply((UINT8 *) request, msgLen, reply);
      }
      else
      {
        bool debugBinCmd = false;
        if (debugBinCmd)
        {
          printf("=== START HEX DUMP ===\n");
          for (int i = 0; i < msgLen; i++)
          {
            unsigned char *ptr = (UINT8 *) request;
            printf("%02x ", ptr[i]);
          }
          printf("\n=== END HEX DUMP ===\n");
        }
        m_nw.sendCommandBuffer((UINT8 *) request, msgLen);
      }
#else

      /*
       * Write a SOPAS variable read request to the device.
       */
      try
      {
        boost::asio::write(socket_, boost::asio::buffer(request, msgLen));
      }
      catch (boost::system::system_error &e)
      {
        ROS_ERROR("write error for command: %s", request);
        diagnostics_.broadcast(getDiagnosticErrorCode(), "Write error for sendSOPASCommand.");
        return ExitError;
      }
#endif
    }

    // Set timeout in 5 seconds
    const int BUF_SIZE = 65536;
    char buffer[BUF_SIZE];
    int bytes_read;
    // !!!
    if (getEmulSensor())
    {

    }
    else
    {
      if (readWithTimeout(getReadTimeOutInMs(), buffer, BUF_SIZE, &bytes_read, 0, cmdIsBinary) == ExitError)
      {
        ROS_INFO_THROTTLE(1.0, "sendSOPASCommand: no full reply available for read after %d ms", getReadTimeOutInMs());
        diagnostics_.broadcast(getDiagnosticErrorCode(),
                               "sendSOPASCommand: no full reply available for read after timeout.");
        return ExitError;
      }


      if (reply)
      {
        reply->resize(bytes_read);

        std::copy(buffer, buffer + bytes_read, &(*reply)[0]);
      }
    }
    return ExitSuccess;
  }


  int SickScanCommonTcp::get_datagram(ros::Time &recvTimeStamp, unsigned char *receiveBuffer, int bufferSize,
                                      int *actual_length,
                                      bool isBinaryProtocol, int *numberOfRemainingFifoEntries)
  {
    if (NULL != numberOfRemainingFifoEntries)
    {
      *numberOfRemainingFifoEntries = 0;
    }
    this->setReplyMode(1);

    if (this->getEmulSensor())
    {
      // boost::this_thread::sleep(boost::posix_time::milliseconds(waitingTimeInMs));
      ros::Time timeStamp = ros::Time::now();
      uint32_t nanoSec = timeStamp.nsec;
      double waitTime10Hz = 10.0 * (double) nanoSec / 1E9;  // 10th of sec. [0..10[

      uint32_t waitTime = (int) waitTime10Hz; // round down

      double waitTimeUntilNextTime10Hz = 1 / 10.0 * (1.0 - (waitTime10Hz - waitTime));

      ros::Duration(waitTimeUntilNextTime10Hz).sleep();

      SickScanRadarSingleton *radar = SickScanRadarSingleton::getInstance();
      radar->setEmulation(true);
      radar->simulateAsciiDatagram(receiveBuffer, actual_length);
      recvTimeStamp = ros::Time::now();
    }
    else
    {
      const int maxWaitInMs = getReadTimeOutInMs();
      std::vector<unsigned char> dataBuffer;
#if 1 // prepared for reconnect
      bool retVal = this->recvQueue.waitForIncomingObject(maxWaitInMs);
      if (retVal == false)
      {
        ROS_WARN("Timeout during waiting for new datagram");
        return ExitError;
      }
      else
      {
        // Look into receiving queue for new Datagrams
        //
        //
        DatagramWithTimeStamp datagramWithTimeStamp = this->recvQueue.pop();
        if (NULL != numberOfRemainingFifoEntries)
        {
          *numberOfRemainingFifoEntries = this->recvQueue.getNumberOfEntriesInQueue();
        }
        recvTimeStamp = datagramWithTimeStamp.timeStamp;
        dataBuffer = datagramWithTimeStamp.datagram;

      }
#endif
      // dataBuffer = this->recvQueue.pop();
      long size = dataBuffer.size();
      memcpy(receiveBuffer, &(dataBuffer[0]), size);
      *actual_length = size;
    }

#if 0
    static int cnt = 0;
    char szFileName[255];
    sprintf(szFileName, "/tmp/dg%06d.bin", cnt++);

    FILE *fout;

    fout = fopen(szFileName, "wb");
    if (fout != NULL)
    {
      fwrite(receiveBuffer, *actual_length, 1, fout);
      fclose(fout);
    }
#endif
    return ExitSuccess;

    if (!socket_.is_open())
    {
      ROS_ERROR("get_datagram: socket not open");
      diagnostics_.broadcast(getDiagnosticErrorCode(), "get_datagram: socket not open.");
      return ExitError;
    }

    /*
     * Write a SOPAS variable read request to the device.
     */
    std::vector<unsigned char> reply;

    // Wait at most 5000ms for a new scan
    size_t timeout = 30000;
    bool exception_occured = false;

    char *buffer = reinterpret_cast<char *>(receiveBuffer);

    if (readWithTimeout(timeout, buffer, bufferSize, actual_length, &exception_occured, isBinaryProtocol) !=
        ExitSuccess)
    {
      ROS_ERROR_THROTTLE(1.0, "get_datagram: no data available for read after %zu ms", timeout);
      diagnostics_.broadcast(getDiagnosticErrorCode(), "get_datagram: no data available for read after timeout.");

      // Attempt to reconnect when the connection was terminated
      if (!socket_.is_open())
      {
#ifdef _MSC_VER
        Sleep(1000);
#else
        sleep(1);
#endif
        ROS_INFO("Failure - attempting to reconnect");
        return init();
      }

      return exception_occured ? ExitError : ExitSuccess;    // keep on trying
    }

    return ExitSuccess;
  }

} /* namespace sick_scan */
