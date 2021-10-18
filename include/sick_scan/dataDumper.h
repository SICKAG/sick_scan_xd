#ifndef DATA_DUMPER_H
#define DATA_DUMPER_H

#include <string>
#include <vector>

#define DEBUG_DUMP_ENABLED 0
//#define DEBUG_DUMP_TO_CONSOLE_ENABLED

class DataDumper
{
public:
  static DataDumper &instance()
  {
    static DataDumper _instance;
    return _instance;
  }

  ~DataDumper()
  {}

  int pushData(double timeStamp, std::string info, double val);

  int writeDataToCsv(std::string fileName);

  int writeToFileNameWhenBufferIsFull(std::string filename);

  int dumpUcharBufferToConsole(unsigned char *buffer, int bufLen);

  /*!
   * Converts and returns binary data to ascii string with non-printable data represented as "\x<hexvalue>"
   * @param[in] binary_data binary input data
   * @return hex string
   */
  static std::string binDataToAsciiString(const uint8_t* binary_data, int length);

  int testbed();

private:
  const int maxFifoSize = 10000;
  std::vector<double> timeStampVec;
  std::vector<std::string> infoVec;
  std::vector<double> dataVec;
  int pushCounter;

  DataDumper()
  {
    timeStampVec.resize(maxFifoSize);
    infoVec.resize(maxFifoSize);
    dataVec.resize(maxFifoSize);
    pushCounter = 0;
  }           // verhindert, dass ein Objekt von au�erhalb von N erzeugt wird.
  // protected, wenn man von der Klasse noch erben m�chte
  DataDumper(const DataDumper &); /* verhindert, dass eine weitere Instanz via
								   Kopier-Konstruktor erstellt werden kann */
  DataDumper &operator=(const DataDumper &); //Verhindert weitere Instanz durch Kopie
  std::string dumpFileName;
};

/* Verwendung:
DataDumper& s = DataDumper::instance();
s.xyz();
//oder
DataDumper::instance().xyz(); */
#endif