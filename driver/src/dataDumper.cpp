// "dataDumper.cpp": Definiert den Einstiegspunkt für die Konsolenanwendung.
//
#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <iomanip>
#include "dataDumper.h"

int DataDumper::pushData(double timeStamp, std::string info, double val)
{
	int retCode = 0;
	if (pushCounter < maxFifoSize)
	{
		timeStampVec[pushCounter] = timeStamp;
		infoVec[pushCounter] = info;
		dataVec[pushCounter] = val;

	}
	else
	{
		retCode = 2;
		if (pushCounter == maxFifoSize)
		{
			if (dumpFileName.length() > 0)
			{
				writeDataToCsv(dumpFileName);
				retCode = 1;
			}
		}
	}
	pushCounter++;
	return (retCode);
}

int DataDumper::writeDataToCsv(std::string filename)
{
	FILE *fout;
	int retCode = 0;
	fout = fopen(filename.c_str(), "w");
	if (fout != NULL)
	{
		for (int i = 0; i < pushCounter; i++)
		{
			fprintf(fout, "%8.6lf;%-10s;%12.8lf\n", timeStampVec[i], infoVec[i].c_str(), dataVec[i]);
		}

		fclose(fout);
	}
	else
	{
		retCode = -1;
	}
	return (retCode);
}


int DataDumper::writeToFileNameWhenBufferIsFull(std::string filename)
{
	dumpFileName = filename;
	return (0);
}

int DataDumper::dumpUcharBufferToConsole(unsigned char *buffer, int bufLen)
{
  int ret = 0;
	char asciiBuffer[255] = {0};
	for (int i = 0; i < bufLen; i++)
	{
		if ((i % 8) == 0)
		{
			printf("%08x: ", i);
		}

		printf("%02x ", buffer[i]);

		char ch = '.';
		if ((buffer[i] >= 0x20) && (buffer[i] < 0x80))
		{
			ch = (char) buffer[i];
		}
		int iStrLen = strlen(asciiBuffer);
		asciiBuffer[iStrLen] = ch;
		asciiBuffer[iStrLen + 1] = '\0';

		if ((i % 8) == 7)
		{
			printf("%s\n", asciiBuffer);
			strcpy(asciiBuffer, "");
		}
	}
	if (bufLen % 8)
	{

		printf("%s\n", asciiBuffer);
	}
	return(ret);
}

/*!
 * Converts and returns binary data to ascii string with non-printable data represented as "\x<hexvalue>"
 * @param[in] binary_data binary input data
 * @return hex string
 */
std::string DataDumper::binDataToAsciiString(const uint8_t* binary_data, int length)
{
  std::stringstream out;
  for(int n = 0; n < length; n++)
  {
    int val = (int)(binary_data[n] & 0xFF);
    if ((val == 0x20) || (val >= 48 && val <= 57) || (val >= 65 && val <= 90) || (val >= 97 && val <= 122))
    {
      out << std::string(1,(char)(val & 0xFF));
    }
    else
    {
      out <<  "\\x" << std::setfill('0') << std::setw(2) << std::hex << val;
    }
  }
  return out.str();
}


int DataDumper::testbed()
{

	double testSignal;
	double sampleRate = 1000.0;
	double freq = 50.0;
	DataDumper::instance().writeToFileNameWhenBufferIsFull("C:\\temp\\000_full.csv");

	int cnt = 0;
	for (double t = 0; t < 5.0; t += 1.0 / sampleRate)
	{
		cnt++;
		testSignal = 1.0 * cos(2 * M_PI * freq * t);
		if (cnt == 1000)
		{
			DataDumper::instance().writeDataToCsv("C:\\temp\\000_partial.csv");
		}
		DataDumper::instance().pushData(t, "COS", testSignal);
		testSignal = 1.0 * sin(2 * M_PI * freq * t);
		DataDumper::instance().pushData(t, "SIN", testSignal);
	}
	return 0;
}

#ifdef TEST_DATA_DUMPER
int main(int argc, char *argv)
{
	DataDumper::instance().testbed();
}
#endif
