/*
 * Copyright (C) 2018, SICK AG
 * Copyright (C) 2018, Ing.-Buero Dr. Michael Lehning
 * All rights reserved.
 *
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*       http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
*
*
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
*     * Neither the name of Osnabrueck University nor the names of its
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
 */

#define  _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <string>
#include <vector>
#include <assert.h>
#include <cstdlib>

#include "pcl_converter/gnuplotPaletteReader.h"

GnuPlotPalette::GnuPlotPalette()
{
  setErrorStatus(0);
}


int GnuPlotPalette::load(std::string palettName)
{
  int errorCode= 0;
	bool debugDump = true;
	FILE *fin;
	const int MAX_LINE_LEN = (1024);
	char szLine[MAX_LINE_LEN] = { 0 };
	std::vector<std::string> paletteData;
	fin = fopen(palettName.c_str(), "r");
	setErrorStatus(0);
	if (NULL == fin)
	{
		setErrorStatus(-1);
	}
	else
	{
		while (fgets(szLine, MAX_LINE_LEN, fin) != NULL)
		{
			paletteData.push_back(szLine);
		}
		fclose(fin);
	}

	std::string lineStylePrefix = "# line styles";
	std::string setStyleLinePrefix = "set style line";

	int tableStart = -1;
	int tableEnd = -1;
	for (size_t i = 0; i < paletteData.size(); i++)
	{
		if (!paletteData[i].compare(0, setStyleLinePrefix.size(), setStyleLinePrefix))
		{
			if (-1 != tableStart)
			{
				tableEnd = i;
			}
		}

		if (!paletteData[i].compare(0, lineStylePrefix.size(), lineStylePrefix))
		{
			tableStart = i + 1;
		}
	}


	if ((tableStart != -1) && (tableEnd != -1))
	{
		size_t tableLen = tableEnd - tableStart + 1;
		std::vector<unsigned long> rgbSupportPoints;
		if (debugDump) printf("Parsing Table-Entries\n");

		for (int i = tableStart; i <= tableEnd; i++)
		{
			size_t startPos = paletteData[i].find_first_of("'");
			size_t endPos = paletteData[i].find_last_of("'");
			if ((startPos != std::string::npos) && (endPos != std::string::npos))
			{
				// The code is given in the format '#AABBCC'
				// We skip the '#' by adding + 1 to startPos
				std::string rgbValue = paletteData[i].substr(startPos + 2, endPos - startPos - 2);
				unsigned long hex_value = std::strtoul(rgbValue.c_str(), 0, 16);
				rgbSupportPoints.push_back(hex_value);
			}
		}

		// lattice fence problem
		int numInterpolationIntervals = tableLen - 1;
		float colorsPerInterval = 255.0f / (float)numInterpolationIntervals;
		for (size_t i = 0; i < 256; i++)
		{
			unsigned char rgbValues[3];
			int supportPntIdx = (int)(i / colorsPerInterval);
			int distanceToSupportPoint = (int)(i - supportPntIdx * colorsPerInterval);
			double frac = distanceToSupportPoint / (double)colorsPerInterval;
			if (debugDump) printf("%3d: ", (int)i);
			for (size_t j = 0; j < 3; j++)
			{
				float frac_epsilon = 1E-6f;
				unsigned char valLower = 0x00;
				unsigned char valUppper = 0x00;
				unsigned char val;
				int bytePosInBits = 8 * (2 - j);
				unsigned long mask = 0xFF << bytePosInBits; // create bit mask
				valLower = (unsigned char)((rgbSupportPoints[supportPntIdx] & mask) >> bytePosInBits);

				val = valLower; // preset as fallback
				if (frac > frac_epsilon)
				{
					if (supportPntIdx < (numInterpolationIntervals - 1)) // paranoid
					{
						valUppper = (unsigned char)((rgbSupportPoints[supportPntIdx + 1] & mask) >> bytePosInBits);
						val = (unsigned char)(valLower + frac * (valUppper - valLower));
					}
				}
				else
				{
					val = valLower;
				}
				rgbValues[j] = val;
				if (debugDump) printf("%02x ", rgbValues[j]);
			}
			for (size_t j = 0; j < 3; j++)
			{
				rgbTable[i][j] = rgbValues[j];
				if (debugDump) printf("%8.6lf ", rgbValues[j] / 255.0); // 254.67
			}
			if (debugDump) printf("\n");
		}

	}

  errorCode = getErrorStatus();
  return(errorCode);
}

unsigned char GnuPlotPalette::getRbgValue(unsigned char greyIdx, unsigned int channelIdx)
{
	// channelIdx: 0: RED, 1: GREEN, 2: BLUE
	assert(channelIdx < 3);
	return(rgbTable[greyIdx][channelIdx]);
}

#if 0
int main()
{
	GnuPlotPalette pal("C:\\temp\\viridis.pal");
	return 0;
}
#endif
