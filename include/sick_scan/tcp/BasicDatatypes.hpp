#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
//
// BasicDatatypes.hpp
//
// Defines very basic structures and types
// Copyright (c) Sick AG
// created: 31.05.2010
/*
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
*/
// HISTORY
//
// 1.0.0	31.05.2010, VWi
//			Initial version.


#ifndef BASICDATATYPES_HPP
#define BASICDATATYPES_HPP

#include <string>	// for std::string
#include <vector>	// for std::vector
#include <stdint.h>

//
// Standard-Datentypen
//
typedef uint64_t      UINT64;
typedef int32_t       INT32;
typedef uint32_t      UINT32;
typedef uint16_t      UINT16;
typedef int16_t       INT16;
typedef uint8_t       UINT8;
typedef int8_t        INT8;
typedef unsigned char BYTE;

#ifndef PI
	#define PI 3.141592653589793238462
#endif
#ifndef deg2rad
	#define deg2rad 0.01745329251994329576923690768 	// (PI / 180.0)
#endif
#ifndef rad2degMultiplier
	#define rad2degMultiplier 57.29577951308232087679815481		// (180.0 / PI)
#endif


//
enum Datatypes
{
	Datatype_Unknown 				= 0x0000,
	
	Datatype_MeasurementList		= 0x0001,
	Datatype_Box2D					= 0x0002,
	Datatype_Line2D					= 0x0003,
	Datatype_Polygon2D				= 0x0004,
	Datatype_Polygon3D				= 0x0005,
	Datatype_Point2D				= 0x0006,
	Datatype_Point3D				= 0x0007,
	Datatype_Circle2D				= 0x0008,
	Datatype_Ellipse2D				= 0x0009,
	Datatype_Msg					= 0x000A,
	Datatype_Scan					= 0x000B,
	Datatype_Objects				= 0x000C,
	Datatype_Scannerinfo			= 0x000D,
	Datatype_Trigger				= 0x000E,
	Datatype_EvalCaseResult			= 0x000F,
	Datatype_EvalCaseResults		= 0x0010,
	Datatype_EvalCase				= 0x0011,
	Datatype_EvalCases				= 0x0012,
	Datatype_FieldParameter			= 0x0013,
	Datatype_FieldDescription		= 0x0014,
	Datatype_Fields					= 0x0015,
	Datatype_SensorStateInfo		= 0x0016
};

//
// Type-IDs of modules (devices, applications, ...)
//
enum Sourcetype
{
	Sourcetype_Unknown 				= 0x0000,
	
	// Devices = 0x0001 - 0x0FFF
	Sourcetype_LDMRS				= 0x0003,

	// Applications = 0x1000 - 0x1FFF
	Sourcetype_MrsApp				= 0x1002,
	Sourcetype_MrsChangeApp			= 0x1003,
	Sourcetype_MrsFieldApp			= 0x1004,
	Sourcetype_MrsNtpTimeApp		= 0x1005,
	Sourcetype_MrsScanpointCoordinateApp = 0x1006
};

namespace datatypes
{

// Datencontainer fuer alle transportierbaren Daten
class BasicData
{
public:
	BasicData() {m_datatype = Datatype_Unknown; m_sourceId = Sourcetype_Unknown;}
	virtual ~BasicData() {}
	
	UINT16 getDatatype() {return m_datatype;}
	UINT16 getSourceId() {return m_sourceId;}
	virtual void setSourceId(UINT16 id) {m_sourceId = id;}
	virtual const UINT32 getUsedMemory() const = 0;
	
protected:
	UINT16 m_datatype;		// Typ dieses Datums
	UINT16 m_sourceId;		// Unique ID of the data source

private:
};


}	// namespace datatypes

#endif // BASICDATATYPES_HPP
