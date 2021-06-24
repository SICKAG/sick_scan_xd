/*
* Copyright (C) 2018, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2018, SICK AG, Waldkirch
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
*     * Neither the name of Osnabr�ck University nor the names of its
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
*  Last modified: 27th Mar 2018
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*
*/

#define WIN32_LEAN_AND_MEAN
#include "boost/filesystem.hpp"
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <algorithm> // for std::min


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#ifndef _MSC_VER
#include <sys/wait.h>
#endif
#include <unistd.h>
#include <cstdio>
#include <cstdlib>


#include "sick_scan/sick_generic_laser.h"

#include <ros/console.h>

#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif

#include "tinystr.h"
#include "tinyxml.h"

#define MAX_NAME_LEN (1024)

// 001.000.000 Initial Version
#define SICK_SCAN_TEST_MAJOR_VER "001"
#define SICK_SCAN_TEST_MINOR_VER "000"  
#define SICK_SCAN_TEST_PATCH_LEVEL "000"



class paramEntryAscii
{
public:
	paramEntryAscii(std::string _nameVal, std::string _typeVal, std::string _valueVal)
	{
		nameVal = _nameVal;
		typeVal = _typeVal;
		valueVal = _valueVal;
    setCheckStatus(999,"untested");
    minMaxGiven = false;
	};

  void setPointerToXmlNode(TiXmlElement *paramEntryPtr)
  {
    this->nodePtr = paramEntryPtr;
  }

    TiXmlElement * getPointerToXmlNode(void)
    {
      return( this->nodePtr);
    }
	void setValues(std::string _nameVal, std::string _typeVal, std::string _valueVal)
	{
		nameVal = _nameVal;
		typeVal = _typeVal;
		valueVal = _valueVal;
	};


  bool isMinMaxGiven()
  {
    return(minMaxGiven);
  }

  void setMinMaxValues(std::string _valueMinVal, std::string _valueMaxVal)
    {

    valueMinVal = _valueMinVal;
    valueMaxVal = _valueMaxVal;
    minMaxGiven = true;

    };

	std::string getName()
	{
		return(nameVal);
	}

	std::string getType()
	{
		return(typeVal);
	}

	std::string getValue()
	{
		return(valueVal);
	}

  std::string getMinValue()
  {
    return(valueMinVal);
  }

  std::string getMaxValue()
  {
    return(valueMaxVal);
  }

    void setCheckStatus(int errCode, std::string errMsg)
  {
    errorCode = errCode;
    errorMsg = errMsg;
  };

    int getErrorCode()
    {
      return(errorCode);
    }

    std::string getErrorMsg()
    {
      return(errorMsg);
    }

private:
	std::string nameVal;
	std::string typeVal;
	std::string valueVal;
  std::string valueMinVal;
  std::string valueMaxVal;
  bool minMaxGiven;
  int errorCode;
  std::string errorMsg;
    TiXmlElement *nodePtr;
};





void sudokill(pid_t tokill)
{
#ifndef _MSC_VER
  kill(tokill, SIGTERM);
#endif
  sleep(5);
}

std::vector<paramEntryAscii> getParamList(TiXmlNode *paramList)
{
	std::vector<paramEntryAscii> tmpList;


	TiXmlElement *paramEntry = (TiXmlElement *)paramList->FirstChild("param"); // first child
	while (paramEntry)
	{
		std::string nameVal = "";
		std::string typeVal = "";
		std::string valueVal = "";
    std::string minValueVal = "";
    std::string maxValueVal = "";

    bool minValFnd = false;
    bool maxValFnd = false;
		// is this a param-node?
		// if this is valid than process attributes
		const char *val = paramEntry->Value();
		bool searchAttributes = true;
		if (strcmp(val,"param") == 0)
		{
			// expected value
		}
		else
		{
			searchAttributes = false;
		}
		if (paramEntry->Type() == TiXmlNode::TINYXML_ELEMENT)
		{
			// expected value
		}
		else
		{
			searchAttributes = false;
		}
		if (searchAttributes)
		{
		for (TiXmlAttribute* node = paramEntry->FirstAttribute(); ; node = node->Next())
		{
			const char *tag = node->Name();
			const char *val = node->Value();

			if (strcmp(tag, "name") == 0)
			{
				nameVal = val;
			}
			if (strcmp(tag, "type") == 0)
			{
				typeVal = val;
			}
			if (strcmp(tag, "value") == 0)
			{
				valueVal = val;
			}
      if (strcmp(tag, "valueMin") == 0)
      {
        minValFnd = true;
        minValueVal = val;

      }
      if (strcmp(tag, "valueMax") == 0)
      {
        maxValFnd = true;
        maxValueVal = val;
      }
			if (node == paramEntry->LastAttribute())
			{

				break;
			}
		}

		paramEntryAscii tmpEntry(nameVal, typeVal, valueVal);
    if (maxValFnd && minValFnd)
    {
      tmpEntry.setMinMaxValues(minValueVal, maxValueVal);
    }

    tmpEntry.setPointerToXmlNode(paramEntry);
		tmpList.push_back(tmpEntry);
		}
		paramEntry = (TiXmlElement *)paramEntry->NextSibling();  // go to next sibling
	}

	return(tmpList);
}



void replaceParamList(TiXmlNode * node, std::vector<paramEntryAscii> paramOrgList)
{
	bool fndParam = false;
	do
	{
		fndParam = false;
		TiXmlElement *paramEntry = (TiXmlElement *)node->FirstChild("param");
		if (paramEntry != NULL)
		{
			fndParam = true;
			node->RemoveChild(paramEntry);
		}
	} while (fndParam == true);

	for (int i = 0; i < paramOrgList.size(); i++)
	{
		paramEntryAscii tmp = paramOrgList[i];
		TiXmlElement *paramTmp = new TiXmlElement("param");
		paramTmp->SetAttribute("name", tmp.getName().c_str());
		paramTmp->SetAttribute("type", tmp.getType().c_str());
		paramTmp->SetAttribute("value", tmp.getValue().c_str());
		node->LinkEndChild(paramTmp);
	}
}

pid_t pid;

pid_t launchRosFile(std::string cmdLine)
{
  std::string cmd = cmdLine;
  typedef std::vector<std::string > split_vector_type;

  split_vector_type splitVec; // #2: Search for tokens
  boost::split(splitVec, cmdLine, boost::is_any_of(" "), boost::token_compress_on); // SplitVec == { "hello abc","ABC","aBc goodbye" }



  pid_t pid = fork(); // create child process
  int status;

  switch (pid)
  {
    case -1: // error
      perror("fork");
      exit(1);

    case 0: // child process
      execl("/opt/ros/kinetic/bin/roslaunch", splitVec[0].c_str(), splitVec[1].c_str(),  splitVec[2].c_str(), NULL); // run the command
      perror("execl"); // execl doesn't return unless there is a problem
      exit(1);

    default: // parent process, pid now contains the child pid
  /*
      while (-1 == waitpid(pid, &status, 0)); // wait for child to complete
      if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
      {
        // handle error
        std::cerr << "process " << cmd << " (pid=" << pid << ") failed" << std::endl;
      }
      */
      break;

  }

  return(pid);
}


int createTestLaunchFile(std::string launchFileFullName, std::vector<paramEntryAscii> entryList, std::string& testLaunchFile)
{
	printf("Try loading launchfile :%s",launchFileFullName.c_str());
	TiXmlDocument doc;
	doc.LoadFile(launchFileFullName.c_str());

	if (doc.Error() == true)
	{
		ROS_ERROR("ERROR parsing launch file %s\nRow: %d\nCol: %d", doc.ErrorDesc(), doc.ErrorRow(), doc.ErrorCol());
		return(-1);
	}
	TiXmlNode *node = doc.FirstChild("launch");
	if (node != NULL)
	{
		node = node->FirstChild("node");
		std::vector<paramEntryAscii> paramOrgList = getParamList(node);
		for (int i = 0; i < entryList.size(); i++)
		{
			bool replaceEntry = false;
			for (int j = 0; j < paramOrgList.size(); j++)
			{
				if (entryList[i].getName().compare(paramOrgList[j].getName()) == 0)
				{
					paramOrgList[j].setValues(entryList[i].getName(), entryList[i].getType(), entryList[i].getValue());
					replaceEntry = true;
				}
			}
			if (replaceEntry == false)  // add new one to list
			{
				paramEntryAscii tmp(entryList[i].getName(), entryList[i].getType(), entryList[i].getValue());
				paramOrgList.push_back(tmp);
			}
		}
		// delete all parameters and replace with new one
		replaceParamList(node, paramOrgList);
	}

	// append source extension if any 
	size_t pos = launchFileFullName.rfind('.');
	std::string resultFileName = "";
	testLaunchFile = "";
	if (pos != std::string::npos)
	{
		resultFileName = launchFileFullName.substr(0, pos);
		resultFileName += "_test.launch";
		doc.SaveFile(resultFileName.c_str());
		testLaunchFile = resultFileName;
	}
	return(0);
}

int cloud_width = 0;  // hacky 
int cloud_height = 0;
int callbackCnt = 0;
double intensityStdDev = 0.0;
double rangeStdDev = 0.0;


//
int callbackScanCnt = 0;


void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	static int cnt = 0;
  callbackCnt++;
	cloud_width = cloud_msg->width;
	cloud_height = cloud_msg->height;
	int intensity_idx = -1;
	int cartesianIdxArr[3];
	for (int i = 0; i < 3; i++)
	{
		cartesianIdxArr[i] = -1;
	}


	for (int i = 0; i < cloud_msg->fields.size(); i++)
	{
		std::string fieldName = 	cloud_msg->fields[i].name;
		if (fieldName.compare("intensity") == 0)
		{
			intensity_idx = i;
		}
		if (fieldName.compare("x") == 0)
		{
			cartesianIdxArr[0] = i;
		}
		if (fieldName.compare("y") == 0)
		{
			cartesianIdxArr[1] = i;
		}
		if (fieldName.compare("z") == 0)
		{
			cartesianIdxArr[2] = i;
		}

	}
	if (intensity_idx != -1)
	{
		float intensitySum = 0.0;
		float intensity2Sum = 0.0;
		int intensityNum = cloud_width * cloud_height;
	for (int i = 0; i < cloud_height; i++)
		for (int j = 0; j < cloud_width; j++)
		{
			int idx = i * cloud_width + j;
			idx *= cloud_msg->point_step;  // calculate byte address offset
			int relOff = cloud_msg->fields[intensity_idx].offset;
			float *intensityPtr = (float *)(&(cloud_msg->data[idx+relOff]));
			float intensity = *intensityPtr;
			intensitySum += intensity;
			intensity2Sum += intensity * intensity;
		}
		if (intensityNum > 1)
		{
			intensityStdDev = (intensity2Sum - 1.0 / intensityNum * (intensitySum * intensitySum));
			{
				intensityStdDev /= (intensityNum - 1);
				intensityStdDev = sqrt(intensityStdDev);
			}
		}
	}

	if (cartesianIdxArr[0] != -1 &&cartesianIdxArr[1] != -1 &&cartesianIdxArr[2] != -1)
	{
		float rangeSum = 0.0;
		float range2Sum = 0.0;
		int rangeNum = cloud_width * cloud_height;
		for (int i = 0; i < cloud_height; i++)
			for (int j = 0; j < cloud_width; j++)
			{
				int idx = i * cloud_width + j;
				idx *= cloud_msg->point_step;  // calculate byte address offset
				int relOffX = cloud_msg->fields[cartesianIdxArr[0]].offset;
				float *XPtr = (float *)(&(cloud_msg->data[idx+relOffX]));
				float X = *XPtr;
				int relOffY = cloud_msg->fields[cartesianIdxArr[1]].offset;
				float *YPtr = (float *)(&(cloud_msg->data[idx+relOffY]));
				float Y = *YPtr;
				int relOffZ = cloud_msg->fields[cartesianIdxArr[2]].offset;
				float *ZPtr = (float *)(&(cloud_msg->data[idx+relOffZ]));
				float Z = *ZPtr;
				float range =sqrt(X*X+Y*Y+Z*Z);
				rangeSum += range;
				range2Sum += range * range;
			}
		if (rangeNum > 1)
		{
			rangeStdDev = (range2Sum - 1.0 / rangeNum * (rangeSum * rangeSum));
			{
				rangeStdDev /= (rangeNum - 1);
				rangeStdDev = sqrt(intensityStdDev);
			}
		}
	}
	ROS_INFO("valid PointCloud2 message received");

}

float ERR_CONST = -1E3;
float scan_angle_min = -1E3;
float scan_angle_max = -1E3;
float scan_time_increment = -1E3;
float scan_angle_increment = -1E3;
float range_min = -1E3;
float range_max = -1E3;


void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  if (callbackScanCnt == 0)
  {
    scan_angle_min = scan->angle_min;
    scan_angle_max = scan->angle_max;
    scan_time_increment = scan->time_increment;
    scan_angle_increment = scan->angle_increment;
    range_min = scan->range_min;
    range_max = scan->range_max;
  }
  callbackScanCnt++;
}


int startCallbackTest(int argc, char** argv)
{

	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	ros::Subscriber sub;
  ros::Subscriber scanSub;
  ROS_INFO("Cloudtester started."
                  );
	sub = nh.subscribe("cloud", 1, cloud_callback);
  scanSub=nh.subscribe<sensor_msgs::LaserScan>("/scan",10,processLaserScan);

  while (callbackCnt < 10)
  {
  ros::spinOnce();
  }
  sub.shutdown();
  ros::shutdown();

  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	return(0);

}


void createPrefixFromExeName(std::string exeName, std::string& prefix)
{
#ifdef _MSC_VER
#else
#endif
	std::string searchPattern = "(.*)cmake\\-build\\-debug.*";
	boost::regex expr(searchPattern);
	boost::smatch what;

	if (boost::regex_search(exeName, what, expr))
	{
		std::string prefixPath = what[1];  // started from CLION IDE - build complete path
		prefix = prefixPath;
		prefix += "test";
	}
	else
	{
		prefix = ".";
	}

}


void searchForXmlFileInPathList(std::string pathList, std::string& packagePath, std::string xmlFileName)
{
	typedef std::vector<std::string > split_vector_type;
	packagePath = "???";
	split_vector_type splitVec; // #2: Search for tokens
	boost::split(splitVec, pathList, boost::is_any_of(":"), boost::token_compress_on); // SplitVec == { "hello abc","ABC","aBc goodbye" }
	if (splitVec.size() > 0)
	{
		packagePath = splitVec[0];
	}

	for (int i = 0; i < splitVec.size(); i++)
	{
		FILE *fin;
		std::string tmpFileName = splitVec[i] + "/" + xmlFileName;
		fin = fopen(tmpFileName.c_str(),"r");
		if (fin != NULL)
		{
			packagePath = splitVec[i];
			fclose(fin);
		}
	}
}

bool getPackageRootFolder(std::string exePath, std::string& packageRootFolder)
{
	 packageRootFolder = "";
	// try to retrive package name from exePat
	typedef std::vector<std::string > split_vector_type;
	split_vector_type splitVec; // #2: Search for tokens
	boost::split(splitVec, exePath, boost::is_any_of("/"), boost::token_compress_on); // SplitVec == { "hello abc","ABC","aBc goodbye" }
	bool bFnd = false;
	for (int i = splitVec.size() - 1; i >= 0; i--)  // last part is exe-Name - so forget this
	{
		std::string tmpPath = "";
		for (int j = 1; j < i; j++)
		{
			tmpPath += '/';
			tmpPath += splitVec[j];
		}

		std::string fileTmpPath = tmpPath;
		fileTmpPath += "/package.xml";
		if (boost::filesystem::exists(fileTmpPath))
		{
			packageRootFolder = tmpPath;
			bFnd = true;
			break;
		}
	}

	return(bFnd);

}

/*!
\brief Startup routine
\param argc: Number of Arguments
\param argv: Argument variable
\return exit-code
*/
int main(int argc, char **argv)
{
	int result = 0;
	char sep = boost::filesystem::path::preferred_separator;
	std::string prefix = ".";
	std::string packageRootFolder;

	std::string exeName = argv[0];

  if (argc == 1)
  {
    printf("Usage: %s <XML-Test-Ctrl-File>\n", exeName.c_str());
    printf("\n");
    printf("The XML-Test-Ctrl-File controls the parameter checking of various scanner types.");
    printf("The XML-File sick_scan_test.xml in the test-Directory shows an example of this file type.\n");
    exit(-1);
  }
  else
  {
    printf("Given program arguments:\n");
      for (int i = 0; i < argc; i++)
      {
          printf("%d: %s\n", i, argv[i]);
      }
  }
#if 0
	createPrefixFromExeName(exeName, prefix);
#ifdef _MSC_VER
	prefix = std::string("..") + sep + std::string("..") + sep + "test";
#endif
	ROS_INFO("sick_scan_test V. %s.%s.%s", SICK_SCAN_TEST_MAJOR_VER, SICK_SCAN_TEST_MINOR_VER, SICK_SCAN_TEST_PATCH_LEVEL);
#endif

	ros::init(argc, argv, "cloudtester");

	std::string nodeName = ros::this_node::getName();

	printf("Nodename:  %s\n", nodeName.c_str());
	bool bFnd = getPackageRootFolder(argv[0], packageRootFolder);

	printf("Package Path: %s\n", packageRootFolder.c_str());

	char* pPath;
	pPath = getenv("ROS_PACKAGE_PATH");
#ifdef _MSC_VER
	pPath = "..\\..:\tmp";
#endif
	if (pPath != NULL)
	{
		ROS_INFO("Current root path entries for launch file search: %s\n", pPath);
	}
	else
	{
		ROS_FATAL("Cannot find ROS_PACKAGE_PATH - please run <Workspace>/devel/setup.bash");
	}

	std::string packagePath;
  // try to find xml-file in path list ... and give the result back in the variable packagePath
  std::string testCtrlXmlFileName = argv[1];
  searchForXmlFileInPathList(pPath, packagePath, testCtrlXmlFileName);

//	std::string testCtrlXmlFileName = packagePath + std::string(1, sep) + "sick_scan" + std::string(1, sep) + "test" + std::string(1, sep) + "sick_scan_test.xml";
//  $ROS_PACKAGE_PATH/sick_scan/test/sick_scan_test.xml
  boost::replace_all(testCtrlXmlFileName, "$ROS_PACKAGE_PATH", packagePath);


	testCtrlXmlFileName = packagePath + std::string(1,'/') + testCtrlXmlFileName;
  TiXmlDocument doc;
	doc.LoadFile(testCtrlXmlFileName.c_str());
	if (doc.Error())
	{
		ROS_ERROR("Package root folder %s", packageRootFolder.c_str());
		ROS_ERROR("Cannot load/parse %s", testCtrlXmlFileName.c_str());
		ROS_ERROR("Details: %s\n", doc.ErrorDesc());
		exit(-1);
	}

  bool launch_only = false;
	boost::filesystem::path p(testCtrlXmlFileName);
	boost::filesystem::path parentPath = p.parent_path();
	std::string pathName = parentPath.string();
	std::string launchFilePrefix = parentPath.parent_path().string() + std::string(1, sep) + "launch" + std::string(1, sep);
	TiXmlNode *node = doc.FirstChild("launch");
	if (node == NULL)
	{
		ROS_ERROR("Cannot find tag <launch>\n");
		exit(-1);
	}

	if (node)
	{
		TiXmlElement *fileNameEntry;
		fileNameEntry = (TiXmlElement *)node->FirstChild("filename");
		if (fileNameEntry == NULL)
		{
			ROS_ERROR("Cannot find tag <filename>\n");
			exit(-1);
		}
		TiXmlText *fileNameVal = (TiXmlText *)fileNameEntry->FirstChild();
		if (fileNameVal != NULL)
		{
			ROS_INFO("Processing launch file %s\n", fileNameVal->Value());
			std::string launchFileFullName = launchFilePrefix + fileNameVal->Value();
			ROS_INFO("Processing %s\n", launchFileFullName.c_str());
			TiXmlNode *paramListNode = (TiXmlNode *)node->FirstChild("paramList");
			if (paramListNode != NULL)
			{
				std::vector<paramEntryAscii> paramList = getParamList(paramListNode);
        for (int i = 0; i < paramList.size(); i++)
        {
          if (paramList[i].getName().compare("launch_only") == 0)
          {
            if (paramList[i].getValue().compare("true") == 0)
            {
              printf("launch_only set to true. Just modifying launch file and launch (without testing).");
              launch_only = true;
            }
          }


        }

        std::string testLaunchFile;
				createTestLaunchFile(launchFileFullName, paramList, testLaunchFile);
				std::string commandLine;
				boost::filesystem::path p(testLaunchFile);
				std::string testOnlyLaunchFileName = p.filename().string();
				commandLine = "roslaunch sick_scan " + testOnlyLaunchFileName;
				ROS_INFO("launch ROS test ... [%s]", commandLine.c_str());

        // int result = std::system(commandLine.c_str());

        pid_t pidId = launchRosFile(commandLine.c_str());

        if (launch_only)
        {
          printf("Launch file [ %s ] started ...\n", commandLine.c_str());
          printf("No further testing...\n");
          exit(0);
        }
				startCallbackTest(argc, argv); // get 10 pointcloud messages 

        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
				ROS_INFO("If you receive an error message like \"... is neither a launch file ... \""
					"please source ROS env. via source ./devel/setup.bash");


				TiXmlNode *resultListNode = (TiXmlNode *)node->FirstChild("resultList");
				if (resultListNode != NULL)
				{
					std::vector<paramEntryAscii> resultList = getParamList(resultListNode);

          enum KeyWord_Idx {MIN_ANG_KEYWORD_IDX, MAX_ANG_KEYWORD_IDX,
              SCAN_TIME_INCREMENT_KEYWORD_IDX,
              SCAN_ANGLE_INCREMENT_KEYWORD_IDX,
              MIN_RANGE_KEYWORD_IDX,
              MAX_RANGE_KEYWORD_IDX,
              KEYWORD_IDX_NUM};

          std::vector<std::string> keyWordList;
          keyWordList.resize(KEYWORD_IDX_NUM);

          keyWordList[MIN_ANG_KEYWORD_IDX] = "min_ang";
          keyWordList[MAX_ANG_KEYWORD_IDX] = "max_ang";
          keyWordList[MIN_RANGE_KEYWORD_IDX] = "range_min";
          keyWordList[MAX_RANGE_KEYWORD_IDX] = "range_max";
          keyWordList[SCAN_ANGLE_INCREMENT_KEYWORD_IDX] = "angle_increment";
          keyWordList[SCAN_TIME_INCREMENT_KEYWORD_IDX] = "time_increment";

					for (int i = 0; i < resultList.size(); i++)
					{

            for (int ki = 0; ki < keyWordList.size(); ki++)
            {

            float measurementVal = .0F;
            std::string keyWordTag = keyWordList[ki];

            switch(ki)
            {
              case MIN_ANG_KEYWORD_IDX:  measurementVal = scan_angle_min; break;
              case MAX_ANG_KEYWORD_IDX:  measurementVal = scan_angle_max; break;
              case SCAN_ANGLE_INCREMENT_KEYWORD_IDX:  measurementVal = scan_angle_increment; break;
              case SCAN_TIME_INCREMENT_KEYWORD_IDX: measurementVal = scan_time_increment; break;
              case   MIN_RANGE_KEYWORD_IDX: measurementVal = range_min; break;
              case   MAX_RANGE_KEYWORD_IDX: measurementVal = range_max; break;
              default: printf("Did not find a correspondence for index %d\n", ki); break;
            }
            char errMsg[255] = {0};
            if (keyWordTag.compare("min_ang") == 0)
            {
              measurementVal = scan_angle_min;
            }
            if (resultList[i].getName().compare(keyWordTag) == 0)
            {

              enum Expected_Idx {MIN_EXP_IDX, DEF_EXP_IDX, MAX_EXP_IDX, EXP_IDX_NUM};
              float expectedValues[EXP_IDX_NUM];
              std::string valString;

              for (int j = 0; j < EXP_IDX_NUM; j++)
              {
                switch(j)
                {
                  case MIN_EXP_IDX:  valString = resultList[i].getMinValue(); break;
                  case DEF_EXP_IDX:  valString = resultList[i].getValue(); break;
                  case MAX_EXP_IDX:  valString = resultList[i].getMaxValue(); break;
                  default: ROS_ERROR("Check index"); break;

                }

                const char *valPtr = valString.c_str();
                if (valPtr != NULL)
                {
                  float tmpVal = .0F;
                  sscanf(valPtr, "%f", &tmpVal);
                  expectedValues[j] = tmpVal;
                }
              }
              valString = resultList[i].getMinValue();

              int errorCode = 0;
              std::string resultStr = "FAILED";
              std::string appendResultStr = " <-----";
              if ( (measurementVal >=  expectedValues[MIN_EXP_IDX]) && (measurementVal <=  expectedValues[MAX_EXP_IDX]))
              {
                appendResultStr = "";
                resultStr = "PASSED";
                errorCode = 0;
              }
              else
              {
                errorCode = 1;
              }
              sprintf(errMsg, "CHECK %s! Key: %-20s %14.9f in [%14.9f,%14.9f] %s", resultStr.c_str(), keyWordTag.c_str(), measurementVal,
                      expectedValues[MIN_EXP_IDX], expectedValues[MAX_EXP_IDX], appendResultStr.c_str());

              printf("%s\n", errMsg);
              resultList[i].setCheckStatus(errorCode, (errorCode == 0) ? "OK" : errMsg);

            }
            }


            if (resultList[i].getName().compare("shotsPerLayer") == 0)
						{
							int expectedWidth = -1;
							std::string valString = resultList[i].getValue();
							const char *valPtr = valString.c_str();
							if (valPtr != NULL)
							{
								sscanf(valPtr, "%d", &expectedWidth);
							}


              ROS_INFO("Test\n");
              int errorCode = 0;
							if (expectedWidth == cloud_width)
							{
								printf("CHECK PASSED! WIDTH %d == %d\n", expectedWidth, cloud_width);
                errorCode = 0;
							}
							else
							{
                printf("CHECK FAILED! WIDTH %d <> %d\n", expectedWidth, cloud_width);
                errorCode = 1;

							}
              resultList[i].setCheckStatus(errorCode, (errorCode == 0) ? "OK" : "Unexpected number of shots");

						}
						if (resultList[i].getName().compare("pointCloud2Height") == 0)
              {
                int expectedHeight = -1;
                std::string valString = resultList[i].getValue();
                const char *valPtr = valString.c_str();
                if (valPtr != NULL)
                {
                  sscanf(valPtr, "%d", &expectedHeight);
                }
                ROS_INFO("Test\n");
                int errorCode = 0;
                if (expectedHeight == cloud_height)
                {
                  printf("CHECK PASSED! HEIGHT %d == %d\n", expectedHeight, cloud_height);
                  errorCode = 0;
                }
                else
                {
                  printf("CHECK FAILED! HEIGHT %d <> %d\n", expectedHeight, cloud_height);
                  errorCode = 1;
                }
                resultList[i].setCheckStatus(errorCode, (errorCode == 0) ? "OK" : "Unexpected pointCloud2 height");
                cloud_height = 0;
              }

            if (resultList[i].getName().compare("RSSIEnabled") == 0)
            {
              int expectedWidth = -1;
              std::string valString = resultList[i].getValue();
              bool rssiEnabled = false;
              if (boost::iequals(valString, "true"))
              {
                rssiEnabled = true;
              } else if (boost::iequals(valString, "false"))
              {
                rssiEnabled = false;
              } else
              {
                ROS_WARN("RSSIEnabled parameter wrong. True or False are valid parameters!\n");
              }

              ROS_INFO("Test\n");
              int errorCode = 0;
              if (rssiEnabled == true)
              {
                if (intensityStdDev >= 1e-5)
                {
                  printf("CHECK PASSED! RSSI Standard deviation %.3e is not 0.\n",intensityStdDev);
                  errorCode = 0;
                } else
                {
                  printf("CHECK FAILED!  RSSI standard deviation is samller than %.10e even though RSSI is enabled\n",intensityStdDev);
                  errorCode = 1;

                }
              }

              if (rssiEnabled == false)
              {
                if (intensityStdDev <= 1e-5)
                {
                  printf("CHECK FAILED! RSSI standard deviation %.3e is bigger than 1e-5 even though RSSI is disabled\n",intensityStdDev);
                  errorCode = 1;
                } else
                {
                  printf("CHECK PASSED!  RSSI Standard deviation %.3e is smaller than 1e-5.\n",intensityStdDev);
                  errorCode = 0;

                }
              }
            }
              if (resultList[i].getName().compare("ranges") == 0)
              {
                int expectedWidth = -1;
                std::string valString = resultList[i].getValue();
                bool rangeTest=false;
                if (boost::iequals(valString, "true"))
                {
                  rangeTest=true;
                }
                else
                {
                  ROS_WARN("ranges parameter wrong. True or False are valid parameters!\n");
                }

                ROS_INFO("Test\n");
                int errorCode = 0;
                if (rangeTest==true)
                {
                  if (rangeStdDev >= 1e-5)
                  {
                    printf("CHECK PASSED! Range Standard deviation %.3e is not 0.\n", rangeStdDev);
                    errorCode = 0;
                  }
                  else
                  {
                    printf("CHECK FAILED!  Range standard deviation %.3e is smaller than 1e-5!\n", rangeStdDev);
                    errorCode = 1;

                  }
                  resultList[i].setCheckStatus(errorCode, (errorCode == 0) ? "OK" : "Range standard deviation is 0 !\n");
                }


            }
            TiXmlElement *paramSet = resultList[i].getPointerToXmlNode();
            paramSet->SetAttribute("errorCode", resultList[i].getErrorCode());
            paramSet->SetAttribute("errorMsg", resultList[i].getErrorMsg().c_str());
					}

				}

        printf("Killing process %d\n", pidId);
        sudokill(pidId);

        std::string testCtrlResultXmlFileName = "";
        size_t pos = testCtrlXmlFileName.rfind('.');
        if (pos != std::string::npos)
        {
          boost::filesystem::path tmpFilePath =testCtrlXmlFileName.substr(0,pos);
          boost::filesystem::path xmlDir= tmpFilePath.parent_path();
					std::string xmlDirName = xmlDir.string();
					xmlDirName.append("/results");
          if (boost::filesystem::exists(xmlDirName))
          {

          }
          else
          {
            boost::filesystem::create_directory(xmlDirName);
          }
          std::string tmpFilNamewoExtension= tmpFilePath.stem().string();
          std::string resultFileName=xmlDir.string();
          resultFileName.append("/");
          resultFileName.append(tmpFilNamewoExtension);
          resultFileName.append(".res");
          printf("Save to %s\n", resultFileName.c_str());
          doc.SaveFile(resultFileName.c_str());
        }

			}
		}
		else
		{
			ROS_WARN("Cannot find filename entry");
		}


	}
	return result;

}

