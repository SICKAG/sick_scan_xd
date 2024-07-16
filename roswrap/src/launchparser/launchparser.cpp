//
// Created by rosuser on 11.07.19.
//

#include "launchparser.h"
//#include "launchparser/launchparser.h"
#include <sick_scan/sick_ros_wrapper.h>

#include "tinystr.h"
#include "tinyxml.h"

class TINYXML_EXPORT_ATTR paramEntryAscii
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


#ifndef _MSC_VER
#include <signal.h>
//#include <unistd.h>
void sudokill(pid_t tokill)
{
  kill(tokill, SIGTERM);
  rosSleep(5); // sleep(5);
}
#endif

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
    const char *entryVal = paramEntry->Value();
    bool searchAttributes = true;
    if (strcmp(entryVal,"param") == 0)
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



bool LaunchParser::parseFile(std::string launchFileFullName, std::vector<std::string>& nameVec,
    std::vector<std::string>& typeVec, std::vector<std::string>& valVec)
{
  bool ret = false;
  ROS_INFO_STREAM("Try loading launchfile : " << launchFileFullName);
  TiXmlDocument doc;
  doc.LoadFile(launchFileFullName.c_str());

  if (doc.Error() == true)
  {
    ROS_ERROR_STREAM("## ERROR parsing launch file " << doc.ErrorDesc() << "\nRow: " << doc.ErrorRow() << "\nCol: " << doc.ErrorCol() << "");
    return(ret);
  }
  TiXmlNode *node = doc.FirstChild("launch");
  if (node != NULL)
  {
    std::map<std::string, std::string> default_args;
    TiXmlElement *arg_node = (TiXmlElement *)node->FirstChild("arg");
    while(arg_node)
    {
      if(strcmp(arg_node->Value(), "arg") == 0 && arg_node->Type() == TiXmlNode::TINYXML_ELEMENT)
      {
        // parse default arguments, f.e. <arg name="hostname" default="192.168.0.1"/>
        const char* p_attr_name = arg_node->Attribute("name");
        const char* p_attr_default = arg_node->Attribute("default");
        if(p_attr_name && p_attr_default)
        {
          std::string attr_name(p_attr_name), attr_default(p_attr_default);
          default_args[attr_name] = attr_default;
          ROS_INFO_STREAM("LaunchParser::parseFile(" << launchFileFullName << "): default_args[\"" << attr_name << "\"]=\"" << default_args[attr_name] << "\"");
        }
      }
      arg_node = (TiXmlElement *)arg_node->NextSibling();  // go to next sibling
    }

    // Parse optional group for param "scanner_type"
    bool node_with_scanner_type_found = false;
    TiXmlNode * group_node = node->FirstChild("group");
    if (group_node)
    {
      group_node = group_node->FirstChild("node");
      if (group_node)
      {
        std::vector<paramEntryAscii> paramOrgList = getParamList(group_node);
        for (size_t j = 0; j < paramOrgList.size(); j++)
        {
          if (paramOrgList[j].getName() == "scanner_type")
          {
            node_with_scanner_type_found = true;
            ROS_INFO_STREAM("LaunchParser::parseFile(" << launchFileFullName << "): group node found with param scanner_type");
            node = group_node;
            break;
          }
        }
      }
    }
    if (!node_with_scanner_type_found)
    {
      node = node->FirstChild("node");
    }

    // parse all node specific parameters
    std::vector<paramEntryAscii> paramOrgList = getParamList(node);

    for (size_t j = 0; j < paramOrgList.size(); j++)
    {
      nameVec.push_back(paramOrgList[j].getName());
      typeVec.push_back(paramOrgList[j].getType());
      valVec.push_back(paramOrgList[j].getValue());
      if(valVec.back().substr(0, 6) == "$(arg ") // overwrite with default argument, f.e. name="hostname", type="string", value="$(arg hostname)"
      {
        std::string default_arg_name = valVec.back().substr(6, valVec.back().length() - 1 - 6);
        if (default_args.find(default_arg_name) != default_args.end())
        {
          std::string default_arg_val = default_args[default_arg_name];
          ROS_INFO_STREAM("LaunchParser::parseFile(" << launchFileFullName << "): name=\"" << nameVec.back() << "\", type=\""  << typeVec.back() << "\", value=\""  << valVec.back()
            << "\" overwritten by default value \""  << default_arg_val << "\"");
          valVec.back() = default_arg_val;
        }
      }
      ROS_INFO_STREAM("LaunchParser::parseFile(" << launchFileFullName << "): name=\"" << nameVec.back() << "\", type=\""  << typeVec.back() << "\", value=\""  << valVec.back() << "\"");
    }

    ret = true;

  }

  return(ret);
};