//
// Created by rosuser on 11.07.19.
//

#include "launchparser/launchparser.h"
#include <ros/ros.h>


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


#ifndef _MSC_VER

void sudokill(pid_t tokill)
{
  kill(tokill, SIGTERM);
  sleep(5);
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



bool LaunchParser::parseFile(std::string launchFileFullName, std::vector<std::string>& nameVec,
    std::vector<std::string>& typeVec, std::vector<std::string>& valVec)
{
  bool ret = false;
  ROS_INFO("Try loading launchfile :%s",launchFileFullName.c_str());
  TiXmlDocument doc;
  doc.LoadFile(launchFileFullName.c_str());

  if (doc.Error() == true)
  {
    ROS_INFO("ERROR parsing launch file %s\nRow: %d\nCol: %d", doc.ErrorDesc(), doc.ErrorRow(), doc.ErrorCol());
    return(ret);
  }
  TiXmlNode *node = doc.FirstChild("launch");
  if (node != NULL)
  {
    node = node->FirstChild("node");
    std::vector<paramEntryAscii> paramOrgList = getParamList(node);

      for (size_t j = 0; j < paramOrgList.size(); j++)
      {
        nameVec.push_back(paramOrgList[j].getName());
        typeVec.push_back(paramOrgList[j].getType());
        valVec.push_back(paramOrgList[j].getValue());
      }

      ret = true;

  }

  return(ret);
};