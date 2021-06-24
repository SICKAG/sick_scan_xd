/**
* \file
* \brief PointCloud to depth image converter as well as Pointcloud to csv
* Copyright (C) 2019, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2019, SICK AG, Waldkirch
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
*  Last modified: 10th Jan 2019
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*
*/

#include <ros/ros.h>
// PCL specific includes
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>

using namespace ros;

#include "pcl_converter/gnuplotPaletteReader.h"

ros::Publisher pub;

GnuPlotPalette pal;
bool usingPal = false;

bool ignoreIntensityMissing = true;

double minRange = 255.99 * 0.25;
double maxRange = 0.00;
float minRangeGreyVal = 0.0;
float maxRangeGreyVal = 255.99;

int numEchoOutput = 1;

class ParamSettings
{
private:
  bool dump2csv;
public:
  bool isDump2csv() const
  {
    return dump2csv;
  }

  void setDump2csv(bool dump2csv)
  {
    ParamSettings::dump2csv = dump2csv;
  }

};

ParamSettings paramSettings;


void dump2csv(const sensor_msgs::PointCloud2ConstPtr &cloud, int x_offset, int y_offset, int z_offset)
{
  static int cnt = 0;
  char szFileName[255] = {0};
  sprintf(szFileName,"/tmp/cloud_dump_%04d.csv", cnt);
  FILE *fout;
  fout = fopen(szFileName,"w");
  if (fout != NULL)
  {
    int i,j;
    fprintf(fout,"%-8s;%-8s;%-8s;%-8s;%-8s;%-8s\n","X(col)","Y(Col)","X","Y","Z","Range");
    for (i = 0; i < cloud->height; i++)
    {
      for (j = 0; j < cloud->width; j++)
      {
        float xRange,yRange,zRange;
        float range;
        memcpy(&xRange, &(cloud->data[x_offset]), sizeof(float));
        memcpy(&yRange, &(cloud->data[y_offset]), sizeof(float));
        memcpy(&zRange, &(cloud->data[z_offset]), sizeof(float));
        range = sqrt(xRange*xRange + yRange*yRange + zRange*zRange);
        fprintf(fout,"%5d;%5d;", i, j);
        fprintf(fout,"%8.3f;%8.3f;%8.3f;%8.3f", xRange, yRange, zRange, range);
        fprintf(fout,"\n");

        x_offset += cloud->point_step;
        y_offset += cloud->point_step;
        z_offset += cloud->point_step;
      }
    }
  }
  fclose(fout);
  cnt++;
}





/** \brief Copy the RGB fields of a PCLPointCloud2 msg into pcl::PCLImage format
  * \param cloud the point cloud message
  * \param msg the resultant pcl::PCLImage
  * will throw std::runtime_error if there is a problem
  * http://wiki.ros.org/perception_pcl
  * pcl_conversions.h
  */
void
cloud2image_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  sensor_msgs::Image msg;
  sensor_msgs::Image msgVGA;
  int rgb_index = -1;
  int x_index = -1;
  int y_index = -1;
  int z_index = -1;

  int conversionTask = 3;
  float range_eps = 0.1;


  int heightUsed = cloud->height;
  if (numEchoOutput == 1)
  {
    if (heightUsed > 24)
    {
      heightUsed = 24;
    }
  }
  // Get the index we need
  for (size_t d = 0; d < cloud->fields.size(); ++d)
  {
    if (cloud->fields[d].name == "intensity")
    {
      rgb_index = static_cast<int>(d); // relative index for each field entry

    }
    if (cloud->fields[d].name == "x")
    {
      x_index = static_cast<int>(d);

    }
    if (cloud->fields[d].name == "y")
    {
      y_index = static_cast<int>(d);

    }
    if (cloud->fields[d].name == "z")
    {
      z_index = static_cast<int>(d);

    }
  }
  if (rgb_index == -1)
  {
    if (ignoreIntensityMissing)
    {
      return;
      // intensity flag is missing - ignore the scan
    }
    else
    {
      throw std::runtime_error("No intensity entry!!");
    }
  }



  int heightStretch = 5;

  int imgChannelCnt = 0;

  const int maxAllowedEncodings = 2;
  bool encodeFlagArray[maxAllowedEncodings] = {0};
  bool useHDFormat = false;
  if (conversionTask & 1)
  {
    encodeFlagArray[0] = true;
    imgChannelCnt++;
  }
  if (conversionTask & 2)
  {
    encodeFlagArray[1] = true;
    imgChannelCnt++;
  }

  if (cloud->width == 0 && heightUsed == 0)
  {
    throw std::runtime_error("Needs to be a dense like cloud!!");
  }
  else
  {
    msg.height = heightStretch * heightUsed * imgChannelCnt;
    msg.width = cloud->width;
    if (useHDFormat)
    {
      if (1080 > msg.height)
      {
        msg.height = 1080;
      }
      if (1920 > msg.width)
      {
        msg.width = 1920;
      }
    }
  }
  int rgb_offset = cloud->fields[rgb_index].offset;
  int x_offset = cloud->fields[x_index].offset;
  int y_offset = cloud->fields[y_index].offset;
  int z_offset = cloud->fields[z_index].offset;



  int point_step = cloud->point_step;

  /*
   * point_step: size of each element
   * x_offset etc: relative index
   *
   */
  if (paramSettings.isDump2csv())
  {
    dump2csv(cloud, x_offset, y_offset, z_offset);
  }
  // pcl::image_encodings::BGR8;
  msg.encoding = "bgr8";
  msg.step = static_cast<uint32_t>(msg.width * sizeof(uint8_t) * 3);
  msg.data.resize(msg.step * msg.height);


  int maxY = imgChannelCnt * heightStretch * heightUsed;

  for (size_t y = 0; y < heightUsed; y++)
  {
    for (size_t x = 0; x < cloud->width; x++)
    {
      for (int chIdx = 0; chIdx < imgChannelCnt; chIdx++)
      {
        int encodingCnt = 0;
        int encodeIdx = -1; // Kanal, fuer den encodiert werden soll....
        for (int j = 0; j < maxAllowedEncodings; j++)
        {
          if (encodeFlagArray[j])
          {
            if (chIdx == encodingCnt)
            {
              encodeIdx = j;
              break;
            }
            encodingCnt++;
          }
        }

        for (int rowInner = 0; rowInner < heightStretch; rowInner++)
        {
          int rowTmp = (y * heightStretch + rowInner);
          rowTmp += encodingCnt * heightStretch * heightUsed;

          int rowInnerIdx = maxY - (rowTmp) - 1;
          int xTmp = x;
          xTmp = cloud->width - 1 - x;
          uint8_t *pixel = &(msg.data[rowInnerIdx * msg.step + xTmp * 3]);
          uint8_t *pixelVGA = NULL;

          float intensity;
          float xRange;
          float yRange;
          float zRange;
          float range = 0.0;
          memcpy(&xRange, &(cloud->data[x_offset]), sizeof(float));
          memcpy(&yRange, &(cloud->data[y_offset]), sizeof(float));
          memcpy(&zRange, &(cloud->data[z_offset]), sizeof(float));
          range = sqrt(xRange * xRange + yRange * yRange + zRange * zRange);
          memcpy(&intensity, &(cloud->data[rgb_offset]), sizeof(float));
          if (intensity > 255.0)
          {
            intensity = 255.0;
          }
          if (intensity < 0.0)
          {
            intensity = 0.0;
          }
          unsigned char r, g, b;
          unsigned char grey = 0;
          if (encodeIdx == 0)
          {
            grey = (unsigned char) intensity;
          }

          float minRealRange = (minRange < maxRange) ? minRange : maxRange;
          float maxRealRange = (minRange < maxRange) ? maxRange : minRange;

          if (encodeIdx == 1)
          {
            bool setBlack = false;
            if (range < range_eps)
            {
              setBlack = true;
            }
            if (range < minRealRange)
            {
              range = minRealRange;

            }
            if (range > maxRealRange)
            {
              range = maxRealRange;
            }
            // calculate from range to grey value
            range = (range - minRange) / (maxRange - minRange) * (maxRangeGreyVal - minRangeGreyVal);
            range += minRangeGreyVal;

            grey = (unsigned char) range;
            if (setBlack)
            {
              grey = 0;
            }
          }

          // grey = 0xFF;
          r = grey;
          g = grey;
          b = grey;


          if (usingPal)
          {
            r = pal.getRbgValue(grey, 0);
            g = pal.getRbgValue(grey, 1);
            b = pal.getRbgValue(grey, 2);
          }
          pixel[2] = r;
          pixel[1] = g;
          pixel[0] = b;

          //memcpy(pixel, &(cloud->data[rgb_offset]), 3 * sizeof (uint8_t));
        }
      }
      rgb_offset += point_step;
      x_offset += point_step;
      y_offset += point_step;
      z_offset += point_step;

    }

  }

// Publish the data.
  pub.publish(msg);
}

void argUsage()
{
  printf("sick_pcl_converter <input_pointcloud2-topic> <output_image-topic>");
}

int
main(int argc, char **argv)
{
  bool dumpCsvFlag = false;
  // Initialize ROS
  if (argc < 3)
  {
    argUsage();
    exit(-1);
  }

  // sensor_msgs::PointCloud2 pclTmp;

  std::string inputTopic = argv[1];
  std::string outputTopic = argv[2];
  ros::init(argc, argv, "sick_pcl_converter");


  ros::NodeHandle nh("sick_pcl_converter");
  std::string heatMap;
  nh.getParam("heat_map", heatMap);

  nh.getParam("dump_csv", dumpCsvFlag);

  paramSettings.setDump2csv(dumpCsvFlag);

  if (paramSettings.isDump2csv())
  {
    ROS_INFO("Dump to csv file for debugging enabled.\n Files will be dumped at /tmp ");
  }

  std::string path = ros::package::getPath("sick_scan");

  if (heatMap.length() > 0)
  {
    std::string heatMapFileName = path + "/config/" + heatMap;
    pal.load(heatMapFileName);
    usingPal = true;
  }
  ROS_INFO("Subscribing to %s and publishing images to %s\n", inputTopic.c_str(), outputTopic.c_str());
  // Create a ROS subscriber for the input point cloud
//  ros::Subscriber sub = nh.subscribe (inputTopic, 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe(inputTopic, 1, cloud2image_cb);


  // Create a ROS publisher for the output point cloud
//  pub = nh.advertise<sensor_msgs::PointCloud2> (outputTopic, 1);
  pub = nh.advertise<sensor_msgs::Image>(outputTopic, 1);

  // Spin
  ros::spin();
}
