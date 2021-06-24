/*
 * Copyright (C) 2018, SICK AG
 * Copyright (C) 2018, Ing.-Buero Dr. Michael Lehning
 * Copyright (C) 2016, DFKI GmbH
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
 *     * Neither the name of DFKI GmbH nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <sick_scan/RadarScan.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include <sick_scan/RadarScan.h>
#include "radar_object_marker/radar_object_marker.h"
#include "pcl_converter/gnuplotPaletteReader.h"
#include <boost/serialization/singleton.hpp>

float ballRadius = 0.1;
float objectArrowScale = 1.0;
GnuPlotPalette pal;
bool usingPal = false;

ros::Publisher pub;
ros::Publisher pub_cloud;




void normalize( float& angle )
{
    while ( angle < -M_PI ) angle += (2 * M_PI);
    while ( angle >  M_PI ) angle -= (2 * M_PI);
}

bool isWithinRange( float testAngle, float a, float b )
{
    a -= testAngle;
    b -= testAngle;
    normalize( a );
    normalize( b );
    if ( a * b >= 0 )
        return false; // lying on the same side
    return fabs( a - b ) < M_PI;
}



/* check for both directions */
bool checkForAngleInterval(float orgAngle, float* dstAngle, float mainAngle, float mainAngleTol)
{
   float tmpAngle;
   bool bRet = isWithinRange(orgAngle, mainAngle - mainAngleTol, mainAngle + mainAngleTol);
   if (bRet == true)
   {
       tmpAngle = mainAngle;
       normalize(tmpAngle);
       *dstAngle = tmpAngle;
       return(bRet);
   }
   bRet = isWithinRange(orgAngle, M_PI + mainAngle - mainAngleTol, M_PI + mainAngle + mainAngleTol);
   if (bRet == true)
   {
       tmpAngle = mainAngle + M_PI;
       normalize(tmpAngle);
       *dstAngle = tmpAngle;
       return(bRet);

   }



   return(bRet);



}

bool checkForAngleIntervalTestbed()
{
    bool bRet = false;
    double testAngleArrDeg[] = {-161.0, -10.0, -20.0, -30.0, 330.0, 10.0, 20.0, 30.0, -160.0};

    std::vector<double> testAngleArr;

    for ( int i =0; i  < sizeof(testAngleArrDeg)/sizeof(testAngleArrDeg[0]); i++)
    {
        double tmpAngle = testAngleArrDeg[i] / 180.0 * M_PI;
        testAngleArr.push_back(tmpAngle);
    }


    double mainAngle = 20.0/180.0 * M_PI;
    double mainAngleTol = 30.0/180.0 * M_PI;
    for (int i  = 0; i < testAngleArr.size(); i++)
    {
        float angle = testAngleArr[i];
        float dstAngle;
        bRet = checkForAngleInterval(angle, &dstAngle, mainAngle, mainAngleTol);

        printf("Ergebnis: %2d %6.2lf %6.2lf %s\n", i, angle, dstAngle, bRet ? "JA" : "NEIN");
    }
    return bRet;
}



void callback(const sick_scan::RadarScan::ConstPtr &oa)
  {
  RadarObjectMarkerCfg *cfgPtr = &boost::serialization::singleton<RadarObjectMarkerCfg>::get_mutable_instance();

  enum TARGET_MARKER_TYPE {TARGET_MARKER_BALL, TARGET_MARKER_ARROW, TARGET_MARKER_NUM};
  visualization_msgs::MarkerArray rawTargetArray[TARGET_MARKER_NUM];  // ball and arrow
  sensor_msgs::PointCloud2 cloud_msg;

  for (int i = 0; i < TARGET_MARKER_NUM; i++)
  {
    rawTargetArray[i].markers.resize(oa->targets.width);
  }

  int coordIdx[3] = {0};
  int vradIdx = -1;
  int amplitudeIdx = -1;

  for (int i = 0; i < 3; i++)
  {
    coordIdx[i] = -1;
  }


  bool useCurrTimeStamp = true;
  ros::Time  currTimeStamp = ros::Time::now();  // timestamp incoming package, will be overwritten by get_datagram
  for (int i = 0; i < oa->targets.fields.size(); i++)
  {
    std::string fieldName = oa->targets.fields[i].name;
    // x,y,z, vrad, amplitude
    if (fieldName.compare("x") == 0)
    {
      coordIdx[0] = i;
    }
    if (fieldName.compare("y") == 0)
    {
      coordIdx[1] = i;
    }
    if (fieldName.compare("z") == 0)
    {
      coordIdx[2] = i;
    }
    if (fieldName.compare("vrad") == 0)
    {
      vradIdx = i;
    }
    if (fieldName.compare("amplitude") == 0)
    {
      amplitudeIdx = i;
    }

    // printf("%s\n", fieldName.c_str());
  }

  int numRawTargets = oa->targets.width;

  if (numRawTargets > 0)
  {
    cloud_msg.header = oa->header;

    if (useCurrTimeStamp)
    {
      cloud_msg.header.stamp = currTimeStamp;
    }

    cloud_msg.height = oa->targets.height;
    cloud_msg.width  = oa->targets.width;
    cloud_msg.row_step = oa->targets.row_step;
    cloud_msg.point_step = oa->targets.point_step;
    cloud_msg.data = oa->targets.data;
    cloud_msg.fields  = oa->targets.fields;
    cloud_msg.is_dense = oa->targets.is_dense;
    cloud_msg.is_bigendian = oa->targets.is_bigendian;
    pub_cloud.publish(cloud_msg);
  }
  for (size_t i = 0; i < oa->targets.width; i++)
  {
    int tmpId = i;
    float ampl = 0.0;
    float xs, ys;
    float pts3d[3];
    float vrad = 0.0;
    for (int ii = 0; ii < 3; ii++)
    {
      if (coordIdx[ii] != -1)
      {
        int tmpIdx = coordIdx[ii];
        float *valPtr = (float *) (&(oa->targets.data[0]) + i * oa->targets.point_step +
                                   oa->targets.fields[tmpIdx].offset);
        pts3d[ii] = *valPtr;
      }
    }
    if (vradIdx != -1)
    {
      int tmpIdx = vradIdx;
      float *valPtr = (float *) (&(oa->targets.data[0]) + i * oa->targets.point_step +
                                 oa->targets.fields[tmpIdx].offset);
      vrad = *valPtr;
    }
    if (amplitudeIdx)
    {
      int tmpIdx = amplitudeIdx;
      float *valPtr = (float *) (&(oa->targets.data[0]) + i * oa->targets.point_step +
                                 oa->targets.fields[tmpIdx].offset);
      ampl = *valPtr;

    }

    for (int iLoop = 0; iLoop < TARGET_MARKER_NUM; iLoop++)
    {

      rawTargetArray[iLoop].markers[i].header = oa->header;

      if (useCurrTimeStamp)
      {
        rawTargetArray[iLoop].markers[i].header.stamp = currTimeStamp;
      }

      rawTargetArray[iLoop].markers[i].ns = "rawtargets";
      rawTargetArray[iLoop].markers[i].id = tmpId + iLoop * numRawTargets;
      switch (iLoop)
      {
        case TARGET_MARKER_ARROW:
        {
          rawTargetArray[iLoop].markers[i].type = visualization_msgs::Marker::ARROW;
          rawTargetArray[iLoop].markers[i].scale.x = 0.1;
          rawTargetArray[iLoop].markers[i].scale.y = 0.2;

          rawTargetArray[iLoop].markers[i].points.resize(2);
          rawTargetArray[iLoop].markers[i].points[0].x = pts3d[0];
          rawTargetArray[iLoop].markers[i].points[0].y = pts3d[1];
          rawTargetArray[iLoop].markers[i].points[0].z = pts3d[2];
          float lineOfSight = atan2(pts3d[1], pts3d[0]);
          rawTargetArray[iLoop].markers[i].points[1].x = pts3d[0] + cos(lineOfSight) * vrad * 0.1;
          rawTargetArray[iLoop].markers[i].points[1].y = pts3d[1] + sin(lineOfSight) * vrad * 0.1;
          rawTargetArray[iLoop].markers[i].points[1].z = pts3d[2];

        }
          break;
        case TARGET_MARKER_BALL:
          rawTargetArray[iLoop].markers[i].type = visualization_msgs::Marker::SPHERE;
          rawTargetArray[iLoop].markers[i].scale.x = ballRadius;
          rawTargetArray[iLoop].markers[i].scale.y = ballRadius;
          rawTargetArray[iLoop].markers[i].scale.z = ballRadius;


          rawTargetArray[iLoop].markers[i].pose.position.x = pts3d[0];
          rawTargetArray[iLoop].markers[i].pose.position.y = pts3d[1];
          rawTargetArray[iLoop].markers[i].pose.position.z = pts3d[2];
          rawTargetArray[iLoop].markers[i].pose.orientation.x = 0.0;
          rawTargetArray[iLoop].markers[i].pose.orientation.y = 0.0;
          rawTargetArray[iLoop].markers[i].pose.orientation.z = 0.0;
          rawTargetArray[iLoop].markers[i].pose.orientation.w = 1.0;
          break;

      }
      rawTargetArray[iLoop].markers[i].action = visualization_msgs::Marker::ADD;
      if (cfgPtr->isPaletteUsed())
      {
          GnuPlotPalette pal = cfgPtr->getGnuPlotPalette();
         float idxRange = ampl - cfgPtr->getPaletteMinAmpl();
         double rangeAmpl = cfgPtr->getPaletteMaxAmpl() - cfgPtr->getPaletteMinAmpl();
         idxRange /= rangeAmpl;
         if (idxRange < 0.0)
         {
           idxRange = 0.0;
         }
         if (idxRange > 1.0)
         {
          idxRange = 1.0;
         }
         unsigned char greyIdx = (unsigned char)(255.999 * idxRange);
         unsigned char red = pal.getRbgValue(greyIdx, 0U);
         unsigned char green = pal.getRbgValue(greyIdx, 1U);
         unsigned char blue = pal.getRbgValue(greyIdx, 2U);

         float redF, greenF, blueF;
        redF = red / 255.00;
        greenF = green / 255.00;
        blueF = blue / 255.00;
        rawTargetArray[iLoop].markers[i].color.a = 0.75;
        rawTargetArray[iLoop].markers[i].color.r = redF;
        rawTargetArray[iLoop].markers[i].color.g = greenF;
        rawTargetArray[iLoop].markers[i].color.b = blueF;
      }
      else{
      rawTargetArray[iLoop].markers[i].color.a = 0.75;
      rawTargetArray[iLoop].markers[i].color.r = GLASBEY_LUT[tmpId * 3] / 255.0;
      rawTargetArray[iLoop].markers[i].color.g = GLASBEY_LUT[tmpId * 3 + 1] / 255.0;
      rawTargetArray[iLoop].markers[i].color.b = GLASBEY_LUT[tmpId * 3 + 2] / 255.0;
      }
      rawTargetArray[iLoop].markers[i].lifetime = ros::Duration(0.5);


    }
  }

  for (int iLoop = 0; iLoop < TARGET_MARKER_NUM; iLoop++)
  {
    pub.publish(rawTargetArray[iLoop]);
  }

  /***********************************************************************
   *
   *
   * Drawing marker for objects
   *
   ***********************************************************************
   */
  visualization_msgs::MarkerArray object_boxes;
  object_boxes.markers.resize(2 * oa->objects.size());

  visualization_msgs::MarkerArray object_labels;
  object_labels.markers.resize(1 * oa->objects.size());

  float vp_dir = 0.0;
  float vp_dir_deg = 0.0;

  double angleMainDirDeg = 20;
  double angleMainDir = angleMainDirDeg / 180.0 * M_PI;
  double angleMainDirTolDeg = 20.0;
  double angleMainDirTol = angleMainDirTolDeg / 180.0 * M_PI;

  std::vector<float> vehiclePredefinedDir;
  std::vector<float> vehicleAbsoluteSpeed;
  std::vector<bool>  vehicleIsInPredefinedDir;

  int objNum = oa->objects.size();

  vehiclePredefinedDir.resize(objNum);
  vehicleIsInPredefinedDir.resize(objNum);  // flag arraw whether the vehicle drives in tihs dir. or not
  vehicleAbsoluteSpeed.resize(objNum);

  for (int iLoop = 0; iLoop < 2; iLoop++)
  {
    for (size_t i = 0; i < oa->objects.size(); i++)
    {
        int colorId = oa->objects[i].id % 256;
        float vp[3];
        vp[0] = oa->objects[i].velocity.twist.linear.x;
        vp[1] = oa->objects[i].velocity.twist.linear.y;
        vp[2] = oa->objects[i].velocity.twist.linear.z;
        float vabs = sqrt(vp[0] * vp[0] + vp[1]*vp[1] + vp[2]*vp[2]);
        vehicleAbsoluteSpeed[i] = vabs;
        vp_dir = atan2(vp[1], vp[0]);
        vp_dir_deg = vp_dir / 3.141592 * 180.0;
        float dstAngle;

        bool bRet = checkForAngleInterval(vp_dir, &dstAngle, angleMainDir, angleMainDirTol);
        vehiclePredefinedDir[i] = dstAngle;
        vehicleIsInPredefinedDir[i] = bRet;

      int idx = i + iLoop * oa->objects.size();
      int tmpId = i; // better: oa->objects[i].id;
      object_boxes.markers[idx].header = oa->header;

      if (useCurrTimeStamp)
      {
        object_boxes.markers[idx].header.stamp = currTimeStamp;
      }
      std::string nameSpaceBoxes = "object_boxes_slow";
      if (vabs > 5.0)
      {
          nameSpaceBoxes = "object_boxes_fast";
      }
      object_boxes.markers[idx].ns = nameSpaceBoxes;
      object_boxes.markers[idx].id = tmpId + iLoop * oa->objects.size(); // i; // oa->objects[i].id;
      object_boxes.markers[idx].action = visualization_msgs::Marker::ADD;
      object_boxes.markers[idx].color.a = 0.75;
      object_boxes.markers[idx].color.r = GLASBEY_LUT[colorId * 3] / 255.0;
      object_boxes.markers[idx].color.g = GLASBEY_LUT[colorId * 3 + 1] / 255.0;
      object_boxes.markers[idx].color.b = GLASBEY_LUT[colorId * 3 + 2] / 255.0;
      object_boxes.markers[idx].lifetime = ros::Duration(1.0);

      if (iLoop == 0)
      {
          object_labels.markers[i].header = oa->header;
          if (useCurrTimeStamp)
          {
              if (iLoop == 0) {
                  object_labels.markers[i].header.stamp = currTimeStamp;
              }
          }

          object_labels.markers[i].id = tmpId + iLoop * oa->objects.size(); // i; // oa->objects[i].id;
          object_labels.markers[i].action = visualization_msgs::Marker::ADD;
          object_labels.markers[i].color.a = 0.75;
          object_labels.markers[i].color.r = GLASBEY_LUT[colorId * 3] / 255.0;
          object_labels.markers[i].color.g = GLASBEY_LUT[colorId * 3 + 1] / 255.0;
          object_labels.markers[i].color.b = GLASBEY_LUT[colorId * 3 + 2] / 255.0;
          object_labels.markers[i].lifetime = ros::Duration(1.0);

          object_labels.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          object_labels.markers[i].pose = oa->objects[i].object_box_center.pose;
          object_labels.markers[i].scale.z = 1.0;

          {

              char szLabel[255];
              sprintf(szLabel,"%5.1lf [m/s]", vabs );

              if (vabs > 10.0)
              {
              // printf("Dir: %8.3lf [deg]\n", vp_dir_deg);
              }
              std::string object_label_ns = "???";
              if (vabs > 5.0) // 18 km/h
              {
                  object_label_ns = "object_label_fast";
              }
              else
              {
                  object_label_ns = "object_label_slow";

              }
              object_labels.markers[i].text = szLabel;
              object_labels.markers[i].ns = object_label_ns;

              if (vehicleIsInPredefinedDir[i] == false)
              {
                  object_labels.markers[i].scale.z = 0.00;
              }
          }


      }

      /* object box center is the reference ...
       * */
      object_boxes.markers[idx].pose = oa->objects[i].object_box_center.pose;  // Position and Orientation
      object_boxes.markers[idx].scale = oa->objects[i].object_box_size;

      switch (iLoop)
      {
        case 0:  // Drawing object cubes
          object_boxes.markers[idx].type = visualization_msgs::Marker::CUBE;
          object_boxes.markers[idx].pose.position.z  +=  oa->objects[i].object_box_size.z * 0.5;


          break;
        case 1:
          object_boxes.markers[idx].type = visualization_msgs::Marker::ARROW;

          break;
      }

      switch (iLoop)
      {
        case 0:
          if (object_boxes.markers[idx].scale.x == 0.0)
          {
            object_boxes.markers[idx].scale.x = 0.01;
          }
          if (object_boxes.markers[idx].scale.y == 0.0)
          {
            object_boxes.markers[idx].scale.y = 0.01;
          }
          if (object_boxes.markers[idx].scale.z == 0.0)
          {
            object_boxes.markers[idx].scale.z = 0.01;
          }

          if (vehicleIsInPredefinedDir[i] == false) {
              object_boxes.markers[idx].scale.x = 0.0;
              object_boxes.markers[idx].scale.y = 0.0;
              object_boxes.markers[idx].scale.z = 0.0;
              object_boxes.markers[idx].lifetime = ros::Duration(0.001);
          } else{


              if (vehicleIsInPredefinedDir[i] == true)
              {
                  float theta = vehiclePredefinedDir[i];


                  // Falls wir eine Zwangsrichtung einpraegen wollen, dann muessen wir es am besten hier tun.

              // (n_x, n_y, n_z) = (0, 0, 1), so (x, y, z, w) = (0, 0, sin(theta/2), cos(theta/2)).
              // see https://answers.ros.org/question/9772/quaternions-orientation-representation/
                  float theta2 = 0.5  * theta;
                  object_boxes.markers[idx].pose.orientation.z = sin(theta2);
                  object_boxes.markers[idx].pose.orientation.w = cos(theta2);

              }


          }
          break;
        case 1:
        {
          float xTmp;
          float yTmp;

          object_boxes.markers[idx].scale.x = 0.5;
          object_boxes.markers[idx].scale.y = 0.5;
          object_boxes.markers[idx].scale.z = 0.5;



/*          float cosVal =  oa->objects[i].object_box_center.pose.orientation.x;
          float sinVal =  oa->objects[i].object_box_center.pose.orientation.y;
*/
          xTmp = oa->objects[i].object_box_center.pose.position.x;
          yTmp = oa->objects[i].object_box_center.pose.position.y;

          double vehLen = oa->objects[i].object_box_size.x;


          float quaternion[4] = {0};
          // rotation around z-axis: [cos(theta/2.0), 0, 0, sin(theta/2.0) ]
          quaternion[0] = oa->objects[i].object_box_center.pose.orientation.x;
          quaternion[1] = oa->objects[i].object_box_center.pose.orientation.y;
          quaternion[2] = oa->objects[i].object_box_center.pose.orientation.z;
          quaternion[3] = oa->objects[i].object_box_center.pose.orientation.w;


          // Falls wir eine Zwangsrichtung einpraegen wollen, dann muessen wir es am besten hier tun.

          // (n_x, n_y, n_z) = (0, 0, 1), so (x, y, z, w) = (0, 0, sin(theta/2), cos(theta/2)).
          // see https://answers.ros.org/question/9772/quaternions-orientation-representation/
          float theta2 = atan2(quaternion[2], quaternion[3]);
          float theta = 2.0f * theta2;
          if (vehicleIsInPredefinedDir[i] == true)
          {
                theta = vehiclePredefinedDir[i];
          }

          float cosVal = cos(theta);
          float sinVal = sin(theta);
          xTmp += cosVal * vehLen * 0.5;
          yTmp += sinVal * vehLen * 0.5;
          object_boxes.markers[idx].pose.position.x = xTmp;
          object_boxes.markers[idx].pose.position.y = yTmp;
          object_boxes.markers[idx].pose.position.z = 0.0;

          // oa->objects[i].object_box_center.pose.orientation;
          // Arrow orientation is already rotated
          object_boxes.markers[idx].pose.orientation.x = 0.0;
          object_boxes.markers[idx].pose.orientation.y = 0.0;
          object_boxes.markers[idx].pose.orientation.z = 0.0;
          object_boxes.markers[idx].pose.orientation.w = 0.0;

          object_boxes.markers[idx].points.resize(2);
          object_boxes.markers[idx].points[0].x = 0.0;
          object_boxes.markers[idx].points[0].y = 0.0;
          object_boxes.markers[idx].points[0].z = 0.0;

          float absSpeed = 0.0;
          for (int j = 0; j < 3; j++)
          {
            float speedPartial = 0.0;
            switch(j)
            {
              case 0: speedPartial = oa->objects[i].velocity.twist.linear.x; break;
              case 1: speedPartial = oa->objects[i].velocity.twist.linear.y; break;
              case 2: speedPartial = oa->objects[i].velocity.twist.linear.z; break;
            }
            absSpeed += speedPartial * speedPartial;
          }
          absSpeed = sqrt(absSpeed);
          double lengthOfArrow = objectArrowScale * absSpeed;
          if (objectArrowScale < 0.0)
          {
            lengthOfArrow = -objectArrowScale;  // if scale is negative take its absolute value as length of arrow in [m]
          }
          // lengthOfArrow = 5.0;

          if (vehicleIsInPredefinedDir[i] == false) {
              object_boxes.markers[idx].lifetime = ros::Duration(0.001);
              lengthOfArrow = 0.0;
          }
          object_boxes.markers[idx].points[1].x = 0.0 + lengthOfArrow * cosVal;
          object_boxes.markers[idx].points[1].y = 0.0 + lengthOfArrow * sinVal;
          object_boxes.markers[idx].points[1].z = 0.0;
          // pub.publish(object_boxes);

        }
          break;
      }

      // printf("Idx: %2d X: %8.3lf Y: %8.3lf\n", idx, object_boxes.markers[idx].pose.position.x, object_boxes.markers[idx].pose.position.y);
    }
  }
  pub.publish(object_boxes);
  pub.publish(object_labels);

  }


  int main(int argc, char **argv)
  {
    // checkForAngleIntervalTestbed();
    ROS_INFO("radar_object_marker, compiled at [%s] [%s]", __DATE__, __TIME__);
    RadarObjectMarkerCfg *cfgPtr = &boost::serialization::singleton<RadarObjectMarkerCfg>::get_mutable_instance();
    ros::init(argc, argv, "radar_object_marker");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    std::string heatMap;
    double minAmpl = 10.0;
    double maxAmpl = 90.0;
    bool paramRes = nhPriv.getParam("rawtarget_sphere_radius", ballRadius);
    paramRes = nhPriv.getParam("rawtarget_palette_name", heatMap);
    paramRes = nhPriv.getParam("rawtarget_palette_min_ampl", minAmpl);
    paramRes = nhPriv.getParam("rawtarget_palette_max_ampl", maxAmpl);

    paramRes = nhPriv.getParam("object_arrow_scale", objectArrowScale);

    cfgPtr->setPaletteName(heatMap);
    cfgPtr->setPaletteMinAmpl(minAmpl);
    cfgPtr->setPaletteMaxAmpl(maxAmpl);



    std::string path = ros::package::getPath("sick_scan");

    if (cfgPtr->getPaletteName().length() > 0)
    {
      std::string heatMapFileName = path + "/config/" + cfgPtr->getPaletteName();
      pal.load(heatMapFileName);
      usingPal = true;
      cfgPtr->setGnuPlotPalette(pal);
      cfgPtr->setPaletteUsed(true);
    }
    else
    {
      cfgPtr->setPaletteUsed(false);
    }

    ROS_INFO("Subscribing to radar and pulishing to radar_markers");

    ros::Subscriber sub = nh.subscribe("radar", 1, callback);
    pub = nh.advertise<visualization_msgs::MarkerArray>("radar_markers", 1);

    pub_cloud =  nh.advertise<sensor_msgs::PointCloud2>("radar_cloud", 1);

    ros::spin();

    return 0;
    }