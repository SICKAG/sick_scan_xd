/*
 * @brief Pointcloud utilities for ROS simu
 *
 * Copyright (C) 2021, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2021, SICK AG, Waldkirch
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
 *  Created on: 12.01.2021
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 */

#include <sick_scan/pointcloud_utils.h>

#include "toojpeg.h"

FILE *foutJpg;

void jpegOutputCallback(unsigned char oneByte)
{
  fwrite(&oneByte, 1, 1, foutJpg);
}

/*
** Plots a pointcloud and saves to jpg and csv-file
*/
void plotPointCloud(const ros_sensor_msgs::PointCloud2& cloud_, int intervall, const std::string & outputfolder)
{
                static int cnt = 0;
                cnt++;
                
                ROS_DEBUG_STREAM("plotPointCloud: outputfolder=\"" << outputfolder << "\"");
                const unsigned char *cloudDataPtr = &(cloud_.data[0]);
                int w = cloud_.width;
                int h = cloud_.height;

                int numShots = w * h;

                float *ptr = (float *) cloudDataPtr;

                if (cnt == intervall)
                {
                  std::string filename_tmpl = outputfolder + "/scan";
#if linux				  
                  std::replace( filename_tmpl.begin(), filename_tmpl.end(), '\\', '/');
#else
                  std::replace( filename_tmpl.begin(), filename_tmpl.end(), '/', '\\');
#endif
                  std::string jpgFileName_tmp = filename_tmpl + ".jpg_tmp";

                  int xic = 400;
                  int yic = 400;
                  int w2i = 400;
                  int h2i = 400;
                  int hi = h2i * 2 + 1;
                  int wi = w2i * 2 + 1;
                  int pixNum = hi * wi;
                  int numColorChannel = 3;
                  unsigned char *pixel = (unsigned char *) malloc(numColorChannel * hi * wi);
                  memset(pixel, 0, numColorChannel * pixNum);
                  double scaleFac = 50.0;

                  for (int i = 0; i < hi; i++)
                  {
                    int pixAdr = numColorChannel * (i * wi + xic);
                    pixel[pixAdr] = 0x40;
                    pixel[pixAdr + 1] = 0x40;
                    pixel[pixAdr + 2] = 0x40;
                  }
                  for (int i = 0; i < wi; i++)
                  {
                    int pixAdr = numColorChannel * (yic * wi + i);
                    pixel[pixAdr] = 0x40;
                    pixel[pixAdr + 1] = 0x40;
                    pixel[pixAdr + 2] = 0x40;
                  }

                  scaleFac *= -1.0;
                  for (int i = 0; i < numShots; i++)
                  {
                    double x, y, z, intensity;
                    x = ptr[0];
                    y = ptr[1];
                    z = ptr[2];
                    intensity = ptr[3];
                    ptr += 4;
                    int xi = (x * scaleFac) + xic;
                    int yi = (y * scaleFac) + yic;
                    if ((xi >= 0) && (xi < wi))
                    {
                      if ((yi >= 0) && (xi < hi))
                      {
                        // yi shows left (due to neg. scaleFac)
                        // xi shows up (due to neg. scaleFac)
                        int pixAdr = numColorChannel * (xi * wi + yi);
                        int layer = i / w;
                        unsigned char color[3] = {0x00};
                        switch (layer)
                        {
                          case 0:
                            color[0] = 0xFF;
                            break;
                          case 1:
                            color[1] = 0xFF;
                            break;
                          case 2:
                            color[2] = 0xFF;
                            break;
                          case 3:
                            color[0] = 0xFF;
                            color[1] = 0xFF;
                            break;
                        }

                        for (int kk = 0; kk < 3; kk++)
                        {
                          pixel[pixAdr + kk] = color[kk];

                        }
                      }
                    }

                  }



                  // Write JPEG Scan Top View
                  foutJpg = fopen(jpgFileName_tmp.c_str(), "wb");
                  if (foutJpg == NULL)
                  {
                    ROS_INFO_STREAM("PANIC: Can not open " << jpgFileName_tmp << " for writing. Check existience of demo dir. or patch  code.\n");
                  }
                  else
                  {
                  TooJpeg::writeJpeg(jpegOutputCallback, pixel, wi, hi, true, 99);
                  fclose(foutJpg);

                  free(pixel);
                  std::string jpgFileName_out = filename_tmpl + ".jpg";
#if linux				  
				  rename(jpgFileName_tmp.c_str(), jpgFileName_out.c_str());
#else
				  _unlink(jpgFileName_out.c_str());
				  rename(jpgFileName_tmp.c_str(), jpgFileName_out.c_str());
#endif

                  }
                  // Write CSV-File
                  std::string csvFileNameTmp = filename_tmpl + ".csv_tmp";
                  FILE *foutCsv = fopen(csvFileNameTmp.c_str(), "w");
                  if (foutCsv)
                  {
                    // ZIEL: fprintf(foutCsv,"timestamp;range;elevation;azimuth;x;y;z;intensity\n");
                    fprintf(foutCsv,"timestamp_sec;timestamp_nanosec;range;azimuth_deg;elevation_deg;x;y;z;intensity\n");
                    const unsigned char *cloudDataPtr = &(cloud_.data[0]);

                    int numShots = w * h;

                    float *ptr = (float *) cloudDataPtr;


                    long timestamp_sec = cloud_.header.stamp.sec;
                    long timestamp_nanosec = cloud_.header.stamp.nsec;
                    for (int i = 0; i < numShots; i++)
                    {
                      double x, y, z, intensity;
                      x = ptr[0];
                      y = ptr[1];
                      z = ptr[2];
                      float range_xy = sqrt(x*x+y*y);
                      float range_xyz = sqrt(x*x+y*y+z*z);
                      float elevation = atan2(z, range_xy);
                      float azimuth = atan2(y,x);
                      float elevationDeg = elevation * 180.0 / M_PI;
                      float azimuthDeg = azimuth * 180.0 / M_PI;

                      intensity = ptr[3];
                      ptr += 4;
                      fprintf(foutCsv,"%12ld;%12ld;%8.3lf;%8.3lf;%8.3lf;%8.3f;%8.3f;%8.3f;%8.0f\n", timestamp_sec, timestamp_nanosec, range_xyz, azimuthDeg, elevationDeg, x,y,z,intensity);
                    }
                    fclose(foutCsv);
                    std::string csvFileNameOut = filename_tmpl + ".csv";
#ifdef linux
                     rename(csvFileNameTmp.c_str(), csvFileNameOut.c_str());
#else
					  _unlink(csvFileNameOut.c_str());
					  rename(csvFileNameTmp.c_str(), csvFileNameOut.c_str());
#endif
                  }
                  else
                  {
                        ROS_INFO_STREAM("PANIC: Can not open " << csvFileNameTmp << " for writing. Check existience of demo dir. or patch  code.\n");
                  }
                  cnt = 0;
                }

}