# Radar
## Table of contents

- [Introduction](#introduction)
- [Measuring Principle](#measuring-principle)
- [Raw Targets](#raw-targets)
- [Tracking Objects](#tracking-objects)
- [ROS message for Radar](#ros-message-for-radar)
- [Parameter for Radar Usage](#parameter_for_radar_usage)
- [Radar Datagram](radar_datagram.md)
- [Visualization](#visualization)
- [Launch Files](#launch-files)

## Introduction

This driver supports the radar type RMS3xx. This radar records raw targets and tracking objects. The tracking objects are determined on the basis of the raw targets. Two variants of a tracking method are already installed in the radar, which enables the radar to be put into operation quickly.

## Measuring Principle

The RMS3xx is based on FMCW radar. 
With frequency-modulated continuous wave radar (FMCW radar), the transmission frequency is changed periodically. 
Triangle functions are usually used for distance measurement. 
While the transmission frequency changes as linearly as possible to the target object and back during the propagation time of the signal, 
the signal reflected by the object and received by the radar is time-shifted to the original transmitted frequency. 
By mixing the transmitted signal with the received signal, the frequency shift and thus the time shift can be determined. 
Based on the known modulation parameters of the transmitter, the propagation time of the signal can be determined, which in turn is proportional to the distance of the object. 
For precise distance measurement, therefore, the transmission frequency must be modulated as precisely as possible in linear fashion, 
since any non-linearity impairs the distance accuracy of the radar.

Through this indirect time measurement via the frequency change of the transmitter, even very close targets can be measured with high accuracy and cost-efficiency using the FMCW method, provided that the modulation parameters are selected appropriately. The distance resolution is determined by the bandwidth of the transmitted signal.

## Raw Targets

Raw targets correspond to individual reflectors that are detected by the radar. Each individual reflector carries the following information:
* Range
* Horizontal angle (azimuth)
* Doppler speed
* Reflectivity of the target (aka rcs - radar cross section)

The radar RMS3xx does not resolve elevation angles.  Therefore, the radar assumes the elevation values (z values) with 0.0. The error in distance estimation is usually negligible and is 0.4% (1.0 - cos(5°)) at an elevation angle of 5° compared to horizontal.

## Tracking Objects

Tracking objects are determined from the raw targets via a tracking procedure over the spatial and temporal
distribution of the raw targets. The track method estimates the location, direction and speed of the object based on an initial estimate.  After initialization, new raw targets are assigned to the track if they "fit" to the track.  This process is called "gating". Once these raw targets have been assigned to the track, 
the track is updated and the new estimate is used for further processing.

The distribution of raw targets over the object also determines the object length during the tracking process.

The tracking object therefore has the following properties:
* Distance from radar in Cartesian coordinates
* Direction vector in Cartesian coordinates
* Direction of travel as an angle in the X/Y plane 
* Vehicle speed
* Vehicle length

## ROS message for Radar

After parsing the telegram, the driver sends an ROS message of type RadarScan. RadarScan consists of the following components:
```
Header header
RadarPreHeader radarPreHeader
sensor_msgs/PointCloud2 targets
sick_scan/RadarObject[] objects
```
### RadarPreHeader
The radar preheader contains information that provides general information about the radar. This data record can usually be ignored for object recognition with regard to raw targets and tracking objects.
For details please refer to the message specification of Sick.

### targets

The list with the raw targetss of type sick_scan/targets contains the information about the raw targets.
Each raw target contains the following data fields in a pointcloud2-message (height: 1, width: number of raw targets):
```
 std::string channelRawTargetId[] = { "x", "y", "z", "vrad","amplitude" };
```
This raw target contains cartesian coordinates, which are derived from range and azimuth angle (horizontal angle) in the following way (code snippet):
```
valSingle[0] = rawTargetList[i].Dist() cos(angle);    // x
valSingle[1] = rawTargetList[i].Dist() * sin(angle);  // y
valSingle[2] = 0.0;                                   // z 
valSingle[3] = rawTargetList[i].Vrad();               // vrad  
valSingle[4] = rawTargetList[i].Ampl();               // amplitude
```


### objects

The list with the objects of type sick_scan/RadarObject[] contains the information about the track objects. 

```
int32 id

time tracking_time                          // valid
time last_seen                              // not set

geometry_msgs/TwistWithCovariance velocity  // valid

geometry_msgs/Pose bounding_box_center      // valid 
geometry_msgs/Vector3 bounding_box_size     // valid

geometry_msgs/PoseWithCovariance object_box_center // valid
geometry_msgs/Vector3 object_box_size              // valid

geometry_msgs/Point[] contour_points        // not set   
```

Please note that not all fields are filled in the object messages. The message specification contains valid ones in the areas marked here in the code section.

The corresponding code fills the object list in the following manner:

```
        float heading = atan2( objectList[i].V3Dy(), objectList[i].V3Dx());

        radarMsg_.objects[i].velocity.twist.linear.x = objectList[i].V3Dx();
        radarMsg_.objects[i].velocity.twist.linear.y = objectList[i].V3Dy();
        radarMsg_.objects[i].velocity.twist.linear.z = 0.0;

        radarMsg_.objects[i].bounding_box_center.position.x = objectList[i].P3Dx();
        radarMsg_.objects[i].bounding_box_center.position.y = objectList[i].P3Dy();
        radarMsg_.objects[i].bounding_box_center.position.z = 0.0;
        radarMsg_.objects[i].bounding_box_center.orientation.x = cos(heading);
        radarMsg_.objects[i].bounding_box_center.orientation.y = sin(heading);
        radarMsg_.objects[i].bounding_box_center.orientation.z = 0.0;
        radarMsg_.objects[i].bounding_box_center.orientation.w = 1.0; // homogeneous coordinates


        radarMsg_.objects[i].bounding_box_size.x = objectList[i].ObjLength();
        radarMsg_.objects[i].bounding_box_size.y = 1.7;
        radarMsg_.objects[i].bounding_box_size.z = 1.7;
        for (int ii = 0; ii < 6; ii++)
        {
          int mainDiagOffset = ii * 6 + ii;  // build eye-matrix
          radarMsg_.objects[i].object_box_center.covariance[mainDiagOffset] = 1.0;  // it is a little bit hacky ...
          radarMsg_.objects[i].velocity.covariance[mainDiagOffset] = 1.0;
        }
        radarMsg_.objects[i].object_box_center.pose = radarMsg_.objects[i].bounding_box_center;
        radarMsg_.objects[i].object_box_size= radarMsg_.objects[i].bounding_box_size;

```
As you can see there are default values for object height and object width of 1.7 (typical private vehicle)

## Visualization

For the visualization a ROS node was developed, which receives 
the radar messages and exports them as boxes for the objects and as arrows for the raw data. 
The ROS node **radar_object_marker** receives the radar data and exports marker arrays that can be visualized in rviz.

The visualization could be controlled by the following parameters:

|Parameters | Description |
|-----------| ------- |
| rawtarget_sphere_radius | radius of the sphere for the raw data display |
| rawtarget_arrow_scale | Scaling factor for the arrow at the raw targets |
| rawtarget_palette_name | name of the color palette |
| rawtarget_palette_min_ampl | Minimum amplitude value mapped to color idx 0. |
| rawtarget_palette_max_ampl | Maximum amplitude value mapped to color index 255. |
| object_arrow_scale | Scaling factor for the arrow at the raw targets |

The scaling values can be interpreted as the time an object or a raw target moves during this period. Using the distance/time equation, the distance corresponding to the arrow length is calculated from the product of this time period and the object speed or Doppler speed.

Example: Doppler speed: 20[m/s], rawtarget_arrow_scale: 0.4
An arrow of length 0.4 * 20[m] = 8[m] is displayed in rviz.

## Launch Files

The following launch files serve as examples for use:

* sick_rms_3xx.launch: Communication with the RMS3xx and sending of radar ROS messages after successful parsing of SOPAS telegrams coming directly from the radar.
* radar_object_marker.launch : Conversion of radar messages to visualization messages
* sick_rms_3xx_emul.launch: Additionally an emulation was created, which allows testing the interface chain without a physical radar.

### Data visualization example video
[Can be found here](200326_5_video_track.mp4)

## Parameter for Radar Usage
The following parameters are support by the node **sick_generic_caller** in
combination with the RADAR RMS3xx:

* ~scanner_type (string, default: "")<br>
    Must be set to **sick_rms_3xx**
* ~range_max (double, default: 25.0)<br>
    Maximum range
* ~hostname
* ~port
* ~timelimit
* ~tracking_mode<br>
  0: BASIC-Tracking - use for tracking smaller objects <br>
  1: TRAFFIC-Tracking - use for tracking larger objects like vehicles <br>
* transmit_raw_targets (bool, default: true)
* transmit_objects (bool, default: true)
* emul_sensor (bool, default: false)


