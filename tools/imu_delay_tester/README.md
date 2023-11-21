# IMU Delay Test

## Introduction

This test program plots the lidar measurement data of a spring-mass system over time. 
The two time series include the Z components of the IMU unit and the corresponding 
lidar measurements against an object whose width changes over height.

## Installation

run `pip install .` to install the software.

## Usage

The call is made according to the following logic:

```
python3 sickag/imu_delay_tester.py <CSV-Datafile>
```
