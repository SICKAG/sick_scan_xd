# Radar Datagram
## Table of contents

- [Introduction](#introduction)
- [Structure of Radar Datagram](#structure_of_radar_datagram)
## Introduction

This documentation gives an 
overview of the structure of the radar datagrams. 
With regard to the PreHeader, final consultations will be carried out with SICK, so that slight changes may 
still occur here in Q3/2018.

## Structure of Radar Datagram

composition

The message sick_scan/RadarScan consists of four parts:
- Header in standard format
- radarPreHeader with higher-level information
- targets: Raw targets output from radar
- objects: Tracking objects that are determined based on the raw targets using the internal tracking algorithm.

The complete structure can be determined using the command:
```
rosmsg show sick_scan/RadarScan'
```


## Structure of Radar Preheader

The following is a short datagram showing the structure of the radar datagram. 
The position of the individual elements for the data of the PreHeader is explained below.
For further information you can look up in the attached [PDF](telegram_listing_RMS320_20171101.pdf), 
which has been provided by SICK in a preliminary form and 
may be updated again in the near future.

###### Example of very short radar datagram:

```
sSN LMDradardata 1 1 112F6E9 0 0 BCC DC0C 730E9D16 730EA06D 0 0 0 0 0 0 1 0 0 4 DIST1 42200000 00000000 0 AZMT1 3C23D70A 00000000 0 VRAD1 3C23D70A 00000000 0 AMPL1 3DCCCCCD 00000000 0 1 MODE1 3F800000 00000000 0 0 0 0 0 0
```

In the following, the individual tokens are numbered one after another and their meaning is explained:


```
  0: sSN               
  1: LMDradardata
  
  MeasurementData
  ===============
  2: 1             MeasurementData.uiVersionNo  : Version Information for this while structureValue 
                   Value   Range: 0 ... 65535                                 
  DeviceBlock
  ===========
  3: 1             DeviceBlock.uiIdent      : Logical number of the device
                   Value   Range: 0 ... 65535                                 
  4: 112F6E9       DeviceBlock.udiSerialNo  : Serial number of the device
                   Value Range  : 0..4294967295
                   
    
  5: 0             DeviceBlock.xbState      : State of the device
                   Bit length   : 16
                   
                   0.0 Bool     : Value Range False, True
                   Initialisation: False
                   Meaning       : bDeviceError

                   0.1 Bool      : Value Range False, True
                   Initialisation: False                  
                   Meaning       : bContaminationWarning
                   
                   0.2 Bool      : Value Range False, True
                   Initialisation: False
                   Meaning       : bContaminationError
                                      
                   0.3 ...  0.7
  6: 0             1.0 ...  1.7 Bool      : Value Range False, True
                                           Reserved

StatusBlock
===========                   
  7: BCC            uiTelegramCount  
  8: DC0C           uiCycleCount (or uiScanCount???)
  9: 730E9D16       udiSystemCountScan
 10: 730EA06D       udiSystemCountTransmit
 11: 0              xbInputs (Bit 0.0 .. 0.7)
 12: 0              xbInputs (Bit 1.0 .. 1.7)
 13: 0              xbOutputs (Bit 0.0 .. 0.7)
 14: 0              xbOutputs (Bit 1.0 .. 1.7)
 
MeasurementParam1Block
======================
 15: 0              MeasurementParam1Block.uiCycleDuration
 16: 0              MeasurementParam1Block.uiNoiseLevel

aEncoderBlock
============= 
 17: 1              Number of aEncoderBlocks
 
 
 18: 0              aEncoderBlock[0].udiEncoderPos                     
 19: 0              aEncoderBlock[0].iEncoderSpeed
 
 20: 4              Number of following data channels
 21: DIST1
 22: 42200000
 23: 00000000
 24: 0
 25: AZMT1
 26: 3C23D70A
 27: 00000000
 28: 0
 29: VRAD1
 30: 3C23D70A
 31: 00000000
 32: 0
 33: AMPL1
 34: 3DCCCCCD
 35: 00000000
 36: 0
 37: 1
 38: MODE1
 39: 3F800000
 40: 00000000
 41: 0
 42: 0
 43: 0
 44: 0
 45: 0
 46: 0
```
