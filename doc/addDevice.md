# Adding new Device
## Table of contents

- [Introduction](#introduction)
- [Launch Files](#launch-files)

## Introduction

This driver is designed to support several different scanner types (including radar) from Sick. 
This documentation describes how to add additional devices to the driver.

## Naming Convention

For each device type a name pattern is assigned as follows: 
``
sick_<device family>_<identifier> 
``

The name type is used in the code to decide which scanner-specific parameters are set.
The name type is passed as a parameter as follows:
```
<param name="scanner_type" type="string" value="sick_lms_5xx" />
```

## Launch Files

A launch file is created for each device type, 
which usually has the same naming convention as the scanner type. 
To create a new device, it is recommended to copy, rename and edit an existing launch file.

## Code Modification

1. Hint: Construction of parser:
```
  sick_scan::SickGenericParser *parser = new sick_scan::SickGenericParser(scannerName);
```
2. Add string constant like the constant SICK_SCANNER_RMS_3XX_NAME

3. Append this constant to allowedScannerNames 
   like allowedScannerNames.push_back(SICK_SCANNER_RMS_3XX_NAME);
   in the file sick_generic_parser.cpp
   
4. Add new parameter block like
	```
	if (basicParams[i].getScannerName().compare(SICK_SCANNER_MRS_1XXX_NAME) == 0) 
	{...
	} in the file sick_generic_parser.cpp
	```

5. Copy the file sick_generic_radar.cpp and add a new class following the structure
of this file.











