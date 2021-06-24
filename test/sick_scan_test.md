# sick_scan_test
##### Devel-Branch
## Table of contents

- [Structure Test Control File](#structure-test-control-file)
- [Structure Test Result File](#structure-test-result-file)
- [Creators](#creators)

## Structure Test Control File
The following XML is an example of a test control file:

```
<?xml version="1.0" encoding="UTF-8"?>
<launch>
	<filename>sick_tim_5xx.launch</filename>
	<paramList>
		<!-- Set the ip address to the ip address of our TiM scanner -->
	<param name="hostname" type="string" value="192.168.0.61" />
	</paramList>
	<resultList>
		<param name="shotsPerLayer" type="int" value="811" />
	</resultList>
</launch>
```

Explanation: The XML file contains the two blocks paramList and resultList. 
The parameters to be overwritten in the launch file are specified in the paramList block. 
The corresponding launch file is referenced by the entry filename. The resultList block specifies 
which measured values are expected. In a first approach, the parameter shotsPerLayer is checked here.

## Structure Test Result File
The following XML is an example of a result file:

```
<?xml version="1.0" encoding="UTF-8"?>
<launch>
	<filename>sick_tim_5xx.launch</filename>
	<paramList>
		<!-- Set the ip address to the ip address of our TiM scanner -->
	<param name="hostname" type="string" value="192.168.0.61" />
	</paramList>
	<resultList>
		<param name="shotsPerLayer" type="int" value="811"  errorCode="0" errorMsg="OK" />
	</resultList>
</launch>
```

Explanation: The result XML file corresponds to the control file. 
The errorCode and errorMsg attributes are added to the parameters 
in the resultList block. A simple grep command with "errorCode" can be used to check the test result after the test has been performed.

## Creators

**Michael Lehning**

- <http://www.lehning.de>

on behalf of SICK AG 

- <http://www.sick.com>

------------------------------------------------------------------------

![SICK Logo](https://sick-syd.data.continum.net/static_2018013123/_ui/desktop/common/images/base/pics/logo.png "SICK Logo")
![Lehning Logo](http://www.lehning.de/style/banner.jpg "LEHNING Logo")


