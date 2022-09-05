## Bugs and Feature Requests

- Sopas protocol mapping:
-- All scanners: COLA-B (Binary)
- Software should be further tested, documented and beautified

## Troubleshooting

The software is based on the ROS drivers sick_scan, sick_scan_base and sick_scan2. For FAQ and troubleshooting please also have a look at https://github.com/SICKAG/sick_scan , https://github.com/SICKAG/sick_scan_base and https://github.com/SICKAG/sick_scan2 .
Common problems might be solved in closed issues.

### General troubleshooting

1. Check Scanner IP in the launch file.
2. Check Ethernet connection to scanner with netcat e.g. ```nc -z -v -w5 $SCANNERIPADDRESS 2112```.
   For further details about setting up the correct ip settings see [IP configuration](doc/ipconfig/ipconfig.md)
3. View node startup output wether the IP connection could be established
4. Check the scanner status using the LEDs on the device. The LED codes are described in the above mentioned operation manuals.
5. Further testing and troubleshooting informations can found in the file test/readme_testplan.txt
6. If you stop the scanner in your debugging IDE or by other hard interruption (like Ctrl-C), you must wait until 60 sec. before
   the scanner is up and running again. During this time the MRS6124 reconnects twice.
   If you do not wait this waiting time you could see one of the following messages:
   * TCP connection error
   * Error-Message 0x0d
7. Amplitude values in rviz: If you see only one color in rviz try the following:
   Set the min/max-Range of intensity display in the range [0...200] and switch on the intensity flag in the launch file  
8. In case of network problems check your own ip address and the ip address of your laser scanner (by using SOPAS ET).
   * List of own IP-addresses: ifconfig|grep "inet addr"
   * Try to ping scanner ip address (used in launch file)
9. If the driver stops during init phase please stop the driver with ctrl-c and restart (could be caused due to protocol ASCII/Binary cola-dialect).

## Support

* In case of technical support please open a new issue. For optimal support, add the following information to your request:
 1. Scanner model name,
 2. Ros node startup log,
 3. Sopas file of your scanner configuration.
  The instructions at http://sickusablog.com/create-and-download-a-sopas-file/ show how to create the Sopas file.
* In case of application support please use [https://supportportal.sick.com ](https://supportportal.sick.com).
* Issue Handling: Issues, for which no reply was received from the questioner for more than 7 days,                     
  are closed by us because we assume that the user has solved the problem.

