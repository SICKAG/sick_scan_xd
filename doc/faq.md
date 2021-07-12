# sick_scan FAQ

## rviz shows a grey point cloud

:question: rviz shows a grey point cloud. The size of points can be adjusted.

:white_check_mark: Check in the launch file that the intensity flag is set to True.

:question: rviz shows a grey point cloud and the size of points can not be adjusted.

:white_check_mark: Probably in this case you are running Linux in a virtual machine. In this case, OpenGL may not work correctly in the VM. rviz then chooses a kind of "fallback solution" and deactivates the colors.
Also, changing the "Size" and "Style" display in rviz has no effect on the display of the pointcloud data.

The problem can be avoided by starting rviz with the following sequence:

```
export LIBGL_ALWAYS_SOFTWARE=1
rosrun rviz rviz
```

## Angular resolution and scan frequency

:question: The angular resolution/ number of shots is too small

:white_check_mark: Possibly Mean or Median filters are activated. Use Sopas ET to deactivate them and store this settings permanent on the device, see picture.
![Sopas_filter](tim5xxx_filter.PNG)
Further information can be found at http://wiki.ros.org/rviz/Troubleshooting.

## "ERROR: Tcp::open: Failed to open TCP connection to 192.168.0.1, aborting."

:question: Question:
sick_generic_caller gives you an answer like:
```bash
"ERROR: Tcp::open: Failed to open TCP connection to 192.168.0.1, aborting."
```

:white_check_mark: Answer:
1. Try to ping your device:
   ```bash
   ping 192.168.0.1
   ```
2. Disconnect your scanner and retry ping

   ```bash
   ping 192.168.0.1
   ```  
   The result of ping contains a pattern like
   ```bash
    ... Destination Host Unreachable
   ```
3. Reconnect your device and try to ping:
   ```bash
   ping 192.168.0.1
   ```

If you do not know the ip address, try to find the ip address in your subnet:
```bash
apt-get install fping
```

scan your network (for example, subnet 192.168.10.0/24):
```bash
192.168.0.1/24
```
search for all ip addresses from 192.168.0.0 to 192.168.0.255

The result is similar to:
```bash
192.168.0.4 is alive
192.168.0.22 is alive
```
and a lot of unreachable entries.
In the example the ip address 192.168.0.4 is the laserscanner MRS1104 and the ip address 192.168.0.22 is the computer running linux. Check this with
```bash
ifconfig|grep 192.168.0.22
```
## IP Address of Laser Scanner

:question: Question:
My scanner does not use the default ip address. What shall I do?



:white_check_mark: Answer:
There are two options doing this:
* Permanently:  
Replace the following entry with your ip address.
```bash
  <param name="hostname" type="string" value="192.168.0.1" />
```
* Temporarily  
Use a command line argument in addition to the launch file argument:
```bash
   hostname:=192.168.0.2
```

## Timeout Warning

:question: Question:
During start phase the are warning/error message like
```bash
no answer received after 5000 ms. Maybe sopas mode is wrong.
```
and some more warning/error messages:

:white_check_mark: Answer:
In this case the driver tries to start the scanner in binary mode. If this is not possible, warnings and error messages are generated.
The driver switches the scanner from ASCII mode to binary mode and then restarts communication. The messages can therefore be ignored.
For a long-term solution, we recommend switching from ASCII to binary communication with SOPAS ET under Windows.

## Own Data Handling

:question: Question:
I would like to process data with my own methods.

:white_check_mark: Answer:
Search for keyword "PUBLISH_DATA:" in the code and replace the code for writing
jpeg-files and CSV-files with your own source code.

## Occasionally missing scan data

:question: Question:
Occasionally, no scan data appear, but the scanner is still reachable using `ping <ip-address>` or `nc -z -v <ip-address> <port>`

:white_check_mark: Answer:
This is likely to be a data capture issue. In any case it's recommend (if not already set) to use SOPAS-Binary instead of SOPAS-ASCII, because here the data rate is lower.

In addition, the following measures can help to determine the source of the problems:
a) Exchange the cables to the lidar(s) incl. the network cables
b) Exchange the power supply to the lidar(s)
c) avoidance of interconnection of other components (like switch etc.)
d) upgrade hardware and firmware of devices (if new versions available)

If it is not a hardware problem (e.g. cable), check if there are other software components using the network interface.

If there are still problems, use Wireshark to see if there are any communication problems reported. Wireshark can export the recorded network traffic as text (see screenshot). Search for entries like "LMDscandata" in the generated text file. This marks the beginning of a new scan message. You can then compare the order of the timestamps of these packets to see if there were any failures.

![Wireshark screenshot](https://user-images.githubusercontent.com/33296049/124088216-01aa2280-da53-11eb-91ae-2b88b37e08eb.png)

## python_d.exe not found

:question: Question:
On Windows with ROS, cmake complains "python_d.exe not found" when running rosidl generator

:white_check_mark: Answer:
Workaround: Copy python.exe to python_d.exe in the python folder.
