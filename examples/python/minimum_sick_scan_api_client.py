"""Minimalistic usage example for sick_scan_api

Usage: minimum_sick_scan_api_client.py launchfile

Example:
    export LD_LIBRARY_PATH=.:./build:$LD_LIBRARY_PATH
    export PYTHONPATH=.:./python/api:$PYTHONPATH
    python3 ./examples/python/minimum_sick_scan_api_client.py ./launch/sick_tim_7xx.launch

See doc/sick_scan_api/sick_scan_api.md for further information.

"""

import os
import sys
import time

# Make sure sick_scan_api is searched in all folders configured in environment variable PYTHONPATH
def appendPythonPath():
    pythonpath = os.environ['PYTHONPATH']
    for folder in pythonpath.split(";"):
        sys.path.append(os.path.abspath(folder))
try:
    # import sick_scan_api
    from sick_scan_api import *
except ModuleNotFoundError:
    print("import sick_scan_api failed, module sick_scan_api not found, trying with importlib...")
    appendPythonPath()
    import importlib
    sick_scan_api = importlib.import_module("sick_scan_api")

def pyCustomizedPointCloudMsgCb(api_handle, msg):
    """
    Implement a callback to process pointcloud messages
    Data processing to be done
    """
    print("Python PointCloudMsgCb: {} x {} pointcloud message received".format(msg.contents.width, msg.contents.height))

# Pass launchfile and commandline arguments to sick_scan_library
cli_args = " ".join(sys.argv[1:])

# Load sick_scan_library
if os.name == "nt": # Load windows dll
    sick_scan_library = SickScanApiLoadLibrary(["build/Debug/", "build_win64/Debug/", "../../build/Debug/", "../../build_win64/Debug/", "./", "../"], "sick_scan_xd_shared_lib.dll")
else: # Load linux so
    sick_scan_library = SickScanApiLoadLibrary(["build/", "build_linux/", "../../build/", "../../build_linux/", "./", "../"], "libsick_scan_xd_shared_lib.so")

# Create a sick_scan instance and initialize a TiM-7xx
api_handle = SickScanApiCreate(sick_scan_library)
SickScanApiInitByLaunchfile(sick_scan_library, api_handle, cli_args)

# Register for pointcloud messages
cartesian_pointcloud_callback = SickScanPointCloudMsgCallback(pyCustomizedPointCloudMsgCb)
SickScanApiRegisterCartesianPointCloudMsg(sick_scan_library, api_handle, cartesian_pointcloud_callback)

# Run application or main loop
time.sleep(10)

# Close lidar and release sick_scan api
SickScanApiDeregisterCartesianPointCloudMsg(sick_scan_library, api_handle, cartesian_pointcloud_callback)
SickScanApiClose(sick_scan_library, api_handle)
SickScanApiRelease(sick_scan_library, api_handle)
SickScanApiUnloadLibrary(sick_scan_library)
