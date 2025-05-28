import argparse
import json
import os
import shutil
from datetime import datetime
from pathlib import Path

class SickScanXdSimuConfig:
    """
    Configuration and commandline parameter for a sick_scan_xd simulation
    """

    def __init__(self, simu_start_time):
        """
        Initializing constructor
        """

        # Parse command line arguments
        os_name = "linux"
        if os.name == "nt":
            os_name = "windows"
        self.data_zip = "./src/sick_scan_xd/test/docker/data.zip"
        self.data_folder = "./src/sick_scan_xd/test/docker/data"
        shutil.unpack_archive(self.data_zip, self.data_folder)
        self.config_file = f"{self.data_folder}/multiscan_compact_test01_cfg.json"
        self.log_folder = f"./log/sick_scan_xd_simu/{simu_start_time.strftime('%Y%m%d_%H%M%S')}"
        parser = argparse.ArgumentParser(description="Run a sick_scan_xd simulation")
        parser.add_argument('--os', type=str, help="OS (linux or windows)", default=os_name, choices=["linux", "windows"])
        parser.add_argument('--ros', type=str, help="ROS version (none, noetic, foxy or humble)", default="noetic", choices=["none", "noetic", "foxy", "humble"])
        parser.add_argument('--api', type=str, help="C++ or Python API if ros version is none", default="", choices=["", "none", "cpp", "python"])
        parser.add_argument('--cfg', type=str, help="configuration file", default=self.config_file)
        parser.add_argument('--log_folder', type=str, help="log folder", default=self.log_folder)
        parser.add_argument('--save_messages_jsonfile', type=str, help="optional jsonfile to save all received messages", default="")
        parser.add_argument('--run_seconds', type=int, help="run simulation in seconds, delay before shutdown", default=-1)
        parser.add_argument('--run_rviz', type=int, help="run rviz (0: rviz disabled, otherwise rviz settings from configuration file)", default=0)
        
        args = parser.parse_args()
        self.os_name = args.os # "linux" or "windows"
        self.ros_version = args.ros # "none", "noetic", "foxy" or "humble"
        self.api = args.api # "", "cpp" or "python"
        self.config_file = args.cfg
        self.log_folder = args.log_folder
        self.report_md_filename = "sick_scan_xd_testreport.md"
        self.error_messages = []
        self.sick_scan_xd_pointcloud_topics = []   # list of sick_scan_xd PointCloud2 topics to be monitored
        self.sick_scan_xd_laserscan_topics = []    # list of sick_scan_xd LaserScan topics to be monitored
        self.sick_scan_xd_imu_topics = []          # list of sick_scan_xd IMU topics to be monitored
        self.reference_messages_jsonfile = ""      # jsonfile with reference pointcloud and laserscan messages, which are verified against sick_scan_xd messages
        self.save_messages_jsonfile = ""           # optionally save all received messages in a json-file, e.g. "sick_scan_xd_simu_received_messages.json"
        self.run_simu_seconds_before_shutdown = 10 # delay before simulation shutdown

        # Read configuration from json file
        if not os.path.isfile(self.config_file):
            self.appendErrorMessage(f"## ERROR SickScanXdSimuConfig: configuration file \"{self.config_file}\" not readable")
        try:
            with open(self.config_file, "r") as file_stream:
                json_config = json.load(file_stream)
            reference_messages_jsonfile = json_config["reference_messages_jsonfile"]
            self.save_messages_jsonfile = json_config["save_messages_jsonfile"]
            self.report_md_filename = json_config["report_md_filename"]
            sick_scan_xd_pointcloud_topics = json_config["sick_scan_xd_pointcloud_topics"]
            sick_scan_xd_laserscan_topics = json_config["sick_scan_xd_laserscan_topics"]
            sick_scan_xd_imu_topics = json_config["sick_scan_xd_imu_topics"]
            args_sick_scan_xd_launch = json_config["args_sick_scan_xd_launch"]
            args_udp_scandata_sender = json_config["args_udp_scandata_sender"]
            args_sopas_server = json_config["args_sopas_server"]
            self.run_simu_seconds_before_shutdown = json_config["run_simu_seconds_before_shutdown"]
            if args.run_rviz > 0:
                args_rviz = json_config["args_rviz_{}_{}".format(self.os_name, self.ros_version)]
            else:
                args_rviz = []
                print("sick_scan_xd_simu: rviz disable, use option --run_rviz=1 to activate rviz visualization.")
            args_shutdown_nodes = json_config["args_shutdown_nodes"]
            args_shutdown_server = json_config["args_shutdown_server"]
            # overwrite parameter with commandline options
            if len(args.save_messages_jsonfile) > 0:
                self.save_messages_jsonfile = args.save_messages_jsonfile
            if len(self.save_messages_jsonfile) > 0:
                self.save_messages_jsonfile = os.path.basename(self.save_messages_jsonfile)
            else:
                self.save_messages_jsonfile = "sick_scan_xd_simu_received_messages.json"
            if args.run_seconds >= 0:
                self.run_simu_seconds_before_shutdown = args.run_seconds
            # with open("/tmp/sick_scan_xd_simu_cfg.json", "w") as file_stream:
            #    json.dump(json_config, file_stream, indent=2)
        except Exception as exc:
            self.appendErrorMessage(f"## ERROR SickScanXdSimuConfig: exception {exc}, check configuration file \"{self.config_file}\"")
        if self.os_name != "linux" and  self.os_name != "windows":
            self.appendErrorMessage(f"## ERROR SickScanXdSimuConfig: OS \"{self.os_name}\" not supported")

        # Create log and output folder
        Path(self.log_folder).mkdir(parents=True, exist_ok=True)
        if not os.path.exists(self.log_folder):
            self.appendErrorMessage(f"## ERROR SickScanXdSimuConfig: could not create output folder \"{self.log_folder}\"")

        # Prepare toolchain commands
        self.cmd_init = []
        self.cmd_sopas_server = []
        self.cmd_rviz = []
        self.cmd_sick_scan_xd = []
        self.cmd_udp_scandata_sender = []
        self.cmd_shutdown = []
        python3_exe = "python3"
        pkill_exe = "pkill -f"
        sleep_exe = "sleep"
        exe_file_extension = ""
        if self.os_name == "windows":
            python3_exe = "python"
            pkill_exe = "taskkill /f /t /im"
            sleep_exe = "timeout /t"
            exe_file_extension = ".exe"
        
        if self.ros_version == "none":

            sick_scan_xd_api_dockertest_exe = "./src/sick_scan_xd/build/sick_scan_xd_api_dockertest"
            if self.os_name == "windows":
                sick_scan_xd_api_dockertest_exe = ".\\src\\sick_scan_xd\\build\\debug\\sick_scan_xd_api_dockertest.exe"
            if self.api == "python":
                sick_scan_xd_api_dockertest_exe = f"{python3_exe} ./src/sick_scan_xd/test/python/sick_scan_xd_api/sick_scan_xd_api_dockertest.py"
            for arg_sick_scan_xd_launch in args_sick_scan_xd_launch:
                self.cmd_sick_scan_xd.append(f"{sick_scan_xd_api_dockertest_exe} ./src/sick_scan_xd/launch/{arg_sick_scan_xd_launch} _jsonfile:={self.log_folder}/{self.save_messages_jsonfile}")
            if len(self.cmd_sick_scan_xd) < 1:
                self.appendErrorMessage(f"## ERROR SickScanXdSimuConfig: sick_scan_xd launch parameter not configured, check configuration file \"{self.config_file}\"")

            self.cmd_shutdown.append(f"{pkill_exe} sick_scan_xd_api_dockertest{exe_file_extension}")
            self.cmd_shutdown.append(f"{pkill_exe} sick_scan_xd_api_dockertest.py")
            for arg_shutdown_server in args_shutdown_server:
                self.cmd_shutdown.append(f"{pkill_exe} {arg_shutdown_server}")

        elif self.ros_version == "noetic" and self.os_name == "linux":

            self.cmd_init.append(f"source /opt/ros/noetic/setup.bash")
            self.cmd_init.append(f"source ./devel_isolated/setup.bash")

            for arg_rviz in args_rviz:
                self.cmd_rviz.append(f"(rosrun rviz rviz -d {arg_rviz} &)")
                self.cmd_rviz.append(f"{sleep_exe} 1")

            for arg_sick_scan_xd_launch in args_sick_scan_xd_launch:
                self.cmd_sick_scan_xd.append(f"roslaunch sick_scan_xd {arg_sick_scan_xd_launch}")
            if len(self.cmd_sick_scan_xd) < 1:
                self.appendErrorMessage(f"## ERROR SickScanXdSimuConfig: sick_scan_xd launch parameter not configured, check configuration file \"{self.config_file}\"")

            for arg_shutdown_nodes in args_shutdown_nodes:
                self.cmd_shutdown.append(f"rosnode kill {arg_shutdown_nodes}")
            self.cmd_shutdown.append(f"{sleep_exe} 3")
            self.cmd_shutdown.append(f"rosnode kill -a")
            for arg_shutdown_server in args_shutdown_server:
                self.cmd_shutdown.append(f"{pkill_exe} {arg_shutdown_server}")

            for sick_scan_xd_pointcloud_topic in sick_scan_xd_pointcloud_topics:
                self.sick_scan_xd_pointcloud_topics.append(sick_scan_xd_pointcloud_topic)
            for sick_scan_xd_laserscan_topic in sick_scan_xd_laserscan_topics:
                self.sick_scan_xd_laserscan_topics.append(sick_scan_xd_laserscan_topic)
            for sick_scan_xd_imu_topic in sick_scan_xd_imu_topics:
                self.sick_scan_xd_imu_topics.append(sick_scan_xd_imu_topic)

        elif self.ros_version == "foxy" or self.ros_version == "humble":

            if self.os_name == "windows":
                self.cmd_init.append(f"call .\\install\\setup.bat")
            else:
                self.cmd_init.append("source /opt/ros/{}/setup.bash".format(self.ros_version))
                self.cmd_init.append("source ./install/setup.bash")

            for arg_rviz in args_rviz:
                if self.os_name == "windows":
                    self.cmd_rviz.append(f"(start /min ros2 run rviz2 rviz2 -d {arg_rviz})")
                else:
                    self.cmd_rviz.append(f"(ros2 run rviz2 rviz2 -d {arg_rviz} &)")
                self.cmd_rviz.append(f"{sleep_exe} 1")

            for arg_sick_scan_xd_launch in args_sick_scan_xd_launch:
                arg_sick_scan_xd_launch = arg_sick_scan_xd_launch.replace(".launch", ".launch.py")
                self.cmd_sick_scan_xd.append(f"ros2 launch sick_scan_xd {arg_sick_scan_xd_launch}")
            if len(self.cmd_sick_scan_xd) < 1:
                self.appendErrorMessage(f"## ERROR SickScanXdSimuConfig: sick_scan_xd launch parameter not configured, check configuration file \"{self.config_file}\"")

            self.cmd_shutdown.append(f"{pkill_exe} sick_generic_caller{exe_file_extension}")
            self.cmd_shutdown.append(f"{sleep_exe} 3")
            self.cmd_shutdown.append(f"{pkill_exe} rviz2{exe_file_extension}")
            for arg_shutdown_server in args_shutdown_server:
                self.cmd_shutdown.append(f"{pkill_exe} {arg_shutdown_server}")

            for sick_scan_xd_pointcloud_topic in sick_scan_xd_pointcloud_topics:
                self.sick_scan_xd_pointcloud_topics.append(sick_scan_xd_pointcloud_topic)
            for sick_scan_xd_laserscan_topic in sick_scan_xd_laserscan_topics:
                self.sick_scan_xd_laserscan_topics.append(sick_scan_xd_laserscan_topic)
            for sick_scan_xd_imu_topic in sick_scan_xd_imu_topics:
                self.sick_scan_xd_imu_topics.append(sick_scan_xd_imu_topic)

        else:
            self.appendErrorMessage(f"## ERROR SickScanXdSimuConfig: combination of os_name={self.os_name} and ros_version={self.ros_version} is not supported")

        # Sopas server configuration for all dockertest (Linux, Windows, ROS1, ROS1, API)
        for arg_sopas_server in args_sopas_server:
            self.cmd_sopas_server.append(f"{python3_exe} {arg_sopas_server}")
        if len(self.cmd_sopas_server) < 1:
            print(f"SickScanXdSimuConfig: sopas server parameter not configured, sopas server not started. This is a valid option for error testcases, but check configuration file \"{self.config_file}\" if this is not intended.")

        # Udp sender configuration for all dockertest (Linux, Windows, ROS1, ROS1, API)
        for arg_udp_scandata_sender in args_udp_scandata_sender:
            self.cmd_udp_scandata_sender.append(f"{python3_exe} {arg_udp_scandata_sender}")

        # Copy config and reference files to logfolder for all dockertest (Linux, Windows, ROS1, ROS1, API)
        if len(reference_messages_jsonfile) > 0 and os.path.exists(reference_messages_jsonfile):
            shutil.copy(reference_messages_jsonfile, self.log_folder)
            self.reference_messages_jsonfile = os.path.basename(reference_messages_jsonfile)
        if len(self.config_file) > 0 and os.path.exists(self.config_file):
            shutil.copy(self.config_file, self.log_folder)
            self.config_file = os.path.basename(self.config_file)

    def appendErrorMessage(self, error_msg):
        """
        Print and append error message
        """
        self.error_messages.append(error_msg)
        print(error_msg)
