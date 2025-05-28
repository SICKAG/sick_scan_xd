import os, shutil, signal, subprocess, sys, time
from datetime import datetime, timezone
from sick_scan_xd_simu_cfg import SickScanXdSimuConfig
from sick_scan_xd_subscriber import SickScanXdMonitor
from sick_scan_xd_simu_report import filter_logfile, SickScanXdStatus, SickScanXdMsgStatus, SickScanXdSimuReport

def start_process(os_name, cmd_list, wait_before, wait_after, logfilepath=""):
    """
    Start a subprocess running a given list of commands
    """
    proc = None
    if len(cmd_list) > 0:
        time.sleep(wait_before)
        if os_name == "windows":
            cmd = " & ".join(cmd_list)
            proc = subprocess.Popen(["cmd", "/c", cmd]) # proc = subprocess.Popen(f"{cmd}", creationflags=subprocess.CREATE_NEW_PROCESS_GROUP, shell=True)
        else:
            cmd = " ; ".join(cmd_list)
            if len(logfilepath) > 0:
                cmd = f"(({cmd}) | tee {logfilepath})"
            proc = subprocess.Popen(["bash", "-c", cmd])
        print(f"Started subprocess {proc}, running command {cmd}")
        time.sleep(wait_after)
    return proc

def kill_processes(proc_list, os_name, wait_after):
    """
    Kill all processes incl. their subprocess
    """
    for proc in proc_list:
        print(f"Killing pid={proc.pid} (\"{proc.args}\")")
        if os_name == "windows":
            os.system(f"taskkill /f /t /pid {proc.pid}")
        else:
            os.system(f"kill -9 {proc.pid}")
    return [] # return empty process list

def simu_main():
    """
    Runs a sick_scan_xd simulation:
    * Start a tiny sopas test server to emulate sopas responses of a multiScan,
    * Launch sick_scan_xd,
    * Start rviz to display pointclouds and laserscan messages
    * Replay UDP packets with scan data, previously recorded and converted to json-file
    * Receive the pointcloud-, laserscan- and IMU-messages published by sick_scan_xd
    * Compare the received messages to predefined reference messages
    * Check against errors, verify complete and correct messages
    * Return status 0 for success or an error code in case of any failures.
    """

    # Parse commandline arguments and read configuration
    simu_start_time = datetime.now(timezone.utc)
    simu_start_time_str = f"{simu_start_time.strftime('%a %m/%d/%Y %H:%M:%S utc')}"
    report = SickScanXdSimuReport()
    config = SickScanXdSimuConfig(simu_start_time)
    if len(config.error_messages) > 0:
        report.append_message(SickScanXdMsgStatus.ERROR, f"## ERROR in sick_scan_xd_simu: invalid configuration:")
        for error_message in config.error_messages:
            report.append_message(SickScanXdMsgStatus.ERROR, error_message)
        report.set_exit_status(SickScanXdStatus.CONFIG_ERROR)
    elif len(config.cmd_sick_scan_xd) == 0:
        report.append_message(SickScanXdMsgStatus.ERROR, f"## ERROR in sick_scan_xd_simu: ROS version {config.ros_version} on {config.os_name} not supported")
        report.set_exit_status(SickScanXdStatus.CONFIG_ERROR)
    if report.get_exit_status() != SickScanXdStatus.SUCCESS:
        report.print_messages()
        return int(report.get_exit_status())
    report.append_message(SickScanXdMsgStatus.INFO, f"sick_scan_xd_simu started {simu_start_time_str} on {config.os_name}, ROS {config.ros_version}, API {config.api}")
    report.append_file_links(SickScanXdMsgStatus.INFO, "sick_scan_xd_simu config file: ", [config.config_file])

    # Run simulation and monitoring
    proc_list = []
    proc_sopas_server_logfile = "proc_sopas_server.log"
    if len(config.cmd_sopas_server) > 0:
        proc_sopas_server = start_process(config.os_name, config.cmd_init + config.cmd_sopas_server, 0, 5, logfilepath=f"{config.log_folder}/{proc_sopas_server_logfile}")
        proc_list.append(proc_sopas_server)
    if len(config.cmd_rviz) > 0:
        proc_rviz = start_process(config.os_name, config.cmd_init + config.cmd_rviz, 0, 5)
        proc_list.append(proc_rviz)
    proc_sick_scan_xd_logfile = "proc_sick_scan_xd.log"
    proc_sick_scan_xd = start_process(config.os_name, config.cmd_init + config.cmd_sick_scan_xd, 0, 5, logfilepath=f"{config.log_folder}/{proc_sick_scan_xd_logfile}")
    proc_list.append(proc_sick_scan_xd)
    sick_scan_xd_monitor = SickScanXdMonitor(config, True)
    proc_scandata_sender = None
    proc_scandata_sender_logfile = ""
    if len(config.cmd_udp_scandata_sender) > 0:
        proc_scandata_sender_logfile = "proc_scandata_sender.log"
        proc_scandata_sender = start_process(config.os_name, config.cmd_init + config.cmd_udp_scandata_sender, 3, 5, logfilepath=f"{config.log_folder}/{proc_scandata_sender_logfile}")
        proc_list.append(proc_scandata_sender)
    
    # Wait until simulation finished
    if proc_scandata_sender is not None:
        status_scandata_sender = proc_scandata_sender.wait()
        time.sleep(1)    
        print(f"Finished process {proc_scandata_sender}, exit_status = {status_scandata_sender}")
    if config.run_simu_seconds_before_shutdown > 0:
        time.sleep(config.run_simu_seconds_before_shutdown)
    log_error_messages = []
    if len(proc_sick_scan_xd_logfile) > 0:
        # Get all error messages from logfile, except for ros shutdown messages at simulation exit
        log_error_messages = filter_logfile(f"{config.log_folder}/{proc_sick_scan_xd_logfile}", [ "ERROR" ], [ "process has died" ])
    if config.ros_version == "none": # Shutdown and cleanup 
        proc_shutdown = start_process(config.os_name, config.cmd_init + config.cmd_shutdown, 0, 5)
        proc_shutdown.wait()
        proc_list = kill_processes(proc_list, config.os_name, 5)

    # Verification of received pointcloud and laserscan messages
    verify_success = False
    if len(config.save_messages_jsonfile) > 0:
        if config.ros_version == "none":
            sick_scan_xd_monitor.import_received_messages_from_jsonfile(f"{config.log_folder}/{config.save_messages_jsonfile}")
        else:
            sick_scan_xd_monitor.export_received_messages_to_jsonfile(f"{config.log_folder}/{config.save_messages_jsonfile}")
        report.append_file_links(SickScanXdMsgStatus.INFO, "sick_scan_xd_simu: received messages exported to file ", [config.save_messages_jsonfile])
    if len(config.reference_messages_jsonfile) > 0:
        report.append_file_links(SickScanXdMsgStatus.INFO, "sick_scan_xd_simu: references messages from file ", [config.reference_messages_jsonfile])
        print("sick_scan_xd_simu finished, verifying messages ...")
        verify_success = sick_scan_xd_monitor.verify_messages(report)
    if len(proc_sick_scan_xd_logfile) > 0:
        report.append_file_links(SickScanXdMsgStatus.INFO, "sick_scan_xd_simu: sick_scan_xd process log in file ", [proc_sick_scan_xd_logfile])
    if len(log_error_messages) > 0:
        report.set_exit_status(SickScanXdStatus.TEST_ERROR)
        report.append_message(SickScanXdMsgStatus.ERROR, f"\n## ERROR messages found in logfile {proc_sick_scan_xd_logfile}:\n")            
        for log_error_message in log_error_messages:
            report.append_message(SickScanXdMsgStatus.ERROR, log_error_message)
        report.append_message(SickScanXdMsgStatus.ERROR, f"\n## ERROR in sick_scan_xd_simu: ERROR messages found in logfile {proc_sick_scan_xd_logfile}, TEST FAILED\n")            
    if len(proc_sopas_server_logfile) > 0:
        report.append_file_links(SickScanXdMsgStatus.INFO, "sick_scan_xd_simu: sopas server process log in file ", [proc_sopas_server_logfile])
    if len(proc_scandata_sender_logfile) > 0:
        report.append_file_links(SickScanXdMsgStatus.INFO, "sick_scan_xd_simu: udp scandata sender log in file ", [proc_scandata_sender_logfile])
    if verify_success:
        report.append_message(SickScanXdMsgStatus.INFO, f"\nsick_scan_xd_monitor.verify_messages sucessful, TEST PASSED\n")
    else:
        report.set_exit_status(SickScanXdStatus.TEST_ERROR)
        report.append_message(SickScanXdMsgStatus.ERROR, f"\n## ERROR in sick_scan_xd_simu: sick_scan_xd_monitor.verify_messages returned without success, TEST FAILED\n")
    if report.get_exit_status() == SickScanXdStatus.SUCCESS:    
        print("\nsick_scan_xd_simu finished, messages successfully verified, TEST PASSED\n")
    else:
        print("\n## sick_scan_xd_simu finished with ERROR, TEST FAILED\n")

    # Shutdown and cleanup
    if config.ros_version != "none": # Shutdown and cleanup 
        proc_shutdown = start_process(config.os_name, config.cmd_init + config.cmd_shutdown, 0, 5)
        proc_shutdown.wait()
        proc_list = kill_processes(proc_list, config.os_name, 5)

    # Print final report
    print(f"\nsick_scan_xd_simu finished with exit status {int(report.get_exit_status())}\n")
    report.save_md_file(config.log_folder, config.report_md_filename)
    with open(f"{config.log_folder}/sick_scan_xd_summary.md", "w") as file_stream:
        if report.get_exit_status() == SickScanXdStatus.SUCCESS:    
            status_text = "**test passed**"
        else:
            status_text = "**TEST FAILED**"
        report_md_filepath = f"{os.path.basename(config.log_folder)}/{config.report_md_filename}"
        report_html_filepath = f"{report_md_filepath}.html"
        file_stream.write(f"sick_scan_xd_simu {simu_start_time_str} on {config.os_name}, ROS {config.ros_version}, API {config.api}, {os.path.basename(config.config_file)}: {status_text}, [{report_md_filepath}]({report_md_filepath}), [{report_html_filepath}]({report_html_filepath})\n\n")
    report.print_messages()
    shutil.rmtree(config.data_folder, ignore_errors=True)
    if report.get_exit_status() == SickScanXdStatus.SUCCESS:
        return 0 # 0 = success, otherwise error
    else:
        return 1

if __name__ == '__main__':
    status = simu_main()
    print(f"sick_scan_xd_simu exits with status {status}")
    sys.exit(status)
