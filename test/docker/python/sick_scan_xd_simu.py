import os, shutil, signal, subprocess, sys, time
from datetime import datetime, timezone
from sick_scan_xd_simu_cfg import SickScanXdSimuConfig
from sick_scan_xd_subscriber import SickScanXdMonitor
from sick_scan_xd_simu_report import (
    filter_logfile,
    SickScanXdStatus,
    SickScanXdMsgStatus,
    SickScanXdSimuReport,
)


def start_process(os_name, cmd_list, wait_before, wait_after, logfilepath=""):
    """
    Start a subprocess running a given list of commands.

    On Linux each subprocess is started in its own process group/session.
    This allows terminating the complete process tree later.
    """
    proc = None

    if len(cmd_list) > 0:
        time.sleep(wait_before)

        if os_name == "windows":
            cmd = " & ".join(cmd_list)
            proc = subprocess.Popen(["cmd", "/c", cmd])

        else:
            cmd = " ; ".join(cmd_list)

            if len(logfilepath) > 0:
                cmd = f"(({cmd}) | tee {logfilepath})"

            proc = subprocess.Popen(
                ["bash", "-c", cmd],
                start_new_session=True
            )

        print(
            f"Started subprocess pid={proc.pid}, "
            f"running command {cmd}"
        )

        time.sleep(wait_after)

    return proc


def terminate_process_group(proc, os_name, timeout=1):
    """
    Gracefully terminate a process and all its subprocesses.
    Kill remaining processes after timeout.
    """
    if proc is None:
        return

    if os_name == "windows":
        os.system(f"taskkill /f /t /pid {proc.pid}")
        return

    pgid = proc.pid

    print(
        f'Terminating process group pgid={pgid} '
        f'("{proc.args}")'
    )

    # First try graceful termination
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return

    # Wait until process group has terminated
    end_time = time.time() + timeout

    while time.time() < end_time:
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return

        time.sleep(0.1)

    # Force kill anything still remaining
    print(
        f"Process group pgid={pgid} did not terminate, "
        f"sending SIGKILL"
    )

    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def kill_processes(proc_list, os_name, wait_after):
    """
    Kill all processes including their subprocesses.
    """
    for proc in proc_list:
        terminate_process_group(
            proc,
            os_name,
            timeout=wait_after
        )

    return []


def run_shutdown_command(config):
    """
    Run configured shutdown command with a timeout.
    """
    if len(config.cmd_shutdown) <= 0:
        return

    proc_shutdown = start_process(
        config.os_name,
        config.cmd_init + config.cmd_shutdown,
        0,
        0
    )

    if proc_shutdown is None:
        return

    try:
        proc_shutdown.wait(timeout=10)

    except subprocess.TimeoutExpired:
        print(
            "WARNING: shutdown process did not terminate "
            "within 10 seconds"
        )

        terminate_process_group(
            proc_shutdown,
            config.os_name,
            timeout=1
        )

        try:
            proc_shutdown.wait(timeout=2)
        except subprocess.TimeoutExpired:
            pass


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
    simu_start_time_str = (
        f"{simu_start_time.strftime('%a %m/%d/%Y %H:%M:%S utc')}"
    )

    report = SickScanXdSimuReport()
    config = SickScanXdSimuConfig(simu_start_time)

    if len(config.error_messages) > 0:
        report.append_message(
            SickScanXdMsgStatus.ERROR,
            "## ERROR in sick_scan_xd_simu: invalid configuration:"
        )

        for error_message in config.error_messages:
            report.append_message(
                SickScanXdMsgStatus.ERROR,
                error_message
            )

        report.set_exit_status(
            SickScanXdStatus.CONFIG_ERROR
        )

    elif len(config.cmd_sick_scan_xd) == 0:
        report.append_message(
            SickScanXdMsgStatus.ERROR,
            f"## ERROR in sick_scan_xd_simu: "
            f"ROS version {config.ros_version} "
            f"on {config.os_name} not supported"
        )

        report.set_exit_status(
            SickScanXdStatus.CONFIG_ERROR
        )

    if report.get_exit_status() != SickScanXdStatus.SUCCESS:
        report.print_messages()
        return int(report.get_exit_status())

    report.append_message(
        SickScanXdMsgStatus.INFO,
        f"sick_scan_xd_simu started {simu_start_time_str} "
        f"on {config.os_name}, ROS {config.ros_version}, "
        f"API {config.api}"
    )

    report.append_file_links(
        SickScanXdMsgStatus.INFO,
        "sick_scan_xd_simu config file: ",
        [config.config_file]
    )

    # Run simulation and monitoring
    proc_list = []

    proc_sopas_server_logfile = "proc_sopas_server.log"

    if len(config.cmd_sopas_server) > 0:
        proc_sopas_server = start_process(
            config.os_name,
            config.cmd_init + config.cmd_sopas_server,
            0,
            5,
            logfilepath=(
                f"{config.log_folder}/"
                f"{proc_sopas_server_logfile}"
            )
        )

        proc_list.append(proc_sopas_server)

    if len(config.cmd_rviz) > 0:
        proc_rviz = start_process(
            config.os_name,
            config.cmd_init + config.cmd_rviz,
            0,
            5
        )

        proc_list.append(proc_rviz)

    proc_sick_scan_xd_logfile = "proc_sick_scan_xd.log"

    proc_sick_scan_xd = start_process(
        config.os_name,
        config.cmd_init + config.cmd_sick_scan_xd,
        0,
        5,
        logfilepath=(
            f"{config.log_folder}/"
            f"{proc_sick_scan_xd_logfile}"
        )
    )

    proc_list.append(proc_sick_scan_xd)

    sick_scan_xd_monitor = SickScanXdMonitor(
        config,
        True
    )

    proc_scandata_sender = None
    proc_scandata_sender_logfile = ""

    if len(config.cmd_udp_scandata_sender) > 0:
        proc_scandata_sender_logfile = (
            "proc_scandata_sender.log"
        )

        proc_scandata_sender = start_process(
            config.os_name,
            config.cmd_init +
            config.cmd_udp_scandata_sender,
            3,
            5,
            logfilepath=(
                f"{config.log_folder}/"
                f"{proc_scandata_sender_logfile}"
            )
        )

        proc_list.append(proc_scandata_sender)

    try:

        # Wait until simulation finished
        if proc_scandata_sender is not None:
            status_scandata_sender = (
                proc_scandata_sender.wait()
            )

            time.sleep(1)

            print(
                f"Finished process "
                f"{proc_scandata_sender}, "
                f"exit_status = "
                f"{status_scandata_sender}"
            )

        if config.run_simu_seconds_before_shutdown > 0:
            time.sleep(
                config.run_simu_seconds_before_shutdown
            )

        log_error_messages = []

        if len(proc_sick_scan_xd_logfile) > 0:
            # Get all error messages from logfile,
            # except for ros shutdown messages
            log_error_messages = filter_logfile(
                f"{config.log_folder}/"
                f"{proc_sick_scan_xd_logfile}",
                ["ERROR"],
                ["process has died"]
            )

        if config.ros_version == "none":
            run_shutdown_command(config)

            proc_list = kill_processes(
                proc_list,
                config.os_name,
                1
            )

        # Verification of received pointcloud
        # and laserscan messages
        verify_success = False

        if len(config.save_messages_jsonfile) > 0:

            if config.ros_version == "none":
                sick_scan_xd_monitor.import_received_messages_from_jsonfile(
                    f"{config.log_folder}/"
                    f"{config.save_messages_jsonfile}"
                )

            else:
                sick_scan_xd_monitor.export_received_messages_to_jsonfile(
                    f"{config.log_folder}/"
                    f"{config.save_messages_jsonfile}"
                )

                print(
                    "sick_scan_xd_simu: received messages "
                    "exported to file "
                    f"{config.log_folder}/"
                    f"{config.save_messages_jsonfile}"
                )

            report.append_file_links(
                SickScanXdMsgStatus.INFO,
                "sick_scan_xd_simu: received messages "
                "exported to file ",
                [config.save_messages_jsonfile]
            )

        if len(config.reference_messages_jsonfile) > 0:
            report.append_file_links(
                SickScanXdMsgStatus.INFO,
                "sick_scan_xd_simu: references "
                "messages from file ",
                [config.reference_messages_jsonfile]
            )

            print(
                "sick_scan_xd_simu finished, "
                "verifying messages ..."
            )

            verify_success = (
                sick_scan_xd_monitor.verify_messages(
                    report
                )
            )

        if len(proc_sick_scan_xd_logfile) > 0:
            report.append_file_links(
                SickScanXdMsgStatus.INFO,
                "sick_scan_xd_simu: "
                "sick_scan_xd process log in file ",
                [proc_sick_scan_xd_logfile]
            )

        if len(log_error_messages) > 0:
            report.set_exit_status(
                SickScanXdStatus.TEST_ERROR
            )

            report.append_message(
                SickScanXdMsgStatus.ERROR,
                f"\n## ERROR messages found in logfile "
                f"{proc_sick_scan_xd_logfile}:\n"
            )

            for log_error_message in log_error_messages:
                report.append_message(
                    SickScanXdMsgStatus.ERROR,
                    log_error_message
                )

            report.append_message(
                SickScanXdMsgStatus.ERROR,
                "\n## ERROR in sick_scan_xd_simu: "
                "ERROR messages found in logfile "
                f"{proc_sick_scan_xd_logfile}, "
                "TEST FAILED\n"
            )

        if len(proc_sopas_server_logfile) > 0:
            report.append_file_links(
                SickScanXdMsgStatus.INFO,
                "sick_scan_xd_simu: "
                "sopas server process log in file ",
                [proc_sopas_server_logfile]
            )

        if len(proc_scandata_sender_logfile) > 0:
            report.append_file_links(
                SickScanXdMsgStatus.INFO,
                "sick_scan_xd_simu: "
                "udp scandata sender log in file ",
                [proc_scandata_sender_logfile]
            )

        if verify_success:
            report.append_message(
                SickScanXdMsgStatus.INFO,
                "\nsick_scan_xd_monitor.verify_messages "
                "sucessful, TEST PASSED\n"
            )

        else:
            report.set_exit_status(
                SickScanXdStatus.TEST_ERROR
            )

            report.append_message(
                SickScanXdMsgStatus.ERROR,
                "\n## ERROR in sick_scan_xd_simu: "
                "sick_scan_xd_monitor.verify_messages "
                "returned without success, TEST FAILED\n"
            )

        if (
            report.get_exit_status()
            == SickScanXdStatus.SUCCESS
        ):
            print(
                "\nsick_scan_xd_simu finished, "
                "messages successfully verified, "
                "TEST PASSED\n"
            )

        else:
            print(
                "\n## sick_scan_xd_simu finished "
                "with ERROR, TEST FAILED\n"
            )

        # --------------------------------------------------
        # Shutdown and cleanup
        # --------------------------------------------------

        if config.ros_version != "none":

            #
            # IMPORTANT:
            #
            # Stop the complete ros2 launch process group FIRST.
            #
            # Otherwise ros2 launch can still be alive while
            # sick_generic_caller terminates and can start or
            # manage the node again.
            #
            proc_list = kill_processes(
                proc_list,
                config.os_name,
                1
            )

            #
            # Run optional additional shutdown command only
            # after the launched processes have been stopped.
            #
            run_shutdown_command(config)

        # Print final report
        print(
            f"\nsick_scan_xd_simu finished with "
            f"exit status "
            f"{int(report.get_exit_status())}\n"
        )

        report.save_md_file(
            config.log_folder,
            config.report_md_filename
        )

        with open(
            f"{config.log_folder}/"
            "sick_scan_xd_summary.md",
            "w"
        ) as file_stream:

            if (
                report.get_exit_status()
                == SickScanXdStatus.SUCCESS
            ):
                status_text = "**test passed**"

            else:
                status_text = "**TEST FAILED**"

            report_md_filepath = (
                f"{os.path.basename(config.log_folder)}/"
                f"{config.report_md_filename}"
            )

            report_html_filepath = (
                f"{report_md_filepath}.html"
            )

            file_stream.write(
                f"sick_scan_xd_simu "
                f"{simu_start_time_str} "
                f"on {config.os_name}, "
                f"ROS {config.ros_version}, "
                f"API {config.api}, "
                f"{os.path.basename(config.config_file)}: "
                f"{status_text}, "
                f"[{report_md_filepath}]"
                f"({report_md_filepath}), "
                f"[{report_html_filepath}]"
                f"({report_html_filepath})\n\n"
            )

        report.print_messages()

        # shutil.rmtree(
        #     config.data_folder,
        #     ignore_errors=True
        # )

        print(
            "[INFO] You can remove the folder if the "
            "latest local changes have already been "
            "integrated into data.zip. Automatic removal "
            "is currently disabled."
        )

        if (
            report.get_exit_status()
            == SickScanXdStatus.SUCCESS
        ):
            return 0

        return 1

    except KeyboardInterrupt:

        print(
            "\nWARNING: sick_scan_xd_simu interrupted "
            "by user, cleaning up processes ..."
        )

        proc_list = kill_processes(
            proc_list,
            config.os_name,
            1
        )

        return 130


if __name__ == '__main__':
    status = simu_main()

    print(
        f"sick_scan_xd_simu exits with status {status}"
    )

    sys.exit(status)

