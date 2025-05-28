import json
import os
from enum import IntEnum

def line_contains_keyword(line, keywords):
    for keyword in keywords:
        if line.find(keyword) >= 0:
            return True
    return False

def filter_logfile(logfile, keywords_pos, keywords_neg):
    filtered_lines = []
    try:
        if os.path.isfile(logfile):
            with open(logfile, "r") as file_stream:
                lines = file_stream.readlines()
                for line in lines:
                    if line_contains_keyword(line, keywords_pos) and not line_contains_keyword(line, keywords_neg):
                        filtered_lines.append(line)
                        break
    except Exception as exc:
        print(f"## ERROR sick_scan_xd_simu: logfile {logfile} not readable, exception {exc}\n")
    return filtered_lines

class SickScanXdStatus(IntEnum):
    SUCCESS = 0,
    ERROR = 1,
    CONFIG_ERROR = 2,
    TEST_ERROR = 3

class SickScanXdMsgStatus(IntEnum):
    ERROR = 1,
    INFO = 2,
    DEBUG = 3

class SickScanXdSimuReport:
    """
    Generate test reports for sick_scan_xd simulations
    """

    def __init__(self):
        """
        Initializing constructor
        """
        self.messages = []
        self.exit_status = SickScanXdStatus.SUCCESS

    def set_exit_status(self, exit_status):
        self.exit_status = exit_status

    def get_exit_status(self):
        return self.exit_status

    def append_message(self, msg_status, message):
        self.messages.append((msg_status, message))

    def append_file_links(self, msg_status, msg, filepaths):
        message = msg
        for filepath in filepaths:
            message = message + f" [{filepath}]({filepath})"
        self.append_message(msg_status, message)

    def print_messages(self, msg_status = [SickScanXdMsgStatus.ERROR, SickScanXdMsgStatus.INFO]):
        for message in self.messages:
            if message[0] in msg_status:
                print(message[1])
        if self.exit_status == SickScanXdStatus.SUCCESS: # success
            print(f"sick_scan_xd_simu exit status: {int(self.exit_status)}, success\n")
        else: # error
            print(f"## ERROR sick_scan_xd_simu exit status: {int(self.exit_status)}, FAILED\n")
    
    def save_md_file(self, log_folder, report_filename, msg_status = [SickScanXdMsgStatus.ERROR, SickScanXdMsgStatus.INFO]):
        with open(f"{log_folder}/{report_filename}", "w") as file_stream:
            if self.exit_status == SickScanXdStatus.SUCCESS: # success
                file_stream.write(f"# sick_scan_xd test report: TEST PASSED\n\n")
            else: # error
                file_stream.write(f"# sick_scan_xd test report: TEST FAILED\n\n")
            for message in self.messages:
                if message[0] in msg_status:
                    file_stream.write(f"{message[1]}\n\n")
            if self.exit_status == SickScanXdStatus.SUCCESS: # success
                file_stream.write(f"\n**sick_scan_xd_simu exit status: {int(self.exit_status)}, TEST PASSED**\n")
            else: # error
                file_stream.write(f"\n**## ERROR sick_scan_xd_simu exit status: {int(self.exit_status)}, TEST FAILED**\n")
            