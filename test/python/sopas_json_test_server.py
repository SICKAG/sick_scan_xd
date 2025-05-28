"""
    A simple sopas test server using json input. A listening tcp socket is opened, incoming connections are accepted and cola telegrams are responded on client requests.
    After 10 seconds, datagrams are sent continously.
    All data (sopas responses and telegrams) are read from json file, which can be created from pcapng-file by pcap_json_converter.py
    Note: This is just a simple test server for sick_scan_xd unittests. It does not emulate any device.

    Usage:
    python sopas_json_test_server.py --tcp_port=<int> --json_file=<filepath>
    
    Example:
    python ../test/python/sopas_test_server.py --tcp_port=2111 --json_file=../emulator/scandata/20221018_rms_1xxx_ascii_rawtarget_object.pcapng.json

"""

import argparse
import datetime
import json
import select
import socket
import time
import threading

"""
    SopasTestServer connects to a tcp client, receives cola telegrams and sends a response to the client.
"""
class SopasTestServer:

    # Constructor
    def __init__(self, tcp_port = 2112, json_tcp_payloads = [], verbosity = 0):
        self.tcp_port = tcp_port
        self.json_tcp_payloads = json_tcp_payloads
        self.verbosity = verbosity
        self.run_message_loop = False

    # Waits for an incoming tcp connection and connects to the tcp client
    def connect(self):
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.serversocket.bind(("", self.tcp_port))
        print("SopasTestServer: listening on tcp port {}".format(self.tcp_port))
        self.serversocket.listen(1)
        (clientsocket, clientaddress) = self.serversocket.accept()
        self.clientsocket = clientsocket
        self.clientaddress = clientaddress
        self.clientsocket.setblocking(0)
        self.run_message_loop = True
        print("SopasTestServer: tcp connection to {} established".format(self.clientaddress))

    # Receives a cola telegram and returns its payload (i.e. the telegram without header and CRC)
    def receiveTelegram(self, recv_timeout_sec):
        payload = bytearray(b"")
        payload_idx = -1
        payload_len = -1
        ready_to_recv = select.select([self.clientsocket], [], [], recv_timeout_sec)
        if ready_to_recv[0]:
            try:
                byte_recv = b"\x00"
                while byte_recv != b"\x02":
                    byte_recv = self.clientsocket.recv(1)
                payload = payload + byte_recv
                while True:
                    byte_recv = self.clientsocket.recv(1)
                    payload = payload + byte_recv
                    if len(payload) == 8 and payload[0:4] == b"\x02\x02\x02\x02": # i.e. 4 byte 0x02020202 + 4 byte payload length
                        payload_len = int.from_bytes(payload[4:8], byteorder='big', signed=False)
                        # print("SopasTestServer.receiveTelegram(): decoded payload_len = {} byte".format(payload_len))
                    if payload in self.json_tcp_payloads:
                        payload_idx = self.json_tcp_payloads.index(payload)
                        break
                    if payload_len > 0 and len(payload) >= payload_len + 9: # 4 byte 0x02020202 + 4 byte payload length + payload + 1 byte CRC
                        break
            except Exception as exc:
                print("## ERROR SopasTestServer.receiveTelegram(): exception {}".format(exc))
                print("## ERROR SopasTestServer.receiveTelegram(): received {} byte telegram {}".format(len(payload), payload))
            if self.verbosity > 1:
                print("SopasTestServer.receiveTelegram(): received {} byte telegram {}".format(len(payload), payload))
            elif self.verbosity > 0:
                print("SopasTestServer.receiveTelegram(): received {} byte telegram".format(len(payload)))
        return payload, payload_idx

    # Sends a cola telegram "as is"
    def sendTelegram(self, telegram):
        if self.verbosity > 1:
            print("SopasTestServer.sendTelegram(): sending {} byte telegram {}".format((len(telegram)), telegram))
        elif self.verbosity > 0:
            print("SopasTestServer.sendTelegram(): sending {} byte telegram".format(len(telegram)))
        self.clientsocket.send(telegram)

    # Runs the message loop, i.e. receives sopas telegrams and sends a response to the client
    def run(self):
        print("SopasTestServer: running event loop...")
        while self.run_message_loop:
            # Receive a cola telegram
            received_telegram, json_tcp_payload_idx = self.receiveTelegram(1)
            if len(received_telegram) <= 0: # timeout (no message rececived)
                continue
            # Lookup sopas response to sopas request
            if received_telegram[8:25] == b"sRN SCdevicestate":
                response_payload = b"\x02\x02\x02\x02\x00\x00\x00\x13\x73\x52\x41\x20\x53\x43\x64\x65\x76\x69\x63\x65\x73\x74\x61\x74\x65\x20\x01\x1e" # always send "sRA SCdevicestate 1" (simulate device ready) even if json file recorded "sRA SCdevicestate 0" (device not ready)
                if self.verbosity > 0:
                    print("SopasTestServer: request={}, response={}".format(received_telegram, response_payload))
                self.sendTelegram(response_payload)
            elif json_tcp_payload_idx >= 0 and json_tcp_payload_idx + 1 < len(self.json_tcp_payloads):
                response_payload = self.json_tcp_payloads[json_tcp_payload_idx + 1]
                if self.verbosity > 0:
                    print("SopasTestServer: request={}, response={}".format(received_telegram, response_payload))
                self.sendTelegram(response_payload)
                # Some NAV-350 sopas requests have 2 responses (1. response: acknowledge, 2. response: data), send next telegram in this case
                if received_telegram[8:28] == b"sMN mNEVAChangeState" or received_telegram[8:26] == b"sMN mNMAPDoMapping":
                    if json_tcp_payload_idx + 2 < len(self.json_tcp_payloads):
                        response_payload = self.json_tcp_payloads[json_tcp_payload_idx + 2]
                        if self.verbosity > 0:
                            print("SopasTestServer: request={}, response={}".format(received_telegram, response_payload))
                        self.sendTelegram(response_payload)
            elif received_telegram[8:28] == b"sMN mNLAYAddLandmark":
                response_payload = b"\x02\x02\x02\x02\x00\x00\x00\x20\x73\x41\x4e\x20\x6d\x4e\x4c\x41\x59\x41\x64\x64\x4c\x61\x6e\x64\x6d\x61\x72\x6b\x20\x00\x00\x04\x00\x00\x00\x01\x00\x02\x00\x03\x7c" # "....... sAN mNLAYAddLandmark ............"
                if self.verbosity > 0:
                    print("SopasTestServer: request={}, response={}".format(received_telegram, response_payload))
                self.sendTelegram(response_payload)
            elif received_telegram[8:33] == b"sRN SetActiveApplications":
                response_payload = b"\x02\x02\x02\x02\x00\x00\x00\x19sAN SetActiveApplications\x1b"
                if self.verbosity > 0:
                    print("SopasTestServer: request={}, response={}".format(received_telegram, response_payload))
                self.sendTelegram(response_payload)
            elif received_telegram[8:27] == b"sWN ScanLayerFilter":
                response_payload = b"\x02\x02\x02\x02\x00\x00\x00\x14\x73\x57\x41\x20\x53\x63\x61\x6e\x4c\x61\x79\x65\x72\x46\x69\x6c\x74\x65\x72\x20\x39" # "........sWA ScanLayerFilter"
                if self.verbosity > 0:
                    print("SopasTestServer: request={}, response={}".format(received_telegram, response_payload))
                self.sendTelegram(response_payload)
            elif received_telegram[8:24] == b"sWN FREchoFilter":
                response_payload = b"\x02\x02\x02\x02\x00\x00\x00\x11\x73\x57\x41\x20\x46\x52\x45\x63\x68\x6f\x46\x69\x6c\x74\x65\x72\x20\x70" # "........sWA FREchoFilter"
                if self.verbosity > 0:
                    print("SopasTestServer: request={}, response={}".format(received_telegram, response_payload))
                self.sendTelegram(response_payload)
            else:
                request_hex_str = ":".join("{:02x}".format(x) for x in received_telegram)
                print("## ERROR SopasTestServer: request={} not found in json file, request_hex={}".format(received_telegram, request_hex_str))
                #response_payload = received_telegram
                #if response_payload[8:11] == b"sWN":
                #    response_payload[8:11] = b"sWA"
                #if response_payload[8:11] == b"sMN":
                #    response_payload[8:11] = b"sMA"
                #print("## ERROR SopasTestServer: dummy_response={}".format(response_payload))
                #self.sendTelegram(response_payload)
            time.sleep(0.01)
        
    # Starts the message loop in a background thread
    def start(self):
        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True
        thread.start()

    # Stops the message loop thread
    def stop(self):
        self.run_message_loop = False

"""
    Run sopas test server using json input for sopas requests and responses
"""
if __name__ == "__main__":

    # Configuration
    tcp_port = 2111 # tcp port to listen for tcp connections
    scandata_ids = [ "sSN LMDradardata", "sSN LMDscandata", "sSN InertialMeasurementUnit" ]
    json_file = "../emulator/scandata/20221018_rms_1xxx_ascii_rawtarget_object.pcapng.json"  # input jsonfile with sopas requests, responses and telegrams
    repeat = 0 # number of repetions (default: 0 for endless loop)
    verbosity = 2  # print all telegrams
    send_rate = 10 # send 10 scandata telegrams per second
    send_scandata_after = 5 # start to send scandata 5 seconds after server start 
    
    # Overwrite with command line arguments
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--tcp_port", help="tcp port to listen for tcp connections", default=tcp_port, type=int)
    arg_parser.add_argument("--json_file", help="input jsonfile with sopas requests, responses and telegrams", default=json_file, type=str)
    arg_parser.add_argument("--scandata_id", help="sopas id of scandata telegrams, e.g. \"sSN LMDradardata\"", default="", type=str)
    arg_parser.add_argument("--send_rate", help="send rate in telegrams per second", default=send_rate, type=float)
    arg_parser.add_argument("--repeat", help="number of repetions (or 0 for endless loop)", default=repeat, type=int)
    arg_parser.add_argument("--verbosity", help="verbosity (0, 1 or 2)", default=verbosity, type=int)
    arg_parser.add_argument("--scandata_after", help="start to send scandata after some seconds", default=send_scandata_after, type=float)
    cli_args = arg_parser.parse_args()
    tcp_port = cli_args.tcp_port
    json_file = cli_args.json_file
    if len(cli_args.scandata_id) > 0:
        # scandata_ids = [ cli_args.scandata_id ]
        scandata_ids = [ ]
        for scandata_id in cli_args.scandata_id.split(","):
            scandata_ids.append(scandata_id)
        for n, scandata_id in enumerate(scandata_ids):
            scandata_ids[n] = scandata_ids[n].replace("?", " ")
            scandata_ids[n] = scandata_ids[n].replace("\"", "")
        print("sopas_json_test_server: scandata_ids = {}".format(scandata_ids))
    verbosity = cli_args.verbosity
    send_rate = cli_args.send_rate
    send_scandata_after = cli_args.scandata_after
    if repeat <= 0: # run in endless loop
        repeat = 0x7FFFFFFF
    
    # Parse json file
    print("sopas_json_test_server: parsing json file \"{}\":".format(json_file))
    with open(json_file, 'r') as file_stream:
        json_input = json.load(file_stream)
    json_tcp_payloads = [] # list of bytearray of the tcp payload
    for json_entry in json_input:
        try:
            tcp_description = json_entry["_source"]["layers"]["tcp"]["tcp.description"]
            tcp_payload_json = json_entry["_source"]["layers"]["tcp"]["tcp.payload"]
            tcp_payload_hex_str = "".join(tcp_payload_json.split(":"))
            tcp_payload = bytearray.fromhex(tcp_payload_hex_str)
            json_tcp_payloads.append(tcp_payload)
            # print("tcp_description: \"{}\", tcp_payload: \"{}\", payload_bytes: {}".format(tcp_description, tcp_payload, payload))
        except Exception as exc:
            print("## ERROR parsing file {}: \"{}\", exception {}".format(json_file, json_entry, exc))
    if verbosity > 1:
        for json_tcp_payload in json_tcp_payloads:     
            print("{}".format(json_tcp_payload))

    # Run sopas test server
    print("sopas_json_test_server: running event loop ...")
    server = SopasTestServer(tcp_port, json_tcp_payloads, verbosity)
    server.connect()
    server.start()

    # Send sopas telegrams, e.g. "sSN LMDradardata ..." or "sSN LMDscandata ..."
    time.sleep(send_scandata_after)
    print("sopas_json_test_server: start sending telegrams {} ...".format(" , ".join(scandata_ids)))
    for n in range(len(scandata_ids)):
        scandata_ids[n] = bytearray(scandata_ids[n].encode())
    for repeat_cnt in range(repeat):
        for payload in json_tcp_payloads:
            for scandata_id in scandata_ids:
                if payload.find(scandata_id,0) >= 0:
                    # print("sopas_json_test_server: sending scandata \"{}\" ...".format(payload))
                    server.sendTelegram(payload)
                    time.sleep(1.0 / send_rate)

    server.stop()
