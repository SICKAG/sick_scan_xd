"""
    A simple sopas test server. A listening tcp socket is opened, incoming connections are accepted and some basic cola telegrams are responded on client requests.
    Note: This is just a simple test server for basic unittests of sick_scansegment_xd cola commands. It does not emulate any device.

    Usage:
    python sopas_test_server.py --tcp_port=<int> --cola_binary=<int>
    
    Example:
    python ../test/python/sopas_test_server.py --tcp_port=2111 --cola_binary=0

"""

import argparse
import datetime
import select
import socket
import time

# Decodes and returns the payload length of a cola message, i.e. returns message_payload_length in a message := { 4 byte STX 0x02020202 } +  { 4 byte message_payload_length } + { message_payload } + { 1 byte CRC }
def parseBinaryColaPayloadLength(payload):
    length = 0
    if len(payload) > 7 and payload.startswith(b"\x02\x02\x02\x02"):
        length = (payload[4] << 24) + (payload[5] << 16) + (payload[6] << 8) + (payload[7] << 0)
    return length

# Returns a string as bytearray
def stringToByteArray(message):
    return bytearray(message.encode())

# Removes optional ":" from message and returns hex values as bytearray
def hexStringToByteArray(message):
    message_hex = message.replace(":", "")
    return bytearray.fromhex(message_hex)

"""
    ColaResponseMap maps a cola request like "sMN SetAccessMode" to a binary or ascii cola response.
"""
class ColaResponseMap:

    # Constructor
    def __init__(self, cola_binary = 0, val_FREchoFilter = 0):
        if cola_binary > 0:
            self.mapped_response = { 
                "sMN SetAccessMode": "02:02:02:02:00:00:00:13:73:41:4e:20:53:65:74:41:63:63:65:73:73:4d:6f:64:65:20:01:38" ,
                "sWN EIHstCola": "02:02:02:02:00:00:00:0e:73:57:41:20:45:49:48:73:74:43:6f:6c:61:20:07",
                "sRN FirmwareVersion": "02:02:02:02:00:00:00:1f:73:52:41:20:46:69:72:6d:77:61:72:65:56:65:72:73:69:6f:6e:20:00:09:56:31:2e:38:30:20:20:20:20:43",
                "sRN SCdevicestate": "02:02:02:02:00:00:00:13:73:52:41:20:53:43:64:65:76:69:63:65:73:74:61:74:65:20:00:1f",
            }
        else:
            self.mapped_response = { 
                "sRN SCdevicestate": "\x02sRA SCdevicestate 1\x03",                 # "sRN SCdevicestate" -> "sRA SCdevicestate 1"
                "sMN IsSystemReady": "\x02sAN IsSystemReady 1\x03",                 # "sMN IsSystemReady" -> "sAN IsSystemReady 1"
                "sMN SetAccessMode": "\x02sAN SetAccessMode 1\x03",                 # "sMN SetAccessMode 3 F4724744" -> "sAN SetAccessMode 1"
                "sMN Run": "\x02sAN Run 1\x03",                                     # "sMN Run" -> "sAN Run 1"
                "sMN LMCstartmeas": "\x02sAN LMCstartmeas\x03",                     # "sMN LMCstartmeas" -> "sAN LMCstartmeas"
                "sWN ScanDataEnable": "\x02sWA ScanDataEnable\x03",                 # "sWN ScanDataEnable 1" -> "sWA ScanDataEnable"
                "sWN ScanDataFormatSettings": "\x02sWA ScanDataFormatSettings\x03", # "sWN ScanDataFormatSettings 1" -> "sWA ScanDataFormatSettings"
                "sWN ScanDataPreformattingSettings": "\x02sWA ScanDataPreformattingSettings\x03", # "sWN ScanDataPreformattingSettings 1" -> "sWA ScanDataPreformattingSettings"  
                "sWN ScanDataFormat": "\x02sWA ScanDataFormat\x03",                               # "sWN ScanDataFormat 1" -> "sWA ScanDataFormat"
                "sWN ScanDataPreformatting": "\x02sWA ScanDataPreformatting\x03",                 # "sWN ScanDataPreformatting 1" -> "sWA ScanDataPreformatting"
                "sWN ScanDataEthSettings": "\x02sWA ScanDataEthSettings\x03",       # "sWN ScanDataEthSettings 1 +127 +0 +0 +1 +2115" -> "sWA ScanDataEthSettings"
                "sRN FREchoFilter": "\x02sRA FREchoFilter {}\x03".format(val_FREchoFilter),                            # "sRN FREchoFilter" -> "sRA FREchoFilter 0" (default: 0, i.e. first echo only, echo_count = 1)
                "sRN LFPangleRangeFilter": "\x02sRA LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1\x03",  # "sRN LFPangleRangeFilter" -> "sRA LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1"
                "sRN LFPlayerFilter": "\x02sRA LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1\x03",                  # "sRN LFPlayerFilter" -> "sRA LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
                "sWN FREchoFilter": "\x02sWA FREchoFilter\x03",                                                        # "sWN FREchoFilter 1" -> "sWA FREchoFilter"
                "sWN LFPangleRangeFilter": "\x02sWA LFPangleRangeFilter\x03",                                          # "sWN LFPangleRangeFilter 0 C0490FF9 40490FF9 BFC90FF9 3FC90FF9 1" -> "sWA LFPangleRangeFilter"
                "sWN LFPlayerFilter": "\x02sWA LFPlayerFilter\x03",                                                    # "sWN LFPlayerFilter 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1" -> "sWA LFPlayerFilter"
                "sWN LFPintervalFilter": "\x02sWA LFPintervalFilter\x03",                                              # "sWN LFPintervalFilter 0 1" -> "sWA LFPintervalFilter"
                "sRN ContaminationResult": "\x02sRA ContaminationResult 0 0\x03",                                      # "sRN ContaminationResult" -> "sRA ContaminationResult 0 0"
                "sEN InertialMeasurementUnit": "\x02sEA InertialMeasurementUnit\x03",                                  # "sEN InertialMeasurementUnit" -> "sEA InertialMeasurementUnit"
                "sWN ImuDataEnable": "\x02sWA ImuDataEnable 1\x03",                                                    # "sWN ImuDataEnable" -> "sWA ImuDataEnable"
                "sWN ImuDataEthSettings": "\x02sWA ImuDataEthSettings\x03",                                            # "sWN ImuDataEthSettings" -> "sWA ImuDataEthSettings"
                # Simulate picoScan150 w/o addons (no IMU available): "sWN ImuDataEthSettings" -> "sFA 3" (unknown sopas index, no IMU or IMU license error)
                # "sWN ImuDataEnable": "\x02sFA 3\x03",            # "sWN ImuDataEnable" -> "sFA 3"
                # "sWN ImuDataEthSettings": "\x02sFA 3\x03",       # "sWN ImuDataEthSettings" -> "sFA 3"
            }

    # Search for a mapped response given a cola request and returns key and response as strings
    def findResponse(self, request):
        for key, response in self.mapped_response.items():
            if request.find(stringToByteArray(key)) >= 0:
                return key, response
        return "", ""

"""
    SopasTestServer connects to a tcp client, receives cola telegrams and sends a response to the client.
"""
class SopasTestServer:

    # Constructor
    def __init__(self, tcp_port = 2111, cola_binary = 0, val_FREchoFilter = 0):
        self.tcp_port = tcp_port
        self.cola_binary = cola_binary
        self.val_FREchoFilter = val_FREchoFilter

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
        print("SopasTestServer: tcp connection to {} established".format(self.clientaddress))

    # Receives a cola telegram and returns its payload (i.e. the telegram without header and CRC)
    def receiveTelegram(self, recv_timeout_sec):
        ready_to_recv = select.select([self.clientsocket], [], [], recv_timeout_sec)
        if not ready_to_recv[0]:
            return ""
        payload = bytearray(b"")
        if self.cola_binary > 0:
            header = self.clientsocket.recv(8)
            payload_length = parseBinaryColaPayloadLength(header) # message := { 4 byte STX 0x02020202 } +  { 4 byte message_payload_length } + { message_payload } + { 1 byte CRC }
            if payload_length <= 1:
                print("## ERROR SopasTestServer.receiveTelegram(): unexpected binary payload_length {} in received header {}".format(payload_length, header))
                return header
            while len(payload) < payload_length:
                chunk = self.clientsocket.recv(payload_length - len(payload))
                payload = payload + chunk
            crc = bytearray(b"")
            while len(crc) < 1:
                crc = self.clientsocket.recv(1)
        else:
            header = b"\x00"
            while header != b"\x02":
                header = self.clientsocket.recv(1)
            while True:
                byte_recv = self.clientsocket.recv(1)
                if byte_recv == b"\x03":
                    break
                payload = payload + byte_recv
        print("SopasTestServer.receiveTelegram(): received {} byte telegram {}".format((len(header) + len(payload) + 1), payload))
        return payload

    # Sends a cola telegram "as is"
    def sendTelegram(self, telegram, verbosity):
        if verbosity > 1:
            print("SopasTestServer.sendTelegram(): sending {} byte telegram {}".format((len(telegram)), telegram))
        elif verbosity > 0:
            print("SopasTestServer.sendTelegram(): sending {} byte telegram".format(len(telegram)))
        self.clientsocket.send(telegram.encode("utf-8"))

    # Runs the message loop, i.e. receives binary cola telegrams and sends a response to the client
    def run(self):
        response_map = ColaResponseMap(self.cola_binary, self.val_FREchoFilter)
        print("SopasTestServer: running main loop...")
        while True:
            # Receive a cola telegram
            received_telegram = self.receiveTelegram(1)
            if len(received_telegram) <= 0: # timeout (no message rececived)
                continue
            # Lookup for keywords and send a response from predefined (mapped) responses
            request_key, response_str = response_map.findResponse(received_telegram)
            if len(request_key) > 0 and len(response_str) > 0: # request and response found in map, send response to client
                print("SopasTestServer: received {}: {}".format(request_key, received_telegram))
                if self.cola_binary > 0:
                    self.sendTelegram(hexStringToByteArray(response_str), 2)
                else:
                    self.sendTelegram(response_str, 2)
            elif len(received_telegram) > 0: # request not found in map
                print("## ERROR SopasTestServer: received unsupported telegram {}".format(received_telegram))

            time.sleep(0.01)
        
if __name__ == "__main__":

    # Configuration
    tcp_port = 2111 # tcp port to listen for tcp connections
    cola_binary = 0 # cola ascii (0) or binary (1)
    val_FREchoFilter = 0 # default configuration: FREchoFilter=0
    
    # Overwrite with command line arguments
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--tcp_port", help="tcp port to listen for tcp connections", default=tcp_port, type=int)
    arg_parser.add_argument("--cola_binary", help="cola ascii (0) or binary (1)", default=cola_binary, type=int)
    arg_parser.add_argument("--FREchoFilter", help="FREchoFilter, 0 (first echo, default), 1 (all echos) or 2 (last echo)", default=val_FREchoFilter, type=int)
    cli_args = arg_parser.parse_args()
    tcp_port = cli_args.tcp_port
    cola_binary = cli_args.cola_binary
    val_FREchoFilter = cli_args.FREchoFilter

    # Run test server
    server = SopasTestServer(tcp_port, cola_binary, val_FREchoFilter)
    server.connect()
    server.run()
    