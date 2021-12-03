"""
    A simple test server for sick_scan_base. A listening tcp socket is opened, incoming connections are accepted and some basic cola telegrams are responded on client requests.
    Note: This is just a simple test server for basic unittests of sick_scan_base drivers. It does not emulate any real lidar sensor.

    Usage:
    python test_server.py --scandata_file=<scandatafile> --scandata_frequency=<freq> --tcp_port=<int>
    
    Example:
    python ../test/emulator/test_server.py --scandata_file=../test/emulator/scandata/20210302_lms511.pcapng.scandata.txt --scandata_frequency=2.0 --tcp_port=2112

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
    ColaLMDscandataGenerator generates LMDscandata from a list of predefined cola scandatatelegrams.
"""
class ColaLMDscandataGenerator:

    # Constructor
    def __init__(self, scandata_file):
        self.LMDscandata_list = []
        self.read_index = -1;
        with open(scandata_file, 'r') as f:
            self.LMDscandata_list = [line.strip() for line in f]
        print("ColaLMDscandataGenerator: {} scandata telegrams from file {}".format(len(self.LMDscandata_list), scandata_file))
        if len(self.LMDscandata_list) <= 0: # at least one hardcoded scandata
            self.LMDscandata_list = [ "02:02:02:02:00:00:0d:d8:73:53:4e:20:4c:4d:44:73:63:61:6e:64:61:74:61:20:00:00:00:01:01:14:58:56:01:00:d3:52:d4:d4:f4:75:09:51:f4:75:a8:e6:00:00:00:00:00:00:00:00:09:c4:00:00:02:1c:00:00:00:01:44:49:53:54:31:3f:80:00:00:00:00:00:00:ff:ff:3c:b0:06:83:04:75:05:19:05:13:05:1b:05:13:05:1b:05:1b:05:1b:05:19:05:1f:05:1f:05:22:05:23:05:21:05:22:05:1d:05:24:05:28:05:21:05:24:05:1e:05:28:05:24:05:27:05:28:05:26:05:28:05:2d:05:2e:05:30:05:32:05:2e:05:30:05:31:05:34:05:31:05:37:05:37:05:3f:05:3b:05:35:05:3d:05:37:05:40:05:3b:05:41:05:44:05:45:05:3d:05:45:05:49:05:43:05:47:05:48:05:4a:05:4d:05:4a:05:4c:05:4b:05:51:05:51:05:54:05:57:05:4f:05:57:05:56:05:59:05:56:05:58:05:59:05:62:05:5d:05:5f:05:64:05:65:05:68:05:67:05:63:05:6b:05:64:05:72:05:70:05:71:05:71:05:6f:05:74:05:70:05:74:05:73:05:7b:05:7f:05:7e:05:88:05:87:05:84:05:83:05:8a:05:81:05:8c:05:8c:05:8a:05:8f:05:95:05:93:05:9b:05:9a:05:9f:05:9a:05:9e:05:a4:05:a8:05:a0:05:a4:05:a8:05:b0:05:ad:05:af:05:b4:05:b2:05:bc:05:bc:05:ba:05:b9:05:bf:05:c1:05:c9:05:cb:05:ce:05:d1:05:cf:05:d4:05:d2:05:da:05:db:05:d9:05:db:05:de:05:e6:05:e7:05:e6:05:ea:05:f3:05:f3:05:f5:05:f8:05:f9:05:fb:06:00:06:08:05:fe:06:0d:06:0d:06:0d:06:15:06:10:06:14:06:18:06:25:06:24:06:26:06:24:06:20:06:1d:06:0f:06:12:05:fe:05:fc:05:fc:05:f7:05:ec:05:e4:05:e1:05:e0:05:db:05:d6:05:ce:05:c2:05:c0:05:b8:05:ae:05:aa:05:a9:05:a6:05:9b:05:a0:05:8b:05:91:05:8d:05:86:05:84:05:77:05:81:05:76:05:75:05:68:05:62:05:5b:05:54:05:51:05:4c:05:52:05:4d:05:46:05:40:05:39:05:35:05:34:05:32:05:2f:05:21:05:29:05:21:05:20:05:16:05:0c:05:0c:05:04:05:08:05:04:04:ff:04:f7:04:fb:04:f7:04:ec:04:e8:04:e9:04:f0:04:e7:04:e3:04:dc:04:d9:04:df:04:d7:04:ce:04:ca:04:c4:04:c2:04:b9:04:be:04:bd:04:b8:04:ae:04:aa:04:b2:04:ac:04:a8:04:9c:04:a4:04:a2:04:9f:04:9b:04:96:04:91:04:91:04:94:04:8c:04:8a:04:8a:04:80:04:80:04:7c:04:76:04:74:04:77:04:6f:04:68:04:72:04:6f:04:6a:04:63:04:64:04:62:04:5d:04:5f:04:5d:04:57:04:55:04:56:04:58:04:4c:04:50:04:4e:04:53:04:49:04:4f:04:47:04:40:04:3e:04:3d:04:39:04:3a:04:32:04:31:04:32:04:30:04:28:04:26:04:25:04:2c:04:23:04:22:04:20:04:22:04:18:04:21:04:18:04:16:04:18:04:1d:04:11:04:1a:04:15:04:16:04:14:04:11:04:13:04:0d:04:0c:04:02:03:fc:03:fd:04:04:03:f7:03:f7:03:f9:03:f2:03:f4:03:f2:03:f3:03:f6:03:f3:03:e7:03:f1:03:eb:03:ee:03:e6:03:ea:03:e1:03:de:03:e8:03:e8:03:e4:03:df:03:e4:03:dd:03:db:03:d9:03:dc:03:db:03:dc:03:d3:03:d3:03:d2:03:d2:03:ca:03:c0:03:c5:03:c3:03:c4:03:c6:03:bf:03:c2:03:c5:03:c0:03:bf:03:ba:03:b9:03:b4:03:b9:03:b5:03:b4:03:b6:03:b1:03:bc:03:b0:03:b1:03:b3:03:b2:03:b0:03:a8:03:ae:03:a9:03:ad:03:ae:03:a7:03:b9:03:a3:03:af:03:ab:03:a6:03:9e:03:99:03:9e:03:98:03:9d:03:98:03:95:03:9e:03:9a:03:94:03:93:03:96:03:95:03:95:03:94:03:97:03:90:03:91:03:8d:03:92:03:8d:03:92:03:88:03:86:03:8b:03:80:03:8b:03:89:03:88:03:83:03:86:03:84:03:86:03:88:03:7b:03:8a:03:84:03:84:03:84:03:7e:03:84:03:7d:03:71:03:79:03:77:03:6b:03:71:03:73:03:71:03:6e:03:6c:03:6c:03:4e:02:c1:02:61:02:2c:02:20:02:00:01:fc:01:ec:01:e9:01:e6:01:e3:01:e2:01:e4:01:de:01:da:01:de:01:dd:01:ce:01:d7:01:df:01:d5:01:db:01:d4:01:dc:01:de:01:d7:01:d5:01:d7:01:d0:01:d3:01:d0:01:d5:01:d1:01:c9:01:d2:01:d1:01:cf:01:ce:01:ce:01:cf:01:c3:01:cc:01:c8:01:c5:01:cb:01:d1:01:c7:01:c3:01:c8:01:c4:01:ca:01:bc:01:c2:01:c3:01:c4:01:bd:01:bd:01:c1:01:c0:01:cc:01:c0:01:c0:01:bc:01:b6:01:bb:01:b9:01:c0:01:bc:01:bd:01:b9:01:b8:01:bb:01:b4:01:bb:01:be:01:b5:01:b8:01:b5:01:b6:01:b4:01:b5:01:b3:01:ba:01:b6:01:bb:01:af:01:b3:01:b0:01:b9:01:b2:01:b2:01:b2:01:ad:01:ad:01:ad:01:ae:01:b7:01:ac:01:ac:01:ab:01:b9:01:b0:01:b8:01:bf:01:c7:01:c8:01:db:02:09:02:30:02:58:02:a9:03:28:03:60:03:6a:03:70:03:6b:03:6c:03:6d:03:72:03:79:03:7e:03:80:03:85:03:80:03:84:03:8e:03:87:03:81:03:8d:03:91:03:92:03:9f:03:a1:03:97:03:a7:03:a5:03:9e:03:9e:03:ab:03:a2:03:b0:03:ac:03:ad:03:b1:03:c0:03:d8:03:e4:03:da:03:ce:03:ae:03:9d:03:9c:03:9e:03:a2:03:9a:03:a6:03:a8:03:ab:03:a9:03:af:03:a8:03:ad:03:af:03:b0:03:b3:03:ae:03:b9:03:be:03:c1:03:c5:03:cb:03:cb:03:c6:03:c8:03:d0:03:d1:03:cf:03:d5:03:dc:03:df:03:df:03:df:03:e7:03:f6:04:05:04:00:03:f3:03:e5:03:d8:03:cf:03:ca:03:cc:03:d2:03:d0:03:d1:03:d3:03:d8:03:de:03:e0:03:da:03:ec:03:e5:03:e9:03:ed:03:f2:03:ea:03:ee:03:fc:03:f5:03:fc:03:f8:03:fb:03:ff:03:ff:04:0c:04:07:04:0a:04:11:04:18:04:21:04:32:04:28:04:1a:04:10:04:06:04:03:04:09:04:0a:04:08:04:0e:04:16:04:17:04:19:04:1b:04:17:04:1a:04:1e:04:21:04:29:04:2a:04:24:04:39:04:2f:04:3b:04:32:04:39:04:39:04:3f:04:4d:04:43:04:40:04:47:04:42:04:41:04:40:04:49:04:4b:04:4d:04:4d:04:4c:04:4d:04:5a:04:52:04:57:04:59:04:5d:04:64:04:6d:04:67:04:67:04:70:04:73:04:71:04:6d:04:77:04:76:04:7e:04:86:04:84:04:86:04:81:04:88:04:88:04:88:04:85:04:89:04:8f:04:92:04:8f:04:94:04:99:04:a1:04:a7:04:a3:04:a7:04:ac:04:b2:04:af:04:be:04:c1:04:be:04:c8:04:cc:04:cc:04:d1:04:d2:04:d4:04:cd:04:d0:04:d2:04:d4:04:e1:04:e5:04:e8:04:e8:04:f1:04:fb:04:fa:04:fb:05:01:05:01:05:02:05:0b:05:12:05:1d:05:1c:05:1a:05:1c:05:1c:05:17:05:23:05:23:05:2c:05:36:05:38:05:38:05:40:05:4a:05:47:05:48:05:52:05:53:05:5a:05:5a:05:7c:05:87:05:ad:05:d9:06:45:06:8b:06:c8:06:d0:06:85:06:52:06:0b:05:d9:05:b9:05:aa:05:a6:05:c1:06:0c:06:cb:06:f6:07:28:07:2f:07:3a:07:46:07:43:07:13:06:c9:06:6e:06:39:06:1c:06:2b:06:6b:06:f8:07:50:07:47:06:e8:06:81:06:5b:06:4b:06:41:06:4e:06:59:06:7b:06:9a:06:c1:06:ff:07:8e:07:d5:07:ff:07:88:07:33:06:e6:06:c6:06:cf:06:e0:07:18:07:64:08:0d:08:58:08:76:08:7d:08:88:08:91:08:47:08:1a:07:ef:07:ed:08:48:08:cf:09:21:09:71:09:84:09:20:08:4a:07:e8:07:ad:07:9f:07:93:07:ab:07:fe:09:1c:09:38:08:a9:08:46:08:13:07:ee:07:d5:07:dd:07:e7:08:12:08:84:08:9a:08:7b:08:57:08:44:08:25:08:31:08:3f:08:56:08:90:08:c3:08:b4:08:9f:08:8a:08:7d:08:80:08:90:08:b5:08:f5:09:04:08:f5:08:eb:08:ec:08:f5:09:12:09:47:09:64:09:5b:09:52:09:5b:09:75:09:a2:09:c2:09:c0:09:c3:09:cb:09:e9:0a:2b:0a:3b:0a:50:0a:98:0a:b7:0a:c1:0a:eb:0b:09:0b:03:0b:00:0b:1b:0b:50:0b:60:0b:4f:0a:ed:0a:6f:0a:30:0a:21:0a:10:0a:14:0a:11:0a:07:0a:05:0a:07:09:f4:09:d6:09:ba:09:9c:09:91:09:81:09:7a:09:7b:09:7c:09:78:09:75:09:76:09:75:09:70:09:70:09:6d:09:6e:09:69:09:69:09:62:09:68:09:63:09:61:09:62:09:61:09:51:09:58:09:59:09:55:09:54:09:51:09:4c:09:50:09:42:09:47:09:41:09:45:09:43:09:41:09:45:09:42:09:45:09:38:09:3a:09:3d:09:3c:09:31:09:3a:09:47:09:4b:09:54:09:7d:09:7f:09:78:09:6d:09:61:09:60:09:52:09:59:09:62:09:6b:09:71:09:6f:09:6e:09:6f:09:6d:09:6e:09:6c:09:69:09:6b:09:66:09:62:09:6a:09:6a:09:67:09:6b:09:6a:09:62:09:64:09:69:09:63:09:61:09:5e:09:6a:09:5f:09:61:09:5f:09:5f:09:61:09:67:09:64:09:63:09:5e:09:66:09:5d:09:61:09:66:09:60:09:5f:09:5c:09:62:09:63:09:66:09:67:09:6b:09:6f:09:75:09:7f:03:ee:03:ba:03:56:03:43:03:2c:03:25:03:1c:03:1f:03:16:03:18:03:11:03:0f:03:00:03:03:02:fb:02:fb:02:fe:02:f3:02:ee:02:ec:02:ec:02:e3:02:e2:02:d9:02:db:02:d9:02:d6:02:d2:02:cf:02:c8:02:ca:02:c3:02:b9:02:be:02:b8:02:b3:02:b6:02:b2:02:ae:02:ab:02:a6:02:a3:02:a0:02:97:02:93:02:90:02:98:02:8b:02:8a:02:84:02:84:02:82:02:86:02:7f:02:83:02:7b:02:77:02:79:02:77:02:6f:02:77:02:69:02:70:02:6d:02:71:02:68:02:66:02:62:02:5d:02:5f:02:60:02:68:02:5d:02:5a:02:55:02:5c:02:56:02:5a:02:57:02:5c:02:59:02:61:02:57:02:5d:02:5e:02:62:02:68:02:65:02:62:02:68:02:6b:02:69:02:6a:02:6a:02:6c:02:72:02:6f:02:70:00:01:52:53:53:49:31:3f:80:00:00:00:00:00:00:ff:ff:3c:b0:06:83:04:75:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fc:ef:e9:f0:f6:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:f8:f4:f1:e3:e4:e6:e5:e4:e2:e4:e4:e5:ef:e3:e8:ed:e7:e4:e7:e5:e6:e3:e6:df:e6:e5:e6:e4:e0:e3:e6:ec:e2:e6:e7:e5:e3:e5:e3:e2:e3:de:e4:dd:e7:e2:e1:e3:e4:e0:de:e2:e1:e5:e4:e7:e6:e2:df:e3:e2:e1:e0:e1:e7:e2:e2:dd:e3:e8:e3:e0:e2:e0:e4:e1:e8:e6:e0:db:dd:e4:e1:e5:de:e5:e3:e1:e2:e4:e0:e0:e0:de:e0:e1:df:e3:e8:e7:e2:e2:e1:e0:e2:e1:dc:db:df:df:e0:e4:e6:e0:e1:e1:e3:dd:e5:dd:e7:e5:e7:e2:e0:e1:e5:e6:e3:e1:e6:e2:e2:df:df:df:e4:e2:e5:df:e6:e3:e3:e3:e6:e2:e6:e4:e7:e6:e4:e7:e5:e6:e3:e9:eb:e5:e9:ea:e8:e6:e8:eb:e6:eb:e6:e9:e8:e1:e8:e6:e5:e5:e4:e2:e8:ea:e7:e5:e7:e8:ea:ea:e4:eb:e4:ea:e6:eb:e9:ee:eb:eb:ec:e8:e9:ea:ea:e8:eb:eb:e9:e9:e9:f0:eb:ef:ec:f0:ed:f1:ea:ed:eb:ee:ed:ea:ee:ee:ec:ec:ed:f0:ec:ef:f2:f4:ed:ef:eb:ef:f1:f2:ea:f2:f0:ec:e8:f2:ea:f3:ec:ef:f1:ee:ed:ef:ef:f2:ef:ef:ec:ed:f0:f4:ec:ef:f5:f0:ee:f3:ee:f0:ef:f7:f3:f5:f3:f1:f5:f9:f7:f6:f4:f6:f8:f5:f0:ec:f2:fe:fe:fe:fe:fe:fd:eb:e8:ed:ec:e9:e8:e2:e4:ea:e6:e2:ea:ef:e9:ee:e6:ee:ed:e9:e7:ed:eb:eb:e8:ed:ee:e9:eb:ee:eb:e8:ed:ed:e8:eb:ea:e2:e5:e4:e6:e3:e5:e4:e3:e1:e9:eb:e9:e9:e6:e7:ec:e6:e2:e6:e3:e3:e4:e7:eb:e1:e4:e2:e4:e8:e7:e6:ed:e7:e8:ec:e6:e3:e5:e4:e6:e5:e3:e4:e6:eb:e7:ec:eb:e8:e7:e5:e6:eb:ea:e6:db:d1:c7:bb:b4:b7:b6:b5:c6:ec:fe:fe:fa:e7:d9:d9:e5:e0:e2:e1:e5:e2:e8:e4:e1:e2:e3:de:de:db:de:de:dc:e2:dd:df:df:e0:d9:dd:e0:e0:e0:e0:db:de:eb:fe:fe:fe:fe:fb:e8:e5:dd:df:db:e2:dd:dc:dd:db:dc:e1:d5:d8:d8:d4:db:db:d9:d9:df:db:db:db:da:de:d6:db:d9:e1:d8:da:e0:ec:f7:fe:fe:fe:ef:e0:da:dd:dc:db:d7:da:dd:dc:dd:d8:dd:d8:db:dd:da:d8:d8:e0:d8:dd:da:dc:d5:d8:dc:dc:de:e4:e8:f4:fe:fe:fd:f4:e0:dc:da:de:db:db:dc:d9:da:dd:d7:db:d6:db:d9:dc:d9:dd:d8:df:d9:dd:da:de:e1:e2:e5:e9:e3:e8:df:e6:e5:df:db:dc:da:da:d9:df:db:da:db:da:da:da:d9:dd:d9:d5:dc:e0:e0:e4:e3:e6:e1:e4:ea:e5:e1:df:dd:dc:d8:db:dd:db:df:d7:db:dc:d7:d7:dd:e0:dc:e0:de:e3:e3:ea:e8:e4:e1:df:db:dd:df:dd:da:d8:da:df:e3:dd:db:da:df:e1:e4:e8:e7:e5:e9:e2:e6:dc:db:df:e0:da:e0:e4:e4:dc:df:df:e2:d9:f7:fe:fe:fe:fe:fe:fe:f8:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fc:fd:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:f2:e9:e7:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:f7:fa:f4:f5:fa:fe:fe:fe:fe:fe:fe:fe:fc:fe:fe:fe:fe:fe:fe:fe:fe:fe:f5:ed:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:f1:ea:fe:fe:fe:fe:fe:f4:eb:ed:e8:f4:f9:fb:f1:ea:e5:e9:ef:f8:f7:fa:eb:ec:ed:f5:fb:f7:e6:e8:ee:f5:f6:f5:f6:fb:fe:fe:fe:fe:ea:e9:ed:ef:e7:df:d5:d6:bc:be:cf:d4:e2:e8:ec:f1:ef:f7:f4:ef:eb:e6:e2:e5:e6:e9:e9:e9:e9:e9:e8:e7:e7:ea:ec:e8:e6:ed:e4:e8:e9:e8:e5:ea:f1:ec:ea:eb:e8:ed:e5:f0:eb:ea:ea:e6:ec:ea:e9:e9:e7:f2:ec:e8:dd:e3:e2:d4:c7:d8:eb:ed:ee:f7:f6:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:fe:f0:d8:fe:fe:c2:c4:bd:bf:be:bf:be:bf:be:c1:bc:c1:bd:be:c1:bf:bf:be:c5:bf:c3:c6:c5:c2:c4:c5:c6:c5:c8:c6:bf:c3:c1:c4:c5:c7:c2:c7:c9:c5:c4:c5:c2:c6:c9:c7:c3:c2:c4:c5:c5:c3:c4:c8:c3:c3:c3:c6:c6:c4:c8:c6:c7:c3:cb:c7:c8:c5:c7:c8:c3:c3:c7:ca:cc:cf:d5:d4:d1:d9:d0:d0:d2:d2:d2:d0:d1:d3:d3:d4:d2:d2:d0:d3:ce:d0:00:00:00:00:00:00:00:01:07:b2:01:01:03:1f:1e:00:0e:29:00:00:00:45" ]

    def getNextScandata(self):
        self.read_index = self.read_index + 1;
        if self.read_index >= len(self.LMDscandata_list):
            self.read_index = 0
        return self.LMDscandata_list[self.read_index]


"""
    ColaResponseMap maps a cola request like "sMN SetAccessMode" to a binary cola response.
"""
class ColaResponseMap:

    # Constructor
    def __init__(self):
        self.mapped_response = { 
            "sMN SetAccessMode": "02:02:02:02:00:00:00:13:73:41:4e:20:53:65:74:41:63:63:65:73:73:4d:6f:64:65:20:01:38" ,
            "sWN EIHstCola": "02:02:02:02:00:00:00:0e:73:57:41:20:45:49:48:73:74:43:6f:6c:61:20:07",
            "sRN FirmwareVersion": "02:02:02:02:00:00:00:1f:73:52:41:20:46:69:72:6d:77:61:72:65:56:65:72:73:69:6f:6e:20:00:09:56:31:2e:38:30:20:20:20:20:43",
            "sRN SCdevicestate": "02:02:02:02:00:00:00:13:73:52:41:20:53:43:64:65:76:69:63:65:73:74:61:74:65:20:00:1f",
            "sRN ODoprh": "02:02:02:02:00:00:00:0f:73:52:41:20:4f:44:6f:70:72:68:20:00:00:04:ab:c1",
            "sRN ODpwrc": "02:02:02:02:00:00:00:0f:73:52:41:20:4f:44:70:77:72:63:20:00:00:00:63:1e",
            "sRN LocationName": "02:02:02:02:00:00:00:1e:73:52:41:20:4c:6f:63:61:74:69:6f:6e:4e:61:6d:65:20:00:0b:53:4e:20:31:38:31:31:30:35:35:30:45",
            "sRN LMPoutputRange": "02:02:02:02:00:00:00:21:73:52:41:20:4c:4d:50:6f:75:74:70:75:74:52:61:6e:67:65:20:00:01:00:00:06:83:ff:ff:3c:b0:00:1c:3a:90:cf",
            "sWN LMPoutputRange": "02:02:02:02:00:00:00:13:73:57:41:20:4c:4d:50:6f:75:74:70:75:74:52:61:6e:67:65:20:74",
            "sRN field000": "02:02:02:02:00:00:00:4c:73:52:41:20:66:69:65:6c:64:30:30:30:20:40:00:00:00:00:00:00:00:00:00:06:83:ff:ff:3c:b0:02:01:00:01:00:03:01:2c:ff:ff:01:62:01:d2:ff:ff:01:a3:01:e6:ff:ff:00:ce:00:00:00:00:00:00:00:01:00:0b:73:65:67:6d:65:6e:74:65:64:5f:31:00:00:6a",
            "sRN field001": "02:02:02:02:00:00:00:54:73:52:41:20:66:69:65:6c:64:30:30:31:20:40:00:00:00:00:00:00:00:00:00:06:83:ff:ff:3c:b0:01:02:00:00:00:01:00:0d:bb:a0:00:7d:00:00:00:00:00:00:00:c8:00:00:00:c8:00:00:00:00:00:01:00:15:72:65:63:74:61:6e:67:6c:65:5f:66:69:65:6c:64:5f:30:5f:64:65:67:00:00:0d",
            "sRN field002": "02:02:02:02:00:00:00:4f:73:52:41:20:66:69:65:6c:64:30:30:32:20:40:00:00:00:00:00:00:00:00:00:06:83:ff:ff:3c:b0:01:03:00:00:00:01:00:0f:75:6b:00:7f:00:06:dd:d0:00:00:00:c8:00:00:00:c8:00:00:00:00:00:01:00:10:72:65:63:74:66:69:65:6c:64:5f:34:35:5f:64:65:67:00:00:0b",
            "sRN field003": "02:02:02:02:00:00:00:52:73:52:41:20:66:69:65:6c:64:30:30:33:20:40:00:00:00:00:00:00:00:00:00:06:83:ff:ff:3c:b0:03:04:00:00:00:00:00:00:00:01:00:1b:77:40:00:fa:00:00:00:00:00:00:03:e8:00:00:01:2c:09:60:00:00:05:dc:00:01:00:0d:64:79:6e:61:6d:69:63:5f:66:69:65:6c:64:00:00:9a",
            "sRN field004": "02:02:02:02:00:00:00:2d:73:52:41:20:66:69:65:6c:64:30:30:34:20:40:00:00:00:00:00:00:00:00:00:13:88:ff:ff:3c:b0:02:00:00:00:00:00:00:00:00:00:00:01:00:00:00:00:62",
            "sRN field005": "02:02:02:02:00:00:00:2d:73:52:41:20:66:69:65:6c:64:30:30:35:20:40:00:00:00:00:00:00:00:00:00:13:88:ff:ff:3c:b0:02:00:00:00:00:00:00:00:00:00:00:01:00:00:00:00:63",
            "sRN field006": "02:02:02:02:00:00:00:2d:73:52:41:20:66:69:65:6c:64:30:30:36:20:40:00:00:00:00:00:00:00:00:00:13:88:ff:ff:3c:b0:02:00:00:00:00:00:00:00:00:00:00:01:00:00:00:00:60",
            "sRN field007": "02:02:02:02:00:00:00:2d:73:52:41:20:66:69:65:6c:64:30:30:37:20:40:00:00:00:00:00:00:00:00:00:13:88:ff:ff:3c:b0:02:00:00:00:00:00:00:00:00:00:00:01:00:00:00:00:61",
            "sRN field008": "02:02:02:02:00:00:00:2d:73:52:41:20:66:69:65:6c:64:30:30:38:20:40:00:00:00:00:00:00:00:00:00:13:88:ff:ff:3c:b0:02:00:00:00:00:00:00:00:00:00:00:01:00:00:00:00:6e",
            "sRN field009": "02:02:02:02:00:00:00:2d:73:52:41:20:66:69:65:6c:64:30:30:39:20:40:00:00:00:00:00:00:00:00:00:13:88:ff:ff:3c:b0:02:00:00:00:00:00:00:00:00:00:00:01:00:00:00:00:6f",
            "sRN field010": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:31:30:20:00:33",
            "sRN field011": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:31:31:20:00:32",
            "sRN field012": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:31:32:20:00:31",
            "sRN field013": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:31:33:20:00:30",
            "sRN field014": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:31:34:20:00:37",
            "sRN field015": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:31:35:20:00:36",
            "sRN field016": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:31:36:20:00:35",
            "sRN field017": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:31:37:20:00:34",
            "sRN field018": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:31:38:20:00:3b",
            "sRN field019": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:31:39:20:00:3a",
            "sRN field020": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:32:30:20:00:30",
            "sRN field021": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:32:31:20:00:31",
            "sRN field022": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:32:32:20:00:32",
            "sRN field023": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:32:33:20:00:33",
            "sRN field024": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:32:34:20:00:34",
            "sRN field025": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:32:35:20:00:35",
            "sRN field026": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:32:36:20:00:36",
            "sRN field027": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:32:37:20:00:37",
            "sRN field028": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:32:38:20:00:38",
            "sRN field029": "02:02:02:02:00:00:00:0e:73:52:41:20:66:69:65:6c:64:30:32:39:20:00:39",
            "sRN LIDinputstate": "02:02:02:02:00:00:00:29:73:52:41:20:4c:49:44:69:6e:70:75:74:73:74:61:74:65:20:00:00:00:29:74:2d:00:00:00:00:00:01:07:b2:01:01:00:00:02:00:07:99:50:28",
            "sWN LMDscandatacfg": "02:02:02:02:00:00:00:13:73:57:41:20:4c:4d:44:73:63:61:6e:64:61:74:61:63:66:67:20:4d",
            "sRN LMDscandatacfg": "02:02:02:02:00:00:00:20:73:52:41:20:4c:4d:44:73:63:61:6e:64:61:74:61:63:66:67:20:01:00:01:00:00:00:00:00:00:00:01:00:01:48",
            "sMN LMCstartmea": "02:02:02:02:00:00:00:12:73:41:4e:20:4c:4d:43:73:74:61:72:74:6d:65:61:73:20:00:44",
            "sMN Run": "02:02:02:02:00:00:00:09:73:41:4e:20:52:75:6e:20:01:34",
            "sEN LMDscandata": "02:02:02:02:00:00:00:11:73:45:41:20:4c:4d:44:73:63:61:6e:64:61:74:61:20:01:3c",
            "sEN LFErec": "02:02:02:02:00:00:00:0c:73:45:41:20:4c:46:45:72:65:63:20:01:4d",
            "sEN LIDoutputstate": "02:02:02:02:00:00:00:14:73:45:41:20:4c:49:44:6f:75:74:70:75:74:73:74:61:74:65:20:01:5f",
            "sEN LIDinputstate": "02:02:02:02:00:00:00:13:73:45:41:20:4c:49:44:69:6e:70:75:74:73:74:61:74:65:20:01:36",
            "sWN FREchoFilter": "02:02:02:02:00:00:00:10:73:57:41:20:46:52:45:63:68:6f:46:69:6c:74:65:72:50"
        }

    # Search for a mapped response given a cola request and returns key and response as strings
    def findBinaryResponse(self, request):
        for key, response in self.mapped_response.items():
            if request.find(stringToByteArray(key)) >= 0:
                return key, response
        return "", ""

"""
    TestServer connects to a tcp client, receives binary cola telegrams and sends a response to the client.
"""
class TestServer:

    # Constructor
    def __init__(self, tcp_port = 2112):
        self.tcp_port = tcp_port

    # Waits for an incoming tcp connection and connects to the tcp client
    def connect(self):
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.serversocket.bind(("", self.tcp_port))
        print("TestServer: listening on tcp port {}".format(self.tcp_port))
        self.serversocket.listen(1)
        (clientsocket, clientaddress) = self.serversocket.accept()
        self.clientsocket = clientsocket
        self.clientaddress = clientaddress
        self.clientsocket.setblocking(0)
        print("TestServer: tcp connection to {} established".format(self.clientaddress))

    # Receives a binary cola telegram and returns its payload (i.e. the telegram without header and CRC)
    def receiveBinaryTelegram(self, recv_timeout_sec):
        ready_to_recv = select.select([self.clientsocket], [], [], recv_timeout_sec)
        if not ready_to_recv[0]:
            return ""
        header = self.clientsocket.recv(8)
        payload_length = parseBinaryColaPayloadLength(header) # message := { 4 byte STX 0x02020202 } +  { 4 byte message_payload_length } + { message_payload } + { 1 byte CRC }
        if payload_length <= 1:
            print("## ERROR TestServer.receiveBinaryTelegram(): unexpected payload_length {} in received header {}".format(payload_length, header))
            return header
        payload = bytearray(b"")
        while len(payload) < payload_length:
            chunk = self.clientsocket.recv(payload_length - len(payload))
            payload = payload + chunk
        crc = bytearray(b"")
        while len(crc) < 1:
            crc = self.clientsocket.recv(1)
        print("TestServer.receiveBinaryTelegram(): received {} byte telegram {}".format((len(header) + len(payload) + 1), payload))
        return payload

    # Sends a binary cola telegram "as is"
    def sendBinaryTelegram(self, telegram, verbosity):
        if verbosity > 1:
            print("TestServer.sendBinaryTelegram(): sending {} byte telegram {}".format((len(telegram)), telegram))
        elif verbosity > 0:
            print("TestServer.sendBinaryTelegram(): sending {} byte telegram".format(len(telegram)))
        self.clientsocket.send(telegram)

    # Runs the message loop, i.e. receives binary cola telegrams and sends a response to the client
    def run(self, send_scandata_frequency, scandata_file):
        response_map = ColaResponseMap()
        scandata_generator = ColaLMDscandataGenerator(scandata_file)
        receive_timeout = max(0.001, 1.0 / send_scandata_frequency)
        time_to_send_scandata = datetime.datetime.now() + datetime.timedelta(seconds=3600*24*364)
        print("TestServer: running main loop...")
        while True:
            # Receive a cola telegram
            received_telegram = self.receiveBinaryTelegram(receive_timeout)
            if len(received_telegram) <= 0: # timeout (no message rececived)
                if datetime.datetime.now() >= time_to_send_scandata: # send LMDscandata
                    scandata_str = scandata_generator.getNextScandata()
                    self.sendBinaryTelegram(hexStringToByteArray(scandata_str), 1)
                    time_to_send_scandata = datetime.datetime.now() + datetime.timedelta(seconds=receive_timeout)
                continue;
            # Lookup for keywords and send a response from predefined (mapped) responses
            request_key, response_str = response_map.findBinaryResponse(received_telegram)
            if len(request_key) > 0 and len(response_str) > 0: # request and response found in map, send response to client
                print("TestServer: received {}: {}".format(request_key, received_telegram))
                self.sendBinaryTelegram(hexStringToByteArray(response_str), 2)
                if request_key.find("sMN Run") >= 0: # start to send LMDscandata in 2 seconds
                    time_to_send_scandata = datetime.datetime.now() + datetime.timedelta(seconds=2)
            elif len(received_telegram) > 0: # request not found in map
                print("## ERROR TestServer: received unsupported telegram {}".format(received_telegram))

            time.sleep(0.01)
        
if __name__ == "__main__":

    # Configuration
    tcp_port = 2112 # tcp port to listen for tcp connections
    send_data_frequency = 2.0 # send LMDscandata with 2 Hz
    scandata_file = "../test/emulator/scandata/20210302_lms511.pcapng.scandata.txt" # read LMDscandata from file
    
    # Overwrite with command line arguments
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--scandata_file", help="scandata filepath", default=scandata_file, type=str)
    arg_parser.add_argument("--scandata_frequency", help="frequency in Hz to send scandata telegrams", default=send_data_frequency, type=float)
    arg_parser.add_argument("--tcp_port", help="tcp port to listen for tcp connections", default=tcp_port, type=int)
    cli_args = arg_parser.parse_args()
    scandata_file = cli_args.scandata_file
    send_data_frequency = cli_args.scandata_frequency
    tcp_port = cli_args.tcp_port

    # Run test server
    server = TestServer(tcp_port)
    server.connect()
    server.run(send_data_frequency, scandata_file)
    
