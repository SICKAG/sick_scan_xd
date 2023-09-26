"""
    pcap_json_converter converts TiM781S pcap files to json

    Usage:

    pip install scapy
    pip install pypcapfile
    pip install python-pcapng

    python pcap_json_converter.py --pcap_filename=<filepath.pcapng>

"""

import argparse
import socket
import time
import os
import sys

from pcapng import FileScanner
from pcapng.blocks import EnhancedPacket #, InterfaceDescription, SectionHeader

import scapy.all
import scapy.packet
from scapy.layers.l2 import Ether

def isJsonPrintable(b):
    return (b == 0x20) or (b >= 48 and b <= 57) or (b >= 65 and b <= 90) or (b >= 97 and b <= 122)

def appendToJsonfile(json_filename, relative_timestamp, payload, pcap_filename):
    if os.path.isfile(json_filename):
        json_file = open(json_filename, "a")
        json_file.write(",\n")
    else:
        json_file = open(json_filename, "a")
        json_file.write("[\n")
    json_file.write("  {\n")
    json_file.write("    \"_source\": {\n")
    json_file.write("      \"layers\": {\n")
    json_file.write("        \"tcp\": {\n")
    json_file.write("          \"tcp.analysis\": {\n")
    json_file.write("            \"tcp.analysis.push_bytes_sent\": \"{}\"\n".format(len(payload)))
    json_file.write("          },\n")
    json_file.write("          \"Timestamps\": {\n")
    json_file.write("            \"tcp.time_relative\": \"{}\"\n".format(relative_timestamp))
    json_file.write("          },\n")
    json_file.write("          \"tcp.comment\": \"All entries created by pcap_json_converter, not by Wireshark.\",\n")
    json_file.write("          \"tcp.producer\": \"{} --pcap_filename={}\",\n".format(os.path.basename(__file__), os.path.basename(pcap_filename)))
    readable_values = [chr(b) if isJsonPrintable(b) else '.' for b in payload]
    description = "".join(readable_values)
    hex_values = [format(b,"02x") for b in payload]
    hex_payload = ":".join(hex_values)
    json_file.write("          \"tcp.description\": \"{}\",\n".format(description))
    json_file.write("          \"tcp.payload\": \"{}\"\n".format(hex_payload))
    json_file.write("        }\n")
    json_file.write("      }\n")
    json_file.write("    }\n")
    json_file.write("  }")
    json_file.close()

def closeJsonfile(json_filename):
    if os.path.isfile(json_filename):
        json_file = open(json_filename, "a")
        json_file.write("\n]\n")
        json_file.close()

def appendToCppfile(cpp_filename, payload, is_cola_ascii):
    if os.path.isfile(cpp_filename):
        cpp_file = open(cpp_filename, "a")
    else:
        cpp_file = open(cpp_filename, "a")
        cpp_file.write("    std::map<std::string, sick_scan_xd::SickLocColaTelegramMsg> emulator_responses = { // emulator responses to driver requests\n")
    payload_all = [b for b in payload]
    if is_cola_ascii: # cola ascii: remove 1 leading byte <STX> = 0x02 and remove trailing <ETX> = 0x03
        payload_unpacked = payload_all[1:-1]
    else: # cola binary: remove 8 byte 0x02020202 + { 4 byte length } and remove trailing CRC byte
        payload_unpacked = payload_all[8:-1]
    # Split command into <type> <name> <parameter>
    sep_idx1 = 0
    sep_idx2 = 0
    for n, b in enumerate(payload_unpacked):
        if b == 0x20:
            if sep_idx1 == 0:
                sep_idx1 = n # first space separates command type
            else:
                sep_idx2 = n # second space separates parameter
                break
    if sep_idx2 == 0: # no parameter if second space is missing
        sep_idx2 = len(payload_unpacked)
    cola_type = "".join([chr(b) for b in payload_unpacked[0:sep_idx1]])
    cola_name = "".join([chr(b) for b in payload_unpacked[sep_idx1+1:sep_idx2]])
    if sep_idx2+1 < len(payload_unpacked):
        cola_args = "".join([format(b,"02x") for b in payload_unpacked[sep_idx2+1:]])
    else:
        cola_args = ""
    # Write C++ emulator call
    cpp_file.write("      {{\"{}\", sick_scan_xd::ColaParser::createColaTelegram(sick_scan_xd::ColaParser::convertSopasCommand(\"{}\"), \"{}\", {{\"{}\"}})}},\n".format(cola_name, cola_type, cola_name, cola_args));
    cpp_file.close()

def closeCppfile(cpp_filename):
    if os.path.isfile(cpp_filename):
        cpp_file = open(cpp_filename, "a")
        cpp_file.write("    };\n")
        cpp_file.close()

# Decodes and returns the payload length of a cola message, i.e. returns message_payload_length in a message := { 4 byte STX 0x02020202 } +  { 4 byte message_payload_length } + { message_payload } + { 1 byte CRC }
def parseColaPayloadLength(payload):
    length = 0
    if len(payload) > 9 and payload.startswith(b'\x02\x02\x02\x02'):
        length = (payload[4] << 24) + (payload[5] << 16) + (payload[6] << 8) + (payload[7] << 0)
    return length

if __name__ == "__main__":

    pcap_filename = "example.pcapng"
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--pcap_filename", help="pcapng filepath", default=pcap_filename, type=str)
    cli_args = arg_parser.parse_args()
    pcap_filename = cli_args.pcap_filename
    print("pcap_json_converter {} started.".format(pcap_filename))
    
    # Read and parse pcap file, extract tcp raw data
    cpp_filename = pcap_filename + ".cpp"
    json_filename = pcap_filename + ".json"
    if os.path.isfile(cpp_filename):
        os.remove(cpp_filename)
    if os.path.isfile(json_filename):
        os.remove(json_filename)
    start_timestamp = -1.0
    payload = b''
    payload_completed = False
    cola_ascii = False # default: cola binary
    with open(pcap_filename, 'rb') as pcap_file:
        pcap_scanner = FileScanner(pcap_file)
        for block_cnt, block in enumerate(pcap_scanner):
            if isinstance(block, EnhancedPacket):
            
                # Decode a single pcap block
                if block.captured_len != block.packet_len:
                    print("## pcap_json_converter block {}: {} byte block truncated to {} bytes".format(block_cnt, block.packet_len, block.captured_len))
                block_data = Ether(block.packet_data)
                block_decoded = block_data
                for n in range(0,10):
                  if isinstance(block_decoded.payload, scapy.packet.Raw):
                      break                  
                  elif isinstance(block_decoded.payload, scapy.packet.Packet):
                      block_decoded = block_decoded.payload
                  else:
                      break
                
                # Write json file, if payload starts with 0x02020202 (i.e. payload is a lidar message)
                if start_timestamp < 0:
                    start_timestamp = block.timestamp
                if isinstance(block_decoded.payload, scapy.packet.Raw) and len(block_decoded.payload) > 0:                
                    payload_chunk = bytes(block_decoded.payload)
                    # Check start resp. continuation of binary messages
                    if payload_chunk.startswith(b'\x02\x02\x02\x02'): # start of a new binary message
                        relative_timestamp = block.timestamp - start_timestamp
                        payload = payload_chunk
                        payload_completed = (len(payload) >= (parseColaPayloadLength(payload) + 9)) # binary message := { 4 byte STX 0x02020202 } +  { 4 byte message_payload_length } + { message_payload } + { 1 byte CRC }
                    elif payload_completed == False and payload.startswith(b'\x02\x02\x02\x02'):    # binary message continued
                        payload = payload + payload_chunk
                        payload_completed = (len(payload) >= (parseColaPayloadLength(payload) + 9)) # message := { 4 byte STX 0x02020202 } +  { 4 byte message_payload_length } + { message_payload } + { 1 byte CRC }
                    # Check start resp. continuation of ascii messages
                    elif payload_chunk.startswith(b'\x02\x73'): # start of a new ascii message <STX>s...<ETX> with <STX>=\x02 and <ETX>=\x03
                        relative_timestamp = block.timestamp - start_timestamp
                        payload = payload_chunk
                        payload_completed = (payload.find(b'\x03') > 0)                     # ascii message ends with <ETX>=\x03
                    elif payload_completed == False and payload.startswith(b'\x02\x73'):    # ascii message continued
                        payload = payload + payload_chunk
                        payload_completed = (payload.find(b'\x03') > 0)                     # ascii message ends with <ETX>=\x03
                    # Write payload (binary or ascii messages)
                    is_cola_ascii = payload.startswith(b'\x02\x73')
                    is_cola_binary = payload.startswith(b'\x02\x02\x02\x02')
                    if payload_completed == True and (is_cola_binary or is_cola_ascii):
                        appendToCppfile(cpp_filename, payload, is_cola_ascii)
                        appendToJsonfile(json_filename, relative_timestamp, payload, pcap_filename)
                        print("block {}: rel_timestamp = {}, payload = {}".format(block_cnt, relative_timestamp, payload))
                        payload = b''
                        payload_completed = False

    closeCppfile(cpp_filename)
    closeJsonfile(json_filename)
    print("pcap_json_converter finished.")
