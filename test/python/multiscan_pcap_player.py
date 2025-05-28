"""
    multiScan test emulator to parse pcapng files recorded from multiScan and replay the UDP packages
    to emulate a local multiScan lidar.

    The UDP Sender sends packets via UDP over localhost, Port:2115

    Usage:

    pip install scapy
    pip install pypcapfile
    pip install python-pcapng

"""

import argparse
import json
import socket
import sys
import time

pcapng_supported = False
try:
    from pcapng import FileScanner
    from pcapng.blocks import EnhancedPacket #, InterfaceDescription, SectionHeader
    import scapy.all
    import scapy.packet
    from scapy.layers.l2 import Ether
    pcapng_supported = True
except ModuleNotFoundError:
    print("import pcapng or scapy failed, pcapng-files not supported")

# Force a delay by active polling. Brute-force alternative if timing in time.sleep is not accurate enough
def forced_delay(seconds):
    timestamp_end = time.perf_counter() + seconds
    while time.perf_counter() < timestamp_end:
        pass

# Returns the payload starting from STX = '\x02\x02\x02\x02' (or unmodified payload if STX not found)
def extractMessageStart(payload):
    stx_index = payload.find(b'\x02\x02\x02\x02')
    if stx_index > 0:
        payload = payload[stx_index:]
    return payload

# Filter pcap blocks by src ip, dst ip and port: send a pcap block if its src ip, dst ip and port is found in these lists (or all if this list empty), default: empty
class PcapFilter:
    def __init__(self, src_ip = [], dst_ip = [], port = [], proto = []):
        self.src_ip = src_ip  # optional filter pcap blocks by src ip: send a pcap block if its src ip is found in this list (or all if this list empty), default: empty
        self.dst_ip = dst_ip  # optional filter pcap blocks by dst ip: send a pcap block if its dst ip is found in this list (or all if this list empty), default: empty
        self.port = port      # optional filter pcap blocks by ip port: send a pcap block if its ip port is found in this list (or all if this list empty), default: empty
        self.proto = proto    # optional filter pcap blocks by protocoll ("TCP", UDP. or "IP"): send a pcap block if its protocol name is found in this list (or all if this list empty), default: empty

# Container for decoded pcap blocks, containing the payload, timestamp, ip port, src ip and dst ip
class PcapDecodedBlock:
    def __init__(self, rawblock = None, timestamp = 0.0):
        self.timestamp = timestamp
        self.payload = bytes()
        self.dst_ip = ""
        self.src_ip = ""
        self.dst_port = 0
        self.proto = ""
        if rawblock is not None:
            self.payload = bytes(rawblock.payload)
            self.dst_ip = rawblock.underlayer.dst
            self.src_ip = rawblock.underlayer.src
            self.proto = rawblock.name
            if "dport" in rawblock.underlayer.payload.fields:
                self.dst_port = rawblock.underlayer.payload.fields["dport"]
            if rawblock.name == "IP":
                self.dst_ip = rawblock.dst
                self.src_ip = rawblock.src
    def print(self):
        payload_start_hex_str = "".join("\\x{:02x}".format(payload_byte) for payload_byte in self.payload[:4])
        return "{} byte payload {}..., timestamp={}, src_ip={}, dst_ip={}, port={}".format(len(self.payload), payload_start_hex_str, self.timestamp, self.src_ip, self.dst_ip, self.dst_port)

def readPcapngFile(pcap_filename, pcap_filter, verbose):
    pcap_decoded_blocks = []
    payload_length_accumulated_since_stx = 0
    if not pcapng_supported:
        print("## ERROR readPcapngFile(): import pcapng or scapy failed, pcapng-file {} is not supported".format(pcap_filename))
        return pcap_decoded_blocks
    with open(pcap_filename, 'rb') as pcap_file:
        pcap_scanner = FileScanner(pcap_file)
        for block_cnt, block in enumerate(pcap_scanner):
            if isinstance(block, EnhancedPacket):
                # Decode a single pcap block
                if block.captured_len != block.packet_len:
                    print("## multiscan_pcap_player block {}: {} byte block truncated to {} bytes".format(block_cnt, block.packet_len, block.captured_len))
                block_data = Ether(block.packet_data)
                block_decoded = block_data
                for n in range(0,10):
                  if isinstance(block_decoded.payload, scapy.packet.Raw):
                      break                  
                  elif isinstance(block_decoded.payload, scapy.packet.Packet):
                      block_decoded = block_decoded.payload
                  else:
                      break                    
                # if len(block_decoded.payload) <= 0:
                #     print("## multiscan_pcap_player block {}: empty payload ignored in data {}".format(block_cnt, block_data))
                # if not isinstance(block_decoded.payload, scapy.packet.Raw):
                #     print("## multiscan_pcap_player block {}: block_decoded.payload = {} is no instance of scapy.packet.Raw".format(block_cnt, block_decoded.payload))
                # print("block {}: timestamp = {}".format(block_cnt, block.timestamp))
                # print("block {}: packet_data = {}".format(block_cnt, block.packet_data))
                # print("block {}: payload = {}".format(block_cnt, block_decoded.payload))
                
                # Decode payload
                if isinstance(block_decoded.payload, scapy.packet.Raw) and len(block_decoded.payload) > 0:
                    pcap_decoded_block = PcapDecodedBlock(block_decoded, block.timestamp)
                    if len(pcap_filter.src_ip) > 0 and pcap_decoded_block.src_ip not in pcap_filter.src_ip:
                        continue # wrong source ip
                    if len(pcap_filter.dst_ip) > 0 and pcap_decoded_block.dst_ip not in pcap_filter.dst_ip:
                        continue # wrong destination ip
                    if pcap_decoded_block.dst_port > 0 and len(pcap_filter.port) > 0 and pcap_decoded_block.dst_port not in pcap_filter.port:
                        continue # wrong port
                    if len(pcap_filter.proto) > 0 and pcap_decoded_block.proto not in pcap_filter.proto:
                        continue; # wrong protocol
                    if pcap_decoded_block.payload.find(b'\x02\x02\x02\x02') >= 0:
                        payload_length_accumulated_since_stx = 0
                    payload_length_accumulated_since_stx = payload_length_accumulated_since_stx + len(pcap_decoded_block.payload)
                    if verbose > 0:
                        print("pcap block {}: {}, {} byte since stx".format(block_cnt, pcap_decoded_block.print(), payload_length_accumulated_since_stx))
                    if len(pcap_decoded_block.payload) < 64000:
                        pcap_decoded_block.payload = extractMessageStart(pcap_decoded_block.payload)
                        pcap_decoded_blocks.append(pcap_decoded_block)
    return pcap_decoded_blocks

def readJsonFile(json_filename, verbose):
    pcap_blocks = []
    with open(json_filename, "r") as file_stream:
        json_blocks = json.load(file_stream)
        for json_block in json_blocks:
            if len(json_block) == 2:
                pcap_block = PcapDecodedBlock()
                pcap_block.timestamp = json_block[0]
                pcap_block.payload = bytes.fromhex(json_block[1])
                pcap_blocks.append(pcap_block)
            elif len(json_block) == 3:
                pcap_block = PcapDecodedBlock()
                pcap_block.timestamp = json_block[0]
                pcap_block.dst_port = json_block[1]
                pcap_block.payload = bytes.fromhex(json_block[2])
                pcap_blocks.append(pcap_block)
    return pcap_blocks

if __name__ == "__main__":

    pcap_filename = "" # default: read upd packets rom pcapng-file
    json_filename = "" # alternative: read upd packets from json-file
    save_udp_jsonfile = "" # save upd packets to json-file
    udp_port = -1 # 2115 # UDP port to send msgpack datagrams (-1 for udp port from pcapng file)
    udp_send_rate = 0 # send rate in msgpacks per second, 240 for multiScan, or 0 to send corresponding to pcap-timestamps, or udp_send_rate > 1000 for max. rate
    udp_prompt = 0 # prompt for key after sending each udp packet (debugging only)
    udp_dst_ip = "<broadcast>"
    num_repetitions = 1
    verbose = 0
    max_seconds = 3600.0

    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--pcap_filename", help="read upd packets rom pcapng-file", default=pcap_filename, type=str)
    arg_parser.add_argument("--json_filename", help="read upd packets from json-file", default=json_filename, type=str)
    arg_parser.add_argument("--save_udp_jsonfile", help="save upd packets to json-file", default=save_udp_jsonfile, type=str)
    arg_parser.add_argument("--udp_port", help="dst udp port, or -1 for udp port from pcapng file)", default=udp_port, type=int)
    arg_parser.add_argument("--send_rate", help="udp send rate in msgpacks per second, 240 for multiScan, or 0 to send by pcap-timestamps, or > 10000 for max. rate", default=udp_send_rate, type=int)
    arg_parser.add_argument("--prompt", help="prompt for key after sending each udp packet (debugging only)", default=udp_prompt, type=int)
    arg_parser.add_argument("--dst_ip", help="udp destination ip, e.g. 127.0.0.1 or <broadcast>", default=udp_dst_ip, type=str)
    arg_parser.add_argument("--repeat", help="number of repetitions", default=num_repetitions, type=int)
    arg_parser.add_argument("--verbose", help="print verbose messages", default=verbose, type=int)
    arg_parser.add_argument("--filter", help="enable pcap filter by name, e.g. pcap_filter_multiscan_hildesheim for src_ip=192.168.0.1, dst_ip=192.168.0.100", default="", type=str)
    arg_parser.add_argument("--max_seconds", help="max seconds to play", default=max_seconds, type=float)

    cli_args = arg_parser.parse_args()
    pcap_filename = cli_args.pcap_filename
    json_filename = cli_args.json_filename
    save_udp_jsonfile = cli_args.save_udp_jsonfile
    udp_port = cli_args.udp_port
    udp_send_rate = cli_args.send_rate
    udp_prompt = cli_args.prompt
    udp_dst_ip = cli_args.dst_ip
    num_repetitions = cli_args.repeat
    verbose = cli_args.verbose
    max_seconds = cli_args.max_seconds

    # Optional filter pcap blocks by src ip, dst ip and port: send a pcap block if its src ip, dst ip and port is found in these lists (or all if this list empty), default: empty
    pcap_filter = PcapFilter()
    if cli_args.filter == "pcap_filter_multiscan_hildesheim": # pcapng filter multiscan Hildesheim: src_ip=192.168.0.1, dst_ip=192.168.0.100, ports 2115 (scandata) and 7503 (imu)
        pcap_filter = PcapFilter([ "192.168.0.1" ], [ "192.168.0.100" ], [ 2115, 7503 ], [ "UDP", "IP" ] ) 

    # Read and parse pcap file, extract udp raw data
    pcap_blocks = []
    if pcap_filename != "":
        print("multiscan_pcap_player: reading pcapfile \"{}\" ...".format(pcap_filename))
        pcap_blocks = readPcapngFile(pcap_filename, pcap_filter, verbose)
    elif json_filename != "":
        pcap_blocks = readJsonFile(json_filename, verbose)
    else:
        print("## ERROR multiscan_pcap_player: neither pcapng of json file specified, use option --pcap_filename or --json_filename to configure inputfile with upd packets")
    if len(pcap_blocks) == 0:
        print("## ERROR multiscan_pcap_player: no udp packets found, aborting.")
        sys.exit(-1)
    print("multiscan_pcap_player: sending {} udp packets ...".format(len(pcap_blocks)))
    
    # Init upd sender
    udp_sender_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) # UDP socket
    udp_sender_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) # Enable broadcasting mode
    print("multiscan_pcap_player: sending on udp port {}, send_rate={}".format(udp_port, udp_send_rate))
   
    # Send udp raw data
    save_udp_json_blocks = []
    timestamp_end = time.perf_counter() + max_seconds
    udp_port_last = udp_port
    for repeat_cnt in range(num_repetitions):    
        if time.perf_counter() >= timestamp_end:
            break
        send_timestamp = 0
        for block_cnt, pcap_block in enumerate(pcap_blocks):
            block_payload = pcap_block.payload
            block_timestamp = pcap_block.timestamp
            dst_udp_port = 0
            if udp_port >= 0:
                dst_udp_port = udp_port # send to configured udp port
            elif pcap_block.dst_port > 0:
                dst_udp_port = pcap_block.dst_port # send to udp port from pcapng file
            elif udp_port_last > 0:
                dst_udp_port = udp_port_last # udp port = 0: fragmented ip packet, send to udp port of previous udp packet
            else:
                continue # invalid udp port
            udp_port_last = dst_udp_port
            # Send block_payload
            if verbose > 0 or udp_prompt > 0:
                payload_hex_str = "".join("\\x{:02x}".format(payload_byte) for payload_byte in block_payload[:4])
                print("pcap message {}: sending {} byte {}... (udp {}:{})".format(block_cnt, len(block_payload), payload_hex_str, udp_dst_ip, dst_udp_port))
                if udp_prompt > 0:
                    payload_hex_str = "".join("{:02x}".format(payload_byte) for payload_byte in block_payload)
                    if len(payload_hex_str) > 32:
                        payload_hex_str = payload_hex_str[0:32] + "..."
                    if block_payload.find(b'\x02\x02\x02\x02') >= 0:
                        time.sleep(0.1)
                        input("pcap message {}: press ENTER to send {} byte {} >".format(block_cnt, len(block_payload), payload_hex_str))
                    else:
                        print("pcap message {}: sending {} byte {} ...".format(block_cnt, len(block_payload), payload_hex_str))
            # udp_sender_socket.sendto(block_payload, ('<broadcast>', dst_udp_port))
            # udp_sender_socket.sendto(block_payload, ('127.0.0.1', dst_udp_port))
            udp_sender_socket.sendto(block_payload, (udp_dst_ip, dst_udp_port))
            if save_udp_jsonfile:
                payload_hex_str = "".join("{:02x}".format(payload_byte) for payload_byte in block_payload)
                save_udp_json_blocks.append((block_timestamp, dst_udp_port, payload_hex_str))
            if udp_prompt > 0:
                print("pcap message {}: {} byte sent".format(block_cnt, len(block_payload), udp_dst_ip, dst_udp_port))
                # payload_hex_str = "".join("\\x{:02x}".format(payload_byte) for payload_byte in block_payload)
                # print("block_payload dump ({} byte): {}".format(len(block_payload), payload_hex_str))
            # Send next message with delay from pcap timestamps or with configured rate
            if time.perf_counter() >= timestamp_end:
                break
            if send_timestamp > 0 and block_timestamp > send_timestamp and udp_send_rate < 10000:
                if udp_send_rate <= 0: #  delay from pcap timestamps
                    delay = block_timestamp - send_timestamp
                else: # delay from configured rate
                    delay = 1.0 / udp_send_rate
                time.sleep(delay)
            # else: # brute force delay, for performance tests on 2. PC only
            #     forced_delay(2.0e-4)
            send_timestamp = block_timestamp
    if save_udp_jsonfile:
        with open(save_udp_jsonfile, "w") as file_stream:
            json.dump(save_udp_json_blocks, file_stream, indent=2)
            print(f"multiscan_pcap_player: udp packets saved to file {save_udp_jsonfile}")

    print("multiscan_pcap_player finished.")
