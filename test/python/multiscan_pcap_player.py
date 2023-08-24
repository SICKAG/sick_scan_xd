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
import socket
import time

from pcapng import FileScanner
from pcapng.blocks import EnhancedPacket #, InterfaceDescription, SectionHeader

import scapy.all
import scapy.packet
from scapy.layers.l2 import Ether

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

def readPcapngFile(pcap_filename, verbose):
    blocks_payload = []
    blocks_timestamp = []
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
                    payload = bytes(block_decoded.payload)
                    if len(payload) < 64000:
                        payload = extractMessageStart(payload)
                        if verbose > 0:
                            print("pcap block {}: {} byte payload".format(block_cnt, len(payload)))
                        blocks_payload.append(payload)
                        blocks_timestamp.append(block.timestamp)
    return blocks_payload, blocks_timestamp

if __name__ == "__main__":

    pcap_filename = "multiscan.pcapng" # "../../../../30_LieferantenDokumente/40_Realdaten/20201103_Realdaten/multiscan.pcapng"
    udp_port = 2115 # UDP port to send msgpack datagrams
    udp_send_rate = 0 # send rate in msgpacks per second, 240 for multiScan, or 0 to send corresponding to pcap-timestamps, or udp_send_rate > 1000 for max. rate
    udp_prompt = 0 # prompt for key press after sending each udp packet (debugging only)
    udp_dst_ip = "<broadcast>"
    num_repetitions = 1
    verbose = 0

    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--pcap_filename", help="pcapng filepath", default=pcap_filename, type=str)
    arg_parser.add_argument("--udp_port", help="udp port", default=udp_port, type=int)
    arg_parser.add_argument("--send_rate", help="udp send rate in msgpacks per second, 240 for multiScan, or 0 to send by pcap-timestamps, or > 10000 for max. rate", default=udp_send_rate, type=int)
    arg_parser.add_argument("--prompt", help="prompt for key press after sending each udp packet (debugging only)", default=udp_prompt, type=int)
    arg_parser.add_argument("--dst_ip", help="udp destination ip, e.g. 127.0.0.1 or <broadcast>", default=udp_dst_ip, type=str)
    arg_parser.add_argument("--repeat", help="number of repetitions", default=num_repetitions, type=int)
    arg_parser.add_argument("--verbose", help="print verbose messages", default=verbose, type=int)
    cli_args = arg_parser.parse_args()
    pcap_filename = cli_args.pcap_filename
    udp_port = cli_args.udp_port
    udp_send_rate = cli_args.send_rate
    udp_prompt = cli_args.prompt
    udp_dst_ip = cli_args.dst_ip
    num_repetitions = cli_args.repeat
    verbose = cli_args.verbose
    
    # Read and parse pcap file, extract udp raw data
    print("multiscan_pcap_player: reading pcapfile \"{}\" ...".format(pcap_filename))
    blocks_payload, blocks_timestamp = readPcapngFile(pcap_filename, verbose)
    print("multiscan_pcap_player: sending {} udp packets ...".format(len(blocks_payload)))
    
    # Init upd sender
    udp_sender_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) # UDP socket
    udp_sender_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) # Enable broadcasting mode
    print("multiscan_pcap_player: sending on udp port {}, send_rate={}".format(udp_port, udp_send_rate))
   
    # Send udp raw data
    for repeat_cnt in range(num_repetitions):    
        send_timestamp = 0
        for block_cnt, payload in enumerate(blocks_payload):
            # Send payload
            if verbose > 0 or udp_prompt > 0:
                print("pcap message {}: sending {} byte (udp {}:{})".format(block_cnt, len(payload), udp_dst_ip, udp_port))
                if udp_prompt > 0:
                    payload_hex_str = "".join("{:02x}".format(payload_byte) for payload_byte in payload)
                    if len(payload_hex_str) > 32:
                        payload_hex_str = payload_hex_str[0:32] + "..."
                    if payload.find(b'\x02\x02\x02\x02') >= 0:
                        time.sleep(0.1)
                        input("pcap message {}: press ENTER to send {} byte {} >".format(block_cnt, len(payload), payload_hex_str))
                    else:
                        print("pcap message {}: sending {} byte {} ...".format(block_cnt, len(payload), payload_hex_str))
            # udp_sender_socket.sendto(payload, ('<broadcast>', udp_port))
            # udp_sender_socket.sendto(payload, ('127.0.0.1', udp_port))
            udp_sender_socket.sendto(payload, (udp_dst_ip, udp_port))
            if udp_prompt > 0:
                print("pcap message {}: {} byte sent".format(block_cnt, len(payload), udp_dst_ip, udp_port))
                # payload_hex_str = "".join("\\x{:02x}".format(payload_byte) for payload_byte in payload)
                # print("payload dump ({} byte): {}".format(len(payload), payload_hex_str))
            # Send next message with delay from pcap timestamps or with configured rate
            msg_timestamp = blocks_timestamp[block_cnt]
            if send_timestamp > 0 and msg_timestamp > send_timestamp and udp_send_rate < 10000:
                if udp_send_rate <= 0: #  delay from pcap timestamps
                    delay = msg_timestamp - send_timestamp
                else: # delay from configured rate
                    delay = 1.0 / udp_send_rate
                time.sleep(delay)
            # else: # brute force delay, for performance tests on 2. PC only
            #     forced_delay(2.0e-4)
            send_timestamp = msg_timestamp
    print("multiscan_pcap_player finished.")
