"""
    MRS100 test emulator to parse pcapng files recorded from MRS100 and replay the UDP packages
    to emulate a local MRS100 lidar.

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

# Returns the payload starting from STX = '\x02\x02\x02\x02' (or unmodified payload if STX not found)
def extractMessageStart(payload):
    stx_index = payload.find(b'\x02\x02\x02\x02')
    if stx_index > 0:
        payload = payload[stx_index:]
    return payload

def readPcapngFile(pcap_filename):
    blocks_payload = []
    blocks_timestamp = []
    with open(pcap_filename, 'rb') as pcap_file:
        pcap_scanner = FileScanner(pcap_file)
        for block_cnt, block in enumerate(pcap_scanner):
            if isinstance(block, EnhancedPacket):
                # Decode a single pcap block
                if block.captured_len != block.packet_len:
                    print("## mrs100_pcap_player block {}: {} byte block truncated to {} bytes".format(block_cnt, block.packet_len, block.captured_len))
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
                #     print("## mrs100_pcap_player block {}: empty payload ignored in data {}".format(block_cnt, block_data))
                # if not isinstance(block_decoded.payload, scapy.packet.Raw):
                #     print("## mrs100_pcap_player block {}: block_decoded.payload = {} is no instance of scapy.packet.Raw".format(block_cnt, block_decoded.payload))
                # print("block {}: timestamp = {}".format(block_cnt, block.timestamp))
                # print("block {}: packet_data = {}".format(block_cnt, block.packet_data))
                # print("block {}: payload = {}".format(block_cnt, block_decoded.payload))
                
                # Decode payload
                if isinstance(block_decoded.payload, scapy.packet.Raw) and len(block_decoded.payload) > 0:                
                    payload = bytes(block_decoded.payload)
                    if len(payload) < 64000:
                        payload = extractMessageStart(payload)
                        # print("pcap block {}: {} byte payload".format(block_cnt, len(payload)))
                        blocks_payload.append(payload)
                        blocks_timestamp.append(block.timestamp)
    return blocks_payload, blocks_timestamp

if __name__ == "__main__":

    pcap_filename = "mrs100.pcapng" # "../../../../30_LieferantenDokumente/40_Realdaten/20201103_Realdaten/mrs100.pcapng"
    udp_port = 2115 # UDP port to send msgpack datagrams
    udp_send_rate = 0 # send rate in msgpacks per second, 240 for MRS100, or 0 to send corresponding to pcap-timestamps, or udp_send_rate > 1000 for max. rate
    udp_dst_ip = "<broadcast>"
    num_repetitions = 1

    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--pcap_filename", help="pcapng filepath", default=pcap_filename, type=str)
    arg_parser.add_argument("--udp_port", help="udp port", default=udp_port, type=int)
    arg_parser.add_argument("--send_rate", help="udp send rate in msgpacks per second, 240 for MRS100, or 0 to send by pcap-timestamps, or > 10000 for max. rate", default=udp_send_rate, type=int)
    arg_parser.add_argument("--dst_ip", help="udp destination ip, e.g. 127.0.0.1 or <broadcast>", default=udp_dst_ip, type=str)
    arg_parser.add_argument("--repeat", help="number of repetitions", default=num_repetitions, type=int)
    cli_args = arg_parser.parse_args()
    pcap_filename = cli_args.pcap_filename
    udp_port = cli_args.udp_port
    udp_send_rate = cli_args.send_rate
    udp_dst_ip = cli_args.dst_ip
    num_repetitions = cli_args.repeat
    
    # Read and parse pcap file, extract udp raw data
    print("mrs100_pcap_player: reading pcapfile \"{}\" ...".format(pcap_filename))
    blocks_payload, blocks_timestamp = readPcapngFile(pcap_filename)
    print("mrs100_pcap_player: sending {} udp packets ...".format(len(blocks_payload)))
    
    # Init upd sender
    udp_sender_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) # UDP socket
    udp_sender_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) # Enable broadcasting mode
    print("mrs100_pcap_player: sending on udp port {}, send_rate={}".format(udp_port, udp_send_rate))
   
    # Send udp raw data
    for repeat_cnt in range(num_repetitions):    
        send_timestamp = 0
        for block_cnt, payload in enumerate(blocks_payload):
            # Send payload
            # print("pcap message {}: sending {} byte udp block".format(block_cnt, len(payload)))
            # udp_sender_socket.sendto(payload, ('<broadcast>', udp_port))
            # udp_sender_socket.sendto(payload, ('127.0.0.1', udp_port))
            udp_sender_socket.sendto(payload, (udp_dst_ip, udp_port))
            # Send next message with delay from pcap timestamps or with configured rate
            msg_timestamp = blocks_timestamp[block_cnt]
            if send_timestamp > 0 and msg_timestamp > send_timestamp and udp_send_rate < 10000:
                if udp_send_rate <= 0: #  delay from pcap timestamps
                    delay = msg_timestamp - send_timestamp
                else: # delay from configured rate
                    delay = 1.0 / udp_send_rate
                time.sleep(delay)
            send_timestamp = msg_timestamp
    print("mrs100_pcap_player finished.")
