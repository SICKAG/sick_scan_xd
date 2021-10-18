#!/usr/bin/env python3

import socket
import random
import xml.etree.ElementTree as ET
from threading import Thread, Event
import sys
sys.stderr = object

print(r'If you have more than one network interface it may happen that no scanner is found'
      r' because the broadcast ip address does not match and the discovery packets are sent to the wrong interface.'
      r'To fix this problem change the parameter <UDP_IP = "192.168.0.255"> '
      r'to the broadcast address that ifconfig returns for your network interface e.g. "192.168.178.255".')
UDP_IP = "192.168.0.255"
UDP_PORT = 30718
RANDOM_KEY=random.randrange(4294967295)
MESSAGE = bytes.fromhex('10000008ffffffffffffc8f4b6270102c0a8007effffff00')
MESSAGE=MESSAGE.replace(bytes.fromhex('c8f4b627'),RANDOM_KEY.to_bytes(4, byteorder='big', signed=False))
HOST = ''                 # Symbolic name meaning all available interfaces
DEBUGMSGENABLED = False
ANSWERTIMEOUT=10 #timeout in seconds
# print("Test .... {<Var-ID>}".format(<VAR_ID>=<VAR|Wert>))
print("UDP target IP: {ip}".format(ip=UDP_IP))
print("UDP target port: {port}".format(port=UDP_PORT))
if(DEBUGMSGENABLED):
    print("Message: {message}".format(message=MESSAGE))
print("The scan result is available in "+str(ANSWERTIMEOUT)+" seconds.")
#print("Random key: {random_key}".format(random_key=RANDOM_KEY.to_bytes(4, byteorder='big', signed=False)))
#inet udp communication

# Event object used to send signals from one thread to another
stop_event = Event()


rawData =[]

def uniq(input):
  output = []
  for x in input:
    if x not in output:
      output.append(x)
  return output

def getScannerXML():
    global rawData
    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.settimeout(ANSWERTIMEOUT - 5)
    sock.bind((HOST, UDP_PORT))
    # broadcast rechte holen
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    while True:
        DATA, addr = sock.recvfrom(2048) # buffer size is 1024 bytes
        if(DEBUGMSGENABLED):
            print("received message: {data}".format(data=DATA))
        rawData.append(DATA)
        #DATA=b'\x90\x00\x04\xae\x00\x06w"H\x83\xc8\xf4\xb6\'\x00\x00<?xml version="1.0" encoding="UTF-8"?><NetScanResult MACAddr="00:06:77:22:48:83"><Item key="IPAddress" value="192.168.0.51" readonly="FALSE"></Item><Item key="IPMask" value="255.255.255.0" readonly="FALSE"></Item><Item key="IPGateway" value="0.0.0.0" readonly="FALSE"></Item><Item key="HostPortNo" value="2112" readonly="TRUE"></Item><Item key="HostModeClnEna" value="0" readonly="TRUE"></Item><Item key="AuxPortNo" value="2111" readonly="TRUE"></Item><Item key="DeviceType" value="TiM5xx V2.54-22.06.16" readonly="TRUE"></Item><Item key="FirmwareVersion" value="V2.54" readonly="TRUE"></Item><Item key="SerialNumber" value="16300542" readonly="TRUE"></Item><Item key="OrderNumber" value="1060445" readonly="TRUE"></Item><Item key="IPConfigDuration" value="5000" readonly="TRUE"></Item><Item key="ModeToHandleIP" value="NoExtraCmd" readonly="TRUE"></Item><Item key="LocationName" value="not defined" readonly="TRUE"></Item><Item key="HasDHCPClient" value="TRUE" readonly="TRUE"></Item><Item key="DHCPClientEnabled" value="FALSE" readonly="FALSE"></Item><Item key="IsBeepSupported" value="FALSE" readonly="TRUE"></Item><Item key="AddressingMode" value="index" readonly="TRUE"></Item></NetScanResult>'
        if stop_event.is_set():
            sock.close()
            print("Wait time out")
            if (DEBUGMSGENABLED):
                print(rawData)
            break
            print("Wait time out 2")

def getIpFromXmlData(DataListInput):
    DataList=uniq(DataListInput)
    for Data in DataList:
        if b"<?xml"  in Data:
            if (DEBUGMSGENABLED):
                print('Scanner found:')
            XML_INDEX=Data.find(b'<?xml')
            #print("XML index in Answerstring is: {xml_index}".format(xml_index=XML_INDEX))
            XML_DATA=Data[XML_INDEX:]
            if (DEBUGMSGENABLED):
                print(XML_DATA)
            root = ET.fromstring(XML_DATA)
            ipAddress = "???"
            IPMask= "???"
            IPGateway = "???"
            DeviceType = "???"
            SerialNumber = "???"
            DHCPClientEnabled= "???"
            FirmwareVersion= "????"
            for child in root:
                    k = child.attrib['key']
                    v = child.attrib['value']
                    if (k == 'IPAddress'):
                        ipAddress=v
                    if (k == 'IPMask'):
                        IPMask=v
                    if (k== 'IPGateway'):
                        IPGateway=v
                    if (k=='DeviceType'):
                        DeviceType=v
                    if (k=='SerialNumber'):
                        SerialNumber=v
                    if (k=='DHCPClientEnabled'):
                        DHCPClientEnabled=v
                    if (k=='FirmwareVersion'):
                        FirmwareVersion=v		   
            print("Device type = "+DeviceType+" SN = "+SerialNumber+" IP = "+ipAddress+" IPMask = "+IPMask+" Gatway = "+IPGateway+" DHCPEnable = "+DHCPClientEnabled+"Firmwarevers.= "+FirmwareVersion)



if __name__ == '__main__':
    # We create another Thread
    action_thread= Thread(target=getScannerXML)

    # Here we start the thread and we wait 5 seconds before the code continues to execute.
    action_thread.start()
    action_thread.join(timeout=ANSWERTIMEOUT+1)

    # We send a signal that the other thread should stop.
    stop_event.set()
    getIpFromXmlData(rawData)
    sys.exit()



