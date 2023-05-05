# -*- coding: utf-8 -*-
"""
Created on Wed Oct 21 11:27:24 2020

@author: honalma

Version 1.1.1R
"""

from common import connectionHandler
import zlib
import msgpack
import numpy as np
import struct
from matplotlib import cm
from matplotlib.colors import ListedColormap, LinearSegmentedColormap
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Map of tokenized msgpack keys:
msgpack_tokenized_keys_str2int = {
    "class" 		    : 0x10,
    "data" 			    : 0x11,
    "numOfElems" 		: 0x12,
    "elemSz" 		    : 0x13,
    "endian" 		    : 0x14,
    "elemTypes" 		: 0x15,
    "little" 		    : 0x30,
    "float32" 		    : 0x31,
    "ChannelTheta" 		: 0x50,
    "ChannelPhi" 		: 0x51,
    "DistValues" 		: 0x52,
    "RssiValues" 		: 0x53,
    "PropertiesValues" 	: 0x54,
    "Scan" 			    : 0x70,
    "TimestampStart" 	: 0x71,
    "TimestampStop" 	: 0x72,
    "ThetaStart" 		: 0x73,
    "ThetaStop" 		: 0x74,
    "ScanNumber" 		: 0x75,
    "ModuleId" 		    : 0x76,
    "BeamCount" 		: 0x77,
    "EchoCount" 		: 0x78,
    "ScanSegment" 		: 0x90,
    "SegmentCounter" 	: 0x91,
    "FrameNumber" 		: 0x92,
    "Availability" 		: 0x93,
    "SenderId" 		    : 0x94,
    "SegmentSize" 		: 0x95,
    "SegmentData" 		: 0x96,
    "TelegramCounter" 	: 0xB0,
    "TimestampTransmit" : 0xB1
}
msgpack_tokenized_keys_int2str = {
    0x10 : "class",
    0x11 : "data",
    0x12 : "numOfElems",
    0x13 : "elemSz",
    0x14 : "endian",
    0x15 : "elemTypes",
    0x30 : "little",
    0x31 : "float32",
    0x50 : "ChannelTheta",
    0x51 : "ChannelPhi",
    0x52 : "DistValues",
    0x53 : "RssiValues",
    0x54 : "PropertiesValues",
    0x70 : "Scan",
    0x71 : "TimestampStart",
    0x72 : "TimestampStop",
    0x73 : "ThetaStart",
    0x74 : "ThetaStop",
    0x75 : "ScanNumber",
    0x76 : "ModuleId",
    0x77 : "BeamCount",
    0x78 : "EchoCount",
    0x90 : "ScanSegment",
    0x91 : "SegmentCounter",
    0x92 : "FrameNumber",
    0x93 : "Availability",
    0x94 : "SenderId",
    0x95 : "SegmentSize",
    0x96 : "SegmentData",
    0xB0 : "TelegramCounter",
    0xB1 : "TimestampTransmit"
}

# Old version previous to integer keys uses string keys
# def msgpackKeyToken(s):
#     return s

# Return the integer keys of a tokenized msgpack key
def msgpackKeyToken(s):
    return msgpack_tokenized_keys_str2int[s]

class MultiScanReceiver:
    
    def __init__(self, scannerIP = "192.168.0.1", hostIP = "192.168.0.102", port=2115, nbScans = 100, writeMsgpackFile = ""):
        self.scannerIP = scannerIP
        self.hostIP = hostIP
        self.port = port
        self.crownName = "MSGPACKUDPOutput"
        self.restAPI = connectionHandler.RestAPI_Handler(self.scannerIP, self.crownName)
        self.nbScans = nbScans
        self.writeMsgpackFile = writeMsgpackFile
        self.bufferSize = 100000
        self.showCount = 0
        self.enableRestAPI = False
        self.enableMagicalActivate = False
        self.enableVisualization = False
        self.debug = False
    
    def setEnableRestAPI(self, value):
        self.enableRestAPI = value

    def setEnableMagicalActivate(self, value):
        self.enableMagicalActivate = value

    def setEnableVisualization(self, value):
        self.enableVisualization = value


    def setupRecording(self):
        if self.enableRestAPI:
          self.restAPI.postRequest("setPort",  {'args': {'port': self.port}})
          self.restAPI.postRequest("setIpAddress", {'args': {'ipAddress': self.hostIP}})
        else:
          print("REST API not active, nothing to setup")

        
    def doRecording(self):
          
        UDPsocket = connectionHandler.UPD_Handler(self.scannerIP, self.port, self.bufferSize)
        
        if self.enableMagicalActivate:
            UDPsocket.sendData("magicalActivate")

        if self.enableRestAPI:
            self.restAPI.postRequest("start")
        
        if(self.restAPI.hasNoError() == True) or (self.enableRestAPI == False):
            for i in range(0, self.nbScans):
                data = UDPsocket.waitOnNewScan()
                if(UDPsocket.hasNoError() == True):
                    print("received scan", i)
                    payload = self.verifyData(data)
                    if payload != "":
                        scanVector = self.parseData(payload)
                        pointCloud = self.convertToPointCloud(scanVector)
                        if self.enableVisualization:
                            self.showScan(pointCloud)
                        if self.writeMsgpackFile != "":
                            msgpackfile = "{}_{}".format(self.writeMsgpackFile, len(payload))
                            with open(msgpackfile + ".msgpack", "wb") as f:
                                f.write(payload)
                            payload_hex_str = ' '.join(format(b, '02X') for b in payload)
                            with open(msgpackfile + ".msgpack.hex", "w") as f:
                                f.write(payload_hex_str)
                            self.writeMsgpackFile = ""
                    else:
                        print("Error extracting payload from data")
                else:
                    print("error receiving scan ",i)
                    
        if self.enableRestAPI:
            self.restAPI.postRequest("stop")
                    
        del UDPsocket
        
    def verifyData(self, data):
        msgAsBytes = data[0]         
        stxString = b'\x02\x02\x02\x02'
        status = True 
    
        #check stxString
        stxReceived = msgAsBytes[0:4]
        if stxString != stxReceived:
            status = False
            print("Error receiving stx")
            return ""
        #else:
        #    print("STX")
            
        # check string lengths
        stringLength = len(msgAsBytes) - 12 # msgAsBytes := 4 byte STX + 4 byte payloadLength + payload + 4 byte CRC32
        payloadlength = int.from_bytes(msgAsBytes[4:8], 'little')
        if stringLength < payloadlength:
            status = False
            print("Error payloadlengths do not match: received payloadlength =", stringLength, "decoded payloadlength =", payloadlength)
            return ""

        # cutout stx, string length and checksum
        payLoad = msgAsBytes[8:8+payloadlength]

        #check crc-Checksum
        crcReceived = int.from_bytes(msgAsBytes[8+payloadlength:8+payloadlength+4], 'little')
        crcComputed = zlib.crc32(payLoad)
        if crcReceived != crcComputed:
            status = False
            print("Error Checksum: crcReceived =", crcReceived, "crcComputed =", crcComputed)
            return ""
        
        if status == False:
            payLoad = ""
            
        return payLoad
    
    def parseData(self, payload):
        dataDict = msgpack.unpackb(payload, strict_map_key=False)
        # extract meta data
        segmentData = []
        try:
            availability = dataDict[msgpackKeyToken('data')][msgpackKeyToken('Availability')]
            frameNumber =  dataDict[msgpackKeyToken('data')][msgpackKeyToken('FrameNumber')]
            segmentCounter =  dataDict[msgpackKeyToken('data')][msgpackKeyToken('SegmentCounter')]
            segmentSize =  dataDict[msgpackKeyToken('data')][msgpackKeyToken('SegmentSize')]
            senderId =  dataDict[msgpackKeyToken('data')][msgpackKeyToken('SenderId')]
            telegramCounter =  dataDict[msgpackKeyToken('data')][msgpackKeyToken('TelegramCounter')]
            timeStampTransmit = dataDict[msgpackKeyToken('data')][msgpackKeyToken('TimestampTransmit')]
            segmentData = dataDict[msgpackKeyToken('data')][msgpackKeyToken('SegmentData')]
            
            # just for debugging
            # if self.debug:
            #     phi = segmentData[0][msgpackKeyToken('data')][msgpackKeyToken('ChannelPhi')][msgpackKeyToken('data')]
            #     phi0 = struct.unpack('f', phi)
            #     theta = segmentData[0][msgpackKeyToken('data')][msgpackKeyToken('ChannelTheta')][msgpackKeyToken('data')]
            #     theta0 = struct.unpack('60f', theta)
            #     scanNumber = segmentData[0][msgpackKeyToken('data')][msgpackKeyToken('ScanNumber')]
            #     print("tgmCnt = {0}, theta={1:1.2f}, phi={2:1.2f}".format(telegramCounter, np.rad2deg(theta0[0]), np.rad2deg(phi0[0])) )
            #     print("theta =", theta0)
            #     print("phi =", phi0)
            # end debugging
        except Exception:
            print("multiscan_receiver: parseData failed")
        return segmentData
        
    def convertToPointCloud(self, scanVector):
        segmentXYZI = [] # 2d list with dimensions nbLayers x nbEchos

        nbLayers = len(scanVector) # equals to segmentSize
        segmentXYZI = [None for _ in range(nbLayers)] # nbLayers None objects
        for layerIdx in range(nbLayers):
            nbEchoes = scanVector[layerIdx][msgpackKeyToken('data')][msgpackKeyToken('EchoCount')]
            segmentXYZI[layerIdx] = [None for _ in range(nbEchoes)] # nbEchoes None objects for this layer

            # Elevation and azimuth are the same for each echo.
            # Moreover these quantities are the same for each frame, i.e. they 
            # could be precomputed for speed. For the sake of simplicity this is not done here however.
            nbBeams = scanVector[layerIdx][msgpackKeyToken('data')][msgpackKeyToken('BeamCount')]
            elevationPseudoArr = struct.unpack('f', scanVector[layerIdx][msgpackKeyToken('data')][msgpackKeyToken('ChannelPhi')][msgpackKeyToken('data')])
            # Elevation must be negated. A positive pitch-angle yields negative z-coordinates.
            elevation = -1 * elevationPseudoArr[0]
            formatFloatArray = str(nbBeams ) + 'f'
            azimuthPseudoArr = struct.unpack(formatFloatArray, scanVector[layerIdx][msgpackKeyToken('data')][msgpackKeyToken('ChannelTheta')][msgpackKeyToken('data')])
            azimuth = np.asarray(azimuthPseudoArr)

            # just for debugging
            if self.debug:
                print("phi =", elevationPseudoArr[0])
                print("theta =", azimuth)
            # end debugging

            for echoIdx in range(nbEchoes):
                segmentXYZI[layerIdx][echoIdx] = np.zeros([nbBeams , 4]) # 4 due to x,y,z,i
                destArr = segmentXYZI[layerIdx][echoIdx]
                
                distPseudoArr = struct.unpack(formatFloatArray, scanVector[layerIdx][msgpackKeyToken('data')][msgpackKeyToken('DistValues')][echoIdx][msgpackKeyToken('data')])
                dist = np.asarray(distPseudoArr) * 0.001 # conversion to m
                rssiPseudoArr = struct.unpack(formatFloatArray, scanVector[layerIdx][msgpackKeyToken('data')][msgpackKeyToken('RssiValues')][echoIdx][msgpackKeyToken('data')])
                rssi = np.asarray(rssiPseudoArr)
                
                # Convert to cartesian coordinates
                destArr[:,0] = dist * np.cos(azimuth) * np.cos(elevation) # x
                destArr[:,1] = dist * np.sin(azimuth) * np.cos(elevation) # y
                destArr[:,2] = dist * np.sin(elevation) # z
                destArr[:,3] = rssi

                # just for debugging
                if self.debug:
                    print("dist =", distPseudoArr)
                    print("rssi =", rssiPseudoArr)
                # end debugging
            # self.debug = False

        return segmentXYZI
                

        
    def showScan(self, pointCloud):
        if (self.showCount == 0):
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111, projection='3d')


        else:
            self.ax.clear()
            
        self.ax.set_title('multiScan Emulation Segment {}'.format(self.showCount))
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(-1, 1)
        self.ax.set_xlabel('X Label')
        self.ax.set_ylabel('Y Label')
        self.ax.set_zlabel('Z Label')

        for layerIdx in range(len(pointCloud)):
            for echoIdx in range(len(pointCloud[layerIdx])):
                curCloud = pointCloud[layerIdx][echoIdx]
                self.ax.scatter(curCloud[:, 0], curCloud[:, 1], curCloud[:, 2], s=1)


        plt.show(block=False)
        plt.draw()
    
    
        plt.pause(0.001)
        self.showCount += 1

    

    def readFromMSGPACKFile(self, fileName):      
        with open(fileName, "rb") as f:
            byte_data = f.read()
        msgpack_dict = msgpack.unpackb(byte_data)

        for d in msgpack_dict:
            if 'BeginMagic' in d:
                dataDict = d['Tracks'][0]['Data'][0] # the is a ScanSegment now
                # print some meta data
                frameNumber =  dataDict['data']['FrameNumber']
                segmentCounter =  dataDict['data']['SegmentCounter']
                telegramCounter =  dataDict['data']['TelegramCounter']
                timeStampTransmit = dataDict['data']['TimestampTransmit']

                print("tgmCounter = {}, timeStampTransmit = {}, frameNumber = {}, segmentCounter = {}"
                      .format(telegramCounter, timeStampTransmit, frameNumber, segmentCounter))
                
                # convert to point cloud and plot
                segmentData = dataDict['data']['SegmentData']
                pointCloud = self.convertToPointCloud(segmentData)
                if self.enableVisualization:
                    self.showScan(pointCloud)
    
        
if __name__ == "__main__":
    receiver = MultiScanReceiver(scannerIP = "127.0.0.1", hostIP = "127.0.0.1", port = 2115, nbScans = 1000, writeMsgpackFile = "./multiscan_dump")
    
    mode = 'live'
    
    if mode == 'live':
        receiver.setEnableVisualization(False)
        receiver.setEnableMagicalActivate(False)
        receiver.setEnableRestAPI(False)
        receiver.setupRecording()
        receiver.doRecording()
    elif mode == 'file':
        receiver.readFromMSGPACKFile("data\\recordFile_16.msgpack")
    else:
        print("Error: Unkown mode")
          
          
