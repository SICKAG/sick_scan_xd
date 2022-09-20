# -*- coding: utf-8 -*-
"""
Created on Fri Oct  2 11:57:33 2020

@author: albrejo
"""
import socket
import time
import json

class UPD_Handler:     
    def __init__( self , _ipAddress, _port, _bufferSize):
        self.ip = _ipAddress
        self.port = _port
        self.bufferSize = _bufferSize
        self.recTimeout = 3
        self.__noErrorFlag = False
        
        self.__openUDPSocket()
        self.counter = 0
        self.lastErrorCode = None
        
    def __del__(self):        
        self.client.close()
        
        
    def __openUDPSocket(self):
        self.client = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
       
        #if self.ip == "127.0.0.1":
        #    self.client.bind(( self.ip, self.port))
        #else:
        #    self.client.bind(( "", self.port))   
        self.client.bind(( "", self.port))
        self.client.settimeout(self.recTimeout)
        
        
    def sendData(self, _data):
        bytesToSend = str.encode(_data)
        print("Send Data UDP: ", _data)
        
        try: 
            self.__noErrorFlag = True
            self.client.sendto(bytesToSend, (self.ip, self.port))
            if self.ip == "127.0.0.1":
                data = self.client.recv(1024) # buffer size is 1024 bytes
                print("Ack UDP Data: %s" % data)
            
        except socket.error as error:
            # print error code 
            self.__noErrorFlag = False
            self.lastErrorCode = error.errno
            print("no connection to sensor . Error Code: {}".format(error.errno))
           
        
    def waitOnNewScan(self):
        try:
            self.__noErrorFlag = True
            # timer_start = time.time_ns()
            stxReceived = False
            while stxReceived == False:
                data_chunk = self.client.recvfrom(self.bufferSize)
                # print("waitOnNewScan: data_chunk[0] =", data_chunk[0])
                if data_chunk[0][0:4] == b'\x02\x02\x02\x02':
                    stxReceived = True
                    payloadLength = int.from_bytes(data_chunk[0][4:8], 'little')
                    # print("STX received, payloadLength =", payloadLength)
            data0 = bytearray(data_chunk[0]) # udp telegram data bytes
            data1 = data_chunk[1]            # udp telegram address
            while len(data0) < payloadLength + 12: # receive 4 byte STX + 4 byte payloadLength + payload + 4 byte CRC32
                data_chunk = self.client.recvfrom(self.bufferSize)
                data0 = data0 + bytearray(data_chunk[0])
            # print(len(data0)," byte received");
            # print("Time wait on UDP-Data: " , (time.time_ns()-timer_start) /1000 /1000 , "[ms]")
            self.counter += 1
            return (data0, data1)
        except socket.error as error:
            # print error code 
            self.__noErrorFlag = False
            self.lastErrorCode = error.errno
            print("Error receiving udp packet. Error Code: {}".format(error.errno))
               
    
    def hasNoError(self):
        return self.__noErrorFlag
    
    def getDataCounter(self):
        return self.counter

    def getLastErrorCode(self):
        return self.lastErrorCode
    

import requests

class RestAPI_Handler:
    def __init__(self, _ipAddress, _crownName):
        self.ip = _ipAddress
        self.crown = _crownName
        self.__noErrorFlag = False
    
    def postRequest(self, _functionName, _argunments = None , _crownName = "default"):
        
        #create postRequest
        if _crownName == "default":
            _crownName = self.crown
        
        postString = 'http://' + self.ip + '/api/crown/' + _crownName + '/' + _functionName
        
        #send request 
        print("URL: ",postString)
        try:
            if _argunments == None:
                r = requests.post(postString)
            else:
                print("ARG:" , json.dumps(_argunments))
                r = requests.post(postString, data = json.dumps(_argunments))
            
            print("RESULT: ", r.text)
            
            #check request has error
            result = json.loads(r.text)
            if result["header"]["status"] == 0:
                self.__noErrorFlag = True
            else:
                self.__noErrorFlag = False  
            return r.text
        
        except:
            self.__noErrorFlag = False
            print("ERROR send restAPIrequest")
        
        
    def hasNoError(self):
        return self.__noErrorFlag
        
        
        
        
        
        
        
        
        
        
