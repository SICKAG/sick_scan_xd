# -*- coding: utf-8 -*-
"""
Created on Fri Oct  2 11:39:55 2020

@author: albrejo
"""

from enum import Enum
from datetime import datetime
import os 
import time
import msgpack
import json



class SDR_FormatType(Enum):
    JSON = 0
    MSGPACK = 1
    
class MSGPACK_Bytes:
    beginMagic = b'\x85\xaaBeginMagic\xd9$7fe5b904-11b3-4396-9819-6149ab7a7bc6'
    fragment = b'\xa8Fragment'
    stamps = b'\xa6Stamps\x81\xa6TimeUs'
    tracks = b'\xa6Tracks\x91\x82\xa2ID\x00\xa4Data\x91'
    endMagic = b'\xa8EndMagic\xd9$889783e1-1b22-49c2-8773-f2249d00838e'

class SDR_Handler:
    def __init__(self,  _filepath, _formatType = "JSON" , _flagTimestamp = "False"):
        
               
        ####### Global #######
        self.CHAR_FILE_START = "["
        self.CHAR_FILE_END = "]"
        self.CHAR_SEPARATOR = ","
        self.timerStart = 0
        self.counterFragment = 0
        if(_formatType == "JSON"):
            self.formatType = SDR_FormatType.JSON
        else:
            self.formatType = SDR_FormatType.MSGPACK
        ########################  
       
        
        ####### SDR-File #######
        self.file = self.__createSDRFilename(_filepath, self.formatType, _flagTimestamp)
        self.__removeFile(self.file)
        self.sdrFile = 0 
        self.headerSDR = 0
        self.bytesMSGPACK = MSGPACK_Bytes()
        ########################
    
    
        ####### Index-File #######
        self.indexFilePath = self.file.replace("sdr", "sdri")
        self.__removeFile(self.indexFilePath)
        self.indexFile = 0
        self.headerINDEX = 0
        self.counterIndexOffset = 0
        ########################
        
        self.__loadHeader()


    def __removeFile(self, filepath):
        if os.path.exists(filepath):
            #os.remove(self.file)
            f = open(filepath, "w")
            f.write("")
            f.close()
            
            
    def __createSDRFilename(self, _name, _format, _flagTimestamp):
        
        file = _name
        
        if _flagTimestamp == "True":
            file += datetime.now().strftime("_%Y%m%d_%H%M%S")
        
        if _format == SDR_FormatType.JSON:
            file += ".sdr.json"
        if _format == SDR_FormatType.MSGPACK:
            file += ".sdr.msgpack"
        
        return file
        
    
    def __openFileStream(self):
        if self.formatType == SDR_FormatType.JSON:
             self.sdrFile = open(self.file, "a")
             self.indexFile = open(self.indexFilePath, "a")
            
        if self.formatType == SDR_FormatType.MSGPACK: #binary mode
            self.sdrFile = open(self.file, "ab")
            self.indexFile = open(self.indexFilePath, "ab")
    
    
    def __loadHeader(self):
        
        self.initHeader_FileName = 'initHeader.json'
        
        if os.path.isfile(self.initHeader_FileName) == False:
            self.__writeDefault_initHeader(self.initHeader_FileName)
        
        with open(self.initHeader_FileName) as json_file:
            data = json.load(json_file)
            
        indexID = data["ID"] + 'Index'
        
        if self.formatType == SDR_FormatType.JSON:
            self.headerSDR = json.dumps(data).replace(" ", "")
            
            
            index = {"ID":indexID,
                     "Version":data["Version"]}
            self.headerINDEX = json.dumps(index).replace(" ", "")
            
            
        if self.formatType == SDR_FormatType.MSGPACK:
             
            data["Reserved1"] = None
            data["Reserved2"] = None
            data["Reserved3"] = None
            data["Reserved4"] = None
            self.headerSDR = data
            
            index = {"ID":indexID,
                                "Version":data["Version"],
                                "Reserved1" : None,
                                "Reserved2" : None,
                                }
            self.headerINDEX = index
            
    def __writeDefault_initHeader(self, filename):
        
        json_initHeader = {
                                "ID": "SensorDataRecording",
                                "Version": 1,
                                "MetaInfos": [{
                                        "Name": "crown",
                                        "MimeType": "MessagePack string",
                                        "Data": "PScanRecorder.OnNewScan"
                                    }
                                ],
                                "Tracks": [{
                                        "ID": 0,
                                        "Crown": "PScanRecorder",
                                        "Event": "OnNewScan",
                                        "Config": "",
                                        "InstanceNr": 0,
                                        "App": "",
                                        "Types": ["object"]
                                    }
                                ]
                            }
        
        with open(filename, 'w') as outfile:
            json.dump(json_initHeader, outfile)
        
        
        
    def writeHeader(self):
        self.__openFileStream()
        
        #SDR-File
        newHeader = self.getHeader(self.formatType)
        self.sdrFile.write(newHeader)
        
        #Index-File
        self.counterIndexOffset = len(newHeader)
        self.indexFile.write(self.getIndexHeader(self.formatType))
    
        #Start FragmentTimer
        self.timerStart = time.time_ns()
    
    
    def writeData(self, data):
        #get time since create file
        timeUs = round((time.time_ns() - self.timerStart) / 1000)

        #write index file
        self.indexFile.write(self.getIndexElement(self.formatType, self.counterFragment, timeUs, self.counterIndexOffset))
        
        #write sdr file
        newFragment = self.getFragment(self.formatType, self.counterFragment, timeUs, data)
        self.sdrFile.write(newFragment)
     
        #set counter for next fragment
        self.counterIndexOffset += len(newFragment)
        self.counterFragment += 1

       
    def close(self):
        
        if self.formatType == SDR_FormatType.JSON:
            self.sdrFile.write( self.CHAR_FILE_END)
            self.indexFile.write( self.CHAR_FILE_END)
        

        self.sdrFile.close()
        self.indexFile.close()
        
    
    def getHeader(self, _formatType):
    
        if _formatType == SDR_FormatType.JSON:
            header = self.CHAR_FILE_START
            header += self.headerSDR
            return header
        
        if _formatType == SDR_FormatType.MSGPACK:
            return msgpack.packb(self.headerSDR)


    def getIndexHeader(self, _formatType):
        if _formatType == SDR_FormatType.JSON:
            indexHeader = self.CHAR_FILE_START
            indexHeader += self.headerINDEX
            return indexHeader
        
        if _formatType == SDR_FormatType.MSGPACK:
            return msgpack.packb(self.headerINDEX)


    def getFragment(self, _formatType, _fragmentNumber, _timeUs, _data):
        

        if _formatType == SDR_FormatType.JSON:
            
            fragment = self.CHAR_SEPARATOR
            fragment += '''{"BeginMagic":"7fe5b904-11b3-4396-9819-6149ab7a7bc6","Fragment":<counterFragment>,"Stamps":{"TimeUs":<timeUs>},"Tracks":[{"ID":0,"Data":[<data>]}],"EndMagic":"889783e1-1b22-49c2-8773-f2249d00838e"}'''
            fragment = fragment.replace("<counterFragment>", str(_fragmentNumber))
            fragment = fragment.replace("<timeUs>", str(_timeUs))
            fragment = fragment.replace("<data>", str(msgpack.unpackb(_data)))  
         

            return fragment
        
        if _formatType == SDR_FormatType.MSGPACK:
            
            fragment = self.bytesMSGPACK.beginMagic
            fragment += self.bytesMSGPACK.fragment
            fragment += msgpack.packb(_fragmentNumber)
            fragment += self.bytesMSGPACK.stamps
            fragment += msgpack.packb(_timeUs)
            fragment += self.bytesMSGPACK.tracks
            fragment += _data
            fragment += self.bytesMSGPACK.endMagic
            
            return fragment

    
    
    def getIndexElement(self, _formatType, _fragmentNumber, _timeUs, _offset):
        
        if _formatType == SDR_FormatType.JSON:
            indexElement = self.CHAR_SEPARATOR
            indexElement += '''{"Stamps":{"TimeUs":<timeUs>},"Fragment":<counterFragment>,"Offset":<offset>}'''
            indexElement = indexElement.replace("<counterFragment>", str(_fragmentNumber))
            indexElement = indexElement.replace("<timeUs>", str(_timeUs))
            indexElement = indexElement.replace("<offset>", str(_offset + len(self.CHAR_SEPARATOR)))            
            
            return indexElement
            
        if _formatType == SDR_FormatType.MSGPACK:
            indexElement = {"Stamps":{"TimeUs":_timeUs},
                                   "Fragment":_fragmentNumber,
                                   "Offset":_offset}
        
            return msgpack.packb(indexElement)
                    
            
    
        
        
        