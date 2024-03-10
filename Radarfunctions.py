##Radar functions
import time
import numpy as np
import serial
import serial.serialutil
import struct
import json,os
from parser_mmv_demo_2 import parser_one_mmw_demo_output_packet

class ULong: #uint64
    length: int = 8
    def __init__(self):
        pass
    
    def fromBytes(self, data: bytes, offset: int = 0):
        return struct.unpack_from('<Q', data, offset)[0]

class UInt: #uint32
    length: int = 4
    def __init__(self):
        pass
    
    def fromBytes(self, data: bytes, offset: int = 0):
        return struct.unpack_from('<I', data, offset)[0]

class UShort: #uint16
    length: int = 2
    def __init__(self):
        pass
    
    def fromBytes(self, data: bytes, offset: int = 0):
        return struct.unpack_from('<H', data, offset)[0]

class UByte: #uint8
    length: int = 1
    def __init__(self):
        pass
    
    def fromBytes(self, data: bytes, offset: int = 0):
        return struct.unpack_from('<B', data, offset)[0]

class Float:
    length: int = 4
    def __init__(self):
        pass
    
    def fromBytes(self, data: bytes, offset: int = 0):
        return struct.unpack_from('<f', data, offset)[0]

class Radarfunctions:
    def __init__(self,usbportCLI,usbportDATA,baudrateCLI,baudrateData,bytezise,parity,timeout,configFileName,byteBufferLength,byteBuffer) -> None:
            self.usbportCLI = usbportCLI
            self.usbportDATA = usbportDATA
            self.baudrateCLI = baudrateCLI
            self.baudrateData = baudrateData
            self.bytezise = bytezise
            self.parity = parity
            self.timeout = timeout
            self.configFileName = configFileName
            self.byteBufferLength = byteBufferLength
            self.byteBuffer = byteBuffer
            self.DataPort = None
            self.CLIport = None
            #self.configParameters = None
            self.maxBufferSize = 2**15;
            self.magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
            #self.detObj = {}  
            self.frameData = {}    
            self.currentIndex = 0
            # word array to convert 4 bytes to a 32 bit number
            self.word = [1, 2**8, 2**16, 2**24]
            
    def serialConfig(self):
        CLIport = {}
        Dataport = {}
        
        try:
            CLIport = serial.Serial(port=self.usbportCLI, baudrate=self.baudrateCLI, bytesize=self.bytezise, parity=self.parity, timeout=self.timeout)
            Dataport = serial.Serial(port=self.usbportDATA, baudrate=self.baudrateData, bytesize=self.bytezise, parity=self.parity, timeout=self.timeout)
            if Dataport.isOpen():
                Dataport.close()
            if CLIport.isOpen():
                CLIport.close()

            Dataport.open()
            CLIport.open()
            print(f"Serial connection opened.")
                        
            config = [line.rstrip('\r\n') for line in open(self.configFileName)]
            for i in config:
                CLIport.write((i+'\n').encode())
                print(i)
                print(CLIport.readline())
                time.sleep(0.01)# Sleep for 10 milliseconds to prevent excessive CPU usage

        except serial.SerialException as e:
            print(f"Error opening the serial connection: {e}")


        # Read the configuration file and send it to the board
        self.Dataport = Dataport
        self.CLIport = CLIport
        return CLIport , Dataport
    
    def readAndParseData14xx(self,verbose):
        #byteBufferLength = self.byteBufferLength
        #byteBuffer = self.byteBuffer
        # Initialize variables
        magicOK = 0 # Checks if magic number has been read
        dataOK = 0 # Checks if the data has been read correctly
        frameNumber = 0
        targets = []
        points = []
        infos = []
        azimuthHeatMapObj = {}

        readBuffer = self.Dataport.read(self.Dataport.in_waiting)
        #readBuffer = self.D
        #readBuffer = self.DataPort.read(self.DataPort.in_waiting)
        byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
        byteCount = len(byteVec)
        
        # Check that the buffer is not full, and then add the data to the buffer
        #if (byteBufferLength + byteCount) < self.maxBufferSize:
        self.byteBuffer[self.byteBufferLength:self.byteBufferLength + byteCount] = byteVec[:byteCount]
        self.byteBufferLength = self.byteBufferLength + byteCount
                
        # Check that the buffer has some data
        if self.byteBufferLength > 16:
            
            # Check for all possible locations of the magic word
            possibleLocs = np.where(self.byteBuffer == self.magicWord[0])[0]

            # Confirm that is the beginning of the magic word and store the index in startIdx
            startIdx = []
            for loc in possibleLocs:
                check = self.byteBuffer[loc:loc+8]
                if np.all(check == self.magicWord):
                    startIdx.append(loc)

            # Check that startIdx is not empty
            if startIdx:
                
                # Remove the data before the first start index
                if startIdx[0] > 0 and startIdx[0] < self.byteBufferLength:
                    self.byteBuffer[:self.byteBufferLength-startIdx[0]] = self.byteBuffer[startIdx[0]:self.byteBufferLength]
                    self.byteBuffer[self.byteBufferLength-startIdx[0]:] = np.zeros(len(self.byteBuffer[self.byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                    self.byteBufferLength = self.byteBufferLength - startIdx[0]
                    
                # Check that there have no errors with the byte buffer length
                if self.byteBufferLength < 0:
                    self.byteBufferLength = 0

                # Read the total packet length
                totalPacketLen = np.matmul(self.byteBuffer[12:12+4],self.word)
                # Check that all the packet has been read
                if (self.byteBufferLength >= totalPacketLen) and (self.byteBufferLength != 0):
                    magicOK = 1
        
        # If magicOK is equal to 1 then process the message
        if magicOK:
            # Read the entire buffer
            readNumBytes = self.byteBufferLength
            #if(verbose):
                #print("readNumBytes: ", readNumBytes)
            allBinData = self.byteBuffer
            #if(verbose):
                #print("allBinData: ", allBinData[0], allBinData[1], allBinData[2], allBinData[3])

            # init local variables
            totalBytesParsed = 0;
            numFramesParsed = 0;

            # parser_one_mmw_demo_output_packet extracts only one complete frame at a time
            # so call this in a loop till end of file
            #             
            # parser_one_mmw_demo_output_packet function already prints the
            # parsed data to stdio. So showcasing only saving the data to arrays 
            # here for further custom processing
            parser_result, \
            headerStartIndex,  \
            totalPacketNumBytes, \
            numDetObj,  \
            numTlv,  \
            subFrameNumber, \
            frameNumber, \
            targets, \
            points, \
            infos, \
            azimuthHeatMapObj, \
            p_error = parser_one_mmw_demo_output_packet(allBinData[totalBytesParsed::1], readNumBytes-totalBytesParsed,verbose)

            # Check the parser result
            #if(verbose):
                #print ("Parser result: ", parser_result)
            if (parser_result == 0): 
                totalBytesParsed += (headerStartIndex+totalPacketNumBytes)    
                numFramesParsed+=1
            
                
                #if(verbose):
                #    print("totalBytesParsed: ", totalBytesParsed)
                #dataOK = 1         
            elif(parser_result == 1): 
                # error in parsing; exit the loop
                dataOK = 0
                print("error in parsing this frame; continue")

            
            if headerStartIndex > -1:
                shiftSize = totalPacketNumBytes            
                self.byteBuffer[:self.byteBufferLength - shiftSize] = self.byteBuffer[shiftSize:self.byteBufferLength]
                self.byteBuffer[self.byteBufferLength - shiftSize:] = np.zeros(len(self.byteBuffer[self.byteBufferLength - shiftSize:]),dtype = 'uint8')
                self.byteBufferLength = self.byteBufferLength - shiftSize
            
            # Check that there are no errors with the buffer length
            if self.byteBufferLength < 0:
                self.byteBufferLength = 0
            # All processing done; Exit
            #if(verbose):
            #    print("numFramesParsed: ", numFramesParsed)

        return dataOK, frameNumber, targets, points, infos, azimuthHeatMapObj

    def update(self,verbose):
        targets = []
        points = []
        infos = []
        dataOk = 0
        #x = []
        #y = []

        # Read and parse the received data
        dataOk, frameNumber, targets, points, infos, azimuthHeatMapObj = self.readAndParseData14xx(verbose)
        if(verbose):
            print('dataOk: ', dataOk)
     
        return dataOk,targets, points, infos, azimuthHeatMapObj
