# ****************************************************************************
# * (C) Copyright 2020, Texas Instruments Incorporated. - www.ti.com
# ****************************************************************************
# *
# *  Redistribution and use in source and binary forms, with or without
# *  modification, are permitted provided that the following conditions are
# *  met:
# *
# *    Redistributions of source code must retain the above copyright notice,
# *    this list of conditions and the following disclaimer.
# *
# *    Redistributions in binary form must reproduce the above copyright
# *    notice, this list of conditions and the following disclaimer in the
# *     documentation and/or other materials provided with the distribution.
# *
# *    Neither the name of Texas Instruments Incorporated nor the names of its
# *    contributors may be used to endorse or promote products derived from
# *    this software without specific prior written permission.
# *
# *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# *  PARTICULAR TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# *  A PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR
# *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# *  EXEMPLARY, ORCONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# *  LIABILITY, WHETHER IN CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING
# *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# *

# import the required Python packages
import struct
import math as m
import math
import binascii
import codecs
import time
import numpy as np

# definations for parser pass/fail
TC_PASS   =  0
TC_FAIL   =  1

def rotX(angle_degrees):
    """
    Generate a 3x3 rotation matrix for rotation around the X-axis.

    Parameters:
        angle_degrees (float): The rotation angle in degrees.

    Returns:
        numpy.ndarray: The 3x3 rotation matrix.
    """
    angle_radians = np.radians(angle_degrees)
    cos_angle = np.cos(angle_radians)
    sin_angle = np.sin(angle_radians)

    rotation_matrix = np.array([[1, 0, 0],
                                [0, cos_angle, -sin_angle],
                                [0, sin_angle, cos_angle]])

    return rotation_matrix

# Assuming you already have the `rotX` function defined.
def rotate_point_x(x, y, z, angle_degrees):
    """
    Rotate a 3D point around the X-axis.

    Parameters:
        x (float): X-coordinate of the point.
        y (float): Y-coordinate of the point.
        z (float): Z-coordinate of the point.
        angle_degrees (float): The rotation angle in degrees.

    Returns:
        Tuple[float, float, float]: The rotated (x, y, z) coordinates.
    """
    rotation_matrix = rotX(angle_degrees)
    point = np.array([[x], [y], [z]])
    rotated_point = np.dot(rotation_matrix, point)
    return rotated_point[0, 0], rotated_point[1, 0], rotated_point[2, 0]

# Function to rotate a set of coordinates [x,y,z] about the various axis via the tilt angles
# Tilt angles are in degrees
def eulerRot(x, y, z, elevTilt, aziTilt):
    # Convert to radians
    elevTilt = np.deg2rad(elevTilt)
    aziTilt = np.deg2rad(aziTilt)

    elevAziRotMatrix = np.matrix([  [  math.cos(aziTilt),  math.cos(elevTilt)*math.sin(aziTilt), math.sin(elevTilt)*math.sin(aziTilt)],
                                    [ -math.sin(aziTilt),  math.cos(elevTilt)*math.cos(aziTilt), math.sin(elevTilt)*math.cos(aziTilt)],
                                    [                  0,                   -math.sin(elevTilt),                   math.cos(elevTilt)],
                                ])
   
    # Old matrix for only Elevation tilt
    # elevRotMatrix = np.matrix([ [ 1,                   0,                  0 ],
    #                             [ 0,  math.cos(elevTilt), math.sin(elevTilt) ],
    #                             [ 0, -math.sin(elevTilt), math.cos(elevTilt) ]
    #                         ])

    target =  np.array([[x],[y],[z]])
    rotTarget = elevAziRotMatrix*target
    rotX = rotTarget[0,0]
    rotY = rotTarget[1,0]
    rotZ = rotTarget[2,0]
    return rotX, rotY, rotZ


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

def getUint32(data):
    """!
       This function coverts 4 bytes to a 32-bit unsigned integer.

        @param data : 1-demension byte array  
        @return     : 32-bit unsigned integer
    """ 
    return (data[0] +
            data[1]*256 +
            data[2]*65536 +
            data[3]*16777216)

def getUint16(data):
    """!
       This function coverts 2 bytes to a 16-bit unsigned integer.

        @param data : 1-demension byte array
        @return     : 16-bit unsigned integer
    """ 
    return (data[0] +
            data[1]*256)

def getHex(data):
    """!
       This function coverts 4 bytes to a 32-bit unsigned integer in hex.

        @param data : 1-demension byte array
        @return     : 32-bit unsigned integer in hex
    """         
    #return (binascii.hexlify(data[::-1]))
    word = [1, 2**8, 2**16, 2**24]
    return np.matmul(data,word)

#7
sideinfo = {
    'snr': UShort(),
    'noise': UShort()
}

#1011
targetIndex = {
    'targetID': UByte()
}

#1000
pointUnit = { # 16 bytes
    'range':        Float(), # Range, in m
    'azimuth':          Float(), # Angle, in rad
    'elevation':          Float(), # Elevation, in rad
    'doppler':            Float() # Doppler, in m/s
}

#pointStruct = { # 8 bytes
#    'elevation': UByte(),
#    'azimuth':	UByte(),
#    'doppler':	UShort(),
#    'range':	UShort(),
#    'snr':	UShort()       
#}

#1010
targetStruct = { #112 bytes
    'tid':              UInt(),# Track ID
    'posX':             Float(), # Target position in X dimension, m
    'posY':             Float(), # Target position in Y dimension, m
    'posZ':             Float(),
    'velX':             Float(), # Target velocity in X dimension, m/s
    'velY':             Float(), # Target velocity in Y dimension, m/s
    'velZ':             Float(),
    'accX':             Float(), # Target acceleration in X dimension, m/s2
    'accY':             Float(), # Target acceleration in Y dimension, m/s
    'accZ':             Float(),
    'ec1':              Float(), # Error covariance matrix, 1,1
    'ec2':              Float(), # Error covariance matrix, 1,2
    'ec3':              Float(), # Error covariance matrix, 1,3
    'ec4':              Float(), # Error covariance matrix, 1,4
    'ec5':              Float(), # Error covariance matrix, 2,1
    'ec6':              Float(), # Error covariance matrix, 2,2
    'ec7':              Float(), # Error covariance matrix, 2,3
    'ec8':              Float(), # Error covariance matrix, 2,4
    'ec9':              Float(), # Error covariance matrix, 3,1
    'ec10':             Float(), # Error covariance matrix, 3,2
    'ec11':             Float(), # Error covariance matrix, 3,3
    'ec12':             Float(), # Error covariance matrix, 3,4
    'ec13':             Float(), # Error covariance matrix, 4,1
    'ec14':             Float(), # Error covariance matrix, 4,2
    'ec15':             Float(), # Error covariance matrix, 4,3
    'ec16':             Float(), # Error covariance matrix, 4,4
    'g':                Float(),
    'confidence':       Float()
}

def checkMagicPattern(data):
    """!
       This function check if data arrary contains the magic pattern which is the start of one mmw demo output packet.  

        @param data : 1-demension byte array
        @return     : 1 if magic pattern is found
                      0 if magic pattern is not found 
    """ 
    found = 0
    if (data[0] == 2 and data[1] == 1 and data[2] == 4 and data[3] == 3 and data[4] == 6 and data[5] == 5 and data[6] == 8 and data[7] == 7):
        found = 1
    return (found)
          
def parser_helper(data, readNumBytes,verbose=False):
    """!
       This function is called by parser_one_mmw_demo_output_packet() function or application to read the input buffer, find the magic number, header location, the length of frame, the number of detected object and the number of TLV contained in this mmw demo output packet.

        @param data                   : 1-demension byte array holds the the data read from mmw demo output. It ignorant of the fact that data is coming from UART directly or file read.  
        @param readNumBytes           : the number of bytes contained in this input byte array  
            
        @return headerStartIndex      : the mmw demo output packet header start location
        @return totalPacketNumBytes   : the mmw demo output packet lenght           
        @return numDetObj             : the number of detected objects contained in this mmw demo output packet          
        @return numTlv                : the number of TLV contained in this mmw demo output packet           
        @return subFrameNumber        : the sbuframe index (0,1,2 or 3) of the frame contained in this mmw demo output packet
    """ 
    
    headerStartIndex = -1

    for index in range (readNumBytes):
        if checkMagicPattern(data[index:index+8:1]) == 1:
            headerStartIndex = index
            break
  
    if headerStartIndex == -1: # does not find the magic number i.e output packet header 
        totalPacketNumBytes = -1
        numDetObj           = -1
        numTlv              = -1
        subFrameNumber      = -1
        platform            = -1
        frameNumber         = -1
        timeCpuCycles       = -1
    else: # find the magic number i.e output packet header 
        totalPacketNumBytes = getUint32(data[headerStartIndex+12:headerStartIndex+16:1])
        platform            = getHex(data[headerStartIndex+16:headerStartIndex+20:1])
        frameNumber         = getUint32(data[headerStartIndex+20:headerStartIndex+24:1])
        timeCpuCycles       = getUint32(data[headerStartIndex+24:headerStartIndex+28:1])
        numDetObj           = getUint32(data[headerStartIndex+28:headerStartIndex+32:1])
        numTlv              = getUint32(data[headerStartIndex+32:headerStartIndex+36:1])
        subFrameNumber      = getUint32(data[headerStartIndex+36:headerStartIndex+40:1])
        #numStatDetObj       = getUint32(data[headerStartIndex+40:headerStartIndex+44:1])
        
    if(verbose):
        #print("headerStartIndex    = %d" % (headerStartIndex))
        #print("totalPacketNumBytes = %d" % (totalPacketNumBytes))
        #print("platform            = %s" % (platform)) 
        #print("frameNumber         = %d             detObj               = %d" % (frameNumber) % (numDetObj)) 
        #print("timeCpuCycles       = %d" % (timeCpuCycles))   
        #print("numDetObj           = %d" % (numDetObj)) 
        print("numTlv              = %d" % (numTlv))
        #print("subFrameNumber      = %d" % (subFrameNumber))   
                            
    return (headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber, frameNumber)


def parser_one_mmw_demo_output_packet(data, readNumBytes,verbose=False):
    """!
       This function is called by application. Firstly it calls parser_helper() function to find the start location of the mmw demo output packet, then extract the contents from the output packet.
       Each invocation of this function handles only one frame at a time and user needs to manage looping around to parse data for multiple frames.

        @param data                   : 1-demension byte array holds the the data read from mmw demo output. It ignorant of the fact that data is coming from UART directly or file read.  
        @param readNumBytes           : the number of bytes contained in this input byte array  
            
        @return result                : parser result. 0 pass otherwise fail
        @return headerStartIndex      : the mmw demo output packet header start location
        @return totalPacketNumBytes   : the mmw demo output packet lenght           
        @return numDetObj             : the number of detected objects contained in this mmw demo output packet          
        @return numTlv                : the number of TLV contained in this mmw demo output packet           
        @return subFrameNumber        : the sbuframe index (0,1,2 or 3) of the frame contained in this mmw demo output packet
    """

    headerNumBytes = 40   

    PI = 3.14159265
    
    result = TC_PASS
    error = 0
    
    targets = list()
    points = list()
    infos = list()
    azimMapObject = {}

    # call parser_helper() function to find the output packet header start location and packet size 
    (headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber, frameNumber) = parser_helper(data, readNumBytes, verbose)
                         
    if headerStartIndex == -1:
        result = TC_FAIL
        print("************ Frame Fail, cannot find the magic words *****************")
    else:
        nextHeaderStartIndex = headerStartIndex + totalPacketNumBytes 

        if headerStartIndex + totalPacketNumBytes > readNumBytes:
            result = TC_FAIL
            print("********** Frame Fail, readNumBytes may not long enough ***********")
        elif nextHeaderStartIndex + 8 < readNumBytes and checkMagicPattern(data[nextHeaderStartIndex:nextHeaderStartIndex+8:1]) == 0:
            result = TC_FAIL
            print("********** Frame Fail, incomplete packet **********") 
        elif numDetObj <= 0:
            result = TC_FAIL
            #print("************ Frame Fail, numDetObj = %d *****************" % (numDetObj))
        elif subFrameNumber > 3:
            result = TC_FAIL
            print("************ Frame Fail, subFrameNumber = %d *****************" % (subFrameNumber))
        else:
            #if(verbose):
            #    print("Number of TLV = %d" % (numTlv))
            tlvIndex = 0
            tlvLenSum = 0
            targets = []
            infos = []
            points = []
            
            while tlvIndex < numTlv:
                try:
                    tlvStart = headerStartIndex + headerNumBytes + (tlvIndex*8) + tlvLenSum
                    tlvType    = getUint32(data[tlvStart+0:tlvStart+4:1])
                    tlvLen     = getUint32(data[tlvStart+4:tlvStart+8:1])
                    tlvLenSum += tlvLen

                    if(verbose): 
                        print("    type %d" % (tlvType))
                    
                   
                    if(verbose): 
                        #print("    frameNumber %d" % (subFrameNumber))       
                        print("    len %d bytes" % (tlvLen))
                        print("    tlvStart %d" % (tlvStart))
                        print("    tlvIndex %d" % (tlvIndex))
                        
                    if tlvType == 7: # Side Info
                        #print("    Side Info")
                        numDetObj = m.floor(tlvLen/4)
                        offset = tlvStart + 8
                        for i in range(0, numDetObj):
                            mySideInfo = {}
                            #Get the unit info
                            for t in sideinfo.items():
                                mySideInfo[t[0]] = t[1].fromBytes(data, offset)
                                offset += t[1].length
                            
                            #print("Side Info")
                            #print(str(mySideInfo))
                            infos.append(mySideInfo)

                    if tlvType == 4: # Heatmap
                        numTxAzimAnt = 2
                        numRxAnt = 4
                        rangeBins = 208
                        digOutSampleRate = 12499
                        freqSlopeConst = 30.0
                        numBytes = numTxAzimAnt * numRxAnt * rangeBins * 4
                        idX = tlvStart + 8

                        q = data[idX:idX + numBytes]

                        qrows = numTxAzimAnt * numRxAnt
                        qcols = rangeBins 
                        NUM_ANGLE_BINS = 64

                        real = q[::4] + q[1::4] * 256
                        error = 2
                        imaginary = q[2::4] + q[3::4] * 256
                        error = 3

                        real = real.astype(np.int16)
                        imaginary = imaginary.astype(np.int16)

                        error = 4
                        q = real + 1j * imaginary
                        error = 5

                        q = np.reshape(q,(qrows,qcols),order="F")
                        error = 6

                        Q = np.fft.fft(q,NUM_ANGLE_BINS,axis=0)
                        error = 7
                        QQ = np.fft.fftshift(abs(Q),axes=0)
                        error = 8
                        QQ = QQ.T

                        QQ = QQ[:,1:]
                        error = 9
                        QQ = np.fliplr(QQ)
                        error = 10

                        # round QQ to 4 decimal places
                        QQ = np.round(QQ,2)

                        # Store the data in the azimMapObject dictionary
                        azimMapObject = {'raw_map': QQ, 'rangeBins': rangeBins, 'digOutSampleRate': digOutSampleRate, 'freqSlopeConst': freqSlopeConst}
                        
                    if tlvType == 1: # Point Cloud
                        offset = tlvStart + 8

                        #offset = tlvStart + 8 + 20
                        numTargets = m.floor((tlvLen)/16)

                        print("Number of Points = %d" % (numTargets))
                        error = 15
                        for i in range(0, numTargets):
                            point = {}
                            # Decode Target data
                            for t in pointUnit.items():
                                point[t[0]] = t[1].fromBytes(data, offset)
                                #print(str(t[0]) + " = " + str(point[t[0]]))
                                
                                if (t[0] == "azimuth" and point[t[0]] >= 128):
                                    #print ('Az greater than 127')
                                    point[t[0]] -= 256
                                if (t[0] == "elevation" and point[t[0]] >= 128):
                                    #print ('Elev greater than 127')
                                    point[t[0]] -= 256
                                if (t[0] == "doppler" and point[t[0]] >= 32768):
                                    #print ('Doppler greater than 32768')
                                    point[t[0]] -= 65536
                                error = 16

                                #point[t[0]] *= myPointUnit[t[0]]
                                #print(str(t[0]) + " = " + str(point[t[0]]))
                                offset += t[1].length
                            
                            #print(str(point))
                            newPoint = {}
                            newPoint['range'] = point['range']
                            newPoint['elevation'] = point['elevation']
                            newPoint['azimuth'] = point['azimuth']
                            newPoint['doppler'] = point['doppler']
                            #newPoint['snr'] = point['snr']
                            
                            #x = point['range'] * np.sin(point['azimuth']) * np.cos(point['elevation'])
                            #y = point['range'] * np.cos(point['azimuth']) *np.cos(point['elevation'])
                            x = point['range'] * np.cos(point['azimuth'])
                            y = point['range'] * np.sin(point['azimuth'])
                            z = point['range'] * np.cos(point['elevation'])
                            
                            #rotatedX, rotatedY, rotatedZ = eulerRot(x, y, z, 0, 0 )
                            newPoint['x'] = x
                            newPoint['y'] = abs(y)
                            newPoint['z'] = z
                            newPoint['velocity_x'] = point['doppler'] * np.sin(point['azimuth']) * np.cos(point['elevation'])
                            newPoint['velocity_y'] = point['doppler'] * np.cos(point['azimuth']) * np.cos(point['elevation'])
                            newPoint['velocity_z'] = point['doppler'] * np.sin(point['elevation'])
                                
                            points.append(newPoint)
                   
                except Exception as e:
                    print("##Fail - could not retrieve TLV data##")
                    print(e)
                    
                tlvIndex = tlvIndex + 1

    return(result, headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber, frameNumber, targets, points, infos, azimMapObject, error)           
    #return (result, headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber, detectedX_array, detectedY_array, detectedZ_array, detectedV_array, detectedRange_array, detectedAzimuth_array, detectedElevAngle_array, detectedSNR_array, detectedNoise_array)


