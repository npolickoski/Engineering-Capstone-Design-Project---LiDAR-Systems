#--------------------------------------------------------------------------------------------------------------
# LightWare 2022
#--------------------------------------------------------------------------------------------------------------
# Description:
#   This samples communicates with LightWare devices that use the LWNX protocol.
#
# Notes:
# 	Requires the pySerial module.
# 
#      *********  REPORTS OUT ONLY LAST RAW DISTANCE AND YAW ANGLE   *********
#                     Scans from High Angle to Low Angle
#                 Prints Last Raw and Yaw Angles to Python Shell
#                   Saves scan to angle[] and distance[] lists
#                   Saves one set of (angle and distance lists) to a file
#                            while true break at line 352
#                     Calculates and characterizes objects between minRange and maxRange
#
#     Crontab Protocol: @reboot python3 /home/jpicciri/Documents/AutoNavTeamSoftware/SF45pythonV9.py &
#--------------------------------------------------------------------------------------------------------------

import os
import time
import serial
import numpy as np
import struct
import LidarObjectDetectionV5 as objDetect

numPts = 0           #  number of points in a complete scan 
temp = 0             # temporary scan angle
maxRange   = 3100     # max object detection distance (cm) (1 meter)
minRange   = 10
# gapSize    = 10      # min angular separation of objects

DegToRad = 3.14159/180
scanAngle = []           #  raw scan data
scanDist = []

#--------------------------------------------------------------------------------------------------------------
# LWNX library functions.
#--------------------------------------------------------------------------------------------------------------
packetParseState = 0
packetPayloadSize = 0
packetSize = 0
packetData = []

# Create a CRC-16-CCITT 0x1021 hash of the specified data.
def createCrc(data):
	crc = 0
	
	for i in data:
		code = crc >> 8
		code ^= int(i)
		code ^= code >> 4
		crc = crc << 8
		crc ^= code
		code = code << 5
		crc ^= code
		code = code << 7
		crc ^= code
		crc &= 0xFFFF

	return crc

# Create raw bytes for a packet.
def buildPacket(command, write, data=[]):
	payloadLength = 1 + len(data)
	flags = (payloadLength << 6) | (write & 0x1)
	packetBytes = [0xAA, flags & 0xFF, (flags >> 8) & 0xFF, command]
	packetBytes.extend(data)
	crc = createCrc(packetBytes)
	packetBytes.append(crc & 0xFF)
	packetBytes.append((crc >> 8) & 0xFF)

	return bytearray(packetBytes)

# Check for packet in byte stream.
def parsePacket(byte):
	global packetParseState
	global packetPayloadSize
	global packetSize
	global packetData

	if packetParseState == 0:
		if byte == 0xAA:
			packetParseState = 1
			packetData = [0xAA]

	elif packetParseState == 1:
		packetParseState = 2
		packetData.append(byte)

	elif packetParseState == 2:
		packetParseState = 3
		packetData.append(byte)
		packetPayloadSize = (packetData[1] | (packetData[2] << 8)) >> 6
		packetPayloadSize += 2
		packetSize = 3

		if packetPayloadSize > 1019:
			packetParseState = 0

	elif packetParseState == 3:
		packetData.append(byte)
		packetSize += 1
		packetPayloadSize -= 1

		if packetPayloadSize == 0:
			packetParseState = 0
			crc = packetData[packetSize - 2] | (packetData[packetSize - 1] << 8)
			verifyCrc = createCrc(packetData[0:-2])
			
			if crc == verifyCrc:
				return True

# Wait (up to timeout) for a packet to be received of the specified command.
def waitForPacket(port, command, timeout=1):
	global packetParseState
	global packetPayloadSize
	global packetSize
	global packetData

	packetParseState = 0
	packetData = []
	packetPayloadSize = 0
	packetSize = 0

	endTime = time.time() + timeout

	while True:
		if time.time() >= endTime:
			return None

		c = port.read(1)

		if len(c) != 0:
			b = ord(c)
			if parsePacket(b) == True:
				if packetData[3] == command:
					return packetData

# Extract a 16 byte string from a string packet.
def readStr16(packetData):
	str16 = ''
	for i in range(0, 16):
		if packetData[4 + i] == 0:
			break
		else:
			str16 += chr(packetData[4 + i])

	return str16

#   *******  lidar variables selected here
#         Must match does in command 27 parameters
#            and in Main Application # Read distance data

# Extract signal data from a signal data packet.
def readSignalData(packetData):
# 	firstRaw = packetData[4] << 0
# 	firstRaw |= packetData[5] << 8
# 	firstRaw /= 100.0


# 	firstFiltered = packetData[6] << 0
# 	firstFiltered |= packetData[7] << 8
# 	firstFiltered /= 100.0
# 
# 	firstStrength = packetData[8] << 0
# 	firstStrength |= packetData[9] << 8

	lastRaw = packetData[4] << 0
	lastRaw |= packetData[5] << 8    # cm
#	lastRaw /= 100.0                 # meters

# 	lastFiltered = packetData[12] << 0
# 	lastFiltered |= packetData[13] << 8
# 	lastFiltered /= 100.0
# 
# 	lastStrength = packetData[14] << 0
# 	lastStrength |= packetData[15] << 8
# 
# 	noise = packetData[16] << 0
# 	noise |= packetData[17] << 8
# 
# 	temperature = packetData[18] << 0
# 	temperature |= packetData[19] << 8
# 	temperature /= 100

	yawAngle = packetData[6] << 0
	yawAngle |= packetData[7] << 8
	if yawAngle > 32000:
		yawAngle = yawAngle - 65535
	yawAngle /= 100.0                # decimal deg

# 	return firstRaw, firstFiltered, firstStrength, lastRaw, lastFiltered, lastStrength, noise, temperature , yawAngle 

	return lastRaw, yawAngle 
# -------------------------------------------------------------

# Send a request packet and wait for response.
def executeCommand(port, command, write, data=[], timeout=1):
	packet = buildPacket(command, write, data)
	retries = 4

	while retries > 0:
		retries -= 1
		port.write(packet)

		response = waitForPacket(port, command, timeout)

		if response != None:
			return response

	raise Exception('LWNX command failed to receive a response.')

def float_to_bin(num):
    return format(struct.unpack('!I', struct.pack('!f', num))[0], '032b')

def High_bin_to_Dec(num1):
        a4 = num1[0:8]
        a3 = num1[8:16]
        a2 = num1[16:24]
        a1 = num1[24:32]
        msbh = int(a4, 2)
        lsbh = int(a3, 2)
        dk1  = int(a2, 2)
        dk2  = int(a1, 2)
        return msbh, lsbh, dk1, dk2

def Low_bin_to_Dec(num2):
        a4 = num2[0:8]
        a3 = num2[8:16]
        a2 = num2[16:24]
        a1 = num2[24:32]
        msbl = int(a4, 2)
        lsbl = int(a3, 2)
        dk3  = int(a2, 2)
        dk4  = int(a1, 2)
        return msbl, lsbl, dk3, dk4

def method_a(sample_string):
    c =' '.join(format(ord(x), 'b032') for x in sample_string)
    return c

def Convert_speed(num3):
        Speed1 = "{0:016b}".format(num3)
        s1 = Speed1[0:8]
        s2 = Speed1[8:16]
        msbs = int(s1, 2)
        lsbs = int(s2, 2)
        return msbs, lsbs
#   End of LWNX Library Functions    
    
def skip(scanAngle,temp):
    if(scanAngle == temp):
        skipFlag = False
    else:
        temp = scanAngle
        skipFlag = True
    return skipFlag

#--------------------------------------------------------------------------------------------------------------
# Main application.
#--------------------------------------------------------------------------------------------------------------
print('Running LWNX sample.')
os.chdir("/home/jpicciri/Documents/AutoNavTeamSoftware/")	# explicitly change working directory in the program

# NOTE: Using the SF45/B commands as detailed here: http://support.lightware.co.za/sf45b/#/introduction

# Make a connection to the com port.	
#serialPortName  = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0' # exact path of lidar when plugged in
serialPortName = '/dev/ttyACM0' 
#serialPortName = 'COM46'

serialPortBaudRate = 921600
port = serial.Serial(serialPortName, serialPortBaudRate, timeout = 0.1)

# Get product information.
response = executeCommand(port, 0, 0, timeout = 0.1)
print('Product: ' + readStr16(response))

#Enable/Disable scan
Enable1 = 1
#Enable1 = input('Enable (1)/Disable(0) scan : ')
Enable = int(Enable1)
executeCommand(port, 96, 1, [Enable])

# Configure Update Rate
        # https://support.lightware.co.za/sf45b/#/command_detail/command%20descriptions/66.%20update%20rate
        # Value can be one of:
        # 1	= 50 Hz
        # 2	= 100 Hz
        # 3	= 200 Hz
        # 4	= 400 Hz
        # 5	= 500 Hz
        # 6	= 625 Hz
        # 7	= 1000 Hz
        # 8	= 1250 Hz
        # 9	= 1538 Hz
        # 10 = 2000 Hz
        # 11 = 2500 Hz
        # 12 = 5000 Hz
Update1 = 12
#Update1 = input('Input a update rate (1...12) : ')
Update = int(Update1)
#print(Update)
#msbs,lsbs = Convert_speed(Update)
executeCommand(port, 66, 1, [Update])

# Set output to all information.
executeCommand(port, 27, 1, [8, 1, 0, 0])

if Enable == 1:
	# Set scan speed
	Speed1 = 5
	#Speed1 = input('Input a scan rate (5...2000) : ')
	Speed = int(Speed1)
	msbs,lsbs = Convert_speed(Speed)
	executeCommand(port, 85, 1, [lsbs, msbs])

	#Change high scan angle
	High1 = 160
	#High1 = input('Input a high scan angle (10...160) : ')
	High0 = int(High1)
	High = float_to_bin(High0)
	msbh,lsbh,dk1,dk2 = High_bin_to_Dec(High)
	executeCommand(port, 99, 1, [dk1,dk2,lsbh,msbh])

	#Change low scan angle
	Low1 = 160
	#Low1 = input('Input a low scan angle (10...160) : ')
	Low0 = int(Low1)
	Low = float_to_bin(-Low0)
	msbl,lsbl,dk3,dk4 = Low_bin_to_Dec(Low)
	executeCommand(port, 98, 1,[dk3,dk4,lsbl,msbl])
	response = executeCommand(port, 44,0)

	#Open file for writing
	filePath = "/home/jpicciri/Documents/AutoNavTeamSoftware/"
	fName = filePath + "deleteLaterTest.txt"
	#fName = input("Enter SF45 file name:  ")
	fHandle = open(fName, 'w')

	# Read distance data
	while True:
		if response != None:
			response = executeCommand(port, 44, 0)
			# variables must be consistent with command 27 parameters
			lastRaw, yawAngle = readSignalData(response)
			# process scan from high to low angle limits  
			if int(yawAngle) >= High0 - 1:
				print("")
				print("")
				temp = yawAngle
				while int(yawAngle) > -Low0 + 1:
					response = executeCommand(port, 44, 0)
					lastRaw, yawAngle = readSignalData(response)

					# skip over multiple angle values                        
					skipFlag = skip(yawAngle, temp)
					if skipFlag == True:
						scanDist.append(lastRaw)
						scanAngle.append(yawAngle)
						temp = yawAngle
						numPts += 1
		
				print("")
				print("")

				# print lists to shell after a scan
				for index in range(numPts):
					print(scanAngle[index], "  ", scanDist[index])

				# Create file of (scanAngle[], scanDist[])
				fHandle.write('File Name: ' + fName + ', ')
				fHandle.write('Update Rate: ' + str(Update1) + ', ')
				fHandle.write('Scan Rate: ' + str(Speed1) + ', ')
				fHandle.write('High Angle: ' + str(High1) + ', ')
				fHandle.write('Low Angle: ' + str(Low1) + ', ')
				fHandle.write('\n\n')

				fHandle.write('Angle | Distance\n')
				fHandle.write('================\n')
				for index in range(numPts):
					fHandle.write(str(scanAngle[index]))
					fHandle.write(', ')
					fHandle.write(str(scanDist[index]))
					fHandle.write('\n')

				# Process scan to detect objects                    
				print("", 'numPts = ', numPts)

				# filter scan for objects in detection range, separate, and characterize  
				objDetect.ObjectDetect(fHandle, numPts, minRange, maxRange, scanDist, scanAngle)
				# objDetect.ObjectDetect(numPts, minRange, maxRange, gapSize, scanDist, scanAngle)

				# Clear Lists + Close File 
				fHandle.close()  
				scanDist = []
				scanAngle = []

				# break from while true loop
				break
				


