#--------------------------------------------------------------------------------------------------------------
# LightWare 2021
#--------------------------------------------------------------------------------------------------------------
# Description:
#	This samples communicates with the SF45.
#
# Notes:
#	Requires the pySerial module.
#--------------------------------------------------------------------------------------------------------------

import time
import serial
import numpy as np
import struct


#--------------------------------------------------------------------------------------------------------------
# LWNX library functions.
#--------------------------------------------------------------------------------------------------------------
_packet_parse_state = 0
_packet_payload_size = 0
_packet_size = 0
_packet_data = []




# Create a CRC-16-CCITT 0x1021 hash of the specified data.
def create_crc(data):
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
def build_packet(command, write, data=[]):
	payload_length = 1 + len(data)
	flags = (payload_length << 6) | (write & 0x1)
	packet_bytes = [0xAA, flags & 0xFF, (flags >> 8) & 0xFF, command]
	packet_bytes.extend(data)
	crc = create_crc(packet_bytes)
	packet_bytes.append(crc & 0xFF)
	packet_bytes.append((crc >> 8) & 0xFF)

	return bytearray(packet_bytes)

# Check for packet in byte stream.
def parse_packet(byte):
	global _packet_parse_state
	global _packet_payload_size
	global _packet_size
	global _packet_data

	if _packet_parse_state == 0:
		if byte == 0xAA:
			_packet_parse_state = 1
			_packet_data = [0xAA]

	elif _packet_parse_state == 1:
		_packet_parse_state = 2
		_packet_data.append(byte)

	elif _packet_parse_state == 2:
		_packet_parse_state = 3
		_packet_data.append(byte)
		_packet_payload_size = (_packet_data[1] | (_packet_data[2] << 8)) >> 6
		_packet_payload_size += 2
		_packet_size = 3

		if _packet_payload_size > 1019:
			_packet_parse_state = 0

	elif _packet_parse_state == 3:
		_packet_data.append(byte)
		_packet_size += 1
		_packet_payload_size -= 1

		if _packet_payload_size == 0:
			_packet_parse_state = 0
			crc = _packet_data[_packet_size - 2] | (_packet_data[_packet_size - 1] << 8)
			verify_crc = create_crc(_packet_data[0:-2])
			
			if crc == verify_crc:
				return True

	return False

# Wait (up to timeout) for a packet of the specified command to be received.
def wait_for_packet(port, command, timeout=1):
	global _packet_parse_state
	global _packet_payload_size
	global _packet_size
	global _packet_data

	_packet_parse_state = 0
	_packet_data = []
	_packet_payload_size = 0
	_packet_size = 0

	end_time = time.time() + timeout

	while True:
		if time.time() >= end_time:
			return None

		c = port.read(1)

		if len(c) != 0:
			b = ord(c)
			if parse_packet(b) == True:
				if _packet_data[3] == command:
					return _packet_data
				
# Send a request packet and wait (up to timeout) for a response.
def execute_command(port, command, write, data=[], timeout=1):
	packet = build_packet(command, write, data)
	retries = 4

	while retries > 0:
		retries -= 1
		port.write(packet)

		response = wait_for_packet(port, command, timeout)

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

#--------------------------------------------------------------------------------------------------------------
# SF45 API helper functions.
# NOTE: Using the SF45 commands as detailed here: https://support.lightware.co.za/sf45b/#/commands
#--------------------------------------------------------------------------------------------------------------
# Extract a 16 byte string from a string packet.
def get_str16_from_packet(packet):
	str16 = ''
	for i in range(0, 16):
		if packet[4 + i] == 0:
			break
		else:
			str16 += chr(packet[4 + i])

	return str16

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

def print_product_information(port):
	# https://support.lightware.co.za/sf45b/#/command_detail/command%20descriptions/0.%20product%20name
	response = execute_command(port, 0, 0, timeout = 0.1)
	print('Product: ' + get_str16_from_packet(response))

	# https://support.lightware.co.za/sf45b/#/command_detail/command%20descriptions/2.%20firmware%20version
	response = execute_command(port, 2, 0, timeout = 0.1)
	print('Firmware: {}.{}.{}'.format(response[6], response[5], response[4]))

	# https://support.lightware.co.za/sf45b/#/command_detail/command%20descriptions/3.%20serial%20number
	response = execute_command(port, 3, 0, timeout = 0.1)
	print('Serial: ' + get_str16_from_packet(response))
	time.sleep(1)

def set_update_rate(port, value):
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

	if value < 1 or value > 12:
		raise Exception('Invalid update rate value.')
	
	execute_command(port, 66, 1, [3]) # You can change the value inside [] to the desired 


def set_default_distance_output(port, use_last_return = False):
	# https://support.lightware.co.za/sf45b/#/command_detail/command%20descriptions/27.%20distance%20output
	if use_last_return == True:
		# Configure output to have 'last return raw' and 'yaw angle'.
		execute_command(port, 27, 1, [1, 1, 0, 0])
	else:
		# Configure output to have 'first return raw' and 'yaw angle'.
		execute_command(port, 27, 1, [8, 1, 0, 0])

def set_distance_stream_enable(port, enable):
	# https://support.lightware.co.za/sf45b/#/command_detail/command%20descriptions/30.%20stream
	if enable == True:
		execute_command(port, 30, 1, [5, 0, 0, 0])
		#Change high scan angle
		High1 = input('Input a high scan angle (10...160) : ')
		High = int(High1)
		High = float_to_bin(High)
		msbh,lsbh,dk1,dk2 = High_bin_to_Dec(High)
		execute_command(port, 99, 1, [dk1,dk2,lsbh,msbh])
		#Change low scan angle
		Low1 = input('Input a low scan angle (10...160) : ')
		Low = int(Low1)
		Low = float_to_bin(-Low)
		msbl,lsbl,dk3,dk4 = Low_bin_to_Dec(Low)
		execute_command(port, 98, 1,[dk3,dk4,lsbl,msbl])
	else:
		execute_command(port, 30, 1, [0, 0, 0, 0])

def wait_for_reading(port, timeout=1):
	# https://support.lightware.co.za/sf45b/#/command_detail/command%20descriptions/44.%20distance%20data%20in%20cm
	response = wait_for_packet(port, 44, timeout)
	
	if response == None:
		return -1, 0
	
	distance = (response[4] << 0 | response[5] << 8) / 100.0
	
	yaw_angle = response[6] << 0 | response[7] << 8
	if yaw_angle > 32000:
		yaw_angle = yaw_angle - 65535

	yaw_angle /= 100.0

	return distance, yaw_angle

def Convert_speed(num3):
        Speed1 = "{0:016b}".format(num3)
        s1 = Speed1[0:8]
        s2 = Speed1[8:16]
        msbs = int(s1, 2)
        lsbs = int(s2, 2)
        return msbs, lsbs

#--------------------------------------------------------------------------------------------------------------
# Main application.
#--------------------------------------------------------------------------------------------------------------
print('Running SF45/B LWNX sample.')

# Make a connection to the serial port.
# NOTE: You will need to change the port name and baud rate to match your connected SF45.
# Common Rapsberry Pi port name: /dev/ttyACM1
serial_port_name = '/dev/ttyACM0'
#serial_port_name = 'COM246'
serial_port_baudrate = 921600
sensor_port = serial.Serial(serial_port_name, serial_port_baudrate, timeout = 0.1)

# Get sensor information.
print_product_information(sensor_port)

# Configure the sensor.
# NOTE: See the set_update_rate function for values that can be used.
set_update_rate(sensor_port, 1)
set_default_distance_output(sensor_port)



# Start streaming distances.
set_distance_stream_enable(sensor_port, True)

# Set scan speed
Speed1 = input('Input a scan rate (5...2000) : ')
Speed = int(Speed1)
msbs,lsbs = Convert_speed(Speed)
executeCommand(port, 85, 1, [lsbs, msbs])

while True:
	distance, yaw_angle = wait_for_reading(sensor_port)

	if distance != -1:
		print('{} m {} deg'.format(distance, yaw_angle))
	else:
		print('No reading streamed.')
