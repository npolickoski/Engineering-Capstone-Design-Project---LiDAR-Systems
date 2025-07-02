"""
CPE 495/496 Senior Capstone Project: QuadRover Autonomous Navigation
Main Serial Communication Helper Functions
 
File Author: Nick Polickoski, njp0008
File Creation: 04/21/2025
"""

## Libraries
import time
import serial


## Function Definitions
def initSerialComms(portStr, baudRate, timeOut):
    """
    Initializes all serial communication with peripheral devices
    """
    comms = serial.Serial(port=portStr, baudrate=baudRate, timeout=timeOut)
    time.sleep(2) 
    comms.reset_input_buffer()
    return comms


def sendToArduino(serialCom, data):
    """
    Send a single ASCII character to the Arduino via UART.
    """
    if serialCom.is_open:
        serialCom.write(data.encode('utf-8'))                   # Send ASCII character
        print(f"Sent: {data}")                                  # Debugging output
        time.sleep(0.1)                                         # for serial comms stability


def recieveFromArduino(serialCom):
    """
    Recieve LiDAR mode flag from Odometry in Arduino Mega
    """
    if serialCom.is_open:
        packet = serialCom.readline().decode('utf-8').rstrip()                        
        print(f"Recieved: {packet}")                                  
        time.sleep(0.1)
        return packet                                         


def parseDataPacket(rawPacket):
    """
    Intakes recieved data and parses it based on its tag

    Tag Character Chart:
    L - Landmark GPS point from GUI
    N - number of Landmark points being sent over from GUI
    M - LiDAR data encoding mode
    C - Motor control charcater
    G - GPS data 
    
    """
    rawPacket = rawPacket.strip()
    if len(rawPacket) >= 3 and rawPacket[0] == rawPacket[-1]:  # Match framing characters
        tag = rawPacket[0]
        payload = rawPacket[1:-1]
        return tag, payload
    else:
        raise ValueError("Malformed Data Packet Recieved");

