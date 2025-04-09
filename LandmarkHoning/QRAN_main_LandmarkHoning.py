"""
CPE495 Senior Capstone Project: QuadRover Autonomous Navigation
Landmark Honing Data Encoder
 
File Author: Nick Polickoski, njp0008
File Creation: 03/15/2025

Change Log: 
    - 03/15/2025:   - copied major functionality of from Obstacle Avoidance Data Encoder
                    - began written honing algorithm
    - 03/17/2025:   - finished algorithm and kept threading/baud rate changes from original file
    - 03/22/2025:   - reverted serial communication changes due to not fixing issue:
                        - threading to handle incoming LiDAR data
                        - queue to hold data across threads
"""                   

## Libraries
import time
import serial
import QRAN_LiDARsetup as obsAvoid



## Define Obstacle Detection Thresholds (in cm)
DISTANCE_EDGE = 243.84  
DISTANCE_SAFE = 162.56
DISTANCE_DANGER = 81.28
PROTECTION_ZONE = 120               # 60cm rover width + 2*30cm on each side of rover

## Angle Values from Model (inn degrees)
ANGLE_DANGER = 17.458
ANGLE_SAFE   = 8.627
ANGLE_EDGE   = 5.739

## UART Serial Communication Ports
PORT_ARDUINO = '/dev/serial/by-id/usb-LightWare_Optoelectronics_lwnx_device_38S45-15306-if00'
PORT_LIDAR = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_8513831303435110D051-if00'



## Function Definitions
def encodeLandmarkHoning(d, theta):
    """
    Encode the landmark position based on distance (d) and angle (theta).
    Returns an ASCII string corresponding to how the rover should respond
    in motor control functions in the Arduino MEGA

    Character Chart:
    A - arrived at landmark
    L - turn left
    R - turn right
    N - continue forward

    """
    # When Landmark is within Danger Zone of the Rover
    if d <= DISTANCE_DANGER:
        return 'A'                                              # stop at arrived destination (the landmark)

    # Check Which Direction Motor Controls Needs to Be
    if DISTANCE_DANGER < d and d <= DISTANCE_EDGE:
        if -ANGLE_DANGER <= theta and theta <= ANGLE_DANGER:
            if theta > 0:                                       # if on right side
                return 'R'
            elif theta <= 0:                                    # if on left side OR in the middle
                return 'L'
    
    # Continue moving forward
    return 'N'


def initSerialComms(portStr, baudRate, timeOut):
    """
    Initializes all serial communication with peripheral devices
    """
    comms = serial.Serial(port=portStr, baudrate=baudRate, timeout=timeOut)
    time.sleep(2) 
    return comms


def sendToArduino(serialCom, data):
    """
    Send a single ASCII character to the Arduino via UART.
    """
    if serialCom.is_open:
        serialCom.write(data.encode())                          # Send ASCII character
        print(f"Sent: {data}")                                  # Debugging output
        time.sleep(0.1)                                         # for serial comms stability


def main():
    """
    Main Driver Function
    - Order of Plugging in Devices Upon Raspberry Pi Bootup:
        1) Arudino Mega
        2) SF45 Lightware LiDAR
    """
    # Define Serial Comms for Arduino MEGA & SF45 Lightware LiDAR 
    arduino = initSerialComms(PORT_ARDUINO, 921600, 1)  
    lidar = initSerialComms(PORT_LIDAR, 921600, 0.1) 


    # Initialize LiDAR System
    enable = 1                                                  # enable scan (1/0)
    update = 12                                                 # update rate between 1-12
    speed = 15                                                  # speed between 5-2000
    angleH = 25                                                 # high angle from 10-160
    angleL = 25                                                 # low angle from 10-160
    obsAvoid.initLiDARSystem(lidar, enable, update, speed, angleH, angleL)


    # LiDAR Data Processing
    try: 
        response = obsAvoid.executeCommand(lidar, 44,0)

        while True:
            # Non-Response Gaurd Clause
            if response == None:
                continue

            # Retrieve LiDAR Data
            response = obsAvoid.executeCommand(lidar, 44, 0)
            d, theta = obsAvoid.readSignalData(response)

            # Encode and Send Data to Mega
            encodedData = encodeLandmarkHoning(d, theta)
            print(encodedData)                              # for debugging purposes
            sendToArduino(arduino, encodedData)

    except KeyboardInterrupt:
        print("\nExiting Program\n")



## Call to Main 
if __name__ == "__main__":    
    main()

