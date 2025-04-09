"""
CPE495 Senior Capstone Project: QuadRover Autonomous Navigation
Main LiDAR Systems Program
 
File Author: Nick Polickoski, njp0008
File Creation: 03/25/2025

Change Logs
===========
Obstacle Avoidance:
    - 01/30/2025:   - Created function to parse encoded data via switch
                        case on encoded characters.
    - 01/31/2025:   - Adjusted case statements and made test motor controls
    - 02/04/2025:   - Rewrote encodeObstacle algorithm using desmos simulation:
                        - https://www.desmos.com/calculator/9l4twiwq04
    - 02/27/2025:   - Integrated LiDAR Scan Data:
                        - created initSerialComms() to initialize multiple
                          serial communications
                            - Serial comms info: https://rfc1149.net/blog/2013/03/05/what-is-the-difference-between-devttyusbx-and-devttyacmx/
                        - created initLiDARSystem() to initialize all
                          LiDAR system values
    - 03/14/2025:   - fixed serial queueing issues via:
                        - changing Arduino serial comms baud rate from 115200 to 921600
                        - created a thread to handle incoming LiDAR data
                        - created queue to hold data across threads
    - 03/22/2025:   - reverted following serial communication changes due to not fixing issue:
                        - threading to handle incoming LiDAR data
                        - queue to hold data across threads

Landmark Honing:
    - 03/15/2025:   - copied major functionality of from Obstacle Avoidance Data Encoder
                    - began written honing algorithm
    - 03/17/2025:   - finished algorithm and kept threading/baud rate changes from original file
    - 03/22/2025:   - reverted serial communication changes due to not fixing issue:
                        - threading to handle incoming LiDAR data
                        - queue to hold data across threads

Main:
    - 03/25/2025:   -  Merged Obstacle Avoidance & Landmark Honing algorithms together
    - 03/26/2025:   - implemented LiDAR mode switching with Arudino serial comms recieving
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
PORT_ARDUINO= '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_8513831303435110D051-if00'
PORT_LIDAR = '/dev/serial/by-id/usb-LightWare_Optoelectronics_lwnx_device_38S45-15306-if00'



## Function Definitions
def encodeObstacleAvoidance(d, theta):
    """
    Encode the obstacle position based on distance (d) and angle (theta).
    Returns an ASCII string corresponding to how the rover should respond
    in motor control functions in the Arduino MEGA

    Character Chart:
    S - stop if obstacle makes it to Danger Zone, reverse until obstacle in Safe Zone
    L - turn left
    R - turn right
    N - no obstacle detected, continue forward

    """
    # If obstacle makes it to the Danger Zone
    if d <= DISTANCE_DANGER:
        return 'S'                                              # stop and reverse to find a new way 

    # Check Which Direction Motor Controls Needs to Be
    if DISTANCE_DANGER < d and d <= DISTANCE_EDGE:
        if -ANGLE_DANGER <= theta and theta <= ANGLE_DANGER:
            if theta > 0:                                       # if on right side, turn left
                return 'L'
            elif theta <= 0:                                    # if on left side OR in the middle, turn right
                return 'R'
    
    # If No Obstacle in the Protection Zone
    return 'N'


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
    comms.reset_input_buffer()
    return comms


def sendToArduino(serialCom, data):
    """
    Send a single ASCII character to the Arduino via UART.
    """
    if serialCom.is_open:
        serialCom.write(data.encode())                          # Send ASCII character
        print(f"Sent: {data}")                                  # Debugging output
        time.sleep(0.1)                                         # for serial comms stability


def recieveFromArduino(serialCom):
    """
    Recieve LiDAR mode flag from Odometry in Arduino Mega
    """
    if serialCom.is_open:
        mode = serialCom.readline().decode('utf-8').rstrip()                        
        print(f"Recieved: {mode}")                                  
        time.sleep(0.1)
        return mode                                         


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

            # Encode LiDAR Data (1 - Obstacle Avoidance, 0 - Landmark Honing)
            if mode == 1:
                encodedData = encodeObstacleAvoidance(d, theta)
            elif mode == 0:
                encodedData = encodeLandmarkHoning(d, theta)
                
            print(encodedData)                              # for debugging purposes
            sendToArduino(arduino, encodedData)
            
    except KeyboardInterrupt:
        print("\nExiting Program\n")



## Call to Main 
if __name__ == "__main__":    
    main()

