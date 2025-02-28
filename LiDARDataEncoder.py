"""
CPE495 Senior Capstone Project: QuadRover Autonomous Navigation
Encoded LiDAR Data Parser V1
 
File Author: Nick Polickoski, njp0008
File Creation: 01/30/2025

Changes Log: 
    - 01/30/2025:  - Created function to parse encoded data via switch
                     case on encoded characters.
    - 01/31/2025:  - Adjusted case statements and made test motor controls
    - 02/04/2025:  - Rewrote encodeObstacle algorithm using desmos simulation
                        https://www.desmos.com/calculator/9l4twiwq04
    - 02/272025:   - Integrated LiDAR Scan Data
                        - created initSerialComms() to initialize multiple
                          serial communications
                            - Serial comms info: https://rfc1149.net/blog/2013/03/05/what-is-the-difference-between-devttyusbx-and-devttyacmx/
                        - created initLiDARSystem() to initialize all
                          LiDAR system values
"""                   

# Libraries
import serial
import time
import LiDARObstalceAvoidance as obsAvoid


## Define obstacle detection thresholds (in cm)
DISTANCE_EDGE = 600   
DISTANCE_SAFE = 400
DISTANCE_DANGER = 200
PROTECTION_ZONE = 120       # 60cm rover width + 2*30cm on each side of rover

# Angle Values from Model (inn degrees)
ANGLE_DANGER = 17.458
ANGLE_SAFE   = 8.627
ANGLE_EDGE   = 5.739


## Function Definitions
def encodeObstacle(d, theta):
    """
    Encode the obstacle position based on distance (d) and angle (theta).
    Returns an ASCII string corresponding to how the rover should respond
        in motor control functions in the Arduino MEGA

    Character Chart:
    S - stop if obstacle makes it to Danger Zone, reverse until obstacle in Safe Zone
    R - if obstacle on left, turn right
    L - if obstacle on right, turn left
    N - no obstacle detected (for debugging purposes only)

    """
    # If obstacle makes it to the Danger Zone
    if d <= DISTANCE_DANGER:
        if -ANGLE_DANGER <= theta and theta <= ANGLE_DANGER:
            return 'S'       # stop and reverse to find a new way 

    # Check Which Direction Motor Controls Needs to Be
    if DISTANCE_DANGER < d and d <= DISTANCE_EDGE:
        if -ANGLE_DANGER <= theta and theta <= ANGLE_DANGER:
            if theta > 0:               # if on right side
                return 'L'
            elif theta <= 0:            # if on left side OR in the middle
                return 'R'
    
    return 'N'


def sendToArduino(serialCom, data):
    """
    Send a single ASCII character to the Arduino via UART.
    """
    if serialCom.is_open:
        serialCom.write(data.encode())  # Send ASCII character
        print(f"Sent: {data}")  # Debugging output


def main():
    """
    Main Driver Function
    - Order of Plugging in Devices Upon Raspberry Pi Bootup:
        1) Arudino Mega
        2) SF45 Lightware LiDAR
    """
    # Define Serial Comms for Arduino MEGA & SF45 Lightware LiDAR
    arduino = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_8513831303435110D051-if00', 115200, timeout=1) 
    time.sleep(2)   
    lidar = serial.Serial('/dev/serial/by-id/usb-LightWare_Optoelectronics_lwnx_device_38S45-15306-if00', 921600, timeout=0.1)
    time.sleep(2) 

    # Initialize LiDAR System
    obsAvoid.initLiDARSystem(lidar)

    # LiDAR Data Processing 
    response = obsAvoid.executeCommand(lidar, 44,0)
    while True:
        # Non-Response Gaurd Clause
        if response == None:
            continue

        # Retrieve LiDAR Data
        response = obsAvoid.executeCommand(lidar, 44, 0)
        d, theta = obsAvoid.readSignalData(response)

        # Encode and Send Data to Mega
        encodedData = encodeObstacle(d, theta)
        print(encodedData)                      # for testing purposes
        sendToArduino(arduino, encodedData)
        time.sleep(0.1)  # Small delay for serial communication stability



## Call to Main 
main()

