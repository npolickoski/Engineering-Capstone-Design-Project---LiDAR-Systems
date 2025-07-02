"""
CPE 495/496 Senior Capstone Project: QuadRover Autonomous Navigation
Main LiDAR Systems Program
 
File Author: Nick Polickoski, njp0008
File Creation: 03/25/2025

Change Logs:
===========
Obstacle Avoidance:
    - 01/30/2025:   - created function to parse encoded data via switch
                        case on encoded characters.
    - 01/31/2025:   - adjusted case statements and made test motor controls
    - 02/04/2025:   - rewrote encodeObstacle algorithm using desmos simulation:
                        - https://www.desmos.com/calculator/9l4twiwq04
    - 02/27/2025:   - integrated LiDAR Scan Data:
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
    - 03/25/2025:   - merged Obstacle Avoidance & Landmark Honing algorithms together
    - 03/26/2025:   - implemented LiDAR mode switching with Arudino serial comms recieving
    - 04/09/2025:   - added LoRa Module code for transmitting GPS dara from MEGA
                        - code from repo: https://github.com/sbcshop/Lora-HAT-for-Raspberry-Pi/tree/main
    - 04/20/2025:   - completed integration with Arduino Mega navigation control program 
    - 04/26/2025:   - completed all delimted encoding/decoding of serially
                        communicated data packets on both Raspberry Pi 4B
                        and Arduino Mega end
    - 04/26/2025:   - implemented logging file function to find integration weak points in code
    - 04/28/2025:   - implemented logging library instead of file logging
                    - changed up data parsing and concatonating between serial connections with arduino
"""             

## External Libraries
import serial
import queue
import logging
import time

## Project Libraries
import QRAN_LiDARsetup as QRANlidarSetup
import QRAN_lidarDataAlgorithms as QRANlidarData
import QRAN_serialComms as QRANSerial
import QRAN_loraRadioModule as QRANLora

## UART Serial Communication Ports
PORT_ARDUINO = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_74934303030351615052-if00' 
PORT_LIDAR = '/dev/serial/by-id/usb-LightWare_Optoelectronics_lwnx_device_38S45-15306-if00'

## Logging Configuration
logging.basicConfig(
    level = logging.INFO,
    format = '%(asctime)s - %(levelname)s - %(message)s',
    handlers = [
        logging.FileHandler('quadrover.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

## Function Definition
def main():
    """
    Main Driver Function
    - Order of Plugging in Devices Upon Raspberry Pi Bootup:
        1) Arudino Mega
        2) SF45 Lightware LiDAR
    """
    # Initialize Serial Comms for Arduino MEGA, SF45 Lightware LiDAR, and LoRA Module
    arduino = QRANSerial.initSerialComms(PORT_ARDUINO, 9600, 1)  
    lidar = QRANSerial.initSerialComms(PORT_LIDAR, 921600, 0.1) 
    lora = serial.Serial(port='/dev/ttyS0', baudrate=9600, parity=serial.PARITY_NONE, 
                        stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)

    # Initialize LiDAR System
    enable = 1                                                  # enable scan (1 - yes, 0 - no)
    update = 12                                                 # update rate between 1-12
    speed = 10                                                  # speed between 5-2000
    angleH = 160                                                 # high angle from 10-160
    angleL = 160                                                 # low angle from 10-160
    QRANlidarSetup.initLiDARSystem(lidar, enable, update, speed, angleH, angleL)

    # Initialize GPS Landmark Points
    try:
       # Wait for GUI to Send Over Landmark Points
       while lora.in_waiting <= 0:
           continue

       # Send Points to Navigation in Arduino Mega
       numPoints = QRANLora.recieveFromLoRa(lora)
       QRANSerial.sendToArduino(arduino, 'N' + numPoints + 'N')         # send over # of points
       for point in range(numPoints):
           landmarkPoint = QRANLora.recieveFromLoRa(lora)
           QRANSerial.sendToArduino(arduino, 'L' + landmarkPoint + 'L') # send over points themselves
    except serial.SerialException as err:
       print(f"\nLoRa Serial Error: {err}\n")


    # Wait for Calibration to Finish Before Entering LiDAR System Modes
    while arduino.is_open:
        logger.info("Waiting to Enter LiDAR Systems")
        line = arduino.readline().decode('utf-8').strip()
        if line:
            tag, packet = QRANSerial.parseDataPacket(line)
            logger.info(f"Tag: {tag}\nPacket: {packet}")
            if tag == 'C': 
                if int(packet) == 1:
                    logger.info("Entering LiDAR Systems Now")
                    break
            elif tag == 'G':
                loraSend_packet = 'G' + packet
                print(loraSend_packet)
                QRANLora.sendToLoRa(lora, loraSend_packet)   # send over GPS point to lora 
                lora.flush()
                

    # LiDAR Data Processing
    try: 
        # Initialize Loop Parameters
        mode = 0                        # initialize mode to Stand-By
        isObstacleDetected = 'N'        # if an obstacle is detected flag
        encodedData = 'N'               # initialize lidar data algorithm variable
        
        # Main Data Recieve/Transmit Loop
        while True:
            # Non-Response Guard Clause
            response = QRANlidarSetup.executeCommand(lidar, 44, 0)
            if response == None:
                continue

            # Processing Incoming Packets from Arduino Mega
            if arduino.in_waiting > 0:
                logger.info("Received from Arduino")
                line = arduino.readline().decode('utf-8').strip()
                if line:
                    tag, packet = QRANSerial.parseDataPacket(line)
                    logger.info(f"Tag: {tag}\nPacket: {packet}")
                    if tag == 'M':
                        mode = int(packet)                   # mode determined by data recieved
                        logger.info(f"Mode: {mode}")
                    elif tag == 'O':                         # reset obstacle detection flag
                        isObstacleDetected = 'N'
                    elif tag == 'G':
                        loraSend_packet = 'G' + packet
                        print(loraSend_packet)
                        QRANLora.sendToLoRa(lora, loraSend_packet)   # send over GPS point to lora   
                        lora.flush()

            # Encode LiDAR Data (0 - Obstacle Avoidance, 1 - Landmark Honing)
            d, theta = QRANlidarSetup.readSignalData(response)

            # Obstacle Avoidance Mode
            if QRANlidarData.isObstacleDetected(d, theta, isObstacleDetected, logger):
                logger.info("Entering Obstacle Avoidance")
                isObstacleDetected = 'D'
                encodedData = QRANlidarData.encodeObstacleAvoidance(d, theta)
                if encodedData == None:
                    continue

                # Send to Arduino
                arduinoSend_DetectFlag = 'C' + isObstacleDetected + 'C'
                arduinoSend_Distance = 'C' + str(d) + 'C'
                arduinoSend_encodedData = 'C' + str(encodedData) + 'C'
                QRANSerial.sendToArduino(arduino, arduinoSend_DetectFlag)
                QRANSerial.sendToArduino(arduino, arduinoSend_Distance)
                QRANSerial.sendToArduino(arduino, arduinoSend_encodedData)

                # Send to LoRa
                loraSend_Distance = 'O' + str(d) + ' '
                QRANLora.sendToLoRa(lora, loraSend_Distance)
                
                # End Obstacle Avoidance
                mode = 0 # reset mode to base case
                

            # Landmark Honing Mode
            elif mode == 1:
                logger.info("Entering Landmark Honing")
                encodedData = QRANlidarData.encodeLandmarkHoning(d, theta)

                # Send to Motor Controls
                time.sleep(2)                   
                logger.info(f"Encoded Nav Command: {encodedData}")
                motorCommand = 'C' + str(encodedData) + 'C'
                QRANSerial.sendToArduino(arduino, motorCommand)

            # LiDAR in Stand-By Mode
            elif mode == 0:
                continue

            
    # Error Handling
    except KeyboardInterrupt:
        logger.info("Exiting Program")
    except ValueError as vErr:
        logger.error(f"Value Error: {str(vErr)}")
    except queue.Empty as qErr:
        logger.error(f"Queue Empty Error: {str(qErr)}")
    except (OSError, IOError) as fileErr:
        logger.error(f"File Operation Error: {str(fileErr)}")
    finally:
        # Making Sure to Close All Serial Connections
        try:
            arduino.close()
            lidar.close()
            lora.close()
        except Exception as e:
            logger.error(f"Error closing serial connections: {str(e)}")


## Call to Main 
if __name__ == "__main__":    
    main()

