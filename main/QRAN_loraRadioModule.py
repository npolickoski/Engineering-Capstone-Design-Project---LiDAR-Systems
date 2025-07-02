"""
CPE 495/496 Senior Capstone Project: QuadRover Autonomous Navigation
Main LoRa Radio Module Helper Functions
 
File Author: Nick Polickoski, njp0008
File Creation: 04/21/2025
"""

## Libraires
import time


## Function Definitions
def sendToLoRa(serialCom, message):
    """
    Transmits GPS data from Arduino Mega to External Laptop LoRa Module 
    """
    b = bytes(message,'utf-8')	            # convert string into bytes
    s = serialCom.write(b)		            # send the data to other lora
    time.sleep(1)                           # delay 1 sec


def recieveFromLoRa(serialCom):
    """
    Transmits GPS landmark points from Arduino Mega to External LoRa Module 
    """
    data_read = serialCom.readline()    # read data from other lora
    data = data_read.decode("utf-8")    # convert byte into string
    print(data)

