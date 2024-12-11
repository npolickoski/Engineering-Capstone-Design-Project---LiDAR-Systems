#!/bin/bash

#
# QuadRover Autonomous Navigation Senior Capstone Project
# - Fall 2024 - Spring 2025
# - njp0008
#
# Service routine to run SF45 Lightware LiDAR Sensor at bootup
#   of the rover, and subsequently the Raspberry Pi 4b and LiDAR
#
# Edit Log
# - 12/09/2024: Created file; added delay, directory change, and python script running
# - 12/10/2024: Added error log to debug why program isn't writing data to file
# - 12/11/2024: Script runs LiDAR and writes data to file correctly
#


# Reboot Script Commands 
#set -x || exit 1		# displays echo statements when only the bash script is run
						# use when debugging

echo "Starting AutoNav Reboot Bash Script" || exit 1
sleep 10 || exit 1
cd /home/jpicciri/Documents/AutoNavTeamSoftware/ || exit 1
/usr/bin/python3 /home/jpicciri/Documents/AutoNavTeamSoftware/SF45pythonV9.py || exit 1
echo "AutoNavTeamSoftware_RebootScript Completed " || exit 1


