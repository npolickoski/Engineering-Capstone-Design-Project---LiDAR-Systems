#!/bin/bash

#
# QuadRover Autonomous Navigation Senior Capstone Project
# - Fall 2024 - Spring 2025
# - njp0008, 12/09/2024
#
# Service routine to run SF45 Lightware LiDAR Sensor at bootup
#   of the rover, and subsequently the Raspberry Pi 4b and LiDAR
#

# Commands
sleep 30 || exit 1
cd /home/jpicciri/Documents/AutoNavTeamSoftware || exit 1
/usr/bin/python3 /home/jpicciri/Documents/AutoNavTeamSoftware/SF45pythonV9.py || exit 1
echo "AutoNavTeamSoftware_RebootScript started successfuly" || exit 1

