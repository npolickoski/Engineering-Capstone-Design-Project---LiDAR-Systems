"""
CPE 495/496 Senior Capstone Project: QuadRover Autonomous Navigation
Main LiDAR Data Algorithms

File Author: Nick Polickoski, njp0008
File Creation: 04/21/2025

Background Info:
- Desmos Model: https://www.desmos.com/calculator/9l4twiwq04
"""

## Obstacle Detection Thresholds (in cm)
DISTANCE_EDGE = 600  
DISTANCE_SAFE = 400
DISTANCE_DANGER = 200

## Angle Value Thresholds (in degrees) (see Desmos model as well)
ANGLE_DANGER = 17.458
ANGLE_SAFE   = 8.627
ANGLE_EDGE   = 5.739


## Function Definitions
def isObstacleDetected(d, theta, isObstacleDetected, logger):
    """
    First condition check if an obstacle has been detected
    """
    # First Detection of Obstacle
    logger.info("isObstacleDetected Function")
    if isObstacleDetected == 'N':
        if DISTANCE_SAFE < d and d <= DISTANCE_EDGE:
            if -ANGLE_DANGER <= theta and theta <= ANGLE_DANGER:
                logger.info("obstacle")
                return True
    logger.info("no obstacle")
    return False
                

def encodeObstacleAvoidance(d, theta):
    """
    Encode the obstacle position based on distance (d) and angle (theta).
    Returns an ASCII character of which direction to turn based on 
    detected angle

    Character Chart:
    S - stop if obstacle makes it to Danger Zone, reverse until obstacle in Safe Zone
    L - turn left
    R - turn right
    N - no obstacle detected, continue forward

    """
    # First Detection of Obstacle
    if DISTANCE_SAFE < d and d < DISTANCE_EDGE - 50:
        if -ANGLE_DANGER <= theta and theta <= ANGLE_DANGER:
            if theta > 0:                                       # if on right side, turn left
                return 'L'
            elif theta <= 0:                                    # if on left side OR in the middle, turn right
                return 'R'


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
    O - base case -> no change to motor commands

    """
    threshold = 3
    # When Landmark is within Danger Zone of the Rover
    if d <= DISTANCE_DANGER:
        return 'A'                                              # stop at arrived destination (the landmark)

    # Check Which Direction Motor Controls Needs to Be
    if DISTANCE_DANGER < d and d <= DISTANCE_EDGE:
        if theta > threshold:                                       # if on right side
            return 'R'
        elif theta < -threshold:                                    # if on left side OR in the middle
            return 'L'
        elif theta >= -threshold & theta <= threshold:
            return 'N'
            
    return 'O'

