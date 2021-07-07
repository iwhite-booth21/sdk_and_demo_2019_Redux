#===========================================================================================================================
# Section One - Setting up the Decision and Creating our Baseline for avoidning Obstacles via situational awareness
#===========================================================================================================================

# Multiple functions will use the readings from the sensors

# Let create some placeholder values for the collision avoidance items

# Merging the code created by MSB with the collision avoidance functions requires alot of imports
import time, sys
import ps_drone
from time import sleep
import signal
import os
import sys
import math
import matplotlib.pyplot as plt
import numpy as np
import board
import busio
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag

i2c = busio.I2C(board.SCL, board.SDA)
mag = adadruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)





# I suggest a few bool functions to be our checkers

def FlightDecision(choice):
    if(choice == "Yes"):
        print("Parrot Drone beginning takeoff")
        return True
    elif(choice=="No"):
        print("Entering Rest Mode")
        return False
    
    else:
        print("Invalid Response")
        return False
    

# A variable that will be used to create the trigger for the aforemention function
flightchoice = input("Whats your choice Y or N: ")


# Acceleromter Reading Data
def accelData(mag_x,mag_y,mag_z):
    # X, Y, and Z acceleration information
    accel_x = accel.acceleration[0]
    accel_y = accel.acceleration[1]
    accel_y = accel.acceleration[2]
    
    
    # Magnetomter Information
    mag_x = float(mag_x)
    mag_y = float(mag_y)
    mag_z = float(mag_z)
    
    try:
        thetaAcc = math.pi - math.asin(accel_z/g)
        phiAcc = math.atan2(mag_y,mag_z)
        return thetaAcc, phiAcc
    # For a bad/false reading
    except ValueError:
        return 0,0

    

    
    
    
    
    
    
# Emergency land for the Drone, must be done in terminal
def exit_gracefully(signal, frame):
    drone.shutdown()
signal.signal(signal.SIGQUIT, exit_gracefully)




# Get position information from UAV
def droneLocation(dist,mag_x,mag_y,mag_z):
    theta, pi = accelData(mag_x,mag_y,mag_z)
    x = dist * math.cos((math.pi/2) - phi)
    y = dist * math.sin((math.pi/2) - phi)
    droneTuple = (x,y)
    return droneTuple


list = []


# min distance that the UAV can fly through
marginDist = 20   # Centimeters



# Cartesian Converter
def calculateCoordinate(dist, theta, phi):
    x = dist * math.cos((math.pi/2) - phi)
    y = dist * math.sin((math.pi/2) - phi) 
    tuple = (x, y)
    list.append(tuple)

# magnetomter data
# a list of mag data 
def magData(magnetomerData):
    magnetometerData = magnetometerData[1:-1]
    x, y, z = magnetometerData.split(',')
    return x, y, z

def distanceLibrary(distances):
    mydistances = []
    # for every distance read, place it in a list
    for distance in TFMini_distance:
        mydistances.append(distance) 
    # Returns the list of distances to be looked through
    # will be converted to float
    return mydistances




# Check the collision distance
def CollisionMenuCheck(result):
    
    #Possible line holder, to do this we need a "imaginary" runtime to accomodate for the sensors being active
    # Sample point array (Horizonal Array / Vector)
    TFMini_distances = []
    # myCoords -> Use the values from pi readings so myCoords = []
    
    # We will use append to build our collection of points
    vectorOne = np.array(TFMini_distances)

    # Some values that should be used to control the duration of the program
    # Currently not used in favor of a list approach
    
    # Set a min/collision distance max range is 1200 centimeters, blind range occurs is distances 
    coldist = 200
    if(result == True):
        for thisdis in vectorOne:
            if(thisdis < coldist):
                # Code for turning drone and moving in a a direction
                print("Obstacle Detected, beginnning manuever")
                
            elif(thisdis > coldist):
                #FLight remains normal
                print("I am far enough to breathe: No issue")

            else:
                print("Critical Error Occured") 
        thisdecision = readyUp




# Runs the program (Subject to be moved to bottommost area of the code)
CollisionMenuCheck(readyUp)

#===========================================================================================================================
# Section Two - Focusing on our line edge detection and developing our avoidance manuevers
#===========================================================================================================================
# This section will add new ideas to the concept for the of our collision avoidance algs
# First we will choose an arbitrary angle (45 Degrees, which can be in either direction) -> Manuever purposes
# Second we will build our line through the use of for loops and if statements
# Lastly we will work on artificial potential fields 

# We will use this to get around our obstacle OR as a value to test in if statement.
# Its likely we will need to account for 45.00 degrees in the left direction or -45.00
# collisionDodgeAngle = 45.00
# Unsure of how to conduct that with our positive setup, however we could directional arrays that will hold the distances in both directions.
# Possible double array function.

# A function that loops through the points, and builds a array of closely related plots.
# Preferably less than 2cm apart
# A function within MSB code exists already
# Lets use the RPLidar information as a test //
def lineDetector(Cartesian_Coordinates):
    # Where the line information is stored
    # "Line Drawer" using the information stored within, easiest use with X & Y coordinates
    lineFormation = []
    # minimal distance is 2 centimeters
    mindist = 25
    # Find closely alligned coordinates, then start to put them into a list
    for myInitialCoord in Cartesian_Coordinates:
        for mySecondaryCoord in Cartesian_Coordinates:
            # Take the absolute value for negative values
            if (abs(myInitialCoord-mySecondaryCoord) == 0):
                continue
            # if the distance between coordinates are less than 25, collect the two points
            if (abs(myInitialCoord-mySecondaryCoord) < mindist):
                lineFormation.append((myInitialCoord, mySecondaryCoord))
            else:
                # Some generic code
                continue
    return lineFormation
    
# Using the information of the line, utilize the sorted angle list
# Focus on the beginning and endoints of the lines
# These decide where we exit the obstacle
def conductManuever(line_angles):
    # Arbitrary Angle for dodging
    collisionDodgeAngle = 45.00
    # When the smallest angle is used to exit
    if(line_angles[0][0] > collisionDodgeAngle):
        # Add that smallest angle to the collisionDodgeAngle
        collisionDodgeAngle = collisionDodgeAngle + line_angles[0][0]
    # When the largest angle is used to exit
    elif(line_angles[-1][-1] > collisionDodgeAngle):
        # Add the largest angle to the collisionDodgeAngle
        collisionDodgeAngle = collisionDodgeAngle + line_angles[-1][-1]
        
    # Return the angle to be used later    
    return collisionDodgeAngle



# Compares every element in a list with one another to see if they are too close for the drone to fit through
def line_edge(coordinateList):
    j = 0
    # iterate through each tuple in list and compares it to every other tuple
    # every tuple is compared to all other tuples in the list
    for aTuple in coordinateList:
        print("comparisons with " , aTuple)
        for otherTuples in coordinateList:
            dist = distBetweenPoints(aTuple, otherTuples)
            print(j)
            j = j + 1
            if dist < marginDist and aTuple != otherTuples:
                print(dist)
                print("Too Close")
    
    
# This will run the line edge function 
def distTooClose():
    line_edge(coordinateList)


#===========================================================================================================================
# Section Three - Flight Controller
#===========================================================================================================================
                                
#plotPoints(list)




def onBoardFlightSystem(Switch):
    if(Switch == True):
        # Trigger all functions from this function with all relevant information
        print("Initializing")
        drone = ps_drone.Drone()   # Initialize the PS-DRONE-API
        print("Starting")
        drone.startup()            # Connect to drone and start subprocesses
        print("Resetting")
        drone.reset()
        # Wait until Drone completes its reset
        while drone.getBattery()[0] == -1: 
            time.sleep(0.1)
        # Get Drones Battery Status
            print("Battery: " + str(drone.getBattery()[0]) + "% " + str(drone.getBattery()[1])) 
            drone.useDemoMode(False)
        # Packets that will be decoded
            drone.getNDpackage(["demo", "pressure_raw", "altitude", "magneto", "wifi", "wind_speed", "euler_angles"])
            time.sleep(0.5)

        # i is the number of the data points to be captured in point cloud, choosen randomly
        i = 50
        j = 0 
    
        while i > 0:
        # Reads terminal program information for the TFMini (Python2 Programm TFMINI.py)
            distanceStr = os.popen('./tfminiTEST.py').read()
            magnetometerData = str(drone.NavData["magneto"][0])
            mag_x, mag_y, mag_z = magData(magnetomterData)
    
        # Gravity
            g = 9.81
            k = 0
        while k < 1:
            theta, phi = accelData(mag_x, mag_y, mag_z)
            k = k + 1
        break
        
        # Only uses the measurements when all three (distance, theta, phi) are all valid
        if len(distanceStr) > 1 and distanceStr != "65535" and theta != "0" and phi != "0":
            distSplit = distanceStr.split("\n")[0]
            distance = float(distSplit)
        # Exception Handling for when there is no acc[1], when no value is captured for phi 
            print(j)
            j = j + 1
            calculateCoordinates(distance, theta, phi)
        i = i - 1
    
        # Coordinates of the front of the UAV
        dist_drone_to_lidar = 5
        magnetometerData = str(drone.NavData["magneto"][0])
        mag_x, mag_y, mag_z = magData(magnetometerData)
        droneTuple = droneLocation(dist_drone_to_lidar, mag_x, mag_y, mag_z)
        
        
        readings_list = lineDetector(list)

        
        # Use TF for distance list
        drone_distance = CollisionMenuCheck(TFMini_Distances)

        
        plotPoints(list)
        
    return 0











# This will hold the value returned from the function, will be used to trigger the CollisionMenuCheck
# Merger of codes should result in a starter for the program if True
readyUp = FlightDecision(flightchoice)

# Trigger for main function that handles entire program, this can be done away with after testing
switcher = CollisionMenuCheck(readyUp)

# Main function for this program
onBoardFlightSysterm(switcher)

