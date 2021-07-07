#===========================================================================================================================
# Section One - Setting up the Decision and Creating our Baseline for avoidning Obstacles via situational awareness
#===========================================================================================================================

# Multiple functions will use the readings from the sensors

# Let create some placeholder values for the collision avoidance items
# Incorporating info from RPLidar Scan

import numpy as np
import os
import sys
import matplotlib.pyplot as plt
import csv
import math
xcoord = []
ycoord = []
coords = []
count = 0

with open('CartesianPoints.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for line in readCSV:
       #print(line)
       #print(count)
       #tuple = (line[0], line[1])
        coords.append(tuple)
        xcoord.append(line[0])
        ycoord.append(line[1])
        coords.append((line[0], line[1]))
        count = count + 1






# This will be expanded to incorporate the specific algorithms necessary to controll drone flight subroutines and also algorithm specific info

# I suggest a few bool functions to be our checkers

def FlightDecision(choice):
    if(choice == "Yes"):
        print("Lets begin the example")
        return True
    elif(choice=="No"):
        print("Thats too bad")
        return False
    
    else:
        return False
    

# A variable that will be used to create the trigger for the aforemention function
flightchoice = input("Whats your choice Y or N: ")

# We can use a trigger inplace of a input trigger
# TFMINI.dist && Accelerometer/Magnetometer 
# Possibly trying accelData.angles and MagData.angles



# This will hold the value returned from the function, will be used to trigger the CollisionMenuCheck
readyUp = FlightDecision(flightchoice)


#
def CollisionMenuCheck(result):
    
    #Possible line holder, to do this we need a "imaginary" runtime to accomodate for the sensors being active
    # Sample point array (Horizonal Array / Vector)
    mySample = [10000, 200, 6 , 199, 200, 378, 9000, 1200]
    # myCoords -> Use the values from pi readings so myCoords = []
    
    # We will use append to build our collection of points
    vectorOne = np.array(mySample)

    # Some values that should be used to control the duration of the program
    # Currently not used in favor of a list approach
    
    # Set a min/collision distance
    coldist = 400
    if(result == True):
        for thisdis in vectorOne:
            if(thisdis < coldist):
                # Simulate Alg1
                print("I am too close: I need to move within artificial potential fields alg")
                #return True
                # mycharge = negative_c
            elif(thisdis > coldist):
                print("I am far enough to breathe: No issue")
                #return True
            else:
                print("Critical Error Occured") 
                #return False
        thisdecision = readyUp

        #return False



# Runs the program (Subject to be moved to bottommost area of the code)
CollisionMenuCheck(readyUp)

#===========================================================================================================================
# Section Two - Focusing on our line edge detection and developing our avoidance manuevers
#===========================================================================================================================

#Lets see if the information from the prior cell can be interacted with
# print(xcoord)
# print(ycoord)

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

# A will be used to detect skippable string in the list information
a = ['X', 'Y']

# Convert the list to float and take all numeric values
xcoord = list([float(x) for x in xcoord if x[0] not in a])
ycoord = list([float(x) for x in ycoord if x[0] not in a])

# This needs to use angle distances for the coordinates
lineSafety = lineDetector(xcoord[:10])
print(lineSafety[:10])

# This needs to adjusted to use angles in terms of 0 degrees to 180 degrees
manueverAngle = conductManuever(lineSafety)
print('\n')
print(manueverAngle)
