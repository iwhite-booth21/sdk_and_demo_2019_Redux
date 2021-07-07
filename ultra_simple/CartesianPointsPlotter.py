import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import csv
xcoord = []
ycoord = []
coords = []
count = 0

with open('CartesianPoints.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for line in readCSV:
        print(line)
        print(count)
        #tuple = (line[0], line[1])
        coords.append(tuple)
        xcoord.append(line[0])
        ycoord.append(line[1])
        coords.append((line[0], line[1]))
        count = count + 1

#print(xcoord)
#print(ycoord)

# This process prevent non float values from being evaluated
# Checks for y and x in the strings
a = ['X', 'Y']
xcoord = list([float(x) for x in xcoord if x[0] not in a])
ycoord = list([float(x) for x in ycoord if x[0] not in a])

plt.scatter(xcoord,ycoord, s=1) #marker='o')
# How to save figure
plt.savefig('Dorm_Room', dpi=300, bbox_inches='tight')
plt.show()




