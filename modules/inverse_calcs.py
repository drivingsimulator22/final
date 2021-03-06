#!/usr/bin/python3


import numpy as np
from math import pi
# Coordinates of the points where the rods 
# are attached to the base.

# Coordinates of the points where the rods 
# are attached to the base 
#  [
#  [X1,Y1,Z1],
#  [X2,Y2,Z2],
#  [X3,Y3,Z3],
#  [X4,Y4,Z4],
#  [X5,Y5,Z5],
#  [X5,Y5,Z6].
#             ]
def MAP(value, inMin, inMax, outMin, outMax):
    return outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))

Base = np.array( [ 
    [-739.1012846,-377.6628879,0],
    [ -42.48607053,828.9118768,0],
    [ 42.53585743,828.8790286,0],
    [ 739.1023057,-377.6610181,0],
    [696.6661185,-451.2819517,0],
     [-696.618678,-451.1913206,0],

    ])
Base = np.transpose(Base)
    
# Coordinates of the points where the rods 
# are attached to the platform.
#  [
#  [X1,Y1,Z1],
#  [X2,Y2,Z2],
#  [X3,Y3,Z3],
#  [X4,Y4,Z4],
#  [X5,Y5,Z5],
#  [X5,Y5,Z6].
#             ]


# Platform = np.array([ 
#     [222.65,161.33,0],
#     [251,112.31, 0],
#     [28.333,-273.33,0],
#     [-28.333,-273.33, 0],
#     [-251,112.31, 0],
#     [-222.65,161.33, 0] ])
# Platform = np.transpose(Platform)

# ## MOAAZ
Platform = np.array([ 
    [-337,146,0],
    [-295,219,0],
    [295,219,0],
    [337,146, 0],
    [42,-365,0],
    [-42,-365,0],
   ])
Platform = np.transpose(Platform)

# print('Leg lengths to command in order to achieve desired position of plate: \n')
# #########################################################################################




# Rotation matrices used later
def rotX(roll):
    rotx = np.array([
        [1,     0    ,    0    ],
        [0,  np.cos(roll), -np.sin(roll)],
        [0,  np.sin(roll), np.cos(roll)] ])
    return rotx

def rotY(pitch):    
    roty = np.array([
        [np.cos(pitch), 0,  np.sin(pitch) ],
        [0         , 1,     0       ],
        [-np.sin(pitch), 0,  np.cos(pitch) ] ])   
    return roty
    
def rotZ(yaw):    
    rotz = np.array([
        [ np.cos(yaw),-np.sin(yaw), 0 ],
        [ np.sin(yaw), np.cos(yaw), 0 ],
        [   0        ,     0      , 1 ] ])   
    return rotz

#####################################################################################################
# Function name: calculatePistonLength()
# Function description: This function will be in the main while loop. It takes the readings from
# client function and calculates the desired lengths of pistons. 
#
# Input : Roll,Pitch,Yaw from client (Raspberry Pi)  
#
# Output: Desired length of pistons in list of integers 
#
# Passed to: PID Functions
#####################################################################################################

def calculatePistonLength(readings):
        # Given input trans, rotation
        try:
            roll,pitch,yaw = readings
            pitch = MAP(pitch, 454.75, 567.265, 0.3455751919, -0.3455751919)
            # pitch = MAP(pitch, 475.5, 546.5, -0.2199115, 0.2199115)
            yaw = MAP(yaw, 454.75, 567.265, -0.3455751919, 0.3455751919)
            roll = MAP(roll, 454.75,  567.265, 0.3455751919, -0.3455751919)
            trans = np.transpose(np.array([0,0,0])) # X, Y, Z
            rotation = np.transpose(np.array([pitch,yaw,roll])) # pitch,yaw,roll (radian)
            # # # Definition of the platform home position.
            home_pos= np.array([0, 0, 670])
            
            # Allocate for variables
            len = np.zeros((3,6))
            pistonlength = np.zeros((6))
            
            # Get rotation matrix of platform. RotZ* RotY * RotX -> matmul
            # R = np.matmul( np.matmul(rotZ(rotation[2]), rotY(rotation[1])), rotX(rotation[0]) )
            R = np.matmul( np.matmul(rotX(rotation[0]), rotY(rotation[1])), rotZ(rotation[2]) )
            
            # Get leg length for each leg
            # leg =( (translation vector + (rotational matrix * platform coordinates)) - base coordinates )
            len = np.repeat(trans[:, np.newaxis], 6, axis=1) + np.repeat(home_pos[:, np.newaxis], 6, axis=1) + np.matmul(R, Platform) - Base 
            
            # print('Leg lengths to command in order to achieve desired position of plate: \n', len)
            pistonlength = ( np.linalg.norm(len, axis=0)-822)
            pistonlengthlist=[]
            for i in pistonlength:
                pistonlengthlist.append(i)
            
            # Position of leg in global frame
            L = len +Base
            pistonlengthlist= [int(m) for m in pistonlengthlist]
            return pistonlengthlist
        except:
            pass
        # print('Leg lengths to command in order to achieve desired position of plate: \n', pistonlength)
# print(calculatePistonLength(0,0,0))
