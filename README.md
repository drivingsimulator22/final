# Final Driving Simulator Code
This repo contains all the codes used to operate our 6 DOF Hydraulic Driving Simulator.

## Hardware:
1. Raspberry Pi
2. MCP 3008
3. MOSFETs
4. Diodes
5. Linear Wire Potentiometer
6. Proportional Valves
****

## Codes:

### 1-Socket Programming:
The socket programming portion of the project is split into two parts:

1- The Server Code *PC*

The server code contains two functions:

````

#serialDefine()
This function defines the serial port which communicates with the SIMTOOLS program to get readings from the BeamNG Drive game

#socketDefine()
This function defines the listening socket, which acts as a server that accepts communication from the Raspberry Pi, then send the 
readings to the Raspberry Pi for further calculations.

````

2- The Client Code *Raspberry Pi*

````

#clientConnect()
This function defines a socket and makes it connect to the IP address of the PC

#takeReadings()
This function is inside the main loop. It takes readings from the server, formats them into a readable format,
then send the Roll,Pitch and Yaw angles as a list of integers to the calculations code.



````

### 2-Inverse Kinematics Calculations:
The inverse kinematics calculations are done inside the calcs module, in the _calculatePistonLength()_ function. 

In the inverse kinematics code, we define the base and platform positions using the following code:

````
Base = np.array( [ 
    [ 42.53585743,828.8790286,0],
    [ 739.1023057,-377.6610181,0],
    [696.6661185,-451.2819517,0],
    [-696.618678,-451.1913206,0],
    [-739.1012846,-377.6628879,0],
    [ -42.48607053,828.9118768,0] ])
Base = np.transpose(Base)

Platform = np.array([ 
    [333.9848185,242.098462,0],
    [376.4888746,168.479082, 0],
    [42.50440403,-410,0],
    [-42.50440403,-410, 0],
    [-376.4888746,168.479082, 0],
    [-333.9848185, 242.098462, 0] ])
Platform = np.transpose(Platform)

````

Then we define the three rotational matrices around the x,y, and z axis using the following code:

````
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
   
````

After defining the matrices and base and platform positions, we start calculating the piston lengths using the function _calculatePistonLength(readings)_ which takes a list of integers containing roll,pitch and yaw as the argument. This function returns a list containing the 6 desired piston lengths.
````
def calculatePistonLength(readings):
        # Given input trans, rotation
        if readings is not None:
            roll,pitch,yaw = readings
            trans = np.transpose(np.array([0,0,0])) # X, Y, Z
            rotation = np.transpose(np.array([roll,pitch,yaw])) # roll,pitch,yaw
            # # # Definition of the platform home position.
            home_pos= np.array([0, 0, 997])
            
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
            pistonlength = ( np.linalg.norm(len, axis=0)-892)
            pistonlengthlist=[]
            for i in pistonlength:
                pistonlengthlist.append(i)
            
            # Position of leg in global frame
            L = len +Base
            pistonlengthlist= [int(m) for m in pistonlengthlist]
            return pistonlengthlist
````

### 3-Sensor Readings:
In this step, we get the real length of the 6 pistons using 6 linear wire potentiometers. This is done using MCP3008 Analog to Digital converted.

We first define the _map()_ function that maps the values according to each piston
````
def map(value, inMin, inMax, outMin, outMax):
    return outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))
````

After defining the _map()_ function, we initialize the SPI and configure the MCP, using the _SPI_MCP3008_Init()_ Function.
````
def SPI_MCP3008_Init():
    # to be created as object from the MCP class
    global mcp
    # SPI0 module in Raspberry Pi
    SPI_PORT = 0
    # SPI0.0 first and only slave
    SPI_DEVICE = 0
    mcp = MCP.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE)
                      )    # configure the MCP class
````

The following step is to start taking readings from each sensor, and putting them in a list of integers. This is done using the _ADC_MPC3008_Readings()_ function.
````
def ADC_MCP3008_Readings():
    # Read all the 6 ADC channel values in a list.
    Sensor_Readings = [0]*6
    for i in range(6):
        # The read_adc function will get the value of the specified channel (0-7).
        Sensor_Readings[i] = mcp.read_adc(i)

    # mapping and calibration of every reading
    Sensor_Readings[0] = map(
        Sensor_Readings[0], 39,  980, 0, 1851) - Sensor_Zero_Errors[0]
    Sensor_Readings[1] = map(
        Sensor_Readings[1], 78, 1023, 0,  466) - Sensor_Zero_Errors[1]
    Sensor_Readings[2] = map(
        Sensor_Readings[2],  0, 1023, 0,  520) - Sensor_Zero_Errors[2]
    Sensor_Readings[3] = map(
        Sensor_Readings[3],  0, 1023, 0,  500) - Sensor_Zero_Errors[3]
    Sensor_Readings[4] = map(
        Sensor_Readings[4],  0, 1023, 0,  500) - Sensor_Zero_Errors[4]
    Sensor_Readings[5] = map(
        Sensor_Readings[5],  0, 1023, 0,  500) - Sensor_Zero_Errors[5]

    # convert all reading from floats to integers and save them in new list
    Values = [int(values) for values in Sensor_Readings]

    return Values
````
### 4-PID and PWM:






  
