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

****

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

****

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
****

### 4-PID and PWM:
In this step, we use the setpoint we got from the calculations in step 2 and compare them to the current sensor readings, then we apply a PID function that sends a PWM that moves the pistons to the desired position, trying to reduce the overshoot and oscillation as much as possible

This code is in the PID file in Modules folder.


We start by initializing a class for PID:

````
class pid:
    kp = 0.0
    ki = 0.0
    kd = 0.0

    limMax = 100.0
    limMin = -100.0

    limMax_init = 0.0
    limMin_init = 0.0

    taw = 0.0
    T = 0.1

    integ = 0.0
    diff = 0.0
    prop = 0.0
    prevError = 0.0
    prevMeasurement = 0.0

    outPut = 0.0
````

Then we initialize the parameters, and GPIO pins of the Raspberry Pi 
````
def _init_ (void) :
    #
    # Define pins 
    #
    global piston1_open 
    global piston2_open 
    global piston3_open 
    global piston4_open 
    global piston5_open 
    global piston6_open 

    global piston1_close
    global piston2_close
    global piston3_close
    global piston4_close
    global piston5_close
    global piston6_close

    piston1_open = 0
    piston2_open = 1
    piston3_open = 2
    piston4_open = 3
    piston5_open = 4
    piston6_open = 5

    piston1_close = 6
    piston2_close = 7
    piston3_close = 8
    piston4_close = 9
    piston5_close = 10
    piston6_close = 11


    #
    # Configure Raspberry Pi Pins
    #
    GPIO.setWarnings(False)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(piston1_open, GPIO.OUT)
    GPIO.setup(piston2_open, GPIO.OUT)
    GPIO.setup(piston3_open, GPIO.OUT)
    GPIO.setup(piston4_open, GPIO.OUT)
    GPIO.setup(piston5_open, GPIO.OUT)
    GPIO.setup(piston6_open, GPIO.OUT)

    GPIO.setup(piston1_close, GPIO.OUT)
    GPIO.setup(piston2_close, GPIO.OUT)
    GPIO.setup(piston3_close, GPIO.OUT)
    GPIO.setup(piston4_close, GPIO.OUT)
    GPIO.setup(piston5_close, GPIO.OUT)
    GPIO.setup(piston6_close, GPIO.OUT)

    #
    # Inizialize thw PWM variables
    #
    freq = 100
    
    global pwm_1
    global pwm_2
    global pwm_3
    global pwm_4
    global pwm_5
    global pwm_6
    global pwm_7
    global pwm_8
    global pwm_9
    global pwm_10
    global pwm_11
    global pwm_12

    pwm_1 = GPIO.PWM(piston1_open, freq)
    pwm_2 = GPIO.PWM(piston2_open, freq)
    pwm_3 = GPIO.PWM(piston3_open, freq)
    pwm_4 = GPIO.PWM(piston4_open, freq)
    pwm_5 = GPIO.PWM(piston5_open, freq)
    pwm_6 = GPIO.PWM(piston6_open, freq)
    pwm_7 = GPIO.PWM(piston1_close, freq)
    pwm_8 = GPIO.PWM(piston2_close, freq)
    pwm_9 = GPIO.PWM(piston3_close, freq)
    pwm_10 = GPIO.PWM(piston4_close, freq)
    pwm_11 = GPIO.PWM(piston5_close, freq)
    pwm_12 = GPIO.PWM(piston6_close, freq)

    #
    # Initialize the 6 Pistons PID Parameters
    #
    global piston_1
    global piston_2
    global piston_3
    global piston_4
    global piston_5
    global piston_6

    piston_1 = pid
    piston_2 = pid
    piston_3 = pid
    piston_4 = pid
    piston_5 = pid
    piston_6 = pid
    
    piston_1.kp = 0.0
    piston_1.kd = 0.0
    piston_1.ki = 0.0

    piston_2.kp = 0.0
    piston_2.kd = 0.0
    piston_2.ki = 0.0

    piston_3.kp = 0.0
    piston_3.kd = 0.0
    piston_3.ki = 0.0

    piston_4.kp = 0.0
    piston_4.kd = 0.0
    piston_4.ki = 0.0

    piston_5.kp = 0.0
    piston_5.kd = 0.0
    piston_5.ki = 0.0

    piston_6.kp = 0.0
    piston_6.kd = 0.0
    piston_6.ki = 0.0
````
After initializing the PID and GPIO parameters, we use a function called _update()_  to update the system parameters for each piston:
````
def update(system, setPoint, measurement):
    #
    # Calculating error signal
    #
    error = setPoint - measurement
    #
    # Calculating proportional term
    #
    system.prop = system.kp * error
    #
    # Calculating Integral Term
    #
    system.integ = system.integ + \
        (0.5 * system.ki * system.T * (error + system.prevError))
    #
    # Anti windUp + Clamp the Integrator
    #
    if (system.integ > system.limMax_init):
        system.integ = system.limMax_init
    elif (system.integ < system.limMin_init):
        system.integ = system.limMin_init
    #
    # Derivative (Band Limit) + Low Pass Filter
    #
    system.diff = ((2.0 * system.kd) * (measurement - system.prevMeasurement) +
                   (2 * (system.taw - system.T) * system.diff)) / (2 * system.taw + system.T)
    #
    # Compare O/P and Apply clipping limits
    #
    system.outPut = system.prop + system.integ + system.diff

    if (system.outPut > system.limMax):
        system.outPut = system.limMax
    elif (system.outPut < system.limMin):
        system.outPut = system.limMin
    #
    # Store the new values: Error & Measurements
    #
    system.prevError = error
    system.prevMeasurement = measurement
    #
    # Return PID O/P
    #
    return system.outPut
````

****



  
