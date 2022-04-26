# Final Driving Simulator Code
This repo contains all the codes used to operate our 6 DOF Hydraulic Driving Simulator.

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
  
