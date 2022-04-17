import serial
import socket

#####################################################################################################
# Function name: serialDefine()
# Function description: This function defines the serial port which communicates with the SIMTOOLS 
#####################################################################################################

def serialDefine():
    global ser
    ser = serial.Serial(
        port='COM2',\
        baudrate=9600,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
        timeout=0)
    return ser

#####################################################################################################
# Function name: socketDefine()
# Function description: This function defines the socket server, which listens to incoming
# connection. It then accepts the connection and returns the address to be used in main server loop
#####################################################################################################

def socketDefine():
    global addr
    global client1
    socketServer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  ##### Creating a socket        
    print ("Socket successfully created")    ## Indicator that the socket creation was successfull
    port = 5050                                ### Declaring a random unused port
    socketServer.bind(('', port))                          ### Binding the socket to the port and making it open to all devices in the network
    print ("socket binded to %s" %(port))       ### Indicator that the socket is binded
    socketServer.listen(5)                                 ### Configuring the socket to listen if anyone connects
    print ("socket is listening")               ### Indicator that the socket is listening
    ourIP = socket.gethostbyname(socket.gethostname())
    print("The host IP is : "+ourIP)
    #################################
    print("Connected to: " + ser.portstr) #   <--- Making sure we're connected to COM2
    client1, addr = socketServer.accept()                                                 ### At this point, a client is trying to connect to the server. 
    ### This line accepts that connection and takes the hostname and address
    print ('Got connection from', addr )
    return addr,client1

serialDefine()
socketDefine()

##### Input: readings from SIMTOOLS

if addr:
    while True:

        reading = str(ser.read(17))                                     ## Take reading from serial communication from SIMTOOLS
        lstReading = list(reading)                                      ## put the reading into a list to remove unwanted characters.

        if "R" in reading:                                              ## if the reading is not empty:

            lstReading=[x for x in lstReading if x!="b" and x!="'"]     ## List comprehension to remove "b" and quote from the reading.
            lstReading.append(",")                                      ## Add a comma to the end of the list as a delimiter to split later
            y = "".join(lstReading)                                     ## Join the list into a string to send to client
            client1.send(y.encode())                                    ## Encode the string and send it to the client. 

##### Output: Sending a string with reading to client
