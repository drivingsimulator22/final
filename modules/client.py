import socket

#####################################################################################################
# Function name: clientConnect()
# Function description: This function defines a client socket, then connects to the main server
#####################################################################################################
def clientConnect():
    # Create a socket object
    global socketClient
    socketClient = socket.socket(socket.AF_INET,socket.SOCK_STREAM)        
    # Define the port on which you want to connect
    port = 5050   
    # connect to the server on local computer
    socketClient.connect(('192.168.130.20', port))
    print("Connected to host PC")

#####################################################################################################
# Function name: takeReadings()
# Function description: This function will be in the main while loop. It takes readings from the 
# server and filters the errors, then produces the final result in a list of integers.
#
# Input : Readings from PC server.
#
# Output: List of integers containing Roll, Pitch and Yaw
#
# Passed to: calculatePistonLength()
#####################################################################################################

def takeReadings():
    #Defining the variables            
    recieved = socketClient.recv(32768).decode()     ##### Put the recieved readings into variable
    if recieved[0] == "R" and recieved.count("R")==1 and recieved[-1]=="," and len(recieved.split(","))==4: ## This line to ensure the reading doesn't come with errors
        ## Making sure the first element is "R" and there's only one "R" also, the last element is "," and the length of the split string is equal to 4 ("R","P","Y"," ")
        rollrecieved = recieved.split(",")[0].replace("R","")                                               ## Split the recieved string and remove R,P,Y.. Roll
        pitchrecieved = recieved.split(",")[1].replace("P","")                                              ## Pitch
        yawrecieved = recieved.split(",")[2].replace("Y","")                                                ## Yaw
        if rollrecieved and pitchrecieved and yawrecieved:
            final_list = [rollrecieved,pitchrecieved,yawrecieved]                                               ## Put the values into the final list
            final_list = [int(x) for x in final_list]                                                           ## Convert list of strings into list of integers
            # print(final_list)
            
            return final_list  ## Order of list: Roll, Pitch, Yaw
