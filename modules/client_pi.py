import socket
def clientConnect():
    # Create a socket object
    global socketClient
    socketClient = socket.socket(socket.AF_INET,socket.SOCK_STREAM)        
    # Define the port on which you want to connect
    port = 5050   
    # connect to the server on local computer
    socketClient.connect(('192.168.137.1', port))
    print("Connected to host PC")
def takeReadings():
    #Defining the variables            
    try:
        recieved = socketClient.recv(32768).decode()     ##### Put the recieved readings into variable
        if recieved[0] == "R" and recieved.count("R")==1 and recieved[-1]=="," and len(recieved.split(","))==4: ## This line to ensure the reading doesn't come with errors
            ## Making sure the first element is "R" and there's only one "R" also, the last element is "," and the length of the split string is equal to 4 ("R","P","Y"," ")
            rollrecieved = recieved.split(",")[0].replace("R","")                                               ## Split the recieved string and remove R,P,Y.. Roll
            pitchrecieved = recieved.split(",")[1].replace("P","")                                              ## Pitch
            yawrecieved = recieved.split(",")[2].replace("Y","")                                                ## Yaw
            final_list = [rollrecieved,pitchrecieved,yawrecieved]                                               ## Put the values into the final list
            final_list = [int(x) for x in final_list]                                                           ## Convert list of strings into list of integers
            # print(final_list)
            return final_list  ## Order of list: Roll, Pitch, Yaw
    except:
        pass

### Input : Take readings from server

# clientConnect()
# while True:
#     roll,pitch,yaw = takeReadings()    ### This list contains the roll and pitch and yaw.
#     print(readings)
### Output: Roll, Pitch, Yaw in a list of integers.