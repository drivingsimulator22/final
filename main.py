import modules.client_pi as client
import modules.sensorReading as sensor
import modules.inverse_calcs as calcs
from time import sleep
import RPi.GPIO as GPIO 
import modules.pid as PID
import matplotlib as matplot
import numpy
import Adafruit_MCP3008 as MCP
import Adafruit_GPIO.SPI as SPI
import modules.map as MAP

# Piston parameters:(kp,ki,kd,extractionPin,retractionPin)
piston_1 = PID.pid(0.14,0,0.51,7,1)
piston_2 = PID.pid(0.12,0,0.0000001,12,16)
piston_3 = PID.pid(0.1052,0,0.175,20,21)                                                                               
piston_4 = PID.pid(0.2,0,0.2,26,19)
piston_5 = PID.pid(0.1048,0,0.175,13,6)
piston_6 = PID.pid(1,0,0.16,5,0)

def GPIO_init():
    freq = 100
    global pwm_1,pwm_7,pwm_2,pwm_8,pwm_3,pwm_9,pwm_4,pwm_10,pwm_5,pwm_11,pwm_6,pwm_12
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(piston_1.openPin, GPIO.OUT)
    GPIO.setup(piston_1.closePin, GPIO.OUT)
    GPIO.setup(piston_2.openPin, GPIO.OUT)
    GPIO.setup(piston_2.closePin, GPIO.OUT)
    GPIO.setup(piston_3.openPin, GPIO.OUT)
    GPIO.setup(piston_3.closePin, GPIO.OUT)
    GPIO.setup(piston_4.openPin, GPIO.OUT)
    GPIO.setup(piston_4.closePin, GPIO.OUT)
    GPIO.setup(piston_5.openPin, GPIO.OUT)
    GPIO.setup(piston_5.closePin, GPIO.OUT)
    GPIO.setup(piston_6.openPin, GPIO.OUT)
    GPIO.setup(piston_6.closePin, GPIO.OUT)

    pwm_1 = GPIO.PWM(piston_1.openPin, freq)
    pwm_7 = GPIO.PWM(piston_1.closePin, freq)
    pwm_2 = GPIO.PWM(piston_2.openPin, freq)
    pwm_8 = GPIO.PWM(piston_2.closePin, freq)
    pwm_3 = GPIO.PWM(piston_3.openPin, freq)
    pwm_9 = GPIO.PWM(piston_3.closePin, freq)
    pwm_4 = GPIO.PWM(piston_4.openPin, freq)
    pwm_10 = GPIO.PWM(piston_4.closePin, freq)
    pwm_5 = GPIO.PWM(piston_5.openPin, freq)
    pwm_11 = GPIO.PWM(piston_5.closePin, freq)
    pwm_6 = GPIO.PWM(piston_6.openPin, freq)
    pwm_12 = GPIO.PWM(piston_6.closePin, freq)


GPIO_init()
sensor_zero_errors = [25, 41, 37, 36, 36, 24] # List of zero errors for each sensor
SPI_PORT = 0
SPI_DEVICE = 0
mcp = MCP.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
sensor_readings = [0]*6



client.clientConnect()
sensor.SPI_MCP3008_Init()
previous_reading_recieved=0
run=True
try:
    while(run):
        try:
            roll_recieved,pitch_recieved,yaw_recieved=client.takeReadings()
            current_readings_recieved = [roll_recieved,pitch_recieved,yaw_recieved]
            if previous_reading_recieved!=current_readings_recieved:
                real_Length = sensor.ADC_MCP3008_Readings()
                desired_Length = calcs.calculatePistonLength(current_readings_recieved)
                previous_reading_recieved=current_readings_recieved
            piston_1.setPoint = int(MAP.MAP(desired_Length[0], -6, 265, 70, 320))
            piston_2.setPoint = int(MAP.MAP(desired_Length[1], -6, 265, 70, 320))
            piston_3.setPoint = int(MAP.MAP(desired_Length[2], -6, 265, 70, 320))
            piston_4.setPoint = int(MAP.MAP(desired_Length[3], -6, 265, 70, 320))
            piston_5.setPoint = int(MAP.MAP(desired_Length[4], -6, 265, 70, 320))
            piston_6.setPoint = int(MAP.MAP(desired_Length[5], -6, 265, 70, 320))
            setpoints = [piston_1.setPoint,piston_2.setPoint,piston_3.setPoint,piston_4.setPoint,piston_5.setPoint,piston_6.setPoint]

            # piston_1.setPoint = 200
            # piston_2.setPoint = 200
            # piston_3.setPoint = 200
            # piston_4.setPoint = 200
            # piston_5.setPoint = 200
            # piston_6.setPoint = 200


            runPID = True
            countPID=0

            while (runPID):

                for i in range(6):
                    # The read_adc function will get the value of the specified channel (0-5).
                    sensor_readings[i] = mcp.read_adc(i)

                sensor_readings[0] = MAP.MAP(sensor_readings[0], 39,  980, 0, 1851) - sensor_zero_errors[0]
                sensor_readings[1] = MAP.MAP(sensor_readings[1], 78, 1023, 0,  466) - sensor_zero_errors[1]
                sensor_readings[2] = MAP.MAP(sensor_readings[2], 0, 1023, 0,  520) - sensor_zero_errors[2]
                sensor_readings[3] = MAP.MAP(sensor_readings[3], 0, 1023, 0,  500) - sensor_zero_errors[3]
                sensor_readings[4] = MAP.MAP(sensor_readings[4],  0, 1023, 0,  500) - sensor_zero_errors[4]
                sensor_readings[5] = MAP.MAP(sensor_readings[5],  0, 1023, 0,  500) - sensor_zero_errors[5]
                # convert all reading from floats to integers and save them in new list
                Values = [int(sensor_readings) for sensor_readings in sensor_readings]
                #
                # Compute PID parameters
                #
                piston1_pwm = PID.update(piston_1, piston_1.setPoint, Values[3])
                piston2_pwm = PID.update(piston_2, piston_2.setPoint, Values[1])
                piston3_pwm = PID.update(piston_3, piston_3.setPoint, Values[2])
                piston4_pwm = PID.update(piston_4, piston_4.setPoint, Values[0])
                piston5_pwm = PID.update(piston_5, piston_5.setPoint, Values[4])
                piston6_pwm = PID.update(piston_6, piston_6.setPoint, Values[5])
                #
                # PWM Output
                #
                if piston1_pwm > 0:           # Perform Extraction if PWM is +ve
                    piston1_pwm = MAP.MAP(piston1_pwm, 0, 100, 25, 40)
                    pwm_1.start(piston1_pwm)
                    pwm_7.start(0)
                elif piston1_pwm < 0:         # Perform Retraction if PWM is -ve
                    piston1_pwm = MAP.MAP(piston1_pwm, -100, 0, -40, -25)
                    pwm_7.start(-piston1_pwm)
                    pwm_1.start(0)
                else:
                    pwm_7.start(0)
                    pwm_1.start(0)

                #######
                
                if piston2_pwm > 0:           # Perform Extraction if PWM is +ve
                    piston2_pwm = MAP.MAP(piston2_pwm, 0, 100, 27, 50)
                    pwm_2.start(piston2_pwm)
                    pwm_8.start(0)
                elif piston2_pwm < 0:         # Perform Retraction if PWM is -ve
                    piston2_pwm = MAP.MAP(piston2_pwm, -100, 0, -50, -27)
                    pwm_8.start(-piston2_pwm)
                    pwm_2.start(0)
                else:
                    pwm_8.start(0)
                    pwm_2.start(0)
                #######
                
                if piston3_pwm > 0:           # Perform Extraction if PWM is +ve
                    piston3_pwm = MAP.MAP(piston3_pwm, 0, 100, 27, 50)
                    pwm_3.start(piston3_pwm)
                    pwm_9.start(0)
                elif piston3_pwm < 0:         # Perform Retraction if PWM is -ve
                    piston3_pwm = MAP.MAP(piston3_pwm, -100, 0, -50, -27)
                    pwm_9.start(-piston3_pwm)
                    pwm_3.start(0)
                else:
                    pwm_9.start(0)
                    pwm_3.start(0)
                #######
                
                if piston4_pwm > 0:           # Perform Extraction if PWM is +ve
                    piston4_pwm = MAP.MAP(piston4_pwm, 0, 100, 27, 50)
                    pwm_4.start(piston4_pwm)
                    pwm_10.start(0)
                elif piston4_pwm < 0:         # Perform Retraction if PWM is -ve
                    piston4_pwm = MAP.MAP(piston4_pwm, -100, 0, -50, -27)
                    pwm_10.start(-piston4_pwm)
                    pwm_4.start(0)
                else:
                    pwm_10.start(0)
                    pwm_4.start(0) 
                #######
                
                if piston5_pwm > 0:           # Perform Extraction if PWM is +ve
                    piston5_pwm = MAP.MAP(piston5_pwm, 0, 100, 27, 45)
                    pwm_5.start(piston5_pwm)
                    pwm_11.start(0)
                elif piston5_pwm < 0:         # Perform Retraction if PWM is -ve
                    piston5_pwm = MAP.MAP(piston5_pwm, -100, 0, -45, -27)
                    pwm_11.start(-piston5_pwm)
                    pwm_5.start(0)
                else:
                    pwm_11.start(0)
                    pwm_5.start(0)
                #######
                
                if piston6_pwm > 0:           # Perform Extraction if PWM is +ve
                    piston6_pwm = MAP.MAP(piston6_pwm, 0, 100, 27, 50)
                    pwm_6.start(piston6_pwm)
                    pwm_12.start(0)
                elif piston6_pwm < 0:         # Perform Retraction if PWM is -ve
                    piston6_pwm = MAP.MAP(piston6_pwm, -100, 0, -50, -27)
                    pwm_12.start(-piston6_pwm)
                    pwm_6.start(0)
                else:
                    pwm_12.start(0)
                    pwm_6.start(0) 
                

                print("|  {}  |  {}  | {} |  {}  |  {}  | {} | {} ".format(Values[3],Values[1],Values[2],Values[0],Values[4],Values[5],setpoints))
                countPID+=1
                if countPID == 50:
                    runPID=False
                    print("Done PID")

        except:
            pass

except KeyboardInterrupt:
    GPIO.cleanup()
