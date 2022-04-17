##################################################
# Code Created by "Lankash"
#  @7/4/2022
# File Contents: PID Application
#


import modules.pid as PID
#import modules.sens_read as sens_read
import modules.map
import RPi.GPIO as GPIO
from time import sleep
import matplotlib as matplot
import numpy
import Adafruit_MCP3008 as MCP
import Adafruit_GPIO.SPI as SPI


#
# Define pins
#
piston1_open = 20
piston1_close = 21


#
# Configure Raspberry Pi Pins
#
# GPIO.setWarning(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(piston1_open, GPIO.OUT)
GPIO.setup(piston1_close, GPIO.OUT)


#
# Initialize variables
#
sensors = [0, 0, 0, 0, 0, 0]

#
# Configure SPI connection for sensor readings
#
# sens_read.SPI_MCP3008_Init()
#
# Variable_decleration
#
Sensor_Zero_Errors = [7, 0, 0, 0, 0, 0]  # list of the sensors zero error


# SPI0 module in Raspberry Pi
SPI_PORT = 0
# SPI0.0 first and only slave
SPI_DEVICE = 0
mcp = MCP.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE)
                  )    # configure the MCP class


# Read all the 8 ADC channel values in a list.
Sensor_Readings = [0]*8

print("|  PID  |   1   |   2   |   3   |   4   |   5   |   6   |")

#
# Initialize the Piston PID Parameters
#
piston_1 = PID.pid

piston_1.kp = 5.0
piston_1.kd = 1.0
piston_1.ki = 2.0

piston_1.limMax = 450
piston_1.limMin = -450

piston_1.limMax_init = 100
piston_1.limMin_init = 0

piston_1.taw = 0.5
piston_1.T = 0.1

piston_1.setPoint = 100

#
# Inizialize thw PWM variables
#
freq = 200
pwm_1 = GPIO.PWM(piston1_open, freq)
pwm_7 = GPIO.PWM(piston1_close, freq)


#
# Invers kinematics equations
#
setPoint_1 = piston_1.setPoint

while (True):
    #
    # Receive the Sesnsor Readings via SPI
    #
    #sensors = sens_read.ADC_MCP3008_Readings()

    for i in range(5):
        # The read_adc function will get the value of the specified channel (0-7).
        Sensor_Readings[i] = mcp.read_adc(i)

    Sensor_Readings[0] = Sensor_Readings[0] * 0.4945

    # convert all reading from floats to integers and save them in new list
    Values = [int(values) for values in Sensor_Readings]

    #
    # Compute PID parameters
    #
    piston1_pwm = PID.update(piston_1, setPoint_1, Values[0])

    print("|  {}  |  {}  |  {}  |  {}  |  {}  |  {}  |  {}  |".format(int(piston1_pwm),
          Values[0], Values[1], Values[2], Values[3], Values[4], Values[5]))
    sleep(0.1)

    piston1_pwm = piston1_pwm * 0.2

    #
    # PWM Output
    #
    pwm_1.start(0)
    pwm_7.start(0)
    # if (setPoint_1 - Values[0]) > 0:
    #     pwm_1.start(piston1_pwm)
    # elif (setPoint_1 - Values[0]) < 0:
    #     pwm_7.start(piston1_pwm)

    if (piston1_pwm) > 0:
        pwm_1.start(piston1_pwm)
    elif (piston1_pwm) < 0:
        pwm_7.start(-piston1_pwm)