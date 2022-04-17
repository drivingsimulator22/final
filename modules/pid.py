##################################################
# Code Created by "Lankash"
#  @5/2/2022
# File Contents: 2 PID Functions + PID Class
# /

# import pid
import time
import os.path
from dataclasses import dataclass
import RPi.GPIO as GPIO

# Initializing the class for PID Conroller.
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
#........................Class End.........................#

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





# Function Updates the PID Parameters.
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
# ...........................Function End........................#