import time
import os.path
from dataclasses import dataclass


# Initializing the class for PID Conroller.
class pid:
    def __init__(self,kp,ki,kd,openPin,closePin):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limMax = 100
        self.limMin = -100
        self.limMax_init = 300
        self.limMin_init = 0
        self.taw = 0.5
        self.T = 0.01
        self.openPin = openPin
        self.closePin = closePin


    setPoint = 0.0

    integ = 0.0
    diff = 0.0
    prop = 0.0
    prevError = 0.0
    prevMeasurement = 0.0

    outPut = 0.0
#........................Class End.........................#


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