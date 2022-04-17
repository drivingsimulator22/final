import modules.client as client
import modules.sensorReading as sensor
import modules.inverse_calcs as calcs
import modules.pid as PID
import pid as PID
def core_1():
    piston1_pwm = PID.update(PID.piston1, desired_Length[0], real_Length[0])
    piston2_pwm = PID.update(PID.piston2, desired_Length[1], real_Length[1])
    if (piston1_pwm) > 0 :
            PID.pwm_1.start(piston1_pwm)
    elif (piston1_pwm) < 0 :
            PID.pwm_7.start(piston1_pwm)
    else :
            PID.pwm_1.start(0)
            PID.pwm_7.start(0)
    if (piston2_pwm) > 0 :
        PID.pwm_2.start(piston2_pwm)
    elif (piston2_pwm) < 0 :
        PID.pwm_8.start(piston2_pwm)
    else :
        PID.pwm_2.start(0)
        PID.pwm_8.start(0)
def core_2():
    piston3_pwm = PID.update(PID.piston3, desired_Length[2], real_Length[2])
    piston4_pwm = PID.update(PID.piston4, desired_Length[3], real_Length[3])
    if (piston3_pwm) > 0 :
        PID.pwm_3.start(piston3_pwm)
    elif (piston3_pwm) < 0 :
        PID.pwm_9.start(piston3_pwm)
    else :
        PID.pwm_4.start(0)
        PID.pwm_10.start(0)
    if (piston4_pwm) > 0 :
        PID.pwm_4.start(piston4_pwm)
    elif (piston4_pwm) < 0 :
        PID.pwm_10.start(piston4_pwm)
    else :
        PID.pwm_4.start(0)
        PID.pwm_10.start(0)
def core_3():
    piston5_pwm = PID.update(PID.piston5, desired_Length[4], real_Length[4])
    piston6_pwm = PID.update(PID.piston6, desired_Length[5], real_Length[5])
    if (piston5_pwm) > 0 :
        PID.pwm_5.start(piston5_pwm)
    elif (piston5_pwm) < 0 :
        PID.pwm_11.start(piston5_pwm)
    else :
        PID.pwm_5.start(0)
        PID.pwm_11.start(0)

    if (piston6_pwm) > 0 :
        PID.pwm_6.start(piston6_pwm)
    elif (piston6_pwm) < 0 :
        PID.pwm_12.start(piston6_pwm)
    else :
        PID.pwm_6.start(0)
        PID.pwm_12.start(0)
