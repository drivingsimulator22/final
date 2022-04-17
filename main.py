import modules.client as client
import modules.sensorReading as sensor
import modules.inverse_calcs as calcs
import modules.pid as PID
import modules.pidMultiProcFunc as mpfunc
# from time import sleep
# import RPi.GPIO as GPIO 


PID.__init__()
client.clientConnect()
# sensor.SPI_MCP3008_Init()
counter=0
run=True
previous_reading_recieved=[]
while(run):
    if client.takeReadings() is not None: # Sometimes, the readings from the client come as a NoneType reading. So this line ensures it's not None.

        current_readings_recieved=client.takeReadings()

        ##### This if condition is to ensure the readings are not none. And also to not duplicate readings ###
        if previous_reading_recieved!=current_readings_recieved and previous_reading_recieved is not None and current_readings_recieved is not None:
            real_Length = sensor.ADC_MCP3008_Readings()
            desired_Length = calcs.calculatePistonLength(current_readings_recieved)
            previous_reading_recieved=current_readings_recieved

            ### This if condition to make sure calculated lengths don't come as a NoneType list. To eliminate errors ###
            if desired_Length is not None and real_Length is not None:
                mpfunc.core_1()
                print(desired_Length)



    # piston1_pwm = PID.update(PID.piston1, desired_Length[0], real_Length[0])
    # piston2_pwm = PID.update(PID.piston2, desired_Length[1], real_Length[1])
    # piston3_pwm = PID.update(PID.piston3, desired_Length[2], real_Length[2])
    # piston4_pwm = PID.update(PID.piston4, desired_Length[3], real_Length[3])
    # piston5_pwm = PID.update(PID.piston5, desired_Length[4], real_Length[4])
    # piston6_pwm = PID.update(PID.piston6, desired_Length[5], real_Length[5])
    

    
    # if (piston1_pwm) > 0 :
    #     PID.pwm_1.start(piston1_pwm)
    # elif (piston1_pwm) < 0 :
    #     PID.pwm_7.start(piston1_pwm)
    # else :
    #     PID.pwm_1.start(0)
    #     PID.pwm_7.start(0)
    
    # if (piston2_pwm) > 0 :
    #     PID.pwm_2.start(piston2_pwm)
    # elif (piston2_pwm) < 0 :
    #     PID.pwm_8.start(piston2_pwm)
    # else :
    #     PID.pwm_2.start(0)
    #     PID.pwm_8.start(0)

    # if (piston3_pwm) > 0 :
    #     PID.pwm_3.start(piston3_pwm)
    # elif (piston3_pwm) < 0 :
    #     PID.pwm_9.start(piston3_pwm)
    # else :
    #     PID.pwm_4.start(0)
    #     PID.pwm_10.start(0)

    # if (piston4_pwm) > 0 :
    #     PID.pwm_4.start(piston4_pwm)
    # elif (piston4_pwm) < 0 :
    #     PID.pwm_10.start(piston4_pwm)
    # else :
    #     PID.pwm_4.start(0)
    #     PID.pwm_10.start(0)

    # if (piston5_pwm) > 0 :
    #     PID.pwm_5.start(piston5_pwm)
    # elif (piston5_pwm) < 0 :
    #     PID.pwm_11.start(piston5_pwm)
    # else :
    #     PID.pwm_5.start(0)
    #     PID.pwm_11.start(0)

    # if (piston6_pwm) > 0 :
    #     PID.pwm_6.start(piston6_pwm)
    # elif (piston6_pwm) < 0 :
    #     PID.pwm_12.start(piston6_pwm)
    # else :
    #     PID.pwm_6.start(0)
    #     PID.pwm_12.start(0)