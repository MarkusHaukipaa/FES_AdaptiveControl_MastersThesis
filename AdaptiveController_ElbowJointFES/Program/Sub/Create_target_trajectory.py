'''
Makes target trajectory
'''
import numpy as np
import random

PI = np.pi

def Target_Angle(target_pattern, target_period, time_array, INITIAL_ANGLE_RAD,
                 TIME_STEP, end_time):

    target_angle = np.zeros_like(time_array)
    target_velocity = np.zeros_like(time_array) # not used
    target_acceleration = np.zeros_like(time_array) # not used

    # Rectangular waves
    # 2 sec
    if target_pattern == 0:
        for i, t in enumerate(time_array):
            if t % 4 < 2:
                target_angle[i] = np.radians(40)
            else:
                target_angle[i] = np.radians(80)
    # 4 sec
    elif target_pattern == 1:
        for i, t in enumerate(time_array):
            if t % 8 < 4:
                target_angle[i] = np.radians(40)
            else:
                target_angle[i] = np.radians(80)
    # 6 sec
    elif target_pattern == 2:
        for i, t in enumerate(time_array):
            if t % 12 < 6:
                target_angle[i] = np.radians(40)
            else:
                target_angle[i] = np.radians(80)

    # 40 sec
    elif target_pattern == 3:
        for i, t in enumerate(time_array):
            if t % 40 < 20:
                target_angle[i] = np.radians(20)
            else:
                target_angle[i] = np.radians(90)



    #SinWave
    elif target_pattern == 4:
        f = 1/5
        y_angle = np.sin(2 * PI * f * time_array)
        y_velocity = 2 * PI * f * np.cos(2 * PI * f * time_array)
        y_accel = -((2 * PI * f)**2) * np.sin(2 * PI * f * time_array)
        for i, t in enumerate(time_array):
            if t <= 1:
                target_angle[i] = INITIAL_ANGLE_RAD
                target_velocity[i] = 0
                target_acceleration[i] = 0
            elif t <= 2:
                target_angle[i] = INITIAL_ANGLE_RAD + (np.radians(60) - INITIAL_ANGLE_RAD) * (t - 1)
                target_velocity[i] = np.radians(60) - INITIAL_ANGLE_RAD
                target_acceleration[i] = 0
            elif t <= 6:
                target_angle[i] = INITIAL_ANGLE_RAD + (np.radians(60) - INITIAL_ANGLE_RAD)
                target_velocity[i] = 0
                target_acceleration[i] = 0
            else:
                index = i - int(6 / TIME_STEP)
                target_angle[i] = INITIAL_ANGLE_RAD + (np.radians(60) - INITIAL_ANGLE_RAD) + np.radians(30) * y_angle[index]
                target_velocity[i] = np.radians(30) * y_velocity[index]
                target_acceleration[i] = np.radians(30) * y_accel[index]

    # Random Waveform
    elif target_pattern == 5:
        T1 = random.uniform(1.8, 3.2)
        T2 = random.uniform(3.2, 4.6)
        T3 = random.uniform(4.6, 6.0)
        if target_period == 0: # small
            T1 = 1.8
            T2 = 3.2
            T3 = 4.6
        if target_period == 1: # Mid
            T1 = 2.5
            T2 = 3.9
            T3 = 5.3
        if target_period == 2: # Big
            T1 = 3.2
            T2 = 4.6
            T3 = 6.0
        if target_period == 3: # any
            T1 = 2.97670895630521
            T2 = 3.70181955176688
            T3 = 4.75809449349219
        f1 = 1/T1
        f2 = 1/T2
        f3 = 1/T3
        y_angle_1 = np.sin(2 * PI * f1 * time_array)
        y_angle_2 = np.sin(2 * PI * f2 * time_array)
        y_angle_3 = np.sin(2 * PI * f3 * time_array)
        y_angle = y_angle_1 + y_angle_2 + y_angle_3
        y_velocity_1 = 2 * PI * f1 * np.cos(2 * PI * f1 * time_array)
        y_velocity_2 = 2 * PI * f2 * np.cos(2 * PI * f2 * time_array)
        y_velocity_3 = 2 * PI * f3 * np.cos(2 * PI * f3 * time_array)
        y_velocity = y_velocity_1 + y_velocity_2 + y_velocity_3
        y_accel_1 = -((2 * PI * f1)**2) * np.sin(2 * PI * f1 * time_array)
        y_accel_2 = -((2 * PI * f2)**2) * np.sin(2 * PI * f2 * time_array)
        y_accel_3 = -((2 * PI * f3)**2) * np.sin(2 * PI * f3 * time_array)
        y_accel = y_accel_1 + y_accel_2 + y_accel_3

        for i, t in enumerate(time_array):
            if t <= 1:
                target_angle[i] = INITIAL_ANGLE_RAD
                target_velocity[i] = 0
                target_acceleration[i] = 0
            elif t <= 2:
                target_angle[i] = INITIAL_ANGLE_RAD + (np.radians(60) - INITIAL_ANGLE_RAD) * (t - 1)
                target_velocity[i] = np.radians(60) - INITIAL_ANGLE_RAD
                target_acceleration[i] = 0
            elif t <= 6:
                target_angle[i] = INITIAL_ANGLE_RAD + (np.radians(60) - INITIAL_ANGLE_RAD)
                target_velocity[i] = 0
                target_acceleration[i] = 0
            else:
                index = i - int(6 / TIME_STEP)
                target_angle[i] = INITIAL_ANGLE_RAD + (np.radians(60) - INITIAL_ANGLE_RAD) + np.radians(10) * y_angle[index]
                target_velocity[i] = np.radians(10) * y_velocity[index]
                target_acceleration[i] = np.radians(10) * y_accel[index]

    elif target_pattern == 6:  # Step
        minValue = INITIAL_ANGLE_RAD
        maxValue = np.radians(60)

        for i, t in enumerate(time_array):
            if t <= 2:
                target_angle[i] = minValue
            else:
                target_angle[i] = maxValue

    #Trapezoidal
    elif target_pattern == 7:

        period = 16
        ramp_up_duration = 3
        ramp_down_duration = ramp_up_duration
        stay_duration = period-ramp_up_duration-ramp_down_duration-3
        initial_angle = INITIAL_ANGLE_RAD
        target_angle_max = np.radians(45) + initial_angle

        for i, t in enumerate(time_array):

            t_mod = t % period
            if t_mod < ramp_up_duration:
                target_angle[i] = (initial_angle
                                   + (target_angle_max - initial_angle)
                                   * (t_mod / ramp_up_duration))

            elif t_mod < ramp_up_duration + stay_duration:
                target_angle[i] = target_angle_max

            elif t_mod < ramp_up_duration + stay_duration + ramp_down_duration:
                target_angle[i] = (target_angle_max
                                   - (target_angle_max - initial_angle)
                                   * ((t_mod - ramp_up_duration - stay_duration)
                                      / ramp_down_duration))

            else:
                target_angle[i] = initial_angle

    #Constant
    elif target_pattern == 8:
        for i, t in enumerate(time_array):
            target_angle[i] = np.radians(40)

    elif target_pattern == 9:


        #Test sequence of ramp, sin, and random and stationary
        timeOfsquence = int ((end_time-25) // 5)
        timeOfTrapz = 60 # 100//16 = 6 , 100%16 = 4 (top position
        timeOfSin = timeOfTrapz + timeOfsquence + 15
        timeOfRand = timeOfSin + timeOfsquence + 14

        #Ramp
        period = 16
        ramp_up_duration = 3
        stay_duration = 7
        ramp_down_duration = 3
        initial_angle = INITIAL_ANGLE_RAD
        target_angle_max = np.radians(45) + initial_angle

        #Sin

        f = 1 / 20
        y_angle_sin = np.sin(2 * PI * f * time_array)
        y_velocity_sin = 2 * PI * f * np.cos(2 * PI * f * time_array)
        y_accel_sin = -((2 * PI * f) ** 2) * np.sin(2 * PI * f * time_array)

        #Rand

        T1 = random.uniform(1.8, 3.2)
        T2 = random.uniform(3.2, 4.6)
        T3 = random.uniform(4.6, 6.0)
        if target_period == 0:  # small
            T1 = 1.8
            T2 = 3.2
            T3 = 4.6
        if target_period == 1:  # Mid
            T1 = 2.5
            T2 = 3.9
            T3 = 5.3
        if target_period == 2:  # Big
            T1 = 3.2
            T2 = 4.6
            T3 = 6.0
        if target_period == 3:  # any
            T1 = 2.97670895630521
            T2 = 3.70181955176688
            T3 = 4.75809449349219

        f1 = 1 / T1
        f2 = 1 / T2
        f3 = 1 / T3
        y_angle_1 = np.sin(2 * PI * f1 * time_array)
        y_angle_2 = np.sin(2 * PI * f2 * time_array)
        y_angle_3 = np.sin(2 * PI * f3 * time_array)
        y_angle = y_angle_1 + y_angle_2 + y_angle_3
        y_velocity_1 = 2 * PI * f1 * np.cos(2 * PI * f1 * time_array)
        y_velocity_2 = 2 * PI * f2 * np.cos(2 * PI * f2 * time_array)
        y_velocity_3 = 2 * PI * f3 * np.cos(2 * PI * f3 * time_array)
        y_velocity = y_velocity_1 + y_velocity_2 + y_velocity_3
        y_accel_1 = -((2 * PI * f1) ** 2) * np.sin(
            2 * PI * f1 * time_array)
        y_accel_2 = -((2 * PI * f2) ** 2) * np.sin(
            2 * PI * f2 * time_array)
        y_accel_3 = -((2 * PI * f3) ** 2) * np.sin(
            2 * PI * f3 * time_array)
        y_accel = y_accel_1 + y_accel_2 + y_accel_3

        # stationaty
        targetAngleChagned = False
        angleLvlChange = np.radians(45)
        target = initial_angle + angleLvlChange  # + INITIAL_ANGLE_RAD
        maxTarget = np.radians(103.5)


        for i, t in enumerate(time_array):

            if t <= timeOfTrapz-3:
                t_mod = t % period

                if t_mod < ramp_up_duration:
                    target_angle[i] = (initial_angle
                                       + (target_angle_max - initial_angle)
                                       * (t_mod / ramp_up_duration))

                elif t > (timeOfTrapz - ramp_up_duration):

                    target_lvl = np.radians(60)
                    if target_angle[i - 1 ] - target_lvl > 0.01:
                        target_angle[i] =  target_angle[i - 1 ] - np.radians(
                            0.02)

                    elif target_angle[i - 1 ]  - target_lvl <  -0.01:
                        target_angle[i] =  target_angle[i - 1 ] + np.radians(
                            0.02)

                elif t_mod < ramp_up_duration + stay_duration:
                    target_angle[i] = target_angle_max

                elif t_mod < ramp_up_duration + stay_duration + ramp_down_duration:
                    target_angle[i] = (target_angle_max
                                       - (target_angle_max - initial_angle)
                                       * ((t_mod - ramp_up_duration - stay_duration)
                                          / ramp_down_duration))

                else:
                    target_angle[i] = initial_angle



            elif t <= 60.0916:
                # quick Visual correction on transition to sinusoidal, no major
                # difference
                target_angle[i] = target_angle_max
            elif t <= (timeOfSin):

                index = i - int(10 / TIME_STEP)
                target_angle[i] = (np.radians(60) + np.radians(50) *
                                   y_angle_sin[index])
                target_velocity[i] = np.radians(30) * y_velocity_sin[index]
                target_acceleration[i] = np.radians(30) * y_accel_sin[index]


            elif  t <= timeOfRand:

                if t >= (timeOfRand - ramp_up_duration):
                    target_lvl = np.radians(60)
                    if target_angle[i - 1] - target_lvl > 0.01:
                        target_angle[i] = target_angle[i - 1] - np.radians(
                            0.02)

                    elif target_angle[i - 1] - target_lvl < -0.01:
                        target_angle[i] = target_angle[i - 1] + np.radians(
                            0.02)

                    else:
                        target_angle[i] = np.radians(60)

                else:
                    index = i - int(6 / TIME_STEP)
                    target_angle[i] = np.radians(60) + np.radians(10)* y_angle[index]
                    target_velocity[i] = np.radians(10) * y_velocity[index]
                    target_acceleration[i] = np.radians(10) * y_accel[index]

            else:

                period = 20
                stay_duration = period-ramp_up_duration
                t_mod = t % period

                if t_mod < ramp_up_duration:
                    target_angle[i] = (initial_angle
                                       + (target - initial_angle)
                                       * (t_mod / ramp_up_duration))
                    targetAngleChagned = False

                elif t_mod < (period-TIME_STEP):
                    target_angle[i] = target

                else:
                    if not targetAngleChagned:
                        initial_angle = target
                        target += angleLvlChange
                        targetAngleChagned = True

                        if target >= maxTarget:
                            target = maxTarget
                            angleLvlChange = (-1)*angleLvlChange

                    if target <= INITIAL_ANGLE_RAD:
                        target = INITIAL_ANGLE_RAD

                    target_angle[i] = initial_angle

        for i in range(1, len(target_angle)):
            if abs(target_angle[i] - target_angle[i - 1]) > np.radians(0.1):
                target_angle[i] = target_angle[i - 1] + np.sign(
                    target_angle[i] - target_angle[i - 1]) * np.radians(0.0001)

    return target_angle, target_velocity, target_acceleration
    # Smoothing function

