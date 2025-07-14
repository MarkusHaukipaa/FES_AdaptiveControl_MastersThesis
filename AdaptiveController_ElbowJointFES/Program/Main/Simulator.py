"""
Holds the functionality of the Program. Controlled by Control_Panel. Main
changes, influencing the research, can be made from Control_Panel.
"""
import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))
sys.path.append(parent_dir)

from AdaptiveController_ElbowJointFES.Program.Sub.Create_target_trajectory import Target_Angle
from AdaptiveController_ElbowJointFES.Program.Sub.Save_data_CSV import saveDataToCSV
from AdaptiveController_ElbowJointFES.Program.Sub.Normalization_functions import normalize
from AdaptiveController_ElbowJointFES.Program.Main.SES_algorithm import (StoaticExtrenumSeeking as
                                                 SES_3_2)
from Osim import OsimModel

import importlib

# Module import
module = importlib.import_module('Program.Sub.Recruitment_characteristics')


# TODO: remove unnessecary parameters

def simulate(TIME_STEP, control_condition, target_muscle, target_pattern,
             quiet_time, end_time, excitation_BIC_max, excitation_TRI_max,
             excitation_BIC_min, excitation_TRI_min,
             model_file_name, model_folder_name, INITIAL_ANGLE_RAD,
             initial_sto_file, iteration,
             Flags, target_period,
             recruitmentDict,
             add_delay_time, Pid_controller_BIC,
             Pid_controller_TRI, sectionLengthSES,
             excitation_frequency):

    #Main variables
    start_time = 0.0
    ramp_start = quiet_time
    ramp_end = end_time
    ramp_length = ramp_end-ramp_start
    step_start = quiet_time
    # FLAGS
    bug_flag = False
    (plot_flag, SES_flag, save_flag,
     PID_flag, incrementalChange_flag, changeRecruitment_Flag) = (
        Flags)

    if not incrementalChange_flag:
        recruitmentPeriods =  np.linspace(start_time, end_time,
                                          len(recruitmentDict.keys())+1)
        for key in recruitmentDict.keys():
            print("Recruitement Period: ", recruitmentPeriods[key],
                  " - ", recruitmentPeriods[key+1])
            print("Recruitment Models: ", recruitmentDict[key]["name"])
        currentRecPeriod = 0

    excitation_BIC = excitation_BIC_min
    excitation_TRI = excitation_TRI_min
    excitation_range_BIC = excitation_BIC_max - excitation_BIC_min

    cut_index = int(2 / TIME_STEP)


    # Controller Name
    #---------------------------------
    if control_condition == 0:
        controller_name = "Step"
        TargetFlag = False
    elif control_condition == 1:
        controller_name = "Ramp"
        TargetFlag = False
    elif control_condition == 2:
        controller_name = "PID"
        TargetFlag = True

    if SES_flag:
        controller_name = controller_name + "_SES" + str(sectionLengthSES)
    # ---------------------------------

    # Osim muscle Muscle_models initialization
    osimMuslceModel = OsimModel(model_folder_name, model_file_name, initial_sto_file,
              excitation_BIC_min, excitation_TRI_min, TIME_STEP)

    # Data storge initilize

    n_samples = int(end_time/TIME_STEP) + 1

    Time = np.full(n_samples, np.nan)
    Excitation_BIC = np.full(n_samples, np.nan)
    Excitation_TRI = np.full(n_samples, np.nan)
    ObservedAngle_rad = np.full(n_samples, np.nan)
    Error_rad = np.full(n_samples, np.nan)
    Error_degree = np.full(n_samples, np.nan)
    Error_degreeDT_accounted = np.full(n_samples, np.nan)
    Mae_col = np.full(n_samples, np.nan)
    Rmse_col = np.full(n_samples, np.nan)
    end_time_col = np.full(n_samples, np.nan)
    time_step_col = np.full(n_samples, np.nan)
    elapsed_time_col = np.full(n_samples, np.nan)
    kp_col_BIC = np.full(n_samples, np.nan)
    ki_col_BIC = np.full(n_samples, np.nan)
    kd_col_BIC = np.full(n_samples, np.nan)


    gain_Kp = []
    gain_Ki = []
    gain_Kd = []

    gainEstimate_Kp = []
    gainEstimate_Ki = []
    gainEstimate_Kd = []


    n_delay_samples_BIC = int(add_delay_time / TIME_STEP)
    n_delay_samples_TRI = int(add_delay_time/TIME_STEP)
    delay_BIC = list(np.full(n_delay_samples_BIC, excitation_BIC_min))
    delay_TRI = list(np.full(n_delay_samples_TRI, excitation_TRI_min))

    muscleLagEstimate = add_delay_time
    muscleLagEstimateIndx = n_delay_samples_BIC

    # Get Initial gain values
    Kp, Ki, Kd = Pid_controller_BIC.getGains()
    Pid_controller_BIC.doAdjustToChange()
    Pid_controller_TRI.doAdjustToChange()

    #TODO: delete target acc and vel you dont use them

    # Formation of target angle
    if TargetFlag:
        print("Creating target trajectory")
        time_array = np.arange(start_time, end_time + TIME_STEP + 0.25, TIME_STEP)
        target_angles, target_velocities, target_accelerations \
            = Target_Angle(target_pattern, target_period, time_array,
                           INITIAL_ANGLE_RAD, TIME_STEP, end_time)
        # Velocity and accelerations are not used, in the future they might

    # Plot parameter initialization
    if plot_flag:
        plt.ion()
        # 2 plot areas
        fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(12, 8))

        # List for storing data during simulation
        current_times = []
        state_values_ObAng = []
        state_values_BIC = []
        state_values_TRI = []
        if TargetFlag:
            state_values_target = []

    #initiliaze SES Class
    if SES_flag:
        # Adapt by integral and derivate constants
        Optimizer_SES_BIC = SES_3_2(param_estimate=[Kp, Kp / Ki, Kd / Kp])

        # By gains
        # Optimizer_SES_BIC = SES_3_2(param_estimate=[Kp, Ki, Kd])

        # FLAGS FOR:

        # Algorithm negates output scores if false (to take muscle lag to
        # account)
        muscleLagAccounted = False

        # To calculate the "estimate"(exact in simulation due to known constant)
        # muscle lag
        delayCounter = 0

        # Ignore value just before changing estimate
        estimateChangeIncoming = False

    #__________________________________________________________________#
    # start_time of the simulation
    TIME_SIMULATION_START = time.time()


    # Simulation execution part
    # The data to be saved is the data after integration at time i.
    # Initial settings at the start of a simulation

    for i in range (n_samples):
        current_time = TIME_STEP * i

        # Initial state at (t=0.0000)
        if current_time == 0:
            # save initial state
            Time[i] = current_time
            Excitation_BIC[i] = excitation_BIC
            Excitation_TRI[i] = excitation_TRI
            ObservedAngle_rad[i] = osimMuslceModel.getObservedAngle()

            if SES_flag:
                gainEstimate_Kp.append(
                    Optimizer_SES_BIC.getEstimateGains()[0])
                gainEstimate_Ki.append(
                    Optimizer_SES_BIC.getEstimateGains()[1])
                """ gainEstimate_Kd.append(
                    Optimizer_SES_BIC.getEstimateGains()[2])"""
            Kp_BIC, Ki_BIC, Kd_BIC = Pid_controller_BIC.getGains()

            gain_Kp.append(Kp_BIC)
            gain_Ki.append(Ki_BIC)

            if TargetFlag:
                Error_rad[i] = target_angles[0] - INITIAL_ANGLE_RAD
                Error_degreeDT_accounted [i] = 0
            print("{} Simulation in progress:{}/{}".format( controller_name,
                                                                   round(current_time, 2),
                                                                 end_time))

            # Plot graph, Initial value generation
            if plot_flag:
                current_times.append(current_time)
                state_values_ObAng.append(np.degrees(ObservedAngle_rad[i]))
                state_values_BIC.append(excitation_BIC)
                state_values_TRI.append(excitation_BIC)
                if TargetFlag:
                    state_values_target.append(np.degrees(target_angles[0]))

            continue

        # Control setting
        if control_condition == 0:  # Step
            if current_time < step_start:
                excitation_BIC = excitation_BIC_min
                excitation_TRI = excitation_TRI_min

            elif current_time <= end_time-2:
                excitation_BIC = excitation_BIC_max
                excitation_TRI = excitation_TRI_min

            else:
                excitation_BIC = excitation_BIC_min
                excitation_TRI = excitation_TRI_min

        if control_condition == 1:  # Ramp
            if current_time < ramp_start:
                excitation_BIC = excitation_BIC_min
                excitation_TRI = excitation_TRI_min
            elif current_time <= ramp_end:
                excitation_BIC = (excitation_BIC_min
                                  + (current_time - ramp_start)
                                  * excitation_range_BIC
                                  / ramp_length)

                excitation_TRI = 0
            else:
                excitation_BIC = excitation_BIC_min
                excitation_TRI = excitation_TRI_min

        if control_condition == 2 and SES_flag and current_time >= 1: #PID SES

            target_angle = target_angles[i-muscleLagEstimateIndx]

            # SES Parameter update (SES)
            error = target_angle- ObservedAngle_rad[i-1]

            # change values according to excitation frequency of PID
            if i % int(50/excitation_frequency/TIME_STEP) == 0:

                if not estimateChangeIncoming: # Negating necessary
                    # perturbation before adaption
                    Optimizer_SES_BIC.perputateGains()
                    gains_BIC = Optimizer_SES_BIC.getGains()
                    gains_TRI = np.negative(gains_BIC)

                if PID_flag:

                    Pid_controller_BIC.updateGains(gains_BIC[0], gains_BIC[1],
                                               gains_BIC[2])

                    Pid_controller_TRI.updateGains(gains_TRI[0],
                                                       gains_TRI[1],
                                                       gains_TRI[2])
                else: # PI

                    Pid_controller_BIC.updateGains(gains_BIC[0], gains_BIC[1],
                                                   0)
                    Pid_controller_TRI.updateGains(gains_TRI[0],
                                                   gains_TRI[1],
                                                   0)

            # Taking delay of the muscle response to an account. Won't let the
            # delay effect the error accumulation of the new parameter

            if not muscleLagAccounted:
                delayCounter += 1
                if delayCounter*TIME_STEP > muscleLagEstimate:
                    delayCounter = 0
                    muscleLagAccounted = True



            if muscleLagAccounted:
                Optimizer_SES_BIC.updateError(error)

            # Change of value incoming. Flag for values are not utilized in
            # creating of the gradient because of the muscle lag
            if i % int(sectionLengthSES // TIME_STEP) == 1:
                estimateChangeIncoming = True

            # Change estimate at section+muscle lag.
            if (i % (int(sectionLengthSES / TIME_STEP))
                    == muscleLagEstimateIndx):

                period = int(current_time // sectionLengthSES) # 0, 1, 2, 3...


                if period != 0:
                    Optimizer_SES_BIC.updateOptimizer()

                    # Perturbate after adaption
                    # before adaption
                    Optimizer_SES_BIC.perputateGains()
                    gains_BIC = Optimizer_SES_BIC.getGains()
                    gains_TRI = np.negative(gains_BIC)

                    if PID_flag:
                        Pid_controller_BIC.updateGains(gains_BIC[0], gains_BIC[1],
                                                       gains_BIC[2])

                        Pid_controller_TRI.updateGains(gains_TRI[0],
                                                           gains_TRI[1],
                                                           gains_TRI[2])
                    else:  # PI
                        Pid_controller_BIC.updateGains(gains_BIC[0], gains_BIC[1],
                                                       0)

                        Pid_controller_TRI.updateGains(gains_TRI[0],
                                                       gains_TRI[1],
                                                       0)
                    estimateChangeIncoming = False
                    muscleLagAccounted = False

        if control_condition == 2 and current_time >= muscleLagEstimate: #PI(D)
            """ After initial muscle lag time, PID controller controls muscle
                angle
            """

            # UPDATE Arrays
            target_angle = target_angles[i]
            error = target_angle - ObservedAngle_rad[i - 1]

            if i % int(1/excitation_frequency/TIME_STEP) == 1:
                # Control output is done in wanted excitation freq
                excitation_BIC = Pid_controller_BIC.compute(
                    error, 1 / excitation_frequency)

                excitation_TRI = (Pid_controller_TRI.compute(
                        error, 1 / excitation_frequency))

        Kp_BIC, Ki_BIC ,Kd_BIC = Pid_controller_BIC.getGains()

        # COLLECTING DATA
        kp_col_BIC[i - 1] = Kp_BIC
        ki_col_BIC[i - 1] = Ki_BIC
        kd_col_BIC[i - 1] = Kd_BIC

        gain_Kp.append(Kp_BIC)
        gain_Ki.append(Ki_BIC)
        # gain_Kd.append(Kd_BIC)

        if SES_flag:
            gainEstimate_Kp.append(
                Optimizer_SES_BIC.getEstimateGains()[0])
            gainEstimate_Ki.append(
                Optimizer_SES_BIC.getEstimateGains()[1])
        """ gainEstimate_Kd.append(
                Optimizer_SES_BIC.getThetaCircumflex()[2])
        """
        # Simulate Muscle Lag: laten the input
        # ------------------------------#
        excitation_BIC = checkExcitation(excitation_BIC,
                                         excitation_BIC_min,
                                         excitation_BIC_max)

        excitation_TRI = checkExcitation(excitation_TRI,
                                         excitation_TRI_min,
                                         excitation_TRI_max)
        excitation_BIC_input = excitation_BIC
        excitation_TRI_input = excitation_TRI

        if add_delay_time != 0:
            excitation_BIC_input = delay_BIC[i%n_delay_samples_BIC]
            delay_BIC[i%n_delay_samples_BIC] = excitation_BIC
            excitation_TRI_input = delay_TRI[i%n_delay_samples_TRI]
            delay_TRI[i%n_delay_samples_TRI] = excitation_TRI
        # -------------------------------#
        if incrementalChange_flag:
            recruitment_name = (recruitmentDict[0]["name"] + " + " +
                                recruitmentDict[1]["name"])

            recruitment_func1 = recruitmentDict[0]["func"]
            recruitment_func2 = recruitmentDict[1]["func"]
            # Recruiment chraresteristics to the input
            excitation_BIC_input1 = getattr(module,
                                            recruitment_func1)(
                excitation_BIC_input,
                excitation_BIC_min,
                excitation_BIC_max)
            excitation_TRI_input1 = getattr(module,
                                            recruitment_func1)(
                excitation_TRI_input,
                excitation_TRI_min,
                excitation_TRI_max)
            # Recruiment chraresteristics to the input
            excitation_BIC_input2 = getattr(module,
                                            recruitment_func2)(
                excitation_BIC_input,
                excitation_BIC_min,
                excitation_BIC_max)
            excitation_TRI_input2 = getattr(module,
                                            recruitment_func2)(
                excitation_TRI_input,
                excitation_TRI_min,
                excitation_TRI_max)
            excDifB = excitation_BIC_input1 - excitation_BIC_input2
            excDifT = excitation_TRI_input1 - excitation_TRI_input2

            if excDifB < 0:
                lowerExcB = excitation_BIC_input1
            else:
                lowerExcB = excitation_BIC_input2

            if excDifT < 0:
                lowerExcT = excitation_TRI_input1
            else:
                lowerExcT = excitation_TRI_input2

            # make it lower by percentage of the time passed reagerds to end time
            excitation_BIC_input = lowerExcB + excDifB * (
                        1 - current_time / end_time)

            excitation_TRI_input = lowerExcT + excDifT * (
                        1 - current_time / end_time)

        else:
            if current_time > recruitmentPeriods[currentRecPeriod]:
                recruitment_name = recruitmentDict[currentRecPeriod]["name"]
                recruitment_func = recruitmentDict[currentRecPeriod]["func"]
                if changeRecruitment_Flag:
                    currentRecPeriod += 1

            # Recruiment chraresteristics to the input
            excitation_BIC_input = getattr(module,
                                           recruitment_func)(excitation_BIC_input,
                                                             excitation_BIC_min,
                                                             excitation_BIC_max)
            excitation_TRI_input = getattr(module,
                                           recruitment_func)(excitation_TRI_input,
                                                             excitation_TRI_min,
                                                             excitation_TRI_max)


        # Set the control value to the controller
        # 0 : BIC || 1: TRI || 2: BIC & TRI
        # BIC activation
        if target_muscle == 0 or target_muscle == 2:
            osimMuslceModel.changeExcitationBIC(excitation_BIC_input)

            #If TRI not used
            if target_muscle == 0:
                excitation_TRI_input = excitation_TRI_min
                excitation_TRI = excitation_TRI_min

        #TRI activation
        if target_muscle == 1 or target_muscle == 2:
            osimMuslceModel.changeExcitationTRI(excitation_TRI_input)

            # If BIC not used
            if target_muscle == 1:
                excitation_BIC = excitation_BIC_min

        # time_step, advance 1 step
        osimMuslceModel.setStateTime(current_time)

        #Integrate
        osimMuslceModel.integrateTime(current_time, TIME_STEP)

        # Data storage
        Time[i] = current_time
        # Normalize the excitation for simpler visualisation

        Excitation_BIC[i] = normalize(excitation_BIC,
                                      excitation_BIC_min,
                                      excitation_BIC_max)*100

        Excitation_TRI[i] = normalize(excitation_TRI,
                                      excitation_TRI_min,
                                      excitation_TRI_max)*100

        ObservedAngle_rad[i] = osimMuslceModel.getObservedAngle()

        if TargetFlag:
            Error_rad[i] = target_angles[i] -  ObservedAngle_rad[i]
            if(i > muscleLagEstimateIndx):
                Error_degreeDT_accounted[i] =  np.degrees((target_angles[
                                                          i-muscleLagEstimateIndx] -
                                         ObservedAngle_rad[i]))
            else: Error_degreeDT_accounted[i] = 0

        #Info every 1000 iterations
        if i % 10000 == 0:
            print("Iteration : Type: {} Time: {}/{} "
                  "Angle: {} Recruiment :{}".format(controller_name,
                                                           round(current_time,2),
                                                           end_time,
                                                           round(np.rad2deg(ObservedAngle_rad[i]),2),
                                                            recruitment_name))


        # VISUALIZE PROGRESS
        if plot_flag:
            current_times.append(current_time)
            state_values_ObAng.append(np.degrees(ObservedAngle_rad[i]))
            state_values_BIC.append(Excitation_BIC[i])
            state_values_TRI.append(Excitation_TRI[i])
            if TargetFlag:
                if i >= muscleLagEstimateIndx:
                    state_values_target.append(np.degrees(target_angles[
                                                              i - muscleLagEstimateIndx]))
                else:
                    state_values_target.append(np.degrees(target_angles[0]))

            if i % 1000 == 0:
                ax1.cla()  #Clear old graph
                #New data Plot
                ax1.plot(current_times, state_values_ObAng,
                        label='Observed Angle')

                if TargetFlag:
                    ax1.plot(current_times, state_values_target,
                             label='Target Angle')

                ax1.set_xlabel('Time [s]')
                ax1.set_ylabel('Angle [deg]')
                ax2.cla()  # clear
                ax2.plot(current_times, state_values_BIC, label='Excitation_BIC')
                ax2.plot(current_times, state_values_TRI, label='Excitation_TRI')
                ax2.set_xlabel('Time [s]')
                ax2.set_ylabel('Excitation %')
                plt.draw()
                plt.pause(0.01)

            if (i % (int((sectionLengthSES+TIME_STEP) / TIME_STEP))
                    == muscleLagEstimateIndx
                    and SES_flag):
                # New data Plot
                print("Time now", current_time)
                print("Constants now. ", Optimizer_SES_BIC.getThetaCircumflex())

                ax4.cla()
                ax3.cla()

                ax3.plot(current_times, gainEstimate_Kp,label='Kp',color='r')
                ax3.set_ylabel('Gain: Kp')
                ax3.tick_params(axis='y', labelcolor='r')

                # Plot Kp gain with a light red color

                ax3.plot(current_times, gain_Kp,color='#FF9999',linestyle =
                'dotted')

                ax4.plot(current_times,gainEstimate_Ki,
                         color = 'b', label='Ki')
                ax4.set_ylabel('Gain: Ki')
                ax4.tick_params(axis='y', labelcolor='b')

                # Plot gain Ki with a light blue color
                ax4.plot(current_times, gain_Ki, color='#99CCFF',
                         linestyle = 'dotted')

                ax3.set_xlabel('Time [s]')
                ax4.set_xlabel('Time [s]')

                ax5.cla()
                ax5.plot(Optimizer_SES_BIC.getCosts())
                ax5.set_ylabel('Cost Function', color='black')

    # When the simulation ends
    TIME_SIMULATION_END = time.time()
    elapsed_time = TIME_SIMULATION_END - TIME_SIMULATION_START
    print("Stimulation ended")
    print(f"Stimulation time: {elapsed_time}s")
    print("Saving data")

    ObservedAngle_degree = np.degrees(ObservedAngle_rad)


    if TargetFlag:

        Error_degree = np.degrees(Error_rad)
        Mae = np.average(np.abs(Error_degree[cut_index:]))
        Mae_col[0] = Mae

        Rmse = np.sqrt(np.average(np.square(Error_degree[:])))
        RMSE_DTAccounted = np.sqrt(np.average(np.square(
            Error_degreeDT_accounted[:])))
        print(f"RMSE = {Rmse}")
        print(f"RMSE_DT_accounted = {RMSE_DTAccounted}")
        Rmse_col[0] = Rmse
        Rmse_col[1] = RMSE_DTAccounted


    end_time_col[0] = end_time
    time_step_col[0] = TIME_STEP
    elapsed_time_col[0] = elapsed_time
    Excitation_BIC[0] = Excitation_BIC[1]
    Excitation_TRI[0] = Excitation_TRI[1]

    if save_flag:
        print("Adding to Excel")
        data = {
            "Time": Time,
            "Excitation_BIC": Excitation_BIC,
            "Excitation_TRI": Excitation_TRI,
            "ObservedAngle": ObservedAngle_degree,
            "TargetAngle": state_values_target,
            "ErrorAngle": Error_degree,
            "ErrorDTaccoundet": Error_degreeDT_accounted,
            "end_time": end_time_col,
            "TIME_STEP": time_step_col,
            "elapsed_time": elapsed_time_col,
            "MAE": Mae_col,
            "RMSE": Rmse_col,
            "Kp_BIC": kp_col_BIC,
            "Ki_BIC": ki_col_BIC,
            "Kd_BIC": kd_col_BIC,
        }

        folder_path = os.path.abspath(os.path.join(parent_dir, "Result_Folder"))

        saveDataToCSV(data, bug_flag, folder_path, iteration,
                      controller_name)



def checkExcitation(excitation, excitationMin, excitationMax):
    # Cap excitation to the extrenum values

    if excitation < excitationMin:
        excitation = excitationMin

    if excitation > excitationMax:
        excitation = excitationMax

    return excitation
