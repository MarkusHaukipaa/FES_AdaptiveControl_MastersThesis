"""
Control Panel is the main control element and by running it you run the whole
Program. Here you may change all main parameters affecting  muscle Muscle_models
characteristics, controller settings and trajectory to begin with. This
utilizes the Simulator to actualize the wanted test.

"""
import os
import numpy as np
from AdaptiveController_ElbowJointFES.Program.Main.PID_controller import PID
from Simulator import simulate
import json
from AdaptiveController_ElbowJointFES.Program.Sub.Get_text_from_file import get_text


# To Script file
current_dir = os.path.dirname(__file__)
Script_dir = os.path.abspath(os.path.join(current_dir, '..', "Sub"))

def control_panel():
    """
    Main function. Controls the world.
    """
    # Iterations for continuing testing
    total_iteration = 4
    iteration = 1

    while (iteration - 1) < total_iteration:

        # 0；Step　1；Ramp　2；PID
        control_condition = 2

        # ======================================#

        # Controlled muscle 　0；only BIC　1；TRI (CHR method)　2；BIC & TRI
        target_muscle = 2

        TIME_STEP = 1e-4 * 5 #0.0005 by trial and error is default
        excitation_frequency = 50 #Hz

        # ======================================#
        # Models       0         1       2      3        4          5         6           7            8         9
        models = ["default", "type1", "fat", "thin", "powerful", "weak", "athletic", "sedentary", "elderly", 'Aftest',

                  "tendontest", "testmodel", "default_TRI_fixed", "default_tau_fixed"]
                            # 10           11              12                   13'

        # Choose used Model
        model_type = models[0]


        # Target trajectories
        # 0-3: Rectangular Waves (2, 4, 6 adn 40 s)
        # 4: Sin wave 5: Random 6: Step 7: Trapezoidal;
        # 8:constant, 9: Complex: (Trapezoidal+ Sin + Rand+ Level)
        target_pattern = 7

        # ======================================#
        # Flags
        # ======================================#
        plot_flag = True
        SES_flag = True
        save_flag = True
        PID_flag = False
        incrementalChange_flag = False
        Fixed_controller = 0
        
        if target_pattern == 7: #change recruitment only in trapezoidal
            changeRecruitment_Flag = True
        else: changeRecruitment_Flag = False

        Flags = [plot_flag, SES_flag, save_flag, PID_flag,
                 incrementalChange_flag,  changeRecruitment_Flag]

        # ======================================#

        # Start and stop time based on controller selection
        quiet_time, end_time = setUpController(control_condition,target_pattern)

        # Random wave period ( if included in trajectory (5&9)
        # 0 short, 1 medium (TEST default), 2 large 3: any
        target_period = 1

        #Training muscle models ( See Program/Sub/Recruitment_characteristics.py)
        # "0": "linear", "1": "log", 2：x^2 3：x^3 4：FOD5 8：default 10：FOD4
        # 11：FOD3 12：FOD2.5, 13：FOD2: 14：FOD1.5 15：FOD1 16：FOD0.5 17：x^1.5
        # 18：x^1.07 19：x^1.3, :  21~40：rec1~20

        # Pairs used in the research and made to inflict change of muscle Muscle_models
        recruitmentPairs = [ [15 + 20, 19 + 20], #1-2
                            [9 + 20, 6 + 20], #3-4
                            [5 + 20, 18 + 20]] #5-6

        recruitment_pattern_1  = recruitmentPairs[2][0]
        recruitment_pattern_2 = recruitmentPairs[2][1]

        recruitments = [recruitment_pattern_1,recruitment_pattern_2]

        # The muscle lag addition is calculated so that added Arm26 herited
        # muscle lag of 0.082, the muscle lag adds up to :

        # Deadtime / Lag Size:     NO L     Small   Medium    Large   ExtraLarge
        # In milliseconds          0：0ms  1：150ms　2：225ms　3：300ms　4: 350ms
        delay_time_pattern = 1

        add_delay_time_BIC, add_delay_time_TRI = getDelayedTime(delay_time_pattern)

        key = 0
        recruitmentDict = {}
        for recruitment in recruitments:
            recruitment_name, recruitment_func = getRecruitment(recruitment)
            recruitmentDict[key] = {"name":recruitment_name,
                                    "func":recruitment_func}
            key += 1


        model_file_name = "arm26.osim"
        IniSta_file_name = "InitialState.sto"

        model_folder_name = f"{Script_dir}/../../Muscle_models/{model_type}"
        # Initial state folder
        IniSta_folder_name = model_folder_name

        initial_sto_file = "{}/{}".format(IniSta_folder_name,
                                          IniSta_file_name)
        INITIAL_ANGLE_RAD = np.radians(float(get_text(initial_sto_file,
                                                      8,
                                                      31,
                                                      44)))


        # Input settings, according to OSim muscle excitation
        #======================================#
        excitation_BIC_max = 0.14
        excitation_TRI_max = 0.15
        excitation_BIC_min = 0.01
        excitation_TRI_min = 0.01
        # ======================================#

        # Default parameters: (determined with tdCalc with default settings)
        SensitivityTuner =  0.425 # Decrease tuning
        tau = 0.425 # time constant
        L = 0.082 + add_delay_time_BIC  # deadtime , 0.082 is the inherit
                                        # deadtime ot Arm26 Muscle_models
        print("Deadtime :", L, " and ratio TC/DT",  tau/L)
        R =   2.929546 #295457131235865 (greatest change, output/input in
        # radians)
        Xo = excitation_BIC_max-excitation_BIC_min # change in control output

        # Ziegler-Nicholas like tuning

        if PID_flag:
            Kp = 1.2* Xo * SensitivityTuner / (R * L )
            Ti = (2 * L)
            Td = (0.5 * L )

        else:
            Kp = 0.9 * Xo  *SensitivityTuner / (R * L )
            Ti = (3.33 * L )
            Td = 0

        Ki = Kp / Ti
        Kd = Kp * Td


        print("Inital gains: Kp:", Kp,"Ki:", Ki,"Kd:", Kd)
        print("Inital 'constants': K:", Kp, "Ti:", Ti, "Kd:", Td)

        Pid_Controller_BIC = PID(Kp, Ki, Kd)
        Pid_Controller_TRI = PID(-Kp, -Ki, -Kd)



        # Used if want to try best values found ( if manually tuned constant
        # value PID controller would have succeeded)
        if Fixed_controller:
            # [Kp,Ki,Kd]
            gains = [Kp,Ki,Kd]
            Pid_Controller_BIC.updateGains(gains[0],gains[1],gains[2])
            #SES_flag =
            Flags[1] = False # SES_flag



        #SES lenght of estimate change
        sectionLengthSES = 8 #8

        simulate(TIME_STEP, control_condition, target_muscle,
                 target_pattern, quiet_time, end_time,
                 excitation_BIC_max, excitation_TRI_max,
                 excitation_BIC_min, excitation_TRI_min,
                 model_file_name, model_folder_name,
                 INITIAL_ANGLE_RAD, initial_sto_file,
                 iteration,
                 Flags, target_period,
                 recruitmentDict,
                 L,
                 Pid_Controller_BIC, Pid_Controller_TRI,
                 sectionLengthSES,
                 excitation_frequency)

        iteration += 1

def setUpController(control_condition,target_pattern):
    """
    Sets up target time window according to controller chosen
    Args:
        control_condition: int, controller by nmbr

    Returns: quiet time(start time)  _ int, end time _ int

    """
    # Step
    if control_condition == 0:
        quiet_time, end_time = 5, 20
    # Ramp
    elif control_condition == 1:
        quiet_time, end_time = 0, 20

    # PID (and fel)
    elif control_condition == 2 or control_condition == 6:
        if target_pattern != 7:
            quiet_time, end_time = 0,250
        else: quiet_time, end_time = 0, 4*16

    return quiet_time,end_time

def getRecruitment(recruitment_pattern):
    """
    reads and returns the simulated muscle recruiment Muscle_models. Essentially how
    the muscle responds to different magnitudes of excitement . Made to
    increase individual differences and nonlinearity.
    Args:
        recruitment_pattern: int, number code of the pattern

    Returns: String, name of recruitment, String

    """
    # recruitment_names from JSON
    recruitment_names_path = os.path.join(Script_dir,
                                          'recruitment_names.json')
    with open(recruitment_names_path, 'r') as f:
        recruitment_names_data = json.load(f)
    recruitment_name = recruitment_names_data.get(str(recruitment_pattern),
                                                  "Unknown")
    recruitment_func_names_path = os.path.join(Script_dir,
                                               'recruitment_func_names.json')


    with open(recruitment_func_names_path, 'r') as f:
        recruitment_func_names_data = json.load(f)
    recruitment_func_name = recruitment_func_names_data.get(
        str(recruitment_pattern), "Unknown")

    return recruitment_name, recruitment_func_name

def getDelayedTime(add_delay_time_pattern_target):
    """
    Gets the muscle lag / Deadtime / delay from json

    Args:
        add_delay_time_pattern_target:  int , number name of delay

    Returns: float, delays of TRI and BIC

    """
    # Add delayed_time
    add_delay_time_path = os.path.join(Script_dir,
                                       'add_delay_time.json')
    with open(add_delay_time_path, 'r') as f:
        add_delay_time_data = json.load(f)
    add_delay_time_BIC = add_delay_time_data[str(add_delay_time_pattern_target)]["BIC"]
    add_delay_time_TRI = add_delay_time_data[str(add_delay_time_pattern_target)]["TRI"]

    return add_delay_time_BIC, add_delay_time_TRI

if __name__ == '__main__':
    control_panel()