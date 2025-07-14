"""The total simulation time (end_time = 20) and the TIME_STEP (= 1e-4 * 5) must be aligned for both the step response and the ramp response.

Step response:
    From 0 to 5 seconds: no input (excitation = 0.01)
    From 5 to 15 seconds: maximum input (excitation = 0.14 for BIC, or 0.15 for TRI)
    (quiet_time, end_time = 5, 20)
Ramp response:
    The ramp input starts immediately at the beginning of the simulation.
    The ramp input is applied over 20 seconds from 0 to 20 seconds.
    (quiet_time, end_time = 0, 20)
For the ramp response, extract the waveform under the conditions below and calculate the gain using the least squares method within this interval:
    Movement threshold: The first time the angle reaches the initial angle + 0.5°
    Maximum angle: The first time the angle reaches the final target angle - 0.5°

To obtain the ramp and step responses of the triceps, use the Muscle_models file "arm26_for_TRI_PID.osim." This Muscle_models file is configured so that the shoulder
joint angle is fixed at 90° rearward, and the elbow joint angle is approximately 90° when relaxed. If you make any changes to the Muscle_models parameters,
you must apply those same modifications to both "arm26_for_TRI_PID.osim" (for measuring the triceps' time delay and time constant) and the "arm26.osim"
Muscle_models that you normally use, ensuring that all settings are reflected consistently.
"""
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import glob

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))
mainDir = os.path.dirname(__file__)
# -------------------------------------
# Settings
# -------------------------------------
file_select = 0
plot_Flag = 1
condition = 0 # 0 : Biceps (BIC) 1: Triceps (TRI)

# -------------------------------------
# Explanation of parameters:
#
# file_select:
#    If file_select = 0:
#       Automatically obtains the step response and ramp response CSV files
#       from the specified folder. The step response file must be named in the
#       form '0_Step_*.csv' and the ramp response file in the form '0_Ramp_*.csv'.
#       If multiple CSV files match these patterns, the first file in name order
#       will be selected (e.g., step_file_path = step_files[0]).
#       Because of this, it is recommended to use file_select = 0 only when there
#       is exactly one step response file and one ramp response file in the specified folder.
#
#    If file_select = 1:
#       You must directly specify the full paths of the step response and ramp response CSV files.
#
# plot_Flag:
#    Controls whether to plot the ramp response gain and the tangent of the step response.
#    plot_Flag = 0: Do not plot
#    plot_Flag = 1: Plot
#
# condition:
#    Indicates whether calculations are for the biceps or triceps.
#    condition = 0: Biceps (BIC)
#    condition = 1: Triceps (TRI)
#    This affects how data is obtained from the CSV files, as well as the selection
#    of maximum or minimum slope points for the inflection point detection.
#
# When file_select == 0:
#    Specify folder_path as the directory containing both the step and ramp response CSV files.
#
# When file_select == 1:
#    Directly specify the full file paths in the arguments of pd.read_csv().
# -------------------------------------

# If file_select = 0, automatically find the step and ramp CSV files
if file_select == 0:
    folder_path = f'{parent_dir}\ResultFile'
    print(folder_path)
    print(os.listdir(folder_path))
    step_files = glob.glob(os.path.join(folder_path, '1_Step_*.csv'))
    ramp_files = glob.glob(os.path.join(folder_path, '1_Ramp_*.csv'))
    if step_files:
        step_file_path = step_files[0]
    else:
        print('No Step file found.')
        sys.exit()

    if ramp_files:
        ramp_file_path = ramp_files[0]
    else:
        print('No Ramp file found.')
        sys.exit()

    Step = pd.read_csv(step_file_path)
    Ramp = pd.read_csv(ramp_file_path)

# If file_select = 1, directly specify the full paths to the CSV files
elif file_select == 1:
    Step = pd.read_csv(r"{mainDir}/Responsees/Step")
    Ramp = pd.read_csv(r"{mainDir}/Responsees/Ramp")

Time = 'Time'
if condition == 0:
    Exc = "Excitation_BIC"
elif condition == 1:
    Exc = "Excitation_TRI"

Angle = "ObservedAngle"
time_step = Step["TIME_STEP"][0]

# Ensure time_step matches between Step and Ramp responses
if time_step != Ramp["TIME_STEP"][0]:
    print('The time_step of the Step response and the Ramp response differ. Stopping Program.')
    sys.exit()

ta = int(5 / time_step)  # Range for inflection point search
t = np.arange(0, Step["end_time"][0] + time_step, time_step)

# Determine the start and end indices of nonzero excitation in step and ramp responses
T_Step_Start = Step[Step[Exc] > 0.01].index[0]
T_Step_Finish = Step[Step[Exc] > 0.01].index[-1]
T_Ramp_Start = Ramp[Ramp[Exc] > 0.01].index[0]
T_Ramp_Finish = Ramp[Ramp[Exc] > 0.01].index[-1]

# Determine L and T from the step response by finding the inflection point
delta = [Step[Angle][i] - Step[Angle][i - 1] for i in range(T_Step_Start, T_Step_Finish)]


if condition == 0:
    # For biceps: use maximum slope
    max_delta = max(delta[:ta])
elif condition == 1:
    # For triceps: use minimum slope
    max_delta = min(delta[:ta])

ta = delta.index(max_delta)  # Index of the inflection point

slope = max_delta / time_step  # Slope at the inflection point
print("slope is in deg: ",slope )
print("slope is in rad: ", np.radians(slope) )


# Calculate dead time (L) and time constant (T) (the slope to get 63.3% outpuit)
t0 = Step[Time][T_Step_Start + ta] - ((Step[Angle][T_Step_Start + ta] - Step[Angle][T_Step_Start]) / slope)
t1 = Step[Time][T_Step_Start + ta] - ((Step[Angle][T_Step_Start + ta] -
                                        Step[Angle][T_Step_Finish])*0.633 /
                                      slope)
L = t0 - Step[Time][T_Step_Start]
T = t1 - t0
# Compute threshold indices for the ramp response linear approximation
if condition == 0:
    # For biceps: angles increase, so pick threshold accordingly
    threshold_start = Ramp[Angle][0] + 0.5
    threshold_end = Ramp[Angle].iloc[-1] - 0.5
    threshold_index = Ramp[Ramp[Angle] > threshold_start].index[0]
    final_index = Ramp[Ramp[Angle] > threshold_end].index[0]
elif condition == 1:
    # For triceps: angles decrease, so pick threshold accordingly
    threshold_start = Ramp[Angle][0] - 0.5
    threshold_end = Ramp[Angle].iloc[-1] + 0.5
    threshold_index = Ramp[Ramp[Angle] < threshold_start].index[0]
    final_index = Ramp[Ramp[Angle] < threshold_end].index[0]

# Perform a linear fit on the ramp response between threshold_index and final_index
p = np.polyfit(Ramp[Exc][threshold_index:final_index],
               Ramp[Angle][threshold_index:final_index], 1)
p1 = np.polyval(p, Ramp[Exc][threshold_index:final_index])
print("thisthing" , p[0])
p[0] = np.radians(p[0])  # Convert slope from degrees to radians
print("thisthing" , p[0])
# Compute PID parameters based on Ziegler-Nichols-like tuning
Kp = 0.6 * T / L / p[0]
Ki = 0.6 / L / p[0]
Kd = 0.3 * T / p[0]

PI_Kp = 0.9 * 0.13 * T / (slope * L )
PI_Ki = Kp / (3.33 * L)

print(f"excitation_threshold: {Ramp[Exc][threshold_index]}")
print(f"excitation_max: {Ramp[Exc][final_index]}")
print(f"dead time L: {L} ")
print(f"time constant T: {T} ")
print(f"ramp gain m: {p[0]} ")
print(f"Kp: {Kp} ")
print(f"Ki: {Ki} ")
print(f"Kd: {Kd} ")
print(f"[{Kp}, {Ki}, {Kd}]")
print(f"[{PI_Kp}, {PI_Ki}, 0]")
print(f"[{Ramp[Exc][threshold_index]}, {Ramp[Exc][final_index]}]")

# Prepare for plotting the tangent line on the step response
t00 = round(t0 / time_step) * time_step  # Closest multiple of time_step to t0
t00_T = round((t0 + T) / time_step) * time_step
t2 = np.arange(t00, t00_T, time_step)
a = []
for j in range(len(t2)):
    x = slope * (t2[j] - (T_Step_Start + ta) * time_step) + Step[Angle][T_Step_Start + ta]
    a.append(x)

# Plotting results (if plot_Flag = 1)
plt.figure(1)

ax1 = plt.gca()
ax1.plot(t, Step[Angle], 'b-', label="Step Response")
ax1.plot(t2, a, 'g--', label="Tangent")
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Elbow Joint Angle [deg]', color='b')
ax1.tick_params('y', colors='b')
ax1.legend(loc='center right')

ax2 = ax1.twinx()
ax2.plot(t, Step[Exc], 'r-', label="Excitation")
ax2.set_ylabel('Excitation', color='r')
ax2.tick_params('y', colors='r')
ax2.legend(loc='center')

plt.figure(2)
plt.plot(Ramp[Exc][threshold_index:final_index], Ramp[Angle][threshold_index:final_index])
plt.plot(Ramp[Exc][threshold_index:final_index], p1, 'r--')
# plt.plot(Ramp[Exc][threshold_index:final_index], p, 'g--')
plt.xlabel('Excitation')
plt.ylabel('Elbow Joint Angle [deg]')

if plot_Flag:
    plt.show()