# AIMS
Aircraft Integrated Management Sandbox (AIMS) is a aircraft control sandbox using JSBSim as a flight dynamics model. 

Details on the features, architecture, and design of AIMS can be found in AIMS_Block_0.pdf. This file will explain the setup and use of AIMS.

# Setup
To run AIMS the C++ code must be compiled into a .pyd (Windows) or .so (MacOS/Linux) file and the libraries listed in requirements.txt must be installed. It is recommended to set up a Conda environment and install Slycot using conda-forge.

To compile on Windows, open Anaconda Prompt with the desired Conda environment activated, cd into "/aims_telemetry/src" and enter:
```bash
cmake -B build_fresh -S .
cmake --build .\build_fresh --config Release
```
After the .pyd finishes compiling, copy the file into the AIMS folder.

# Main Menu
AIMS is run using main.py. AIMS opens into the main menu.
```bash
=======AIMS=======
No aircraft model read
[1]  Read gains from JSON
[2]  Launch controller tuning manage
[3]  Flight Planner
[4]  Launch FDM Environment
[5]  Launch Output Visualization
[Q]  Quit
Make a selection:
```
Entering "1" will prompt you to choose a aircraft .json file to read in. Aircraft files must be in "AIMS/gain schedules" in order to be detected and read. When new aircraft are created using the tuning manager they will be automatically saved to this folder. After an aircraft file is read, it can be modified in the controller tuning manager or used in the FDM environment. 

"2" will launch the tuning manager. The tunning manager allows you to create a new aircraft gain schedule or to modify and existing aircraft's controller. Use of the tuning manager will be discussed in greater detail.

"3" starts the flight planner, allowing the user to create a flight plan. A flight plan is a matrix of state commands over a set time with a set number of intervals per second. Flight plans are saved to a .json file and can be read by the FDM environment. 

"4" launches the FDM environment and allows the user to test the controller with a flight plan. Both a flight plan and an aircraft are required to use the FDM environment. Output from the FDM environment must be saved as a .csv in order to be visualized. 

"5" launches output visualization, a fairly basic GUI used to plot the state output from FDM env. 

# Tuning Manager
In order to run the FDM gains must be read, which can be done in either the main menu or from the FDM environment menu. AIMS will automatically detect .json files in the 'gain schedules' folder. The example aircraft file used in AIMS_Block_0.pdf is included. 

If it is desired to modify an existing gain schedule, first load the gains from .json using '1' in the AIMS main menu, then launch the tuning manager with '2' in the main menu, after which the menu should be:
```bash
Current aircraft: f15Test.json
[1] Modify current aircraft
[2] Create new aircraft
[Q] Return to main menu
Enter selection:
```
To modify the aircraft enter '1', and something similar to this should be displayed:
```bash
=== Linearization Points Database: f15Test.json ===
  ID   |  Mach |   q (Pa) |  Flap |  Gear | Fuel/Pl (lb) |  Alt (ft) |  V (kts)
  -------------------------------------------------------------------------
  0    |  0.65 |    13828 |     0 |     0 |       5000.0 |   20000.0 |    400.0
  1    |  0.65 |    13828 |     0 |     0 |        500.0 |   20000.0 |    400.0
  2    |  0.65 |    13828 |     0 |     0 |      10000.0 |   20000.0 |    400.0
  3    |  0.81 |    21606 |     0 |     0 |       5000.0 |   20000.0 |    500.0
  4    |  0.49 |     7778 |     0 |     0 |       5000.0 |   20000.0 |    300.0
  5    |  0.68 |     9719 |     0 |     0 |       5000.0 |   30000.0 |    400.0
  6    |  0.63 |    19156 |     0 |     0 |       5000.0 |   10000.0 |    400.0


[A]dd a tuning point
Make a [C]opy of a tuning point
[M]odify a tuning point
[D]elete a tuning point
[S]ave gain schedule to JSON
[R]ead a linearization point
[Q]uit to AIMS menu
Enter selection:
```
Adding a tuning point with 'A' will prompt the user to create a new LinearizationPoint for the Aircraft. The process of creating a new point starts with prompting the user for an altitude (ft), and airspeed (kts), a fuel weight (lbs), a flap position, and a gear position. AIMS will linearize the aircraft for the specified condition and display the state space matrices if it is possible to trim the aircraft at the specified condition. 

The process of tuning the LQR controller involves choosing values for the Q and R matrices. Increasing the value of Q for a given variable increases the error penalty the controller applies for the selected state while increasing R increases the penalty of using the selected control input. The command line interface during tuning will look something like:
```bash
Longitudinal tuning: 

Q values: 
[1]     Q_int_q = 1.0
[2]     Q_alpha = 1.0
[3]     Q_q = 1.0
[4]     Q_delta_e = 1.0

R values: 
[5]     R_delta_e_cmd = 1.0

Select a value to modify. Enter "c" to continue.
```
To change a weight, enter the number associated with the desired variable and enter the new value. 

After the Q and R matrices are set as desired, stability metrics are shown similar to this. Eigenvalue, step response, Bode, Nyquist, and singular values plots can be generated if desired. 
```bash
  Gain Margin         : 22.82 dB
  Phase Margin        : 66.99 deg
  Overshoot           : 9.04 %
  Damping             : [0.568, 0.568, 1.000, 1.000]
  Rise Time           : 0.26 sec
  Settling Time       : 1.79 sec
  Eigs:
    [0]  -2.5666 + 3.7218j
    [1]  -2.5666 - 3.7218j
    [2]  -4.8575
    [3]  -0.6229
  Peak Sensitivity    : 1.00
  Peak Cosensitivity  : 1.10
```

Copying a point with 'C' will create a duplicate the selected point.

The process of modifying a point with 'M' is similar to that of creating a point except that each of the values are pre-filled with those of the selected point.

Saving the gain schedule with 'S' will prompt the user for a filename if there is not already one, but if there is already a filename it will overwrite the old file with the current gain schedule.

Reading a point with 'R' will allow the user to see the flight conditions, state-space matrices, stability metrics, and stability plots associated with the selected point. 

# Flight Planner
Similar to the aircraft gains file, a flight plan file must be read in order for the FDM environment to run. An existing flight plan can be modified by loading it in the Flight Planner and creating new commands or a new flight plan can be generated by declining to read a flight plan before launching the Flight Planner. The Flight Planner menu looks like:
```bash
Welcome to the Flight Planner!
Loaded flight plan: multimodalTest
[A]dd command
[M]odify aircraft configuration
[P]lot flight plan
[S]ave flight plan
[C]hange settings
[Q]uit Flight Planner
Enter selection:
```

The flight plan is specified using step, ramp, and sine commands, added by selecting 'A' at the Flight Planner menu. Currently, commands may be made to V, alpha, q, beta, p, or r. Note that if the controller does not have an integrator on a state, commands to the state cannot be executed. All commands have a specified initial and final time, step commands have a value, ramp commands an initial and final value, and sine commands an amplitude and frequency. 

The gear and flap configuration can be changed by selecting 'M'. Similar to commands a initial and final time for the position is specified.

By selecting 'C', the FDM simulation frequency (and thus the timestep), the autothrottle frequency, aerodynamic controller frequency, and simulation time can be modified. 

By selecting 'P', the current flight plan is plotted.

# FDM Environment
When a aircraft file and a flight plan file are read by AIMS, the FDM Environment menu opens:
```bash
Aircraft model f15Test.json loaded.
Flight plan multimodalTest loaded.

[L]oad flight plan from .json
[P]lot loaded flight plan
[O]pen flight planner
[E]xecute flight plan
[M]odify FDM settings
[Q]uit
Enter selection:
```

'L' can be used to load a different flight plan and 'P' can be used to plot the currently loaded flight plan. If a modified or new flight plan is desired, the Flight Planner can be quickly opened using 'O'. 'M' opens the same settings menu that 'C' in the Flight Planner does. 

To run the flight plan with the loaded aircraft controller, enter 'E'. If the flight plan successfully executed, the CLI should return:
```bash
[SUCCESS] Flight plan executed successfully.
Save results to CSV? (Y/N) 
```
To visualize the output, enter 'Y' and, when prompted, a filename.

# Output Visualizer
To visualize output, select the desired .csv file.

Currently the Output Visualizer has up to four plots (by default longitudinal, lateral, airspeed, and control input plots), although fewer plots can be displayed if desired by unchecking one or more 'Display plot' checkboxes. A state can be added or removed from a plot by selecting a state from the dropdown menu in the plot container and clicking the 'Add/Remove' button. The 'Settings' button for each plot allows the user to change the legend visibility/location, axis scaling, and line colors. 
