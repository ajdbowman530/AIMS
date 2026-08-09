# AIMS
Aircraft Integrated Management Sandbox (AIMS) is a aircraft control sandbox using JSBSim as a flight dynamics model. 

Details on the features and design of AIMS can be found in the reports folder. This file will explain the setup and use of AIMS.

# Setup
To run AIMS the C++ code must be compiled into a .pyd (Windows) or .so (MacOS/Linux) file and the libraries listed in requirements.txt must be installed. It is recommended to set up a Conda environment and install Slycot using conda-forge.

To compile on Windows, open Anaconda Prompt with the desired Conda environment activated, cd into "/aims_telemetry/src" and enter:
```bash
cmake --build .\build_fresh --config Release
```

After the .pyd finishes compiling, copy the file into the AIMS folder.

# Tutorial
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
