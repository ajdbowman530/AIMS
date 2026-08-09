# AIMS
Aircraft Integrated Management Sandbox (AIMS) is a aircraft control sandbox using JSBSim as a flight dynamics model.

# Setup
To run AIMS the C++ code must be compiled into a .pyd (Windows) or .so (MacOS/Linux) file and the libraries listed in requirements.txt must be installed. It is recommended to set up a Conda environment and install Slycot using conda-forge.

To compile on Windows, open Anaconda Prompt, cd into "/aims_telemetry/src" and enter:
```bash
cmake --build .\build_fresh --config Release
```

After the .pyd finishes compiling, copy the file into the AIMS folder.
