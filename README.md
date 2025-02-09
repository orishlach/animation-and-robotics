# animation-and-robotics


## Instructions

Preliminary steps:

1. Install Chocolatey, and use it to install Python and VS Code.
2. Open VS Code and install the Python extension (`CTRL-SHIFT-X` and search `python` and then `install`), and Jupyter extension.

Setup steps

1. clone the repository with `git clone`:

    `git clone https://github.com/orishlach/animation-and-robotics.git`
    
2. Open the folder with VS Code.
   
3. Choose folder and create a new Python environment (`CTRL-SHIFT-P`, type `python env` and select `Python: Create Environment`). Follow the steps. VS Code should create a new folder called `.venv`.
4. Open a new terminal (`` CTRL-SHIFT-` ``). If VS Code detected the Python environment correcly, the prompt should begin with `(.venv)`. If not, restart VS Code and try again. If it still doesn't make sure the default terminal is `cmd` or `bash` (use `CTRL-SHIFT-P` and then `Terminal: Select Default Profile` to change it) and start a new terminal. If it still fails, ask for help.
5. Install Vedo, a scientific visualization package for python, using `pip install vedo` in the terminal.
6. How to run:
   
   - **1-optimization-and-visualization-basics**
  
        Open `Assignment1.py` . The file is divided into cell, where each cell is defined by `#%%`. Run the first cell, which has this code, but pressing `CTRL-ENTER`.
        ```python
            #%% Imports
            import vedo as vd
            import numpy as np
            from vedo.pyplot import plot
            from vedo import Latex

            vd.settings.default_backend= 'vtk'
        ```

        On the first run, VS Code will tell you that it needs to install the ipykernel runtime.
        - Run the whole file, cell-by-cell, by pressing `SHIFT-ENTER`. Running the last cell should result in a new window appearing, with a surface on it.

    - **2-deformation-mass-spring-systems**
  
        Open `Assignment2.py`. The file is divided into cells, where each cell is defined by `#%%`. Run the first cell. Recall that VS Code will tell you that it needs to install the ipykernel runtime. Make sure it ran without any errors.

    - **3-kinematics**
        Open `Assignment3.py`. The file is divided into cells, where each cell is defined by `#%%`. Run the first cell. Recall that VS Code will tell you that it needs to install the ipykernel runtime. Make sure it ran without any errors.
        - Run all the cells. A window with a 2D robotic arm should appear.
