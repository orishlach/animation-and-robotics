<div align="center">
  <h1>Animation and Robotics</h1>
  <p>Interactive computational methods for optimization, deformation, and kinematics</p>
  
  <!-- Badges -->
  <p>
    <img src="https://img.shields.io/badge/Python-3.8+-blue.svg" alt="Python" />
    <img src="https://img.shields.io/badge/Vedo-Scientific%20Visualization-green.svg" alt="Vedo" />
  </p>
  
  <!-- Links -->
  <h4>
    <a href="#projects">View Projects</a>
    <span> · </span>
    <a href="#quick-start">Quick Start</a>
  </h4>
  
   
</div> 

> [!NOTE]
> - This repository contains **3 comprehensive projects**,<br> exploring the fundamental computational methods used in *animation and robotics*.
> - Each project combines theoretical foundations with **interactive GUI** implementations,<br> allowing real-time parameter manipulation and immediate visual feedback.

## Projects

<details>
<summary>
Optimization & Visualization
</summary>
<p><em>Interactive optimization algorithms with real-time visualization</em></p>

**Features:**
- Interactive 3D surface navigation
- **Gradient Descent vs Newton's Method** comparison
- Real-time parameter adjustment
- Visual convergence tracking
</details>

<div>
<a href="1-optimization-and-visualization-basics" target="_blank">
  <img src="1-optimization-and-visualization-basics/screenshots/right-click.gif" width="250"/>
</a>

> Click the image to checkout the project.

</div>

##

<details>
<summary>Mass-Spring Systems</summary>
<p><em>Deformable mesh simulation with interactive controls</em></p>
 
**Features:**
- Custom mesh generation (donut geometry)
- Click-and-drag vertex manipulation
- Multiple energy models
- Real-time deformation simulation
</details>

<div>


<a href="2-deformation-mass-spring-systems" target="_blank">
  <img src="2-deformation-mass-spring-systems/screenshots/2.gif" width="250"/>
</a>

> Click the image to checkout the project.

</div>

##

<details>
<summary>Kinematics</summary>
<p><em>2D robotic arm with forward and inverse kinematics</em></p>

**Features:**
- Configurable arm geometry
- Interactive target positioning
- **Jacobian** visualization
- Multiple **IK algorithms**
</details>

<div>
  
<a href="3-kinematics" target="_blank">
  <img src="3-kinematics/screenshots/2.1-arrows_2.png" width="250"/>
</a>

> Click the image to checkout the project.

</div>

##

<details>
<summary>Path Planning (RRT)</summary>
<p><em>Interactive simulation of the Rapidly-exploring Random Tree algorithm</em></p>

**Features:**
- Obstacle-aware path planning
- Configurable step size & goal bias
- Real-time tree growth visualization
- Extracted final path with highlighted waypoints
</details>

<div>

<a href="4-path-planning" target="_blank">
  <img src="4-path-planning/screenshots/run.gif" width="250"/>
</a>

> Click the image to checkout the project.

</div>

## Quick Start

1. clone the repository with `git clone`:

    `git clone https://github.com/orishlach/animation-and-robotics.git`
    
2. Open the folder with VS Code.
   
3. Choose folder and create a new Python environment (`CTRL-SHIFT-P`, type `python env` and select `Python: Create Environment`).<br>Follow the steps. VS Code should create a new folder called `.venv`.
4. Open a new terminal (`` CTRL-SHIFT-` ``). If VS Code detected the Python environment correcly, the prompt should begin with `(.venv)`.<br>If not, restart VS Code and try again.<br>
   If it still doesn't make sure the default terminal is `cmd` or `bash` (use `CTRL-SHIFT-P` and then `Terminal: Select Default Profile` to change it) and start a new terminal.
6. Install [Vedo](https://vedo.embl.es/) , a scientific visualization package for python, using `pip install vedo` in the terminal.
7. How to run:
   
   - The file is divided into cell, where each cell is defined by `#%%`.
   - Run the first cell, which has this code, but pressing `CTRL-ENTER`.

    - On the first run, VS Code will tell you that it needs to install the ipykernel runtime.
    - Run the whole file, cell-by-cell, by pressing `SHIFT-ENTER`.
    - Running the last cell should result in a new window appearing with the graphics.
 
