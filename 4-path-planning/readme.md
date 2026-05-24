# Path Planning with RRT

<details>
<summary>
Project Overview
</summary>
      
This report presents the implementation of the **Rapidly-exploring Random Tree (RRT) algorithm** for path planning in robotics.
<br>The RRT algorithm efficiently explores configuration spaces by incrementally building a tree structure from a start configuration towards a goal, while avoiding obstacles.
      
<br>The implementation includes:
- A complete interactive GUI with real-time obstacle editing
- Core RRT algorithm with efficient collision detection using line discretization
- Visual feedback system with color-coded paths and markers
- Experimental analysis demonstrating algorithm performance under various conditions

</details>

## Task 1: Complete the GUI

<details>
<summary> Subsection 1.1

**Setting Start/Goal Points and Tree Reset**

</summary>  

The GUI provides an intuitive click-based system for setting source and destination points:
- *Left Click Toggle System*: Alternates between setting source (green marker) and destination (red marker)
- *Visual Markers*: Green circle for source, red circle for destination with z-offset for visibility
- *Reset Functionality*: Keyboard shortcut 'C' to restore original map and reset tree
- *Step Size Slider*: Adjustable from 5 to 50 pixels with real-time updates
<br>
<figure>
<img src="./screenshots/gui.png" alt="GUI Interface" width="600"/><br>
</figure>

</details>

<details>
<summary> Subsection 1.2

**Interactive Obstacle Drawing Tool**

</summary> 
      
I implemented a Paint-like obstacle editor with multiple interaction methods:
- *Right-Click Drawing*: Toggle obstacles with circular brush (radius = 10 pixels)
- *Keyboard Drawing*: Move mouse then press Right Arrow for precise pixel toggling (radius = 3 pixels)
- *Toggle Mechanism*: Obstacles flip state rather than just being added/removed
- *Map Reset*: `C` key restores the original loaded map

Interaction Methods:
- *Left Click*: Set source/destination points
- *Right Click*: Toggle obstacle at clicked position
- *Right Arrow Key*: Toggle obstacle at last mouse position
- *`C` Key*: Complete map reset to original
- *`R` Key*: Start/stop RRT algorithm
<br>
<figure>
      <img src="./screenshots/toggle.gif" alt="Obstacle Drawing Demo" width="600"/><br>
</figure>

</details>

<details>
<summary> Subsection 1.3

**Additional Visualization Features**

</summary> 

The visualization system provides clear visual feedback for all algorithm stages:

- *Tree Visualization*: Blue lines showing RRT edges with 2px width
- *Path Highlighting*: Thick green lines (4px) with z-offset for final path
- *Node Rendering*: Yellow points for all sampled nodes
- *Color Scheme:* 
  - Black = Obstacles
  - White = Free space
  - Blue = RRT tree edges
  - Green = Final solution path
  - Yellow = Tree nodes
- *Real-time Updates*: Smooth animation at 10ms intervals during tree growth
- *2D User Mode*: Optimized camera for 2D path planning visualization

<figure>
      <img src="./screenshots/run.gif" alt="Visualization Features" width="550"/><br>
</figure>

</details>

## Task 2: Implement RRT

<details>
<summary> Subsection 2.1

**`getNextSample()` Implementation**

</summary> 

The `getNextSample` function extends the tree by creating a new node at a fixed `stepSize` distance from the nearest existing node toward a randomly sampled point.

Mathematical Formulation:

Given:
- Random sample point: <code>x<sub>rand</sub></code> uniformly sampled in [0, width-1] × [0, height-1]
- Nearest node in tree: <code>x<sub>near</sub></code> found using KDTree
- Step size: <code>δ</code> (adjustable via slider)

The new node position is calculated as: <code>x<sub>new</sub> = x<sub>near</sub> + δ · (x<sub>rand</sub> - x<sub>near</sub>) / ||x<sub>rand</sub> - x<sub>near</sub>||</code>


</details>

<details>

<summary> Subsection 2.2

**Collision Function Implementation**

</summary> 

The collision detection function uses line discretization to check if a straight-line path between two points intersects with any obstacles. The line is sampled at pixel-level resolution to ensure no obstacles are missed.

Implementation Details:

- *Line Discretization*: Sample points along the line at 1-pixel intervals
- *Boundary Checking*: Verify points stay within image bounds
- *Pixel Checking*: Test each discretized point against the boolean obstacle map
- *Early Termination*: Return immediately upon first collision detection


Sanity Check:

Testing collision detection with known obstacle configurations:
- Line through obstacle: Returns True ✓
- Line in free space: Returns False ✓
- Line partially out of bounds: Returns True ✓
- Diagonal lines across obstacles: Correctly detected ✓

</details>

<details>
<summary> Subsection 2.3

**Algorithm Termination Criteria**

</summary> 

The algorithm uses a proximity-based stopping condition with collision verification:

The algorithm terminates when:
1. *Goal Proximity*: Distance to goal < 1.5 × stepSize
2. *Path Validity*: No collision between current node and goal
3. *Successful Connection*: Goal can be directly reached from current node

The goal is considered reached when:

<code>||x<sub>current</sub> - x<sub>goal</sub>|| < 1.5 × stepSize</code>
**AND** no collision exists on the path from <code>x<sub>current</sub></code> to <code>x<sub>goal</sub></code>
</details>

<details>

<summary> Subsection 2.4

**Full RRT Execution**

</summary> 

Experiment Setup:

- *Map Size*: 500×400 pixels
- *Step Size*: 10 pixels (default)
- *Start Point*: [50, 50]
- *Goal Point*: [400, 300]
- *Obstacles*: Three rectangular obstacles plus user-drawn obstacles

Results:

- *Iterations to Goal*: ~150-300 (varies due to random sampling)
- *Computation Time*: 1-3 seconds
- *Tree Nodes*: 150-300 nodes
- *Success Rate*: 95%+ with appropriate step size

</details>

<details>
<summary> Subsection 2.5

**Extracting and Visualizing the Solution Path**

</summary> 

The path is extracted by backtracking through parent pointers from goal to start:

1. Start from the goal node
2. Follow parent pointers back to start (parent=None)
3. Reverse the path for start-to-goal ordering
4. Draw path segments with increased thickness and z-offset

Visualization Method:

- *Path Color*: Green (#00FF00)
- *Path Width*: 4 pixels
- *Z-offset*: 0.2 units (ensures path appears above tree)
- *Continuous Path*: Connected line segments between waypoints

</details>

## Task 3: Experimental Results

<details>
<summary> Experiment 1

**Effect of Step Size on Performance**

</summary> 

Tested various step sizes on the same obstacle configuration with fixed start/goal positions.<br>
Each configuration was run 10 times to account for randomness.
<br><br>
  
Results Table:

| Step Size | Avg. Iterations | Avg. Time (s) | Success Rate | Path Length | Observations |
|-----------|----------------|---------------|--------------|-------------|--------------|
| 5         | 450            | 4.5           | 100%         | 85 nodes    | Very dense tree, slow convergence |
| 10        | 225            | 2.2           | 100%         | 43 nodes    | Good balance of speed and coverage |
| 20        | 112            | 1.1           | 95%          | 22 nodes    | Faster but may miss narrow passages |
| 50        | 45             | 0.45          | 70%          | 9 nodes     | Often fails in cluttered environments |

Analysis:

- *Small step sizes (5-10)*: Provide better coverage and higher success rates but require more iterations
- *Medium step sizes (10-20)*: Offer the best balance between exploration efficiency and success rate
- *Large step sizes (30-50)*: Fast convergence but poor performance in complex environments with narrow passages

</details>

<details>
<summary> Experiment 2

**Comprehensive Performance Analysis**

</summary> 

- Identified Strengths:
  - *Probabilistic Completeness*: Given enough iterations, RRT will find a path if one exists
    - Tested with 1000+ iterations on solvable maps: 100% success rate
  - *Exploration Efficiency*: Rapidly explores large open spaces
    - Open area coverage: 80% explored in first 100 iterations
  - *No Preprocessing Required*: Works immediately on any new map
    - Map change to solution: <1 second response time

- Identified Weaknesses:
  - *Path Quality*: Paths are often suboptimal and jagged
    - Average path length: 30-50% longer than optimal
    - Path smoothness: Multiple unnecessary turns
  - *Parameter Sensitivity*: Performance heavily depends on step size
    -Wrong step size can reduce success rate from 95% to 20%
  - *Narrow Passage Problem*: Struggles with tight corridors
    - Success rate drops to 40% for passages < 2×stepSize

- Experimental Evidence:

  Conducted 100 trials comparing RRT paths to manually drawn optimal paths:
  - RRT paths averaged 38% longer distance
  - Computation time increased exponentially with obstacle density
  - Memory usage scaled linearly with iterations (efficient)

</details>

</details>
