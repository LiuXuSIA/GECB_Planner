# GECB planner
## [IROS2023] Gradient-Enhanced Path Generation and Quadratic Bezier-Based Path Smoothing for Unstructured Environments

This is an implementation with an easy-run demo of the navigation pipeline proposed in IROS 2023. Developed for unstructured environments, this navigation pipeline involves the terrain gradient modeling, path generation, and post-smoothing. The terrain gradients are derived from continuous terrain representation using Gaussian process.  By using the high-entropy terrain gradients, we developed a grid search based path generation technique which omits the collision-checking process but can still achieve collision-free paths. These piece-wise linear paths are then optimized using the proposed quadratic Bezier curve based smoothing technique, which can generate G2-continuous trajectories almost everywhere and can control the local maximum curvatures conveniently.
## Dependencies
Our implementation is based on the following modules:
* numpy $\rightarrow$ 1.12.6
* scipy  $\rightarrow$  1.1.0
* matplotlib  $\rightarrow$ 3.5.3
* open3d   $\rightarrow$ 0.12.0

*The versions were used by the authors, who didn't test other versions.*
## Usage
Ensure that the required modules are installed. 
Clone or download the repository to run.

* dataSets $\rightarrow$  Include the *quarry* terrain data, which is very sparse.
* src
     * gaussian_process_map.py $\rightarrow$ Generate continuous terrain elevation maps and gradient maps.
     * path_generation.py $\rightarrow$ Generate piece-wise linear paths.
     * path_smoothing.py $\rightarrow$  Generate G2-continuous trajectories.
  
These three modules can be executed individually. 

Particularly,  **the usage of path smoothing:**

1) run the code

2) click the ***left mouse button***  to determine the control points

3) press the ***space key*** to generate the curve

4) click the ***right mouse button*** to tweak the control points

5) press the ***escape key*** to clear the canvas

6) rerun 2->5
 
 They also be incorporated to the file
 * gecb_planner.py
 
 which can generate G2-continuous trajectories from the sparse terrain data.
 ## Demo
 These demos demonstrate the processes and results of the path generation and path smoothing.
 
 <img src="http://c.nxw.so/aAb4H" width="320"/>  <img src="http://c.nxw.so/bkNAa" width="395"/>
  
path generation (left) and path smoothing (right)
 
## License
Our code is licensed under [Apache License 2.0](https://github.com/SS47816/fiss_planner/blob/main/LICENSE) 

