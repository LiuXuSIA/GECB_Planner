# GECB planner
## Gradient-Enhanced Path Generation and Quadratic Bezier-Based Path Smoothing for Unstructured Environments
[![CodeFactor](https://www.codefactor.io/repository/github/liuxusia/gecb_planner/badge)](https://www.codefactor.io/repository/github/liuxusia/gecb_planner) ![](https://img.shields.io/github/last-commit/LiuXuSIA/GECB_Planner)  ![](https://img.shields.io/github/languages/top/LiuXuSIA/GECB_Planner)
![](https://img.shields.io/github/license/LiuXuSIA/GECB_Planner)

This is an implementation with an easy-run demo of the navigation pipeline named **GECB planner** ( Gradient-Enhanced and Quadratic Bezier-Based path planner). Developed for unstructured environments, this navigation pipeline involves the terrain gradient modeling, path generation, and post-smoothing. The terrain gradients are derived from continuous terrain representation using Gaussian process.  By using the high-entropy terrain gradients, we developed a grid search based path generation technique which omits the collision-checking process but can still achieve collision-free paths. These piece-wise linear paths are then optimized using the proposed quadratic Bezier curve based smoothing technique, which can generate G2-continuous trajectories almost everywhere and can control the local maximum curvatures conveniently.
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

2) click the ***left mouse button***  to determine the control points,
    and ensure there are at least four control points

3) press the ***space key*** to generate the curve

4) click the ***right mouse button*** to tweak the control points

5) press the ***escape key*** to clear the canvas

6) rerun 2->5
 
 They also be incorporated to the file
 * gecb_planner.py
 
 which can generate G2-continuous trajectories from the sparse terrain data.
 ## Demos
 These demos demonstrate the processes and results of the path generation and path smoothing.
 
![](README_md_files/86b710a0-c142-11ed-b917-ef054118fc98_20230313095820.jpeg?v=1&type=image&token=V1:EXevrYBk5otMH8IZ2_MovBqPCsZrx_ET1LzWDi8LTbI =320x)   ![](README_md_files/7a3f1ec0-c143-11ed-b917-ef054118fc98_20230313100509.jpeg?v=1&type=image&token=V1:hiSFuUBsPBUQXYyh-j56-fK9N497wWPjipvrf8X3riY =395x)
path generation (left) and path smoothing (right)
 
## License
Our code is licensed under [Apache License 2.0](https://github.com/SS47816/fiss_planner/blob/main/LICENSE) 

