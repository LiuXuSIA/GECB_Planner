# GECB planner
## [IROS2023] Gradient-Enhanced Path Generation and Quadratic Bezier-Based Path Smoothing for Unstructured Environments

This is an implementation with a easy-run demo of the navigation pipeline proposed in IROS 2023. Developed for unstructured environments, this navigation pipeline involves the terrain gradient modeling, path generation, and post-smoothing. The terrain gradients are derived from continuous terrain representation using Gaussian process.  By using the high-entropy terrain gradients, we developed a grid search based path generation technique which omits the collision-checking process but can still achieve collision-free paths. These piece-wise linear paths are then optimized using the proposed quadratic Bezier curve based smoothing technique, which can generate G2-continuous trajectory almost everywhere and can control the local maximum curvature conveniently.
## Dependencies
Our implementation is based on the following modules:
* numpy $\rightarrow$ 1.12.6
* scipy  $\rightarrow$  1.1.0
* matplotlib  $\rightarrow$ 3.5.3
* open3d   $\rightarrow$ 0.12.0

*The versions are the ones the authors used, the authors didn't test other versions.*
## Usage
Ensure that the required modules are installed. 
Clone or download the repository to run.

* dataSets $\rightarrow$  Include the *quarry* terrain data, which is very sparse.
* src
     * gaussian_process_map.py $\rightarrow$ Generate continuous terrain elevation maps and gradient maps.
     * path_generation.py $\rightarrow$ Generate piece-wise linear paths.
     * path_smoothing.py $\rightarrow$  Generate G2-continuous trajectories.
  
These three modules can be executed individually. 
 **The uasge of path smoothing:**

1) run the code

2) click ***left mouse*** button to determine the control points

3) press ***space key*** to generate curve

4) click ***right mouse button*** to tweak the control points

5) press ***escape key*** to clear the canvas

6) rerun 2->5
 
 They also be incorporated to the 
 * gecb_planner.py
 
 which can generate G2-continuous trajectories from sparse terrain data.
 ## Demo
 <img src="https://lh3.googleusercontent.com/wjO2O6AC3KiT8Tvf1tV632XxUKghCvYSaN67EaIBuutHq9d2D4z2w0MC5YVU3nS2DuDfma4qc7MCUJu0sOKSGgX5GyINN__L-LHEaa2EPtoEPRwMyvZZpjYvuTMuwmbM3dDvpT8VTnYqBZwDRkVeSJu_r0WH0SFSeENxqxqXUntxVX2FAmYSxnB4OtnP9inXfkSYQrNYL_fORaQeXO1SdpBBOAA8_Iqr-Hk-CigIpmetGdVqMzDJYFHfHox9ehpHbZt9bG_9SZ3obZNsf55-6eJeRW_2I6e5Af3dlhDQpvEKGUcu5e1ZPTBNL0bBhbMk8pZ7hs4PIgFfrj4kpQXktkyDN_X2hkKPPy02rLcC2QNffTDAq7Qs12n_j0UTW8w6FGLceeN9PsSUVI__23_HauS67BseWVi1psd5adCTSRklVQQMU1iDN8FSdXFXiwdH9u6Qtfl13u09chQxMWbpwM0mkKkFmBFjMg57pDdpkMs3zXAzXG0YuROcO9TlEUTyA3oOCiMCgW5nD604Ohw7YiWhhlh_Gdamww-nrno0nqnFx81uP-AGEIlNoFiD96fHKtzJg61Hb883kf_DiLxTT1BJm3L6bUn-rtMJyyrM3wvZwXpcJIWk_Ybl3nIQnbB29pVxxFj6-d5iotFd4K1G5JE6Vl2u3fiCSt2gVHQBcpN2Excumb07B5j9jn038JegiqekXXXL097_saep6Qf3jVpETUetHFD9GNcy4QQmg0yCLNtA9AxkW9Qq_ABV4nx4ZWDkTEPUZVN01Tww8i6GSI5IV1r0kLpWrmfEredSSL_ZuTFWOYhcwTIzT-O8eUJ572U5EmVlTbsI_walYJadVrJzFO_vq7BrKuxHNBppqOSihpnluu_TmM0RETutJPvPnJ1xyebXInF5yTwOqe37O2PalgBckBJroFTH4U1bIE2z=w828-h776-no?authuser=0" width="320" />
 <img src="https://lh3.googleusercontent.com/uAaXFLEYdNEFJGdd0DhrdlkDV_hyhSlIiK9fLzdWtJWymkT_o0r7XorM_gbnGuP54H2HfHVV4Az7btDpTUAlea3ykjJrItcgBj2fezdcSPdhc5GrBSJjbRPHXSA5jg2xbvuU6mqK5jCfsR-mpMwSMAIhxoV5Y2NRT7oIHt9tD2RygqpO0_XyKQGPXcjVOcU4OwwPN6dkTWjOqOq6DqQHyYHgrb2aX3LzWzaxRWZPKWwz67kyJY3XLOg8okoXiN1LMT6JllJvwQY-9EAiYtsbSlqSb9oh4PYObNUJaXSt25XcHTirJLVRFMv-A3HVmBxckz5O1ugxSCyzC3Efd9aZFSbmhr780DeM9d-uBu-l5BxKxPxMGzWnmrG5tYpG9QB4n2XOCPZ7QQ9tLNO-gXiYz_qn3iAqLQG4Q0hTz8qPMhq8Hn1wlEc6VMpFgatv7VKSidRFEA8BR5Pv7AwXWz1B486vF7UYwE6jKSqQy1czwkfhHIhXOyIFGcP47kzXoVEMDy__2j0FxWQiwx2Ruf1J1mAUh3Pqew5DsQ0g9Ggin0Gq1_aeLlC5-B1xguSEVy_COtzNjwdL2r7pJpPH8T3dThmuftxLmC-SJMOlWTjO9LRSC4iPcXiNbht-l8B7CqCTRrjhEsiu3F4P1cWV9CU4_A07vjDP2ZqfNcZAhQio5WLdbOR6qrHxg0rbTK2nDjULdQAMCYRuDfkDQTJgQtPqeFAqiLMtMgDTJEdgVdV_JP8DiUZGEqzmZyoOVDlB2TAJAJtKfZgz7EuV04hXg8xXt4GMoGIStyfSAGRIEczIqRoJqeT5GjpwEEXPTbbuYxgSpah5FZaoPrBT7cQZvEwXn4dM9x0tn8ta4tyyqT8MmbP_AoKdi-51EhzN3rlo3KcyoB2Hn57a7gjz3Fd_1smrSyy9mVvd4t6nuw6X037oqUp7=w972-h738-no?authuser=0" width="395" />
  
path generation (left) and path smoothing (right)
 
## License
Our code is licensed under [Apache License 2.0](https://github.com/SS47816/fiss_planner/blob/main/LICENSE) 

