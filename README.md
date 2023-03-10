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
 
 <img src="https://lh3.googleusercontent.com/wUr4ZFPJf3yTUI6Y4uAdjKTxVwDv2EQOOdvIyUWxmd5kxs5Y6qzv0VpXAalI6E3w40GJsP7_2pWdZA9gfayQxTilZlxF_UQGJIYUF3nNPNZuTzJLrcRpONlt3IJBSHNOIx3AMEjgpwiafiayQzcHJWBdNhBhPt-50_axJXFAj-EfcFJjD35i4_7rVnJLAZQfvR94BYz5-hXDQzSJnHxHAgqifGVlHH8OqRCPlVdX9J7X3hdnU48h9VykK-EcOMvIrLINjLyHgMBGAGZqDXa59SEnCDIgGsAyGxQo6rDmFnMiEZ80BNIcXWUhqfuO7zAH4mOcvzKJhMa7_0UhsZMS9XgktjblFhihdmLc7__Ke5biBicIGr3BbNfLiQQqzhSprvy-7OYEUqvlX2nO84H8KZJipqjNO-tJguBq-c6Lxh_Dt7KhSYs2yF4w552deEuI69eH9PqIcDmNwkDdbChyEnyEA9ChH-IfU0PUfZRAabeQR-MD4IAp8jnFo74-D2BIcaa9rVkZ9ncJYzbxgCHXxtwtiF30JoyC7zZCG4XodhgyRVhJnZAM-VjWbt3RYyelBCY5bWdNDzXcTJgit2LProZK8qN7Oc6hoirC5msBMw2KC6lLhsvVHhGEA691H9T-CtVpoUJIhxPNnDtVj02jw1CpALJcGTaIVLDEto4NMY5G0Id7IcSejQpa2rqdDGQ9OCNeJgsdVdhGdEYCO3wMLPAoM7zwPmRxZIVvFf0fMgNF1XtGLD35BZQDJUQAfWnb334ZAP0kDod5iUvLmz4OoDfbAtdkadaH8tKX0xZQnZAq1MCzf31XF0hBWLAAmhSD5tVsBK8h24ORoa9GS2ufFU4GfAnfMiorGNEGpDkUdvhfpERv9bnG01ZlFbBWAFiB5-udx3MNDcplno31OPC_h4p7LNfRI1j2MjsLOjETdPd5=w828-h776-no?authuser=0" width="320"/> <img src="https://lh3.googleusercontent.com/nhNtbevh05r-6sZw3Fs2IBLdw_wYuE45bsV1wZ_-uHM2v0_j1giN934QH9X0x6A_-T67AMHrNGLDYHDo6VVzrDm7uE4qKOtd4IyCLRyQI5swqLZUR8lux0RVD5fYqlfNu9MDPm1BhD3wLD3FW9_Xe6b_BbQIGBFp0lACmieegwJ3zfYaq7C0DtPYuyPXcNxdYTJrXwfbZTFjkGtWKvKJsrcCX0qZmzd8Rr8IPFSpTAwHvD_49b26es52WdGlxcrrRmD6Hy884h7eQQ-JLq75TnJiZZfW30pSn5cOCzl1N_bUobSGkx2koClS3QIHmtWtf3Z8LUqZibyEbK469bpsqX5xo7wzM81JXjcqc9-hg1u52ZPFR5LANinNWQDM9K4o-C5Qs7-CujFpndicgeozhw2NTcfre8mylDaSrGRp4i8e4HArWnTlxFraVgK5tDA4j4ww0Yi6cOAE5Wa2xo1SK8h4GmTYaRRGK4GL2HCgdu5PgBGPWZWH9b-wGHlMoYU40ynKz5htS0LUDBNE3bZ94_vJzJPS3mmFk2X3kUFhYP3VyaEdn1UsWUB5EDqx6KXTWFN053jJVyw2NqDEoYW5cMQS_uWk1-UvpzQOtxLxh2DkefAsq8WNFN4mifZEK8cNIQq6AjbihkFpQw-6fgtg4oE_3OdvGDI6wnhQw-e5D-R5nwtkr91CBMMh_9CziuayTsNlhJg_-w14GGWkz8yNhDvtleBnkpy1P_dkI0VwtDmR-qTsJGgKicznjsfrEgNwpyk3PU9pu4J7Sa-XNydqjAEn_PyjPBW6oaDG9LyQO02gl4OtveQptTzpWHzDne8sXXpcBBoyW3Fjm79iArTc0vD5CGsck3JiomY0-pWV3WEzD5fQW5uvM4H58Pky9-2r3BHd6KZwElDHUUOHI8zKJMWfX7YfRTbrYPsfhrgH2q8h=w972-h738-no?authuser=0" width="395"/>
  
path generation (left) and path smoothing (right)
 
## License
Our code is licensed under [Apache License 2.0](https://github.com/SS47816/fiss_planner/blob/main/LICENSE) 

