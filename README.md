# Agricultural Robot Simulation — Tree Nursery Inspection

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
![MATLAB](https://img.shields.io/badge/MATLAB-R2020b%2B-blue)
![Status](https://img.shields.io/badge/status-simulation-green)

> **Course:** EBS 289K — Sensors & Actuators in Agricultural Automation (UC Davis)  
> **Instructor:** Prof. Stavros Vougioukas  
> **Authors:** Fernando Ferreira Lima dos Santos, Achala Rao  
> **Term:** Spring 2021

## Introduction
This project consisted of simulating an agricultural robot inspecting a tree nursery.  
The robot was equipped with a **GPS**, an **odometer**, and a **2D LiDAR**, and its objectives were:
- Traverse the orchard block autonomously.  
- Detect and localize trees.  
- Estimate the diameter of each tree trunk.  
- Output a text file with all tree data.  

The main tasks included implementing the robot’s **Guidance, Navigation, and Control (GNC)** system, developing an **Extended Kalman Filter (EKF)** for state estimation, and using **image processing techniques** for tree detection and diameter estimation.

---

## Task 1 – Estimate the Odometry and GPS/Compass Covariance Matrices
The **odometry (2×2)** and **GPS-compass (3×3)** covariance matrices were estimated using the script `Covariance_matrix.m`.  

The simulation ran several short movements with known true positions. The robot was commanded to move straight toward random targets, and the difference between odometry-estimated and true positions was recorded. The variance of these errors defined the σ values for the covariance matrices.


---

## Task 2 – Implement an Extended Kalman Filter (EKF)
The `EKF.m` script fused **odometry**, **GPS**, and **compass** measurements.  
Since the GPS operated at 1 Hz and simulation time step was 10 ms, GPS readings were available every 100 iterations. The EKF used these readings as corrections to the predicted state from odometry.

Additionally, a **median filter** was applied to LiDAR range data to remove spikes before integration.

> **Figure 1.** Robot Final path vs. True path.  
> ![EKF Trajectory](https://github.com/FFLSantos/Agricultural-Robot-Simulation-for-Tree-Nursery-Inspection/raw/main/Picture1.jpg)


---

## Task 3 – Traverse the Orchard Block
The robot traversed a simulated orchard using only onboard sensors (no access to the ground-truth map).  
To replicate real-world conditions such as tall grass or twigs, **random noise** was added to the bitmap.  
This produced scattered, isolated pixels in the LiDAR output.

**Figure 2.** Noisy orchard bitmap simulating field conditions  
![Noisy Bitmap](https://github.com/FFLSantos/Agricultural-Robot-Simulation-for-Tree-Nursery-Inspection/raw/main/Picture2.jpg)

---

**Figure 3.** Estimated map of the perceived environment  
![Estimated Map](https://github.com/FFLSantos/Agricultural-Robot-Simulation-for-Tree-Nursery-Inspection/raw/main/Picture3.jpg)

---

**Figure 4.** Filtered map  
![Filtered Map](https://github.com/FFLSantos/Agricultural-Robot-Simulation-for-Tree-Nursery-Inspection/raw/main/Picture4.jpg)


---

## Task 4 – Detect and Localize Trees; Estimate Their Diameters
Tree detection used the script `treeDetector.m`, which performed:
1. **Cropping** to remove border noise.  
2. **Binary conversion** of the LiDAR probability grid.  
3. **Median filtering** to remove isolated pixels.  
4. **Circle detection** to identify trunk shapes and diameters.  
5. **Coordinate correction** and **labeling** of detected trees.

Results were exported to a `.txt` file listing all detected trees.

**Figure 5.** Tree position error  
![Tree Position Error](https://github.com/FFLSantos/Agricultural-Robot-Simulation-for-Tree-Nursery-Inspection/raw/main/Picture5.jpg)

**Figure 6.** Tree diameter error  
![Tree Diameter Error](https://github.com/FFLSantos/Agricultural-Robot-Simulation-for-Tree-Nursery-Inspection/raw/main/Picture6.jpg)


## Task 5 – Write Output File
The output was a text file named `report.txt` with the following format:
