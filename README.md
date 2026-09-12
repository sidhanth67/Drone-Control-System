# Drone Control System

A modular ROS 2 package engineered for autonomous drone simulation in Gazebo, featuring computer vision-based ArUco marker tracking, custom control nodes, and precise spatial coordinate transformations.

## Table of Contents
- [Architecture Overview](#architecture-overview)
- [Features & Modules](#features--modules)
- [Coordinate System Transformations](#coordinate-system-transformations)
- [Package Structure](#package-structure)
- [Prerequisites & Dependencies](#prerequisites--dependencies)
- [Installation & Build](#installation--build)
- [Usage](#usage)

---

## Architecture Overview

This project bridges the gap between simulated physics environments and visual perception algorithms. By leveraging ROS 2 communication pipelines, the system captures real-time imagery from a simulated drone-mounted camera, processes visual targets via OpenCV, maps the extracted optical data into global engineering coordinates, and feeds actionable navigation commands back into the simulation.

---

## Features & Modules

### 1. ArUco Marker Tracker (`aruco_tracker_node.py`)
* **CvBridge Integration**: Safely decodes incoming binary `sensor_msgs/msg/Image` streams into native 3-channel OpenCV (`bgr8`) NumPy arrays without dropping data integrity.
* **Optimized Frame Processing**: Converts incoming RGB/BGR streams to grayscale to maximize edge-detection performance. It instantiates an explicit `aruco.ArucoDetector` using a `DICT_4X4_50` dictionary layout to prevent performance overhead on every tick.
* **3D Pose Estimation (`solvePnP`)**: Detects the 4 pixel corners of a physical marker (`marker_length = 0.20` meters) and computes exact translational vectors (`tvec`) and rotational vectors (`rvec`) utilizing the camera's intrinsic matrix.

### 2. Control Node (`controller_node.py`)
* Intercepts spatial tracking output to execute movement decisions and trajectory tracking for the vehicle.

### 3. Simulation Launcher (`sim_control.launch.py`)
* Orchestrates the spawning of the Gazebo world, loads custom robot descriptions, and spins up the perception and control nodes concurrently.

---

## Coordinate System Transformation

A critical component of this package is mapping between differing software conventions:
* **OpenCV Optical Frame**: $X$ points right, $Y$ points down, and $Z$ points forward (out of the lens).
* **Gazebo Engineering Frame**: $X$ points forward, $Y$ points left, and $Z$ points up.

The tracking node explicitly remaps these axes so controller targets align with native Gazebo ground-plane vectors:
```python
pos_msg.x = -float(tvec[1])  # Camera Down (Y) -> Gazebo Forward (X)
pos_msg.y = -float(tvec[0])  # Camera Right (X) -> Gazebo Left (Y)
pos_msg.z = float(tvec[2])   # Camera Depth (Z) -> Vertical Altitude (Z)
