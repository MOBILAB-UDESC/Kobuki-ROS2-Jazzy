# LQR and LMIs-based controllers for path tracking of a differential drive mobile robot.

This repository provides the configuration and implementation of three control strategies — **Linear Quadratic Regulator (LQR)**, **D-Stability (D-S)**, and **Guaranteed Cost (G-C)** — for path tracking of a differential drive mobile robot in simulation or real environments.

## 1 :scroll: Content
- [2. Controllers](#2-controllers)
  - [2.1. Linear Quadratic Regulator](#21-linear-quadratic-regulator)
  - [2.2. D-Stability](#22-d-stability)
  - [2.3. Guaranteed Cost](#23-guaranteed-cost)
- [3. Parameters](#3-parameters)
  - [3.1. tracking-mode](#31-tracking-mode)
  - [3.2. v_max, w_max](#32-v_max-w_max)
  - [3.3. type-test](#33-type-test)
  - [3.4. dt](#34-dt)
  - [3.5. type-traj](#35-type-traj)
  - [3.6. eta-traj](#36-eta-traj)
  - [3.7. cycl-traj](#37-cycl-traj)
  - [3.8. cent-traj](#38-cent-traj)
  - [3.9. route](#39-route)
  - [3.10. rho-traj](#310-rho-traj)
- [4. Test](#4-test)

## 2. Controllers

### 2.1 LQR
-`Q` and `R`: Weighting matrices for the state and control input, respectively.

### 2.2 D-S
-`type`: Region where poles must lie. There are 3 regions (`cone`, `plane` and `disc`) and the combination between them with a `-` (e.g. `cone-plane`).

### 2.2 G-C
-`Cz` and `Dz`: Weighting matrices

More information about this matrices can be found in this paper: "[Trajectory Tracking for a Differential Drive Dual-Wheeled Robot Using LMI-Based Controllers]()"

## 3. Parameters
All controller configurations are defined in [controllers_param.yaml](https://github.com/MOBILAB-UDESC/Kobuki-ROS2-Jazzy/blob/main/src/kobuki_ros/kobuki_controllers/config/controllers_param.yaml).

### 3.1. tracking-mode
**Type**: `string`
- `path_tracking`: Track a global reference matching the time.
- `path_following`: (not implemented yet) Follow the reference without matching the time.

### 3.2. v_max, w_max
**Type**: `float` (m/s, rad/s)\
Maximum linear and angular velocities of the robot.

### 3.3. type-test

### 3.4. dt
**Type**: `float` (s)\
Sampling time

### 3.5. type-traj
**Type**: `string`\
Trajectory shape: `infinite` and `circle`

### 3.6. eta-traj
**Type**: `float`\
Amplitude of the trajectory

### 3.7. cycl-traj
**Type**: `int`\
Number of trajectory cycles to perform.

### 3.8. cent-traj
**Type**: `list[float]`  
Center of the trajectory in world coordinates `[x, y, theta]`.

### 3.9. route
**Type**: `string`\
Path where results will be saved

### 3.10. rho-traj
**Type**: `float`\
Scalar to set the number of points of the trajectory without changing the sampling time: `n_points = (2 * π * rho * n_cycles) / dt`

## Test
Use the following commands to run the controllers:
```bash
ros2 run kobuki_controllers LQRDelay
ros2 run kobuki_controllers DStabDelay
ros2 run kobuki_controllers GuarCostDelay
```

Note: If necessary, update the odom and cmd_vel topic names in [LQR_delay](https://github.com/MOBILAB-UDESC/Kobuki-ROS2-Jazzy/blob/main/src/kobuki_ros/kobuki_controllers/kobuki_controllers/LQR_delay.py), [D_Stab_delay](https://github.com/MOBILAB-UDESC/Kobuki-ROS2-Jazzy/blob/main/src/kobuki_ros/kobuki_controllers/kobuki_controllers/D_Stab_delay.py) and [Guar_Cost_delay](https://github.com/MOBILAB-UDESC/Kobuki-ROS2-Jazzy/blob/main/src/kobuki_ros/kobuki_controllers/kobuki_controllers/Guar_Cost_delay.py).