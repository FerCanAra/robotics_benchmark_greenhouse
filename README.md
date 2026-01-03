[![mvsim](https://circleci.com/gh/MRPT/mvsim.svg?style=svg)](https://circleci.com/gh/MRPT/mvsim) [![Documentation Status](https://readthedocs.org/projects/mvsimulator/badge/?version=latest)](https://mvsimulator.readthedocs.io/en/latest/?badge=latest)
[![CI Linux](https://github.com/MRPT/mvsim/actions/workflows/build-linux.yml/badge.svg)](https://github.com/MRPT/mvsim/actions/workflows/build-linux.yml)
[![CI Check clang-format](https://github.com/MRPT/mvsim/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MRPT/mvsim/actions/workflows/check-clang-format.yml)

A Benchmark for Evaluating Advanced Control Strategies in Mobile Robots for Mediterranean Greenhouse
======================================
This repository provides a standardized, reproducible benchmark for mobile robotics in greenhouse environments, designed to evaluate control laws and navigation algorithms under realistic operating conditions. The framework combines a physics-based simulation with a high-fidelity greenhouse model and a hierarchical control architecture spanning low-, mid-, and high-level control modules. Representative disturbance scenarios and quantitative performance metrics enable fair, repeatable comparisons of different control strategies, while a plugin-based design enables easy integration of custom controllers and planners. It used [MultiVehicle simulator (MVSim)](https://github.com/MRPT/mvsim), a lightweight, realistic dynamical simulator for 2D ("2.5D") vehicles and robots. It is tailored to the analysis of vehicle dynamics, wheel-ground contact forces, and accurate simulation of typical robot sensors (e.g., 2D and 3D lidars). As a package manager, it is used [Navigation2 (Nav2) ](https://github.com/ros-navigation/navigation2?tab=readme-ov-file), a professionally-supported successor of the ROS Navigation Stack deploying the same kinds of technology powering Autonomous Vehicles brought down, optimized, and reworked for mobile and surface robotics. 

<img width="400" height="300" alt="Mvsim_greenhouse" src="https://github.com/user-attachments/assets/d35b3e2f-8a23-4ca4-bbf0-4aa99508d0f5" /> <img width="420" height="320" alt="Vista_interna" src="https://github.com/user-attachments/assets/8d4e95b9-e20c-42a8-9b1c-aface9250e65" />

The simulator has been tested on ROS2 Humble, Ubuntu 22.04.

Paper citation
------------------

Soon!

License
--------------------
This project is distributed under the **BSD 3-Clause License**.

Copyright © 2026, Individual contributors  
Project owner: Fernando Cañadas Aránega <fernando.ca@ual.es> (University of Almeria) and collaborators
Copyright © 2012–2020, University of Almeria  

See the [LICENSE](LICENSE) file for full license text.

Prerequisites
--------------------
In order to use the simulator, you must have the following packages installed:

1. [Robot Operating System (ROS 2) Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). It is recommended to install the full desktop.
2. [The Mobile Robot Programming Toolkit (MRPT)](https://docs.mrpt.org/reference/latest/download-mrpt.html). It is easily installed with the following command:
```
sudo apt install libmrpt-dev mrpt-apps
```
3. [Navigation 2 (Nav2)](https://docs.nav2.org/getting_started/index.html). It is easily installed with the following command:
```
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2-bringup
```

Install and build
--------------------
In order to run the simulator, the official project repository must be installed. This can be done easily using the following commands.

```
mkdir -p ~/robotics_benchmark_greenhouse/src
cd ~/robotics_benchmark_greenhouse/src
git clone --recurse-submodules https://github.com/FerCanAra/robotics_benchmark_greenhouse.git
cd robotics_benchmark_greenhouse
git submodule update --init --recursive
cd ../../..
colcon build --packages-select mvsim --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Usage
--------------------
The proposed simulator is structured according to a hierarchical control architecture composed of three distinct levels: low-level, mid-level, and high-level control. 

#### Control layers

1. **Low level**: The low-level control layer focuses on the internal control of the robot actuators, specifically the motor dynamics. This level is implemented directly within the MVSim simulation environment, where the robot’s physical parameters and control gains can be configured through dedicated [parameter files](benchmark_bringup/benchmark_bringup/control_params/c1_pid_params.yaml). This layer is responsible for translating velocity commands into motor torques while accounting for realistic vehicle dynamics, including wheel–ground interactions. The low-level control scheme adopted in the benchmark is illustrated in the corresponding block diagram provided in the accompanying article.

<img width="600" height="300" alt="Esquema_1" src="https://github.com/user-attachments/assets/6b02abaa-0042-4508-87c7-4acbd2ca9cb4" />

2. **Mid-level**: The mid-level control layer addresses trajectory tracking, enabling the robot to follow a sequence of waypoints generated by the high-level planner. For this purpose, the benchmark integrates the [Timed Elastic Band (TEB-MPC)](https://github.com/FerCanAra/teb_local_planner_benchmark) local planner, managed through the Nav2 framework. The TEB-MPC module operates as a plugin-based component, allowing its [parameters](benchmark_bringup/benchmark_bringup/control_params/c2_mpc_teb_params.yaml) to be fully replaced or tuned by the user. This design supports the evaluation of different trajectory-following strategies under identical environmental and disturbance conditions. The mid-level control architecture is depicted in the corresponding control diagram.

<img width="600" height="200" alt="Esquema_2_2" src="https://github.com/user-attachments/assets/7dce2b77-8f6b-4e26-a656-6db3897ab9a1" />

3. **High level**: The high-level control layer focuses on global path planning, computing efficient trajectories within the environment while accounting for obstacles and workspace constraints. In this benchmark, the [Lazzy Theta star](https://idm-lab.org/bib/abstracts/papers/aaai10b.pdf) global planner is employed and integrated via the Nav2 framework. All [planning parameters](benchmark_bringup/benchmark_bringup/control_params/c3_theta_start_params.yaml) can be modified, enabling users to analyze the impact of high-level planning decisions on overall navigation performance. The complete high-level control structure is presented in the associated schematic.

<img width="600" height="200" alt="Esquema_3" src="https://github.com/user-attachments/assets/cde26dc5-5c5b-4aae-931d-901f31bc98b5" />


### Benchmark Categories

To support users with different expertise levels and research objectives, the benchmark defines three evaluation categories:

- **Category 1**: Intended for users focusing on classical control techniques, such as Proportional–Integral–Derivative (PID) control and basic advanced controllers. This category is particularly suitable for undergraduate-level education.

- **Category 2**: Designed for users working with model-based predictive control techniques, incorporating fundamental robotic modeling concepts. This category is well suited for master-level studies.

- **Category 3**: Targeted at advanced users developing high-level control and planning strategies, including sophisticated motion planners and decision-making algorithms. This category is appropriate for doctoral-level research.

### Benchmark Categories

To evaluate controller robustness under realistic operating conditions, the benchmark includes three types of disturbances, which can be activated independently or in combination:

- **Payload Sensitivity**: Mobile robots in greenhouse environments frequently transport variable payloads, which significantly affect motor behavior and dynamics. The benchmark allows users to emulate additional payloads up to 70 kg, applied as a static disturbance at the start of the simulation.

- **Terrain Slope**: Variable terrain slopes, typical of Mediterranean greenhouses, are simulated within a range of −3° to +3°. When enabled, a slope pattern is applied to the environment, directly influencing the robot’s dynamic response during navigation tasks.

- **Terrain Change**: Greenhouse environments often contain heterogeneous soil types. This disturbance emulates changes in terrain properties that directly affect the physical friction model. When activated, three distinct soil types are defined, each characterized by specific values of: friction coefficient (μ), rolling resistance coefficient (Crr), and damping factor (D).

### Running

All these variables can be launched with the following command:

```
ros2 launch benchmark_bringup launch_benchmark_bringup.launch.py \
  category:=1 \
  payload:=0 \
  terrain_slope:=false \
  change_terrain:=false
```
where:

- ```category:=1 ``` selects the evaluation category (1, 2, or 3),

- ```payload``` specifies the additional load,

- ```terrain_slope:=false ``` enables or disables terrain inclination,

- ```change_terrain:=false ``` change_terrain enables or disables terrain heterogeneity.

Example:

[Benchmark category 3 demo](https://youtu.be/-QOWn2Ga2w0)

For a complete description of the simulator architecture, control schemes, evaluation metrics, and experimental methodology, the reader is referred to the associated journal/conference article, which provides a detailed and rigorous explanation of the benchmark framework.


Performance Analysis and Data Logging
--------------------
During simulation execution, real-time performance plots are displayed, allowing the user to continuously assess the behavior of the implemented control law. These plots provide immediate qualitative insight into the system response and controller performance under the selected operating conditions. In addition, once the simulation finishes, the benchmark automatically generates a CSV data file containing the relevant performance variables. This file is saved in the directory from which the simulation was launched and includes the data required for offline analysis, post-processing, and quantitative evaluation (```$pwd/result/cetagory_X/yyyy_mm_dd_hh_mm_ss.csv```).

Build matrix status
--------------------

| Distro | Build dev | Build releases | Stable version |
| ---    | ---       | ---            | ---         |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mvsim__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mvsim__ubuntu_jammy_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mvsim__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mvsim__ubuntu_jammy_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mvsim__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mvsim__ubuntu_jammy_arm64__binary/) | [![Version](https://img.shields.io/ros/v/humble/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 2 Jazzy @ u24.04 | [![Build Status](https://build.ros2.org/job/Jdev__mvsim__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mvsim__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Jbin_uN64__mvsim__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mvsim__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/jazzy/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 2 Kilted @ u24.04 | [![Build Status](https://build.ros2.org/job/Kdev__mvsim__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mvsim__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Kbin_uN64__mvsim__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mvsim__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/kilted/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mvsim__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mvsim__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Rbin_uN64__mvsim__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mvsim__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Rbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
