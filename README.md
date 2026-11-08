A ROS 2 Benchmarking Framework for Hierarchical Control Strategies in Mobile Robots for Mediterranean Greenhouses 🌱 
======================================
[![arXiv](https://img.shields.io/badge/arXiv-preprint-b31b1b.svg)](https://arxiv.org/abs/2602.15162)
[![Last Updated](https://img.shields.io/badge/last%20updated-2026--09-blue)](.)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.22679881.svg)](https://doi.org/10.5281/zenodo.22679881)
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)

This repository provides a standardised, reproducible benchmark for mobile robotics in greenhouse environments, designed to evaluate control laws and navigation algorithms under representative operating conditions. The framework combines a physics-based simulation of a real Mediterranean greenhouse (Agroconnect, Almería, Spain) with a hierarchical control architecture spanning low-, mid- and high-level control. Parameterised disturbances (payload, terrain type and slope), normalised performance indices, task-level indicators and a statistical protocol enable fair, repeatable comparisons of control strategies, while a plugin-based design allows custom controllers and planners to be integrated without modifying the benchmark. It used [MultiVehicle simulator (MVSim)](https://github.com/MRPT/mvsim), a lightweight dynamical simulator for 2D ("2.5D") vehicles that models wheel–ground contact forces and typical robot sensors, and [Navigation2 (Nav2) ](https://github.com/ros-navigation/navigation2?tab=readme-ov-file) for the mid- and high-level layers. The complete benchmark runs in real time using less than two CPU cores, so it can be used on standard computers.

<img src="./docs/BenchmarkCat.gif" width="700" height="600">

<!-- 
<img width="400" height="300" alt="Mvsim_greenhouse" src="https://github.com/user-attachments/assets/d35b3e2f-8a23-4ca4-bbf0-4aa99508d0f5" /> <img width="400" height="300" alt="Vista_interna" src="https://github.com/user-attachments/assets/8d4e95b9-e20c-42a8-9b1c-aface9250e65" />-->

The simulator has been tested on ROS 2 Humble and Ubuntu 22.04 LTS. It can also be run on Windows and macOS by installing a [virtual machine](#virtual-machine).

> **Note:** Version v1.1.0 corresponds to the revised version of the associated article. Process tested on 8th November 2026.

📑 What's new in v1.1.0
------------------

- Alternative controllers and planners: besides the reference stack (PI + TEB + Lazy θ*), Category 2 can be run with MPPI and Regulated Pure Pursuit, and Category 3 with Smac Planner 2D and NavFn, selected with launch arguments. All the controllers share the same limits (0.5 m/s, 1.0 rad/s, 1.0 rad/s²).
- Common reference in Category 2: the four waypoints are joined into a fixed reference path that is sent directly to the controller (FollowPath), so every controller follows exactly the same reference.
- Redesigned metrics: time-averaged indices of normalised signals (SAE, SCI), relative indices with respect to the reference stack, and task-level indicators (success, time to goal, path length, clearance, contacts).
- Campaign tools: scripts to run complete campaigns with a real-time validity check and to compute the tables, heatmaps and statistical comparisons (tools/).
- Simulator fixes: odometry twist published by MVSim and lateral friction factor for skid-steer vehicles.
- Reference results: the 336 runs of the reference case are archived in [Zenodo](https://doi.org/10.5281/zenodo.22679881).

📑 Citation
------------------
*Paper citation*
------------------
```
@article{canadas2026ros2,
  title={A ROS2 Benchmarking Framework for Hierarchical Control Strategies in Mobile Robots for Mediterranean Greenhouses},
  author={Ca{\~n}adas-Ar{\'a}nega, Fernando and Ma{\~n}as-{\'A}lvarez, Francisco J. and Guzmán, Jos{\'e} L. Moreno, Jos{\'e} C. and Blanco-Claraco, Jos{\'e} L.},
  journal={arXiv preprint arXiv:2602.15162},
  year={2026}
}
```
*Software citation*
------------------
```
@software{agricultural_benchmark,
  author  = {Ca{\~n}adas-Ar{\'a}nega, Fernando},
  title   = {A Benchmark for Evaluating Advanced Control Strategies in Mobile Robots for Mediterranean Greenhouses},
  version = {1.1.0},
  year    = {2026},
  doi     = {10.5281/zenodo.22679881},
  url     = {https://github.com/FerCanAra/robotics_benchmark_greenhouse}
}
```
📜 License
--------------------
This project is distributed under the **BSD 3-Clause License**.

Copyright © 2026, Individual contributors  
Project owner: Fernando Cañadas Aránega <fernando.ca@ual.es> (University of Almeria) and collaborators

See the [LICENSE](LICENSE) file for full license text.

*International LICENCE: Intellectual property patent no. 2607246522198 held by [Safe Creative](https://www.safecreative.org/es)*


⚙️ Prerequisites
--------------------
In order to use the simulator, you must have the following packages installed:

1. [Robot Operating System (ROS 2) Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). It is recommended to install the full desktop.
2. [Navigation 2 (Nav2)](https://docs.nav2.org/getting_started/index.html). It is easily installed with the following command:
```
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2-bringup
```
3. Dependencies. The following dependencies are required::
```
sudo apt install git
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep2
```
4. [The Mobile Robot Programming Toolkit (MRPT)](https://docs.mrpt.org/reference/latest/download-mrpt.html). They will be installed in the rosdep commands.

🛠️ Install and build
--------------------
In order to run the benchmark, the official project repository must be installed. This can be done easily using the following commands.

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/FerCanAra/robotics_benchmark_greenhouse.git
cd robotics_benchmark_greenhouse
git submodule update --init --recursive
cd ../..
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```
To use exactly the version of the article, check out the release tag before building: git checkout v1.1.0 && git submodule update --recursive.

🚀 Usage
--------------------
The benchmark is structured according to a hierarchical control architecture with three levels. A new strategy can be evaluated at the level it targets while the other two levels remain fixed to the reference stack.

#### Control layers

1. **Low level (PI with anti-windup, Feedforward and Reference Filter)**: wheel-speed control of the motors, implemented inside the MVSim vehicle model [PID parameter](benchmark_bringup/benchmark_bringup/control_params/c1_pid_params.yaml) file. This layer is responsible for translating velocity commands into motor torques while accounting for realistic vehicle dynamics, including wheel–ground interactions. The low-level control scheme adopted in the benchmark is illustrated in the corresponding block diagram provided in the accompanying article.

<img width="800" height="300" alt="PID" src="https://github.com/user-attachments/assets/a970ae16-e4de-4d24-9faf-6c294e1ebfd0" />

2. **Mid-level (path following)**: Nav2 controller plugin. The reference controller is the [Timed Elastic Band (TEB-MPC)](https://github.com/FerCanAra/teb_local_planner_benchmark) local planner, managed through the Nav2 framework. The TEB-MPC module operates as a plugin-based component, allowing the user to fully replace or tune its [MPC parameters](benchmark_bringup/benchmark_bringup/control_params/c2_mpc_teb_params.yaml). This design supports the evaluation of different trajectory-following strategies under identical environmental and disturbance conditions. The mid-level control architecture is depicted in the corresponding control diagram.

<img width="800" height="300"  alt="MPC" src="https://github.com/user-attachments/assets/1d0c3d82-67cf-48c2-a0e4-5736f8eb0600" />

3. **High-level (path planning):**: Nav2 planner plugin. The reference planner is [Lazzy Theta star](https://idm-lab.org/bib/abstracts/papers/aaai10b.pdf) global planner is employed and integrated via the Nav2 framework. All [planning parameters](benchmark_bringup/benchmark_bringup/control_params/c3_theta_start_params.yaml) can be modified, enabling users to analyze the impact of high-level planning decisions on overall navigation performance. The complete high-level control structure is presented in the associated schematic.

<img width="800" height="300" alt="Planner" src="https://github.com/user-attachments/assets/1c7a3c83-0300-4611-8746-6d5256683a16" />

### Benchmark Categories

To support users with different expertise levels and research objectives, the benchmark defines three evaluation categories:

- **Category 1 (low level)**: a fixed sequence of velocity commands; evaluates wheel-speed tracking and control effort. Suitable for undergraduate courses on classical control.

- **Category 2 (low + mid level)**: the robot follows a fixed reference path through four waypoints, (10.2, 13.7), (15.0, 13.7), (18.0, 13.7) and (18.0, 17.4), along a corridor, the headland and a second corridor, without a global planner. Suitable for master-level studies on predictive control and path following.

- **Category 3 (low + mid + high level)**: autonomous navigation to a single goal, (18.0, 17.4) with a final orientation of 0.74 rad, using a global planner on a prior map. Suitable for doctoral research on planning and navigation.

In Categories 2 and 3, a run succeeds when the robot reaches the goal within 0.3 m before 180 s; otherwise it ends as failed (the controller aborts) or timeout.

### Disturbances

The three disturbances can be activated independently or combined (eight conditions):

- **Payload Sensitivity**: additional load from 0 to 70 kg, applied at the start of the simulation. It is not measured by any sensor; its value is declared by the user.

- **Terrain Slope**: a terrain profile with inclinations between approximately −4° (downhill) and +2.3° (uphill) along the reference path. Other profiles can be defined by modifying the terrain model of the world.

- **Terrain Change**: three soil sectors (gravel sand, concrete and compact sand), each with its own friction coefficient (μ), rolling resistance coefficient (Crr) and damping coefficient (C_D).
  

### Running

All these variables can be launched with the following command:

```
ros2 launch benchmark_bringup launch_benchmark_bringup.launch.py \
  category:=2 \
  controller:=teb \
  planner:=theta \
  payload:=0 \
  terrain_slope:=false \
  change_terrain:=false \
  seed:=1
```
where:

- ```category:=1 ``` selects the evaluation category (1, 2, or 3),

- ```controller:=teb ``` selects the mid-level controller (teb, mppi, rpp or dwb; Categories 2 and 3),

- ```planner```  selects the global planner (theta, smac or navfn; Category 3),

- ```payload``` specifies the additional load (from 0 to 70 kg),

- ```terrain_slope:=false ``` enables or disables terrain inclination (true or false),

- ```change_terrain:=false ``` change_terrain enables or disables terrain heterogeneity (true or false).

The launch file applies the common limits to whichever controller is selected, so that all of them are evaluated under the same conditions.

Example:

[Benchmark category 3 demo](https://youtu.be/-QOWn2Ga2w0)

For a complete description of the simulator architecture, control schemes, evaluation metrics, and experimental methodology, the reader is referred to the associated journal/conference article, which provides a detailed and rigorous explanation of the benchmark framework.


📊 Metrics and Data Logging
--------------------
During a run, real-time plots show the errors and control effort of each level. When the run finishes, a folder is created with the time series (CSV), the performance indices and the run metadata (seed, condition, configuration and final status) in ```$pwd/result/category_X/yyyy_mm_dd_hh_mm_ss.csv```). The indices are time averages of normalised signals, so they do not depend on the sampling frequency or the duration of the test:

- SAE (Scaled Absolute Error): mean normalised tracking error of the evaluated level (wheel velocity error in Category 1; distance to the common reference path in Category 2; distance to the first plan in Category 3).
- SCI (Scaled Control Input): mean normalised control input (motor torque or velocity command), bounded between 0 and 1.
J = (SAE + SCI)/2 for each level, and J<sup>rel</sup>, relative to the reference stack in the nominal condition (1 = reference, < 1 better).
- Task-level indicators (Categories 2 and 3): success, time to goal, path length (and planned length in Category 3), minimum distance to the obstacles and number of contacts.

Since a controller that slows down reduces its control effort, the indices should always be read together with the success rate and the time to goal. See the article for the complete definitions.

🔬 Reproducing the results
--------------------
The ```tools/``` directory contains the scripts used for the reference case:
```
cd ~/ros2_ws/src/robotics_benchmark_greenhouse/tools
python3 run_batch.py --out ~/results --categories 2 \
    --controllers teb mppi rpp --conditions all --seeds 6
python3 analyze_results.py ~/results
```
- **run_batch.py** runs a complete campaign without visualisation (any subset of categories, controllers, planners, conditions and seeds), checks the validity of every run (real-time factor within 1 ± 0.05 and no gap longer than 0.5 s in the recorded data), repeats invalid runs and resumes interrupted campaigns.
- **analyze_results.py** computes the indices, the tables (mean ± standard deviation and Mann–Whitney comparisons with the baseline), the heatmaps of the relative indices and the computational cost.

The reference case comprises 336 valid runs (8 conditions × 6 runs; PI in Category 1, three controllers in Category 2 and three planners in Category 3). The individual results of every run are available in Zenodo.

🧩 Evaluating your own control law
--------------------

- Low level: modify or replace the control function in ```VehicleDifferential_ControllerTwistPID.cpp``` (inputs: wheel velocity reference and measurement, estimated slope torque; output: motor torque).
- Mid level: write a configuration file for any existing Nav2 controller, or develop a new controller plugin [tutorial](https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_controller_plugin/writing_new_controller_plugin/), and add it to the list of controllers of the launch file.
- High level: configure any Nav2 planner or develop a new planner plugin [tutorial](https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_planner_plugin/writing_new_planner_plugin/), and add it to the list of planners of the launch file.

Then run the same campaign with run_batch.py and compare with the reference results.


🖥️ Virtual Machine <a name="virtual-machine"></a>
--------------------
A fully configured virtual machine is provided to ensure easy deployment and reproducibility across different operating systems (Windows and IOS). To set up this virtual machine, follow these [instructions](https://github.com/FerCanAra/IIR_ROS2_Tutorial/blob/main/docs/vmwaresetup.md). They first detail how to install VMware Pro 17 for free on Ubuntu 22.04, followed by configuring the ROS 2 Humble environment and installing the benchmark.


<div align="center">

# MultiVehicle Simulator (MVSim)

<img src="https://mrpt.github.io/imgs/mvsim_logo-bg-150px.png" alt="MVSim logo" width="150"/>

**Lightweight, realistic 2.5D dynamics simulator for mobile robots and multi-agent research**

[![CI Linux](https://github.com/MRPT/mvsim/actions/workflows/build-linux.yml/badge.svg)](https://github.com/MRPT/mvsim/actions/workflows/build-linux.yml) [![Check clang-format](https://github.com/MRPT/mvsim/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MRPT/mvsim/actions/workflows/check-clang-format.yml) [![Documentation](https://readthedocs.org/projects/mvsimulator/badge/?version=latest)](https://mvsimulator.readthedocs.io/en/latest/?badge=latest) [![License](https://img.shields.io/badge/license-BSD%203--Clause-blue.svg)](LICENSE)

[**Documentation**](https://mvsimulator.readthedocs.io) · [**Installation**](https://mvsimulator.readthedocs.io/en/latest/install.html) · [**Demo worlds**](https://mvsimulator.readthedocs.io/en/latest/demo_worlds.html) · [**Cite**](#citation)

</div>

<div align="center">
  
# Navigation2 (Nav2)

<img width="120" height="210" alt="image" src="https://github.com/user-attachments/assets/4d7ef1fc-2977-49c5-b2b5-43d74d16de81" />

</div>

| Package | humble Source | humble Debian | jazzy Source | jazzy Debian | kilted Source | kilted Debian |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| navigation2 | [![Build Status](https://build.ros2.org/job/Hsrc_uj__navigation2__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__navigation2__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__navigation2__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__navigation2__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__navigation2__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__navigation2__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__navigation2__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__navigation2__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__navigation2__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__navigation2__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__navigation2__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__navigation2__ubuntu_noble_amd64__binary/) |
| nav2_amcl | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_amcl__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_amcl__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_amcl__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_amcl__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_amcl__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_amcl__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_amcl__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_amcl__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_amcl__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_amcl__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_amcl__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_amcl__ubuntu_noble_amd64__binary/) |
| nav2_behavior_tree | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_behavior_tree__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_behavior_tree__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_behavior_tree__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_behavior_tree__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_behavior_tree__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_behavior_tree__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_behavior_tree__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_behavior_tree__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_behavior_tree__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_behavior_tree__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_behavior_tree__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_behavior_tree__ubuntu_noble_amd64__binary/) |
| nav2_behaviors | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_behaviors__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_behaviors__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_behaviors__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_behaviors__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_behaviors__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_behaviors__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_behaviors__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_behaviors__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_behaviors__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_behaviors__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_behaviors__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_behaviors__ubuntu_noble_amd64__binary/) |
| nav2_bringup | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_bringup__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_bringup__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_bringup__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_bringup__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_bringup__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_bringup__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_bringup__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_bringup__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_bringup__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_bringup__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_bringup__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_bringup__ubuntu_noble_amd64__binary/) |
| nav2_bt_navigator | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_bt_navigator__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_bt_navigator__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_bt_navigator__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_bt_navigator__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_bt_navigator__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_bt_navigator__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_bt_navigator__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_bt_navigator__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_bt_navigator__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_bt_navigator__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_bt_navigator__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_bt_navigator__ubuntu_noble_amd64__binary/) |
| nav2_collision_monitor | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_collision_monitor__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_collision_monitor__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_collision_monitor__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_collision_monitor__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_collision_monitor__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_collision_monitor__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_collision_monitor__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_collision_monitor__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_collision_monitor__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_collision_monitor__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_collision_monitor__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_collision_monitor__ubuntu_noble_amd64__binary/) |
| nav2_common | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_common__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_common__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_common__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_common__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_common__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_common__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_common__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_common__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_common__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_common__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_common__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_common__ubuntu_noble_amd64__binary/) |
| nav2_constrained_smoother | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_constrained_smoother__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_constrained_smoother__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_constrained_smoother__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_constrained_smoother__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_constrained_smoother__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_constrained_smoother__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_constrained_smoother__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_constrained_smoother__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_constrained_smoother__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_constrained_smoother__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_constrained_smoother__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_constrained_smoother__ubuntu_noble_amd64__binary/) |
| nav2_controller | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_controller__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_controller__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_controller__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_controller__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_controller__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_controller__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_controller__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_controller__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_controller__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_controller__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_controller__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_controller__ubuntu_noble_amd64__binary/) |
| nav2_core | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_core__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_core__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_core__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_core__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_core__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_core__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_core__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_core__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_core__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_core__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_core__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_core__ubuntu_noble_amd64__binary/) |
| nav2_costmap_2d | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_costmap_2d__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_costmap_2d__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_costmap_2d__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_costmap_2d__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_costmap_2d__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_costmap_2d__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_costmap_2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_costmap_2d__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_costmap_2d__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_costmap_2d__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_costmap_2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_costmap_2d__ubuntu_noble_amd64__binary/) |
| nav2_docking | [![Build Status](https://build.ros2.org/job/Hsrc_uj__opennav_docking__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__opennav_docking__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__opennav_docking__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__opennav_docking__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__opennav_docking__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__opennav_docking__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__opennav_docking__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__opennav_docking__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__opennav_docking__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__opennav_docking__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__opennav_docking__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__opennav_docking__ubuntu_noble_amd64__binary/) |
| nav2_dwb_controller | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_dwb_controller__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_dwb_controller__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_dwb_controller__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_dwb_controller__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_dwb_controller__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_dwb_controller__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_dwb_controller__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_dwb_controller__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_dwb_controller__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_dwb_controller__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_dwb_controller__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_dwb_controller__ubuntu_noble_amd64__binary/) |
| nav2_graceful_controller | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_graceful_controller__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_graceful_controller__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_graceful_controller__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_graceful_controller__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_graceful_controller__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_graceful_controller__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_graceful_controller__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_graceful_controller__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_graceful_controller__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_graceful_controller__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_graceful_controller__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_graceful_controller__ubuntu_noble_amd64__binary/) |
| nav2_lifecycle_manager | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_lifecycle_manager__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_lifecycle_manager__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_lifecycle_manager__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_lifecycle_manager__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_lifecycle_manager__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_lifecycle_manager__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_lifecycle_manager__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_lifecycle_manager__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_lifecycle_manager__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_lifecycle_manager__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_lifecycle_manager__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_lifecycle_manager__ubuntu_noble_amd64__binary/) |
| nav2_loopback_sim | N/A | N/A | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_loopback_sim__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_loopback_sim__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_loopback_sim__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_loopback_sim__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_loopback_sim__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_loopback_sim__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_loopback_sim__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_loopback_sim__ubuntu_noble_amd64__binary/) |
| nav2_map_server | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_map_server__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_map_server__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_map_server__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_map_server__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_map_server__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_map_server__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_map_server__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_map_server__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_map_server__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_map_server__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_map_server__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_map_server__ubuntu_noble_amd64__binary/) |
| nav2_mppi_controller | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_mppi_controller__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_mppi_controller__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_mppi_controller__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_mppi_controller__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_mppi_controller__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_mppi_controller__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_mppi_controller__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_mppi_controller__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_mppi_controller__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_mppi_controller__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_mppi_controller__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_mppi_controller__ubuntu_noble_amd64__binary/) |
| nav2_msgs | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_msgs__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_msgs__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_msgs__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_msgs__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_msgs__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_msgs__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_msgs__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_msgs__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_msgs__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_msgs__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_msgs__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_msgs__ubuntu_noble_amd64__binary/) |
| nav2_navfn_planner | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_navfn_planner__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_navfn_planner__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_navfn_planner__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_navfn_planner__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_navfn_planner__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_navfn_planner__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_navfn_planner__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_navfn_planner__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_navfn_planner__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_navfn_planner__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_navfn_planner__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_navfn_planner__ubuntu_noble_amd64__binary/) |
| nav2_planner | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_planner__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_planner__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_planner__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_planner__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_planner__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_planner__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_planner__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_planner__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_planner__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_planner__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_planner__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_planner__ubuntu_noble_amd64__binary/) |
| nav2_regulated_pure_pursuit | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_regulated_pure_pursuit_controller__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_regulated_pure_pursuit_controller__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_regulated_pure_pursuit_controller__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_regulated_pure_pursuit_controller__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_regulated_pure_pursuit_controller__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_regulated_pure_pursuit_controller__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_regulated_pure_pursuit_controller__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_regulated_pure_pursuit_controller__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_regulated_pure_pursuit_controller__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_regulated_pure_pursuit_controller__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_regulated_pure_pursuit_controller__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_regulated_pure_pursuit_controller__ubuntu_noble_amd64__binary/) |
| nav2_rotation_shim_controller | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_rotation_shim_controller__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_rotation_shim_controller__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_rotation_shim_controller__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_rotation_shim_controller__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_rotation_shim_controller__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_rotation_shim_controller__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_rotation_shim_controller__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_rotation_shim_controller__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_rotation_shim_controller__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_rotation_shim_controller__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_rotation_shim_controller__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_rotation_shim_controller__ubuntu_noble_amd64__binary/) |
| nav2_route | N/A | N/A | N/A | N/A | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_route__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_route__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_route__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_route__ubuntu_noble_amd64__binary/) |
| nav2_rviz_plugins | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_rviz_plugins__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_rviz_plugins__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_rviz_plugins__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_rviz_plugins__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_rviz_plugins__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_rviz_plugins__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_rviz_plugins__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_rviz_plugins__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_rviz_plugins__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_rviz_plugins__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_rviz_plugins__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_rviz_plugins__ubuntu_noble_amd64__binary/) |
| nav2_simple_commander | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_simple_commander__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_simple_commander__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_simple_commander__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_simple_commander__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_simple_commander__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_simple_commander__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_simple_commander__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_simple_commander__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_simple_commander__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_simple_commander__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_simple_commander__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_simple_commander__ubuntu_noble_amd64__binary/) |
| nav2_smac_planner | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_smac_planner__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_smac_planner__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_smac_planner__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_smac_planner__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_smac_planner__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_smac_planner__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_smac_planner__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_smac_planner__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_smac_planner__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_smac_planner__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_smac_planner__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_smac_planner__ubuntu_noble_amd64__binary/) |
| nav2_smoother | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_smoother__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_smoother__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_smoother__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_smoother__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_smoother__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_smoother__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_smoother__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_smoother__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_smoother__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_smoother__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_smoother__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_smoother__ubuntu_noble_amd64__binary/) |
| nav2_system_tests | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_system_tests__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_system_tests__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_system_tests__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_system_tests__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_system_tests__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_system_tests__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_system_tests__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_system_tests__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_system_tests__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_system_tests__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_system_tests__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_system_tests__ubuntu_noble_amd64__binary/) |
| nav2_theta_star_planner | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_theta_star_planner__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_theta_star_planner__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_theta_star_planner__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_theta_star_planner__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_theta_star_planner__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_theta_star_planner__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_theta_star_planner__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_theta_star_planner__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_theta_star_planner__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_theta_star_planner__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_theta_star_planner__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_theta_star_planner__ubuntu_noble_amd64__binary/) |
| nav2_util | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_util__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_util__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_util__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_util__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_util__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_util__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_util__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_util__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_util__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_util__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_util__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_util__ubuntu_noble_amd64__binary/) |
| nav2_velocity_smoother | [![Build Status](https://build.ros2.org/job/Hsrc_uj__nav2_velocity_smoother__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uj__nav2_velocity_smoother__ubuntu_jammy__source/) | [![Build Status](https://build.ros2.org/job/Hbin_uj64__nav2_velocity_smoother__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uj64__nav2_velocity_smoother__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jsrc_un__nav2_velocity_smoother__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Jsrc_un__nav2_velocity_smoother__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Jbin_un64__nav2_velocity_smoother__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_un64__nav2_velocity_smoother__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ksrc_un__nav2_velocity_smoother__ubuntu_noble__source/badge/icon)](https://build.ros2.org/job/Ksrc_un__nav2_velocity_smoother__ubuntu_noble__source/) | [![Build Status](https://build.ros2.org/job/Kbin_un64__nav2_velocity_smoother__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_un64__nav2_velocity_smoother__ubuntu_noble_amd64__binary/) |

