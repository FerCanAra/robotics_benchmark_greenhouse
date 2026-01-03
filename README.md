[![mvsim](https://circleci.com/gh/MRPT/mvsim.svg?style=svg)](https://circleci.com/gh/MRPT/mvsim) [![Documentation Status](https://readthedocs.org/projects/mvsimulator/badge/?version=latest)](https://mvsimulator.readthedocs.io/en/latest/?badge=latest)
[![CI Linux](https://github.com/MRPT/mvsim/actions/workflows/build-linux.yml/badge.svg)](https://github.com/MRPT/mvsim/actions/workflows/build-linux.yml)
[![CI Check clang-format](https://github.com/MRPT/mvsim/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MRPT/mvsim/actions/workflows/check-clang-format.yml)

A Benchmark for Evaluating Advanced Control Strategies in Mobile Robots for Mediterranean Greenhouse
======================================
Se trata de un benchmarck basado en robots móviles para implementar diferentes tecnicas de control, dividida en tres niveles: Bajo, medio y alto nivel. Como simulador, se usa [MultiVehicle simulator (MVSim) ]([https://github.com/MRPT/mvsim]), a lightweight, realistic dynamical simulator for 2D ("2.5D") vehicles and robots. It is tailored to analysis of vehicle dynamics, wheel-ground contact forces and accurate simulation of typical robot sensors (e.g. 2D and 3D lidars).

License: 3-clause BSD License
Copyright (C) 2014-2025 Fernando Cañadas Aránega <fernando.ca@ual.es> (University of Almeria) and collaborators


Preequisitos
--------------------
Para poder utilizar el simulador, debera tener instalado los siguientes paquetes:

1. [Robot Operating System (ROS 2) Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Se recomenda installar el full desktop
2. [The Mobile Robot Programming Toolkit (MRPT)](https://docs.mrpt.org/reference/latest/download-mrpt.html). Se instala dacilmente con el siguiente comando:
```
sudo apt install libmrpt-dev mrpt-apps
```
3. [Navigation 2 (Nav2)](https://docs.nav2.org/getting_started/index.html). Se instala dacilmente con el siguiente comando:
```
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2-bringup
```

Installation
--------------------
Para poder ejecutar el el simulador, se debe instalar el repositorio oficial del proyecto. Se puede hacer facilmente mediante los siguientes comandos.

```
mkdir -p ~/robotics_benchmark_greenhouse/src
cd ~/robotics_benchmark_greenhouse/src
git clone --recurse-submodules https://github.com/FerCanAra/robotics_benchmark_greenhouse.git
cd robotics_benchmark_greenhouse
git submodule update --init --recursive
cd ../..
colcon build --packages-select mvsim --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```














Build matrix status
--------------------

| Distro | Build dev | Build releases | Stable version |
| ---    | ---       | ---            | ---         |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mvsim__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mvsim__ubuntu_jammy_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mvsim__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mvsim__ubuntu_jammy_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mvsim__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mvsim__ubuntu_jammy_arm64__binary/) | [![Version](https://img.shields.io/ros/v/humble/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 2 Jazzy @ u24.04 | [![Build Status](https://build.ros2.org/job/Jdev__mvsim__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mvsim__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Jbin_uN64__mvsim__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mvsim__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/jazzy/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 2 Kilted @ u24.04 | [![Build Status](https://build.ros2.org/job/Kdev__mvsim__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mvsim__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Kbin_uN64__mvsim__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mvsim__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/kilted/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mvsim__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mvsim__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Rbin_uN64__mvsim__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mvsim__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Rbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uNv8__mvsim__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |


| EOL distro | Stable version |
| ---    | ---       |
| ROS 1 Melodic (u18.04) | [![Version](https://img.shields.io/ros/v/melodic/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 1 Noetic (u20.04) | [![Version](https://img.shields.io/ros/v/noetic/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 2 Foxy (u20.04) | [![Version](https://img.shields.io/ros/v/foxy/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |
| ROS 2 Iron (u22.04) | [![Version](https://img.shields.io/ros/v/iron/mvsim)](https://index.ros.org/?search_packages=true&pkgs=mvsim) |





See [installation documentation](https://mvsimulator.readthedocs.io/en/latest/install.html) for all the details and options. 

The easiest way to install if you already have ROS 1 or ROS 2 is:

    sudo apt install ros-$ROS_DISTRO-mvsim

Then jump to [next steps](https://mvsimulator.readthedocs.io/en/latest/first-steps.html) to see how to launch some of the demo worlds.


## Comenzar navegación 

Lanzar simulador (mvsim) y controladores - Low level (PID + Slope Disturbance), mind level (TEB MPC) y high level (Tehta*)

ros2 launch mvsim launch_mpc_benchmark.launch.py

Lanzar waypoints para escanear todo el invernadero: 

ros2 run follow_waypoints follow_waypoints_exewaypoints_exe

## Hacer SLAM

ros2 launch mvsim launch_slam_benchmark.launch.py

En otra terminal, guardar el mapa:

ros2 run nav2_map_server map_saver_cli -f ~/name_map





```
t clone --recurse-submodules https://github.com/FerCanAra/benchmark_bringup.git



```
