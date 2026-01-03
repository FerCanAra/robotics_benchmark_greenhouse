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
