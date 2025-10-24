# benchmark_robotics_greenhouse


## Comenzar navegación 

ros2 launch mvsim launch_mpc_benchmark.launch.py


## Hacer SLAM

ros2 launch mvsim launch_slam_benchmark.launch.py

En otra terminal, guardar el mapa:

ros2 run nav2_map_server map_saver_cli -f ~/name_map
