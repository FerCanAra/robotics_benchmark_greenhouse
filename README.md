# benchmark_robotics_greenhouse


## Comenzar navegación 

Lanzar simulador (mvsim) y controladores - Low level (PID + Slope Disturbance), mind level (TEB MPC) y high level (Tehta*)

ros2 launch mvsim launch_mpc_benchmark.launch.py

Lanzar waypoints para escanear todo el invernadero: 

ros2 run follow_waypoints follow_waypoints_exewaypoints_exe

## Hacer SLAM

ros2 launch mvsim launch_slam_benchmark.launch.py

En otra terminal, guardar el mapa:

ros2 run nav2_map_server map_saver_cli -f ~/name_map
