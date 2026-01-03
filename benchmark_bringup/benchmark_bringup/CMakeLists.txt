cmake_minimum_required(VERSION 3.8)
project(benchmark_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(benchmark_msg REQUIRED)

# Instalar módulo Python
ament_python_install_package(${PROJECT_NAME})

# Instalar launch
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Instalar definiciones
install(DIRECTORY definitions DESTINATION share/${PROJECT_NAME})

# Instalar mapas
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})

# Instalar parámetros
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

# Instalar resource/
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})

ament_package()
