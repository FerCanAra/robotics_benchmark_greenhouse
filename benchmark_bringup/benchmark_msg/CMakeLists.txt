cmake_minimum_required(VERSION 3.8)
project(benchmark_msg)

# 1. Dependencias de compilación
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 2. Archivos de mensaje
set(msg_files
  "msg/BenchmarkParams.msg"
)

# 3. Generación de interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

# 4. Exportar dependencias para otros paquetes
ament_export_dependencies(rosidl_default_runtime)

ament_package()
