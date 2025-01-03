cmake_minimum_required(VERSION 3.10)

# Nombre del proyecto
project(launch)

# Encuentra las bibliotecas necesarias
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# Incluye los directorios de encabezado
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${REALSENSE2_INCLUDE_DIRS})

# Agrega el ejecutable y especifica los archivos fuente
add_executable(launch launch.cpp)

# Enlaza el ejecutable con las bibliotecas de OpenCV y librealsense2
target_link_libraries(launch ${OpenCV_LIBS} ${REALSENSE2_LIBRARIES})
