# CMake minimum version required
cmake_minimum_required(VERSION 3.22.1)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Project name and version
project(mpu9250_ahrs VERSION 0.1.0)

# Store all the cpp/c/cc file into variable from directory
aux_source_directory(${PROJECT_SOURCE_DIR} mpu9250_ahrs_files)

# build library (build .so file)
add_library(utils SHARED lib/utils.cpp)
add_library(oreientation SHARED lib/orientation.cpp)
add_library(transformation SHARED lib/transformation.cpp)
add_library(mpu6500 SHARED lib/mpu6500.cpp)
add_library(ak8963 SHARED lib/ak8963.cpp)
add_library(mpu9250 SHARED lib/mpu9250.cpp)

# find, add, link third-party libraries
# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

# WiringPi
find_library(WIRINGPI_LIBRARIES NAMES wiringPi) # /usr/local/lib
add_definitions(-lwiringPi)

# yaml-cpp
find_library(YAML_CPP_INCLUDE_DIRS NAMES yaml-cpp) # /usr/local/lib
add_definitions(-lyaml_cpp)

# build executable (link library to executable)
set(main_sources src/main.cpp lib/utils.cpp lib/orientation.cpp lib/transformation.cpp lib/mpu9250.cpp lib/mpu6500.cpp lib/ak8963.cpp)
set(calibration_sources src/calibration.cpp lib/utils.cpp lib/orientation.cpp lib/transformation.cpp lib/mpu9250.cpp lib/mpu6500.cpp lib/ak8963.cpp)
add_executable(main ${main_sources})
add_executable(calibration ${calibration_sources})
target_link_libraries(main ${Eigen3_LIBS} ${WIRINGPI_LIBRARIES} ${YAML_CPP_INCLUDE_DIRS} -lpthread)
target_link_libraries(calibration ${Eigen3_LIBS} ${WIRINGPI_LIBRARIES} ${YAML_CPP_INCLUDE_DIRS} -lpthread)

# set the output bin file directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
