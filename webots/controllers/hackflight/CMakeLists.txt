cmake_minimum_required(VERSION 3.10)
project(hackflight)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Webots libs (auto-detected)
find_package(webots REQUIRED)

# Include src/
include_directories(${PROJECT_SOURCE_DIR}/../src)
include_directories(${PROJECT_SOURCE_DIR}/../src/estimators)
include_directories(${PROJECT_SOURCE_DIR}/../src/sensors)
include_directories(${PROJECT_SOURCE_DIR}/../src/utils)  # For noise.h

# Source files
set(SOURCES
    hackflight.cpp  # Main sim entry (if exists; else add your main-like)
    ../src/estimators/OEKFEstimator.cpp  # Your OEKF
    ../src/estimators/KalmanEstimator.cpp  # Baseline
    ../src/sensors/MPU6050Sensor.cpp  # IMU with noise
    ../src/sensors/GPSSensor.cpp
    ../src/utils/noise.cpp  # If noise has .cpp; else header-only
    ../src/estimators/OEKFEstimator.cpp
    # Add more: ../src/Hackflight.cpp if exists
)

# Link Webots
webots_add_executable(hackflight ${SOURCES})
target_link_libraries(hackflight webots_cpp)  # C++ API

# Install (optional)
webots_install()
