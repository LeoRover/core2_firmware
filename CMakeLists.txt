cmake_minimum_required(VERSION 2.8)
project(LeoFirmware NONE)

include(${HFRAMEWORK_PATH}/hFramework.cmake)
enable_module(hCloudClient)
enable_module(hSensors)
enable_module(hModules)
enable_module(hROS)

add_hexecutable(leo_firmware main.cpp diff_controller.cpp wheel.cpp imu.cpp imu/MPU9250.cpp)