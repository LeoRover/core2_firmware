cmake_minimum_required(VERSION 2.8)
project(LeoFirmware NONE)

include(${HFRAMEWORK_PATH}/hFramework.cmake)
enable_module(hROS)

add_hexecutable(leo_firmware 
    main.cpp 
    diff_controller.cpp 
    wheel.cpp
    config.cpp
    imu.cpp
    imu/MPU9250.cpp)