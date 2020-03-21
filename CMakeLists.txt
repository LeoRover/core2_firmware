cmake_minimum_required(VERSION 2.8)
project(LeoFirmware NONE)

include(${HFRAMEWORK_PATH}/hFramework.cmake)
enable_module(hROS)

include_directories(
    include/
)

add_hexecutable(leo_firmware 
    main.cpp 
    src/diff_drive_controller.cpp 
    src/wheel_controller.cpp
    src/config.cpp
    src/logging.cpp
    src/imu.cpp
    src/MPU9250.cpp
    src/gps.cpp)