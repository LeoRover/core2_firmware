cmake_minimum_required(VERSION 3.0.2)
project(LeoFirmware NONE)

include(${HFRAMEWORK_PATH}/hFramework.cmake)

include_directories(
    include/
    rosserial/
)

add_hexecutable(leo_firmware 
    main.cpp 
    src/diff_drive_controller.cpp 
    src/wheel_controller.cpp
    src/config.cpp
    src/logging.cpp
    src/imu.cpp
    src/MPU9250.cpp
    src/gps.cpp
    src/parameters.cpp
    rosserial/time.cpp
    rosserial/duration.cpp)
