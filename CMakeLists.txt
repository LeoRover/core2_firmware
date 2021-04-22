cmake_minimum_required(VERSION 3.0.2)
project(LeoFirmware NONE)

set(HFRAMEWORK_PATH "/opt/hFramework" CACHE FILEPATH
    "Path to the hFramework SDK")

set(BOARD_TYPE "core2")
set(BOARD_VERSION "1.0.0")

include(${HFRAMEWORK_PATH}/hFramework.cmake)

include_directories(
    include/
    lib/rosserial/
    lib/sensors/
)

add_hexecutable(leo_firmware 
    main.cpp 
    src/diff_drive_controller.cpp 
    src/wheel_controller.cpp
    src/config.cpp
    src/logging.cpp
    src/imu.cpp
    src/gps.cpp
    src/parameters.cpp
    lib/rosserial/time.cpp
    lib/rosserial/duration.cpp
    lib/sensors/MPU9250/MPU9250.cpp)
