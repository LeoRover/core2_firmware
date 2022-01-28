cmake_minimum_required(VERSION 3.0.2)
project(LeoCore2Firmware NONE)

set(HFRAMEWORK_PATH "/opt/hFramework" CACHE FILEPATH
    "Path to the hFramework SDK")

set(BOARD_TYPE "core2")
set(BOARD_VERSION "1.0.0")

include(${HFRAMEWORK_PATH}/hFramework.cmake)

add_compile_definitions(ROS_PLATFORM_HFRAMEWORK)

if (RELEASE)
    optimization(3)
endif()

include_directories(
    include/
    lib/diff_drive_lib/
    lib/rosserial_lib/
    lib/sensors/
)

add_hexecutable(core2_firmware 
    main.cpp
    src/parameters.cpp
    src/motor_controller.cpp
    src/imu_receiver.cpp
    lib/diff_drive_lib/diff_drive_controller.cpp
    lib/diff_drive_lib/pid_regulator.cpp
    lib/diff_drive_lib/wheel_controller.cpp
    lib/rosserial_lib/time.cpp
    lib/rosserial_lib/duration.cpp
    lib/sensors/MPU9250/MPU9250.cpp)
