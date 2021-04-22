# leo_firmware

The firmware for the [Husarion CORE2] board running inside Leo Rover. 

The main functionalities include:
- steering the robot,
- setting position on the servos connected to `hServo` ports,
- reading battery voltage,
- reading motor positions
- estimating velocity of the robot
- IMU and GPS sensor support

It uses [rosserial] client library to expose its functionalities on ROS topics, services and parameters.

## Building and flashing
To build the project you will need to install cmake and GNU Arm Embedded Toolchain. On Ubuntu/Debian, you can do:
```
sudo apt-get install cmake gcc-arm-none-eabi
```
You will also need the [Husarion SDK]. Download it and unpack to the `/opt/hFramework` directory.

To build, enter the following commands on the terminal:
```
mkdir build && cd build
cmake ..
make
```
To flash, you can either use a USB cable to connect your computer to hSerial port on CORE2 board and type:
```
make flash
```
or you can upload the `leo_firmware.bin` file to Leo Rover and use [leo_fw] package to flash it:
```
rosrun leo_fw flash leo_firmware.bin
```

### Using Husarion extension for VSCode
You can also use the VSCode extension which already contains the Husarion SDK. This should work on Windows, Linux and MacOS platforms, but unfortunately break the VSCode IntelliSense configuration.

 Make sure you have installed:
- [Visual Studio Code],
- [Husarion extension] for VSCode

and followed the instructions on the [Husarion extension] page for installing the requirements.

Open the project in VSCode and click `[Ctrl]+[Shift]+[B]` to build it.

To flash the firmware, use a USB cable to connect your computer to hSerial port on CORE2 board. \
Then, click `[Ctrl]+[Shift]+[B]` and select `Flash project to CORE2`.

## ROS API

For the information about exposed ROS topics, services and parameters, visit [leo_fw] on ROS Wiki.


[leo_fw]: http://wiki.ros.org/leo_fw
[Visual Studio Code]: https://code.visualstudio.com
[Husarion extension]: https://marketplace.visualstudio.com/items?itemName=husarion.husarion
[Husarion CORE2]: https://husarion.com/manuals/core2/
[rosserial]: http://wiki.ros.org/rosserial
[Husarion SDK]: http://files.fictionlab.pl/husarion/Husarion_SDK-stable.zip