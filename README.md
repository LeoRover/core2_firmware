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

## Building
To build the project, you will need:
- [Visual Studio Code],
- [Husarion extension] for VSCode

Open the project on VSCode and click `[Ctrl]+[Shift]+[B]`.

To flash the firmware, use a USB cable to connect your computer to hSerial port on CORE2 board. \
Then, click `[Ctrl]+[Shift]+[B]` and select `Flash project to CORE2`.

## ROS API

For the information about exposed ROS topics, services and parameters, visit [leo_fw] on ROS Wiki.


[leo_fw]: http://wiki.ros.org/leo_fw
[Visual Studio Code]: https://code.visualstudio.com
[Husarion extension]: https://marketplace.visualstudio.com/items?itemName=husarion.husarion
[Husarion CORE2]: https://husarion.com/manuals/core2/
[rosserial]: http://wiki.ros.org/rosserial