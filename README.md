# Leo_firmware

## ROS API

### Subscribed topics

* **`cmd_vel`** ([geometry_msgs/Twist])

    Target velocity of the Rover. Only linear.x (m/s) and angular.z (r/s) are used.

* **`servoX/angle`** ([std_msgs/Int16])

    Angular position (in degrees) of servo X. Angle to pulse duration relationship is configurable through `params.h` file

* **`servoX/pwm`** ([std_msgs/UInt16MultiArray])

    Pulse duration and period (in us). The values are passed through `data` array. Publishing to pwm topic overrides angle value and vice versa

### Published topics

* **`odom`** ([geometry_msgs/Twist])

    Current linear and angular velocities estimated from encoder readings

* **`battery`** ([std_msgs/Float32])

    Current battery voltage reading

* **`joint_states`** ([sensor_msgs/JointState])

    Current state of wheel joints. Effort is the percent of applied power (PWM duty)

[geometry_msgs/Twist]: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[std_msgs/Int16]: http://docs.ros.org/melodic/api/std_msgs/html/msg/Int16.html
[std_msgs/Float32]: http://docs.ros.org/api/std_msgs/html/msg/Float32.html
[std_msgs/UInt16MultiArray]: http://docs.ros.org/api/std_msgs/html/msg/UInt16MultiArray.html
[sensor_msgs/JointState]: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html