# Leo_firmware

## ROS API

### Subscribed topics

* **`cmd_vel`** ([geometry_msgs/Twist])

    Target velocity of the Rover. Only linear.x (m/s) and angular.z (r/s) are used.

* **`servoX/command`** ([std_msgs/Int16])

    Angular position (in degrees) of servo X

### Published topics

* **`odom`** ([geometry_msgs/Twist])

    Current linear and angular velocities estimated from encoder readings

* **`battery`** ([sensor_msgs/BatteryState])

    Current battery voltage reading

[geometry_msgs/Twist]: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[std_msgs/Int16]: http://docs.ros.org/melodic/api/std_msgs/html/msg/Int16.html
[sensor_msgs/BatteryState]: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/BatteryState.html