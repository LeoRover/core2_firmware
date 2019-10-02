# Leo_firmware

## ROS API

### Subscribed topics

* **`cmd_vel`** ([geometry_msgs/Twist])

    Target velocity of the Rover. Only linear.x (m/s) and angular.z (r/s) are used.

* **`servoX/angle`** ([std_msgs/Int16])

    Angular position (in degrees) of servo X. Angle to pulse duration relationship is configurable through `params.h` file

* **`servoX/pwm`** ([std_msgs/UInt16MultiArray])

    Pulse duration and period (in us). The values are passed through `data` array. Publishing to pwm topic overrides angle value and vice versa

* **`core2/reset_board`** ([std_msgs/Empty])

    Performs software reset on husarion board

* **`core2/reset_config`** ([std_msgs/Empty])

    Loads the default config and saves it to persistant storage

* **`core2/set_imu`** ([std_msgs/Bool])

    Enables or disables imu and saves the configuration to persistent storage. Needs a reset to apply

### Published topics

* **`odom`** ([geometry_msgs/Twist])

    Current linear and angular velocities estimated from encoder readings

* **`battery`** ([std_msgs/Float32])

    Current battery voltage reading

* **`joint_states`** ([sensor_msgs/JointState])

    Current state of wheel joints. Effort is the percent of applied power (PWM duty)

* **`imu/gyro`** ([geometry_msgs/Vector3Stamped]) (**only if IMU is enabled**)

    Current IMU gyroscope readings

* **`imu/accel`** ([geometry_msgs/Vector3Stamped]) (**only if IMU is enabled**)

    Current IMU accelerometer readings

* **`imu/mag`** ([geometry_msgs/Vector3Stamped]) (**only if IMU is enabled**)

    Current IMU magnetometer readings in North-West-Up world frame

### Services

* **`imu/calibrate_gyro_accel`** ([std_srvs/Trigger]) (**only if IMU is enabled**)

    Calibrates gyroscope and accelerometer biases and stores them in persistent storage.
    The IMU should lay perfectly still, parallel to the ground.

* **`imu/calibrate_mag`** ([std_srvs/Trigger]) (**only if IMU is enabled**)

    Calibrates magnetometer scale and biases and stores them in persistent storage.
    Wave the IMU in a figure eight until done.

[geometry_msgs/Twist]: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[std_msgs/Int16]: http://docs.ros.org/melodic/api/std_msgs/html/msg/Int16.html
[std_msgs/Float32]: http://docs.ros.org/api/std_msgs/html/msg/Float32.html
[std_msgs/UInt16MultiArray]: http://docs.ros.org/api/std_msgs/html/msg/UInt16MultiArray.html
[std_msgs/Bool]: http://docs.ros.org/api/std_msgs/html/msg/Bool.html
[std_msgs/Empty]: http://docs.ros.org/api/std_msgs/html/msg/Empty.html
[sensor_msgs/JointState]: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html
[geometry_msgs/Vector3Stamped]: http://docs.ros.org/api/geometry_msgs/html/msg/Vector3Stamped.html
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html