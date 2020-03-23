# Leo_firmware

## ROS API

### Subscribed topics

* **`cmd_vel`** ([geometry_msgs/Twist])

    Target velocity of the Rover. Only linear.x (m/s) and angular.z (r/s) are used.

* **`servoX/angle`** ([std_msgs/Int16])

    Angular position (in degrees) of servo X. Angle to pulse duration relationship is configurable through `params.h` file.

* **`servoX/pwm`** ([std_msgs/UInt16MultiArray])

    Pulse duration and period (in us). The values are passed through `data` array. Publishing to pwm topic overrides angle value and vice versa.

* **`core2/reset_board`** ([std_msgs/Empty])

    Performs software reset on the CORE2 board.

* **`core2/reset_config`** ([std_msgs/Empty])

    Loads the default config and saves it to persistant storage.

* **`core2/set_imu`** ([std_msgs/Bool])

    Enables or disables the IMU sensor and saves the configuration to persistent storage. Requires a reset to apply

* **`core2/set_gps`** ([std_msgs/Bool])

    Enables or disables the GPS sensor and saves the configuration to persistent storage. Requires a reset to apply

* **`core2/set_debug`** ([std_msgs/Bool])

    Enables or disables debug messages. For the messages to be sent to rosout, you also need to set the logger level of rosserial node to Debug. When enabled, it can cause issues with the rosserial communication due to high throughput.

### Published topics

* **`wheel_odom`** ([geometry_msgs/TwistStamped])

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

* **`gps_fix`** ([sensor_msgs/NavSatFix]) (**only if GPS is enabled**)

    A Navigation Satellite fix returned by the GPS sensor.

### Services

* **`imu/calibrate_gyro_accel`** ([std_srvs/Trigger]) (**only if IMU is enabled**)

    Calibrates gyroscope and accelerometer biases and stores them in persistent storage.
    The IMU should lay perfectly still, parallel to the ground.

* **`imu/calibrate_mag`** ([std_srvs/Trigger]) (**only if IMU is enabled**)

    Calibrates magnetometer scale and biases and stores them in persistent storage.
    Wave the IMU in a figure eight until done.

### Parameters

* **`core2/diff_drive/wheel_radius`** (`float`, default: `0.0625`)

    The radius of the wheel in meters.

* **`core2/diff_drive/wheel_separation`** (`float`, default: `0.33`)

    The distance, in meters, between the centers of the left and right wheels.

* **`core2/diff_drive/angular_velocity_multiplier`** (`float`, default: `1.91`)

    TODO

* **`core2/diff_drive/input_timeout`** (`int`, default: `500`)

    TODO

* **`core2/motors/encoder_resolution`** (`float`, default: `878.4`)

    TODO

* **`core2/motors/encoder_pullup`** (`int`, default: `1`)

    TODO

* **`core2/motors/max_speed`** (`float`, default: `800.0`)

    TODO

* **`core2/motors/pid/p`** (`float`, default: `0.0`)

    TODO

* **`core2/motors/pid/i`** (`float`, default: `0.005`)

    TODO

* **`core2/motors/pid/d`** (`float`, default: `0.0`)

    TODO

* **`core2/motors/power_limit`** (`float`, default: `1000.0`)

    TODO

* **`core2/motors/torque_limit`** (`float`, default: `1000.0`)

    TODO

* **`core2/servo_voltage`** (`int`, default: `2`)

    TODO

* **`core2/servoX/period`** (`int`, default: `20000`)

    TODO

* **`core2/servoX/angle_min`** (`int`, default: `-90`)

    TODO

* **`core2/servoX/angle_max`** (`int`, default: `90`)

    TODO

* **`core2/servoX/width_min`** (`int`, default: `1000`)

    TODO

* **`core2/servoX/width_max`** (`int`, default: `2000`)

    TODO

* **`core2/imu_frame_id`** (`string`, default: `imu`) (**only if IMU is enabled**)

    TODO

* **`core2/gps_frame_id`** (`string`, default: `gps`) (**only if GPS is enabled**)

    TODO


[geometry_msgs/Twist]: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[geometry_msgs/TwistStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html
[std_msgs/Int16]: http://docs.ros.org/api/std_msgs/html/msg/Int16.html
[std_msgs/Float32]: http://docs.ros.org/api/std_msgs/html/msg/Float32.html
[std_msgs/UInt16MultiArray]: http://docs.ros.org/api/std_msgs/html/msg/UInt16MultiArray.html
[std_msgs/Bool]: http://docs.ros.org/api/std_msgs/html/msg/Bool.html
[std_msgs/Empty]: http://docs.ros.org/api/std_msgs/html/msg/Empty.html
[sensor_msgs/JointState]: http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
[geometry_msgs/Vector3Stamped]: http://docs.ros.org/api/geometry_msgs/html/msg/Vector3Stamped.html
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/NavSatFix]: http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html