# Leo_firmware

## ROS API

### Subscribed topics

* **`cmd_vel`** ([geometry_msgs/Twist])

    Target velocity of the Rover.  
    Only linear.x (m/s) and angular.z (r/s) are used.

* **`servoX/angle`** ([std_msgs/Int16])

    Angular position (in user-defined units) of servo X.

* **`servoX/pwm`** ([std_msgs/UInt16MultiArray])

    Pulse duration and period (in nanoseconds) of servo X.  
    The values are passed through `data` array. Publishing to pwm topic overrides angle value and vice versa.

* **`core2/reset_board`** ([std_msgs/Empty])

    Performs software reset on the CORE2 board.

* **`core2/reset_config`** ([std_msgs/Empty])

    Loads the default config and saves it to persistant storage.

* **`core2/set_imu`** ([std_msgs/Bool])

    Enables or disables the IMU sensor and saves the configuration to persistent storage.  
    Requires a reset to apply.

* **`core2/set_gps`** ([std_msgs/Bool])

    Enables or disables the GPS sensor and saves the configuration to persistent storage.  
    Requires a reset to apply.

* **`core2/set_debug`** ([std_msgs/Bool])

    Enables or disables debug messages.  
    For the messages to be sent to rosout, you also need to set the logger level of rosserial node to Debug. When enabled, it can cause issues with the rosserial communication due to high throughput.

### Published topics

* **`wheel_odom`** ([geometry_msgs/TwistStamped])

    Current linear and angular velocities estimated from encoder readings.

* **`battery`** ([std_msgs/Float32])

    Current battery voltage reading.

* **`joint_states`** ([sensor_msgs/JointState])

    Current state of the wheel joints.  
    The units of measurements are as follows:  
    position in radians, velocity in radians per second, effort in the PWM duty cycle (percent).

* **`imu/gyro`** ([geometry_msgs/Vector3Stamped]) (**only if IMU is enabled**)

    Current IMU gyroscope readings in radians per second.

* **`imu/accel`** ([geometry_msgs/Vector3Stamped]) (**only if IMU is enabled**)

    Current IMU accelerometer readings in meters per second squared.

* **`imu/mag`** ([geometry_msgs/Vector3Stamped]) (**only if IMU is enabled**)

    Current IMU magnetometer readings in Gauss.  
    The axes represent the North-West-Up world frame.

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

    The distance (in meters) between the centers of the left and right wheels.

* **`core2/diff_drive/angular_velocity_multiplier`** (`float`, default: `1.91`)

    Upon receiving a `cmd_vel` command, the angular velocity is multiplied by this parameter and the calculated odometry has its angular velocity divided by this parameter. This is done to account for a difference between a two-wheel robot model and the real robot.

* **`core2/diff_drive/input_timeout`** (`int`, default: `500`)

    The timeout (in milliseconds) for the `cmd_vel` commands.  
    The differential drive controller will stop the motors if it does not receive a command within the specified time. Set it to `0` to disable the timeout.

* **`core2/motors/encoder_resolution`** (`float`, default: `878.4`)

    The resolution of the wheel encoders in counts per rotation.

* **`core2/motors/encoder_pullup`** (`int`, default: `1`)

    Whether to use internal pull-up for the encoder logic pins.  
    Value of `1` means `yes`, any other value means `no`

* **`core2/motors/max_speed`** (`float`, default: `800.0`)

    The maximum reachable speed of the motors in encoder counts per second.  
    Used for limiting the value passed to the wheel controllers.

* **`core2/motors/pid/p`** (`float`, default: `0.0`)

    P constant of the motor's PID regulators.

* **`core2/motors/pid/i`** (`float`, default: `0.005`)

    I constant of the motor's PID regulators.

* **`core2/motors/pid/d`** (`float`, default: `0.0`)

    D constant of the motor's PID regulators.

* **`core2/motors/power_limit`** (`float`, default: `1000.0`)

    Limit of the PWM duty applied to the motors.  
    The value should be between `0.0` (0% duty) and `1000.0` (100% duty).

* **`core2/motors/torque_limit`** (`float`, default: `1000.0`)

    This value applies an additional power limit depending on the current speed of the motors.  
    The formula is as follows:   
    power_limit = (current_speed / max_speed) * 1000.0 + torque_limit

* **`core2/servo_voltage`** (`int`, default: `2`)

    The number describing the servo power line voltage to set.  
    The mapping of the values and corresponding voltages is as follows:  
    `0` - 5V, `1` - 6V, `2` - 7.4V, `3` - 8.6V

* **`core2/servoX/period`** (`int`, default: `20000`)

    The period (in nanoseconds) of the PWM wave for servo X.

* **`core2/servoX/angle_min`** (`int`, default: `-90`)

    The minimal angle (in user-defined units) for servo X.

* **`core2/servoX/angle_max`** (`int`, default: `90`)

    The maximal angle (in user-defined units) for servo X.

* **`core2/servoX/width_min`** (`int`, default: `1000`)

    The width of the pulse (in nanoseconds) corresponding to the minimal angle for servo X.

* **`core2/servoX/width_max`** (`int`, default: `2000`)

    The width of the pulse (in nanoseconds) corresponding to the maximal angle for servo X.

* **`core2/imu_frame_id`** (`string`, default: `imu`) (**only if IMU is enabled**)

    The value of the `frame_id` field sent in the header of IMU messages.

* **`core2/gps_frame_id`** (`string`, default: `gps`) (**only if GPS is enabled**)

    The value of the `frame_id` field sent in the header of GPS messages.


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