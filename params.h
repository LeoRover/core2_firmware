#include <cstdint>

// Servos
const uint16_t SERVO_PERIOD = 20000;

enum voltage {VOLTAGE_5V, VOLTAGE_6V, VOLTAGE_7V4, VOLTAGE_8V6};
const voltage SERVO_VOLTAGE = VOLTAGE_7V4;

const int16_t SERVO_1_ANGLE_MIN = 0;
const int16_t SERVO_1_ANGLE_MAX = 90;
const uint16_t SERVO_1_WIDTH_MIN = 1000;
const uint16_t SERVO_1_WIDTH_MAX = 1300;

const int16_t SERVO_2_ANGLE_MIN = 0;
const int16_t SERVO_2_ANGLE_MAX = 90;
const uint16_t SERVO_2_WIDTH_MIN = 1000;
const uint16_t SERVO_2_WIDTH_MAX = 1300;

const int16_t SERVO_3_ANGLE_MIN = 0;
const int16_t SERVO_3_ANGLE_MAX = 90;
const uint16_t SERVO_3_WIDTH_MIN = 1000;
const uint16_t SERVO_3_WIDTH_MAX = 1300;

const int16_t SERVO_4_ANGLE_MIN = 0;
const int16_t SERVO_4_ANGLE_MAX = 90;
const uint16_t SERVO_4_WIDTH_MIN = 1000;
const uint16_t SERVO_4_WIDTH_MAX = 1300;

const int16_t SERVO_5_ANGLE_MIN = 0;
const int16_t SERVO_5_ANGLE_MAX = 90;
const uint16_t SERVO_5_WIDTH_MIN = 1000;
const uint16_t SERVO_5_WIDTH_MAX = 1300;

const int16_t SERVO_6_ANGLE_MIN = 0;
const int16_t SERVO_6_ANGLE_MAX = 90;
const uint16_t SERVO_6_WIDTH_MIN = 1000;
const uint16_t SERVO_6_WIDTH_MAX = 1300;

// Wheels
const float ENCODER_RESOLUTION = 8256; //in ticks per rotation
const float WHEEL_RADIUS = 0.06; //in meters
const float WHEEL_MAX_SPEED = 6500; //in ticks per second
const float ROBOT_WIDTH = 0.33; //in meters

// PID
const float PID_P = 0.00;
const float PID_I = 0.0005;
const float PID_D = 0.0;

// Value between 0 and 1000 describing power limit
// e.g. 1000 means no limit, 800 corresponds to 80%
// Take into account that this value only limits the Voltage (PWM) and not the current.
const int32_t POWER_LIMIT = 1000;

// Input timeout in ms. 
// The controller will stop the motors if it doesn't receive a command for a specified time. 
// If set to 0, the controller won't check for a timeout
const uint32_t INPUT_TIMEOUT = 500;