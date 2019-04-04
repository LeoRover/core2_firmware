#include "hFramework.h"
#include "hCloudClient.h"

#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16MultiArray.h"
#include "sensor_msgs/BatteryState.h"

#include "diff_controller.h"
#include "params.h"

using namespace hFramework;

ros::NodeHandle nh;

sensor_msgs::BatteryState battery;
ros::Publisher *battery_pub;

geometry_msgs::Twist odom;
ros::Publisher *odom_pub;

ros::Subscriber<geometry_msgs::Twist> *twist_sub;

DiffController dc;

class ServoWrapper
{
	int num;
	IServo& servo;

public:
	ServoWrapper(int num, IServo& servo)
		: num(num),
		  servo(servo) {}

	void angleCallback(const std_msgs::Int16& msg)
	{
		servo.rotAbs(msg.data);
		Serial.printf("[servo%dAngleCallback] angle: %d\r\n", num, msg.data);
	}

	void pwmCallback(const std_msgs::UInt16MultiArray& msg)
	{
		servo.setPeriod(msg.data[0]);
		servo.setWidth(msg.data[1]);
		Serial.printf("[servo%dPWMCallback] period: %d width: %d\r\n", num, msg.data[0], msg.data[1]);
	}
};

ServoWrapper servo1(1, hServo.servo1);
ServoWrapper servo2(2, hServo.servo2);
ServoWrapper servo3(3, hServo.servo3);
ServoWrapper servo4(4, hServo.servo4);
ServoWrapper servo5(5, hServo.servo5);
ServoWrapper servo6(6, hServo.servo6);

void cmdVelCallback(const geometry_msgs::Twist& msg)
{
	dc.setSpeed(msg.linear.x, msg.angular.z);
}

void initROS()
{
    battery_pub = new ros::Publisher("/battery", &battery);
	odom_pub = new ros::Publisher("/odom", &odom);
	twist_sub = new ros::Subscriber<geometry_msgs::Twist>("/cmd_vel", &cmdVelCallback);

	ros::Subscriber<std_msgs::Int16, ServoWrapper> *servo1_angle_sub = 
		new ros::Subscriber<std_msgs::Int16, ServoWrapper>("/servo1/angle", &ServoWrapper::angleCallback, &servo1);
	ros::Subscriber<std_msgs::Int16, ServoWrapper> *servo2_angle_sub = 
		new ros::Subscriber<std_msgs::Int16, ServoWrapper>("/servo2/angle", &ServoWrapper::angleCallback, &servo2);
	ros::Subscriber<std_msgs::Int16, ServoWrapper> *servo3_angle_sub = 
		new ros::Subscriber<std_msgs::Int16, ServoWrapper>("/servo3/angle", &ServoWrapper::angleCallback, &servo3);
	ros::Subscriber<std_msgs::Int16, ServoWrapper> *servo4_angle_sub = 
		new ros::Subscriber<std_msgs::Int16, ServoWrapper>("/servo4/angle", &ServoWrapper::angleCallback, &servo4);
	ros::Subscriber<std_msgs::Int16, ServoWrapper> *servo5_angle_sub = 
		new ros::Subscriber<std_msgs::Int16, ServoWrapper>("/servo5/angle", &ServoWrapper::angleCallback, &servo5);
	ros::Subscriber<std_msgs::Int16, ServoWrapper> *servo6_angle_sub = 
		new ros::Subscriber<std_msgs::Int16, ServoWrapper>("/servo6/angle", &ServoWrapper::angleCallback, &servo6);

	ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper> *servo1_pwm_sub =
		new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>("/servo1/pwm", &ServoWrapper::pwmCallback, &servo1);
	ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper> *servo2_pwm_sub =
		new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>("/servo2/pwm", &ServoWrapper::pwmCallback, &servo2);
	ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper> *servo3_pwm_sub =
		new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>("/servo3/pwm", &ServoWrapper::pwmCallback, &servo3);
	ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper> *servo4_pwm_sub =
		new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>("/servo4/pwm", &ServoWrapper::pwmCallback, &servo4);
	ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper> *servo5_pwm_sub =
		new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>("/servo5/pwm", &ServoWrapper::pwmCallback, &servo5);
	ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper> *servo6_pwm_sub =
		new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>("/servo6/pwm", &ServoWrapper::pwmCallback, &servo6);

    nh.advertise(*battery_pub);
	nh.advertise(*odom_pub);
	nh.subscribe(*twist_sub);
	nh.subscribe(*servo1_angle_sub);
	nh.subscribe(*servo2_angle_sub);
	nh.subscribe(*servo3_angle_sub);
	nh.subscribe(*servo4_angle_sub);
	nh.subscribe(*servo5_angle_sub);
	nh.subscribe(*servo6_angle_sub);
	nh.subscribe(*servo1_pwm_sub);
	nh.subscribe(*servo2_pwm_sub);
	nh.subscribe(*servo3_pwm_sub);
	nh.subscribe(*servo4_pwm_sub);
	nh.subscribe(*servo5_pwm_sub);
	nh.subscribe(*servo6_pwm_sub);
}

void setupServos()
{
	hServo.enablePower();
	hServo.setPeriod(SERVO_PERIOD);

	switch(SERVO_VOLTAGE) {
		case VOLTAGE_5V:
			hServo.setVoltage5V();
			break;
		case VOLTAGE_6V:
			hServo.setVoltage6V();
			break;
		case VOLTAGE_7V4:
			hServo.setVoltage7V4();
			break;
		case VOLTAGE_8V6:
			hServo.setVoltage8V6();
	}

	hServo.servo1.calibrate(SERVO_1_ANGLE_MIN, SERVO_1_WIDTH_MIN, 
							SERVO_1_ANGLE_MAX, SERVO_1_WIDTH_MAX); 
	hServo.servo2.calibrate(SERVO_2_ANGLE_MIN, SERVO_2_WIDTH_MIN, 
							SERVO_2_ANGLE_MAX, SERVO_2_WIDTH_MAX); 
	hServo.servo3.calibrate(SERVO_3_ANGLE_MIN, SERVO_3_WIDTH_MIN, 
							SERVO_3_ANGLE_MAX, SERVO_3_WIDTH_MAX);
	hServo.servo4.calibrate(SERVO_4_ANGLE_MIN, SERVO_4_WIDTH_MIN, 
							SERVO_4_ANGLE_MAX, SERVO_4_WIDTH_MAX); 
	hServo.servo5.calibrate(SERVO_5_ANGLE_MIN, SERVO_5_WIDTH_MIN, 
							SERVO_5_ANGLE_MAX, SERVO_5_WIDTH_MAX); 
	hServo.servo6.calibrate(SERVO_6_ANGLE_MIN, SERVO_6_WIDTH_MIN, 
							SERVO_6_ANGLE_MAX, SERVO_6_WIDTH_MAX); 
}

void batteryLoop()
{
    uint32_t t = sys.getRefTime();
    long dt = 1000;
    while(true)
    {
		nh.spinOnce();
        battery.voltage = sys.getSupplyVoltage();
        battery_pub->publish(&battery);
        sys.delaySync(t, dt);
    }
}

void odomLoop()
{
    uint32_t t = sys.getRefTime();
    long dt = 500;
    while(true)
    {
		nh.spinOnce();
		std::vector<float> odo = dc.getOdom();
		odom.linear.x = odo[0];
		odom.angular.z = odo[1];
        odom_pub->publish(&odom);
        sys.delaySync(t, dt);
    }
}

void hMain()
{
	uint32_t t = sys.getRefTime();
	platform.begin(&RPi);
	nh.getHardware()->initWithDevice(&platform.LocalSerial);
	nh.initNode();

	dc.start();
	setupServos();
	initROS();

	sys.taskCreate(&batteryLoop);
	sys.taskCreate(&odomLoop);

	while (true)
	{
		nh.spinOnce();
		sys.delaySync(t, 10);
	}

}
