#include "hFramework.h"
#include "hCloudClient.h"

#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
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

ros::Subscriber<std_msgs::Int16> *servo1_angle_sub;
ros::Subscriber<std_msgs::Int16> *servo2_angle_sub;
ros::Subscriber<std_msgs::Int16> *servo3_angle_sub;
ros::Subscriber<std_msgs::Int16> *servo4_angle_sub;
ros::Subscriber<std_msgs::Int16> *servo5_angle_sub;
ros::Subscriber<std_msgs::Int16> *servo6_angle_sub;
ros::Subscriber<std_msgs::Int16> *servo1_pwm_sub;
ros::Subscriber<std_msgs::Int16> *servo2_pwm_sub;
ros::Subscriber<std_msgs::Int16> *servo3_pwm_sub;
ros::Subscriber<std_msgs::Int16> *servo4_pwm_sub;
ros::Subscriber<std_msgs::Int16> *servo5_pwm_sub;
ros::Subscriber<std_msgs::Int16> *servo6_pwm_sub;

DiffController dc;

void servo1AngleCallback(const std_msgs::Int16& msg)
{
	hServo.servo1.rotAbs(msg.data);
	Serial.printf("[servo1AngleCallback] angle: %d\r\n", msg.data);
}

void servo2AngleCallback(const std_msgs::Int16& msg)
{
	hServo.servo2.rotAbs(msg.data);
	Serial.printf("[servo2AngleCallback] angle: %d\r\n", msg.data);
}

void servo3AngleCallback(const std_msgs::Int16& msg)
{
	hServo.servo3.rotAbs(msg.data);
	Serial.printf("[servo3AngleCallback] angle: %d\r\n", msg.data);
}

void servo4AngleCallback(const std_msgs::Int16& msg)
{
	hServo.servo4.rotAbs(msg.data);
	Serial.printf("[servo4AngleCallback] angle: %d\r\n", msg.data);
}

void servo5AngleCallback(const std_msgs::Int16& msg)
{
	hServo.servo5.rotAbs(msg.data);
	Serial.printf("[servo5AngleCallback] angle: %d\r\n", msg.data);
}

void servo6AngleCallback(const std_msgs::Int16& msg)
{
	hServo.servo6.rotAbs(msg.data);
	Serial.printf("[servo6AngleCallback] angle: %d\r\n", msg.data);
}

void cmdVelCallback(const geometry_msgs::Twist& msg)
{
	dc.setSpeed(msg.linear.x, msg.angular.z);
}

void initROS()
{
    battery_pub = new ros::Publisher("/battery", &battery);
	odom_pub = new ros::Publisher("/odom", &odom);
	twist_sub = new ros::Subscriber<geometry_msgs::Twist>("/cmd_vel", &cmdVelCallback);
	servo1_angle_sub = new ros::Subscriber<std_msgs::Int16>("/servo1/angle", &servo1AngleCallback);
	servo2_angle_sub = new ros::Subscriber<std_msgs::Int16>("/servo2/angle", &servo2AngleCallback);
	servo3_angle_sub = new ros::Subscriber<std_msgs::Int16>("/servo3/angle", &servo3AngleCallback);
	servo4_angle_sub = new ros::Subscriber<std_msgs::Int16>("/servo4/angle", &servo4AngleCallback);
	servo5_angle_sub = new ros::Subscriber<std_msgs::Int16>("/servo5/angle", &servo5AngleCallback);
	servo6_angle_sub = new ros::Subscriber<std_msgs::Int16>("/servo6/angle", &servo6AngleCallback);
    nh.advertise(*battery_pub);
	nh.advertise(*odom_pub);
	nh.subscribe(*twist_sub);
	nh.subscribe(*servo1_angle_sub);
	nh.subscribe(*servo2_angle_sub);
	nh.subscribe(*servo3_angle_sub);
	nh.subscribe(*servo4_angle_sub);
	nh.subscribe(*servo5_angle_sub);
	nh.subscribe(*servo6_angle_sub);
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
        battery.voltage = sys.getSupplyVoltage();
        battery_pub->publish(&battery);
        sys.delaySync(t, dt);
    }
}

void odomLoop()
{
    uint32_t t = sys.getRefTime();
    long dt = 50;
    while(true)
    {
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
