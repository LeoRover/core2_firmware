#include "hFramework.h"
#include "hCloudClient.h"

#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16MultiArray.h"
#include "sensor_msgs/JointState.h"

#include "diff_controller.h"
#include "params.h"
#include "utils.h"

using namespace hFramework;

ros::NodeHandle nh;

std_msgs::Float32 battery;
ros::Publisher *battery_pub;
bool publish_battery = false;

geometry_msgs::Twist odom;
ros::Publisher *odom_pub;
bool publish_odom = false;

sensor_msgs::JointState joint_states;
ros::Publisher *joint_states_pub;
bool publish_joint = false;

ros::Subscriber<geometry_msgs::Twist> *twist_sub;

DiffController *dc;

ServoWrapper servo1(1, hServo.servo1);
ServoWrapper servo2(2, hServo.servo2);
ServoWrapper servo3(3, hServo.servo3);
ServoWrapper servo4(4, hServo.servo4);
ServoWrapper servo5(5, hServo.servo5);
ServoWrapper servo6(6, hServo.servo6);

void cmdVelCallback(const geometry_msgs::Twist& msg)
{
	dc->setSpeed(msg.linear.x, msg.angular.z);
#ifdef DEBUG
	Serial.printf("[cmdVelCallback] linear: %f angular %f\r\n", msg.linear.x, msg.angular.z);
#endif
}

void initROS()
{
    battery_pub = new ros::Publisher("/battery", &battery);
	odom_pub = new ros::Publisher("/odom", &odom);
	joint_states_pub = new ros::Publisher("/joint_states", &joint_states);
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
	nh.advertise(*joint_states_pub);
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

void setupJoints()
{
	joint_states.header.frame_id = "base_link";
	joint_states.name = new char*[4] {"wheelFL", "wheelRL", "wheelFR", "wheelRR"};
	joint_states.position = new float[4];
	joint_states.velocity = new float[4];
	joint_states.effort = new float[4];
	joint_states.name_length = 4;
	joint_states.position_length = 4;
	joint_states.velocity_length = 4;
	joint_states.effort_length = 4;
}

void batteryLoop()
{
    uint32_t t = sys.getRefTime();
    long dt = 1000;
    while(true)
    {
		if (!publish_battery) 
		{
			battery.data = sys.getSupplyVoltage();
			
			publish_battery = true;
		}

        sys.delaySync(t, dt);
    }
}

void odomLoop()
{
    uint32_t t = sys.getRefTime();
    long dt = 50;
    while(true)
    {
		if (!publish_odom)
		{
			std::vector<float> odo = dc->getOdom();
			odom.linear.x = odo[0];
			odom.angular.z = odo[1];

			publish_odom = true;
		}

        sys.delaySync(t, dt);
    }
}

void jointStatesLoop()
{
    uint32_t t = sys.getRefTime();
    long dt = 50;
    while(true)
    {
		if (!publish_joint)
		{
			std::vector<float> pos = dc->getWheelPositions();
			std::vector<float> vel = dc->getWheelVelocities();
			std::vector<float> eff = dc->getWheelEfforts();

			joint_states.header.stamp = nh.now();

			joint_states.position[0] = pos[0];
			joint_states.position[1] = pos[1];
			joint_states.position[2] = pos[2];
			joint_states.position[3] = pos[3];

			joint_states.velocity[0] = vel[0];
			joint_states.velocity[1] = vel[1];
			joint_states.velocity[2] = vel[2];
			joint_states.velocity[3] = vel[3];

			joint_states.effort[0] = eff[0];
			joint_states.effort[1] = eff[1];
			joint_states.effort[2] = eff[2];
			joint_states.effort[3] = eff[3];

			publish_joint = true;
		}

        sys.delaySync(t, dt);
    }
}

void LEDLoop()
{
    uint32_t t = sys.getRefTime();
    long dt = 250;
    while(true)
    {
		if(!nh.connected())
			LED.toggle();
		else
			LED.write(true);

        sys.delaySync(t, dt);
    }
}

void hMain()
{
	uint32_t t = sys.getRefTime();
	platform.begin(&RPi);
	nh.getHardware()->initWithDevice(&platform.LocalSerial);
	//nh.getHardware()->initWithDevice(&RPi);
	nh.initNode();
	
	dc = new DiffController(INPUT_TIMEOUT);
	dc->start();

	setupServos();
	setupJoints();
	initROS();

	LED.setOut();
	sys.taskCreate(&LEDLoop);

	sys.taskCreate(&batteryLoop);
	sys.taskCreate(&odomLoop);
	sys.taskCreate(&jointStatesLoop);

	while (true)
	{
		nh.spinOnce();

		if (publish_battery){
			battery_pub->publish(&battery);
			publish_battery = false;
		}

		if (publish_odom){
			odom_pub->publish(&odom);
			publish_odom = false;
		}

		if (publish_joint){
			joint_states_pub->publish(&joint_states);
			publish_joint = false;
		}

		sys.delaySync(t, 10);
	}


}
