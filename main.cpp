#include "hFramework.h"
#include "hCloudClient.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "diff_controller.h"

using namespace hFramework;

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> *twist_sub;

DiffController df;

void cmdVelCallback(const geometry_msgs::Twist& msg)
{
	df.setSpeed(msg.linear.x, msg.angular.z);
}

void initROS()
{
	twist_sub = new ros::Subscriber<geometry_msgs::Twist>("/cmd_vel", &cmdVelCallback);
	nh.subscribe(*twist_sub);
}

void hMain()
{
	uint32_t t = sys.getRefTime();
	platform.begin(&RPi);
	//sys.setLogDev(&platform.LocalSerial);
	nh.getHardware()->initWithDevice(&platform.LocalSerial);
	nh.initNode();

	df.start();
	initROS();

	while (true)
	{
		nh.spinOnce();

		//do something

		sys.delaySync(t, 10);
	}

}
