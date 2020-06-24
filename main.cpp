#include <hFramework.h>

#include <ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include <leo_firmware/diff_drive_controller.h>
#include <leo_firmware/logging.h>
#include <leo_firmware/utils.h>

#include "params.h"

ros::NodeHandle nh;

hFramework::hMutex mutex;

std_msgs::Float32 battery;
ros::Publisher *battery_pub;
bool publish_battery = false;

geometry_msgs::TwistStamped odom;
ros::Publisher *odom_pub;
bool publish_odom = false;

sensor_msgs::JointState joint_states;
ros::Publisher *joint_states_pub;
bool publish_joint = false;

ros::Subscriber<geometry_msgs::Twist> *twist_sub;
ros::Subscriber<std_msgs::Empty> *reset_board_sub;

DiffDriveController dc;

void cmdVelCallback(const geometry_msgs::Twist &msg) {
  logDebug("[cmdVelCallback] linear: %f angular %f", msg.linear.x,
           msg.angular.z);
  dc.setSpeed(msg.linear.x, msg.angular.z);
}

void resetBoardCallback(const std_msgs::Empty &msg) {
  logDebug("[resetBoardCallback]");
  sys.reset();
}

void initROS() {
  battery_pub = new ros::Publisher("battery", &battery);
  odom_pub = new ros::Publisher("wheel_odom", &odom);
  joint_states_pub = new ros::Publisher("joint_states", &joint_states);

  twist_sub =
      new ros::Subscriber<geometry_msgs::Twist>("cmd_vel", &cmdVelCallback);

  reset_board_sub = new ros::Subscriber<std_msgs::Empty>("core2/reset_board",
                                                         &resetBoardCallback);

  nh.advertise(*battery_pub);
  nh.advertise(*odom_pub);
  nh.advertise(*joint_states_pub);
  nh.subscribe(*twist_sub);
  nh.subscribe(*reset_board_sub);
}

void setupJoints() {
  joint_states.header.frame_id = "base_link";
  joint_states.name = new char *[4] {
    "wheel_FL_joint", "wheel_RL_joint", "wheel_FR_joint", "wheel_RR_joint"
  };
  joint_states.position = new float[4];
  joint_states.velocity = new float[4];
  joint_states.effort = new float[4];
  joint_states.name_length = 4;
  joint_states.position_length = 4;
  joint_states.velocity_length = 4;
  joint_states.effort_length = 4;
}

void setupOdom() { odom.header.frame_id = "base_link"; }

void batteryLoop() {
  uint32_t t = sys.getRefTime();
  uint32_t dt = 1000;

  while (true) {
    if (!publish_battery) {
      battery.data = sys.getSupplyVoltage();
      publish_battery = true;
    }

    sys.delaySync(t, dt);
  }
}

void odomLoop() {
  uint32_t t = sys.getRefTime();
  uint32_t dt = 50;

  while (true) {
    if (!publish_odom) {
      odom.header.stamp = nh.now();

      std::vector<float> odo = dc.getOdom();
      odom.twist.linear.x = odo[0];
      odom.twist.angular.z = odo[1];

      publish_odom = true;
    }

    sys.delaySync(t, dt);
  }
}

void jointStatesLoop() {
  uint32_t t = sys.getRefTime();
  uint32_t dt = 50;

  while (true) {
    if (!publish_joint) {
      std::vector<float> pos = dc.getWheelPositions();
      std::vector<float> vel = dc.getWheelVelocities();
      std::vector<float> eff = dc.getWheelEfforts();

      joint_states.header.stamp = nh.now();

      std::copy(pos.begin(), pos.end(), joint_states.position);
      std::copy(vel.begin(), vel.end(), joint_states.velocity);
      std::copy(eff.begin(), eff.end(), joint_states.effort);

      publish_joint = true;
    }

    sys.delaySync(t, dt);
  }
}

void LEDLoop() {
  uint32_t t = sys.getRefTime();
  uint32_t dt = 250;

  while (true) {
    if (!nh.connected())
      LED.toggle();
    else
      LED.write(true);

    sys.delaySync(t, dt);
  }
}

void hMain() {
  uint32_t t = sys.getRefTime();
  
  RPi.setBaudrate(250000);
  nh.getHardware()->initWithDevice(&RPi);
  nh.initNode();

  LED.setOut();
  sys.taskCreate(&LEDLoop);

  // Wait for rosserial connection
  while (!nh.connected()) {
    nh.spinOnce();
  }

  dc.init(&nh);
  dc.start();

  setupOdom();
  setupJoints();
  initROS();

  sys.setLogDev(&Serial);

  sys.taskCreate(&batteryLoop);
  sys.taskCreate(&odomLoop);
  sys.taskCreate(&jointStatesLoop);

  while (true) {
    nh.spinOnce();

    if (nh.connected()) {
      if (publish_battery) {
        battery_pub->publish(&battery);
        publish_battery = false;
      }

      if (publish_odom) {
        odom_pub->publish(&odom);
        publish_odom = false;
      }

      if (publish_joint) {
        joint_states_pub->publish(&joint_states);
        publish_joint = false;
      }
    }

    sys.delaySync(t, 1);
  }
}
