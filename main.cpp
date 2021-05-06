#include <hFramework.h>

#include <ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <leo_firmware/diff_drive_controller.h>
#include <leo_firmware/logging.h>
#include <leo_firmware/parameters.h>
#include <leo_firmware/utils.h>

#include "params.h"

using hFramework::hServo;
using hFramework::sys;

ros::NodeHandle nh;

static std_msgs::Float32 battery;
static ros::Publisher *battery_pub;
static bool publish_battery = false;

static geometry_msgs::TwistStamped odom;
static ros::Publisher *odom_pub;
static geometry_msgs::PoseStamped pose;
static ros::Publisher *pose_pub;
static bool publish_odom = false;

static sensor_msgs::JointState joint_states;
static ros::Publisher *joint_states_pub;
static bool publish_joint = false;

DiffDriveController dc;

void cmdVelCallback(const geometry_msgs::Twist &msg) {
  logDebug("[cmdVelCallback] linear: %f angular %f", msg.linear.x,
           msg.angular.z);
  dc.setSpeed(msg.linear.x, msg.angular.z);
}

void resetBoardCallback(const std_srvs::EmptyRequest &req,
                        std_srvs::EmptyResponse &res) {
  logDebug("[resetBoardCallback]");
  sys.reset();
}

void resetOdometryCallback(const std_srvs::TriggerRequest &req,
                           std_srvs::TriggerResponse &res) {
  logDebug("[resetOdometryCallback]");
  dc.resetOdom();
  res.success = true;
}

void getFirmwareCallback(const std_srvs::TriggerRequest &req,
                         std_srvs::TriggerResponse &res) {
  logDebug("[getFirmwareCallback]");
  res.message = FIRMWARE_VERSION;
  res.success = true;
}

void initROS() {
  // Publishers
  battery_pub = new ros::Publisher("battery", &battery);
  odom_pub = new ros::Publisher("wheel_odom", &odom);
  pose_pub = new ros::Publisher("wheel_pose", &pose);
  joint_states_pub = new ros::Publisher("joint_states", &joint_states);

  nh.advertise(*battery_pub);
  nh.advertise(*odom_pub);
  nh.advertise(*pose_pub);
  nh.advertise(*joint_states_pub);

  // Subscribers
  auto twist_sub =
      new ros::Subscriber<geometry_msgs::Twist>("cmd_vel", &cmdVelCallback);

  nh.subscribe(*twist_sub);

  // Services
  auto reset_board_srv =
      new ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
          "core2/reset_board", &resetBoardCallback);
  auto reset_odometry_srv = new ros::ServiceServer<std_srvs::TriggerRequest,
                                                   std_srvs::TriggerResponse>(
      "core2/reset_odometry", &resetOdometryCallback);
  auto firmware_version_srv = new ros::ServiceServer<std_srvs::TriggerRequest,
                                                     std_srvs::TriggerResponse>(
      "core2/get_firmware_version", &getFirmwareCallback);

  nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
      *reset_board_srv);
  nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      *firmware_version_srv);
}

void setupJoints() {
  joint_states.name_length = 4;
  joint_states.name = new char *[4] {
    "wheel_FL_joint", "wheel_RL_joint", "wheel_FR_joint", "wheel_RR_joint"
  };
  joint_states.position_length = 4;
  joint_states.position = dc.positions;
  joint_states.velocity_length = 4;
  joint_states.velocity = dc.velocities;
  joint_states.effort_length = 4;
  joint_states.effort = dc.efforts;
}

void setupOdom() {
  odom.header.frame_id = params.robot_frame_id;
  pose.header.frame_id = params.odom_frame_id;
}

void batteryLoop() {
  uint32_t t = sys.getRefTime();
  const uint32_t dt = 1000;

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
  const uint32_t dt = 50;

  while (true) {
    if (!publish_odom) {
      odom.header.stamp = nh.now();

      Odom odo = dc.getOdom();
      odom.twist.linear.x = odo.vel_lin;
      odom.twist.angular.z = odo.vel_ang;
      pose.pose.position.x = odo.pose_x;
      pose.pose.position.y = odo.pose_y;
      pose.pose.orientation.z = std::sin(odo.pose_yaw * 0.5F);
      pose.pose.orientation.w = std::cos(odo.pose_yaw * 0.5F);

      publish_odom = true;
    }

    sys.delaySync(t, dt);
  }
}

void jointStatesLoop() {
  uint32_t t = sys.getRefTime();
  const uint32_t dt = 50;

  while (true) {
    if (!publish_joint) {
      joint_states.header.stamp = nh.now();
      dc.updateWheelStates();

      publish_joint = true;
    }

    sys.delaySync(t, dt);
  }
}

void LEDLoop() {
  uint32_t t = sys.getRefTime();
  const uint32_t dt = 250;

  while (true) {
    if (!nh.connected())
      LED.toggle();
    else
      LED.write(true);

    sys.delaySync(t, dt);
  }
}

void hMain() {
  RPi.setBaudrate(250000);
  nh.getHardware()->initWithDevice(&RPi);
  nh.initNode();

  LED.setOut();
  sys.taskCreate(&LEDLoop, 3);

  // Wait for rosserial connection
  while (!nh.connected()) {
    nh.spinOnce();
  }

  // Load ROS parameters
  params.load(nh);

  dc.init();

  setupOdom();
  setupJoints();
  initROS();

  sys.setLogDev(&Serial);

  sys.taskCreate(&batteryLoop, 3);
  sys.taskCreate(&odomLoop, 3);
  sys.taskCreate(&jointStatesLoop, 3);

  while (true) {
    nh.spinOnce();

    if (nh.connected()) {
      if (publish_battery) {
        battery_pub->publish(&battery);
        publish_battery = false;
      }

      if (publish_odom) {
        odom_pub->publish(&odom);
        pose_pub->publish(&pose);
        publish_odom = false;
      }

      if (publish_joint) {
        joint_states_pub->publish(&joint_states);
        publish_joint = false;
      }
    }
  }
}
