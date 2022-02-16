#include <hFramework.h>

#include <ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <leo_firmware/config.h>
#include <leo_firmware/diff_drive_controller.h>
#include <leo_firmware/logging.h>
#include <leo_firmware/parameters.h>
#include <leo_firmware/sensors/gps.h>
#include <leo_firmware/sensors/imu.h>
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

static IMU *imu;
static geometry_msgs::Vector3Stamped imu_gyro_msg;
static ros::Publisher *imu_gyro_pub;
static geometry_msgs::Vector3Stamped imu_accel_msg;
static ros::Publisher *imu_accel_pub;
static geometry_msgs::Vector3Stamped imu_mag_msg;
static ros::Publisher *imu_mag_pub;
static bool publish_imu = false;

static GPS *gps;
static sensor_msgs::NavSatFix gps_fix;
static ros::Publisher *gps_pub;
static bool publish_gps = false;

static DiffDriveController dc;

static ServoWrapper servo1(1, hServo.servo1);
static ServoWrapper servo2(2, hServo.servo2);
static ServoWrapper servo3(3, hServo.servo3);
static ServoWrapper servo4(4, hServo.servo4);
static ServoWrapper servo5(5, hServo.servo5);
static ServoWrapper servo6(6, hServo.servo6);

static std_msgs::Bool relay1_status;
static ros::Publisher *relay1_pub;
static std_msgs::Bool relay2_status;
static ros::Publisher *relay2_pub;
static std_msgs::Bool relay3_status;
static ros::Publisher *relay3_pub;
static std_msgs::Bool relay4_status;
static ros::Publisher *relay4_pub;
static bool publish_relay = false;

void setRelay1Callback(const std_srvs::SetBoolRequest &req,
                    std_srvs::SetBoolResponse &res) {
  logDebug("[setRelay1Callback] %s", req.data ? "true" : "false");
  if (req.data == true)
    hSens1.pin1.write(1);
  else
    hSens1.pin1.write(0);
  ;
  res.success = true;
}

void setRelay2Callback(const std_srvs::SetBoolRequest &req,
                    std_srvs::SetBoolResponse &res) {
  logDebug("[setRelay2Callback] %s", req.data ? "true" : "false");
  if (req.data == true)
    hSens1.pin2.write(1);
  else
    hSens1.pin2.write(0);
  ;
  res.success = true;
}

void setRelay3Callback(const std_srvs::SetBoolRequest &req,
                    std_srvs::SetBoolResponse &res) {
  logDebug("[setRelay3Callback] %s", req.data ? "true" : "false");
  if (req.data == true)
    hSens1.pin3.write(1);
  else
    hSens1.pin3.write(0);
  ;
  res.success = true;
}

void setRelay4Callback(const std_srvs::SetBoolRequest &req,
                    std_srvs::SetBoolResponse &res) {
  logDebug("[setRelay4Callback] %s", req.data ? "true" : "false");
  if (req.data == true)
    hSens1.pin4.write(1);
  else
    hSens1.pin4.write(0);
  ;
  res.success = true;
}

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

void resetConfigCallback(const std_srvs::TriggerRequest &req,
                         std_srvs::TriggerResponse &res) {
  logDebug("[resetConfigCallback]");
  configReset();
  res.success = true;
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

void setImuCallback(const std_srvs::SetBoolRequest &req,
                    std_srvs::SetBoolResponse &res) {
  logDebug("[setImuCallback] %s", req.data ? "true" : "false");
  conf.imu_enabled = req.data;
  configStore();
  res.success = true;
}

void setGpsCallback(const std_srvs::SetBoolRequest &req,
                    std_srvs::SetBoolResponse &res) {
  logDebug("[setGpsCallback] %s", req.data ? "true" : "false");
  conf.gps_enabled = req.data;
  configStore();
  res.success = true;
}

void setDebugCallback(const std_srvs::SetBoolRequest &req,
                      std_srvs::SetBoolResponse &res) {
  logDebug("[setDebugCallback] %s", req.data ? "true" : "false");
  conf.debug_logging = req.data;
  configStore();
  res.success = true;
}

void calMpuCallback(const std_srvs::TriggerRequest &req,
                    std_srvs::TriggerResponse &res) {
  logDebug("[calMpuCallback]");
  imu->calGyroAccel();
  res.message = "Succesfully calibrated gyroscope and accelerometer biases";
  res.success = true;
}

void calMagCallback(const std_srvs::TriggerRequest &req,
                    std_srvs::TriggerResponse &res) {
  logDebug("[calMagCallback]");
  imu->calMag();
  res.message = "Succesfully calibrated magnetometer";
  res.success = true;
}

void initROS() {
  // Publishers
  battery_pub = new ros::Publisher("battery", &battery);
  odom_pub = new ros::Publisher("wheel_odom", &odom);
  pose_pub = new ros::Publisher("wheel_pose", &pose);
  joint_states_pub = new ros::Publisher("joint_states", &joint_states);
  relay1_pub = new ros::Publisher("relay1_status", &relay1_status);
  relay2_pub = new ros::Publisher("relay2_status", &relay2_status);
  relay3_pub = new ros::Publisher("relay3_status", &relay3_status);
  relay4_pub = new ros::Publisher("relay4_status", &relay4_status);

  nh.advertise(*battery_pub);
  nh.advertise(*odom_pub);
  nh.advertise(*pose_pub);
  nh.advertise(*joint_states_pub);
  nh.advertise(*relay1_pub);
  nh.advertise(*relay2_pub);
  nh.advertise(*relay3_pub);
  nh.advertise(*relay4_pub);

  // Subscribers
  auto twist_sub =
      new ros::Subscriber<geometry_msgs::Twist>("cmd_vel", &cmdVelCallback);

  auto servo1_angle_sub = new ros::Subscriber<std_msgs::Int16, ServoWrapper>(
      "servo1/angle", &ServoWrapper::angleCallback, &servo1);
  auto servo2_angle_sub = new ros::Subscriber<std_msgs::Int16, ServoWrapper>(
      "servo2/angle", &ServoWrapper::angleCallback, &servo2);
  auto servo3_angle_sub = new ros::Subscriber<std_msgs::Int16, ServoWrapper>(
      "servo3/angle", &ServoWrapper::angleCallback, &servo3);
  auto servo4_angle_sub = new ros::Subscriber<std_msgs::Int16, ServoWrapper>(
      "servo4/angle", &ServoWrapper::angleCallback, &servo4);
  auto servo5_angle_sub = new ros::Subscriber<std_msgs::Int16, ServoWrapper>(
      "servo5/angle", &ServoWrapper::angleCallback, &servo5);
  auto servo6_angle_sub = new ros::Subscriber<std_msgs::Int16, ServoWrapper>(
      "servo6/angle", &ServoWrapper::angleCallback, &servo6);

  auto servo1_pwm_sub =
      new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>(
          "servo1/pwm", &ServoWrapper::pwmCallback, &servo1);
  auto servo2_pwm_sub =
      new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>(
          "servo2/pwm", &ServoWrapper::pwmCallback, &servo2);
  auto servo3_pwm_sub =
      new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>(
          "servo3/pwm", &ServoWrapper::pwmCallback, &servo3);
  auto servo4_pwm_sub =
      new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>(
          "servo4/pwm", &ServoWrapper::pwmCallback, &servo4);
  auto servo5_pwm_sub =
      new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>(
          "servo5/pwm", &ServoWrapper::pwmCallback, &servo5);
  auto servo6_pwm_sub =
      new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>(
          "servo6/pwm", &ServoWrapper::pwmCallback, &servo6);

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

  // Services
  auto set_relay1_srv = new ros::ServiceServer<std_srvs::SetBoolRequest,
                                              std_srvs::SetBoolResponse>(
          "core2/set_relay1", &setRelay1Callback);
  auto set_relay2_srv = new ros::ServiceServer<std_srvs::SetBoolRequest,
                                            std_srvs::SetBoolResponse>(
          "core2/set_relay2", &setRelay2Callback);
  auto set_relay3_srv = new ros::ServiceServer<std_srvs::SetBoolRequest,
                                            std_srvs::SetBoolResponse>(
          "core2/set_relay3", &setRelay3Callback);
  auto set_relay4_srv = new ros::ServiceServer<std_srvs::SetBoolRequest,
                                            std_srvs::SetBoolResponse>(
          "core2/set_relay4", &setRelay4Callback);
  auto reset_board_srv =
      new ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
          "core2/reset_board", &resetBoardCallback);
  auto reset_config_srv = new ros::ServiceServer<std_srvs::TriggerRequest,
                                                 std_srvs::TriggerResponse>(
      "core2/reset_config", &resetConfigCallback);
  auto reset_odometry_srv = new ros::ServiceServer<std_srvs::TriggerRequest,
                                                   std_srvs::TriggerResponse>(
      "core2/reset_odometry", &resetOdometryCallback);
  auto firmware_version_srv = new ros::ServiceServer<std_srvs::TriggerRequest,
                                                     std_srvs::TriggerResponse>(
      "core2/get_firmware_version", &getFirmwareCallback);
  auto set_imu_srv = new ros::ServiceServer<std_srvs::SetBoolRequest,
                                            std_srvs::SetBoolResponse>(
      "core2/set_imu", &setImuCallback);
  auto set_gps_srv = new ros::ServiceServer<std_srvs::SetBoolRequest,
                                            std_srvs::SetBoolResponse>(
      "core2/set_gps", &setGpsCallback);
  auto set_debug_srv = new ros::ServiceServer<std_srvs::SetBoolRequest,
                                              std_srvs::SetBoolResponse>(
      "core2/set_debug", &setDebugCallback);

  nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      *set_relay1_srv);
  nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      *set_relay2_srv);
  nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      *set_relay3_srv);
  nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      *set_relay4_srv);
  nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
      *reset_board_srv);
  nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      *reset_config_srv);
  nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      *reset_odometry_srv);
  nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      *firmware_version_srv);
  nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      *set_imu_srv);
  nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      *set_gps_srv);
  nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      *set_debug_srv);

  // IMU
  if (conf.imu_enabled) {
    imu_gyro_pub = new ros::Publisher("imu/gyro", &imu_gyro_msg);
    imu_accel_pub = new ros::Publisher("imu/accel", &imu_accel_msg);
    imu_mag_pub = new ros::Publisher("imu/mag", &imu_mag_msg);
    auto imu_cal_mpu_srv = new ros::ServiceServer<std_srvs::TriggerRequest,
                                                  std_srvs::TriggerResponse>(
        "imu/calibrate_gyro_accel", &calMpuCallback);
    auto imu_cal_mag_srv = new ros::ServiceServer<std_srvs::TriggerRequest,
                                                  std_srvs::TriggerResponse>(
        "imu/calibrate_mag", &calMagCallback);
    nh.advertise(*imu_gyro_pub);
    nh.advertise(*imu_accel_pub);
    nh.advertise(*imu_mag_pub);
    nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
        *imu_cal_mpu_srv);
    nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
        *imu_cal_mag_srv);
  }

  // GPS
  if (conf.gps_enabled) {
    gps_pub = new ros::Publisher("gps_fix", &gps_fix);
    nh.advertise(*gps_pub);
  }
}

void setupRelay() {
  hSens1.pin1.setOut();
  hSens1.pin2.setOut();
  hSens1.pin3.setOut();
  hSens1.pin4.setOut();
}

void setupServos() {
  hServo.enablePower();

  switch (params.servo_voltage) {
    case 0:
      hServo.setVoltage5V();
      break;
    case 1:
      hServo.setVoltage6V();
      break;
    case 2:
      hServo.setVoltage7V4();
      break;
    case 3:
      hServo.setVoltage8V6();
      break;
    default:
      hServo.setVoltage7V4();
  }

  servo1.init(&nh);
  servo2.init(&nh);
  servo3.init(&nh);
  servo4.init(&nh);
  servo5.init(&nh);
  servo6.init(&nh);
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

void setupIMU() {
  IMU_HSENS.selectI2C();
  imu = new IMU(IMU_HSENS.getI2C());
  imu->init();

  imu_gyro_msg.header.frame_id = params.imu_frame_id;
  imu_accel_msg.header.frame_id = params.imu_frame_id;
  imu_mag_msg.header.frame_id = params.imu_frame_id;
}

void setupGPS() {
  GPS_HSENS.selectSerial();
  gps = new GPS(GPS_HSENS.getSerial());
  gps->init();

  gps_fix.header.frame_id = params.gps_frame_id;
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
      pose.header.stamp = odom.header.stamp = nh.now();

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

void imuLoop() {
  uint32_t t = sys.getRefTime();
  const uint32_t dt = 25;
  while (true) {
    imu->update();

    ros::Time stamp = nh.now();

    imu_gyro_msg.header.stamp = stamp;

    imu_gyro_msg.vector.x = imu->gx;
    imu_gyro_msg.vector.y = imu->gy;
    imu_gyro_msg.vector.z = imu->gz;

    imu_accel_msg.header.stamp = stamp;

    imu_accel_msg.vector.x = imu->ax;
    imu_accel_msg.vector.y = imu->ay;
    imu_accel_msg.vector.z = imu->az;

    imu_mag_msg.header.stamp = stamp;

    imu_mag_msg.vector.x = imu->mx;
    imu_mag_msg.vector.y = imu->my;
    imu_mag_msg.vector.z = imu->mz;

    publish_imu = true;

    sys.delaySync(t, dt);
  }
}

void LEDLoop() {
  uint32_t t = sys.getRefTime();
  const uint32_t dt = 500;

  while (true) {
    if (!nh.connected())
      LED.toggle();
    else
      LED.write(true);

    sys.delaySync(t, dt);
  }
}

void GPSLoop() {
  while (true) {
    gps->pollNextMessage();  // Wait for the next GGA message
    const gga &gpgga = gps->getMessage();

    if (!publish_gps) {
      gps_fix.header.stamp = nh.now();
      gps_fix.latitude = gpgga.latitude;
      gps_fix.longitude = gpgga.longitude;

      gps_fix.altitude = gpgga.altitude;

      gps_fix.position_covariance[0] = ((gpgga.hdop) * (gpgga.hdop)) / 2;
      gps_fix.position_covariance[4] = ((gpgga.hdop) * (gpgga.hdop)) / 2;

      publish_gps = true;
    }
  }
}

void relayLoop() {
  uint32_t t = sys.getRefTime();
  const uint32_t dt = 60;

  while (true)
  {
    relay1_status.data = hSens1.pin1.read();
    relay2_status.data = hSens1.pin2.read();
    relay3_status.data = hSens1.pin3.read();
    relay4_status.data = hSens1.pin4.read();

    publish_relay = true;
    
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

  // Load configuration from Persistant storage
  configLoad();

  // Load ROS parameters
  params.load(nh);

  dc.init();

  setupOdom();
  setupServos();
  setupJoints();
  setupRelay();
  initROS();

  sys.setLogDev(&Serial);

  sys.taskCreate(&batteryLoop, 3);
  sys.taskCreate(&odomLoop, 3);
  sys.taskCreate(&jointStatesLoop, 3);
  sys.taskCreate(&relayLoop, 3);

  if (conf.imu_enabled) {
    setupIMU();
    sys.taskCreate(&imuLoop, 3);
  }

  if (conf.gps_enabled) {
    setupGPS();
    sys.taskCreate(&GPSLoop, 3);
  }

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

      if (publish_imu) {
        imu_gyro_pub->publish(&imu_gyro_msg);
        imu_accel_pub->publish(&imu_accel_msg);
        imu_mag_pub->publish(&imu_mag_msg);
        publish_imu = false;
      }

      if (publish_gps) {
        gps_pub->publish(&gps_fix);
        publish_gps = false;
      }

      if (publish_relay)
      {
        relay1_pub->publish(&relay1_status);
        relay2_pub->publish(&relay2_status);
        relay3_pub->publish(&relay3_status);
        relay4_pub->publish(&relay4_status);
        publish_relay = false;
      }
      
    }
  }
}
