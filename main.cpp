#include "hFramework.h"
// #include "hCloudClient.h"
#include "hMutex.h"

#include "ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_srvs/Trigger.h"

#include "diff_drive_controller.h"
#include "params.h"
#include "utils.h"
#include "config.h"

#include "sensors/imu.h"
#include "sensors/gps.h"


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

IMU* imu;
geometry_msgs::Vector3Stamped imu_gyro_msg;
ros::Publisher *imu_gyro_pub;
geometry_msgs::Vector3Stamped imu_accel_msg;
ros::Publisher *imu_accel_pub;
geometry_msgs::Vector3Stamped imu_mag_msg;
ros::Publisher *imu_mag_pub;
bool publish_imu = false;

GPS* gps;
sensor_msgs::NavSatFix gps_fix;
ros::Publisher *gps_pub;
bool publish_gps = false;

ros::Subscriber<geometry_msgs::Twist> *twist_sub;

ros::Subscriber<std_msgs::Empty> *reset_board_sub;
ros::Subscriber<std_msgs::Empty> *reset_config_sub;
ros::Subscriber<std_msgs::Bool> *set_imu_sub;
ros::Subscriber<std_msgs::Bool> *set_gps_sub;

ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>
    *imu_cal_mpu_srv;
ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>
    *imu_cal_mag_srv;

DiffDriveController *dc;

ServoWrapper servo1(1, hServo.servo1);
ServoWrapper servo2(2, hServo.servo2);
ServoWrapper servo3(3, hServo.servo3);
ServoWrapper servo4(4, hServo.servo4);
ServoWrapper servo5(5, hServo.servo5);
ServoWrapper servo6(6, hServo.servo6);

void cmdVelCallback(const geometry_msgs::Twist &msg) {
    dc->setSpeed(msg.linear.x, msg.angular.z);
#ifdef DEBUG
    Serial.printf("[cmdVelCallback] linear: %f angular %f\r\n", msg.linear.x,
        msg.angular.z);
#endif
}

void resetBoardCallback(const std_msgs::Empty &msg) {
    sys.reset();
}

void resetConfigCallback(const std_msgs::Empty &msg) {
    reset_config();
}

void setImuCallback(const std_msgs::Bool &msg) {
    conf.imu_enabled = msg.data;
    store_config();
}

void setGpsCallback(const std_msgs::Bool &msg) {
    conf.gps_enabled = msg.data;
    store_config();
}

void calMpuCallback(const std_srvs::TriggerRequest &req,
    std_srvs::TriggerResponse &res) {
    imu->calGyroAccel();
    res.message = "Succesfully calibrated gyroscope and accelerometer biases";
    res.success = true;
}

void calMagCallback(const std_srvs::TriggerRequest& req,
    std_srvs::TriggerResponse& res) {
    imu->calMag();
    res.message = "Succesfully calibrated magnetometer";
    res.success = true;
}



void initROS() {
    battery_pub = new ros::Publisher("/battery", &battery);
    odom_pub = new ros::Publisher("/wheel_odom", &odom);
    joint_states_pub = new ros::Publisher("/joint_states", &joint_states);

    twist_sub = new ros::Subscriber<geometry_msgs::Twist>("cmd_vel",
        &cmdVelCallback);

    reset_board_sub = new ros::Subscriber<std_msgs::Empty>("core2/reset_board",
        &resetBoardCallback);
    reset_config_sub = new ros::Subscriber<std_msgs::Empty>(
        "core2/reset_config", &resetConfigCallback);
    set_imu_sub = new ros::Subscriber<std_msgs::Bool>("core2/set_imu",
        &setImuCallback);
    set_gps_sub = new ros::Subscriber<std_msgs::Bool>("core2/set_gps",
        &setGpsCallback);

    ros::Subscriber<std_msgs::Int16, ServoWrapper> *servo1_angle_sub =
        new ros::Subscriber<std_msgs::Int16, ServoWrapper>("servo1/angle",
            &ServoWrapper::angleCallback, &servo1);
    ros::Subscriber<std_msgs::Int16, ServoWrapper> *servo2_angle_sub =
        new ros::Subscriber<std_msgs::Int16, ServoWrapper>("servo2/angle",
            &ServoWrapper::angleCallback, &servo2);
    ros::Subscriber<std_msgs::Int16, ServoWrapper> *servo3_angle_sub =
        new ros::Subscriber<std_msgs::Int16, ServoWrapper>("servo3/angle",
            &ServoWrapper::angleCallback, &servo3);
    ros::Subscriber<std_msgs::Int16, ServoWrapper> *servo4_angle_sub =
        new ros::Subscriber<std_msgs::Int16, ServoWrapper>("servo4/angle",
            &ServoWrapper::angleCallback, &servo4);
    ros::Subscriber<std_msgs::Int16, ServoWrapper> *servo5_angle_sub =
        new ros::Subscriber<std_msgs::Int16, ServoWrapper>("servo5/angle",
            &ServoWrapper::angleCallback, &servo5);
    ros::Subscriber<std_msgs::Int16, ServoWrapper> *servo6_angle_sub =
        new ros::Subscriber<std_msgs::Int16, ServoWrapper>("servo6/angle",
            &ServoWrapper::angleCallback, &servo6);

    ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper> *servo1_pwm_sub =
        new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>(
            "servo1/pwm", &ServoWrapper::pwmCallback, &servo1);
    ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper> *servo2_pwm_sub =
        new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>(
            "servo2/pwm", &ServoWrapper::pwmCallback, &servo2);
    ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper> *servo3_pwm_sub =
        new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>(
            "servo3/pwm", &ServoWrapper::pwmCallback, &servo3);
    ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper> *servo4_pwm_sub =
        new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>(
            "servo4/pwm", &ServoWrapper::pwmCallback, &servo4);
    ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper> *servo5_pwm_sub =
        new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>(
            "servo5/pwm", &ServoWrapper::pwmCallback, &servo5);
    ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper> *servo6_pwm_sub =
        new ros::Subscriber<std_msgs::UInt16MultiArray, ServoWrapper>(
            "servo6/pwm", &ServoWrapper::pwmCallback, &servo6);


    nh.advertise(*battery_pub);
    nh.advertise(*odom_pub);
    nh.advertise(*joint_states_pub);
    nh.subscribe(*twist_sub);
    nh.subscribe(*reset_board_sub);
    nh.subscribe(*reset_config_sub);
    nh.subscribe(*set_imu_sub);
    nh.subscribe(*set_gps_sub);
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

    if (conf.imu_enabled) {
        imu_gyro_pub = new ros::Publisher("/imu/gyro", &imu_gyro_msg);
        imu_accel_pub = new ros::Publisher("/imu/accel", &imu_accel_msg);
        imu_mag_pub = new ros::Publisher("/imu/mag", &imu_mag_msg);
        imu_cal_mpu_srv = new ros::ServiceServer<std_srvs::TriggerRequest,
            std_srvs::TriggerResponse>("imu/calibrate_gyro_accel",
            &calMpuCallback);
        imu_cal_mag_srv = new ros::ServiceServer<std_srvs::TriggerRequest,
            std_srvs::TriggerResponse>("imu/calibrate_mag", &calMagCallback);
        nh.advertise(*imu_gyro_pub);
        nh.advertise(*imu_accel_pub);
        nh.advertise(*imu_mag_pub);
        nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>
            (*imu_cal_mpu_srv);
        nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>
            (*imu_cal_mag_srv);
    }

    if (conf.gps_enabled) {
        gps_pub = new ros::Publisher("/gps_fix", &gps_fix);
        nh.advertise(*gps_pub);
    }
}

void setupServos() {
    hServo.enablePower();
    hServo.setPeriod(SERVO_PERIOD);

    switch (SERVO_VOLTAGE) {
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

void setupJoints() {
    joint_states.header.frame_id = "base_link";
    joint_states.name = new char*[4] {
        "wheel_FL_joint", "wheel_RL_joint",
        "wheel_FR_joint", "wheel_RR_joint"
    };
    joint_states.position = new float[4];
    joint_states.velocity = new float[4];
    joint_states.effort = new float[4];
    joint_states.name_length = 4;
    joint_states.position_length = 4;
    joint_states.velocity_length = 4;
    joint_states.effort_length = 4;
}

void setupImu() {
    IMU_HSENS.selectI2C();
    imu = new IMU(IMU_HSENS.getI2C());
    imu->begin();
    imu_gyro_msg.header.frame_id = "imu";
    imu_accel_msg.header.frame_id = "imu";
    imu_mag_msg.header.frame_id = "imu";
}

void setupOdom() {
    odom.header.frame_id = "base_link";
}

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

            std::vector<float> odo = dc->getOdom();
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
            std::vector<float> pos = dc->getWheelPositions();
            std::vector<float> vel = dc->getWheelVelocities();
            std::vector<float> eff = dc->getWheelEfforts();

            joint_states.header.stamp = nh.now();

            std::copy(pos.begin(), pos.end(), joint_states.position);
            std::copy(vel.begin(), vel.end(), joint_states.velocity);
            std::copy(eff.begin(), eff.end(), joint_states.effort);

            publish_joint = true;
        }

        sys.delaySync(t, dt);
    }
}

void imuLoop() {
    uint32_t t = sys.getRefTime();
    uint32_t dt = 25;
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
    uint32_t dt = 250;

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
        gps->receive_next_msg();  // Wait for next GGA message

        if (!publish_gps) {
            gps_fix.header.stamp = nh.now();
            gps_fix.header.frame_id = "/gps";
            gps_fix.latitude = gps->gpgga.latitude;
            gps_fix.longitude = gps->gpgga.longitude;

            gps_fix.altitude = gps->gpgga.altitude;

            gps_fix.position_covariance[0] = ((gps->gpgga.hdop) *
                (gps->gpgga.hdop))/2;
            gps_fix.position_covariance[4] = ((gps->gpgga.hdop) *
                (gps->gpgga.hdop))/2;

            publish_gps = true;
        }
    }
}

void hMain() {
    uint32_t t = sys.getRefTime();
    // platform.begin(&RPi);
    // nh.getHardware()->initWithDevice(&platform.LocalSerial);
    RPi.setBaudrate(250000);
    nh.getHardware()->initWithDevice(&RPi);
    nh.initNode();

    dc = new DiffDriveController(INPUT_TIMEOUT);
    dc->start();

    load_config();

    setupOdom();
    setupServos();
    setupJoints();
    initROS();

    sys.setLogDev(&Serial);

    LED.setOut();
    sys.taskCreate(&LEDLoop);

    sys.taskCreate(&batteryLoop);
    sys.taskCreate(&odomLoop);
    sys.taskCreate(&jointStatesLoop);

    if (conf.imu_enabled) {
        setupImu();
        sys.taskCreate(&imuLoop);
    }

    if (conf.gps_enabled) {
        gps = new GPS;
        gps->begin();
        sys.taskCreate(&GPSLoop);
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
        }

        sys.delaySync(t, 1);
    }
}
