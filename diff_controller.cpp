#include "hFramework.h"
#include "wheel.h"

#include "diff_controller.h"
#include "params.h"
#include "utils.h"

DiffController::DiffController(uint32_t input_timeout)
    : last_wheel_L_ang_pos_(0),
      last_wheel_R_ang_pos_(0),
      input_timeout_(input_timeout)
{
    wheelFL = new Wheel(hMotC, 1, WHEEL_MAX_SPEED, PID_P, PID_I, PID_D, POWER_LIMIT, TORQUE_LIMIT);
    wheelRL = new Wheel(hMotD, 1, WHEEL_MAX_SPEED, PID_P, PID_I, PID_D, POWER_LIMIT, TORQUE_LIMIT);
    wheelFR = new Wheel(hMotA, 0, WHEEL_MAX_SPEED, PID_P, PID_I, PID_D, POWER_LIMIT, TORQUE_LIMIT);
    wheelRR = new Wheel(hMotB, 0, WHEEL_MAX_SPEED, PID_P, PID_I, PID_D, POWER_LIMIT, TORQUE_LIMIT);
}

void DiffController::start()
{
    sys.taskCreate(std::bind(&DiffController::updateWheelLoop, this));
    sys.taskCreate(std::bind(&DiffController::updateOdometryLoop, this));
    if (input_timeout_ > 0.0) {
        last_update_ = sys.getRefTime();
        sys.taskCreate(std::bind(&DiffController::inputWatchdog, this));
    }
#ifdef DEBUG
    sys.taskCreate(std::bind(&DiffController::debugLoop, this));
#endif
}

void DiffController::setSpeed(float linear, float angular)
{
    float wheel_L_lin_vel = linear - (angular * ROBOT_WIDTH / 2);
    float wheel_R_lin_vel = linear + (angular * ROBOT_WIDTH / 2);
    float wheel_L_ang_vel = wheel_L_lin_vel / WHEEL_RADIUS;
    float wheel_R_ang_vel = wheel_R_lin_vel / WHEEL_RADIUS;
    float enc_L_speed = clamp(ENCODER_RESOLUTION * wheel_L_ang_vel / (2 * M_PI), WHEEL_MAX_SPEED);
    float enc_R_speed = clamp(ENCODER_RESOLUTION * wheel_R_ang_vel / (2 * M_PI), WHEEL_MAX_SPEED);

    wheelFL->setSpeed(enc_L_speed);
    wheelRL->setSpeed(enc_L_speed);
    wheelFR->setSpeed(enc_R_speed);
    wheelRR->setSpeed(enc_R_speed);

    if (input_timeout_ > 0.0)
        last_update_ = sys.getRefTime();
}

std::vector<float> DiffController::getOdom()
{
    std::vector<float> odom;
    odom.push_back(lin_vel_);
    odom.push_back(ang_vel_);
    return odom;
}

void DiffController::updateWheelLoop()
{
    uint32_t t = sys.getRefTime();
    uint32_t dt = 10;
    while(true)
    {
        wheelFL->update(dt);
        wheelRL->update(dt);
        wheelFR->update(dt);
        wheelRR->update(dt);
        sys.delaySync(t, dt);
    }
}

void DiffController::updateOdometryLoop()
{
    uint32_t t = sys.getRefTime();
    uint32_t dt = 10;

    while(true)
    {
        // distance in tics
        float enc_FL = wheelFL->getDistance();  
        float enc_RL = wheelRL->getDistance();
        float enc_FR = wheelFR->getDistance();
        float enc_RR = wheelRR->getDistance();

        float enc_L = (enc_FL + enc_RL) / 2;
        float enc_R = (enc_FR + enc_RR) / 2;

        // distance in radians
        float wheel_L_ang_pos = 2 * M_PI * enc_L / ENCODER_RESOLUTION;
        float wheel_R_ang_pos = 2 * M_PI * enc_R / ENCODER_RESOLUTION;

        // velocity in radians per second
        float wheel_L_ang_vel = (wheel_L_ang_pos - last_wheel_L_ang_pos_) / (dt / 1000.0);
        float wheel_R_ang_vel = (wheel_R_ang_pos - last_wheel_R_ang_pos_) / (dt / 1000.0);

        last_wheel_L_ang_pos_ = wheel_L_ang_pos;
        last_wheel_R_ang_pos_ = wheel_R_ang_pos;

        // velocity in meters per second
        float wheel_L_lin_vel = wheel_L_ang_vel * WHEEL_RADIUS;
        float wheel_R_lin_vel = wheel_R_ang_vel * WHEEL_RADIUS;

        // linear (m/s) and angular (r/s) velocities of the robot
        lin_vel_ = (wheel_L_lin_vel + wheel_R_lin_vel) / 2;
        ang_vel_ = (wheel_R_lin_vel - wheel_L_lin_vel) / ROBOT_WIDTH;
        
        sys.delaySync(t, dt);
    }
}

void DiffController::debugLoop()
{
    uint32_t t = sys.getRefTime();
    uint32_t dt = 100;

    while(true)
    {
        Serial.printf("Motor powers: %d %d %d %d\r\n", 
                      wheelFL->getPower(), wheelRL->getPower(), 
                      wheelFR->getPower(), wheelRR->getPower());
        sys.delaySync(t, dt);
    }
}

void DiffController::inputWatchdog()
{
    while (true)
    {
        while (sys.getRefTime() < last_update_ + input_timeout_)
            sys.delay(last_update_ + input_timeout_ - sys.getRefTime() + 1);
        
        setSpeed(0.0, 0.0);
    }
}