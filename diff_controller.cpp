#include "hFramework.h"
#include "wheel.h"

#include "diff_controller.h"
#include "params.h"

DiffController::DiffController()
    : _last_wheel_L_ang_pos(0),
      _last_wheel_R_ang_pos(0)
{
    wheelFL = new Wheel(hMotC, 1, WHEEL_MAX_SPEED);
    wheelRL = new Wheel(hMotD, 1, WHEEL_MAX_SPEED);
    wheelFR = new Wheel(hMotA, 0, WHEEL_MAX_SPEED);
    wheelRR = new Wheel(hMotB, 0, WHEEL_MAX_SPEED);
    wheelFL->setPID(PID_P, PID_I, PID_D);
    wheelRL->setPID(PID_P, PID_I, PID_D);
    wheelFR->setPID(PID_P, PID_I, PID_D);
    wheelRR->setPID(PID_P, PID_I, PID_D);
}

void DiffController::start()
{
    wheelFL->begin();
    wheelRL->begin();
    wheelFR->begin();
    wheelRR->begin();
    sys.taskCreate(std::bind(&DiffController::updateWheelLoop, this));
    sys.taskCreate(std::bind(&DiffController::updateOdometryLoop, this));
}

void DiffController::setSpeed(float linear, float angular)
{
    float wheel_L_lin_vel = linear - (angular * ROBOT_WIDTH / 2);
    float wheel_R_lin_vel = linear + (angular * ROBOT_WIDTH / 2);
    float wheel_L_ang_vel = wheel_L_lin_vel / WHEEL_RADIUS;
    float wheel_R_ang_vel = wheel_R_lin_vel / WHEEL_RADIUS;
    float enc_L_speed = ENCODER_RESOLUTION * wheel_L_ang_vel / (2 * M_PI);
    float enc_R_speed = ENCODER_RESOLUTION * wheel_R_ang_vel / (2 * M_PI);

    //Serial.printf("[DiffController] linear: %f m/s; angular: %f r/s; enc speed: %f t/s\r\n", 
    //            wheel_L_lin_vel, wheel_L_angular_velocity, enc_L_speed);

    wheelFL->setSpeed(enc_L_speed);
    wheelRL->setSpeed(enc_L_speed);
    wheelFR->setSpeed(enc_R_speed);
    wheelRR->setSpeed(enc_R_speed);
}

std::vector<float> DiffController::getOdom()
{
    std::vector<float> odom;
    odom.push_back(_lin_vel);
    odom.push_back(_ang_vel);
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
        float wheel_L_ang_vel = (wheel_L_ang_pos - _last_wheel_L_ang_pos) / (dt / 1000.0);
        float wheel_R_ang_vel = (wheel_R_ang_pos - _last_wheel_R_ang_pos) / (dt / 1000.0);

        _last_wheel_L_ang_pos = wheel_L_ang_pos;
        _last_wheel_R_ang_pos = wheel_R_ang_pos;

        // velocity in meters per second
        float wheel_L_lin_vel = wheel_L_ang_vel * WHEEL_RADIUS;
        float wheel_R_lin_vel = wheel_R_ang_vel * WHEEL_RADIUS;

        // linear (m/s) and angular (r/s) velocities of the robot
        _lin_vel = (wheel_L_lin_vel + wheel_R_lin_vel) / 2;
        _ang_vel = (wheel_R_lin_vel - wheel_L_lin_vel) / ROBOT_WIDTH;
        
        sys.delaySync(t, dt);
    }
}