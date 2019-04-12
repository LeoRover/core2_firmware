#include "hFramework.h"
#include "wheel.h"

#include "diff_controller.h"
#include "params.h"

DiffController::DiffController()
    : _last_wheel_L_ang_pos(0),
      _last_wheel_R_ang_pos(0)
{
    wheelFL = new Wheel(hMotC, 1, WHEEL_MAX_SPEED, PID_P, PID_I, PID_D);
    wheelRL = new Wheel(hMotD, 1, WHEEL_MAX_SPEED, PID_P, PID_I, PID_D);
    wheelFR = new Wheel(hMotA, 0, WHEEL_MAX_SPEED, PID_P, PID_I, PID_D);
    wheelRR = new Wheel(hMotB, 0, WHEEL_MAX_SPEED, PID_P, PID_I, PID_D);
}

void DiffController::start()
{
    sys.taskCreate(std::bind(&DiffController::updateWheelLoop, this));
    sys.taskCreate(std::bind(&DiffController::updateOdometryLoop, this));
}

float clamp(float value, float limit)
{
    if (value > limit) 
        return limit;
    else if (value < -limit)
        return -limit;
    else
        return value;
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