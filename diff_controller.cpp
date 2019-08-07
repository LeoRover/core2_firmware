#include "hFramework.h"
#include "wheel.h"

#include "diff_controller.h"
#include "params.h"
#include "utils.h"

DiffController::DiffController(uint32_t input_timeout)
    : input_timeout_(input_timeout)
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

std::vector<float> DiffController::getWheelPositions()
{
    std::vector<float> positions(4);
    positions[0] = 2 * M_PI * wheelFL->getDistance() / ENCODER_RESOLUTION;
    positions[1] = 2 * M_PI * wheelRL->getDistance() / ENCODER_RESOLUTION;
    positions[2] = 2 * M_PI * wheelFR->getDistance() / ENCODER_RESOLUTION;
    positions[3] = 2 * M_PI * wheelRR->getDistance() / ENCODER_RESOLUTION;
    return positions;
}

std::vector<float> DiffController::getWheelVelocities()
{
    std::vector<float> velocities(4);
    velocities[0] = 2 * M_PI * wheelFL->getSpeed() / ENCODER_RESOLUTION;
    velocities[1] = 2 * M_PI * wheelRL->getSpeed() / ENCODER_RESOLUTION;
    velocities[2] = 2 * M_PI * wheelFR->getSpeed() / ENCODER_RESOLUTION;
    velocities[3] = 2 * M_PI * wheelRR->getSpeed() / ENCODER_RESOLUTION;
    return velocities;
}

std::vector<float> DiffController::getWheelEfforts()
{
    std::vector<float> efforts(4);
    efforts[0] = wheelFL->getPower() * 0.1;
    efforts[1] = wheelRL->getPower() * 0.1;
    efforts[2] = wheelFR->getPower() * 0.1;
    efforts[3] = wheelRR->getPower() * 0.1;
    return efforts;
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
        // speed in ticks/sec
        float FL_speed = wheelFL->getSpeed();  
        float RL_speed = wheelRL->getSpeed();
        float FR_speed = wheelFR->getSpeed();
        float RR_speed = wheelRR->getSpeed();

        float L_speed = (FL_speed + RL_speed) / 2.0;
        float R_speed = (FR_speed + RR_speed) / 2.0;

        // velocity in radians per second
        float L_ang_vel = 2 * M_PI * L_speed / ENCODER_RESOLUTION;
        float R_ang_vel = 2 * M_PI * R_speed / ENCODER_RESOLUTION;

        // velocity in meters per second
        float L_lin_vel = L_ang_vel * WHEEL_RADIUS;
        float R_lin_vel = R_ang_vel * WHEEL_RADIUS;

        // linear (m/s) and angular (r/s) velocities of the robot
        lin_vel_ = (L_lin_vel + R_lin_vel) / 2;
        ang_vel_ = (R_lin_vel - L_lin_vel) / ROBOT_WIDTH;
        
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
        Serial.printf("Motor speeds: %f %f %f %f\r\n", 
                      wheelFL->getSpeed(), wheelRL->getSpeed(), 
                      wheelFR->getSpeed(), wheelRR->getSpeed());
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