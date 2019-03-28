#include "hFramework.h"
#include "diff_controller.h"
#include "wheel.h"

DiffController::DiffController()
{
    wheelFL = new Wheel(hMotC, 1, wheel_max_speed);
    wheelRL = new Wheel(hMotD, 1, wheel_max_speed);
    wheelFR = new Wheel(hMotA, 0, wheel_max_speed);
    wheelRR = new Wheel(hMotB, 0, wheel_max_speed);
}

void DiffController::start()
{
    wheelFL->begin();
    wheelRL->begin();
    wheelFR->begin();
    wheelRR->begin();
    sys.taskCreate(std::bind(&DiffController::updateLoop, this));
}

void DiffController::setSpeed(float linear, float angular)
{
    float L_wheel_lin_speed = linear - (angular * robot_width / 2);
    float R_wheel_lin_speed = linear + (angular * robot_width / 2);
    float L_wheel_angular_velocity = L_wheel_lin_speed / wheel_radius;
    float R_wheel_angular_velocity = R_wheel_lin_speed / wheel_radius;
    float L_enc_speed = enc_res * L_wheel_angular_velocity / (2 * M_PI);
    float R_enc_speed = enc_res * R_wheel_angular_velocity / (2 * M_PI);

    //Serial.printf("[DiffController] linear speed: %f m/s; angular velocity: %f r/s; enc speed: %f t/s\r\n", 
    //            L_wheel_lin_speed, L_wheel_angular_velocity, L_enc_speed);

    wheelFL->setSpeed(L_enc_speed);
    wheelRL->setSpeed(L_enc_speed);
    wheelFR->setSpeed(R_enc_speed);
    wheelRR->setSpeed(R_enc_speed);
}

void DiffController::updateLoop()
{
    uint32_t t = sys.getRefTime();
    long dt = 10;
    while(true)
    {
        wheelFL->update(dt);
        wheelRL->update(dt);
        wheelFR->update(dt);
        wheelRR->update(dt);
        sys.delaySync(t, dt);
    }
}