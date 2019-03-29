#include "hFramework.h"
#include "wheel.h"

#include "diff_controller.h"
#include "params.h"

DiffController::DiffController()
{
    wheelFL = new Wheel(hMotC, 1, WHEEL_MAX_SPEED);
    wheelRL = new Wheel(hMotD, 1, WHEEL_MAX_SPEED);
    wheelFR = new Wheel(hMotA, 0, WHEEL_MAX_SPEED);
    wheelRR = new Wheel(hMotB, 0, WHEEL_MAX_SPEED);
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
    float L_wheel_lin_speed = linear - (angular * ROBOT_WIDTH / 2);
    float R_wheel_lin_speed = linear + (angular * ROBOT_WIDTH / 2);
    float L_wheel_angular_velocity = L_wheel_lin_speed / WHEEL_RADIUS;
    float R_wheel_angular_velocity = R_wheel_lin_speed / WHEEL_RADIUS;
    float L_enc_speed = ENCODER_RESOLUTION * L_wheel_angular_velocity / (2 * M_PI);
    float R_enc_speed = ENCODER_RESOLUTION * R_wheel_angular_velocity / (2 * M_PI);

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