#include "hFramework.h"
#include "diff_controller.h"
#include "wheel.h"

DiffController::DiffController()
{
    wheelFL = new Wheel(hMotA, 0, wheel_max_speed);
}

void DiffController::start()
{
    wheelFL->begin();
    sys.taskCreate(std::bind(&DiffController::updateLoop, this));
}

void DiffController::setSpeed(float linear, float angular)
{
    float L_wheel_lin_speed = linear - (angular * robot_width / 2);
    float L_wheel_angular_velocity = L_wheel_lin_speed / wheel_radius;
    float L_enc_speed = enc_res * L_wheel_angular_velocity / (2 * M_PI);

    wheelFL->setSpeed(L_enc_speed);
}

void DiffController::updateLoop()
{
    uint32_t t = sys.getRefTime();
    long dt = 10;
    while(true)
    {
        wheelFL->update(dt);
        sys.delaySync(t, dt);
    }
}