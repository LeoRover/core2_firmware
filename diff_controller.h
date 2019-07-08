#ifndef _DIFF_CONTROLLER_H_
#define _DIFF_CONTROLLER_H_

#include "wheel.h"

#include "ros.h"
#include <vector>

class DiffController
{
public:
    DiffController(uint32_t input_timeout = 0);
    void start();
    void setSpeed(float linear, float angular);
    std::vector<float> getOdom();

private:
    void updateWheelLoop();
    void updateOdometryLoop();
    void debugLoop();
    void inputWatchdog();

    Wheel *wheelFL;
    Wheel *wheelRL;
    Wheel *wheelFR;
    Wheel *wheelRR;

    float _last_wheel_L_ang_pos;
    float _last_wheel_R_ang_pos;
    float _lin_vel;
    float _ang_vel;

    uint32_t _input_timeout;
    uint32_t _last_update;
};

#endif