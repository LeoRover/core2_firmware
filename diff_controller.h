#ifndef _DIFF_CONTROLLER_H_
#define _DIFF_CONTROLLER_H_

#include "wheel.h"

class DiffController
{
public:
    DiffController();
    void start();
    void setSpeed(float linear, float angular);

private:
    void updateLoop();

    Wheel *wheelFL, *wheelRL, *wheelFR, *wheelRR;

    //constants
    float enc_res = 8256;
    float wheel_radius = 0.06;
    float wheel_max_speed = 6500;
    float robot_width = 0.33;
};

#endif