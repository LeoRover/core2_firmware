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

    Wheel *wheelFL;

    //constants
    int enc_res = 8256;
    int wheel_radius = 0.06;
    int wheel_max_speed = 6000;
    int robot_width = 0.33;
};

#endif