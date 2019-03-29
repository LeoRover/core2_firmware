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
    Wheel *wheelRL;
    Wheel *wheelFR;
    Wheel *wheelRR;
};

#endif