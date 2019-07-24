#ifndef LEO_FIRMWARE_DIFF_CONTROLLER_H_
#define LEO_FIRMWARE_DIFF_CONTROLLER_H_

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

    float last_wheel_L_ang_pos_;
    float last_wheel_R_ang_pos_;
    float lin_vel_;
    float ang_vel_;

    uint32_t input_timeout_;
    uint32_t last_update_;
};

#endif