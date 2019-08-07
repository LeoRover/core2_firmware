#ifndef LEO_FIRMWARE_DIFF_CONTROLLER_H_
#define LEO_FIRMWARE_DIFF_CONTROLLER_H_

#include <vector>

#include "wheel.h"

class DiffController
{
public:
    DiffController(uint32_t input_timeout = 0);
    void start();
    void setSpeed(float linear, float angular);
    std::vector<float> getOdom();
    std::vector<float> getWheelPositions();
    std::vector<float> getWheelVelocities();
    std::vector<float> getWheelEfforts();

private:
    void updateWheelLoop();
    void updateOdometryLoop();
    void debugLoop();
    void inputWatchdog();

    Wheel *wheelFL;
    Wheel *wheelRL;
    Wheel *wheelFR;
    Wheel *wheelRR;

    float lin_vel_;
    float ang_vel_;

    uint32_t input_timeout_;
    uint32_t last_update_;
};

#endif