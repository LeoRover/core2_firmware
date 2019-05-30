#ifndef _WHEEL_H_
#define _WHEEL_H_

#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCyclicBuffer.h"

class Wheel {

	hMotor& mot;
	hPIDRegulator vReg;

	bool pol;
	float _max_speed;

	bool turnedOn;
	float dNow;
	float vTarget;
	float vNow;
	
	float vRange = 1000.0;

public:
	Wheel(hMotor &motor, bool polarity, float max_speed, float Kp, float Ki, float Kd, int32_t power_limit);

	void update(uint32_t dt);

	void setSpeed(float speed);
	float getSpeed();
	
	int32_t getDistance();
	void resetDistance();

	void reset();
	void turnOff();
	void turnOn();
};

#endif
