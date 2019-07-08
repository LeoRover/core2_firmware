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
	int16_t _power;
	uint16_t _power_limit;
	uint16_t _torque_limit;
	float _max_speed;

	bool turnedOn;
	float dNow;
	float vTarget;
	float vNow;
	
	float vRange = 1000.0;

public:
	Wheel(hMotor &motor, bool polarity, float max_speed, 
		  float Kp, float Ki, float Kd, 
		  uint16_t power_limit, uint16_t torque_limit);

	void update(uint32_t dt);

	void setSpeed(float speed);
	float getSpeed();
	int16_t getPower();
	
	int32_t getDistance();
	void resetDistance();

	void reset();
	void turnOff();
	void turnOn();
};

#endif
