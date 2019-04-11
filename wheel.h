#ifndef _WHEEL_H_
#define _WHEEL_H_

#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCyclicBuffer.h"

class Wheel {

	hMotor* mot;
	hPIDRegulator vReg;
	bool pol;

	float dNow = 0.0;

	float vTarget = 0.0;
	float vNow = 0.0;
	float vRange = 1000.0;
	float _max_speed = 0;

	bool turnedOn = 1;

  public:
	Wheel(hMotor &motor, bool polarity, float max_speed);
	void begin();

	void update(uint32_t dt);

	void setSpeed(float speed);
	float getSpeed();

    void setPID(float P, float I, float D);
	
	int32_t getDistance();
	void resetDistance();

	void reset();
	void turnOff();
	void turnOn();
};

#endif
