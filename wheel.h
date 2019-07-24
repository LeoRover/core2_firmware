#ifndef LEO_FIRMWARE_WHEEL_H_
#define LEO_FIRMWARE_WHEEL_H_

#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCyclicBuffer.h"

class Wheel 
{
public:
	Wheel(hMotor &motor, bool polarity, float max_speed, 
		  float kp, float ki, float kd, 
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

private:
	hMotor& motor_;
	hPIDRegulator v_reg_;

	int16_t power_;

	bool turned_on_;
	float d_now_;
	float v_target_;
	float v_now_;
	
	const bool polarity_;
	const float max_speed_;
	const uint16_t power_limit_;
	const uint16_t torque_limit_;
	static constexpr float v_range_ = 1000.0;
};

#endif
