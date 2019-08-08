#ifndef LEO_FIRMWARE_WHEEL_H_
#define LEO_FIRMWARE_WHEEL_H_

#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCyclicBuffer.h"
#include "utils.h"

class Wheel 
{
public:
	Wheel(hMotor &motor, const bool polarity, const float max_speed, 
		  const float kp, const float ki, const float kd, 
		  uint16_t power_limit = 1000, uint16_t torque_limit = 1000,
		  const bool encoder_pullup = false);

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
	CircularBuffer<std::pair<int32_t, uint32_t>> encoder_buffer_;

	int16_t power_;

	bool turned_on_;
	
	int32_t ticks_now_;
	int32_t ticks_sum_;
	uint32_t dt_sum_;

	float v_target_;
	float v_now_;
	
	const bool polarity_;
	const float max_speed_;
	const uint16_t power_limit_;
	const uint16_t torque_limit_;

	static const int encoder_buffer_size_ = 10;
	static constexpr float v_range_ = 1000.0;
};

#endif
