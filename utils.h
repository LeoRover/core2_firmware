#ifndef LEO_FIRMWARE_UTILS_H_
#define LEO_FIRMWARE_UTILS_H_

#include "hFramework.h"

#include "ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16MultiArray.h"

#include "params.h"

inline float clamp(float value, float limit)
{
    if (value > limit) 
        return limit;
    else if (value < -limit)
        return -limit;
    else
        return value;
}

template<class T>
class CircularBuffer
{
	T *values_;
	size_t size_;
	size_t iter_;

public:
	CircularBuffer(uint16_t size)
        : size_(size),
          iter_(0),
          values_(new T[size])
	{ }

	T push_back(T val)
	{
		T tmp = values_[iter_];
		values_[iter_++] = val;
		if (iter_ >= size_)
			iter_ = 0;
		return tmp;
	}
};

class ServoWrapper
{
	int num;
	int per;
	IServo& servo;

public:
	ServoWrapper(int num, IServo& servo)
		: num(num),
		  servo(servo) {}

	void angleCallback(const std_msgs::Int16& msg)
	{
		if (per!=SERVO_PERIOD)
		{
			servo.setPeriod(SERVO_PERIOD);
			per=SERVO_PERIOD;
		}
		servo.rotAbs(msg.data);
#ifdef DEBUG
		Serial.printf("[servo%dAngleCallback] angle: %d\r\n", num, msg.data);
#endif
	}

	void pwmCallback(const std_msgs::UInt16MultiArray& msg)
	{
		if (msg.data_length >= 2){
			per=msg.data[0];
			servo.setPeriod(msg.data[0]);
			servo.setWidth(msg.data[1]);
#ifdef DEBUG
			Serial.printf("[servo%dPWMCallback] period: %d width: %d\r\n", num, msg.data[0], msg.data[1]);
		} else {
			Serial.printf("ERROR: [servo%dPWMCallback] data array should have 2 members\r\n", num);
#endif
		}
	}
};


#endif