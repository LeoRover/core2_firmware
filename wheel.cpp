#include "wheel.h"
#include "utils.h"

Wheel::Wheel(hMotor& motor, bool polarity, float max_speed,
			 float Kp, float Ki, float Kd, 
			 uint16_t power_limit = 1000, uint16_t torque_limit = 1000) 
	: mot(motor),
	  pol(polarity),
	  _max_speed(max_speed),
	  _power_limit(power_limit),
	  _torque_limit(torque_limit),
	  turnedOn(true),
	  dNow(0.0),
	  vNow(0.0),
	  vTarget(0.0)
{
	vReg.setScale(1);
	vReg.setRange(-vRange, vRange);
	vReg.setIRange(-vRange, vRange);
	vReg.setCoeffs(Kp, Ki, Kd);

	if (pol) {
		mot.setMotorPolarity(Polarity::Reversed);
		mot.setEncoderPolarity(Polarity::Reversed);
	}

	mot.setPowerLimit(power_limit);
	mot.resetEncoderCnt();
}

void Wheel::update(uint32_t dt)
{
	float dPrev = dNow;
	dNow = (float)mot.getEncoderCnt();

	vNow = (dNow - dPrev) / (dt * 0.001);

	float vErr = vNow - vTarget;
	float pidOut = vReg.update(vErr, dt);

	float est_power = (std::abs(vNow) / _max_speed) * 1000.0;
	float max_power = std::min(est_power + static_cast<float>(_torque_limit), (float)1000.0);

	if (turnedOn == true) {
		if (vNow == 0.0 && vTarget == 0.0){
			vReg.reset();
			_power = 0;
		} else {
			float power_limited = clamp(pidOut, max_power);
			_power = static_cast<int16_t>(power_limited);
		}
		mot.setPower(_power);
	}
}

void Wheel::setSpeed(float speed)
{
	if (speed > _max_speed) {
		vTarget = _max_speed;
	} 
	else if (speed < -_max_speed) {
		vTarget = -_max_speed;
	} 
	else {
		vTarget = speed;
	}
}

float Wheel::getSpeed()
{
	return vNow;
}

int16_t Wheel::getPower()
{
	return _power;
}

int32_t Wheel::getDistance()
{
	return dNow;
}

void Wheel::resetDistance()
{
	mot.resetEncoderCnt();
}

void Wheel::reset()
{
	mot.resetEncoderCnt();
	vReg.reset();
	vNow = 0;
	vTarget = 0;
	mot.setPower(0);
}

void Wheel::turnOff()
{
	turnedOn = false;
	mot.setPower(0);
}
void Wheel::turnOn()
{
	turnedOn = true;
}