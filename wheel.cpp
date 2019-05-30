#include "wheel.h"

Wheel::Wheel(hMotor& motor, bool polarity, float max_speed,
			 float Kp, float Ki, float Kd, int32_t power_limit = 1000) 
	: mot(motor),
	  pol(polarity),
	  _max_speed(max_speed),
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

	if (turnedOn == true) {
		if (vNow == 0.0 && vTarget == 0.0){
			vReg.reset();
			mot.setPower(0);
		} else {
			mot.setPower(pidOut);
		}
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