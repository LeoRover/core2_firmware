#include "wheel.h"

Wheel::Wheel(hMotor& motor, bool polarity, float max_speed)
{
	mot = &motor;
	pol = polarity;
	_max_speed = max_speed;
}

void Wheel::begin()
{
	vReg.setScale(1);
	vReg.setRange(-vRange, vRange);

	if (pol) {
		mot->setMotorPolarity(Polarity::Reversed);
		mot->setEncoderPolarity(Polarity::Reversed);
	}

	mot->resetEncoderCnt();
}

void Wheel::update(uint32_t dt)
{
	float dPrev = dNow;
	dNow = (float)mot->getEncoderCnt();

	vNow = (dNow - dPrev) / (dt * 0.001);

	float vErr = vNow - vTarget;
	float pidOut = vReg.update(vErr, dt);

	if (turnedOn == true) {
		if (vNow == 0.0 && vTarget == 0.0){
			vReg.reset();
			mot->setPower(0);
		} else {
			mot->setPower(pidOut);
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

void Wheel::setPID(float P, float I, float D)
{
	vReg.setCoeffs(P, I, D);
}

int32_t Wheel::getDistance()
{
	return dNow;
}

void Wheel::resetDistance()
{
	mot->resetEncoderCnt();
}

void Wheel::reset()
{
	mot->resetEncoderCnt();
	vReg.reset();
	vNow = 0;
	vTarget = 0;
	mot->setPower(0);
}

void Wheel::turnOff()
{
	turnedOn = false;
	mot->setPower(0);
}
void Wheel::turnOn()
{
	turnedOn = true;
}