#include "hFramework.h"
#include "imu/MPU9250.h"
#include "imu/RegisterMap.h"

#include "imu.h"

static const float gravitationalAcceleration = 9.80665;
static const float degreeToRadian = 2.0 * M_PI / 360.0;
static const float mGuassToGauss = 0.001;

void IMU::begin()
{
	mpu_.begin(100000);

	uint8_t c = mpu_.getMPU9250ID();
	
	if (c != 0x71) 
		Serial.printf("ERROR: [IMU] MPU9250 WHO_AM_I register is 0x%02x, should be 0x71\r\n", c);

	c = mpu_.getAK8963CID();

	if (c != 0x48)
		Serial.printf("ERROR: [IMU] AK8963 WHO_AM_I register is 0x%02x, should be 0x48\r\n", c);

	mpu_.initMPU9250(AFS_4G, GFS_1000DPS, 0x02);
	mpu_.initAK8963Slave(MFS_14BITS, M_100Hz, magCal_);
	Serial.printf("[IMU] Magnetometer calibration data: %f %f %f\r\n", magCal_[0], magCal_[1], magCal_[2]);

	ares_ = mpu_.getAres(AFS_4G) * gravitationalAcceleration;
	gres_ = mpu_.getGres(GFS_1000DPS) * degreeToRadian;
	mres_ = mpu_.getMres(MFS_14BITS) * mGuassToGauss;
}

void IMU::update()
{
	int16_t accel[3];
	int16_t gyro[3];
	int16_t mag[3];
	
	if (mpu_.checkNewAccelGyroData())
	{
		mpu_.readAccelData(accel);
		ax = (float)accel[0] * ares_;
		ay = (float)accel[1] * ares_;
		az = (float)accel[2] * ares_;

		mpu_.readGyroData(gyro);
		gx = (float)gyro[0] * gres_;
		gy = (float)gyro[1] * gres_;
		gz = (float)gyro[2] * gres_;
	}

	// if (mpu_.checkNewMagData())
	// {
		mpu_.readMagData(mag);
		mx = (float)mag[0] * mres_;
		my = (float)mag[1] * mres_;
		mz = (float)mag[2] * mres_;
	// }
}
