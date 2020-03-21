#include "hFramework.h"

#include "sensors/imu.h"
#include "sensors/imu/MPU9250.h"
#include "sensors/imu/RegisterMap.h"

#include "config.h"

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

	mpu_.initMPU9250(AFS_4G, GFS_500DPS, 0x02);
	mpu_.initAK8963Slave(MFS_14BITS, M_100Hz, magCal_);
	Serial.printf("[IMU] Magnetometer calibration data: %f %f %f\r\n", magCal_[0], magCal_[1], magCal_[2]);

	ares_ = mpu_.getAres(AFS_4G);
	gres_ = mpu_.getGres(GFS_500DPS);
	mres_ = mpu_.getMres(MFS_14BITS);

	ax = ay = az = 0.0;
	gx = gy = gz = 0.0;
	qx = qy = qz = qw = 0.0;
	
	// Load config
	for (int i = 0; i < 3; ++i)
	{
		abias_[i] = conf.accel_bias[i];
		gbias_[i] = conf.gyro_bias[i];
		mscale_[i] = conf.mag_scale[i];
		mbias_[i] = conf.mag_bias[i];
	}
}

void IMU::update()
{
	int16_t accel[3];
	int16_t gyro[3];
	int16_t mag[3];
	
	mpu_mutex_.lock();

		if (mpu_.checkNewAccelGyroData())
		{
			mpu_.readAccelData(accel);
			ax = (float)accel[0] * ares_ - abias_[0];
			ay = (float)accel[1] * ares_ - abias_[1];
			az = (float)accel[2] * ares_ - abias_[2];
			ax *= gravitationalAcceleration;
			ay *= gravitationalAcceleration;
			az *= gravitationalAcceleration;

			mpu_.readGyroData(gyro);
			gx = (float)gyro[0] * gres_ - gbias_[0];
			gy = (float)gyro[1] * gres_ - gbias_[1];
			gz = (float)gyro[2] * gres_ - gbias_[2];
			gx *= degreeToRadian;
			gy *= degreeToRadian;
			gz *= degreeToRadian;		
		}

		mpu_.readMagData(mag);
		// Magnetometer axes are changed to (y,x,-z) to comply with North-West-Up world frame 
		mx = (float)mag[1] * mres_ * magCal_[1] - mbias_[1];
		my = (float)mag[0] * mres_ * magCal_[0] - mbias_[0];
		mz = -((float)mag[2] * mres_ * magCal_[2] - mbias_[2]);
		mx *= mscale_[1] * mGuassToGauss;
		my *= mscale_[0] * mGuassToGauss;
		mz *= mscale_[2] * mGuassToGauss;

	mpu_mutex_.unlock();
}

void IMU::calGyroAccel()
{
	mpu_mutex_.lock();

		mpu_.calibrateMPU9250(gbias_, abias_);
		Serial.printf("[IMU] MPU Calibration:\r\n");
		Serial.printf("Gyro bias: %f %f %f\r\n", gbias_[0], gbias_[1], gbias_[2]);
		Serial.printf("Accel bias: %f %f %f\r\n", abias_[0], abias_[1], abias_[2]);

		mpu_.initMPU9250(AFS_4G, GFS_500DPS, 0x02);

	mpu_mutex_.unlock();

	conf.gyro_bias[0] = gbias_[0];
	conf.gyro_bias[1] = gbias_[1];
	conf.gyro_bias[2] = gbias_[2];
	conf.accel_bias[0] = abias_[0];
	conf.accel_bias[1] = abias_[1];
	conf.accel_bias[2] = abias_[2];
	store_config();
}

void IMU::calMag()
{
	mpu_mutex_.lock();

		mpu_.magcalMPU9250(mbias_, mscale_);
		Serial.printf("[IMU] mag Calibration:\r\n");
		Serial.printf("Mag bias: %f %f %f\r\n", mbias_[0], mbias_[1], mbias_[2]);
		Serial.printf("Mag scale: %f %f %f\r\n", mscale_[0], mscale_[1], mscale_[2]);

	mpu_mutex_.unlock();

	conf.mag_scale[0] = mscale_[0];
	conf.mag_scale[1] = mscale_[1];
	conf.mag_scale[2] = mscale_[2];
	conf.mag_bias[0] = mbias_[0];
	conf.mag_bias[1] = mbias_[1];
	conf.mag_bias[2] = mbias_[2];
	store_config();
}