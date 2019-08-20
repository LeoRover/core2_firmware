#include "hFramework.h"
#include "SparkFunMPU9250-DMP.h"

//undefine min/max macros to fix conflict with standard library
#undef min
#undef max

#include "imu.h"

MPU9250_DMP mpu;

static const signed char rotationMatrix[9] = {
	1, 0, 0,
	0, 1, 0,
	0, 0, 1
};

static const float gravitationalAcceleration = 9.80665;
static const float degreeToRadian = 2.0 * M_PI / 360.0;

void IMU::begin()
{
	if (mpu.begin() != INV_SUCCESS)
	{
        sys.log("[IMU] initialization failed\r\n");
		return;
	}

	mpu.setSensors(INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_XYZ_COMPASS);

	mpu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |  // Enable 6-axis quat
				    DMP_FEATURE_GYRO_CAL,  // Use gyro calibration
				 50);					   // Set DMP FIFO rate to 50 Hz (max 200 Hz)

	mpu.dmpSetOrientation(rotationMatrix);

	mpu.setLPF(42); //188, 98, 42, 20, 10
}

void IMU::update()
{
	while (mpu.fifoAvailable())
	{
		if ( mpu.dmpUpdateFifo() != INV_SUCCESS)
      	{
			sys.log("[IMU] dmp update fifo failed\r\n");
      	}
	}

	if (mpu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS) != INV_SUCCESS)
	{
		sys.log("[IMU] update failed\r\n");
	}
}

std::vector<float> IMU::getAccel()
{
	std::vector<float> acc(3);
	acc[0] = mpu.calcAccel(mpu.ax) * gravitationalAcceleration;
	acc[1] = mpu.calcAccel(mpu.ay) * gravitationalAcceleration;
	acc[2] = mpu.calcAccel(mpu.az) * gravitationalAcceleration;
	return acc;
}

std::vector<float> IMU::getGyro()
{
	std::vector<float> gyro(3);
	gyro[0] = mpu.calcGyro(mpu.gx) * degreeToRadian;
	gyro[1] = mpu.calcGyro(mpu.gy) * degreeToRadian;
	gyro[2] = mpu.calcGyro(mpu.gz) * degreeToRadian;
	return gyro;
}

std::vector<float> IMU::getQuaternion()
{
	std::vector<float> quat(4);
	quat[0] = mpu.calcQuat(mpu.qx);
	quat[1] = mpu.calcQuat(mpu.qy);
	quat[2] = mpu.calcQuat(mpu.qz);
	quat[3] = mpu.calcQuat(mpu.qw);
	return quat;
}

std::vector<float> IMU::getMag()
{
	std::vector<float> mag(3);
	mag[0] = mpu.calcMag(mpu.mx) * 0.000001;
	mag[1] = mpu.calcMag(mpu.my) * 0.000001;
	mag[2] = mpu.calcMag(mpu.mz) * 0.000001;
	return mag;
}