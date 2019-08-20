#include "hFramework.h"
#include "SparkFunMPU9250-DMP.h"

//undefine min/max macros to fix conflict with standard library
#undef min
#undef max

#include "imu.h"

MPU9250_DMP mpu;

const signed char orientationDefault[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};

void IMU::begin()
{

	hSens2.pin1.interruptOn_EdgeRising();

	if (mpu.begin() != INV_SUCCESS)
	{
        sys.log("[IMU] initialization failed\r\n");
		return;
	}

	//mpu.setSensors(INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_XYZ_COMPASS);

	mpu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |  // Enable 6-axis quat
				    DMP_FEATURE_GYRO_CAL,  // Use gyro calibration
				 50);					   // Set DMP FIFO rate to 50 Hz (max 200 Hz)

	mpu.dmpSetOrientation(orientationDefault);

	// Use enableInterrupt() to configure the MPU-9250's
	// interrupt output as a "data ready" indicator.
	mpu.enableInterrupt();

	mpu.setLPF(42); //188, 98, 42, 20, 10

	sys.taskCreate(std::bind(&IMU::updateLoop, this));
}

void IMU::updateLoop()
{
	while (hSens2.pin1.interruptWait())
	{
		while (mpu.fifoAvailable())
		{
			// Use dmpUpdateFifo to update the ax, gx, mx, etc. values
			if (mpu.dmpUpdateFifo() != INV_SUCCESS)
			{
				sys.log("[IMU] update failed");
			}
		}
	}
}

std::vector<float> IMU::getAccel()
{
	std::vector<float> acc(3);
	acc[0] = mpu.calcAccel(mpu.ax);
	acc[1] = mpu.calcAccel(mpu.ay);
	acc[2] = mpu.calcAccel(mpu.az);
	return acc;
}

std::vector<float> IMU::getGyro()
{
	std::vector<float> gyro(3);
	gyro[0] = mpu.calcGyro(mpu.gx);
	gyro[1] = mpu.calcGyro(mpu.gy);
	gyro[2] = mpu.calcGyro(mpu.gz);
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

void IMU::resetFifo()
{
	mpu.resetFifo();
}
