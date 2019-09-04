/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. 
 Addition of 9 DoF sensor fusion using open source Madgwick and Mahony filter algorithms. 
 Sketch runs on the 3.3 V Dragonfly STM32L476 Breakout Board.
 
 Library may be used freely and without limit with attribution.
 
*/
  
#ifndef MPU9250_h
#define MPU9250_h

#include <cstdint> 

class MPU9250
{
public: 
  MPU9250(hFramework::hI2C& i2c)
    : _i2c(i2c) {}

  void begin(uint32_t datarate);
  uint8_t getMPU9250ID();
  uint8_t getAK8963CID();
  void resetMPU9250();
  void initMPU9250(uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate);
  void initAK8963(uint8_t Mscale, uint8_t Mmode, float * destination);
  void initAK8963Slave(uint8_t Mscale, uint8_t Mmode, float * destination);
  float getAres(uint8_t Ascale);
  float getGres(uint8_t Gscale);
  float getMres(uint8_t Mscale);
  void magcalMPU9250(float * dest1, float * dest2);
  void calibrateMPU9250(float * dest1, float * dest2);
  void SelfTest(float * destination);
  void readMPU9250Data(int16_t * destination);
  void readAccelData(int16_t * destination);
  void readGyroData(int16_t * destination);
  bool checkNewAccelGyroData();
  bool checkNewMagData();
  void readMagData(int16_t * destination);
  int16_t readGyroTempData();
  void gyromagSleep();
  void gyromagWake(uint8_t Mmode);
  void accelWakeOnMotion();
  bool checkWakeOnMotion();
//  void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
  void    writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
  uint8_t readByte(uint8_t address, uint8_t subAddress);
  void    readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
private:
  hFramework::hI2C& _i2c;
  float _aRes;
  float _gRes;
  float _mRes;
  uint8_t _Mmode;
  float _fuseROMx;
  float _fuseROMy;
  float _fuseROMz;
  float _magCalibration[3];
};

#endif
