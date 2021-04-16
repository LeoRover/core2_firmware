
#ifndef LEO_FIRMWARE_INCLUDE_CONFIG_H_
#define LEO_FIRMWARE_INCLUDE_CONFIG_H_

#include <cstdint>

static const uint16_t CONFIG_ADDRESS = 0x01;

struct Config {
  uint8_t checksum;

  bool debug_logging = false;
  bool imu_enabled = false;
  bool gps_enabled = false;
  float gyro_bias[3] = {0.0, 0.0, 0.0};
  float accel_bias[3] = {0.0, 0.0, 0.0};
  float mag_scale[3] = {1.0, 1.0, 1.0};
  float mag_bias[3] = {0.0, 0.0, 0.0};
} __attribute__((packed));

extern Config conf;

void configPrint();
void configLoad();
void configStore();
void configReset();

#endif  // LEO_FIRMWARE_INCLUDE_CONFIG_H_
