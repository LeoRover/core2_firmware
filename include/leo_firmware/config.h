
#ifndef LEO_FIRMWARE_INCLUDE_CONFIG_H_
#define LEO_FIRMWARE_INCLUDE_CONFIG_H_

#include <cstdint>

static const uint16_t CONFIG_ADDRESS = 0x01;

struct Config {
  uint8_t checksum;

  bool debug_logging;
  bool imu_enabled;
  bool gps_enabled;
  float gyro_bias[3];
  float accel_bias[3];
  float mag_scale[3];
  float mag_bias[3];

  Config()
      : debug_logging(false),
        imu_enabled(false),
        gps_enabled(false),
        gyro_bias{0.0, 0.0, 0.0},
        accel_bias{0.0, 0.0, 0.0},
        mag_scale{1.0, 1.0, 1.0},
        mag_bias{0.0, 0.0, 0.0} {}
} __attribute__((packed));

extern Config conf;

void configPrint();
void configLoad();
void configStore();
void configReset();

#endif  // LEO_FIRMWARE_INCLUDE_CONFIG_H_
