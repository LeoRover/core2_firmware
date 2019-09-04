
#ifndef LEO_FIRMWARE_CONFIG_H_
#define LEO_FIRMWARE_CONFIG_H_

#include <cstdint>

#define CONFIG_ADDRESS 0x01

struct Config
{
    uint8_t checksum;

    bool imu_enabled;
    float accel_bias[3];
    float mag_scale[3];
    float mag_bias[3];
} __attribute__ ((packed));

extern Config conf;

void load_config();
void store_config();

#endif