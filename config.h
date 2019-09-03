
#ifndef LEO_FIRMWARE_CONFIG_H_
#define LEO_FIRMWARE_CONFIG_H_

#define CONFIG_ADDRESS 0x01

struct Config
{
    bool imu_enabled;
    float accel_bias[3];
    float mag_scale[3];
    float mag_bias[3];

    // this should always be the last member
    uint8_t checksum;
} __attribute__ ((packed));

extern Config conf;

void load_config();
void store_config();

#endif