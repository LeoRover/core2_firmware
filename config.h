
#ifndef LEO_FIRMWARE_CONFIG_H_
#define LEO_FIRMWARE_CONFIG_H_

#define CONFIG_ADDRESS 0x01

struct config
{
    bool imu_enabled;
};

extern config conf;

void load_config();
void store_config();

#endif