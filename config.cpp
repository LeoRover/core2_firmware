#include "hFramework.h"
#include "config.h"

config conf;
hStorage storage;

void print_config()
{
    Serial.printf("imu_enabled: %s\r\n", conf.imu_enabled?"true":"false");
}

void load_config()
{
    storage.load(CONFIG_ADDRESS, conf);
    Serial.printf("Loaded config: \r\n");
    print_config();
}

void store_config()
{
    storage.store(CONFIG_ADDRESS, conf);
    Serial.printf("Stored config: \r\n");
    print_config();
}

