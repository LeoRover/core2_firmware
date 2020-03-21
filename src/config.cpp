#include "config.h"
#include "hFramework.h"

Config conf;
hStorage storage;

uint8_t checksum(Config* config) {
  int size = sizeof(Config);
  uint8_t* data = reinterpret_cast<uint8_t*>(config);

  uint8_t checksum = 0;
  for (int i = 1; i < size; i++) checksum += data[i];
  return checksum;
}

void print_config() {
  Serial.printf("imu_enabled: %s\r\n", conf.imu_enabled ? "true" : "false");
  Serial.printf("gyro_bias: %f %f %f\r\n", conf.gyro_bias[0], conf.gyro_bias[1],
                conf.gyro_bias[2]);
  Serial.printf("accel_bias: %f %f %f\r\n", conf.accel_bias[0],
                conf.accel_bias[1], conf.accel_bias[2]);
  Serial.printf("mag_scale: %f %f %f\r\n", conf.mag_scale[0], conf.mag_scale[1],
                conf.mag_scale[2]);
  Serial.printf("mag_bias: %f %f %f\r\n", conf.mag_bias[0], conf.mag_bias[1],
                conf.mag_bias[2]);
  Serial.printf("checksum: %d\r\n", conf.checksum);
}

void load_config() {
  Config tmp_conf;
  storage.load(CONFIG_ADDRESS, tmp_conf);

  if (tmp_conf.checksum != checksum(&tmp_conf)) {
    Serial.printf(
        "Config checksum incorrect! Default configuration will be used\r\n");
    print_config();
  } else {
    conf = tmp_conf;
    Serial.printf("Loaded config: \r\n");
    print_config();
  }
}

void store_config() {
  conf.checksum = checksum(&conf);
  storage.store(CONFIG_ADDRESS, conf);
  Serial.printf("Stored config: \r\n");
  print_config();
}

void reset_config() {
  conf = Config();
  store_config();
}
