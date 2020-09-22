#include <hFramework.h>

#include <leo_firmware/config.h>
#include <leo_firmware/logging.h>

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
  logInfo("debug_logging: %s", conf.debug_logging ? "true" : "false");
  logInfo("imu_enabled: %s", conf.imu_enabled ? "true" : "false");
  logInfo("gps_enabled: %s", conf.gps_enabled ? "true" : "false");
  logInfo("gyro_bias: %f %f %f", conf.gyro_bias[0], conf.gyro_bias[1],
          conf.gyro_bias[2]);
  logInfo("accel_bias: %f %f %f", conf.accel_bias[0], conf.accel_bias[1],
          conf.accel_bias[2]);
  logInfo("mag_scale: %f %f %f", conf.mag_scale[0], conf.mag_scale[1],
          conf.mag_scale[2]);
  logInfo("mag_bias: %f %f %f", conf.mag_bias[0], conf.mag_bias[1],
          conf.mag_bias[2]);
  logInfo("checksum: %d", conf.checksum);
}

void load_config() {
  Config tmp_conf;
  storage.load(CONFIG_ADDRESS, tmp_conf);

  if (tmp_conf.checksum != checksum(&tmp_conf)) {
    logWarn("Config checksum incorrect! Default configuration will be used");
    store_config();
  } else {
    conf = tmp_conf;
    logInfo("Loaded config:");
    print_config();
  }
}

void store_config() {
  conf.checksum = checksum(&conf);
  storage.store(CONFIG_ADDRESS, conf);
  logInfo("Stored config:");
  print_config();
}

void reset_config() {
  conf = Config();
  store_config();
}
