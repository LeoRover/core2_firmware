#ifndef INCLUDE_SENSORS_GPS_H_
#define INCLUDE_SENSORS_GPS_H_

#include "hFramework.h"

struct gga {
  int time;
  float latitude;
  float longitude;
  float hdop;
  float altitude;
};

class GPS {
 public:
  explicit GPS(hFramework::hSerial &serial) : serial_(serial) {}

  void init();
  void pollNextMessage();
  const gga &getMessage() { return gpgga_; }

 private:
  bool read();
  bool update(const char *sentence);

  gga gpgga_;
  hFramework::hSerial &serial_;
};

#endif  // INCLUDE_SENSORS_GPS_H_
