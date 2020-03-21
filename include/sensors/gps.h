#ifndef INCLUDE_SENSORS_GPS_H_
#define INCLUDE_SENSORS_GPS_H_

#include "hFramework.h"

#define MAX_LENGTH 80

struct gga {
  int time;
  float latitude;
  float longitude;
  float hdop;
  float altitude;
};

class GPS {
 public:
  bool is_new_data = false;
  gga gpgga;

  void begin();
  void receive_next_msg();

 private:
  char received_data[210];

  bool read();
  bool check(char *sentence);
  bool update(char *sentence);
  bool isGGA(char *sentence);
  float NMEAtoDec(char *pos);
};

#endif  // INCLUDE_SENSORS_GPS_H_
