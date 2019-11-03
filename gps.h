#ifndef _GPS_H_
#define _GPS_H_

#include "hFramework.h"

#define MAX_LENGTH 80

struct gga {
    int time;
    float latitude;
    float longitude;
    int fix_quality;
    int satellites_tracked;
    float hdop;
    float altitude;
    float height;
    float dgps_age;
    float dgps_id;
};


class GPS
{
public:
    bool is_new_data=false;
    gga gpgga;
    
    void begin();
    void receive_msgs();
    

private:
    char received_data[210];

    bool read();
    bool check(char *sentence);
    bool update(char *sentence);
    bool isGGA(char *sentence);
    float NMEAtoDec(char *pos);

    
};

#endif