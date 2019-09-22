#ifndef _GPS_H_
#define _GPS_H_

#include "hFramework.h"

#define MAX_LENGTH 80

struct times{
    int hours;
    int minutes;
    int seconds;
};

// struct date{
//     int day;
//     int mounth;
//     int year;
// };



struct gga {
    struct times time;
    float latitude; char latitude_units;
    float longitude; char longitude_units;
    int fix_quality;
    int satellites_tracked;
    float hdop;
    float altitude; char altitude_units;
    float height; char height_units;
    float dgps_age;
};


class GPS
{
public:
    gga gpgga;
    char received_data[110];
    
    void begin();
    void read();
    bool check(char *sentence, bool strict);
    void update(char *sentence);
    bool isGGA(char *sentence);

private:
    
    char x;

    
};

#endif