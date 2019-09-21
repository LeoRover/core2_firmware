#ifndef _GPS_H_
#define _GPS_H_

#include "hFramework.h"

#define MAX_LENGTH 80



class GPS
{
public:
    void begin();
    

private:
    uint8_t checksum(const char *sentence);
    bool check(const char *sentence, bool strict);
};

#endif