#include "hFramework.h"

#include "gps.h"
#include <string.h>

void GPS::begin()
{
    

}

uint8_t GPS::checksum(const char *sentence)
{
    if (*sentence == '$')
        sentence++;

    uint8_t checksum = 0x00;

    while (*sentence && *sentence != '*')
        checksum ^= *sentence++;

    return checksum;
}


static int hex2int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

bool check(const char *sentence, bool strict)
{
    uint8_t sum = 0x00;

    if (strlen(sentence) > MAX_LENGTH + 3)
        return false;

    if (*sentence++ != '$')
        return false;

     while (*sentence && *sentence != '*'&& isprint((unsigned char) *sentence))
        sum ^= *sentence++;


    if (*sentence == '*')
    {
        sentence++;
        int upper = hex2int(*sentence++);
        if (upper == -1)
            return false;
        int lower = hex2int(*sentence++);
        if (lower == -1)
            return false;
        int expected = upper << 4 | lower;

        if (sum != expected)
            return false;
    } else if (strict) {
        return false;
    }

    if (*sentence && strcmp(sentence, "\n") && strcmp(sentence, "\r\n"))
        return false;

    return true;
}

static inline bool minmea_isfield(char c) 
{
    return isprint((unsigned char) c) && c != ',' && c != '*';
}