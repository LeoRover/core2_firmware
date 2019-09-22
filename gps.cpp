#include "hFramework.h"

#include "gps.h"
#include <string.h>

void GPS::begin()
{
    hSens3.selectSerial();
    hSens3.serial.init(9600, Parity::None, StopBits::One);
}

void GPS::read()
{
    int lenght=0;

    while(true)
    {
            hSens3.serial.read(&x,1);
            if(x=='\n') break;
            received_data[lenght]=x;
            lenght++;
    }
    received_data[lenght-1] = NULL;
 
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

bool GPS::check(char *sentence, bool strict)
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

bool GPS::isGGA(char *sentence)
{
    char temp[5];
    strncpy(temp, sentence, 6);
    if (!strcmp(temp, "$GPGGA"))
        return 1;
    return 0;
}

void GPS::update(char *sentence)
{
    int ptr = 0;
    int j = 0;
    int data_no = 0;
    char raw[15][15]; //[number of data raw][max lenght of data]
    float data_f;
    int data_i;
    while(sentence[ptr] != '*')
    {
        if (sentence[ptr] == ',')
        {
            raw[data_no][j] = NULL;
            ptr++;
            j=0;
            data_no++;
            continue;
        }
        if(sentence[ptr] == ',')
        {
            raw[data_no][j] = NULL;
            ptr++;
            data_no++;
            continue;
        }
        if (sentence[ptr] == '*')
        {
            raw[data_no][j] = NULL;
            break;
        }
        raw[data_no][j] = sentence[ptr];
        j++;
        ptr++;
    }

   
    if(strlen(raw[1])!=0)
    {
        data_i = atoi(raw[1]);
        gpgga.time.hours = data_i/10000;
        gpgga.time.minutes = (data_i%10000)/100;
        gpgga.time.seconds = (data_i%100);
    }
    else 
    {
        gpgga.time.hours = 0;
        gpgga.time.minutes = 0;
        gpgga.time.seconds = 0;
    }

    if(strlen(raw[2])!=0)
    {
        data_f = atof(raw[2]);
        if(raw[3][0] == 'N')
            gpgga.latitude = data_f;
        else if (raw[3][0] == 'S')
            gpgga.latitude = -data_f;
    }
    else gpgga.latitude = 0;

    if(strlen(raw[4])!=0)
    {
        data_f = atof(raw[4]);
        if(raw[5][0] == 'E')
            gpgga.longitude = data_f;
        else if (raw[5][0] == 'W')
            gpgga.longitude = -data_f;
    }
    else gpgga.longitude = 0;

    if(strlen(raw[6])!=0)
    {
        data_i = atoi(raw[6]);
        gpgga.fix_quality = data_i;
    }
    else gpgga.fix_quality = 0;

    if(strlen(raw[7])!=0)
    {
        data_i = atoi(raw[7]);
        gpgga.satellites_tracked = data_i;
    }
    else gpgga.satellites_tracked = 0;
    
    if(strlen(raw[8])!=0)
    {
        data_f = atof(raw[8]);
        gpgga.hdop = data_f;
    }
    else gpgga.hdop = 10;


    printf("time: %d:%d:%d\n%f %f\nfix: %d\nnumber of sat: %d\ndilution: %f\n",gpgga.time.hours,gpgga.time.minutes, gpgga.time.seconds, gpgga.latitude, gpgga.longitude, gpgga.fix_quality, gpgga.satellites_tracked, gpgga.hdop);

}



