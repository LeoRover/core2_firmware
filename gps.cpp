#include "hFramework.h"

#include "gps.h"
#include <string.h>

void GPS::begin()
{
    hSens3.selectSerial();
    hSens3.serial.init(9600, Parity::None, StopBits::One);
}

bool GPS::read()
{
    int length=0;
    char x;

    while(true)
    {
        if(hSens3.serial.available())
        {
            hSens3.serial.read(&x,1);
            if(x=='\n') break;
            received_data[length]=x;
            length++;
            if(length>=200) return false;
        }
        else sys.delay(10);
    }

    if (length == 0)
        return false;

    received_data[length-1] = 0;
    return true;  
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

bool GPS::check(char *sentence)
{
    int sum = 0x00;

    if (*sentence++ != '$')
        return false;

    while (*sentence && *sentence != '*' && isprint((unsigned char) *sentence))
        sum ^= *sentence++;


    if (*sentence == '*')
    {
        sentence++;
        if (strlen(sentence) != 2)
            return false;

        int upper = hex2int(*sentence++);
        if (upper == -1)
            return false;
        int lower = hex2int(*sentence++);
        if (lower == -1)
            return false;
        int expected = upper << 4 | lower;

        if (sum != expected)
            return false;
    } 
    else return false;

    return true;
}


bool GPS::isGGA(char *sentence)
{
    char temp[6];
    strncpy(temp, sentence, 6);
    temp[6] = 0;
    if (strcmp(temp, "$GPGGA") == 0) return true;
    else return false;
}

float GPS::NMEAtoDec(char *pos)
{
    float nmea=atof(pos);
    float dec;

    dec=int(nmea/100);
    dec=dec+((nmea-dec*100)/60);
    return dec;
}

bool GPS::update(char *sentence)
{
    int mptr=0;
    int dptr=0;
    int data_no=0;
    char data_raw[16][30];

    while(true)
    {
        if (sentence[mptr] == ',')
        {
            data_raw[data_no][dptr] = 0;
            data_no++;
            mptr++;
            dptr=0;
        }
        else if (sentence[mptr] == '*')
        {
            data_raw[data_no][dptr] = 0;
            data_no++;
            break;
        }
        else
        {
            data_raw[data_no][dptr] = sentence[mptr];
            dptr++;
            mptr++;
        }
    }
    
    if (data_no < 10)
        return false;

    if (data_raw[1][0] != 0) gpgga.time=atoi(data_raw[1]);
    else return false;

    if (data_raw[2][0] != 0)
    {
        switch (data_raw[3][0])
        {
        case 'N':
            gpgga.latitude=NMEAtoDec(data_raw[2]);
            break;
        case 'S':
            gpgga.latitude=-NMEAtoDec(data_raw[2]);
            break;    
        default:
            return false;
        }
    }
    else return false;

    if (data_raw[4][0] != 0)
    {
        switch (data_raw[5][0])
        {
        case 'E':
            gpgga.longitude=NMEAtoDec(data_raw[4]);
            break;
        case 'W':
            gpgga.longitude=-NMEAtoDec(data_raw[4]);
            break;    
        default:
            return false;
        }
    }
    else return false;

    if (data_raw[9][0] != 0) gpgga.altitude=atof(data_raw[9]);
    else return false;

    if (data_raw[8][0] != 0) gpgga.hdop=atof(data_raw[8]);
    else return false;

    return true;

}   

void GPS::receive_next_msg()
{
    while (true)
    {
        if (read() && check(received_data) && isGGA(received_data) && update(received_data))
            break;
    }
}




