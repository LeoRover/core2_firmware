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
    int lenght=0;
    char x;

    while(true)
    {
        if(hSens3.serial.available())
        {
            hSens3.serial.read(&x,1);
            if(x=='\n') break;
            received_data[lenght]=x;
            lenght++;
            if(lenght>=200) return 0;
        }
    }

    received_data[lenght-1] = NULL;
    return 1;  

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

    return true;
}


bool GPS::isGGA(char *sentence)
{
    char temp[6];
    strncpy(temp, sentence, 6);
    temp[6]=NULL;
    if (!strcmp(temp, "$GPGGA")) return 1;
    else return 0;
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
        if (sentence[mptr] == ',' && sentence[mptr+1] == ',')
        {
            data_raw[data_no][dptr]=NULL;
            data_raw[data_no+1][0] = NULL;
            data_no=data_no+2;
            mptr=mptr+2;
        }
        else if (sentence[mptr] == ',')
        {
            data_raw[data_no][dptr] = NULL;
            data_no++;
            mptr++;
            dptr=0;
        }
        
        if (sentence[mptr]=='*')
        {
            data_raw[data_no][dptr] = NULL;
            break;
        }
        
        data_raw[data_no][dptr] = sentence[mptr];
        dptr++;
        mptr++;
    }
#ifdef DEBUG
    for (int i=0 ; i<=13; i++)
    {
        Serial.printf("%s\n", data_raw[i]);

    }
#endif
    
    if (data_raw[1]!=NULL) gpgga.time=atoi(data_raw[1]);
    else return 0;

    if (data_raw[2]!=NULL)
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
            return 0;
        }
    }
    else return 0;

    if (data_raw[4]!=NULL)
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
            return 0;
        }
    }
    else return 0;

    if (data_raw[9]!=NULL) gpgga.altitude=atof(data_raw[9]);
    else return 0;

    if (data_raw[8]!=NULL) gpgga.hdop=atof(data_raw[8]);
    else return 0;

    return 1;

}   

void GPS::receive_msgs()
{
    if (read() && check(received_data) && isGGA(received_data))
    {
        if (update(received_data)==1) is_new_data=true;
    }
}




