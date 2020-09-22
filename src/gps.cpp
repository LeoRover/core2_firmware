#include <string.h>

#include <hFramework.h>

#include <leo_firmware/sensors/gps.h>

#define MAX_LENGTH 200

static char message_buffer[MAX_LENGTH];

static int hex2int(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static bool check(const char *sentence) {
  int sum = 0x00;

  if (*sentence++ != '$') return false;

  while (*sentence && *sentence != '*' && isprint((unsigned char)*sentence))
    sum ^= *sentence++;

  if (*sentence == '*') {
    sentence++;
    if (strlen(sentence) != 2) return false;

    int upper = hex2int(*sentence++);
    if (upper == -1) return false;
    int lower = hex2int(*sentence++);
    if (lower == -1) return false;
    int expected = upper << 4 | lower;

    if (sum != expected) return false;
  } else {
    return false;
  }

  return true;
}

static bool isGGA(const char *sentence) {
  char temp[6];
  strncpy(temp, sentence, 6);
  temp[6] = 0;
  if (strcmp(temp, "$GPGGA") == 0)
    return true;
  else
    return false;
}

static float NMEAtoDec(const char *pos) {
  float nmea = atof(pos);
  float dec;

  dec = static_cast<int>(nmea / 100);
  dec = dec + ((nmea - dec * 100) / 60);
  return dec;
}

void GPS::init() {
  serial_.init(9600, Parity::None, StopBits::One);
}

bool GPS::read() {
  int length = 0;
  char x;

  while (true) {
    if (serial_.available()) {
      serial_.read(&x, 1);
      if (x == '\n') break;
      message_buffer[length] = x;
      length++;
      if (length >= MAX_LENGTH) return false;
    } else {
      sys.delay(10);
    }
  }

  if (length == 0) return false;

  message_buffer[length - 1] = 0;
  return true;
}

bool GPS::update(const char *sentence) {
  int mptr = 0;
  int dptr = 0;
  int data_no = 0;
  char data_raw[16][30];

  while (true) {
    if (sentence[mptr] == ',') {
      data_raw[data_no][dptr] = 0;
      data_no++;
      mptr++;
      dptr = 0;
    } else if (sentence[mptr] == '*') {
      data_raw[data_no][dptr] = 0;
      data_no++;
      break;
    } else {
      data_raw[data_no][dptr] = sentence[mptr];
      dptr++;
      mptr++;
    }
  }

  if (data_no < 10) return false;

  if (data_raw[1][0] != 0)
    gpgga_.time = atoi(data_raw[1]);
  else
    return false;

  if (data_raw[2][0] != 0) {
    switch (data_raw[3][0]) {
      case 'N':
        gpgga_.latitude = NMEAtoDec(data_raw[2]);
        break;
      case 'S':
        gpgga_.latitude = -NMEAtoDec(data_raw[2]);
        break;
      default:
        return false;
    }
  } else {
    return false;
  }

  if (data_raw[4][0] != 0) {
    switch (data_raw[5][0]) {
      case 'E':
        gpgga_.longitude = NMEAtoDec(data_raw[4]);
        break;
      case 'W':
        gpgga_.longitude = -NMEAtoDec(data_raw[4]);
        break;
      default:
        return false;
    }
  } else {
    return false;
  }

  if (data_raw[9][0] != 0)
    gpgga_.altitude = atof(data_raw[9]);
  else
    return false;

  if (data_raw[8][0] != 0)
    gpgga_.hdop = atof(data_raw[8]);
  else
    return false;

  return true;
}

void GPS::pollNextMessage() {
  while (true) {
    if (read() && check(message_buffer) && isGGA(message_buffer) &&
        update(message_buffer))
      break;
  }
}
