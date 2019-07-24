#ifndef LEO_FIRMWARE_UTILS_H_
#define LEO_FIRMWARE_UTILS_H_

inline float clamp(float value, float limit)
{
    if (value > limit) 
        return limit;
    else if (value < -limit)
        return -limit;
    else
        return value;
}

#endif