#ifndef _UTILS_H_
#define _UTILS_H_

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