#pragma once

#include <stdint.h>

struct __attribute__((packed)) Point
{
    Point() {}
    Point(int _id) : id(_id) {}
    Point(float x, float y, float z)
    {
        v[0] = x;
        v[1] = y;
        v[2] = z;
    }
    float & operator [](int i)  { return v[i]; }
    float v [3];
    int32_t intensity;
    int32_t t_lo, t_hi;
    int id;
};