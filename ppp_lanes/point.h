#pragma once

#include <stdint.h>
#include <map>

static std::map<int, std::array<unsigned char,3>> colors = 
{
    {0, {100, 0, 0}},
    {1, {0, 100, 0}},
    {2, {0, 0, 100}},
    {3, {100, 100, 0}},
    {4, {100, 0, 100}},
    {5, {0, 100, 100}},
    {6, {200, 0, 0}},
    {7, {0, 200, 0}},
    {8, {0, 0, 200}},
    {9, {200, 200, 0}},
    {10,{200, 0, 200}},
    {11,{0, 200, 200}},
    {12,{150, 50, 0}},
    {13,{150, 0, 50}},
    {14,{0, 150, 50}},
    {15,{50, 150, 0}},
    {16,{50, 0, 150}},
    {17,{0, 50, 150}}
};

struct __attribute__((packed)) Point
{
    Point() {}
    Point(int _id) : index(_id) {}
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
    unsigned char color [3] = {255,0,0};
    int index;
    bool isVisited {false}; // implicitely used for ANY c-tor, so fits for us.
    int weight {1}; // the number of aggregated points
};