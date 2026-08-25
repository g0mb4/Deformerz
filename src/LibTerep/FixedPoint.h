#pragma once

#include <stdint.h>

// Q8.8
typedef int16_t FixedPoint;

static inline float FixedPoint_toFloat(FixedPoint fp)
{ 
    return (float)fp / 256.0f;
}
