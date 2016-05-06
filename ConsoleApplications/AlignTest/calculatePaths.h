#ifndef CALCULATEPATHS_H
#define CALCULATEPATHS_H
#include <stdint.h>

void calculatePaths (uint32_t radius, double q0[3], double d[3], double stepsize,
                     uint32_t speed, int windSpeed);
#endif
