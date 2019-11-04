/*
 * Library for Kernel Density Estimation with an Epanechnikov Kernel.
 * Created by Cole Savage, November 4, 2019.
 */

#ifndef KDE_h
#define KDE_h

#include "Arduino.h"
#include <math.h>

class Kernel {
  public:
    float datapoint;
    float h;
    Kernel(float datapoint_param, float h_param);
    float calcValue(float x);
    float getU(float x);
};
#endif
