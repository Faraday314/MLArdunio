/*
 * Library for Kernel Density Estimation with an Epanechnikov Kernel.
 * Created by Cole Savage, November 4, 2019.
 */

#ifndef KDE_h
#define KDE_h

#include <Arduino.h>
#include <math.h>

class Kernel {
  public:
    float datapoint;
    float h;
    Kernel(float datapoint_param, float h_param);
    float calcValue(float x);
    float getU(float x);
};

Kernel::Kernel (float datapoint_param, float h_param) {
  datapoint = datapoint_param;
  h = h_param;
}

float Kernel::calcValue(float x){
  return (1.0f/h)*(3.0f/4.0f)*(1.0 - pow(x - datapoint, 2));
}

float Kernel::getU(float x) {
  return (x - datapoint)/h;
}

#endif
