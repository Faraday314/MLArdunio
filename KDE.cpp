#include "Ardunio.h"
#include "KDE.h"

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
