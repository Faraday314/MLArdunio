#include "KDE.h"
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

Kernel::Kernel() {
	datapoint = 0;
	h = 0;
}

Kernel::Kernel (float datapoint_param, float h_param) {
  datapoint = datapoint_param;
  h = h_param;
}

float Kernel::calcValue(float x){
  return (1.0f/h)*(3.0f/4.0f)*(1.0 - pow((x - datapoint)/h, 2));
}

float Kernel::getU(float x) {
  return (x - datapoint)/h;
}

float Kernel::getH() {
	return h;
}

static float Kernel::kernelConsensus(Kernel *kernels, unsigned int listSize, float x) {
	
	float sum = 0;
	for(unsigned int i = 0; i < listSize; i++) {
		if(abs(kernels[i].getU(x)) < 1) {
			sum += kernels[i].calcValue(x);
		}
	}
	return sum/listSize;
}