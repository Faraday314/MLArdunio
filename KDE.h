/*
 * Library for Kernel Density Estimation with an Epanechnikov Kernel.
 * Created by Cole Savage, November 4, 2019.
 */

#ifndef KDE
#define KDE

class Kernel {
  public:
    float datapoint;
    float h;
	Kernel();
    Kernel(float datapoint_param, float h_param);
    float calcValue(float x);
    float getU(float x);
	static float kernelConsensus(Kernel *kernelPtr, unsigned int listSize, float x);
};
#endif
