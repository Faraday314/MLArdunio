#include "Gaussian.h"
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

Gaussian::Gaussian (float mean_param, float standardDeviation_param) {
	mean = mean_param;
	standardDeviation = standardDeviation_param;
}

float Gaussian::getValue(float x) {
	return (1.0/(standardDeviation*sqrt(2.0*PI)))*exp(-0.5*((x-mean)/standardDeviation)*((x-mean)/standardDeviation));
}

float Gaussian::getMean() {
	return mean;
}

float Gaussian::getStandardDeviation() {
	return standardDeviation;
}

float Gaussian::getVariance() {
	return standardDeviation*standardDeviation;
}

void Gaussian::setMean(float newMean) {
	mean = newMean;
}

void Gaussian::setStandardDeviation(float newStandardDeviation) {
	standardDeviation = newStandardDeviation;
}

void Gaussian::setVariance(float newVariance) {
	setStandardDeviation(sqrt(newVariance));
}