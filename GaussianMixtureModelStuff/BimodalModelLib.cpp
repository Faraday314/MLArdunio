#include "BimodalModelLib.h"
#include "GaussianMixtureModel.h"
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

BimodalModel::BimodalModel(Gaussian blob1_param, Gaussian blob2_param) {
	blob1 = blob1_param;
	blob2 = blob2_param;

	weight1 = 0.5;
	weight2 = 0.5;

	resetSums();

	maxDelta = 100000;
}

float BimodalModel::getValue(float x) {
	return weight1 * blob1.getValue(x) + weight2 * blob2.getValue(x);
}

float BimodalModel::getBlob1Gamma(float x) {
	return (weight1 * blob1.getValue(x)) / getValue(x);
}

float BimodalModel::getBlob2Gamma(float x) {
	return (weight1 * blob2.getValue(x)) / getValue(x);
}

void BimodalModel::resetSums() {
	gammaSum1 = 0;
	gammaSum2 = 0;

	gammaProductSum1 = 0;
	gammaProductSum2 = 0;

	gammaVarianceSum1 = 0;
	gammaVarianceSum2 = 0;
}

void BimodalModel::calcMaxDelta(unsigned int numberDatapoints) {
	
	float delta;

	delta = weight1 - (gammaSum1 / numberDatapoints);
	maxDelta = abs(delta);

	delta = weight2 - (gammaSum2 / numberDatapoints);
	if (abs(delta) > maxDelta) {
		maxDelta = abs(delta);
	}

	delta = blob1.getMean() - (gammaProductSum1 / gammaSum1);
	if (abs(delta) > maxDelta) {
		maxDelta = abs(delta);
	}

	delta = blob2.getMean() - (gammaProductSum2 / gammaSum2);
	if (abs(delta) > maxDelta) {
		maxDelta = abs(delta);
	}

	delta = blob1.getVariance() - (gammaVarianceSum1 / gammaSum1);
	if (abs(delta) > maxDelta) {
		maxDelta = abs(delta);
	}

	delta = blob2.getVariance() - (gammaVarianceSum2 / gammaSum2);
	if (abs(delta) > maxDelta) {
		maxDelta = abs(delta);
	}
}

void BimodalModel::updateModel(float x) {
	float gamma1 = getBlob1Gamma(x);

	gammaSum1 += gamma1;
	gammaProductSum1 += gamma1 * x;
	gammaVarianceSum1 += gamma1 * (x - blob1.getMean()) * (x - blob1.getMean());

	float gamma2 = getBlob2Gamma(x);

	gammaSum2 += gamma2;
	gammaProductSum2 += gamma2 * x;
	gammaVarianceSum2 += gamma2 * (x - blob2.getMean()) * (x - blob2.getMean());
}

void BimodalModel::finishUpdate(unsigned int numberDatapoints) {
	calcMaxDelta(numberDatapoints);

	weight1 = gammaSum1 / numberDatapoints;
	weight2 = gammaSum2 / numberDatapoints;

	float newMean1 = gammaProductSum1 / gammaSum1;
	float newMean2 = gammaProductSum2 / gammaSum2;

	blob1.setMean(newMean1);
	blob2.setMean(newMean2);

	float newVariance1 = gammaVarianceSum1 / gammaSum1;
	float newVariance2 = gammaVarianceSum2 / gammaSum2;

	blob1.setVariance(newVariance1);
	blob2.setVariance(newVariance2);

	resetSums();
}

Gaussian BimodalModel::getBlob1() {
	return blob1;
}

Gaussian BimodalModel::getBlob2() {
	return blob2;
}

float BimodalModel::getMaxDelta() {
	return maxDelta;
}

float BimodalModel::getGammaSum1() {
	return gammaSum1;
}

float BimodalModel::getGammaProductSum1() {
	return gammaProductSum1;
}

float BimodalModel::getGammaVarianceSum1() {
	return gammaVarianceSum1;
}

float BimodalModel::getGammaSum2() {
	return gammaSum2;
}

float BimodalModel::getGammaProductSum2() {
	return gammaProductSum2;
}

float BimodalModel::getGammaVarianceSum2() {
	return gammaVarianceSum2;
}

float BimodalModel::getWeight1() {
	return weight1;
}

float BimodalModel::getWeight2() {
	return weight2;
}