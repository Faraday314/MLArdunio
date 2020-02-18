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
	float deltas[6] = {
		abs(weight1 - gammaSum1 / numberDatapoints),
		abs(weight2 - gammaSum2 / numberDatapoints),

		abs(blob1.getMean() - gammaProductSum1 / gammaSum1),
		abs(blob2.getMean() - gammaProductSum2 / gammaSum2),

		abs(blob1.getVariance() - gammaVarianceSum1 / gammaSum1),
		abs(blob2.getVariance() - gammaVarianceSum2 / gammaSum2)
	};

	float maxDeltaCalculated = 0;
	for (unsigned int i = 0; i < 6; i++) {
		if (deltas[i] > maxDelta) {
			maxDeltaCalculated = deltas[i];
		}
	}

	maxDelta = maxDeltaCalculated;
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
	float newMean2 = gammaVarianceSum2 / gammaSum2;

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