/*
 * Library for creating Gaussian Mixture Models. Currently only supported for biomodal data.
 * Created by Cole Savage, November 4, 2019.
 */

#ifndef BimodalModelLib
#define BimodalModelLib

#include "GaussianMixtureModel.h"

class BimodalModel {
	public:
		BimodalModel(Gaussian blob1_param, Gaussian blob2_param);

		float getValue(float x);
		void updateModel(float x);
		void finishUpdate(unsigned int numberDatapoints);
		Gaussian getBlob1();
		Gaussian getBlob2();
		float getMaxDelta();
	
	private:
		Gaussian blob1 = Gaussian(0,0);
		Gaussian blob2 = Gaussian(0,0);
		float weight1;
		float weight2;
		float gammaSum1;
		float gammaSum2;
		float gammaProductSum1;
		float gammaProductSum2;
		float gammaVarianceSum1;
		float gammaVarianceSum2;
		float maxDelta;
		
		float getBlob1Gamma(float x);
		float getBlob2Gamma(float x);
		void resetSums();
		void calcMaxDelta(unsigned int numberDatapoints);
};
#endif