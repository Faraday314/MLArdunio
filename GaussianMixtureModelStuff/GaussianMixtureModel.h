/*
 * Library for creating Gaussian Mixture Models. Currently only supported for biomodal data.
 * Created by Cole Savage, November 4, 2019.
 */

#ifndef GaussianMixtureModel
#define GaussianMixtureModel

class Gaussian {
	public:
		Gaussian(float mean_param, float standardDeviation_param);
	
		float getValue(float x);
		float getMean();
		float getStandardDeviation();
		float getVariance();
		void setMean(float newMean);
		void setStandardDeviation(float newStandardDeviation);
		void setVariance(float newVariance);
	
	private:
		float mean;
		float standardDeviation;
};
#endif