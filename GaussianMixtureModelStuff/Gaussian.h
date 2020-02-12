/*
 * Library for creating Gaussian Mixture Models. Currently only supported for biomodal data.
 * Created by Cole Savage, November 4, 2019.
 */

#ifndef Gaussian
#define Gaussian

class Gaussian {
  public:
	Gaussian(float mean_param, float standardDeviation_param);
	
	float getValue(float x);
	float getMean();
	float getStandardDeviation();
	float getVariance();
	void setMean();
	void setStandardDeviation();
	void setVariance();
	
  private:
	float mean;
	float standardDeviation;
};
#endif
