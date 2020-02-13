/*
 * Library for creating Gaussian Mixture Models. Currently only supported for biomodal data.
 * Created by Cole Savage, November 4, 2019.
 */

#ifndef GaussianMixtureModel
#define GaussianMixtureModel

class Gaussian {
  private:
	float mean;
	float standardDeviation;

  public:
	Gaussian(float mean_param, float standardDeviation_param);

	float getValue(float x);
	float getMean();
	float getStandardDeviation();
	float getVariance();
	void setMean(float newMean);
	void setStandardDeviation(float newStandardDeviation);
	void setVariance(float newVariance);
};

class BimodalModel {
   private:
	Gaussian blob1;
	Gaussian blob2;
	
	float weight1;
	float weight2;

	float maxDelta;

	float getBlob1Gamma(float x);
	float getBlob2Gamma(float x);

	void resetSums();
	void calcMaxDelta();

  public:
	BimodalModel(Gaussian blob1_param, Gaussian blob2_param);
	
	void updateModel(float x);
	void finishUpdate(unsigned int numberDatapoints);
	Gaussian getBlob1();
	Gaussian getBlob2();
	float getMaxDelta();  
}
#endif

