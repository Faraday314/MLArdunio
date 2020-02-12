/*
 * Library for creating Gaussian Mixture Models. Currently only supported for biomodal data.
 * Created by Cole Savage, November 4, 2019.
 */

#ifndef BimodalModel
#define BimodalModel
#include <BimodalModel.h>

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
	Gaussian blob1;
	Gaussian blob2;
	
	float weight1;
	float weight2;
	
	float maxDelta;
	
	float getBlob1Gamma(float x);
	float getBlob2Gamma(float x);
	
	void resetSums();
	void calcMaxDelta();
};
#endif
