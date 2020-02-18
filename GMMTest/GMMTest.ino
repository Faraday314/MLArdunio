#include <GaussianMixtureModel.h>
#include <BimodalModelLib.h>

void setup() {
  // put your setup code here, to run once:
  Gaussian g = Gaussian(15,1);
  Gaussian h = Gaussian(16,1);

  BimodalModel m = BimodalModel(g,h);
}

void loop() {
  // put your main code here, to run repeatedly:

}
