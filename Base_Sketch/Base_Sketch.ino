#include <KDE.h>
#include <math.h>

float *dataPtr;
Kernel *kernelPtr;
int dataSize;
const float H = 0.5;

void setup() {
  // put your setup code here, to run once:
  float data[] = {0.0f,0.1f,0.1f,0.5f,0.5f,0.5f,0.7f,0.7f,1.0f,2.0f,2.1f,2.1f,2.5f,2.5f,2.5f,2.7f,2.7f,3.0f};
  dataSize = sizeof(data)/sizeof(data[0]);
  dataPtr = &data[0];

  Kernel kernels[dataSize];
 
  Serial.begin(9600);

  float *dataPtrCpy = dataPtr;

  //Generate Kernels
  for(unsigned int i = 0; i < dataSize; i++) {   
    kernels[i] = *(new Kernel(*dataPtrCpy, H));
    dataPtrCpy++;
  }


  
};

void loop() {
  // put your main code here, to run repeatedly:  
}

float max(float *arrayStart, unsigned int arrayLength) {
  float *cpyPtr = arrayStart;
  for(i = 1; i < arrayLength; i++) {
  }
}
