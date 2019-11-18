#include <KDE.h>
#include <math.h>

float *dataPtr;
Kernel *kernelPtr;
int dataSize;
const float H = 0.5;
const float MAX_FLOAT_VAL = 3.4028235e38;
const float MIN_FLOAT_VAL = -MAX_FLOAT_VAL;

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
  Kernel *kernelPtr = &kernels[0];  
  float *outputPtr;

  findMin(kernelPtr, dataSize, 0.0, 3.0, 0.1, 0.01, outputPtr);
  
};

void loop() {
  // put your main code here, to run repeatedly:  
}

void findMin(Kernel *kernelPtr, unsigned int dataSize, float lowerBound, float upperBound, float algStep, float lossThreshold, float *outputPtr) {
  Kernel *kernelPtrCpy = kernelPtr;
  float currentX = lowerBound;
  float lastValue = Kernel::kernelConsensus(kernelPtr, dataSize, currentX);
  currentX += algStep/2;
  float currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, currentX);
  float delta = currentValue - lastValue;

  while(delta == 0.0) {
    currentX += algStep;
    lastValue = currentValue;
    currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, currentX);
    delta = currentValue - lastValue;
  }

  currentX += algStep;
  lastValue = currentValue;
  currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, currentX);
  delta = currentValue - lastValue;

  int lastSgn = signum(delta);
  int sgn = lastSgn;

  unsigned int arrSize = 0;

  float currentXCpy = currentX;
  float lastSgnCpy = lastSgn;
  float sgnCpy = sgn;
  float lastValueCpy = lastValue;
  float currentValueCpy = currentValue;
  
  while(currentXCpy < upperBound) {
    lastSgnCpy = sgnCpy;
    currentXCpy += algStep;
    lastValueCpy = currentValueCpy;

    currentValueCpy = Kernel::kernelConsensus(kernelPtr, dataSize, currentXCpy);
    delta = currentValueCpy - lastValueCpy;
    sgnCpy = signum(delta);

    if(sgnCpy - lastSgnCpy > 0) {
      arrSize++;
    }
  }

  float minRanges[arrSize];
  float maxRanges[arrSize];

  unsigned int index = 0;

  while(currentX < upperBound) {
    lastSgn = sgn;
    currentX += algStep;
    lastValue = currentValue;

    currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, currentX);
    delta = currentValue - lastValue;
    sgn = signum(delta);

    if(sgn - lastSgn > 0) {
      minRanges[index] = currentX-algStep;
      maxRanges[index] = currentX;
      index++;
    }
  }

  Serial.println(maxRanges[0]);
}

float maxArray(float *arrayStart, unsigned int arrayLength) {
  float *cpyPtr = arrayStart;
  float maxVal = MIN_FLOAT_VAL;
  for(unsigned int i = 0; i < arrayLength; i++) {
    maxVal = max(*cpyPtr, maxVal);
    cpyPtr++;
  }
  return maxVal;
}

float minArray(float *arrayStart, unsigned int arrayLength) {
  float *cpyPtr = arrayStart;
  float minVal = MAX_FLOAT_VAL;
  for(unsigned int i = 0; i < arrayLength; i++) {
    minVal = min(*cpyPtr, minVal);
    cpyPtr++;
  }
  return minVal;
}

void cpyArrayAndAdd(float *arrInPtr, unsigned int arrInSize, float valToAdd, float *outputPtr) {
  float *cpyArrInPtr = arrInPtr;
  float *cpyArrOutPtr = outputPtr;
  for(int i = 0; i < arrInSize; i++) {
    *cpyArrOutPtr = *cpyArrInPtr;
    cpyArrInPtr++;
    cpyArrOutPtr++;
  }
  *cpyArrOutPtr = valToAdd;
}

int signum(float val) {
    return (0.0 < val) - (val < 0.0);
}
