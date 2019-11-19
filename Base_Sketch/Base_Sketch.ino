#include <KDE.h>
#include <math.h>

float *dataPtr;
Kernel *kernelPtr;
int dataSize;
const float H = 0.5;
const float MAX_FLOAT_VAL = 3.4028235e38;
const float MIN_FLOAT_VAL = -MAX_FLOAT_VAL;

float output[32];

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
  unsigned int outputSize = findMin(kernelPtr, dataSize, 0.0, 3.0, 0.1, 0.01);
  for(int i = 0; i < outputSize; i++) {
    Serial.println(output[i]);
  }
};

void loop() {
  // put your main code here, to run repeatedly:  
}

unsigned int findMin(Kernel *kernelPtr, unsigned int dataSize, float lowerBound, float upperBound, float algStep, float lossThreshold) {
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

  //Find how many mins there are
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
  
  float maxRanges[arrSize];
  unsigned int index = 0;

  //Find the upper bound of all the mins
  while(currentX < upperBound) {
    lastSgn = sgn;
    currentX += algStep;
    lastValue = currentValue;

    currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, currentX);
    delta = currentValue - lastValue;
    sgn = signum(delta);

    if(sgn - lastSgn > 0) {
      maxRanges[index] = currentX;
      index++;
    }
  }

  boolean lastMidFloored = false;
  float dividers[arrSize];
  float flooredDividers[arrSize];
  unsigned int usedVals = 0;
  unsigned int usedFloorVals = 0;

  //Refine the estimated mins
  for(unsigned int i = 0; i < arrSize; i++) {
    lastValue = Kernel::kernelConsensus(kernelPtr, dataSize, maxRanges[i]-algStep);
    float mid = (2*maxRanges[i] - algStep)/2;
    currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, mid);
    float lBound = maxRanges[i] - algStep;
    float uBound = maxRanges[i];
    while(abs(currentValue-lastValue) > lossThreshold) {
      float rmid = (mid + uBound)/2;
      float lmid = (lBound + mid)/2;

      float rmidVal = Kernel::kernelConsensus(kernelPtr, dataSize, rmid);
      float lmidVal = Kernel::kernelConsensus(kernelPtr, dataSize, lmid);

      mid = rmidVal > lmidVal ? lmid : rmid;

      lastValue = currentValue;
      currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, mid);
    }
    if(abs(currentValue - lastValue) == 0) {
      flooredDividers[usedFloorVals] = mid;
      usedFloorVals++;
      lastMidFloored = true;
    }
    else if(lastMidFloored) {
      dividers[usedVals] = (flooredDividers[usedFloorVals - 1] + mid)/2;
      usedVals++;
      lastMidFloored = false;
    }
    else {
      dividers[usedVals] = mid;
      usedVals++;
    }
  }

  if(usedVals < 32) {
    for(int i = 0; i < usedVals; i++) {
      output[i] = dividers[i];
    }
  }
  else {
    Serial.println("If you are reading this, everything has gone horribly wrong and you should give up. Good luck!");
  }

  return usedVals;
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

int signum(float val) {
    return (0.0 < val) - (val < 0.0);
}
