#include <SD.h>
#include <KDE.h>

#define MAX_NUMBER_OF_FILES 3
#define DATACOLLECT 0
#define LEARN 1
#define OPERATE 2
#define RELEARN 3
#define fPEM .1*60*1000
#define H 0.5
#define MAX_FLOAT_VAL 3.4028235e38
#define MIN_FLOAT_VAL -MAX_FLOAT_VAL

#define H 0.5

boolean allowWrite = true;
unsigned int state = LEARN;
int lastFile = 0;
float lastTime = 0;
File file;
File myFile;
File myFile2;

long *timeData;
float *ampData;

Kernel *kernels;
float *mins;

unsigned int dataSize;
unsigned int minsSize;

void setup() {

  Serial.begin(9600);
  if (!SD.begin(4)) {
    Serial.println("begin error");
    return;
  }
}
void loop() {
  //Datacollect state is WIP
  if (state == DATACOLLECT) {
    unsigned int times = fmod(floor(round(millis()) / fPEM), MAX_NUMBER_OF_FILES);
    Serial.println(times);
    if (allowWrite) {
      //If anything is typed in the serial monitor stop writing
      if (Serial.read() != -1) {
        allowWrite = false;
      }
      if (lastFile != times) {
        myFile.close();

        lastFile = times;

        if (SD.exists("ms_" + (String) times + ".txt")) {
          SD.remove("ms_" + (String) times + ".txt");
        }

        myFile = SD.open("ms_" + (String) times + ".txt", FILE_WRITE);
      }
      else {
        if (myFile) {
          Serial.println("Wrote to file");
          myFile.println("time = " + (String) round(millis()));
        } else {
          Serial.println((String) "error opening " + (String) "mills: " + (String) times + (String) ".txt");
        }
      }
    }
    else {
      myFile.close();
    }
  }
  else if (state == LEARN) {
    dataSize = getNumDataPoints();

    ampData = (float*) malloc(dataSize * sizeof(float));
    timeData = (long*) malloc(dataSize * sizeof(long));
    getAllData(timeData, &ampData[0], dataSize);

    kernels = (Kernel*) malloc(dataSize * sizeof(Kernel));

    for (unsigned int i = 0; i < dataSize; i++) {
      kernels[i] = *(new Kernel(ampData[i], H));
    }


    for(unsigned int i = 0;  i< dataSize; i++) {
      Serial.print("time: ");
      Serial.println(timeData[i]);
    }
    //findMin(kernels, dataSize, 0.0, 3.0, 0.1, 0.01);

    state = OPERATE;
  }
}

void getAllData(long *timeOutput, float *ampOutput, unsigned int listSize) {
  unsigned int tracker = 0;
  for (unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {

    file = SD.open("MS_" + String(i) + ".txt", FILE_READ);
    if (!file) {
      Serial.println("open error");
      return;
    }
    long x;
    float y;
    while (readVals(&x, &y)) {
      timeOutput[tracker] = x;
      ampOutput[tracker] = y;
      tracker++;
    }
    file.close();
  }
}

unsigned int getNumDataPoints() {
  unsigned int numDataPoints = 0;

  for (int i = 0; i < MAX_NUMBER_OF_FILES; i++) {
    myFile2 = SD.open("MS_" + String(i) + ".txt", FILE_READ);
    if (myFile2) {
      while (myFile2.available()) {
        if (myFile2.read() == ',') {
          numDataPoints++;
        }
      }
      myFile2.close();
    }
    else {
      Serial.println("error opening MS_" + (String) i + ".txt");
    }
  }
  return numDataPoints;
}

bool readLine(File &f, char* line, size_t maxLen) {
  for (size_t n = 0; n < maxLen; n++) {
    int c = f.read();
    if ( c < 0 && n == 0) return false;  // EOF
    if (c < 0 || c == '\n') {
      line[n] = 0;
      return true;
    }
    line[n] = c;
  }
  return false; // line too long
}

bool readVals(long* v1, float* v2) {
  char line[40], *ptr, *str;
  if (!readLine(file, line, sizeof(line))) {
    return false;  // EOF or too long
  }
  *v1 = strtol(line, &ptr, 10);
  if (ptr == line) return false;  // bad number if equal
  while (*ptr) {
    if (*ptr++ == ',') break;
  }
  *v2 = (float) strtod(ptr, &str);
  return str != ptr;  // true if number found
}

/*void findMin(Kernel *kernels, float *mins, unsigned int dataSize, float lowerBound, float upperBound, float algStep, float lossThreshold) {
  float currentX = lowerBound;
  float lastValue = Kernel::kernelConsensus(kernels, dataSize, currentX);
  currentX += algStep / 2;
  float currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, currentX);
  float delta = currentValue - lastValue;

  while (delta == 0.0) {
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
  while (currentXCpy < upperBound) {
    lastSgnCpy = sgnCpy;
    currentXCpy += algStep;
    lastValueCpy = currentValueCpy;

    currentValueCpy = Kernel::kernelConsensus(kernelPtr, dataSize, currentXCpy);
    delta = currentValueCpy - lastValueCpy;
    sgnCpy = signum(delta);

    if (sgnCpy - lastSgnCpy > 0) {
      arrSize++;
    }
  }

  float maxRanges[arrSize];
  unsigned int index = 0;

  //Find the upper bound of all the mins
  while (currentX < upperBound) {
    lastSgn = sgn;
    currentX += algStep;
    lastValue = currentValue;

    currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, currentX);
    delta = currentValue - lastValue;
    sgn = signum(delta);

    if (sgn - lastSgn > 0) {
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
  for (unsigned int i = 0; i < arrSize; i++) {
    lastValue = Kernel::kernelConsensus(kernelPtr, dataSize, maxRanges[i] - algStep);
    float mid = (2 * maxRanges[i] - algStep) / 2;
    currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, mid);
    float lBound = maxRanges[i] - algStep;
    float uBound = maxRanges[i];
    while (abs(currentValue - lastValue) > lossThreshold) {
      float rmid = (mid + uBound) / 2;
      float lmid = (lBound + mid) / 2;

      float rmidVal = Kernel::kernelConsensus(kernelPtr, dataSize, rmid);
      float lmidVal = Kernel::kernelConsensus(kernelPtr, dataSize, lmid);

      mid = rmidVal > lmidVal ? lmid : rmid;

      lastValue = currentValue;
      currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, mid);
    }
    if (abs(currentValue - lastValue) == 0) {
      flooredDividers[usedFloorVals] = mid;
      usedFloorVals++;
      lastMidFloored = true;
    }
    else if (lastMidFloored) {
      dividers[usedVals] = (flooredDividers[usedFloorVals - 1] + mid) / 2;
      usedVals++;
      lastMidFloored = false;
    }
    else {
      dividers[usedVals] = mid;
      usedVals++;
    }
  }

  if (usedVals < 32) {
    for (int i = 0; i < usedVals; i++) {
      output[i] = dividers[i];
    }
  }
  else {
    Serial.println("If you are reading this, everything has gone horribly wrong and you should give up. Good luck!");
  }

  return usedVals;
}*/
/*
  void copy(long* src, long* dst, unsigned int len) {
  memcpy(dst, src, sizeof(src[0])*len);
  }
  void copy(float* src, float* dst, unsigned int len) {
  memcpy(dst, src, sizeof(src[0])*len);
  }*/
