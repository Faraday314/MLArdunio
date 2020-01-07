#include <SD.h>
#include <KDE.h>

//Amount of millisectonds to train for
#define TRAINING_PERIOD 2*1000*60
#define fPEM .1*60*1000
#define MAX_NUMBER_OF_FILES floor(2/0.1)

#define DATACOLLECT 0
#define LEARN 1
#define OPERATE 2
#define RELEARN 3
#define H 0.5
#define MAX_FLOAT_VAL 3.4028235e38
#define MIN_FLOAT_VAL -MAX_FLOAT_VAL

#define H 0.5

#define AMP_PIN A3

boolean allowWrite = true;
unsigned int state = DATACOLLECT;
int lastFile = 0;
float lastTime = 0;
File file;

long startTime;

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
  pinMode(AMP_PIN, INPUT);
  startTime = millis();
}
void loop() {
  //Datacollect state is WIP
  if (state == DATACOLLECT) {
    unsigned int times = fmod(floor(round(millis()) / fPEM), MAX_NUMBER_OF_FILES);
    if (millis() - startTime < TRAINING_PERIOD) {
      bool fileExists = SD.exists("MS_" + String(times) + ".txt");
      if (!fileExists) {
        writeData("MS_" + String(times) + ".txt");
      }
    }
  }
  else if (state == LEARN) {
    dataSize = getNumDataPoints();

    ampData = (float*) malloc(dataSize * sizeof(float));
    timeData = (long*) malloc(dataSize * sizeof(long));
    getAllData(timeData, ampData, dataSize);

    float data[] = {0.0f, 0.1f, 0.1f, 0.5f, 0.5f, 0.5f, 0.7f, 0.7f, 1.0f, 2.0f, 2.1f, 2.1f, 2.5f, 2.5f, 2.5f, 2.7f, 2.7f, 3.0f};

    for (unsigned int i = 0;  i < dataSize; i++) {
      Serial.print("amps: ");
      ampData[i] = data[i];
      Serial.println(ampData[i]);
    }

    kernels = (Kernel*) malloc(dataSize * sizeof(Kernel));

    for (unsigned int i = 0; i < dataSize; i++) {
      kernels[i] = *(new Kernel(ampData[i], H));
    }

    findMin(kernels, dataSize, 0.0, 3.0, 0.1, 0.01);

    for (unsigned int i = 0;  i < minsSize; i++) {
      Serial.print("divider: ");
      Serial.println(mins[i]);
    }

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
    File dataFile = SD.open("MS_" + String(i) + ".txt", FILE_READ);
    if (dataFile) {
      while (dataFile.available()) {
        if (dataFile.read() == ',') {
          numDataPoints++;
        }
      }
      dataFile.close();
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

void findMin(Kernel *kernels, unsigned int dataSize, float lowerBound, float upperBound, float algStep, float lossThreshold) {
  float currentX = lowerBound;
  float lastValue = Kernel::kernelConsensus(kernels, dataSize, currentX);
  currentX += algStep / 2;
  float currentValue = Kernel::kernelConsensus(kernels, dataSize, currentX);
  float delta = currentValue - lastValue;

  while (delta == 0.0) {
    currentX += algStep;
    lastValue = currentValue;
    currentValue = Kernel::kernelConsensus(kernels, dataSize, currentX);
    delta = currentValue - lastValue;
  }

  currentX += algStep;
  lastValue = currentValue;
  currentValue = Kernel::kernelConsensus(kernels, dataSize, currentX);
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

    currentValueCpy = Kernel::kernelConsensus(kernels, dataSize, currentXCpy);
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

    currentValue = Kernel::kernelConsensus(kernels, dataSize, currentX);
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
    lastValue = Kernel::kernelConsensus(kernels, dataSize, maxRanges[i] - algStep);
    float mid = (2 * maxRanges[i] - algStep) / 2;
    currentValue = Kernel::kernelConsensus(kernels, dataSize, mid);
    float lBound = maxRanges[i] - algStep;
    float uBound = maxRanges[i];
    while (abs(currentValue - lastValue) > lossThreshold) {
      float rmid = (mid + uBound) / 2;
      float lmid = (lBound + mid) / 2;

      float rmidVal = Kernel::kernelConsensus(kernels, dataSize, rmid);
      float lmidVal = Kernel::kernelConsensus(kernels, dataSize, lmid);

      mid = rmidVal > lmidVal ? lmid : rmid;

      lastValue = currentValue;
      currentValue = Kernel::kernelConsensus(kernels, dataSize, mid);
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

  mins = (float*) malloc(usedVals * sizeof(float));
  minsSize = usedVals;

  for (unsigned int i = 0; i < usedVals; i++) {
    mins[i] = dividers[i];
  }
}

bool writeData(String fileName) {

  int amps;
  const float varVolt = .00583;
  const float varProccess = 5e-6;
  float Pc = 0;
  float G = 0;
  float P = 1;
  float Xp = 0;
  float Zp = 0;
  float Xe = 0;
  for (int i = 0; i < 50; i++) {
    amps = abs(511.5 - analogRead(AMP_PIN));

    Pc = P + varProccess;
    G = Pc / (Pc + varVolt);
    P = (1 - G) * Pc;
    Xp = Xe;
    Zp = Xp;
    Xe = G * (amps - Zp) + Xp;
  }

  File writeFile;
  if (digitalRead(10) == HIGH) {
    writeFile = SD.open(fileName, FILE_WRITE);
  }
  else {
    return false;
  }

  // if the file opened okay, write to it:
  if (writeFile) {
    writeFile.println(String(millis()) + "," + String(Xe));
    // close the file:
    writeFile.close();
    return true;
  } else {
    writeFile.close();
    return false;
  }
}

int signum(float val) {
  return (0.0 < val) - (val < 0.0);
}
