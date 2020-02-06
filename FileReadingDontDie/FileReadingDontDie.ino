#include <SD.h>
#include <KDE.h>

//Amount of millisectonds to train for


#define DATACOLLECT 0
#define LEARN 1
#define OPERATE 2
#define RELEARN 3
#define H 5.5
#define MAX_FLOAT_VAL 3.4028235e38
#define MIN_FLOAT_VAL -MAX_FLOAT_VAL

#define ON true
#define OFF false

#define AMP_PIN A3

const float TRAINING_PERIOD_MINS = 0.5;
const float fPEM_MINS = 1 / 6.0;
const float TRAINING_PERIOD = TRAINING_PERIOD_MINS * 1000 * 60;
const float fPEM = fPEM_MINS * 60 * 1000;

float maxAmps;
float minAmps;

char ampDataPt[30];
char timeDataPt[30];

const unsigned int MAX_NUMBER_OF_FILES = floor(TRAINING_PERIOD_MINS / fPEM_MINS);

boolean allowWrite = true;
unsigned int state;
int lastFile = 0;
float lastTime = 0;
boolean resetFileWrite = true;
File file;
File bam;
File file2;

long startTime;

float divider;

long *timeData;
float *ampData;

Kernel *kernels;
float *mins;

unsigned int dataSize;
unsigned int minsSize;

String data;
unsigned int lastTimes;

float ampPt;
long timePt;
long prevTime;

unsigned int charTrackerTime;
unsigned int charTrackerAmps;

void setup() {
  Serial.begin(9600);
  if (!SD.begin(4)) {
    Serial.println("begin error");
    return;
  }

  pinMode(AMP_PIN, INPUT);
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  /*for (unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {
    SD.remove("MS_" + String(i) + ".txt");
    }*/
  data = "";
  lastTimes = 0;
  startTime = millis();
  state = LEARN;
}
void loop() {

  //Datacollect state is WIP
  if (state == DATACOLLECT) {
    long timeMs = millis();
    float amps = getAmps();

    unsigned int times = fmod((timeMs - startTime) / round(fPEM), MAX_NUMBER_OF_FILES);
    Serial.print(F("delta T: "));
    Serial.print(timeMs - startTime);
    Serial.print(F(", "));
    Serial.print(F("lim: "));
    Serial.println(TRAINING_PERIOD);

    if (timeMs - startTime < TRAINING_PERIOD) {
      data = String(timeMs - startTime) + ',' + String(amps) + '\r' + '\n';

      //Serial.println(data);
      if (times != lastTimes) {
        data = String(timeMs - startTime) + ',' + String(amps);
        appendToFile("MS_" + String(times - 1) + ".txt", data, true);
        Serial.println(data);
        data = "";
        Serial.println(F("RESET"));
        lastTimes = times;
        Serial.print(F("file: "));
        Serial.println(times - 1);
      }
      else {
        appendToFile("MS_" + String(times) + ".txt", data, false);
      }
    }
    else {
      data = String(timeMs - startTime) + "," + String(amps);
      appendToFile("MS_" + String(MAX_NUMBER_OF_FILES - 1) + ".txt", data, true);
      data = "";
      Serial.print("file: ");
      Serial.println(MAX_NUMBER_OF_FILES - 1);
      state = LEARN;
    }
  }
  else if (state == OPERATE) {

    charTrackerTime = 0;
    charTrackerAmps = 0;

    bool command;
    bool foundTimestamp;
    bool getTheAmpVal;
    foundTimestamp = false;
    getTheAmpVal = false;
    command = OFF;
    prevTime = 0;

    float deltaT = fmod((millis() - startTime), TRAINING_PERIOD);
    bool reachedComma = false;
    char c;
    for (unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {

      if(foundTimestamp) {
        break;
      }

      File f;
      f = SD.open("MS_" + String(i) + ".txt", FILE_READ);

      while (c = f.read()) {
        if (c < 0 && getTheAmpVal) {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';
          foundTimestamp = true;
          break;
        }
        else if(c < 0) {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';
          break;
        }
        if (c == '\n' && getTheAmpVal) {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';
          charTrackerAmps = 0;
          foundTimestamp = true;

          ampPt = atof(ampDataPt);
          break;
        }
        else if(c == '\n') {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';
        }
        else if (c == '\r') {
          reachedComma = false;
          continue;
        }
        else if (c == ',') {
          reachedComma = true;
          timeDataPt[charTrackerTime] = '\0';
          charTrackerTime = 0;

          timePt = atol(timeDataPt);
          
          if(deltaT >= prevTime && deltaT < timePt) {
            Serial.print(F("prev: "));
            Serial.print(prevTime);
            Serial.print(F(" deltaT: "));
            Serial.print(deltaT);
            Serial.print(F(" timePt: "));
            Serial.println(timePt);
            prevTime = timePt;
            getTheAmpVal = true;
          }
          
          continue;
        }

        if (reachedComma) {
          ampDataPt[charTrackerAmps] = c;
          charTrackerAmps++;
        }
        else {
          timeDataPt[charTrackerTime] = c;
          charTrackerTime++;
        }
      }
      f.close();
    }

    //Serial.print(F("prevTime: "));
    //Serial.print(prevTime);
    //Serial.print(F(" time: "));
    //Serial.print(deltaT);
    //Serial.print(F(" amps: "));
    //Serial.println(ampPt);
    delay(1000);
  }
  else if (state == LEARN) {

    //dataSize = getNumDataPoints();

    //dataSize = 31;

    //ampData = (float*) malloc(10 * sizeof(float));
    //timeData = (long*) malloc(10 * sizeof(long));

    charTrackerTime = 0;
    charTrackerAmps = 0;

    maxAmps = 0;
    minAmps = MAX_FLOAT_VAL;

    bool reachedComma = false;
    char c;
    for (unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {
      File f;
      f = SD.open("MS_" + String(i) + ".txt");

      while (c = f.read()) {
        if (c < 0) {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';
          break;
        }
        if (c == '\n') {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';
          charTrackerAmps = 0;
  
          ampPt = atof(ampDataPt);

          if (ampPt > maxAmps) {
            maxAmps = ampPt;
          }
          if (ampPt < minAmps) {
            minAmps = ampPt;
          }

          continue;
        }
        else if (c == '\r') {
          reachedComma = false;
          continue;
        }
        else if (c == ',') {
          reachedComma = true;
          timeDataPt[charTrackerTime] = '\0';
          charTrackerTime = 0;

          continue;
        }

        if (reachedComma) {
          ampDataPt[charTrackerAmps] = c;
          charTrackerAmps++;
        }
        else  {
          timeDataPt[charTrackerTime] = c;
          charTrackerTime++;
        }
      }
      f.close();
    }
    
    Serial.print(F("min: "));
    Serial.print(minAmps);
    Serial.print(F(" max: "));
    Serial.print(maxAmps);

    divider = (maxAmps + minAmps) / 2.0;
    divider -= 2;

    Serial.print(F(" divider: "));
    Serial.println(divider);
    
    startTime = millis();
    prevTime = 0;

    state = OPERATE;
  }
}


int availableMemory()
{
  int size = 8192;
  byte *buf;
  while ((buf = (byte *) malloc(--size)) == NULL);
  free(buf);
  return size;
}

void getAllData(long * timeOutput, float * ampOutput, unsigned int listSize) {
  unsigned int tracker;
  tracker = 0;
  long x;
  float y;

  File f;
  f = SD.open("MS_0.txt", FILE_READ);
  String timeDataPt;
  String ampDataPt;
  timeDataPt = "";
  ampDataPt = "";
  bool reachedComma = false;
  char c;
  while (c = f.read()) {
    if (c < 0) {
      reachedComma = false;
      ampDataPt += '\0';
      Serial.print(" amps: ");
      Serial.println(ampDataPt);
      ampDataPt = "";
      break;
    }
    if (c == '\n') {
      reachedComma = false;
      ampDataPt += '\0';

      float ampPt;
      ampPt = ampDataPt.toFloat();
      ampOutput[tracker] = ampPt;
      tracker++;
      Serial.println(" amps: " + String(ampPt));
      ampDataPt = "";
      continue;
    }
    else if (c == '\r') {
      reachedComma = false;
      continue;
    }
    else if (c == ',') {
      reachedComma = true;
      timeDataPt += '\0';
      Serial.print("time: ");
      Serial.print(timeDataPt);
      timeDataPt = "";
      continue;
    }

    if (reachedComma) {
      ampDataPt += c;
    }
    else  {
      timeDataPt += c;
    }
  }
  //Serial.println(timeDataPt);
}
unsigned int getNumDataPoints() {
  unsigned int numDataPoints = 0;

  for (int i = 0; i < MAX_NUMBER_OF_FILES; i++) {
    File dataFile = SD.open("MS_" + String(i) + ".txt", FILE_READ);
    if (!dataFile) {
      Serial.println("error opening MS_" + String(i) + ".txt");
      return;
    }
    while (dataFile.available()) {
      if (dataFile.read() == ',') {
        numDataPoints++;
      }
    }
    dataFile.close();
  }

  return numDataPoints;
}

bool readLine(File & f, char* line, size_t maxLen) {
  for (size_t n = 0; n < maxLen; n++) {
    int c;
    c = f.read();
    if ( c < 0 && n == 0) {
      Serial.println("EOF");
      return false;  // EOF
    }
    if (c < 0 || c == '\n') {
      line[n] = 0;
      return true;
    }
    line[n] = c;
  }
  return false; // line too long
}

bool readVals(long * v1, float * v2) {
  char line[40], *ptr, *str;
  if (!readLine(file, line, sizeof(line))) {
    return false;  // EOF or too long
  }

  *v1 = strtol(line, &ptr, 10);

  if (ptr == line) {
    Serial.println("bad number");
    return false;  // bad number if equal
  }

  while (*ptr) {
    if (*ptr++ == ',') break;
  }
  *v2 = (float) strtod(ptr, &str);
  return str != ptr;  // true if number found
}

void findMin(Kernel * kernels, unsigned int dataSize, float lowerBound, float upperBound, float algStep, float lossThreshold) {
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

float getAmps() {
  float amps;
  const float varVolt = .00583;
  const float varProccess = 5e-6;
  float Pc = 0;
  float G = 0;
  float P = 1;
  float Xp = 0;
  float Zp = 0;
  float Xe = 0;
  for (int i = 0; i < 50; i++) {
    float vOut = (analogRead(AMP_PIN) / 1023.0) * 5000;
    float current = (vOut);
    float shift = 2500 - current;
    amps = abs(shift);

    Pc = P + varProccess;
    G = Pc / (Pc + varVolt);
    P = (1 - G) * Pc;
    Xp = Xe;
    Zp = Xp;
    Xe = G * (amps - Zp) + Xp;
  }

  return Xe;
}

void appendToFile(String filePath, String data, boolean closeFile) {
  if (resetFileWrite) {
    file2 = SD.open(filePath, FILE_WRITE);
    resetFileWrite = false;
  }
  file2.print(data);
  if (closeFile) {
    file2.close();
    resetFileWrite = true;
  }
}

int signum(float val) {
  return (0.0 < val) - (val < 0.0);
}
