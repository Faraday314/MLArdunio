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

#define AMP_PIN A3

const float TRAINING_PERIOD_MINS = 0.5;
const float fPEM_MINS = 1 / 6.0;
const float TRAINING_PERIOD = TRAINING_PERIOD_MINS * 1000 * 60;
const float fPEM = fPEM_MINS * 60 * 1000;

char timeDataPt[8];
char ampDataPt[8];

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

long *timeData;
float *ampData;

Kernel *kernels;
float *mins;

unsigned int dataSize;
unsigned int minsSize;

String data;
unsigned int lastTimes;

float ampPt;


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
    Serial.print("delta T: ");
    Serial.print(timeMs - startTime);
    Serial.print(", ");
    Serial.print("lim: ");
    Serial.println(TRAINING_PERIOD);

    if (timeMs - startTime < TRAINING_PERIOD) {
      data = String(timeMs - startTime) + "," + String(amps) + '\r' + '\n';

      //Serial.println(data);
      if (times != lastTimes) {
        data = String(timeMs - startTime) + "," + String(amps);
        appendToFile("MS_" + String(times - 1) + ".txt", data, true);
        Serial.println(data);
        data = "";
        Serial.println("RESET");
        lastTimes = times;
        Serial.print("file: ");
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
    float amps;
    amps = getAmps();
    Serial.print("amps: ");
    Serial.print(amps);
    Serial.print(" state: ");

    if (amps > mins[0]) {
      Serial.println("On");
    }
    else {
      Serial.println("Off");
    }
  }
  else if (state == LEARN) {

    dataSize = getNumDataPoints();

    //dataSize = 31;

    ampData = (float*) malloc(10 * sizeof(float));
    timeData = (long*) malloc(10 * sizeof(long));

    unsigned int tracker;
    unsigned int charTrackerTime;
    unsigned int charTrackerAmps;
    tracker = 0;
    charTrackerTime = 0;
    charTrackerAmps = 0;

    File f;
    f = SD.open("MS_0.txt", FILE_READ);
    
    bool reachedComma = false;
    char c;
    while ((c = f.read()) && tracker < 10) {
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

        //free(ampDataPt);

        ampData[tracker] = ampPt;


        Serial.print("amps: ");
        Serial.print(ampPt);
        Serial.print(" mem: ");
        int mem;
        mem = availableMemory();
        Serial.println(mem);
        tracker++;

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

    /*ampData[0] = 10.36;
      ampData[1] = 9.00;
      ampData[2] = 8.41;
      ampData[3] = 7.48;
      ampData[4] = 8.85;
      ampData[5] = 19.11;
      ampData[6] = 17.88;
      ampData[7] = 15.55;
      ampData[8] = 15.19;
      ampData[9] = 18.04;
      ampData[10] = 6.23;
      ampData[11] = 6.06;
      ampData[12] = 8.08;
      ampData[13] = 8.75;
      ampData[14] = 6.64;
      ampData[15] = 22.27;
      ampData[16] = 19.25;
      ampData[17] = 17.47;
      ampData[18] = 17.88;
      ampData[19] = 6.44;
      ampData[20] = 7.98;
      ampData[21] = 22.63;
      ampData[22] = 18.68;
      ampData[23] = 22.82;
      ampData[24] = 19.73;
      ampData[25] = 9.34;
      ampData[26] = 7.76;
      ampData[27] = 7.00;
      ampData[28] = 18.25;
      ampData[29] = 7.59;
      ampData[30] = 6.56;*/
    for (unsigned int i = 0;  i < 10; i++) {
      float amp;
      amp = ampData[i];

      Serial.print("amps: ");
      Serial.println(amp);
    }
    //getAllData(timeData, ampData, dataSize);
    /*

        float maxAmps = 0;
        for (unsigned int i = 0;  i < dataSize; i++) {
          float amp;
          amp = ampData[i];
          if (amp > maxAmps) {
            maxAmps = amp;
          }
          Serial.print("max: ");
          Serial.print(maxAmps);
          Serial.print(" amps: ");
          Serial.println(amp);
        }
        delay(1000);

        kernels = (Kernel*) malloc(dataSize * sizeof(Kernel));

        for (unsigned int i = 0; i < dataSize; i++) {
          float ampDataPt;
          ampDataPt = ampData[i];
          Kernel k = Kernel(ampDataPt, H);
          kernels[i] = k;
        }
        findMin(kernels, dataSize, 0.0, maxAmps, 0.1, 0.01);
        for (unsigned int i = 0;  i < minsSize; i++) {
          Serial.print("divider: ");
          Serial.println(mins[i]);
        }
    */
    state = 255;
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

  for (unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {

    /*
        Serial.println(String(i) + '\0');
        Serial.println(String(MAX_NUMBER_OF_FILES) + '\0');

        file = SD.open("MS_" + String(i) + ".txt", FILE_READ);

        if (!file) {
          Serial.println("open error opening file MS_" + String(i) + ".txt");
          delay(1000);
          return;
        }
        while (readVals(&x, &y)) {
          timeOutput[tracker] = x;
          ampOutput[tracker] = y;
          tracker++;
        }
        delay(1000);
        file.close();*/
  }
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
