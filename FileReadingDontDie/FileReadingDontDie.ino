#include <SD.h>
#include <KDE.h>

//Amount of millisectonds to train for


#define DATACOLLECT 0
#define LEARN 1
#define OPERATE 2
#define RELEARN 3
#define H 0.5
#define MAX_FLOAT_VAL 3.4028235e38
#define MIN_FLOAT_VAL -MAX_FLOAT_VAL

#define H 0.5

#define AMP_PIN A3

const float TRAINING_PERIOD_MINS = 0.5;
const float fPEM_MINS = 1 / 6.0;
const float TRAINING_PERIOD = TRAINING_PERIOD_MINS * 1000 * 60;
const float fPEM = fPEM_MINS * 60 * 1000;

const unsigned int MAX_NUMBER_OF_FILES = floor(TRAINING_PERIOD_MINS / fPEM_MINS);

boolean allowWrite = true;
unsigned int state;
int lastFile = 0;
float lastTime = 0;
File file;
File bam;

long startTime;

long *timeData;
float *ampData;

Kernel *kernels;
float *mins;

unsigned int dataSize;
unsigned int minsSize;

String data;
unsigned int lastTimes;

size_t readField(File* file, char* str, size_t size, char* delim) {
  char ch;
  size_t n = 0;
  while ((n + 1) < size && file->read(&ch, 1) == 1) {
    // Delete CR.
    if (ch == '\r') {
      continue;
    }
    str[n++] = ch;
    if (strchr(delim, ch)) {
      break;
    }
  }
  str[n] = '\0';
  return n;
}
//------------------------------------------------------------------------------

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
        data = String(timeMs - startTime) + "," + String(amps) + '\0';
        writeData("MS_" + String(times - 1) + ".txt", data);
        Serial.println(data);
        data = "";
        Serial.println("RESET");
        lastTimes = times;
        Serial.print("file: ");
        Serial.println(times - 1);
      }
      else {
        writeData("MS_" + String(times) + ".txt", data);
      }
    }
    else {
      data = String(timeMs - startTime) + "," + String(amps) + '\0';
      writeData("MS_" + String(MAX_NUMBER_OF_FILES - 1) + ".txt", data);
      data = "";
      Serial.print("file: ");
      Serial.println(MAX_NUMBER_OF_FILES - 1);
      state = LEARN;
    }
  }
  else if (state == OPERATE) {
    float val = Kernel::kernelConsensus(kernels, dataSize, getAmps());
    if (val > mins[0]) {
      Serial.println("On");
    }
    else {
      Serial.println("Off");
    }
  }
  else if (state == LEARN) {
    dataSize = getNumDataPoints();


    Serial.println(dataSize);

    ampData = (float*) malloc(dataSize * sizeof(float));
    timeData = (long*) malloc(dataSize * sizeof(long));

    getAllData(timeData, ampData, dataSize);

    for (unsigned int i = 0;  i < dataSize; i++) {
      Serial.print("amps: ");
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

    state = 255;
  }
}

void getAllData(long * timeOutput, float * ampOutput, unsigned int listSize) {
  unsigned int tracker = 0;
  for (unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {

    String num = String(i);

    file = SD.open("MS_" + num + ".txt", FILE_READ);

    if (!file) {
      Serial.println("open error opening file MS_" + num + ".txt");
      delay(1000);
      return;
    }
    long x;
    float y;

    while (readVals(&x, &y)) {
      /*Serial.print("time: ");
        Serial.println(x);
        Serial.print("amps: ");
        Serial.println(y);*/
      timeOutput[tracker] = x;
      ampOutput[tracker] = y;
      //Serial.println(y);
      delay(1000); 
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

bool readLine(File & f, char* line, size_t maxLen) {
  for (size_t n = 0; n < maxLen; n++) {
    int c = f.read();
    Serial.println(String(c) + '\0');
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
  Serial.println("too long");
  return false; // line too long
}

bool readVals(long * v1, float * v2) {
  char line[40], *ptr, *str;
  if (!readLine(file, line, sizeof(line))) {
    Serial.println("EOF/toolong");
    return false;  // EOF or too long
  }
  ptr[39] = '\0';
  line[39] = '\0';
  //Serial.println(line);
  delay(1000);

 
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

bool writeData(String fileName, String data) {

  Serial.println("WHABAM");

  File writeFile;
  if (true) {
    writeFile = SD.open(fileName, FILE_WRITE);
  }
  else {
    return false;
  }

  // if the file opened okay, write to it:
  if (writeFile) {
    writeFile.print(data);
    // close the file:
    writeFile.close();
    return true;
  } else {
    writeFile.close();
    return false;
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
    float vOut = (analogRead(AMP_PIN) / 1023) * 5000;
    float current = (vOut);
    amps = abs(current);

    Pc = P + varProccess;
    G = Pc / (Pc + varVolt);
    P = (1 - G) * Pc;
    Xp = Xe;
    Zp = Xp;
    Xe = G * (amps - Zp) + Xp;
  }

  return Xe;
}

int signum(float val) {
  return (0.0 < val) - (val < 0.0);
}
