#include <SD.h>
#include <GaussianMixtureModel.h>
#include <BimodalModelLib.h>

//Amount of millisectonds to train for


#define DATACOLLECT 0
#define LEARN 1
#define OPERATE 2
#define RELEARN 3
#define MAX_FLOAT_VAL 3.4028235e38
#define MIN_FLOAT_VAL -MAX_FLOAT_VAL

#define ON true
#define OFF false

#define AMP_PIN A3
#define RELAY_PIN 8
#define CHIP_SELECT 4

#define DELTA_THRESH 0.05

const float TRAINING_PERIOD_MINS = 0.5;
const float fPEM_MINS = 1 / 6.0;
const float TRAINING_PERIOD = TRAINING_PERIOD_MINS * 1000 * 60;
const float fPEM = fPEM_MINS * 60 * 1000;

char ampDataPt[30];
char timeDataPt[30];

const unsigned int MAX_NUMBER_OF_FILES = floor(TRAINING_PERIOD_MINS / fPEM_MINS);

unsigned int state;
int lastFile = 0;
float lastTime = 0;
boolean allowWrite = true;
boolean resetFileWrite = true;

long startTime;

float divider;

unsigned int dataSize;
unsigned int minsSize;

String data;
unsigned int lastTimes;

float ampPt;
long timePt;
long prevTime;

unsigned int charTrackerTime;
unsigned int charTrackerAmps;

File file;

bool deleteFiles = false;

void setup() {
  Serial.begin(9600);
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("begin error");
    return;
  }

  pinMode(AMP_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  enablePower();
  if (deleteFiles) {
    for (unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {
      SD.remove("MS_" + String(i) + ".txt");
    }
  }
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
      data = String(timeMs - startTime) + "," + String(amps) + "\r" + "\n";

      //Serial.println(data);
      if (times != lastTimes) {
        data = String(timeMs - startTime) + "," + String(amps);
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

    bool command;
    bool foundTimestamp;
    bool getTheAmpVal;
    long t;
    foundTimestamp = false;
    getTheAmpVal = false;
    command = OFF;
    prevTime = 0;

    float deltaT = fmod((millis() - startTime), TRAINING_PERIOD);
    bool reachedComma = false;
    for (unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {

      charTrackerTime = 0;
      charTrackerAmps = 0;

      if (foundTimestamp) {
        break;
      }

      File f;
      f = SD.open("MS_" + String(i) + ".txt", FILE_READ);

      char c;
      while (c = f.read()) {
        if (c < 0 && getTheAmpVal) {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';
          charTrackerAmps = 0;

          ampPt = atof(ampDataPt);

          foundTimestamp = true;

          break;
        }
        else if (c < 0) {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';
          charTrackerAmps = 0;

          break;
        }
        else if (c == '\n' && getTheAmpVal) {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';
          charTrackerAmps = 0;

          ampPt = atof(ampDataPt);

          foundTimestamp = true;
          break;
        }
        else if (c == '\n') {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';
          charTrackerAmps = 0;

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

          t = atol(timeDataPt);
          if (deltaT >= prevTime && deltaT < t) {
            prevTime = t;
            getTheAmpVal = true;
          }
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
      Serial.print(F("Filenum: "));
      Serial.println(i);
      f.close();
    }

    delay(35);

    Serial.print(F("time: "));
    Serial.print(deltaT);
    Serial.print(F(" amps: "));
    Serial.print(ampPt);
    Serial.print(F(" command: "));

    if (ampPt >= divider) {
      command = ON;
      Serial.println(F("ON"));
    }
    else {
      command = OFF;
      Serial.println(F("OFF"));
    }

    if (command) {
      enablePower();
    }
    else {
      disablePower();
    }
  }
  else if (state == LEARN) {

    unsigned int dataSize = 0;
    float mean = 0;
    float variance = 0;
    float maxVal = 0;
    float minVal = MAX_FLOAT_VAL;

    ////////////////////////////////////////////////
    //Calculate the Mean////////////////////////////
    ////////////////////////////////////////////////

    charTrackerTime = 0;
    charTrackerAmps = 0;

    bool reachedComma = false;
    char c;
    for (unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {
      File f;
      f = SD.open("MS_" + String(i) + ".txt");

      while (c = f.read()) {
        if (c < 0) {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';

          ampPt = atof(ampDataPt);

          if (ampPt > maxVal) {
            maxVal = ampPt;
          }
          if (ampPt < minVal) {
            minVal = ampPt;
          }

          mean += ampPt;
          dataSize++;

          break;
        }
        else if (c == '\n') {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';
          charTrackerAmps = 0;

          ampPt = atof(ampDataPt);

          if (ampPt > maxVal) {
            maxVal = ampPt;
          }
          if (ampPt < minVal) {
            minVal = ampPt;
          }

          mean += ampPt;
          dataSize++;

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

    if (dataSize != 0) {
      mean /= dataSize;
    }

    ////////////////////////////////////////////////
    //Calculate the Variance////////////////////////
    ////////////////////////////////////////////////

    charTrackerTime = 0;
    charTrackerAmps = 0;
    reachedComma = false;
    for (unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {
      File f;
      f = SD.open("MS_" + String(i) + ".txt");

      while (c = f.read()) {
        if (c < 0) {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';

          ampPt = atof(ampDataPt);

          variance += (ampPt - mean) * (ampPt - mean);

          break;
        }
        else if (c == '\n') {
          reachedComma = false;
          ampDataPt[charTrackerAmps] = '\0';
          charTrackerAmps = 0;

          ampPt = atof(ampDataPt);

          variance += (ampPt - mean) * (ampPt - mean);

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

    if (dataSize != 0) {
      variance /= dataSize;
    }

    ////////////////////////////////////////////////
    //DO THE THING//////////////////////////////////
    ////////////////////////////////////////////////

    float avgDivider = 0;
    unsigned int iterations = 0;
    randomSeed(analogRead(A0));
    float randVal1 = random(minVal, maxVal);
    float randVal2 = random(minVal, maxVal);

    while (randVal1 == randVal2) {
      randVal2 = random(minVal, maxVal);
    }

    Serial.println(F("Starting Gaussian Mixture Modeling..."));

    Gaussian blob1Init = Gaussian(randVal1, sqrt(variance));
    Gaussian blob2Init = Gaussian(randVal2, sqrt(variance));

    BimodalModel model = BimodalModel(blob1Init, blob2Init);

    while (model.getMaxDelta() > DELTA_THRESH) {
      charTrackerTime = 0;
      charTrackerAmps = 0;
      reachedComma = false;
      for (unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {
        File f;
        f = SD.open("MS_" + String(i) + ".txt");

        while (c = f.read()) {
          if (c < 0) {
            reachedComma = false;
            ampDataPt[charTrackerAmps] = '\0';

            ampPt = atof(ampDataPt);
            model.updateModel(ampPt);
            
            break;
          }
          else if (c == '\n') {
            reachedComma = false;
            ampDataPt[charTrackerAmps] = '\0';
            charTrackerAmps = 0;

            ampPt = atof(ampDataPt);
            model.updateModel(ampPt);
           
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
      
      model.finishUpdate(dataSize);
    }

    divider = (model.getBlob1().getMean() + model.getBlob2().getMean())/2.0;

    Serial.println(F("Learning Complete"));

    Serial.print(F("Divider: "));
    Serial.println(divider);

    startTime = millis();
    prevTime = 0;

    state = 255;
  }
}

void enablePower() {
  digitalWrite(RELAY_PIN, HIGH);
}

void disablePower() {
  digitalWrite(RELAY_PIN, LOW);
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
    file = SD.open(filePath, FILE_WRITE);
    resetFileWrite = false;
  }
  file.print(data);
  if (closeFile) {
    file.close();
    resetFileWrite = true;
  }
}
