#include <SD.h>
#include <Wire.h>
#include <GaussianMixtureModel.h>
#include <BimodalModelLib.h>

#define DS3231_I2C_ADDRESS 0x68

#define DATACOLLECT 0
#define LEARN 1
#define OPERATE 2
#define RELEARN 3

#define MAX_FLOAT_VAL 3.4028235e38
#define MIN_FLOAT_VAL -MAX_FLOAT_VAL

#define ON true
#define OFF false

#define AMP_PIN A3
#define ENABLE_AMP_PIN 5
#define RELAY_PIN 7
#define CHIP_SELECT 4
#define POWER_CUT_PIN 2
#define WRONG_BUTTON_PIN 6

#define DELTA_THRESH 0.12

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
boolean resetFileWrite = true;
boolean allowWrite = true;

long startTime;

float divider;

unsigned int dataSize;
unsigned int minsSize;

String data;
unsigned int lastTimes;

float ampPt;
long timePt;
long prevTime;

boolean skipFile;

unsigned int charTrackerTime;
unsigned int charTrackerAmps;

File file;



bool deleteFiles = false;

void setup() {

  skipFile = false;
  Wire.begin();

  Serial.begin(9600);
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("begin error");
    return;
  }

  pinMode(AMP_PIN, INPUT);
  pinMode(ENABLE_AMP_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  if (deleteFiles) {
    for (unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {
      SD.remove("MS_" + String(i) + ".txt");
    }
  }
  data = "";
  startTime = getTimeIntoDay();
  lastTimes = floor(fmod(fmod(millis() + startTime, TRAINING_PERIOD) / round(fPEM), MAX_NUMBER_OF_FILES));
  if(!SD.exists("MS_" + String(lastTimes) + ".txt")){
    skipFile = true;
  }
  if(checkAllFilesExist()){
    state = LEARN;
    enablePower();
  }
  else{
    state = OPERATE;
  }
}
void loop() {
  if (state == DATACOLLECT) {
    enablePower();
    enableAmmeter();

    long timeMs = millis() + startTime;
    float amps = getAmps();


    if (powerCut()) {
      allowWrite = false;
    }

    if (allowWrite) {
      unsigned int times = floor(fmod(fmod(timeMs, TRAINING_PERIOD) / round(fPEM), MAX_NUMBER_OF_FILES));
      /*if(fmod(fmod(millis() + startTime, TRAINING_PERIOD), fPEM) > 1000 && times != lastTimes){
        skipFile = true;
      }
      */
      if(skipFile){
        lastTimes = floor(fmod(fmod(millis() + startTime, TRAINING_PERIOD) / round(fPEM), MAX_NUMBER_OF_FILES));
      }
      while (skipFile) {
        Serial.println(String(lastTimes) + " " + String(floor(fmod(fmod(millis() + startTime, TRAINING_PERIOD) / round(fPEM), MAX_NUMBER_OF_FILES))));
        if (floor(fmod(fmod(millis() + startTime, TRAINING_PERIOD) / round(fPEM), MAX_NUMBER_OF_FILES)) != lastTimes) {
          skipFile = false;
          timeMs = millis() + startTime;
          lastTimes = floor(fmod(fmod(timeMs, TRAINING_PERIOD) / round(fPEM), MAX_NUMBER_OF_FILES));
          times = floor(fmod(fmod(timeMs, TRAINING_PERIOD) / round(fPEM), MAX_NUMBER_OF_FILES));
        }
      }
      Serial.print(F("delta T: "));
      Serial.print(fmod(timeMs, TRAINING_PERIOD));
      Serial.print(F(", "));
      Serial.print(F("lim: "));
      Serial.println(TRAINING_PERIOD);



      data = String(fmod(timeMs, TRAINING_PERIOD)) + "," + String(amps) + "\r" + "\n";

      if (times != lastTimes) {
        data = String(fmod(timeMs, TRAINING_PERIOD)) + "," + String(amps);
        appendToFile("MS_" + String(times - 1) + ".txt", data, true);
        Serial.println(data);
        data = "";
        Serial.println(F("RESET"));
        lastTimes = times;
        Serial.print(F("file: "));
        Serial.println(times - 1);
        if (SD.exists("MS_" + String(times) + ".txt")) {
          disableAmmeter();
          if(checkAllFilesExist()){
            state = LEARN;
          }
          else{
            state = OPERATE;
          }
        }

      }
      else {
        appendToFile("MS_" + String(times) + ".txt", data, false);
      }

    }
    else {
      file.close();
      SD.remove(file.name());
      while (true) {};
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

    float deltaT = fmod(fmod(millis() + startTime, TRAINING_PERIOD), TRAINING_PERIOD);
    bool reachedComma = false;
    for (unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {

      if (checkWrongButton()) {
        skipFile = true;
        if (i == 0) {
          SD.remove("MS_" + String(0) + ".txt");
          SD.remove("MS_" + String(MAX_NUMBER_OF_FILES - 1) + ".txt");
          SD.remove("MS_" + String(1) + ".txt");
        }
        else if (i == MAX_NUMBER_OF_FILES - 1) {
          SD.remove("MS_" + String(0) + ".txt");
          SD.remove("MS_" + String(MAX_NUMBER_OF_FILES - 1) + ".txt");
          SD.remove("MS_" + String(MAX_NUMBER_OF_FILES - 2) + ".txt");
        }
        SD.remove("MS_" + String(i - 1) + ".txt");
        SD.remove("MS_" + String(i) + ".txt");
        SD.remove("MS_" + String(i + 1) + ".txt");
      }

      charTrackerTime = 0;
      charTrackerAmps = 0;

      if (foundTimestamp) {
        break;
      }

      File f;
      if (SD.exists("MS_" + String(i) + ".txt")) {
        f = SD.open("MS_" + String(i) + ".txt", FILE_READ);
      }
      else {
        state = DATACOLLECT;
        resetFileWrite = true;
        return;
      }

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

    float randVal1 = random(minVal * 100, maxVal * 100);
    randVal1 = randVal1 / 100.0;
    float randVal2 = random(minVal * 100, maxVal * 100);
    randVal2 = randVal2 / 100.0;

    while (randVal1 == randVal2) {
      randVal2 = random(minVal * 100, maxVal * 100);
      randVal2 = randVal2 / 100.0;
      Serial.println("minVal: " + String(minVal));
      Serial.println("minVal: " + String(maxVal));


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

    divider = (model.getBlob1().getMean() + model.getBlob2().getMean()) / 2.0;

    Serial.println(F("Learning Complete"));

    Serial.print(F("Divider: "));
    Serial.println(divider);

    prevTime = 0;

    state = OPERATE;
  }
}

void enablePower() {
  digitalWrite(RELAY_PIN, HIGH);
}

void disablePower() {
  digitalWrite(RELAY_PIN, LOW);
}

void enableAmmeter() {
  digitalWrite(ENABLE_AMP_PIN, HIGH);
}

void disableAmmeter() {
  digitalWrite(ENABLE_AMP_PIN, LOW);
}

float getAmps() {
  float amps;
  const float varVolt = .03783;
  const float varProccess = 5e-8;
  float Pc = 0;
  float G = 0;
  float P = 1;
  float Xp = 0;
  float Zp = 0;
  float Xe = 0;
  for (int i = 0; i < 70; i++) {
    float vOut = (analogRead(AMP_PIN)- 480);
    delay(1);
    amps = abs(vOut);

    
    
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

//Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val) {
  return ( (val / 10 * 16) + (val % 10) );
}

//Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val) {
  return ( (val / 16 * 10) + (val % 16) );
}

void readTimeOfDay(
  byte *second,
  byte *minute,
  byte *hour) {
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
}

unsigned long getTimeIntoDay() {
  byte second, minute, hour;
  readTimeOfDay(&second, &minute, &hour);
  unsigned long timeMillis = ((unsigned int) second) * 1000;
  timeMillis += ((unsigned long) minute) * 60000;
  timeMillis += ((unsigned long) hour) * 3600000;

  return timeMillis;

}

byte getDay() {
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  Wire.read();
  Wire.read();
  Wire.read();
  return bcdToDec(Wire.read());
}

boolean checkWrongButton() {
  return (digitalRead(WRONG_BUTTON_PIN) == HIGH);
}

boolean powerCut() {
  return digitalRead(POWER_CUT_PIN) == LOW;
}
boolean checkAllFilesExist(){
  for(int i = 0; i < MAX_NUMBER_OF_FILES; i++){
    if(!SD.exists("MS_" + String(i) + ".txt")){
      return false;
    }
  }
  return true;
}
