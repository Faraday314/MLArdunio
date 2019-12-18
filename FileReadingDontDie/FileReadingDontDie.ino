#include <SD.h>
#define MAX_NUMBER_OF_FILES 3
#define DATACOLLECT 0
#define LEARN 1
#define OPERATE 2
#define RELEARN 3
#define fPEM .1*60*1000
#define H 0.5
#define MAX_FLOAT_VAL 3.4028235e38;
#define MIN_FLOAT_VAL -MAX_FLOAT_VAL;

boolean allowWrite = true;
unsigned int state = LEARN;
int lastFile = 0;
float lastTime = 0;
File file;
File myFile;
File myFile2;

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
  else if(state == LEARN) {
    state = OPERATE;
    unsigned int dataSize = getNumDataPoints();
    long timeData[dataSize];
    getAllData(&timeData[0], dataSize);
    for(unsigned int j = 0; j < sizeof(timeData)/sizeof(timeData[0]); j++) {
      Serial.print("time: ");
      Serial.println(String(timeData[j]));
    }
  }
}

void getAllData(long * timeOutput, unsigned int listSize) {
  long timeArr[listSize];
  unsigned int tracker = 0;
  for (unsigned int i = 0; i < 3; i++) {
    
    file = SD.open("MS_"+String(i)+".txt", FILE_READ);
    if (!file) {
      Serial.println("open error");
      return;
    }
    long x;
    float y;
    while (readVals(&x, &y)) {
      Serial.print("x: ");
      Serial.println(x);
      timeArr[tracker] = x;
      tracker++;
      Serial.print("y: ");
      Serial.println(y);
      Serial.println();
    }
    Serial.println("Done");
  }
  copy(timeArr,timeOutput,listSize);
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
void copy(long* src, long* dst, unsigned int len) {
  memcpy(dst, src, sizeof(src[0])*len);
}
