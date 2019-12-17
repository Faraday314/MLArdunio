#include <SD.h>
File file;

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

void setup() {
  long x;
  float y;
  Serial.begin(9600);
  if (!SD.begin(4)) {
    Serial.println("begin error");
    return;
  }

  long timeArr[18];
  unsigned int tracker = 0;
  for (unsigned int i = 0; i < 3; i++) {
    
    file = SD.open("MS_"+String(i)+".txt", FILE_READ);
    if (!file) {
      Serial.println("open error");
      return;
    }
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
  for(unsigned int j = 0; j < sizeof(timeArr)/sizeof(timeArr[0]); j++) {
      Serial.print("time: ");
      Serial.println(String(timeArr[j]));
  }
}
void loop() {}
