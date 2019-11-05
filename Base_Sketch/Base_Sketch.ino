#include <KDE.h>

float *dataPtr;
void setup() {
  // put your setup code here, to run once:
  float data[] = {0.0f,0.1f,0.1f,0.5f,0.5f,0.5f,0.7f,0.7f,1.0f,2.0f,2.1f,2.1f,2.5f,2.5f,2.5f,2.7f,2.7f,3.0f};
  dataPtr = data;
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0; i < sizeof(&dataPtr); i++) {
    printf((*dataPtr)[i]);
  }
}
