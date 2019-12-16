#include <KDE.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>

//File Vars
File myFile;
File myFile2;
File myFile3;
const float fPEM = .1 * 60 * 1000;
int lastFile = 0;
float lastTime = 0;
boolean allowWrite = true;
unsigned const int MAX_NUMBER_OF_FILES = 3;

//PURE EVIL
unsigned int sizeOfData = 0;
long timeDataPtGlobal;

//ML Vars
float *dataPtr;
Kernel *kernelPtr;
unsigned int dataSize;
const float H = 0.5;
const float MAX_FLOAT_VAL = 3.4028235e38;
const float MIN_FLOAT_VAL = -MAX_FLOAT_VAL;
unsigned const int DATACOLLECT = 0;
unsigned const int LEARN = 1;
unsigned const int OPERATE = 2;
unsigned const int RELEARN = 3;
unsigned int state = DATACOLLECT;
float output[32];

void setup() {

  //SD Card
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
 
  //KDE stuff
  float data[] = {0.0f,0.1f,0.1f,0.5f,0.5f,0.5f,0.7f,0.7f,1.0f,2.0f,2.1f,2.1f,2.5f,2.5f,2.5f,2.7f,2.7f,3.0f};
  dataSize = sizeof(data)/sizeof(data[0]);
  dataPtr = &data[0];

  Kernel kernels[dataSize];
 
  Serial.begin(9600);

  float *dataPtrCpy = dataPtr;

  //Generate Kernels
  for(unsigned int i = 0; i < dataSize; i++) {   
    kernels[i] = *(new Kernel(*dataPtrCpy, H));
    dataPtrCpy++;
  }
  Kernel *kernelPtr = &kernels[0];  
  unsigned int outputSize = findMin(kernelPtr, dataSize, 0.0, 3.0, 0.1, 0.01);
  for(int i = 0; i < outputSize; i++) {
    //Serial.println(output[i]);
  }
  state = LEARN;
}

void loop() {

  //Datacollect state is WIP
  if(state == DATACOLLECT) {
    unsigned int times = fmod(floor(round(millis()) / fPEM), MAX_NUMBER_OF_FILES);
    Serial.println(times);
    if(allowWrite){
      //If anything is typed in the serial monitor stop writing
      if(Serial.read() != -1){
       allowWrite = false;
      }
      if(lastFile != times) {
        myFile.close();
 
        lastFile = times;

        if(SD.exists("ms_" + (String) times + ".txt")) {
          SD.remove("ms_" + (String) times + ".txt");
        }
        
        myFile = SD.open("ms_" + (String) times + ".txt", FILE_WRITE);
      }
      else{
          if (myFile) {
            Serial.println("Wrote to file");
            myFile.println("time = " + (String) round(millis()));
         } else {
            Serial.println((String) "error opening " + (String) "mills: " + (String) times + (String) ".txt");
         }
       }
    }
    else{
      myFile.close();
    }
  }
  else if(state == LEARN) {
    dataSize = getNumDataPoints();
    unsigned long timeData[dataSize];
    long timeData2[dataSize];
    float amperageData2[dataSize];
    getAllData(&timeData2[0], &amperageData2[0], dataSize);
    state = OPERATE;
  }
}

void getAllData(long *timeDataPtr, float *amperageDataPtr, unsigned int listSize) {

  long timeArr[listSize];
  float ampArr[listSize];
  unsigned int tracker = 0;

  for(unsigned int i = 0; i < MAX_NUMBER_OF_FILES; i++) {
    
    myFile2 = SD.open("MS_" + (String) i + ".txt" +'\0', FILE_READ);
    Serial.println("ran " + (String) i);
    Serial.println((String)i + '\0');

    if(myFile2) {

      unsigned int idx = 0;
      unsigned int newLineCount = 0;
      char charArray[myFile2.available()/sizeof('\n')];
      while(myFile2.available()) {
        char inChar = myFile.read();
        charArray[idx] = inChar;
        idx++;
        if(inChar == '\n') {
          newLineCount++;
        }
      }
      myFile2.close();

      continue;

      unsigned int count = 0;
      unsigned int newLineIdxes[newLineCount + 1];
      for(unsigned int i = 0; i < sizeof(charArray)/sizeof(charArray[0]); i++) {
        if(charArray[i] == '\n') {
          newLineIdxes[count] = i;
        }
      }      newLineIdxes[count] = idx;
      count++;
      
      unsigned int startingIdx = 0;
      //For each newline
      for(unsigned int i = 0; i < count; i++) {
        unsigned int newLineIdx = newLineIdxes[i];
        char val[newLineIdx - startingIdx + 1];

        unsigned int commaIdx = 0;
        for(unsigned int j = startingIdx; j < newLineIdx; j++) {
          if(charArray[j] == ',') {
            commaIdx = j;
          }
        }

        char timeData[commaIdx-startingIdx + 1];
        char ampData[newLineIdx - (commaIdx + 1)];
        //For each char between the start index and the next newline
        for(unsigned int j = startingIdx; j < newLineIdx; j++) {
          val[j - startingIdx] = charArray[j];
        }

        for(unsigned int k = startingIdx; k < commaIdx; k++) {
          timeData[k - startingIdx] = charArray[k];
        }
       
        for(unsigned int m = commaIdx + 1; m < newLineIdx - 1; m++) {
          ampData[m - commaIdx - 1] = charArray[m];
        }

        timeData[commaIdx - startingIdx] = '\0';


        
        //char timeDataCpy[commaIdx-startingIdx + 1];
        //copy(timeData, timeDataCpy, commaIdx-startingIdx + 1);


        char *endPtr;
        timeDataPtGlobal = strtol(timeData, &endPtr, 10);

        if(!*endPtr) {
          Serial.println("GOOD :)");
        }
        else {
          Serial.println("BAD");
        }

        //Serial.println(String(timeDataPtGlobal));
        //timeArr[tracker] = timeDataPtGlobal;
        //*timeArrPtr = timePt;
        //memcpy(&timeArr[tracker],&timePt,sizeof(timePt));
        tracker++;
        
        if(i == count - 1) {
          char ampDataFinal[newLineIdx - commaIdx];
          for(unsigned int i = 0; i < sizeof(ampData)/sizeof(ampData[0]); i++) {
            ampDataFinal[i] = ampData[i];
          }

          ampDataFinal[newLineIdx - commaIdx - 2] = charArray[newLineIdx - 1];
          ampDataFinal[newLineIdx - commaIdx - 1] = '\0';
        }
        
        ampData[newLineIdx - 1 - (commaIdx + 1)] = '\0';

        val[newLineIdx - startingIdx] = '\0';
        //Serial.println(ampData);
        startingIdx = newLineIdx + 1;
      }
    }
    else {
      Serial.println("error opening MS_" + (String) i + ".txt");
    }
  }

  /*for(unsigned int i = 0; i < sizeof(timeArr)/sizeof(timeArr[0]); i++) {
    Serial.println("time "+(String)timeArr[i]);
  }*/
}

unsigned int getNumDataPoints() {
  unsigned int numDataPoints = 0;
  
  for(int i = 0; i < MAX_NUMBER_OF_FILES; i++) {
    myFile3 = SD.open("MS_" + (String) i + ".txt", FILE_READ);
    if(myFile3) {
      while(myFile3.available()) {
        if(myFile3.read() == '\n') {
          numDataPoints++;
        }
      }
      myFile3.close();
    }
    else {
      Serial.println("error opening MS_" + (String) i + ".txt");
    }
  }
  return numDataPoints;
}

unsigned int findMin(Kernel *kernelPtr, unsigned int dataSize, float lowerBound, float upperBound, float algStep, float lossThreshold) {
  Kernel *kernelPtrCpy = kernelPtr;
  float currentX = lowerBound;
  float lastValue = Kernel::kernelConsensus(kernelPtr, dataSize, currentX);
  currentX += algStep/2;
  float currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, currentX);
  float delta = currentValue - lastValue;

  while(delta == 0.0) {
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
  while(currentXCpy < upperBound) {
    lastSgnCpy = sgnCpy;
    currentXCpy += algStep;
    lastValueCpy = currentValueCpy;

    currentValueCpy = Kernel::kernelConsensus(kernelPtr, dataSize, currentXCpy);
    delta = currentValueCpy - lastValueCpy;
    sgnCpy = signum(delta);

    if(sgnCpy - lastSgnCpy > 0) {
      arrSize++;
    }
  }
  
  float maxRanges[arrSize];
  unsigned int index = 0;

  //Find the upper bound of all the mins
  while(currentX < upperBound) {
    lastSgn = sgn;
    currentX += algStep;
    lastValue = currentValue;

    currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, currentX);
    delta = currentValue - lastValue;
    sgn = signum(delta);

    if(sgn - lastSgn > 0) {
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
  for(unsigned int i = 0; i < arrSize; i++) {
    lastValue = Kernel::kernelConsensus(kernelPtr, dataSize, maxRanges[i]-algStep);
    float mid = (2*maxRanges[i] - algStep)/2;
    currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, mid);
    float lBound = maxRanges[i] - algStep;
    float uBound = maxRanges[i];
    while(abs(currentValue-lastValue) > lossThreshold) {
      float rmid = (mid + uBound)/2;
      float lmid = (lBound + mid)/2;

      float rmidVal = Kernel::kernelConsensus(kernelPtr, dataSize, rmid);
      float lmidVal = Kernel::kernelConsensus(kernelPtr, dataSize, lmid);

      mid = rmidVal > lmidVal ? lmid : rmid;

      lastValue = currentValue;
      currentValue = Kernel::kernelConsensus(kernelPtr, dataSize, mid);
    }
    if(abs(currentValue - lastValue) == 0) {
      flooredDividers[usedFloorVals] = mid;
      usedFloorVals++;
      lastMidFloored = true;
    }
    else if(lastMidFloored) {
      dividers[usedVals] = (flooredDividers[usedFloorVals - 1] + mid)/2;
      usedVals++;
      lastMidFloored = false;
    }
    else {
      dividers[usedVals] = mid;
      usedVals++;
    }
  }

  if(usedVals < 32) {
    for(int i = 0; i < usedVals; i++) {
      output[i] = dividers[i];
    }
  }
  else {
    Serial.println("If you are reading this, everything has gone horribly wrong and you should give up. Good luck!");
  }

  return usedVals;
}

void copy(char* src, char* dst, unsigned int len) {
    memcpy(dst, src, sizeof(src[0])*len);
}

int signum(float val) {
    return (0.0 < val) - (val < 0.0);
}
