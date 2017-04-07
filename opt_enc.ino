int sensor1;
int sensor2;

int stepOld;
int stepNew;

long PulseCount;

#define SENSORA 2
#define SENSORB 3

void setup() {

  Serial.begin(9600);
  attachInterrupt(0, checkState, CHANGE);
  attachInterrupt(1, checkState, CHANGE);
  PulseCount = 0;
}

void checkState(){
  sensor1 = digitalRead(SENSORA);
  sensor2 = digitalRead(SENSORB);

  if(sensor1 == 1 && sensor2 == 1){
    stepNew = 0;
    if(stepOld == 1){
      PulseCount--;
    }
    if(stepOld == 3){
      PulseCount++;
    }
    stepOld = 0;
    //Serial.print("step 0");
  }

  if(sensor1 == 0 && sensor2 == 1){
    stepNew = 1;
    if(stepOld == 2){
      PulseCount--;
    }
    if(stepOld == 0){
      PulseCount++;
    }
    stepOld = 1;
    //Serial.print("step 1");
  }

  if(sensor1 == 0 && sensor2 == 0){
    stepNew = 2;
    if(stepOld == 3){
      PulseCount--;
    }
    if(stepOld == 1){
      PulseCount++;
    }
    stepOld = 2;
    //Serial.print("step 2");
  }

  if(sensor1 == 1 && sensor2 == 0){
    stepNew = 3;
    if(stepOld == 0){
      PulseCount--;
    }
    if(stepOld == 2){
      PulseCount++;
    }
    stepOld = 3;
    //Serial.print("step 3");
  }

}

void loop() {
  printRead();
  
}

void printRead() {

  if(sensor1 == 1){
    Serial.print("HIGH1");
    Serial.print("\n");
  }
  else{
    Serial.print("LOW1");
    Serial.print("\n");
  }

  if(sensor2 == 1){
    Serial.print("HIGH2");
    Serial.print("\n");
  }
  else{
    Serial.print("LOW2");
    Serial.print("\n");
  }
}



