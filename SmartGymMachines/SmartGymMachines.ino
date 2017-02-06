#include <SoftwareSerial.h>

#define MAX_THRESHOLD 950
#define ADD_LIGHT 50
#define MIN_UP_TIME 500
#define MIN_DOWN_TIME 500
SoftwareSerial BTserial(3, 4); // RX | TX
String buffer;

//PhotoResistors
int pR0 = A0; // Highest
int pR1 = A1;
int pR2 = A2;
int pR3 = A3; // Lowest
int pRCount = A4;

int pR0Val = 0;
int pR1Val = 0;
int pR2Val = 0;
int pR3Val = 0;
int pRCountVal = 0;

bool resetSucceeded = false;
int val = 0;
int threshold = 950;
int state = 0, counter = 0;
unsigned long startedUp = 0, startedDown = 0;

unsigned long lastPingSendTime = 0;

void readLasers(){
  pR0Val = analogRead(pR0);
  pR1Val = analogRead(pR1);
  pR2Val = analogRead(pR2);
  pR3Val = analogRead(pR3);
  pRCountVal = analogRead(pRCount);
}

bool moving(){
  return pR0Val > threshold || pR1Val > threshold || pR2Val > threshold || pR3Val > threshold;
}

void reset(){
  resetSucceeded = false;
  counter = 0;
  state = 0;
  startedUp = millis();
  startedDown = millis();
  threshold = analogRead(pR0) + ADD_LIGHT; //Set threshold
  if(threshold > MAX_THRESHOLD){
    BTserial.println("Print=Too much light");
  }
  readLasers();
  if(moving() || pRCountVal < threshold){
    BTserial.println("BadReset");
  }
  else{
    resetSucceeded = true;
    BTserial.println("Ready");
  }
}

int howManyMove(){
  if(!moving()){
    return 0;
  }
  else if(pR3Val > threshold){
    return 4;
  }
  else if(pR2Val > threshold){
    return 3;
  }
  else if(pR1Val > threshold){
    return 2;
  }
  else if(pR0Val > threshold){
    return 1;
  }
}

void trackSet(){
  while(true){
    if(BTserial.available()){
      buffer = BTserial.readString();
      if(buffer == "StopSet"){
        BTserial.println("OkStopSet");
        //BTserial.print("FinalSetCount = ");
        //BTserial.println(counter);
        return;
      }
      else if(buffer == "Reset"){
        BTserial.println("OkReset");
        reset();
        BTserial.println("ResetComplete");
        return;
      }
      else if(buffer == "StartSet"){
        BTserial.println("OkStartSet");
        counter = 0;
      }
      else{
        BTserial.println("UnknownCommand");
      }
    }

    readLasers();
    switch(state){

     case 0: // Resting
        if(!moving()){// Still resting
          break;
        }
        else{ // Started moving
          
          BTserial.print("Lifted=");
          BTserial.println(howManyMove());
          startedUp = millis();
  
          state = 1; // Change to Going up
        }
        break;


     case 1: // Going up
        if(moving()){//Moving
          
          if(pRCountVal < threshold){ //Got to top
            BTserial.print("SomeValue=");
            BTserial.println(millis() - startedUp);
            if(millis() - startedUp < MIN_UP_TIME){
              BTserial.println("Print=For better workout slow down");
            }
            state = 2; //Going down
            break;
          }
          break;
        }
        else{//Resting
          BTserial.println("Print=Only full moves will get you there :)");
          state = 0;
          break;
        }
        break;

      case 2: // Going down
        if(moving()){//Moving
          if(pRCountVal < threshold){//while upper laser covered
            startedDown = millis();
          }
          break;
        }
       else{//Resting
         BTserial.print("SomeValue=");
         BTserial.println(millis() - startedDown);
         if(millis() - startedDown < MIN_DOWN_TIME){
            BTserial.println("Print=For better workout put down slowly");
         }
          state = 0;
          counter++;
          BTserial.print("SetCounter=");
          BTserial.println(counter);
        }
        break;
    } 
  }
}

void setup() {

  BTserial.begin(9600);
  reset();

}

void loop() {
  if(BTserial.available()){
    buffer = BTserial.readString();
    if(buffer == "StartSet"){
      BTserial.println("OkStartSet");
      if(resetSucceeded){
        counter = 0;
        trackSet();
      }
      else {
        BTserial.println("Error=StartSet failed because reset failed");
      }
    }
    else if(buffer == "Reset"){
      BTserial.println("OkReset");
      reset();
      BTserial.println("ResetComplete");
    }
    else{
      BTserial.println("UnknownCommand");
    }
  }else{
    if (millis() - lastPingSendTime > 200){
      BTserial.println("Ping");
      lastPingSendTime = millis();
    }
  }
}

