#include <smartGymMPU6050.h>
#include <SoftwareSerial.h>

#define TIME_TESHHOLD_PUSHUPS 250
#define TIME_TESHHOLD_SITUPS 400


// Enums
enum LiftingMode {NONE, PUSHUPS , SITUPS};

// Global variables
SoftwareSerial BTserial(3, 4); // RX | TX
String buffer;
MPU mpu(&BTserial);       //  A class that represents the MPU and all its methods

LiftingMode mode = NONE;

int count = 0;
long int totalAccel;
unsigned long movementStart, movementFinish;  // needed to measure the movment time
VectorInt16 aaReal;
float ypr_vec[3];

float initAngel;
bool isInit=true;

unsigned long lastPingSendTime = 0;

void reset(){
	mode = NONE;
	count = 0;
	movementStart = 0;
	movementFinish = 0;
	isInit = true;
}

void preLift(){
  if( movementStart!=0 && movementFinish!=0 ){
    count++;
    BTserial.print("SetCounter="); BTserial.println(count);
  }else if( movementStart!=0 && movementFinish==0){
    BTserial.println("Print=Only full moves will get you there :)");
  }
  
	totalAccel=0;
	movementStart=0;  
	movementFinish=0;
}

void midLift(){
	if(movementStart == 0){
		movementStart = millis();
	}
}

void postLift(){
	if(movementFinish==0){
		movementFinish = millis(); 
    if((movementFinish - movementStart < TIME_TESHHOLD_SITUPS && mode == SITUPS) || 
       (movementFinish - movementStart < TIME_TESHHOLD_PUSHUPS && mode == PUSHUPS)){
       BTserial.println("Print=Slow down for a better workout :)");
    }
	}
}

void trackSet(){
	float ypr = 0;
  int runner = 0;

	if(mode == NONE){
		return;
	}

   while(true){
  	if(BTserial.available()){
  		buffer = BTserial.readString();
      Serial.println(buffer);
  		if(buffer == "StopSet"){
  			BTserial.println("OkStopSet");
  			reset();
  			return;
  		}
  		else if(buffer == "Reset"){
  			BTserial.println("OkReset");
        runner = 0;
  			reset();
        BTserial.println("ResetComplete");
  		}else if(buffer == "SetMode=Push-ups"){
        BTserial.println("OkSetMode");
        mode = PUSHUPS;
      }else if(buffer == "SetMode=Sit-ups"){
        BTserial.println("OkSetMode");
        mode = SITUPS;
      }else if(buffer == "StartSet"){
        BTserial.println("OkStartSet");
        runner = 0;
        count = 0;
        movementStart = 0;
        movementFinish = 0;
        isInit = true;
      }else{
        BTserial.println("UnknownCommand");
      }
  	}

    mpu.mpu6050GetMeasurements(ypr_vec,aaReal);
    ypr =  ypr_vec[2]* 180/M_PI;

    if(runner < 3){
      runner++;
      continue;
    }

    if(isInit){
      BTserial.println("Print=OK, GO!");
      initAngel=ypr;
      Serial.println(initAngel);
      isInit=false;
      continue;
    }

    //BTserial.println("Im here");
 
		if(mode == PUSHUPS){
			if( ypr  >= initAngel-10 &&  ypr  < initAngel+10){ // -80<X<-60
        //Serial.println("pre");
        preLift();
      }else if(ypr >= initAngel+20 &&   ypr  < initAngel+30){ // -50<X<-40
        midLift();
      }else if(ypr >= initAngel+70 &&  ypr < initAngel+90){ // -20<X<20
        postLift();
      }
		} else {  // SITUPS
			if( ypr  >= initAngel-10 &&  ypr  < initAngel+10){
				preLift();
			}else if(ypr >= initAngel-40 &&   ypr  < initAngel-20){
				midLift();
			}else if(ypr >= initAngel-100 &&  ypr < initAngel-75){
				postLift();
			}
		}
		
		//BTserial.println(ypr);
	}
}

void setup() {
  Serial.begin(115200);
	BTserial.begin(9600);
	//pu.mpu6050Restart();
}

void loop() {
	if(BTserial.available()){
		buffer = BTserial.readString();
    Serial.println(buffer);
		if(buffer == "Reset"){
      BTserial.println("OkReset");
      mpu.mpu6050Restart();
			reset();
      BTserial.println("ResetComplete");
  	}else if(buffer == "SetMode=Push-ups"){
      BTserial.println("OkSetMode");
      mode = PUSHUPS;
    }else if(buffer == "SetMode=Sit-ups"){
      BTserial.println("OkSetMode");
      mode = SITUPS;
		}else if(buffer == "StartSet"){
			BTserial.println("OkStartSet");
			trackSet();
		}else{
      BTserial.println("UnknownCommand");
		}
	}else{
    if (millis() - lastPingSendTime > 200){
      BTserial.println("Ping");
      lastPingSendTime = millis();
    }
  }
}

