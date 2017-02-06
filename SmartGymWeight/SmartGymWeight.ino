#include <smartGymMPU6050.h>
#include <SoftwareSerial.h>

// Defines
#define REGULAR_MOVEMENT_TIME_DEADZONE 200  // Minimum movement time
#define SIDEWAYS_MOVEMENT_TIME_DEADZONE 250  // Minimum movement time 
#define HEMMERS_MOVEMENT_TIME_DEADZONE 60  // Minimum movement time 
#define WEIGHT_OFFSET 40  // weight messurements

// Enums
enum LiftingMode {NONE, REGULAR , HAMMERS , SIDEWAYS};

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
int initAngel = 0;
bool isInit = true;
int runner =0;
int weight = 0;

unsigned long lastPingSendTime = 0;

bool getWeight(){
	int temp = 0;
	double w1 = analogRead(A6);
	double w2 = analogRead(A7);
	if(w1>w2+WEIGHT_OFFSET || w1<w2-WEIGHT_OFFSET){ // not balanced
		if(weight != -1){
			BTserial.println("Error=The weight is not balanced, please correct it");
		}
		weight = -1;
		return false;
	} else if(w1>= 512-WEIGHT_OFFSET && w1<= 512+WEIGHT_OFFSET){  //base = 4 KG
		temp = 4;
		//BTserial.println("Weight=4");
	} else if(w1>= 341-WEIGHT_OFFSET && w1<= 341+WEIGHT_OFFSET){  //base + big = 8 KG
		temp = 8;
		//BTserial.println("Weight=8");
	} else if(w1>= 409-WEIGHT_OFFSET && w1<= 409+WEIGHT_OFFSET){  //base + small = 6 KG
		temp = 6;
		//BTserial.println("Weight=6");
	} else if(w1>= 292-WEIGHT_OFFSET && w1<= 292+WEIGHT_OFFSET){  //base + big + small = 10 KG
		temp = 10;
		//BTserial.println("Weight=10");
	}

	if(temp != weight){
		weight = temp;
		BTserial.print("Weight=");BTserial.println(weight);
	}
	return true;
}

void preLift(){
	if(movementStart!=0 && movementFinish!=0){
		if(((movementFinish-movementStart >= REGULAR_MOVEMENT_TIME_DEADZONE) && (mode == REGULAR)) ||
				((movementFinish-movementStart >= SIDEWAYS_MOVEMENT_TIME_DEADZONE) && (mode == SIDEWAYS)) || 
				(( movementFinish-movementStart >= HEMMERS_MOVEMENT_TIME_DEADZONE) && (mode == HAMMERS))){
			totalAccel = 0;
			count++;
			BTserial.print("SetCounter="); BTserial.println(count);
		}
	}

	if(movementStart!=0 && movementFinish==0){
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
	if(movementStart!=0 && movementFinish==0){
		movementFinish = millis();
		if(movementFinish-movementStart<250){
			BTserial.println("Print=Slow down for a better workout :)");
		}
	}
}

void reset(){
	mode = NONE;
	count = 0;
	movementStart = 0;
	movementFinish = 0;
	weight = 0;
	runner =0;
	isInit=true;
	initAngel=0;
}

void trackSet(){
	float ypr = 0;

	if(mode == NONE){
		BTserial.println("Print=No mode was set");
		return;
	}

	while(true){
		if(BTserial.available()){
			buffer = BTserial.readString();
			if(buffer == "StopSet"){
				BTserial.println("OkStopSet");
				reset();
				return;
			}
			else if(buffer == "Reset"){
        BTserial.println("OkReset");
				mpu.mpu6050Restart();
				reset();
				BTserial.println("ResetComplete");
				return;
			}
			else if(buffer == "StartSet"){
				BTserial.println("OkStartSet");
				reset();
			}
			else if(buffer == "SetMode=Hammers"){
				BTserial.println("OkSetMode");
				mode = HAMMERS;
			}
			else if(buffer == "SetMode=Regular"){
				BTserial.println("OkSetMode");
				mode = REGULAR;
			}
			else if(buffer == "SetMode=Sideways"){
				BTserial.println("OkSetMode");
				mode = SIDEWAYS;
			}
			else{
        BTserial.println("UnknownCommand");
      }
		}

		// Not suppouse to happened
		if(mode==NONE){
			BTserial.println("Error=No mode is set");
			return;
		}
		
		if(!getWeight()){
			continue;
		}
		
		mpu.mpu6050GetMeasurements(ypr_vec,aaReal);
		if(mode == REGULAR || mode == SIDEWAYS){
			ypr =  ypr_vec[2]*180/M_PI;
		}else{
			ypr =  ypr_vec[1]*180/M_PI;
		}

		if(runner < 3){
			runner++;
			continue;
		}

		if(isInit){
      BTserial.println("Print=OK, GO!");
			initAngel=ypr;
			isInit=false;
			continue;
		}

		if(mode == REGULAR){
			if(initAngel  >= 10 &&  initAngel  < 50){
				if( ypr  >= 20 &&  ypr  < 40){
					preLift();
				}else if(ypr >= -30 &&   ypr  < 0){
					midLift();
				}else if(ypr >= -80 &&  ypr < -60){
					postLift();
				}
			} else if(initAngel  >= 70 &&  initAngel  < 110){
				if( ypr  >= 80 &&  ypr  < 100){
					preLift();
				}else if(ypr >= 20 &&   ypr  < 50){
					midLift();
				}else if(ypr >= -20 &&  ypr < 10){
					postLift();
				}
			} else if(initAngel  >= 130 &&  initAngel  < 170){
				if( ypr  >= 140 &&  ypr  < 160){
					preLift();
				}else if(ypr >= 60 &&   ypr  < 90){
					midLift();
				}else if(ypr >= 20 &&  ypr < 50){
					postLift();
				}
			}else if(initAngel  >= -170 &&  initAngel  < -130){
				if( ypr  >= -160 &&  ypr  < -140){
					preLift();
				}else if(ypr >= 130 &&   ypr  < 160){
					midLift();
				}else if(ypr >= 90 &&  ypr < 120){
					postLift();
				}
			}else if(initAngel  >= -110 &&  initAngel  < -70){
				if( ypr  >= -100 &&  ypr  < -80){
					preLift();
				}else if(ypr >= -160 &&   ypr  < -110){
					midLift();
				}else if(ypr >= 140 &&  ypr < 170){
					postLift();
				}
			}else if(initAngel  >= -50 &&  initAngel  < -10){
				if( ypr  >= -40 &&  ypr  < -20){
					preLift();
				}else if(ypr >= -80 &&   ypr  < -50){
					midLift();
				}else if(ypr >= -140 &&  ypr < -120){
					postLift();
				}
			}
		} else if(mode == HAMMERS){
			if(initAngel  >= -10 &&  initAngel  < 10){
				if( ypr  >= -10 &&  ypr  < 10){
					preLift();
				}else if((ypr >= -90 &&   ypr  < -70)||(ypr >= 70 &&   ypr  < 90)){
					midLift();
				}else if((ypr >= -120 &&  ypr < -105)||(ypr >= 105 &&  ypr < 120)){
					postLift();
				}
			}else if((initAngel  >= -180 &&  initAngel  < -170) || (initAngel  >= 170 &&  initAngel  < 180)){
				if((ypr  >= -180 &&  ypr  < -170) || (ypr  >= 170 &&  ypr  < 180)){
					preLift();
				}else if((ypr >= -100 &&   ypr  < -90)||(ypr >= 90 &&   ypr < 100)){
					midLift();
				}else if((ypr >= -70 &&  ypr < -55)||(ypr >= 55 &&  ypr < 70)){
					postLift();
				}
			}
		} else {  // SIDEWAYS
			if(initAngel  >= 20 &&  initAngel  < 40){
				if( ypr  >= 20 &&  ypr  < 40){
					preLift();
				}else if(ypr >= -20 &&   ypr  < -10){
					midLift();
				}else if(ypr >= -60 &&  ypr < -40){
					postLift();
				}
			} else if(initAngel  >= 80 &&  initAngel  < 100){
				if( ypr  >= 80 &&  ypr  < 100){
					preLift();
				}else if(ypr >= 50 &&   ypr  < 60){
					midLift();
				}else if(ypr >= 0 &&  ypr < 20){
					postLift();
				}
			} else if(initAngel  >= 140 &&  initAngel  < 160){
				if( ypr  >= 140 &&  ypr  < 160){
					preLift();
				}else if(ypr >= 90 &&   ypr  < 100){
					midLift();
				}else if(ypr >= 50 &&  ypr < 70){
					postLift();
				}
			}else if(initAngel  >= -160 &&  initAngel  < -140){
				if( ypr  >= -160 &&  ypr  < -140){
					preLift();
				}else if(ypr >= 160 &&   ypr  < 170){
					midLift();
				}else if(ypr >= 120 &&  ypr < 140){
					postLift();
				}
			}else if(initAngel  >= -100 &&  initAngel  < -80){
				if( ypr  >= -100 &&  ypr  < -80){
					preLift();
				}else if(ypr >= -120 &&   ypr  < -110){
					midLift();
				}else if(ypr >= -170 &&  ypr < -150){
					postLift();
				}
			}else if(initAngel  >= -40 &&  initAngel  < -20){
				if( ypr  >= -40 &&  ypr  < -20){
					preLift();
				}else if(ypr >= -70 &&   ypr  < -60){
					midLift();
				}else if(ypr >= -120 &&  ypr < -100){
					postLift();
				}
			}
		}
//   BTserial.println(ypr);
	}
}


void setup() {
	BTserial.begin(9600);
	//mpu.mpu6050Restart();
}

void loop() {
	if(BTserial.available()){
		buffer = BTserial.readString();
		if(buffer == "SetMode=Hammers"){
			BTserial.println("OkSetMode");
			mode = HAMMERS;
		}else if(buffer == "SetMode=Regular"){
			BTserial.println("OkSetMode");
			mode = REGULAR;
		}else if(buffer == "SetMode=Sideways"){
			BTserial.println("OkSetMode");
			mode = SIDEWAYS;
		}else if(buffer == "StartSet"){
			BTserial.println("OkStartSet");
			if(getWeight()){
				trackSet();
			}
		}else if(buffer == "Reset"){
      BTserial.println("OkReset");
			mpu.mpu6050Restart();
			reset();
			BTserial.println("ResetComplete");
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

