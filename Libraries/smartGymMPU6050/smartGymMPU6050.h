#ifndef _SMART_GYM_MPU6050
#define _SMART_GYM_MPU6050

// Includes
#include "Arduino.h"
#include "helper_3dmath.h"	// for - VectorInt16
#include "MPU6050.h"

#include "I2Cdev.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <SoftwareSerial.h>

// Defines
#define MPU_addr 0x68    // I2C address
#define DEFAULT_OFFSET 0
#define BUFFER_SIZE 1000  //Amount of readings used to average, make it higher to get more precision but sketch will be slower (default:1000)
#define ACEL_DEADZONE 8 //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
#define GYRO_DEADZONE 1 //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
#define DISCARDED_MEASURMENTS 100 // (default:100)
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards (INT)

// CLasses and Structs
class MPU {
	MPU6050 mpu;
	SoftwareSerial* bt;	// bluetooth communication
	
	// Measurments Values
	Quaternion q;           // [w, x, y, z]         quaternion container  
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements 
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements  
	VectorFloat gravity;    // [x, y, z]            gravity vector  
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector 
	
	// For offset caliberation 
	void meansensors();
	void calibration();
	void mpu6050GetMeasurementsAUX();
	
	bool dmpReady = false;  // set true if DMP init was successful  
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU  
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)  
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes) 
	uint16_t fifoCount;     // count of all bytes currently in FIFO 
	uint8_t fifoBuffer[64]; // FIFO storage buffer  
	
	void mpu6050Initialize();  // Initializtion process of the component
	void mpu6050FixOffset(); // Offset corrections
	
public:
	volatile bool mpuInterrupt;     /*	indicates whether MPU interrupt pin has gone high -  
										has to stay public in order for the class to work	*/
	// Methods
	MPU(SoftwareSerial* bt);
	void mpu6050Restart();
	void mpu6050GetMeasurements(float Ret_ypr[3],VectorInt16 &Ret_aaReal);
};

#endif