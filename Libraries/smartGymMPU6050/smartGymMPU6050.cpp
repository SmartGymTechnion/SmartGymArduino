// Includes
#include "smartGymMPU6050.h"

// Global variables
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz;  
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset; 

// Functions
MPU::MPU(SoftwareSerial* bt):mpu(MPU_addr),dmpReady(false), mpuInterrupt(false), bt(bt){}

// INTERRUPT DETECTION ROUTINE - from DPM6
MPU* tmpMPU;	// Bandage - I didn't find a better solution 
void dmpDataReady() {
	tmpMPU->mpuInterrupt = true;
}

void MPU::mpu6050Initialize(){
	tmpMPU = this;
	
	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
	#endif

	
	// initialize device
	bt->println("Initializing I2C devices...");
	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);
	
	// verify connection
	bt->println("Testing device connections...");
	bt->println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	/* load and configure the DMP */
	bt->println("Initializing DMP...");
	devStatus = mpu.dmpInitialize();

	/*  make sure it worked (returns 0 if so) */
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		bt->println("Enabling DMP...");
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		bt->println("Enabling interrupt detection (Arduino external interrupt 0)...");
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		bt->println("DMP ready! Waiting for first interrupt...");
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		bt->print("DMP Initialization failed (code ");
		bt->print(devStatus);
		bt->println(")");
	}
	
	/*  Sets MPU6050 default offsets to 0  */
	mpu.setXAccelOffset(DEFAULT_OFFSET);
	mpu.setYAccelOffset(DEFAULT_OFFSET);
	mpu.setZAccelOffset(DEFAULT_OFFSET);
	mpu.setXGyroOffset(DEFAULT_OFFSET);
	mpu.setYGyroOffset(DEFAULT_OFFSET);
	mpu.setZGyroOffset(DEFAULT_OFFSET);
}

void MPU::meansensors(){
	long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
	int16_t ax, ay, az,gx, gy, gz;

	while (i<(BUFFER_SIZE+101)){
		// read raw accel/gyro measurements from device
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		
		if (i>DISCARDED_MEASURMENTS && i<=(BUFFER_SIZE+100)){ //First 100 measures are discarded
			buff_ax=buff_ax+ax;
			buff_ay=buff_ay+ay;
			buff_az=buff_az+az;
			buff_gx=buff_gx+gx;
			buff_gy=buff_gy+gy;
			buff_gz=buff_gz+gz;
		}
		if (i==(BUFFER_SIZE+100)){
			mean_ax=buff_ax/BUFFER_SIZE;
			mean_ay=buff_ay/BUFFER_SIZE;
			mean_az=buff_az/BUFFER_SIZE;
			mean_gx=buff_gx/BUFFER_SIZE;
			mean_gy=buff_gy/BUFFER_SIZE;
			mean_gz=buff_gz/BUFFER_SIZE;
		}
		i++;
		delay(2); //Needed so we don't get repeated measures
	}
}

void MPU::calibration(){
	ax_offset=-mean_ax/8;
	ay_offset=-mean_ay/8;
	az_offset=(16384-mean_az)/8;

	gx_offset=-mean_gx/4;
	gy_offset=-mean_gy/4;
	gz_offset=-mean_gz/4;
	while (1){
		int ready=0;
		mpu.setXAccelOffset(ax_offset);
		mpu.setYAccelOffset(ay_offset);
		mpu.setZAccelOffset(az_offset);

		mpu.setXGyroOffset(gx_offset);
		mpu.setYGyroOffset(gy_offset);
		mpu.setZGyroOffset(gz_offset);

		meansensors();
		bt->println("...");

		if (abs(mean_ax)<=ACEL_DEADZONE) ready++;
		else ax_offset=ax_offset-mean_ax/ACEL_DEADZONE;

		if (abs(mean_ay)<=ACEL_DEADZONE) ready++;
		else ay_offset=ay_offset-mean_ay/ACEL_DEADZONE;

		if (abs(16384-mean_az)<=ACEL_DEADZONE) ready++;
		else az_offset=az_offset+(16384-mean_az)/ACEL_DEADZONE;

		if (abs(mean_gx)<=GYRO_DEADZONE) ready++;
		else gx_offset=gx_offset-mean_gx/(GYRO_DEADZONE+1);

		if (abs(mean_gy)<=GYRO_DEADZONE) ready++;
		else gy_offset=gy_offset-mean_gy/(GYRO_DEADZONE+1);

		if (abs(mean_gz)<=GYRO_DEADZONE) ready++;
		else gz_offset=gz_offset-mean_gz/(GYRO_DEADZONE+1);

		if (ready==6) break;
	}
}

void MPU::mpu6050FixOffset(){
	int state = 0;
	if (state==0){
		bt->println("Reading sensors for first time...");
		meansensors();
		state++;
		delay(1000);
	}

	if (state==1) {
		bt->println("Calculating offsets...");
		calibration();
		state++;
		delay(1000);
	}

	if (state==2) {
		meansensors();
		mpu.getIntStatus();	// TODO: FIX IT - NO good fix for fifo overflow in calibaration
		mpu.resetFIFO();  // 							-||-
		bt->println("Finished.");
	}
}

void MPU::mpu6050GetMeasurementsAUX(){
	// display Euler angles in degrees
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

	// display real acceleration, adjusted to remove gravity
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
}

void MPU::mpu6050Restart(){
	mpu6050Initialize();
	mpu6050FixOffset();
}

void MPU::mpu6050GetMeasurements(float* Ret_ypr,VectorInt16 &Ret_aaReal){
	if (!dmpReady) return;  // if dmp initialization failed, don't try to do anything  

	while (!mpuInterrupt && fifoCount < packetSize){} // wait for MPU interrupt or extra packet(s) available 

	// reset interrupt flag and get INT_STATUS byte 
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	fifoCount = mpu.getFIFOCount(); // get current FIFO count 

	// check for overflow (this should never happen unless our code is too inefficient) 
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly 
		mpu.resetFIFO();
		//bt->println("FIFO overflow!");	// TODO - fix it

		// otherwise, check for DMP data ready interrupt (this should happen frequently) 
	} else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait 
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO  
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		
		// track FIFO count here in case there is > 1 packet available  
		// (this lets us immediately read more without waiting for an interrupt)  
		fifoCount -= packetSize;

		//  Sampling and and printing MPU6050 values to the bt 
		mpu6050GetMeasurementsAUX();
	}
	Ret_ypr[0]=ypr[0];
	Ret_ypr[1]=ypr[1];
	Ret_ypr[2]=ypr[2];
	Ret_aaReal.x=aaReal.x;
	Ret_aaReal.y=aaReal.y;
	Ret_aaReal.z=aaReal.z;
}