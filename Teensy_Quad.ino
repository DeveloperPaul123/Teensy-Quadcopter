/**
* Custom flight controller written from scratch. Definitely a work in progress. 
* Copyright 2015 Paul T
**/

#include <i2c_t3.h>
#include "PID_v1.h"
#include "Constants.h"
#include "MedianFilter.h"
#include "QuadConfig.h"
#include "FrequencyEvent.h"


// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

/*
* Specify sensor full scale
*/
int Gscale = GFS_2000DPS;
int Ascale = AFS_8G;
float aRes, gRes; // scale resolutions per LSB for the sensors
  
#define blinkPin 13  // Blink LED on Teensy

//Motor pins, 23 and 4 are the front, 3, 22 are the back.
/* Diagram

	    Front
   M1-----------M2
        |  |
        |  |
        |  |
        |  |
   M3-----------M4
		Back

*/

#define M1 23
#define M2 4
#define M3 3
#define M4 22


/* Interrupt lock
 *
 */
boolean interruptLock = false;

/*  
 * //////////////// RC variables //////////////////////////
 */
volatile float ch1, ch2, ch3, ch4, ch5, ch6;         // RC channel inputs

volatile unsigned long rcLastChange1 = micros();
volatile unsigned long rcLastChange2 = micros();
volatile unsigned long rcLastChange3 = micros();
volatile unsigned long rcLastChange4 = micros();
volatile unsigned long rcLastChange5 = micros();
volatile unsigned long rcLastChange6 = micros();
/////////////////////////////////////////////////

//blink variable for  LED (pin 13)
boolean blinkOn = false;

/*
* Stores the 16-bit signed accelerometer sensor output
*/
int16_t accelCount[3];  

/*
* Stores the real accel value in g's
*/
float ax, ay, az;   

/*
* Stores filtered accelerometer values. 
*/   
double filt_ax, filt_ay, filt_az; 

/*
* Stores the 16-bit signed gyro sensor output
*/
int16_t gyroCount[3];   

/*
* Stores the real gyro value in degrees per seconds
*/
double gx, gy, gz;  
 
/*
* Stores the filtered gyro values.     
*/
double filt_gx, filt_gy, filt_gz; 

/*
* Stores the biases for accelerometer and gyroscope. On level ground for
* this system I got:
* Gyro: 	-1.82	0.49	0.24	Accel: 	0.01	-0.02	-0.00
*/
float gyroBias[3] = {-1.89, 0.49, 0.24}, accelBias[3] = {0.01, -0.02, 0}; // Bias corrections for gyro and accelerometer

/*
* Stores the real internal chip temperature in degrees Celsius
*/
int16_t tempCount;   

/*
* Stores the temp in a readable float. 
*/
float temperature;

/*
* Holder for the selftest values. 
*/
float SelfTest[6];

// used to control display output rate
uint32_t delt_t = 0; 
// used to control display output rate
uint32_t count = 0;  

// parameters for 6 DoF sensor fusion calculations

/*
* Gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
*/
float GyroMeasError = PI * (40.0f / 180.0f);  

/*
* compute beta   
*/
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  

/*
* Gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
*/
float GyroMeasDrift = PI * (2.0f / 180.0f);

/*
* compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value     
*/
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  

/*
* Holders for yaw pitch and roll values. Need to be doubles for the PID
*/
double pitch, yaw, roll;

/*
* Holds values of offsets for yaw, pitch, roll values. Used in calibrating the yaw, pitch roll values. 
*/
double pitchCorrect, yawCorrect, rollCorrect;

/*
* Number of samples taken during setup for YPR calibration. 
*/
int numSamples;

/**
* Values used in MadgwickQuaternionUpdate function. 
*/ 

/*
* Integration interval for both filter schemes
*/
float deltat = 0.0f;    

/*
* Used to calculate integration interval
*/
uint32_t lastUpdate = 0, firstUpdate = 0;    

/*
* Used to calculate integration interval
*/     
uint32_t Now = 0;              

/*
* Vector to hold quaternion
*/                   
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            

/*
* ////// PID ///////
*/
double PITCH_P_VAL = 0.3;
double PITCH_I_VAL = 0.1;
double PITCH_D_VAL = 0; 

double ROLL_P_VAL = 0.45;
double ROLL_I_VAL = 0.4;
double ROLL_D_VAL = 0;

double YAW_P_VAL = 35;
double YAW_I_VAL = 15;
double YAW_D_VAL = 0;

/*
* Offsets. 
*/
double pitch_offset, roll_offset, yaw_offset;

/*
* Values read in from the controller. 
*/
double pitch_input, roll_input, yaw_input;

/*
	PIDs for yaw, pitch and roll. 
*/
PID pitchPID(&pitch, &pitch_offset, &pitch_input, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollPID(&roll, &roll_offset, &roll_input, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);
PID yawPID(&filt_gz, &yaw_offset, &yaw_input, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);

/*
* Rate PID Stuff. 
*/
double PITCH_RATE_P_VAL = 49;
double PITCH_RATE_I_VAL = 45;
double PITCH_RATE_D_VAL = 0;

double ROLL_RATE_P_VAL = 50;
double ROLL_RATE_I_VAL = 45;
double ROLL_RATE_D_VAL = 0;

double pitch_rate_offset, roll_rate_offset;
double pitch_rate_input, roll_rate_input;

/*
* Rate PIDS for roll and pitch, yaw is already rate based. 
* Inputs are outputs from orientation based controllers. 
*/
PID ratePitchPID(&filt_gy, &pitch_rate_offset, &pitch_rate_input, PITCH_RATE_P_VAL, PITCH_RATE_I_VAL, PITCH_RATE_D_VAL, REVERSE);
PID rateRollPID(&filt_gx, &roll_rate_offset, &roll_rate_input, ROLL_RATE_P_VAL, ROLL_RATE_I_VAL, ROLL_RATE_D_VAL, REVERSE);

/*
* Used to read the throttle. 
*/
int throttle, lastThrottle, throttle_input;

/*
* Values written to motors to control them. 
*/
int m_one, m_two, m_three, m_four;


/**
* Bluetooth stuff.
**/
const uint16_t BUFFERSIZE = 4;
char buffer[BUFFERSIZE];
volatile bool update_flag = false;

const char CMD_T = 'T';
const char CMD_Y = 'Y';
const char CMD_P = 'P';
const char CMD_R = 'R';
const char CMD_E = 'E';
const char CMD_D = 'D';
const char ROLL_KP = 'A';
const char ROLL_KI = 'B';
const char ROLL_KD = 'C';
const char YAW_KP = 'F';
const char YAW_KI = 'G';
const char YAW_KD = 'H';
const char PITCH_KP = 'I';
const char PITCH_KI = 'J';
const char PITCH_KD = 'K';
const char CHANGE_FLIGHT_MODE = 'M';

const char SEND_PID = 'L';

volatile char incomByte;
char throttleData[4];
volatile byte throttleIndex = 0;
char yawData[4];
volatile byte yawIndex = 0;
char pitchData[4];
volatile byte pitchIndex = 0;
char rollData[4];
volatile byte rollIndex = 0;

char pidData[4];
volatile byte pidIndex = 0;

volatile char command;
volatile char startCommand;

boolean isCalibrated;
long calibrationStart;
boolean sendToBluetooth = false;

/*
* Flymode for the quad, either auto level or acrobatic mode. 
*/
int flyMode;
const int ACRO_MODE = 13;
const int AUTO_MODE = 14;

/*
* Filters for gyro.
*/
MedianFilter gxFilter;
MedianFilter gyFilter;
MedianFilter gzFilter;

/*
* Filters for accelerometer. 
*/
MedianFilter axFilter;
MedianFilter ayFilter;
MedianFilter azFilter;

/**
* Frequency based event handlers. 
**/
FrequencyEvent oneHunHzEvent;

/**
* Setup everything, calibrate, self test...blah blah. 
*/
void setup()
{
	Wire.begin();
	
	#if defined(DEBUG) || defined(DEBUG_CONTROLS) || defined(DEBUG_MPU) || defined(FIRST_RUN) || defined(DEBUG_RC) || defined(COMPUTE_BIAS_CURVE)
		Serial.begin(115200);
	#endif
	
	#ifdef BLUETOOTH_CONTROL
		BLUETOOTH.begin(115200);
	#endif
	
	pinMode(blinkPin, OUTPUT);
	digitalWrite(blinkPin, HIGH);
	delay(1000);
	digitalWrite(blinkPin, LOW);
	delay(1000);
	digitalWrite(blinkPin, HIGH);
	delay(1000);
	
	
	//setup motor pins.
	pinMode(M1, OUTPUT);
	delay(10);
	pinMode(M2, OUTPUT);
	delay(10);
	pinMode(M3, OUTPUT);
	delay(10);
	pinMode(M4, OUTPUT);

	/*
		Set write resolutions, the corresponding frequencies are based on this
		table: https://www.pjrc.com/teensy/td_pulse.html, note that when you change the 
		frequency of one pin on a FTD timer, they all change, so in this case only FTM2 is unchanged. 
		FTM0: 5, 6, 9, 10, 20, 21, 22, 23
		FTM1: 3, 4
		FTM2: 25, 32
	*/
	#ifdef DEBUG
		Serial.print("PWM Resolution: ");
		Serial.println(PWM_RESOLUTION);
		Serial.print("PWM Max: ");
		Serial.println(PWM_MAX);
		Serial.print("PID Influence: ");
		Serial.println(PID_INFLUENCE);
	#endif
	
	switch (PWM_RESOLUTION) {
		case 8:
			analogWriteResolution(8);
			analogWriteFrequency(M1, 187500);
			analogWriteFrequency(M2, 187500);
			analogWriteFrequency(M3, 187500);
			analogWriteFrequency(M4, 187500);
			break;
		case 9:
			analogWriteResolution(9);
			analogWriteFrequency(M1, 93750);
			analogWriteFrequency(M2, 93750);
			analogWriteFrequency(M3, 93750);
			analogWriteFrequency(M4, 93750);
		case 10:
			analogWriteResolution(10);
			analogWriteFrequency(M1, 46875);
			analogWriteFrequency(M2, 46875);
			analogWriteFrequency(M3, 46875);
			analogWriteFrequency(M4, 46875);
			break;
		case 11:
			analogWriteResolution(11);
			analogWriteFrequency(M1, 23438);
			analogWriteFrequency(M2, 23438);
			analogWriteFrequency(M3, 23438);
			analogWriteFrequency(M4, 23438);
		case 12:
			analogWriteResolution(12);
			analogWriteFrequency(M1, 11719);
			analogWriteFrequency(M2, 11719);
			analogWriteFrequency(M3, 11719);
			analogWriteFrequency(M4, 11719);
			break;
		case 13:
			analogWriteResolution(13);
			analogWriteFrequency(M1, 5859);
			analogWriteFrequency(M2, 5859);
			analogWriteFrequency(M3, 5859);
			analogWriteFrequency(M4, 5859);
			break;
		case 14:
			analogWriteResolution(14);
			analogWriteFrequency(M1, 2930);
			analogWriteFrequency(M2, 2930);
			analogWriteFrequency(M3, 2930);
			analogWriteFrequency(M4, 2930);
			break;
		case 15:
			analogWriteResolution(15);
			analogWriteFrequency(M1, 1465);
			analogWriteFrequency(M2, 1465);
			analogWriteFrequency(M3, 1465);
			analogWriteFrequency(M4, 1465);
			break;
		case 16:
			analogWriteResolution(16);
			analogWriteFrequency(M1, 732);
			analogWriteFrequency(M2, 732);
			analogWriteFrequency(M3, 732);
			analogWriteFrequency(M4, 732);
			break;
	}
	
	delay(1000);

	// Read the WHO_AM_I register, this is a good test of communication
	uint8_t c = readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
	Serial.println(c);
	delay(1000); 

	// WHO_AM_I should always be 0x68
	if (c == 0x68) 
	{  
	Serial.println("MPU6050 is online...");

	// Start by performing self test and reporting values
	MPU6050SelfTest(SelfTest); 

	if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
		Serial.println("Self test passed");
		delay(1000);
		
		// Calibrate gyro and accelerometers, load biases in bias registers
		calibrateMPU6050(gyroBias, accelBias);   
		#ifdef FIRST_RUN
			Serial.print("Gyro: ");
			printTab();
			Serial.print(gyroBias[0]);
			printTab();
			Serial.print(gyroBias[1]);
			printTab();
			Serial.print(gyroBias[2]);
			printTab();
			Serial.print("Accel: ");
			printTab();
			Serial.print(accelBias[0]);
			printTab();
			Serial.print(accelBias[1]);
			printTab();
			Serial.println(accelBias[2]);
		#endif
				
		delay(1000); 
		
		// Initialize device for active mode read of acclerometer, gyroscope, and temperature
		initMPU6050(); Serial.println("MPU6050 initialized for active data mode...."); 
	}
	else
	{
		Serial.print("Could not connect to MPU6050: 0x");
		Serial.println(c, HEX);
		digitalWrite(blinkPin, HIGH);
		while(1) ; // Loop forever if communication doesn't happen
		}
	}
	
	/*
		Initialize values.
	*/
	throttle = 0;
	throttle_input = 0;
	lastThrottle = 0;
	pitch_offset = 0.0f;
	roll_offset = 0.0f;
	yaw_offset = 0.0f;
	pitch_input = 0.0f;
	roll_input = 0.0f;
	yaw_input = 0.0f;
	pitch_rate_input = 0.0f;
	roll_rate_input = 0.0f;
	
	m_one = 0;
	m_two = 0;
	m_three = 0;
	m_four = 0;
	
	numSamples = 0;
	pitchCorrect = 0;
	yawCorrect = 0;
	rollCorrect = 0;
	
	isCalibrated = false;
	calibrationStart = 0;
	blinkOn = true;
	
	// flyMode = AUTO_MODE;
	flyMode = ACRO_MODE;
	
	/*
		Setup pids
	*/
	pitchPID.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
	pitchPID.SetMode(AUTOMATIC);
	pitchPID.SetSampleTime(10); //100 Hz so 100 times per second = once every 10 ms
	
	rollPID.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
	rollPID.SetMode(AUTOMATIC);
	rollPID.SetSampleTime(10); 
		
	yawPID.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);
	yawPID.SetMode(AUTOMATIC);
	yawPID.SetSampleTime(10); 
	
	ratePitchPID.SetOutputLimits(-RATE_PITCH_PID_INFLUENCE, RATE_PITCH_PID_INFLUENCE);
	ratePitchPID.SetMode(AUTOMATIC);
	ratePitchPID.SetSampleTime(10); 
	
	rateRollPID.SetOutputLimits(-RATE_ROLL_PID_INFLUENCE, RATE_ROLL_PID_INFLUENCE);
	rateRollPID.SetMode(AUTOMATIC);
	rateRollPID.SetSampleTime(10); 
	
	//initialize the filters. 
	gyFilter.begin();
	gxFilter.begin();
	gzFilter.begin();
	
	axFilter.begin();
	ayFilter.begin();
	azFilter.begin();
	
	#ifdef HAS_RC
		initRC();
	#endif
	
	//set frequency to 100 Hz for PIDs
	oneHunHzEvent.setFrequency(100);
	
}

/**
* Called when serial data is available. 
* This should run after each loop. 
**/ 
void serialEvent1() {
	int x = 0;
	if(BLUETOOTH.available()>=BUFFERSIZE) {
		// throttle = BLUETOOTH.read();
		// yaw = BLUETOOTH.read();
		// pitch = BLUETOOTH.read();
		// roll = BLUETOOTH.read();
		for(int i = 0; i < BUFFERSIZE; i++) {
			buffer[i] = BLUETOOTH.read();
		}
		update_flag = true;
	}
	
}

/**
* Main loop of controller. Does one final calibration and then runs main controller. 
*/
void loop() {

	#ifdef COMPUTE_BIAS_CURVE
		/*
		* Going to calculate a calibration curve for the gyro and accelerometer
		* for temperature compensation. Read values 100 times/second and then
		* construct a curve using serial monitor values. 
		*/
		if(calibrationStart == 0) { //should only be called once. 
			calibrationStart = millis();
		}
		//get the biases. 
		calibrateMPU6050(gyroBias, accelBias);
		//setup the mpu6050 to read values. 
		initMPU6050();
		//update the values
		readRawMPUData();
		//read values 10 times per second. 
		if(millis() - calibrationStart >= 10) {
			//set the time. 
			calibrationStart = millis();
			
			Serial.print(gx);
			Serial.print(",");
			Serial.print(gy);
			Serial.print(",");
			Serial.print(gz);
			Serial.print(",");
			Serial.print(gyroBias[0]);
			Serial.print(",");
			Serial.print(gyroBias[1]);
			Serial.print(",");
			Serial.print(gyroBias[2]);
			Serial.print(",");
			Serial.print(ax);
			Serial.print(",");
			Serial.print(ay);
			Serial.print(",");
			Serial.print(az);
			Serial.print(",");
			Serial.print(accelBias[0]);
			Serial.print(",");
			Serial.print(accelBias[1]);
			Serial.print(",");
			Serial.print(accelBias[2]);
			Serial.print(",");
			Serial.println(temperature);
		}
	#else
		if(!isCalibrated) {
			//TODO: Calibration doesn't really do anything now. Need to make it actually work. 
			if(calibrationStart == 0) {
				calibrationStart = millis();
			}
			if(millis() - calibrationStart > 4000) {
				isCalibrated = true;
				digitalWrite(blinkPin, LOW);
				delay(500);
				digitalWrite(blinkPin, HIGH);
				delay(500);
				digitalWrite(blinkPin, LOW);
				pitchCorrect/=numSamples;
				yawCorrect/=numSamples;
				rollCorrect/=numSamples;
				numSamples = 0;
				oneHunHzEvent.start();
				
			} else {
				digitalWrite(blinkPin, blinkOn);
				blinkOn = !blinkOn;
				updateYPR();
				pitchCorrect+=pitch;
				yawCorrect+=yaw;
				rollCorrect+=roll;
				numSamples+=1;
			}
			
		} else {
			
			//main loop. 
			
			//read data.
			readRawMPUData();
			
			//update yaw, pitch roll values.
			if(flyMode == AUTO_MODE) {
				updateYPR();
			}
			
			//run the filters. 
			runFilters();
	
			/*
				Update the controls if the update flag was set in the serialEvent1 callback.
			*/
			if(update_flag) {
				//lock in case there are interrupts. 
				acquireLock();
				for(int i = 0; i < BUFFERSIZE; i++) {
					throttle = buffer[0];
					int temp_yaw = buffer[1];
					int temp_pitch = buffer[2];
					int temp_roll = buffer[3];
					if(temp_yaw == 0){
						yaw_input = 0;
					} else {
						yaw_input = (double) map(temp_yaw, 0, 100, YAW_MIN, YAW_MAX);
					}
					
					if(flyMode == AUTO_MODE) {
						pitch_input = (double) map(temp_pitch, 0, 100, PITCH_MIN, PITCH_MAX);
						roll_input = (double) map(temp_roll, 0, 100, ROLL_MIN, ROLL_MAX);
					} else if(flyMode == ACRO_MODE) {
						pitch_rate_input = (double) map(temp_pitch, 0, 100, RATE_PITCH_MIN, RATE_PITCH_MAX);
						roll_rate_input = (double) map(temp_roll, 0, 100, RATE_ROLL_MIN, RATE_ROLL_MAX);
					}
					
					#ifdef DEBUG_CONTROLS
						Serial.print(throttle);
						printTab();
						Serial.print(temp_yaw);
						printTab();
						Serial.print(yaw_input);
						printTab();
						Serial.print(pitch_input);
						printTab();
						Serial.println(roll_input);
					#endif
				}
				update_flag = false;
				releaseLock();
			}
			
			#if defined(DEBUG) || defined(DEBUG_CONTROLS)
				readSerialControls();
			#endif
			//run pids at 100Hz
			if(oneHunHzEvent.shouldRunEvent()) {

			#if defined(DEBUG)
				Serial.print("100 Hz Event Time: ");
				Serial.println(micros());
			#endif
				//compute PIDS now. 
				computePID();
			}

			//update motor values. 
			updateMotors();
			
			#ifdef DEBUG
				printValues();
			#endif
			#ifdef DEBUG_CONTROLS
				//printControlValues();
			#endif
			#ifdef DEBUG_RC
				printRCValues();
			#endif
			if(sendToBluetooth && (millis() - count) >= 500) {
				printBluetoothValues();
				count = millis();
			}
			
		}
	
	#endif
	
}

/**
* Runs filters for the gyroscope and accelerometer. 
**/
inline void runFilters() {
	filt_gy = gyFilter.run(gy);
	filt_gz = gzFilter.run(gz);
	filt_gx = gxFilter.run(gx);
	
	filt_ax = axFilter.run((double) ax);
	filt_ay = ayFilter.run((double) ay);
	filt_az = azFilter.run((double) az);
}

/**
* Read the raw data of the MPU6050
*/
inline void readRawMPUData() {
	// If data ready bit set, all data registers have new data
  if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();
    
	// Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes;   
    az = (float)accelCount[2]*aRes;  
   
    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();
 
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes;  
    gz = (float)gyroCount[2]*gRes;
   }  
}

/**
* Update the yaw pitch roll values. 
*/
inline void updateYPR() {
   
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, q);
	
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    roll  *= 180.0f / PI;

	#ifdef DEBUG_MPU
		Serial.print(yaw, 2);
		printTab();
		Serial.print(pitch, 2);
		printTab();
		Serial.print(roll, 2);
		printTab();
		Serial.print((int)1000*ax); 
		printTab();
		Serial.print((int)1000*ay);
		printTab();
		Serial.print((int)1000*az);
		printTab();
		Serial.print(filt_ax);
		printTab();
		Serial.print(filt_ay);
		printTab();
		Serial.print(filt_az);
		printTab();
		Serial.print( gx, 1);
		printTab();
		Serial.print( gy, 1);
		printTab();
		Serial.print( gz, 1);
		printTab();
		Serial.print(filt_gx);
		printTab();
		Serial.print(filt_gy);
		printTab();
		Serial.println(filt_gz);
	#endif
}




/**
* Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
* (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
* which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
* device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
* The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
* but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
* @param ax the x acceleration in g's
* @param ay the y acceleration in g's
* @param az the z acceleration in g's 
* @param gx the x angular velocity in radians/s
* @param gy the y angular velocry in radians/s
* @param gz the z angular velocity in radians/s 
* @param q* the an array to hold the updated quaternion values. 
*/
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float * q) {
	//initialize the quaternion values. 
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
	float norm;                                               // vector norm
	float f1, f2, f3;                                         // objetive funcyion elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float qDot1, qDot2, qDot3, qDot4;
	float hatDot1, hatDot2, hatDot3, hatDot4;
	float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

	// Auxiliary variables to avoid repeated arithmetic
	float _halfq1 = 0.5f * q1;
	float _halfq2 = 0.5f * q2;
	float _halfq3 = 0.5f * q3;
	float _halfq4 = 0.5f * q4;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Compute the objective function and Jacobian
	f1 = _2q2 * q4 - _2q1 * q3 - ax;
	f2 = _2q1 * q2 + _2q3 * q4 - ay;
	f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
	J_11or24 = _2q3;
	J_12or23 = _2q4;
	J_13or22 = _2q1;
	J_14or21 = _2q2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;

	// Compute the gradient (matrix multiplication)
	hatDot1 = J_14or21 * f2 - J_11or24 * f1;
	hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
	hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
	hatDot4 = J_14or21 * f1 + J_11or24 * f2;

	// Normalize the gradient
	norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
	hatDot1 /= norm;
	hatDot2 /= norm;
	hatDot3 /= norm;
	hatDot4 /= norm;

	// Compute estimated gyroscope biases
	gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
	gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
	gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

	// Compute and remove gyroscope biases
	gbiasx += gerrx * deltat * zeta;
	gbiasy += gerry * deltat * zeta;
	gbiasz += gerrz * deltat * zeta;
	gx -= gbiasx;
	gy -= gbiasy;
	gz -= gbiasz;

	// Compute the quaternion derivative
	qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
	qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
	qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
	qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

	// Compute then integrate estimated quaternion derivative
	q1 += (qDot1 -(beta * hatDot1)) * deltat;
	q2 += (qDot2 -(beta * hatDot2)) * deltat;
	q3 += (qDot3 -(beta * hatDot3)) * deltat;
	q4 += (qDot4 -(beta * hatDot4)) * deltat;

	// Normalize the quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

/**
* Updates the values of the motors. 
**/
inline void updateMotors() {

	//motors 3 and 4 are the front.
	//motors 1 and 2 are on the back.
	//motors 1 and 4 spin the same way.
	//motors 2 and 3 spin the same way. 
	
	if(throttle_input >= MIN_THROTTLE) {
		m_one = (int) throttle_input - (int) pitch_rate_offset/2 + (int) roll_rate_offset/2 - (int) yaw_offset/2;
		m_two = (int) throttle_input - (int) pitch_rate_offset/2 - (int) roll_rate_offset/2 + (int) yaw_offset/2;
		m_three = (int) throttle_input + (int) pitch_rate_offset/2 + (int) roll_rate_offset/2 + (int) yaw_offset/2;
		m_four = (int) throttle_input + (int) pitch_rate_offset/2 - (int) roll_rate_offset/2 - (int) yaw_offset/2;
	} else if(throttle_input == 0) {
		m_one = 0;
		m_two = 0;
		m_three = 0;
		m_four = 0;
	} else {
		m_one = throttle_input;
		m_two = throttle_input;
		m_three = throttle_input;
		m_four = throttle_input;
	}
	//make sure motors are enabled. 	
	#ifndef DISABLE_MOTORS
		analogWrite(M1, m_one);	
		analogWrite(M2, m_two);
		analogWrite(M3, m_three);
		analogWrite(M4, m_four);
	#endif
	
	lastThrottle = throttle;
}

/**
* Compute the next PIDs. 
*/
inline void computePID() {
	acquireLock();
	/*	
		need to compute the RC Channels if using RC
		ch1 = roll
		ch2 = pitch
		ch3 = throttle
		ch4 = yaw
		ch5 = aux 1
		ch6 = aux 2
	*/
	#ifdef HAS_RC
		roll_input = (double) map(ch1, RC_LOW_CH1, RC_HIGH_CH1, ROLL_MIN, ROLL_MAX);
		pitch_input = (double) map(ch2, RC_LOW_CH1, RC_HIGH_CH1, PITCH_MIN, PITCH_MAX);
		throttle_input = (double) map(ch3, RC_LOW_CH1, RC_HIGH_CH1, MIN_THROTTLE, MAX_THROTTLE);
		yaw_input = (double) map(ch4, RC_LOW_CH1, RC_HIGH_CH1, YAW_MIN, YAW_MAX);
	#endif
	
	#ifdef BLUETOOTH_CONTROL
		if(throttle != 0) {
			throttle_input = map(throttle, 0, 255, MIN_THROTTLE, MAX_THROTTLE);
		} else {
			throttle_input = 0;
		}
	#endif
	
	switch (flyMode) {
		case ACRO_MODE:
			//inputs are directly read from the controller, see loop 
			//compute rotational rate outputs.
			ratePitchPID.Compute();
			rateRollPID.Compute();
			yawPID.Compute();	
			break;
		case AUTO_MODE:
			//compute based on inputs first.
			if(pitchPID.Compute()) {
				//we computed, now we scale the ouput of the pitchPID value to the inputs
				//of the ratePitchPID values
				pitch_rate_input = map(pitch_offset, -PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE, RATE_PITCH_MIN, RATE_PITCH_MAX);
				ratePitchPID.Compute();
			}
			if(rollPID.Compute()) {
				//we computed now we sacle output of rollPID value to the inputs
				//of the rollRatePID values.
				roll_rate_input = map(roll_offset, -PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE, RATE_ROLL_MIN, RATE_ROLL_MAX);
				rateRollPID.Compute();
			}
			//compute new rates. 
			yawPID.Compute();
			break;
		default:
			//compute based on inputs first.
			pitchPID.Compute();
			rollPID.Compute();
			yawPID.Compute();
			//compute new rates. 
			ratePitchPID.Compute();
			rateRollPID.Compute();
			break;
	}
	
	releaseLock();
	
}

inline void readControlsFast() {
	#ifdef HAS_RC
		acquireLock();
	#endif
	//wait for 4 bytes. 
	//should be throttle, yaw, pitch, roll
	if(BLUETOOTH.available() >= 4) {
		throttle = BLUETOOTH.read();
		yaw_input = BLUETOOTH.read();
		pitch_input = BLUETOOTH.read();
		roll_input = BLUETOOTH.read();
		BLUETOOTH.flush();
	}
	
	#ifdef HAS_RC
		releaseLock();
	#endif
}

/**
* Read the values from the Serial Monitor (aka Bluetooth)
**/
inline void readControls() {
	acquireLock();
	if(BLUETOOTH.available() > 0) {
		incomByte = BLUETOOTH.read();		
		if(incomByte == CMD_T) {
			command = CMD_T;
			memset(throttleData, 0, sizeof(throttleData));
			throttleIndex = 0;
		}
		else if(incomByte == CMD_Y) {
			command = CMD_Y;	
			memset(yawData, 0, sizeof(yawData));
			yawIndex = 0;
		}
		else if(incomByte == CMD_P) {
			command = CMD_P;
			memset(pitchData, 0, sizeof(pitchData));
			pitchIndex = 0;
		}
		else if(incomByte == CMD_R) {
			command = CMD_R;
			memset(rollData, 0, sizeof(rollData));
			rollIndex = 0;
		}
		else if(incomByte == CMD_E) {
			startCommand = command;
			command = CMD_E;
		} 
		else if(incomByte == CMD_D) {
			command = CMD_D;
		}
		else if(incomByte == YAW_KP) {
			command = YAW_KP;
			memset(pidData, 0, sizeof(pidData));
			pidIndex = 0;
		}
		else if(incomByte == YAW_KI) {
			command = YAW_KI;
			memset(pidData, 0, sizeof(pidData));
			pidIndex = 0;
		}
		else if(incomByte == YAW_KD) {
			command = YAW_KD;
			memset(pidData, 0, sizeof(pidData));
			pidIndex = 0;
		}
		else if(incomByte == PITCH_KP) {
			command = PITCH_KP;
			memset(pidData, 0, sizeof(pidData));
			pidIndex = 0;
		}
		else if(incomByte == PITCH_KI) {
			command = PITCH_KI;
			memset(pidData, 0, sizeof(pidData));
			pidIndex = 0;
		}
		else if(incomByte == PITCH_KD) {
			command = PITCH_KD;
			memset(pidData, 0, sizeof(pidData));
			pidIndex = 0;
		}
		else if(incomByte == ROLL_KP) {
			command = ROLL_KP;
			memset(pidData, 0, sizeof(pidData));
			pidIndex = 0;
		}
		else if(incomByte == ROLL_KI) {
			command = ROLL_KI;
			memset(pidData, 0, sizeof(pidData));
			pidIndex = 0;
		}
		else if(incomByte == ROLL_KD) {
			command = ROLL_KD;
			memset(pidData, 0, sizeof(pidData));
			pidIndex = 0;
		}
		else if(incomByte == SEND_PID) {
			command = SEND_PID;
			memset(pidData, 0, sizeof(pidData));
			pidIndex = 0;
		}
		else if(incomByte == CHANGE_FLIGHT_MODE) {
			command = CHANGE_FLIGHT_MODE;	
		}
		
		if(command == CMD_T && incomByte != CMD_T) {
			throttleData[throttleIndex] = incomByte;
			throttleIndex++;
		}
		else if(command == CMD_Y && incomByte != CMD_Y) {
			yawData[yawIndex] = incomByte;
			yawIndex++;
		}
		else if(command == CMD_P && incomByte != CMD_P) {
			pitchData[pitchIndex] = incomByte;
			pitchIndex++;
		}
		else if(command == CMD_R && incomByte != CMD_R) {
			rollData[rollIndex] = incomByte;
			rollIndex++;
		}
		else if(command == CMD_D) {
			sendToBluetooth = !sendToBluetooth;
			command = ' ';
			startCommand = ' ';
		}
		else if(command == CHANGE_FLIGHT_MODE) {
			if(flyMode == ACRO_MODE) {
				flyMode = AUTO_MODE;
			} else {
				flyMode = ACRO_MODE;
			}
			command = ' ';
			startCommand = ' ';
		}
		else if(command == SEND_PID) {
			BLUETOOTH.print("P,");
			BLUETOOTH.print(YAW_P_VAL);
			BLUETOOTH.print(",");
			BLUETOOTH.print(YAW_I_VAL);
			BLUETOOTH.print(",");
			BLUETOOTH.print(YAW_D_VAL);
			BLUETOOTH.print(",");
			BLUETOOTH.print(PITCH_P_VAL);
			BLUETOOTH.print(",");
			BLUETOOTH.print(PITCH_I_VAL);
			BLUETOOTH.print(",");
			BLUETOOTH.print(PITCH_D_VAL);
			BLUETOOTH.print(",");
			BLUETOOTH.print(ROLL_P_VAL);
			BLUETOOTH.print(",");
			BLUETOOTH.print(ROLL_I_VAL);
			BLUETOOTH.print(",");
			BLUETOOTH.print(ROLL_D_VAL);
			BLUETOOTH.print(",");
			BLUETOOTH.println("E");
			
			command = ' ';
		}
		else if(command == (YAW_KP||YAW_KI||YAW_KD||PITCH_KP||PITCH_KI||PITCH_KD||ROLL_KP||ROLL_KI||ROLL_KD) &&
			incomByte != (YAW_KP||YAW_KI||YAW_KD||PITCH_KP||PITCH_KI||PITCH_KD||ROLL_KP||ROLL_KI||ROLL_KD)) {
			pidData[pidIndex] = incomByte;
			pidIndex++;
		}
		else if(command == CMD_E && startCommand != (YAW_KP||YAW_KI||YAW_KD||PITCH_KP||PITCH_KI||PITCH_KD||ROLL_KP||ROLL_KI||ROLL_KD)) {
			throttle = atoi(throttleData);
			if(throttle > MAX_THROTTLE) {
				throttle = MAX_THROTTLE;
			}
			pitch_input = strtod(pitchData, NULL);
			roll_input = strtod(rollData, NULL);
			yaw_input = strtod(yawData, NULL);
			command = ' ';
			startCommand = ' ';
			memset(throttleData, 0, sizeof(throttleData));
			throttleIndex = 0;
			memset(yawData, 0, sizeof(yawData));
			yawIndex = 0;
			memset(pitchData, 0, sizeof(pitchData));
			pitchIndex = 0;
			memset(rollData, 0, sizeof(rollData));
			rollIndex = 0;
		}
		else if(command == CMD_E && startCommand == (YAW_KP||YAW_KI||YAW_KD||PITCH_KP||PITCH_KI||PITCH_KD||ROLL_KP||ROLL_KI||ROLL_KD)) {
			
			double newValue = strtod(pidData, NULL);
			if(startCommand == YAW_KP) {
				yawPID.SetTunings(newValue, YAW_I_VAL, YAW_D_VAL);
			}
			else if(startCommand == YAW_KI) {
				yawPID.SetTunings(YAW_P_VAL,newValue, YAW_D_VAL);
			}
			else if(startCommand == YAW_KD) {
				yawPID.SetTunings(YAW_P_VAL, YAW_I_VAL, newValue);
			}
			else if(startCommand == PITCH_KP) {
				pitchPID.SetTunings(newValue, PITCH_I_VAL, PITCH_D_VAL);
			}
			else if(startCommand == PITCH_KI) {
				pitchPID.SetTunings(PITCH_P_VAL, newValue, PITCH_D_VAL);
			}
			else if(startCommand == PITCH_KD) {
				pitchPID.SetTunings(PITCH_P_VAL, PITCH_I_VAL, newValue);
			}
			else if(startCommand == ROLL_KP) {
				rollPID.SetTunings(newValue, ROLL_I_VAL, ROLL_D_VAL);
			}
			else if(startCommand == ROLL_KI) {
				rollPID.SetTunings(ROLL_P_VAL, newValue, ROLL_D_VAL);
			}
			else if(startCommand == ROLL_KD) {
				rollPID.SetTunings(ROLL_P_VAL, ROLL_I_VAL, newValue);
			}
			
			#ifdef DEBUG
				BLUETOOTH.println("PID Values updated");
			#endif
			command = ' ';
			startCommand = ' ';
			memset(pidData, 0, sizeof(pidData));
			pidIndex = 0;
		}
	}
	
	releaseLock();
}

/**
* Read the values from the Serial Monitor
**/
inline void readSerialControls() {
	acquireLock();
		if(Serial.available() > 0) {
		incomByte = Serial.read();
		
		if(incomByte == CMD_T) {
			command = CMD_T;
			memset(throttleData, 0, sizeof(throttleData));
			throttleIndex = 0;
		}
		else if(incomByte == CMD_Y) {
			command = CMD_Y;	
			memset(yawData, 0, sizeof(yawData));
			yawIndex = 0;
		}
		else if(incomByte == CMD_P) {
			command = CMD_P;
			memset(pitchData, 0, sizeof(pitchData));
			pitchIndex = 0;
		}
		else if(incomByte == CMD_R) {
			command = CMD_R;
			memset(rollData, 0, sizeof(rollData));
			rollIndex = 0;
		}
		else if(incomByte == CMD_E) {
			startCommand = command;
			command = CMD_E;
		} 
		else if(incomByte == CMD_D) {
			command = CMD_D;
		}
		else if(incomByte == YAW_KP) {
			command = YAW_KP;
		}
		else if(incomByte == YAW_KI) {
			command = YAW_KI;
		}
		else if(incomByte == YAW_KD) {
			command = YAW_KD;
		}
		else if(incomByte == PITCH_KP) {
			command = PITCH_KP;
		}
		else if(incomByte == PITCH_KI) {
			command = PITCH_KI;
		}
		else if(incomByte == PITCH_KD) {
			command = PITCH_KD;
		}
		else if(incomByte == ROLL_KP) {
			command = ROLL_KP;
		}
		else if(incomByte == ROLL_KI) {
			command = ROLL_KI;
		}
		else if(incomByte == ROLL_KD) {
			command = ROLL_KD;
		}
		else if(incomByte == SEND_PID) {
			command = SEND_PID;
		}
		else if(incomByte == CHANGE_FLIGHT_MODE) {
			command = CHANGE_FLIGHT_MODE;	
		}
		
		if(command == CMD_T && incomByte != CMD_T) {
			throttleData[throttleIndex] = incomByte;
			throttleIndex++;
		}
		else if(command == CMD_Y && incomByte != CMD_Y) {
			yawData[yawIndex] = incomByte;
			yawIndex++;
		}
		else if(command == CMD_P && incomByte != CMD_P) {
			pitchData[pitchIndex] = incomByte;
			pitchIndex++;
		}
		else if(command == CMD_R && incomByte != CMD_R) {
			rollData[rollIndex] = incomByte;
			rollIndex++;
		}
		else if(command == CMD_D) {
			sendToBluetooth = ~sendToBluetooth;
			command = ' ';
			startCommand = ' ';
		}
		else if(command == CHANGE_FLIGHT_MODE) {
			if(flyMode == ACRO_MODE) {
				flyMode = AUTO_MODE;
			} else {
				flyMode = ACRO_MODE;
			}
			command = ' ';
			startCommand = ' ';
		}
		else if(command == SEND_PID) {
			Serial.print("P,");
			Serial.print(YAW_P_VAL);
			Serial.print(",");
			Serial.print(YAW_I_VAL);
			Serial.print(",");
			Serial.print(YAW_D_VAL);
			Serial.print(",");
			Serial.print(PITCH_P_VAL);
			Serial.print(",");
			Serial.print(PITCH_I_VAL);
			Serial.print(",");
			Serial.print(PITCH_D_VAL);
			Serial.print(",");
			Serial.print(ROLL_P_VAL);
			Serial.print(",");
			Serial.print(ROLL_I_VAL);
			Serial.print(",");
			Serial.print(ROLL_D_VAL);
			Serial.print(",");
			Serial.println("E");
			
			command = ' ';
		}
		else if(command == (YAW_KP||YAW_KI||YAW_KD||PITCH_KP||PITCH_KI||PITCH_KD||ROLL_KP||ROLL_KI||ROLL_KD) &&
			incomByte != (YAW_KP||YAW_KI||YAW_KD||PITCH_KP||PITCH_KI||PITCH_KD||ROLL_KP||ROLL_KI||ROLL_KD)) {
			
			pidData[pidIndex] = incomByte;
			pidIndex++;
		}
		else if(command == CMD_E && startCommand != (YAW_KP||YAW_KI||YAW_KD||PITCH_KP||PITCH_KI||PITCH_KD||ROLL_KP||ROLL_KI||ROLL_KD)) {
			throttle = atoi(throttleData);
			if(throttle > MAX_THROTTLE) {
				throttle = MAX_THROTTLE;
			}
			pitch_input = strtod(pitchData, NULL);
			roll_input = strtod(rollData, NULL);
			yaw_input = strtod(yawData, NULL);
			command = ' ';
			startCommand = ' ';
			memset(throttleData, 0, sizeof(throttleData));
			throttleIndex = 0;
			memset(yawData, 0, sizeof(yawData));
			yawIndex = 0;
			memset(pitchData, 0, sizeof(pitchData));
			pitchIndex = 0;
			memset(rollData, 0, sizeof(rollData));
			rollIndex = 0;
		}
		else if(command == CMD_E && startCommand == (YAW_KP||YAW_KI||YAW_KD||PITCH_KP||PITCH_KI||PITCH_KD||ROLL_KP||ROLL_KI||ROLL_KD)) {
			
			double newValue = strtod(pidData, NULL);
			if(startCommand == YAW_KP) {
				yawPID.SetTunings(newValue, YAW_I_VAL, YAW_D_VAL);
			}
			else if(startCommand == YAW_KI) {
				yawPID.SetTunings(YAW_P_VAL,newValue, YAW_D_VAL);
			}
			else if(startCommand == YAW_KD) {
				yawPID.SetTunings(YAW_P_VAL, YAW_I_VAL, newValue);
			}
			else if(startCommand == PITCH_KP) {
				pitchPID.SetTunings(newValue, PITCH_I_VAL, PITCH_D_VAL);
			}
			else if(startCommand == PITCH_KI) {
				pitchPID.SetTunings(PITCH_P_VAL, newValue, PITCH_D_VAL);
			}
			else if(startCommand == PITCH_KD) {
				pitchPID.SetTunings(PITCH_P_VAL, PITCH_I_VAL, newValue);
			}
			else if(startCommand == ROLL_KP) {
				rollPID.SetTunings(newValue, ROLL_I_VAL, ROLL_D_VAL);
			}
			else if(startCommand == ROLL_KI) {
				rollPID.SetTunings(ROLL_P_VAL, newValue, ROLL_D_VAL);
			}
			else if(startCommand == ROLL_KD) {
				rollPID.SetTunings(ROLL_P_VAL, ROLL_I_VAL, newValue);
			}
			
			#ifdef DEBUG
				Serial.println("PID Values updated");
			#endif
			command = ' ';
			startCommand = ' ';
			memset(pidData, 0, sizeof(pidData));
			pidIndex = 0;
		}
	}
	releaseLock();
}

inline void printValues() {
	#ifdef DEBUG 
		Serial.print(yaw);
		printTab();
		Serial.print(pitch);
		printTab();
		Serial.print(roll);
		printTab();
		Serial.print(filt_gx);
		printTab();
		Serial.print(filt_gy);
		printTab();
		Serial.print(filt_gz);
		printTab();
		Serial.print((yaw_offset));
		printTab();
		Serial.print((pitch_offset));
		printTab();
		Serial.print((roll_offset));
		printTab();
		Serial.print(pitch_rate_offset);
		printTab();
		Serial.print(roll_rate_offset);
		printTab();
		Serial.print((throttle));
		printTab();
		Serial.print(yaw_input);
		printTab();
		Serial.print(pitch_input);
		printTab();
		Serial.print(roll_input);
		printTab();
		Serial.print((m_one));
		printTab();
		Serial.print((m_two));
		printTab();
		Serial.print((m_three));
		printTab();
		Serial.print((m_four));
		Serial.println();
	#endif
}

inline void printBluetoothValues() {
	BLUETOOTH.print(yaw);
	printBluetoothTab();
	BLUETOOTH.print(pitch);
	printBluetoothTab();
	BLUETOOTH.print(roll);
	printBluetoothTab();
	BLUETOOTH.print((yaw_offset));
	printBluetoothTab();
	BLUETOOTH.print((pitch_offset));
	printBluetoothTab();
	BLUETOOTH.print((roll_offset));
	printBluetoothTab();
	BLUETOOTH.print((throttle));
	printBluetoothTab();
	BLUETOOTH.print((m_one));
	printBluetoothTab();
	BLUETOOTH.print((m_two));
	printBluetoothTab();
	BLUETOOTH.print((m_three));
	printBluetoothTab();
	BLUETOOTH.print((m_four));
	BLUETOOTH.println();	
	
}

inline void printControlValues() {
	Serial.print(throttle);
	printTab();
	Serial.print(yaw_input);
	printTab();
	Serial.print(pitch_input);
	printTab();
	Serial.print(roll_input);
	printTab();
	Serial.print(command);
	printTab();
	Serial.print(startCommand);
	printTab();
	Serial.print(incomByte);
	printTab();
	Serial.println();
}

/**
* Prints the RC values. 
**/
inline void printRCValues() {
	Serial.print(ch1);
	printTab();
	Serial.print(ch2);
	printTab();
	Serial.print(ch3);
	printTab();
	Serial.print(ch4);
	printTab();
	Serial.print(ch5);
	printTab();
	Serial.print(ch6);
	printTab();
	Serial.print(throttle);
	printTab();
	Serial.print(yaw_input);
	printTab();
	Serial.print(pitch_input);
	printTab();
	Serial.print(roll_input);
	Serial.println();	
}

inline void printTab() {
	Serial.print(F("\t"));
}

inline void printBluetoothTab() {
	BLUETOOTH.print(F("\t"));
}
