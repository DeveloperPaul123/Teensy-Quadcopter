/*
Functions and routines and setup for use with a RC transmitter. 
*/

/**
* RC reciever pins
**/
#define RC_1 12
#define RC_2 11 
#define RC_3 10
#define RC_4 9
#define RC_5 8
#define RC_6 7

/**
* Initializes the RC stuff. 
**/
inline void initRC() {
	//all pins need to be inputs. 
	pinMode(RC_1, INPUT);
	pinMode(RC_2, INPUT);
	pinMode(RC_3, INPUT);
	pinMode(RC_4, INPUT);
	pinMode(RC_5, INPUT);
	pinMode(RC_6, INPUT);
	
	//six interrupts!!
	attachInterrupt(RC_1, rcInterrupt1, CHANGE);
	attachInterrupt(RC_2, rcInterrupt2, CHANGE);
	attachInterrupt(RC_3, rcInterrupt3, CHANGE);
	attachInterrupt(RC_4, rcInterrupt4, CHANGE);
	attachInterrupt(RC_5, rcInterrupt5, CHANGE);
	attachInterrupt(RC_6, rcInterrupt6, CHANGE);
}

inline void rcInterrupt1(){
   if(!interruptLock) ch1 = micros() - rcLastChange1;
   rcLastChange1 = micros(); 
}

inline void rcInterrupt2(){
  if(!interruptLock) ch2 = micros() - rcLastChange2;
  rcLastChange2 = micros();
}

inline void rcInterrupt3(){
  if(!interruptLock) ch3 = micros() - rcLastChange3;
  rcLastChange3 = micros();
}

inline void rcInterrupt4(){
  if(!interruptLock) ch4 = micros() - rcLastChange4;
  rcLastChange4 = micros();
}

inline void rcInterrupt5(){
  if(!interruptLock) ch5 = micros() - rcLastChange5;
  rcLastChange5 = micros();
}

inline void rcInterrupt6() {
	if(!interruptLock) ch6 = micros() - rcLastChange6;
	rcLastChange6 = micros();
}

inline void acquireLock(){
  interruptLock = true; 
}

inline void releaseLock(){
  interruptLock = false;
}
