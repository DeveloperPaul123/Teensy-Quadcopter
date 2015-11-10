#include "Arduino.h"
#include "FrequencyEvent.h"

/**
* Basic constructor. 
**/
FrequencyEvent::FrequencyEvent() {
	lastTime = 0;
	timeInterval = 0;
}

/**
* Determines if the specified event should be run. 
* @return bool, true if the event should be run. False otherwise. 
**/
bool FrequencyEvent::shouldRunEvent() {
	long now = micros();
	if(now - lastTime >= timeInterval) {
		lastTime = now;
		return true;
	} else {
		return false;
	}
}

/**
* Must be called in setup or at least once before use. 
* @param frequency, the frequency of the event in Hz. 
**/
void FrequencyEvent::setFrequency(long frequency) {
	if(frequency < 0) return;
	
	//time interval should be in microseconds. 
	timeInterval = (long) 1000000/frequency;
}

/**
* Starts the event counter. Don't need to call unless you have a 
* delay before you start your fixed time interval event. 
**/
void FrequencyEvent::start() {
	lastTime = micros();
}