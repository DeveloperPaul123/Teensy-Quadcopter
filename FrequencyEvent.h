#ifndef FrequencyEvent_h
#define FrequencyEvent_h

#include "Arduino.h"

/**
* This is a simple wrapper for a timed event. Set the frequency in Hz and it'll take care of the rest. 
* Example:
 #include "FrequencyEvent.h"

 FrequencyEvent my1HzEvent;
 bool hasStarted = false;
 void setup() {
	my1HzEvent.setFrequency(1);
 }
void loop() {
	if(!hasStarted) {
		my1HzEvent.start();
	} else {
		if(my1HzEvent.shouldRunEvent()) {
			//run my 1 hz event.
		}
	}
}
**/
class FrequencyEvent {
	public:
		FrequencyEvent();
		bool shouldRunEvent();
		void setFrequency(long freq);
		void start();
	private:
		long lastTime;
		long timeInterval;
};

#endif