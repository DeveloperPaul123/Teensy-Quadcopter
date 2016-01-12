#include "Butterworth.h"
#include "math.h"

/**
* Butterworth Filter, second order.
* @param samplingFreq the sampling frequency.
* @param cutoffFreq the cut off frequency.
**/
Butterworth::Butterworth(int samplingFreq, int cutoffFreq) {
	double ff = cutoffFreq/samplingFreq;
	const double ita =1.0/ tan(M_PI*ff);
    const double q=sqrt(2.0);
    b0 = 1.0 / (1.0 + q*ita + ita*ita);
    b1= 2*b0;
    b2= b0;
    a1 = 2.0 * (ita*ita - 1.0) * b0;
    a2 = -(1.0 - q*ita + ita*ita) * b0;
	for(int i = 0; i < 2; i++) {
		x[i] = 0.0;
		y[i] = 0.0;
	}
}

/**
* Runs the butterworth filter.
* @param newValue the new data value.
* @return double the filtered value. 
*/
double Butterworth::run(double newValue) {
	for(;;) {
		x[0] = x[1]; x[1] = x[2];
		x[2] = newValue / GAIN;
		y[0] = y[1]; y[1] = y[2];
		y[2] = ((b0 * x[0]) + (b2 *x[2])) + (b1 * x[1]) 
				+ (a1 * y[0]) + (a2 * y[1]);
		return y[2];
	}
}