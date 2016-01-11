#ifndef BUTTERWORTH_H
#define BUTTERWORTH_H

#define GAIN 2.0

class Butterworth {
	
public:
	Butterworth(int samplingFreq, int cutoffFreq);
	double run(double newVal);
	
private:
	double x[2], y[2];
	double ita;
	double q;
	double b0, b1, b2;
	double a1, a2;
}
#endif