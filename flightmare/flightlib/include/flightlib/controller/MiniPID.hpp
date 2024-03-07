#include <iostream>
#include <cmath>

#ifndef MINIPID_H
#define MINIPID_H

class MiniPID{
public:
	MiniPID(double, double, double);
	MiniPID(double, double, double, double);
	void setP(double);
	void setI(double);
	void setD(double);
	void setF(double);
	void setDt(double);
	void setPID(double, double, double);
	void setPID(double, double, double, double);
	void setMaxIOutput(double);
	void setOutputLimits(double);
	void setOutputLimits(double,double);
	void setDirection(bool);
	void setSetpoint(double);
	void storeReservoir();
	void restoreReservoir();
	void reset();
	void setOutputRampRate(double);
	void setSetpointRange(double);
	void setOutputFilter(double);
	double getP();
	double getI();
	double getD();
	double getOutput();
	double getOutput(double);
	double getOutput(double, double);
	double getOutput(double, double, double);
	double getOutput(double, double, bool, double);

private:
	double clamp(double, double, double);
	bool bounded(double, double, double);
	void checkSigns();
	void init();
	// double lowPass(double input);
	double P;
	double I;
	double D;
	double F;

	double dt;

	double maxIOutput;
	double maxError;
	double maxIStep;

	double maxOutput; 
	double minOutput;

	double setpoint;

	double lastActual;

	bool firstRun;
	bool reversed;

	double outputRampRate;
	double lastOutput;
	double lastlastOutput; //used for autotune

	double outputFilter;

	double setpointRange;

	double errorSum;
	double errorSumSaved;

	//lowpass filter
	double cutoffFreq;
    double samplingFreq;
    double prevOutputLP;
};
#endif
