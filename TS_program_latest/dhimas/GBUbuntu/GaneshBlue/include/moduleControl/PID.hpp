#ifndef _libPID
#define _libPID

#include <iostream>     // cout
#include <sstream>      // istringstream
#include <string>       // string
#include <fstream>      // file operation
#include <iomanip>
#include <cmath>
#include <limits>
#include <algorithm>

//variables for PID calculation (result and process)
struct PID{
	std::string name;
	float error;
	float lasterror;
	float sumerror;
	float refference;
	float current;
	float AW;
	float output;
		
	float outputPercentage;
	//sementara nilainya rentang -0.5 sampai 0.5
	//rubah ke dalam bentuk -100% sampai 100%
	//0.5	== 100%
	//-0.25	== -25%

   	//reset all value (set to surface value)
	void Reset();
    void Print();
	
};

//variables for PID constants
struct PIDconst{
	std::string name;
	float KP;
	float KI;
	float KD;
	float AW;
	float SamplingTime;
	float saturation_upper;
	float saturation_lower;
	
	//load constants values
	int loadConfig();
	void Setup(float _KP, float _KI, float _KD, float _AW, float _samplingtime, float _satUpper, float _satLower);
    void Print();
};

PID CalculatePID(PID _temp, PIDconst constants, float setPoint, float input, bool AW);
/*
param 1 : struct PID that contains current and last values
param 2 : struct PIDconst that contain constants for PID calculation
param 3 : setpoint for PID calculation
param 4 : input for setpoint calculation
param 5 : true : use antiwindup; false : no antiwindup. default NO
output : struct PID with new values (updated after calculation)
*/

#endif