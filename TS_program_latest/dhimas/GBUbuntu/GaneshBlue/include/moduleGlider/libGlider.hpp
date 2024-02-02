#ifndef _libGlider
#define _libGlider

#include <iostream>     // cout
#include <sstream>      // istringstream
#include <string>       // string
#include <fstream>      // file operation
#include <iomanip>
#include <cmath>
#include <limits>

struct GliderConstants
{
	//{innactive val, active val}
	int propeller[2];
	int bow[2];
	int strobe[2];
	float surfaceDepth;
	float surfaceThreshold;
	float depthThreshold;
	float surfacePitch;
	float descendPitch;
	float ascendPitch;
	int ascending;
	int descending;
	int staystill;
	int MMvarHigh;
	int MMvarLow;
	int BEvarMid;
	int offset;
	int rudderMid;

	//load constants values
	int loadConfig();
	
    void Print();
    void Setup(
        int _propellerLow,
        int _propellerHigh,
        int _bowLow,
        int _bowHigh,
        int _strobeOff,
        int _strobeOn,
        int _ascending,
        int _descending,
        int _staystill,
        float _surfaceDepth,
        float _surfaceThreshold,
        float _depthThreshold,
        float _surfacePitch,
        float _descendPitch,
        float _ascendPitch
    );
};

//glider operational variables
struct Glider
{
	bool descendFlag, ascendFlag;
	bool pitchUpFlag, pitchDownFlag;

	//sensor
	bool sensorIsUpdated;
	bool isTransmitingData;

	int pose;
	//navigation
	int guidanceIsFinished;
	int strobe;

	bool datalogReady;
	bool datalogIsCreated;
	int datalogCount;
	std::string datalogPath;

	//navigation
	float pos_e;
	//navigation
	float pos_n;
	//sensor
	float currentDepth;
	//sensor
	float currentPitch;
	float depthTarget;
	//sensor
	float teta_terukur;
	//sensor
	float glideAngle;
	float speed;
	
	//build in TS
	float boardTemp;
	// HullTemp and Humidity dht22
	float hullTemp;
	float humidity;
    void Reset();
    void Print();
};

#endif
