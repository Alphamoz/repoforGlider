#ifndef _main
#define _main

#include <Utils.hpp>

void PIDOperation(int glidestate);
int glideOperation(int override);

void GCS_task();
void Sensor_task();
void Actuator_task();
void sendDatalog();
void debug_override_task();

struct FSM
{
	int targetCount;
	int state;
	int lastState;
	void moveTo(int _target);
	void baseTo(int _target);
	void Print();
	int lowerState();
	int upperState();
	bool isWaitingSensorUpdate;
	bool logIsCreated;
};

enum _fsm
{
	WAITING = 0,
	SETUP = 1,
	//  STOP=  8,
	SINGLETARGETSETUP = 10,
	LOWBATTERYFAILSAFE = 11,
	GLIDING = 12,
	GLIDECOMPLETE = 13,
	MANUALGLIDING = 17,
	WAITINSTRUCTION = 19,
	FAILSAFE = 20, // flag only
	CONTROL = 21,
	GUIDANCE = 22,
	SURFACING = 23,
	TIMETHRESHOLDGLIDE = 24,
	ARDUDATACHECK = 25,
	SETUPRTB = 91,
	RTBOPERATION = 92,
	MISSIONABORTED = 93,
	HALT = 94,
	SINGLETARGETFINISH = 99,
	BASE = 100, // flag only
	// usage example :
	// implementation = targetnumber *  TARGET
	// single target = (1*100) = 100
	// singletargetsetup = 110
	// singletargetsetupRTB = 191

	// 3 target = (3*100)  = 300
	// 1st waypoint setup = 310
	// 3rd waypoint setup = 110
	// 2nd gliding = 212

};
#endif
