#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

#pragma GCC diagnostic ignored "-Wwrite-strings"

using namespace std;

#define PIDdebug 1
#define Active 1
#define Innactive 0

mutex threadmutex;

//variables for PID calculation (result and process)
struct PID{
	char* name;
	float error;
	float lasterror;
	float sumerror;
	float refference;
	float current;
	float AW;
	float output;
	
	//reset all value (set to surface value)
	void reset()
	{
		error		= 0;
	 	lasterror	= 0;
	 	sumerror	= 0;
	 	refference 	= 0;
	 	current		= 0;
	 	AW			= 0;
	 	output		= 0;
 		if (PIDdebug)
		 	cout << "reset all values on : "<<name<<endl;
	
	}
	
	//sementara nilainya rentang -0.5 sampai 0.5
	//rubah ke dalam bentuk -100% sampai 100%
	//0.5	== 100%
	//-0.25	== -25%
	
};
PID heading{"heading"};
PID pitch{"pitch"};
PID buoyancy{"buoyancy"};

//variables for PID constants
struct PIDconst{
	char* name;
	float KP;
	float KI;
	float KD;
	float AW;
	float SamplingTime;
	float saturation_upper;
	float saturation_lower;
	
	//declare constants values
	void setup(float _KP, float _KI, float _KD, float _AW, float _samplingtime, float _satUpper, float _satLower)
		{
	 		KP					= _KP;
	 		KI					= _KI;
	 		KD					= _KD;
	 		AW					= _AW;
		 	SamplingTime		= _samplingtime;
			saturation_upper	= _satUpper;
			saturation_lower	= _satLower;
			if (PIDdebug)
				std::cout <<"PID constants for " << name << " are set"<<std::endl;
		}
};
PIDconst rudder{"rudder"};
PIDconst ballast{"ballast"};
PIDconst bladder{"bladder"};

//variables for glider operational constants
struct GliderConstants
{
	//{innactive val, active val}

	int propeller[2] 		= {0,1600};	
	int bow[2]				= {0,1800};
	int strobe[2]			= {0,1};
	float surfaceDepth		= 0;
	float surfaceThreshold	= 0;
	float depthThreshold	= -17;
	float surfacePitch		= 0;
	float descentPitch		= -0.26;
	float ascentPitch		= 0.26;
	int ascending			= 1;
	int descending			= 0;
	int staystill			= 2;

};
GliderConstants gliderConst;

//glider operational variables
struct Glider
{
	bool descentFlag, ascentFlag;
	bool pitchUpFlag, pitchDownFlag;
	bool sensorIsUpdated;

	int pose;
	int guidanceIsFinished;
	int strobe;
	int datalogCount;

	float pos_e;
	float pos_n;
	float currentDepth;
	float currentPitch;
	float depthTarget;
	float teta_terukur;
	float glideAngle;
	
	void reset()
    {
    descentFlag = false;
    ascentFlag = false;
	pitchUpFlag = false;
    pitchDownFlag = false;
    sensorIsUpdated = false;

    pose = 0;
	guidanceIsFinished = 0;
	strobe = 0;
    datalogCount = 0;
	currentDepth = 0;
	currentPitch = 0;
	depthTarget = 0;
	pos_e = 0;
	pos_n = 0;
	teta_terukur = 0;
	glideAngle = 0;
    }
};
Glider glider;


float PID_heading_rudder(float _yawRefference, float _globalHeading) {
	heading.refference = (_yawRefference*3.1416)/180.0;
	heading.current = (_globalHeading*3.1416)/180.0;

	heading.lasterror = heading.error;
	//need further information
	heading.error = heading.error - heading.current;
	heading.sumerror = heading.sumerror + heading.error + heading.AW;
	
	float PID_rd_pro = heading.error * rudder.KP;
	float PID_rd_int = heading.sumerror * rudder.KI * rudder.SamplingTime;
	float PID_rd_dev = (heading.error - heading.lasterror) * rudder.KD / rudder.SamplingTime;

	float PID_rd_cal = PID_rd_pro + PID_rd_int + PID_rd_dev;
	float PID_rd_out;
	
	if(PID_rd_cal > rudder.saturation_upper) {
		PID_rd_out = rudder.saturation_upper;
	}
	else if(PID_rd_cal < rudder.saturation_lower) {
		PID_rd_out = rudder.saturation_lower;
	}
	else {
		PID_rd_out = PID_rd_cal;
	}

	heading.AW = (PID_rd_out - PID_rd_cal) * rudder.AW;
	
	return PID_rd_out;
}

float PID_pitch_ballast(float _pitchRefference, float _globalPitch) {
	pitch.refference = _pitchRefference;
	pitch.current = _globalPitch;
	
	pitch.lasterror = pitch.error;
	pitch.error = pitch.refference - pitch.current;
	pitch.sumerror = pitch.sumerror + pitch.error + pitch.AW;
	
	float PID_blst_pro = pitch.error * ballast.KP;
	float PID_blst_int = pitch.sumerror * ballast.KI * ballast.SamplingTime;
	float PID_blst_dev = (pitch.error - pitch.lasterror) * ballast.KD / ballast.SamplingTime;

	float PID_blst_cal = PID_blst_pro + PID_blst_int + PID_blst_dev;
	float PID_blst_out;
	
	if(PID_blst_cal > ballast.saturation_upper) {
		PID_blst_out = ballast.saturation_upper;
	}
	else if(PID_blst_cal < ballast.saturation_lower) {
		PID_blst_out = ballast.saturation_lower;
	}
	else {
		PID_blst_out = PID_blst_cal;
	}
	
	//	pitch.AW = (PID_blst_out - PID_blst_cal) * ballast.AW;

	return PID_blst_out;
}

float PID_buoyancy_bladder(float _depthRefference, float _globalDepth) {
	buoyancy.refference = _depthRefference;
	buoyancy.current = _globalDepth;
	
	buoyancy.lasterror = buoyancy.error;
	buoyancy.error = buoyancy.refference - buoyancy.current;
	buoyancy.sumerror = buoyancy.sumerror - buoyancy.error + buoyancy.AW;
	
	float PID_bld_pro = buoyancy.error * bladder.KP;
	float PID_bld_int = buoyancy.sumerror * bladder.KI * bladder.SamplingTime;
	float PID_bld_dev = (buoyancy.error - buoyancy.lasterror) * bladder.KD / bladder.SamplingTime;

	float PID_bld_cal = PID_bld_pro + PID_bld_int + PID_bld_dev;
	float PID_bld_out;
	
	if(PID_bld_cal < bladder.saturation_upper) {
		PID_bld_out = bladder.saturation_upper;
	}
	else if(PID_bld_cal > bladder.saturation_lower) {
		PID_bld_out = bladder.saturation_lower;
	}
	else {
		PID_bld_out = PID_bld_cal;
	}
	
	//depth.AW = (PID_bld_out - PID_bld_cal) * ballast.AW;
	
	//need further information
	/*
	if(PID_bld_out != 0) {
		cout << "PID bld out thread BB : " << PID_bld_out << endl;
		buoyancy.lasterror = buoyancy.error;
	}
	else if(PID_bld_out == 0) {
		std::cout << "PID bld thread BB : 0" << std::endl;
	}
	*/
	return PID_bld_out;
}

void fakeSendtoArduino(float p1, float p2, float p3, float p4, float p5, float p6, int p7)
{
	cout<<"                                           .:.fakesend:.: "<<p1<<" "<<p2<<" "<<p3<<endl;	
}

void PIDOperation(int glidestate)
{
	//data for arduino :
		//float bladder
		//float ballast
		//float teta
		//float rudder
		//float pose
		//float posn
		//int strobe
	
	//bladder <- PID
	//ballast <- PID
	//rudder <- PID
	//teta <- surfacepitch/ascentpitch/descentpitch
	//pos_e <- data BB
	//pos_n <- data BB
	
	//yawref	<-  nav/guidance (?)
	//global heading <- nav/guidance (?)
	
	//pitchref <- teta / user
	//global pitch <- sensor BB
	
	//depthref <- GCS / user
	//globaldepth <- sensor BB
	
	//.:.ondevelopment
	float yawRefference, globalHeading;
	float globalPitch;
	float depthRefference; //ambil dari user (descend target) atau surface depth (ascend target)
	float globalDepth;
	
	
	float pitchRefference;
	float teta_error;
	//:.:ondevelopment
	
	//ascending
	if (glidestate == 1 )
	{
		//propeller off

		//teta <- ascentpitch
		pitchRefference = gliderConst.ascentPitch;

		//strobe on
		glider.strobe = gliderConst.strobe[Active];
		
		heading.output = PID_heading_rudder(yawRefference,glider.teta_terukur);

		pitch.output = PID_pitch_ballast(gliderConst.ascentPitch,glider.currentPitch);

		buoyancy.output = PID_buoyancy_bladder(gliderConst.surfaceDepth,glider.currentDepth);
		
		//pos_e ??
		//pos_n ??
	}
	else 
	//descending
	if (glidestate == 0)
	{
		//propeller off

		//teta <- descentpitch
		pitchRefference = gliderConst.descentPitch;
		
		//strobe off
		glider.strobe = gliderConst.strobe[Innactive];
		
		heading.output = PID_heading_rudder(yawRefference,glider.teta_terukur);

		pitch.output = PID_pitch_ballast(gliderConst.descentPitch,glider.currentPitch);

		buoyancy.output = PID_buoyancy_bladder(glider.depthTarget,glider.currentDepth);

		//pos_e ??
		//pos_n ??

	} else
	//stay still
	if (glidestate == 2)
	{
		//propeller off
		//teta <- surfacepitch
		pitchRefference = gliderConst.surfacePitch;
		
		//reset actuator
		heading.reset();
		
		pitch.reset();
		
		buoyancy.reset();
		
		//pos_e ??
		//pos_n ??
		
		//strobe on		
		glider.strobe = gliderConst.strobe[Active];
		this_thread::sleep_for(chrono::milliseconds(2000));
	}
	teta_error = pitchRefference - glider.teta_terukur;
	fakeSendtoArduino(buoyancy.output, pitch.output, teta_error, heading.output, glider.pos_e, glider.pos_n,glider.strobe);
}

bool sensorIsUpdated;
//return 1 if guidance is finished and glider is on surface, else return 0
//run only when sensor is updated

void overrideSensor()
{
	glider.guidanceIsFinished = 1;
}

int glideOperation(int override = 0)
{
	if (glider.sensorIsUpdated)
	{
		threadmutex.lock();
		if (override) overrideSensor();
		
		glider.sensorIsUpdated = false;
		//ascending
		if (glider.pose == gliderConst.ascending)
		{
			cout << "POSE ASCENDING " <<" depth : "<<glider.currentDepth <<endl;
			if (glider.currentDepth>gliderConst.surfaceDepth || glider.currentDepth>gliderConst.surfaceThreshold)
			{
				//check guidance
				if (glider.guidanceIsFinished)
				{
					cout <<"STAYSTILL"<<endl;
					glider.pose = gliderConst.staystill;
					PIDOperation(gliderConst.staystill);
					//stop glide operation
					return 1;
				}
				else
				{
					cout <<"STAYSTILL"<<endl;
					//change to descending
					glider.pose = gliderConst.descending;
					PIDOperation(gliderConst.staystill);
				}
			} else
			{
				//continue ascending
				PIDOperation(gliderConst.ascending);							
			}
		}
		else
		//descending
		if (glider.pose == gliderConst.descending)
		{
			cout << "POSE DESCENDING " <<" depth : "<<glider.currentDepth <<endl;
			if (glider.currentDepth>glider.depthTarget && glider.currentDepth>gliderConst.depthThreshold)
			{
				//continue descending
				PIDOperation(gliderConst.descending);			
			} 
			else
			{
				cout <<"STAYSTILL"<<endl;
				//change to ascending
				glider.pose = gliderConst.ascending;
				PIDOperation(gliderConst.staystill);			
			}
		}
		threadmutex.unlock();
	}
	{
		//wait sensor update
	}
	return 0;
}

void initializePID()
{
	//.:.Initialize PID
	rudder.setup(70,1,0.1,1,1,0.5,-0.5);
	heading.reset();
	
	ballast.setup(-0.5,-0.05,-0.03,0,1,0.5,-0.5);
	pitch.reset();
	
	bladder.setup(20,0,0.5,1,1,-0.5,0.5);
	buoyancy.reset();
	//:.:Initialize PID
}

void fakesensorthread()
{
	while(1){
	
		this_thread::sleep_for(chrono::seconds(1));
	
		threadmutex.lock();
		glider.guidanceIsFinished = 0;
		glider.sensorIsUpdated = true;
		threadmutex.unlock();
	}
}

void fakedepththread()
{
	while(1)
	{
		this_thread::sleep_for(chrono::milliseconds(300));

		threadmutex.lock();
		float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		r/=3;
		
		if (glider.pose == gliderConst.ascending)
		{
			glider.currentDepth+=r;
		}
		else
		if (glider.pose == gliderConst.descending)
			glider.currentDepth-=r;
	
		threadmutex.unlock();
	}
}
int state = 900;

int main()
{
	initializePID();
	glider.reset();
	//start thread
	thread fakesensor(fakesensorthread);
	thread fakeglider(fakedepththread);
	fakesensor.detach();
	fakeglider.detach();
	
	while(1)
	{
		switch (state)
			{
				case 900 : 
							//wait for initial sensor update
							glider.sensorIsUpdated = false;
							//set descend flag
							glider.pose = gliderConst.descending;
							//set depth
							glider.depthTarget = -5;
							cout <<"START OPERATION SINGLE GLIDE (No Waypoint). Target : "<<glider.depthTarget<<endl<<endl;
							this_thread::sleep_for(chrono::milliseconds(2000));
							state = 901;
							break;
				case 901 :
							//keep until glide finish
							if (glideOperation(1))
								state = 902;
							break;
				case 902 :
							cout << "MISSION FINISHED"<<endl;
							return 1;
							break;
			}
	}
	return 0;
}