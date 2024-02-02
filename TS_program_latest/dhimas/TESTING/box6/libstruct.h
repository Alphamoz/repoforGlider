#ifndef _gliderstruct
#define _gliderstruct

#define PIDdebug 1

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
	}
	
	//sementara nilainya rentang -0.5 sampai 0.5
	//rubah ke dalam bentuk -100% sampai 100%
	//0.5	== 100%
	//-0.25	== -25%
};

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
		}
};

#endif