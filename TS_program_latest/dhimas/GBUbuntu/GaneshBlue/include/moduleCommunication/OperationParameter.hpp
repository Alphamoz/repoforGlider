#ifndef _param
#define _param

#include <string>

/*
 data example : 
 #GB> PARAM 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20

 parsed : 
         2.opMode         : 2
         3.glidemode      : 3
         4.depthOperation : 4
         5.glidingAngle   : 5
         6.sensorInterval : 6
         7.alti           : 7
         8.bar            : 8
         9.bsl            : 9
        10.miniCT         : 10
        11.oseRate        : 11
        12.propeller      : 12
        13.bow            : 13
        14.readStat       : 14
        15.rtoStat        : 15
        16.rtbStat        : 16
        17.latRTB         : 17
        18.lonRTB         : 18
		19.reserved1	  : 19
		20.reserved2	  : 20
*/

//parameter index
extern const char _opMode;
extern const char _alti;
extern const char _bar;           
extern const char _bsl;          			
extern const char _miniCT;        			
extern const char _glidemode; 			
extern const char _rtbStat; 			
extern const char _rtoStat;			
extern const char _depthOperation;			
extern const char _glidingAngle;			
extern const char _oseRate;			
extern const char _propeller;			
extern const char _bow;			
extern const char _readStat;			
extern const char _interv;			
extern const char _latRTB;			
extern const char _lonRTB;		

struct gliderParam {
	//operation mode
	int opMode;
	//glide mode (glide / kss)
	int glideMode;
	//glide depth target
	int depthOperation;
	//glide angle
	int glidingAngle;
	//BB interval
	int interv;
	//??
	int alti;
	//??
	int bar;
	//??
	int bsl;
	//??
	int miniCT;
	//??
	int oseRate;
	//propeller value (for actuator test)
	int propeller;
	//bow value (for actuator test)
	int bow;
	//flags for RTO
	int rtoStat;
	//flags for RTB
	int rtbStat;
	//??
	int readStat;
	//RTB setpoint
	float latRTB;
	float lonRTB;
	
	//print all member
	void Print();

	//sanity check
	//check word count on string,
	//compare with word (param)
	//return true if word on str == word value
	bool isValid(std::string str,char word);

	//parse GCS parameter
	void Set(std::string str);
};

#endif