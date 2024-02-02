#ifndef _param
#define _param

#include <iostream>
#include <string>
#include <sstream>

//parameter index
const char _alti = 3;
const char _bar = 4;
const char _bsl = 5;			
const char _miniCT = 6;			
const char _mode = 8;			
const char _rtbStat = 9;			
const char _rtoStat = 10;			
const char _depthOperation = 11;			
const char _glidingAngle = 12;			
const char _oseRate = 13;			
const char _propeller = 14;			
const char _bow = 15;			
const char _readStat = 16;			
const char _interv = 17;			
const char _latRTB = 18;			
const char _lonRTB = 19;			
	
	
	

// #GB> PARAM 1 1231  54 2 45 3 56 6
struct gliderParam {
	/*
2	 ("#GB> PARAM "+ ParamQuery[0].id + " " 
3				   + ParamQuery[0].alti + " " 
4				   + ParamQuery[0].bar + " " 
5				   + ParamQuery[0].bsl + " " 
6				   + ParamQuery[0].minict + " " 
7				   + ParamQuery[0].kss + " " 
8				   + ParamQuery[0].mode + " " 
9				   + ParamQuery[0].rtb + " " 
10				   + ParamQuery[0].rto + " " 
11				   + ParamQuery[0].depth + " " 
12				   + ParamQuery[0].gad + " " 
13				   + ParamQuery[0].oserate + " " 
14				   + ParamQuery[0].prop + " " 
15				   + ParamQuery[0].bow + " " 
16				   + ParamQuery[0].readstat + " " 
17				   + ParamQuery[0].interv + " " 
18				   + ParamQuery[0].lat_rtb + " " 
19				   + ParamQuery[0].long_rtb + "\n");
	*/
	


	int alti;
	int bar;
	int bsl;
	int miniCT;
	int mode;
	int rtbStat;
	int rtoStat;
	int depthOperation;
	int glidingAngle;
	int oseRate;
	int propeller;
	int bow;
	int readStat;
	int interv;
	float latRTB;
	float lonRTB;
	
	//print all member
	void print(){
		std::cout<<".:.GLIDER PARAMETER.:."<<std::endl;
		std::cout<<"alti           : "<<alti<<std::endl;
		std::cout<<"bar            : "<<bar<<std::endl;
		std::cout<<"bsl            : "<<bsl<<std::endl;
		std::cout<<"miniCT         : "<<miniCT<<std::endl;
		std::cout<<"mode           : "<<mode<<std::endl;
		std::cout<<"rtbStat        : "<<rtbStat<<std::endl;
		std::cout<<"rtoStat        : "<<rtoStat<<std::endl;
		std::cout<<"depthOperation : "<<depthOperation<<std::endl;
		std::cout<<"glidingAngle   : "<<glidingAngle<<std::endl;
		std::cout<<"oseRate        : "<<oseRate<<std::endl;
		std::cout<<"propeller      : "<<propeller<<std::endl;
		std::cout<<"bow            : "<<bow<<std::endl;
		std::cout<<"readStat       : "<<readStat<<std::endl;
		std::cout<<"latRTB         : "<<latRTB<<std::endl;
		std::cout<<"lonRTB         : "<<lonRTB<<std::endl;
		std::cout<<":.:GLIDER PARAMETER:.:"<<std::endl;
	}
	
	//check for RTB / RTO / test bow / test prop flag
	int subProcessExist()
	{
		if (rtbStat) return 9; else
		if (rtoStat) return 10; else
		if (propeller) return 14; else
		if (bow) return 15; 
		return 0;
	}
	
	//parse GCS parameter
	void set(std::string str){
		std::istringstream _sentence (str);
		char _count = 0;
		do
		{
			std::string _word;
			_sentence >> _word;
			switch (_count)
			{
				case _alti : std::istringstream (_word) >> 
										 alti; 
										 break;
				case _bar : std::istringstream (_word) >> 
										 bar; 
										 break;
				case _bsl : std::istringstream (_word) >> 
										 bsl; 
										 break;
				case _miniCT : std::istringstream (_word) >> 
										 miniCT; 
										 break;
				case _mode : std::istringstream (_word) >> 
										 mode; 
										 break;
				case _rtbStat : std::istringstream (_word) >> 
										 rtbStat; 
										 break;
				case _rtoStat : std::istringstream (_word) >> 
										 rtoStat; 
										 break;
				case _depthOperation : std::istringstream (_word) >> 
										 depthOperation; 
										 break;
				case _glidingAngle : std::istringstream (_word) >> 
										 glidingAngle; 
										 break;
				case _oseRate : std::istringstream (_word) >> 
										 oseRate; 
										 break;
				case _propeller : std::istringstream (_word) >> 
										 propeller; 
										 break;
				case _bow : std::istringstream (_word) >> 
										 bow; 
										 break;
				case _readStat : std::istringstream (_word) >> 
										 readStat; 
										 break;
				case _interv : std::istringstream (_word) >> 
										 interv; 
										 break;
				case _latRTB : std::istringstream (_word) >> 
										 latRTB; 
										 break;
				case _lonRTB : std::istringstream (_word) >> 
										 lonRTB; 
										 break;
			}
			_count++;
		} while (_sentence);
	}

};


#endif