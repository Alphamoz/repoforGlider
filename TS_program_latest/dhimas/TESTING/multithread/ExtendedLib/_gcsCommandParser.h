#ifndef _gcsCommandParser
#define _gcsCommandParser

#include <iostream>
#include <string>
#include <sstream>

#define AT 99
#define RTB 1
#define START 2
#define PARAM 3
#define WAYPOINT 4
#define PING 5
#define DATA 6
#define STOP 7

const	std::string GCS_AT = "#GB> AT";
const	std::string GCS_RTB = "#GB> RTB";
const	std::string GCS_START = "#GB> START";
const	std::string GCS_PARAM = "#GB> PARAM";
const	std::string GCS_WAYPOINT = "#GB> WAYPOINT";
const	std::string GCS_PING = "#GB> PING";
const	std::string GCS_DATA = "#GB> DATA";
const	std::string GCS_STOP = "#GB> STOP";
	
int parsing_GCS(std::string str,bool _debug = false){
	int _mode=-1;
		
	if (str.find(GCS_AT) != std::string::npos)
	{
		if (_debug) 
		{
			std::cout << "debug : ack AT"<<std::endl;			
		}
		_mode = AT;

	} else
	if (str.find(GCS_RTB) != std::string::npos)
	{
		if (_debug) 
		{
			std::cout << "debug : ack RTB"<<std::endl;			
		}
		_mode = RTB;
		
	} else
	if (str.find(GCS_START) != std::string::npos)
	{
		if (_debug) 
		{
			std::cout << "debug : ack START"<<std::endl;			
		}
		_mode = START;
		
	} else
	if (str.find(GCS_STOP) != std::string::npos)
	{
		if (_debug) 
		{
			std::cout << "debug : ack STOP"<<std::endl;			
		}
		_mode = STOP;
		
	} else
	if (str.find(GCS_PARAM) != std::string::npos)
	{
		if (_debug) 
		{
			std::string _str = str.erase (0,5);
			std::cout << "debug : ack "<<_str<< std::endl;
		}
		_mode = PARAM;
		
	} else
	if (str.find(GCS_WAYPOINT) != std::string::npos)
	{
		if (_debug) 
		{
			std::string _str = str.erase (0,5);
			std::cout << "debug : ack "<<_str<< std::endl;
		}
		_mode = WAYPOINT;
	} else
	if (str.find(GCS_PING) != std::string::npos)
	{
		if (_debug) 
		{
			std::string _str = str.erase (0,5);
			std::cout << "debug : ack PING"<< std::endl;
		}
		_mode = PING;
	} 
	if (str.find(GCS_DATA) != std::string::npos)
	{
		if (_debug) 
		{
			std::string _str = str.erase (0,5);
			std::cout << "debug : ack DATA"<< std::endl;
		}
		_mode = DATA;
	} 
	//debugging
	else
	if (str.find("#GB> TSENSOR_H") != std::string::npos)
	{
		if (_debug) 
		{
			std::cout << "debug : ack "<< std::endl;
		}
		_mode = 101;
	} 
	else
	if (str.find("#GB> TCONTROL_H") != std::string::npos)
	{
		if (_debug) 
		{
			std::cout << "debug : ack "<< std::endl;
		}
		_mode = 102;
	} 
	else
	if (str.find("#GB> TFSM_H") != std::string::npos)
	{
		if (_debug) 
		{
			std::cout << "debug : ack "<< std::endl;
		}
		_mode = 103;
	} 
	else
	if (str.find("#GB> TSENSOR_S") != std::string::npos)
	{
		if (_debug) 
		{
			std::cout << "debug : ack "<< std::endl;
		}
		_mode = 111;
	} 
	else
	if (str.find("#GB> TCONTROL_S") != std::string::npos)
	{
		if (_debug) 
		{
			std::cout << "debug : ack "<< std::endl;
		}
		_mode = 112;
	} 
	else
	if (str.find("#GB> TFSM_S") != std::string::npos)
	{
		if (_debug) 
		{
			std::cout << "debug : ack "<< std::endl;
		}
		_mode = 113;
	} 
	
	//debugging

	//insert another command here if necessary
		/*
		if (_debug) 
		{
			std::cout << "debug : return mode : "<<_mode<<std::endl;
			if (_mode == 0) {std::cout << "unidentified data : "<<str<<std::endl;}
		}	*/
	return _mode;
}

#endif
