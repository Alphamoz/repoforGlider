#ifndef _gcsCommandParser
#define _gcsCommandParser

#include <iostream>
#include <string>
#include <sstream>

#define AT 0
#define RTB 1
#define START 2
#define PARAM 3
#define WAYPOINT 4

const	std::string GCS_AT = "#GB> AT";
const	std::string GCS_RTB = "#GB> RTB";
const	std::string GCS_START = "#GB> START";
const	std::string GCS_PARAM = "#GB> PARAM";
const	std::string GCS_WAYPOINT = "#GB> WAYPOINT";
	
int parsing_GCS(std::string str,bool _debug = false){
	int _mode=0;
		
	if (str.find(GCS_AT) != std::string::npos)
	{
		if (_debug) 
		{
			std::cout << "debug : ack AT"<<std::endl;			
		}
		_mode = 0;

	} else
	if (str.find(GCS_RTB) != std::string::npos)
	{
		if (_debug) 
		{
			std::cout << "debug : ack RTB"<<std::endl;			
		}
		_mode = 1;
		
	} else
	if (str.find(GCS_START) != std::string::npos)
	{
		if (_debug) 
		{
			std::cout << "debug : ack START"<<std::endl;			
		}
		_mode = 2;
		
	} else
	if (str.find(GCS_PARAM) != std::string::npos)
	{
		if (_debug) 
		{
			std::string _str = str.erase (0,5);
			std::cout << "debug : ack "<<_str<< std::endl;
		}
		_mode = 3;
		
	} else
	if (str.find(GCS_WAYPOINT) != std::string::npos)
	{
		if (_debug) 
		{
			std::string _str = str.erase (0,5);
			std::cout << "debug : ack "<<_str<< std::endl;
		}
		_mode = 4;
		
		
	} 
	//insert another command here if necessary
	
	return _mode;
}

#endif