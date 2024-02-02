// update 9 september 2020
// deprecated
// updated with parse function on utils.

#include <moduleCommunication/CommandParser.hpp>

#include <iostream>
#include <sstream>

char AT = 99;
char RTB = 1;
char START = 2;
char PARAM = 3;
char WAYPOINT = 4;
char PING = 5;
char DATA = 6;
char STOP = 7;
char SELFTEST = 8;
char MANUALGLIDE = 9;
char PITCHCONTROL = 10;
char DEPTHCONTROL = 11;
char NGOTAK = 12;

std::string GCS_AT = "#GB> AT";
std::string GCS_RTB = "#GB> RTB";
std::string GCS_START = "#GB> START";
std::string GCS_PARAM = "#GB> PARAM";
std::string GCS_WAYPOINT = "#GB> WAYPOINT";
std::string GCS_PING = "#GB> PING";
std::string GCS_DATA = "#GB> DATA";
std::string GCS_STOP = "#GB> STOP";
std::string GCS_SELFTEST = "#GB> SELFTEST";
std::string GCS_MANUALGLIDE = "#GB> SELFTEST";
std::string GCS_PITCHCONTROL = "#GB> PITCHCONTROL";
std::string GCS_DEPTHCONTROL = "#GB> DEPTHCONTROL";
std::string GCS_NGOTAK = "#GB> NGOTAK";

int ParsingGCS(std::string str, bool _debug)
{
	int _mode = -1;

	if (str.find(GCS_AT) != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack AT" << std::endl;
		}
		_mode = AT;
	}
	else if (str.find(GCS_RTB) != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack RTB" << std::endl;
		}
		_mode = RTB;
	}
	else if (str.find(GCS_START) != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack START" << std::endl;
		}
		_mode = START;
	}
	else if (str.find(GCS_STOP) != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack STOP" << std::endl;
		}
		_mode = STOP;
	}
	else if (str.find(GCS_PARAM) != std::string::npos)
	{
		if (_debug)
		{
			std::string _str = str.erase(0, 5);
			std::cout << "debug : ack " << _str << std::endl;
		}
		_mode = PARAM;
	}
	else if (str.find(GCS_WAYPOINT) != std::string::npos)
	{
		if (_debug)
		{
			std::string _str = str.erase(0, 13);
			std::cout << "debug : ack " << _str << std::endl;
		}
		_mode = WAYPOINT;
	}
	else if (str.find(GCS_PING) != std::string::npos)
	{
		if (_debug)
		{
			// std::string _str = str.erase (0,5);
			std::cout << "debug : ack PING" << std::endl;
		}
		_mode = PING;
	}
	else if (str.find(GCS_SELFTEST) != std::string::npos)
	{
		if (_debug)
		{
			std::string _str = str.erase(0, 13);
			std::cout << "debug : ack SELFTEST" << _str << std::endl;
		}
		_mode = SELFTEST;
	}
	else if (str.find(GCS_MANUALGLIDE) != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack MANUALGLIDE" << std::endl;
		}
		_mode = MANUALGLIDE;
	}
	else if (str.find(GCS_PITCHCONTROL) != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack PITCHCONTROL" << std::endl;
		}
		_mode = PITCHCONTROL;
	}
	else if (str.find(GCS_DEPTHCONTROL) != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack DEPTHCONTROL" << std::endl;
		}
		_mode = DEPTHCONTROL;
	}
	if (str.find(GCS_NGOTAK) != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack NGOTAK" << std::endl;
		}
		_mode = NGOTAK;
	}

	if (str.find(GCS_DATA) != std::string::npos)
	{
		if (_debug)
		{
			// std::string _str = str.erase (0,5);
			std::cout << "debug : ack DATA" << std::endl;
		}
		_mode = DATA;
	}
	// debugging
	else if (str.find("#GB> TSENSOR_H") != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack " << std::endl;
		}
		_mode = 101;
	}
	else if (str.find("#GB> TCONTROL_H") != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack " << std::endl;
		}
		_mode = 102;
	}
	else if (str.find("#GB> TFSM_H") != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack " << std::endl;
		}
		_mode = 103;
	}
	else if (str.find("#GB> TSENSOR_S") != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack " << std::endl;
		}
		_mode = 111;
	}
	else if (str.find("#GB> TCONTROL_S") != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack " << std::endl;
		}
		_mode = 112;
	}
	else if (str.find("#GB> TFSM_S") != std::string::npos)
	{
		if (_debug)
		{
			std::cout << "debug : ack " << std::endl;
		}
		_mode = 113;
	}

	// debugging

	// insert another command here if necessary
	/*
	if (_debug)
	{
		std::cout << "debug : return mode : "<<_mode<<std::endl;
		if (_mode == 0) {std::cout << "unidentified data : "<<str<<std::endl;}
	}	*/
	return _mode;
}
