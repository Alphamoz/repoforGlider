#include <moduleCommunication/OperationParameter.hpp>

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <algorithm>

// parameter index
const char _opMode = 2;
const char _glidemode = 3;
const char _depthOperation = 4;
const char _glidingAngle = 5;
const char _interv = 6;
const char _alti = 7;
const char _bar = 8;
const char _bsl = 9;
const char _miniCT = 10;
const char _oseRate = 11;
const char _propeller = 12;
const char _bow = 13;
const char _readStat = 14;
const char _rtoStat = 15;
const char _rtbStat = 16;
const char _latRTB = 17;
const char _lonRTB = 18;
const char _reserve1 = 19;
const char _reserve2 = 20;

// print all member
void gliderParam::Print()
{
	std::cout << "\t 1.opMode         : " << opMode << std::endl;
	std::cout << "\t 2.glidemode      : " << glideMode << std::endl;
	std::cout << "\t 3.depthOperation : " << depthOperation << std::endl;
	std::cout << "\t 4.glidingAngle   : " << glidingAngle << std::endl;
	std::cout << "\t 5.sensorInterval : " << interv << std::endl;
	std::cout << "\t 6.alti           : " << alti << std::endl;
	std::cout << "\t 7.bar            : " << bar << std::endl;
	std::cout << "\t 8.bsl            : " << bsl << std::endl;
	std::cout << "\t 9.miniCT         : " << miniCT << std::endl;
	std::cout << "\t10.oseRate        : " << oseRate << std::endl;
	std::cout << "\t11.propeller      : " << propeller << std::endl;
	std::cout << "\t12.bow            : " << bow << std::endl;
	std::cout << "\t13.readStat       : " << readStat << std::endl;
	std::cout << "\t14.rtoStat        : " << rtoStat << std::endl;
	std::cout << "\t15.rtbStat        : " << rtbStat << std::endl;
	std::cout << "\t16.latRTB         : " << latRTB << std::endl;
	std::cout << "\t17.lonRTB         : " << lonRTB << std::endl;
}

bool gliderParam::isValid(std::string str, char word)
{
	std::istringstream _sentence(str);
	int _count = 0;
	bool temp = false;

	do
	{
		std::string _word;
		_sentence >> _word;
		_count++;
		// std::cout << _count << std::endl;

	} while (_sentence);
	if (_count - 3 == word)
		temp = true;
	else
		temp = false;
	return temp;
}

// parse GCS parameter
void gliderParam::Set(std::string str)
{
	std::istringstream _sentence(str);
	char _count = 0;
	do
	{
		std::string _word;
		_sentence >> _word;
		switch (_count)
		{
		case _opMode:
			std::istringstream(_word) >>
				opMode;
			break;
		case _alti:
			std::istringstream(_word) >>
				alti;
			break;
		case _bar:
			std::istringstream(_word) >>
				bar;
			break;
		case _bsl:
			std::istringstream(_word) >>
				bsl;
			break;
		case _miniCT:
			std::istringstream(_word) >>
				miniCT;
			break;
		case _glidemode:
			std::istringstream(_word) >>
				glideMode;
			break;
		case _rtbStat:
			std::istringstream(_word) >>
				rtbStat;
			break;
		case _rtoStat:
			std::istringstream(_word) >>
				rtoStat;
			break;
		case _depthOperation:
			std::istringstream(_word) >>
				depthOperation;
			break;
		case _glidingAngle:
			std::istringstream(_word) >>
				glidingAngle;
			break;
		case _oseRate:
			std::istringstream(_word) >>
				oseRate;
			break;
		case _propeller:
			std::istringstream(_word) >>
				propeller;
			break;
		case _bow:
			std::istringstream(_word) >>
				bow;
			break;
		case _readStat:
			std::istringstream(_word) >>
				readStat;
			break;
		case _interv:
			std::istringstream(_word) >>
				interv;
			break;
		case _latRTB:
			// just in case
			// replace all ',' to '.' using algorithm lib
			std::replace(_word.begin(), _word.end(), ',', '.');
			std::istringstream(_word) >> std::fixed >> std::setprecision(12) >>
				latRTB;
			break;
		case _lonRTB:
			// just in case
			// replace all ',' to '.' using algorithm lib
			std::replace(_word.begin(), _word.end(), ',', '.');
			std::istringstream(_word) >> std::fixed >> std::setprecision(12) >>
				lonRTB;
			break;
		}
		_count++;
	} while (_sentence);
}