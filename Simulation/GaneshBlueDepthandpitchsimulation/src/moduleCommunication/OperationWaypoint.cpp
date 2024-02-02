#include <moduleCommunication/OperationWaypoint.hpp>

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <algorithm>

const char _latStart = 2;
const char _lonStart = 3;
const char _latEnd = 4;			
const char _lonEnd = 5;	
const char _status = 6;

/*
void gliderWP::Print(){
	std::cout<<"\t latStart : "<<std::setprecision(12)<<latStart<<std::endl;
	std::cout<<"\t lonStart : "<<std::setprecision(12)<<lonStart<<std::endl;
	std::cout<<"\t latEnd   : "<<std::setprecision(12)<<latEnd<<std::endl;
	std::cout<<"\t lonEnd   : "<<std::setprecision(12)<<lonEnd<<std::endl;
}
*/

//check if arguments contain 5 set of data
//latstart, latend, lonstart,lonend, status
bool gliderWP::isValid(std::string str){
	std::istringstream _sentence (str);
	char _count = 0;
	bool temp = false;

	do
	{
		std::string _word;
		_sentence >> _word;
		_count++;
	} while (_sentence);
	if ((_count-3)%5==0) temp = true;
	else temp = false;
	return temp;
}

void gliderWP::Set(std::string str){
	std::istringstream _sentence (str);
	char _count = 0;
	
	//clear previous vector
	latStart.clear();
	lonStart.clear();
	latEnd.clear();
	lonEnd.clear();
	status.clear();

	do
	{
		std::string _word;
		_sentence >> _word;
		//just in case
		//replace all ',' to '.' using algorithm lib
		std::replace( _word.begin(), _word.end(), ',', '.');

		float temp;
		if (_word!="")
			switch (_count)
			{			

				case _latStart : 
					std::istringstream (_word) >> std::fixed >> 
						std::setprecision(12) >> temp;
					latStart.push_back(temp);
					break;
				case _lonStart : 
					std::istringstream (_word) >> std::fixed >> 
						std::setprecision(12) >> temp;
					lonStart.push_back(temp);
					break;
				case _latEnd : 
					std::istringstream (_word) >> std::fixed >> 
						std::setprecision(12) >> temp;
					latEnd.push_back(temp);
					break;
				case _lonEnd : 
					std::istringstream (_word) >> std::fixed >> 
						std::setprecision(12) >> temp;
					lonEnd.push_back(temp);
					break;
				case _status : 
					std::istringstream (_word) >> std::fixed >> 
						std::setprecision(12) >> temp;
					status.push_back((int)temp);
					break;
			}
			_count++;
			if (_count == 7)
				_count = 2;
		} while (_sentence);
}

void gliderWP::Print(){
    std::cout << "\tStatus---latStart---latEnd---lonStart---lonEnd" << std::endl; 
	for (int i = 0; i < status.size(); i++) 
	{
        std::cout << "\t" << latStart[i] << " "; 
        std::cout << "\t" << lonStart[i] << " "; 
        std::cout << "\t" << latEnd[i] << " "; 
        std::cout << "\t" << lonEnd[i] << " "; 
        std::cout << "\t" << status[i] << std::endl;
	} 
}