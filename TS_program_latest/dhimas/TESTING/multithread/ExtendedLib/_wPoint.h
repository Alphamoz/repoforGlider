#ifndef _wPoint
#define _wPoint

#include <iostream>
#include <string>
#include <sstream>

const char _latStart = 2;
const char _lonStart = 3;
const char _latEnd = 4;			
const char _lonEnd = 5;	

struct gliderWP{
	float latStart,
		  lonStart,
		  latEnd,
		  lonEnd;

	void print(){
		std::cout<<".:.GLIDER WAYPOINT.:."<<std::endl;
		std::cout<<"latStart : "<<latStart<<std::endl;
		std::cout<<"lonStart : "<<lonStart<<std::endl;
		std::cout<<"latEnd   : "<<latEnd<<std::endl;
		std::cout<<"lonEnd   : "<<lonEnd<<std::endl;
		std::cout<<":.:GLIDER WAYPOINT:.:"<<std::endl;
	}
	
	void set(std::string str){
		std::istringstream _sentence (str);
		char _count = 0;
		do
		{
			std::string _word;
			_sentence >> _word;
			switch (_count)
			{
				case _latStart : std::istringstream (_word) >> 
										 latStart; 
										 break;
				case _lonStart : std::istringstream (_word) >> 
										 lonStart; 
										 break;
				case _latEnd : std::istringstream (_word) >> 
										 latEnd; 
										 break;
				case _lonEnd : std::istringstream (_word) >> 
										 lonEnd; 
										 break;
			}
			_count++;
		} while (_sentence);
	}
};

#endif