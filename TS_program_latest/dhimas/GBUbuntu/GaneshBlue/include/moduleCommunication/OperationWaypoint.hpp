#ifndef _wPoint
#define _wPoint

#include <vector>
#include <string>

extern const char _latStart;
extern const char _lonStart;
extern const char _latEnd;			
extern const char _lonEnd;	
extern const char _status;	

struct gliderWP{
	std::vector <float> latStart;
	std::vector <float> latEnd;
	std::vector <float> lonStart;
	std::vector <float> lonEnd;
	std::vector <int> status;
	
	void Set(std::string str);
	bool isValid(std::string str);
	void Print();
};

#endif