#include <iostream>
#include <string>
#include <sstream>
#include "ExtendedLib/_param.h"
#include "ExtendedLib/_gcsCommandParser.h"
#include "ExtendedLib/_wPoint.h"

using namespace std;

string	data;


///***************************************************************************
gliderParam gParam;
gliderWP gWPoint;

void handlerGCS(int dat,string data, bool _debug = false)
{
	if (dat == AT)
	{
		
	}
	//-------------------------------------------------------------------------
	if (dat == START)
	{
		//check for RTB/RTO/test propeller/test bowthruster
		if (int c = gParam.subProcessExist())
		{
			cout << "unable to start : " << "pending task code "<<c<<endl;
		}else
		{
			cout <<"starting mission"<<endl;
			if (_debug)gParam.print();
			if (_debug)gWPoint.print();
		}
	}else
	//-------------------------------------------------------------------------
	if (dat == RTB)
	{
		cout <<"RTB to "<<gParam.latRTB<< " "<<gParam.lonRTB<<endl;
	} else 
	//-------------------------------------------------------------------------
	if (dat == PARAM)
	{
		gParam.set(data);
		if (_debug) gParam.print();
	} else 
	//-------------------------------------------------------------------------
	if (dat == WAYPOINT)
	{
		gWPoint.set(data);
		if (_debug) gWPoint.print();
	}
}
///***************************************************************************

void test()
{
	string data;

	data = "#GB> AT";
	handler(parsing_GCS(data),data);

	data = "#GB> PARAM 1 0 1 19 1 0 2 0 0 123 25 128 0 0 0 1 8.12312 -18.12312";
	handler(parsing_GCS(data),data);

	data = "#GB> WAYPOINT 8.12312 -18.12312 4.565656 -78.97977";
	handler(parsing_GCS(data),data);

	data = "#GB> START";
	handler(parsing_GCS(data),data,true);

	data = "#GB> RTB";
	handler(parsing_GCS(data),data,true);

}

int main(){
	/*
	data = "#GB> PARAM 1 0 1 19 1 0 2 0 0 123 25 128 0 0 0 1 8.12312 -18.12312";
	
	int dat = parsing_GCS(data);
	if (dat == START)
	{
		cout <<"starting mission"<<endl;
		gParam.print();
		gWPoint.print();
		
	}
	if (dat == PARAM)
	{
		gParam.set(data);
		gParam.print();
	} else 
	if (dat == WAYPOINT)
	{
		gWPoint.set(data);
		gParam.print();
	}
	
	*/
	test();
	return 0;
}	
