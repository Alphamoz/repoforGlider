#ifndef _gcsCommandParser
#define _gcsCommandParser

#include <string>

extern char AT,RTB,START,PARAM,WAYPOINT,PING,DATA,STOP, SELFTEST, MANUALGLIDE, PITCHCONTROL, DEPTHCONTROL, NGOTAK;

extern std::string GCS_AT,GCS_RTB,GCS_START,GCS_PARAM, GCS_WAYPOINT,GCS_PING,GCS_DATA,GCS_STOP, GCS_SELFTEST, 
			GCS_MANUALGLIDE, GCS_PITCHCONTROL,GCS_DEPTHCONTROL, GCS_NGOTAK;
	
int ParsingGCS(std::string str,bool _debug);

#endif


//update 9 september 2020
//deprecated
//updated with parse function on utils.
