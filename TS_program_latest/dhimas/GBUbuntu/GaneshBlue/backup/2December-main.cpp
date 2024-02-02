#pragma GCC diagnostic ignored "-Wwrite-strings"

#include <moduleGlider/libSensor.hpp>
#include <moduleGlider/libGlider.hpp>

#include <Utils.hpp>
#include <DebugTools.hpp>
#include <main.hpp>

#include <moduleControl/NavGdn.hpp>
#include <moduleControl/PID.hpp>

#include <moduleCommunication/OperationParameter.hpp>
#include <moduleCommunication/OperationWaypoint.hpp>

#include <chrono>
#include <iostream>
#include <fstream>
#include <istream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/time.h>

#include <thread>
#include <mutex>
#include <limits>

using namespace std;

#define ACTIVE 1
#define INNACTIVE 0
#define ON 1
#define OFF 0

#define DEBUGTOOLS ON

Sensor sensor;
GliderConstants gliderConstants;
Glider glider;

Guidance guidance;
Navigation navigation;

PID heading{"heading"};
PID pitch{"pitch"};
PID buoyancy{"buoyancy"};

PIDconst rudder{"rudder"};
PIDconst ballast{"ballast"};
PIDconst bladder{"bladder"};

gliderParam gParam;
gliderWP gWPoint;

Utils::Communication GS,AR,BB,debugOverride;

mutex threadmutex;

FSM fsm;

std::string get_current_dir() {
   char buff[FILENAME_MAX]; //create string buffer to hold path
   getcwd( buff, FILENAME_MAX );
   string current_working_dir(buff);
   return current_working_dir;
}

chrono::steady_clock::time_point timeStart,timeNow; //buffer (global)

//.:.override debugging variables
//WARNING OVERRIDE
bool overrideBBACK = true;
bool overrideARACK = true;

bool overrideInterval = false;
int overrideIntervalValue;
bool datalogAlwaysReady = false;

bool echoSerial = false;
//:.:override debugging variables

int main(int argc, char *argv[])
{
	if (!DEBUGTOOLS)
	{
		cout << "DEBUGTOOLS is off, no output will be printed"<<endl;
		cout << "remake with DEBUGTOOLS on to see output"<<endl;
	}

	//.:. variables initialization
	//set all PID output to 0
	heading.Reset();
	pitch.Reset();
	buoyancy.Reset();

	//reset values
	glider.Reset();
	sensor.Reset();
	guidance.Reset();
	navigation.Reset();

	//pays attention to logs path
	//change path on utils if required
	dwatch (get_current_dir());
	
	rudder.loadConfig();
	ballast.loadConfig();
	bladder.loadConfig();
	gliderConstants.loadConfig();
	//:.: variables initialization

	//.:.serial initialization
	dcout (":Opening Serial port ... \n");
	Utils::loadConfig(GS,"GS.cfg");
	Utils::loadConfig(BB,"BB.cfg");
	Utils::loadConfig(AR,"AR.cfg");
	//testbench
	if (argc>=2)
	{
		//testbench location
		GS.loc = "/dev/ttyS5";
		BB.loc = "/dev/ttyS7";
		AR.loc = "/dev/ttyS11";

		// //debugport
		Utils::loadConfig(debugOverride,"OverridePort.cfg");
		debugOverride.port = Utils::openPort(debugOverride.loc,debugOverride.baudrate);
		debugOverride.activateThread();
	}

	//.:.added on December
	Utils::loadConfig(debugOverride,"OverridePort.cfg");
	debugOverride.port = Utils::openPort(debugOverride.loc,debugOverride.baudrate);
	drun(debugOverride.printConnectionStatus());
	//:.:added on December

	//override serial port for debug override
	debugOverride.activateThread();
	thread override_thread(debug_override_task);
		if(DEBUGTOOLS)
			override_thread.detach();
		else
			override_thread.join();

	//open port
	GS.port = Utils::openPort(GS.loc,GS.baudrate);
	BB.port = Utils::openPort(BB.loc,BB.baudrate);	
	AR.port = Utils::openPort(AR.loc,AR.baudrate);

	drun(GS.printConnectionStatus());
	drun(BB.printConnectionStatus());
	drun(AR.printConnectionStatus());

	dcout ("done\n");
	//:.:serial initialization

	//.:.multithread initialization
	dcout (":Create then detach sub-thread \n");
	int harwareConcurency = std::thread::hardware_concurrency();
	dwatch(harwareConcurency);
	
	thread Actuator_thread(Actuator_task);
	thread Sensor_thread(Sensor_task);
	thread GCS_thread(GCS_task);

	Actuator_thread.detach();
	Sensor_thread.detach();
	GCS_thread.detach();
	dcout ("done\n");
	//:.:multithread initialization

	//get last operational parameter (failsafe )
	dcout (":Loading last parameters ... \n");
	ifstream lastOPParam (workPath + "logs/Operational-Parameter.txt");
	string _lastOPParam;
	getline(lastOPParam,_lastOPParam);
	lastOPParam.close();
	
	//set initial param
	gParam.Set(_lastOPParam);
	drun(gParam.Print());
	dcout("done\n");

	//get last operational waypoint (failsafe )
	dcout (":Loading last waypoints ... \n");
	ifstream lastOPWPoint (workPath + "logs/Operational-Waypoint.txt");
	string _lastOPWPoint;
	getline(lastOPWPoint,_lastOPWPoint);
	lastOPWPoint.close();
	
	//make new datalog for sensor based on date and time
    char _buff[300];
	{
		time_t _time;
		struct tm * _timestamp;
		time(&_time);
		_timestamp = localtime(&_time);
		strftime(_buff, sizeof(_buff), "%d%m%Y-%H%M%S", _timestamp);
	}
	glider.datalogIsCreated = false;
	glider.datalogPath = ("sensor/" + Utils::convertToString(_buff));

	dwatch(glider.datalogPath);	

	//set initial waypoint
	gWPoint.Set(_lastOPWPoint);
	drun(gWPoint.Print());
	dcout("done\n");

	//clear buffer
	tcflush(GS.port,TCIOFLUSH);
	tcflush(BB.port,TCIOFLUSH);
	tcflush(AR.port,TCIOFLUSH);
	
	//activate multithread task
	GS.activateThread();
	BB.deactivateThread();
	AR.deactivateThread();

	//set initial FSM state
	fsm.state = 0;
	fsm.lastState = 0;
	fsm.logIsCreated = false;

	int PINGcounterBB = 0; //buffer
	int PINGcounterAR = 0; //buffer
	int PINGcounterGS = 0; //buffer

	int PINGcounterLimit = 20; //constants
	int batteryThreshold = 0; //constants

	//put BB AR on standby
	Utils::serWrite(BB.port,"#TS> OPERATION 2\r\n");
	Utils::serWrite(AR.port,"#TS> OPERATION 2\r\n");

	long long int timeElapsed; //function
	long long int timeThreshold; //constants	
	
	//main loop
	glider.pose = gliderConstants.staystill;
	while (1)
	{
		//check for battery
		if (sensor.analog.volt < batteryThreshold)
		{
			//if state is already triggered by GS
			if (fsm.lowerState() < FAILSAFE && fsm.lowerState() > SETUP)
			{
				fsm.moveTo(LOWBATTERYFAILSAFE);
				//consumption request
				fsm.isWaitingSensorUpdate = true;
			}
			//else state is not yet triggered, failsafe is ignored
		}

		switch (fsm.lowerState())
		{
		case WAITING:

			break;
		case SETUP:
			glider.datalogReady = false;
			//turn off GS thread
			GS.deactivateThread();
			//GS.thread = INNACTIVE;
			//set target according to waypoint number
			fsm.targetCount = gWPoint.status.size() * BASE;
			//target completion requirement
			fsm.targetCount += SINGLETARGETFINISH;
			//initial operation state
			fsm.state = BASE;
			//calculate finished waypoints
			//increase base operation according to finished waypoint
			for (int i=0; i<gWPoint.status.size(); i++)
			{
				if (gWPoint.status[i]==1)
					fsm.state += BASE;
			}

			fsm.moveTo(SINGLETARGETSETUP);
			//fsm.Print();
			
			BB.activateThread();
			if(!overrideInterval)
			{
				char BBparam[300];
				sprintf(BBparam,"#TS> PARAM %d\r\n",gParam.interv);
				Utils::serWrite(BB.port,BBparam);
			} else
			{
				char BBparam[300];
				sprintf(BBparam,"#TS> PARAM %d\r\n",overrideIntervalValue);
				Utils::serWrite(BB.port,BBparam);
			}
			//wait ack param
			//change to PING for bypass, default val : WAITING
			BB.isACK = PING;
			
			//todo : sendparam to AR
				//wait ack param
			AR.activateThread();
			//change to PING for bypass, default val : WAITING
			AR.isACK = PING;
			
			break;
		case SINGLETARGETSETUP:		
			//check for reply from BB
			if (BB.isACK == PING || BB.isACK == OPERATION || overrideBBACK)
			{
				//check for reply from AR
				if (AR.isACK == PING || AR.isACK == OPERATION || overrideARACK)
				{
					//send start signal (all sensor active)
					Utils::serWrite(BB.port,"#TS> OPERATION 3\r\n");;
					//send ACK start/RTO
					GS.sendACK();

					//set waypoint
					guidance.sourceLat = gWPoint.latStart[fsm.upperState()-1];
					guidance.sourceLon = gWPoint.lonStart[fsm.upperState()-1];
					guidance.targetLat = gWPoint.latEnd[fsm.upperState()-1];
					guidance.targetLon = gWPoint.lonEnd[fsm.upperState()-1];
					fsm.isWaitingSensorUpdate = true;
					glider.pose  = gliderConstants.descending;
					fsm.moveTo(GLIDING);
				}
				else
				{
					//send PING to AR
					Utils::serWrite(AR.port,"#TS> OPERATION 1\r\n");
					PINGcounterAR ++;
					if (PINGcounterAR>PINGcounterLimit)
						fsm.moveTo(HALT);
				}
			} 
			else
			{
				//send PING to BB
				Utils::serWrite(BB.port,"#TS> OPERATION 1\r\n");
				PINGcounterBB ++;
				if (PINGcounterBB > PINGcounterLimit)
					fsm.moveTo(HALT);
			}

			break;
		case LOWBATTERYFAILSAFE:
			//make sure BB thread is active
			BB.activateThread();
			if (fsm.isWaitingSensorUpdate)
			{
				//wait producer
				if (glider.sensorIsUpdated)
				{
					glider.sensorIsUpdated = false;
					fsm.isWaitingSensorUpdate = false;
				}
			}
			else
			{
				//end guidance system
				guidance.isFinished = true;
				//check glider position (surface/glide)
				if (sensor.ALTI.depth < gliderConstants.surfaceThreshold)
				{
					//surfacing immediately
					//override guidance
					glideOperation(1);
				} 
				else
				//consume
				{
					//glider is consired on surface
					//activate GS handler
					tcflush(GS.port,TCIOFLUSH);
					GS.activateThread();
					//reset PID
					pitch.Reset();
					heading.Reset();
					buoyancy.Reset();
					//do RTB next
					//full operation mode is needed to acquire waypoint
					Utils::serWrite(BB.port,"#TS> OPERATION 3\r\n");
					fsm.moveTo(SETUPRTB);
				}
				//notify producer
				fsm.isWaitingSensorUpdate = true;
			}			
			break;		
		case GLIDING:
			if (fsm.isWaitingSensorUpdate)
			{
				//wait producer
				if (glider.sensorIsUpdated)
				{
					glider.sensorIsUpdated = false;
					fsm.isWaitingSensorUpdate = false;
				}				
			} 
			else
			//consume
			{
				//do normal glide operation
				glideOperation(0);

				if (guidance.isFinished)
				{
					guidance.isFinished = false;
					glider.datalogReady = true;

					fsm.moveTo(GLIDECOMPLETE);
				} else
				{
					//ask producer
					fsm.isWaitingSensorUpdate = true;
				}
			}
			break;
		case GLIDECOMPLETE:
			//activate GS handler
			tcflush(GS.port,TCIOFLUSH);
			GS.activateThread();
			//put BB AR to standby
			Utils::serWrite(BB.port,"#TS> OPERATION 2\r\n");
			Utils::serWrite(AR.port,"#TS> OPERATION 2\r\n");
			//start timer
			timeStart = chrono::steady_clock::now();

			//reset PID
			pitch.Reset();
			heading.Reset();
			buoyancy.Reset();

			fsm.moveTo(WAITINSTRUCTION);

			break;
		case WAITINSTRUCTION:
			/*
			4 situations :
				1) Timer overflow
				2) receive PING from GS
				3) receive RTB from GS
				4) receive RTO from GS
			2 outcome :
				A) move to SETUPRTB
				B) move to SINGLETARGETFINISH	
			*/
			
			if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count()>120)
			{
				//situation 1
				//full operation mode is needed to acquire waypoint
				Utils::serWrite(BB.port,"#TS> OPERATION 3\r\n");
				fsm.isWaitingSensorUpdate = true;
				tcflush(GS.port,TCIOFLUSH);
				GS.activateThread();
				
				fsm.moveTo(SETUPRTB);
				//notify producer
				fsm.isWaitingSensorUpdate = true;
			}
			//situation 2,3, and 4 detected by GS handler
			//situation 2 will reset timeStart
			//situation 3 cause outcome A
			//situation 4 cause outcome B 
			break;
		case SETUPRTB:
			if (fsm.isWaitingSensorUpdate)
			{
				//wait producer
				if (glider.sensorIsUpdated)
				{
					glider.sensorIsUpdated = false;
					fsm.isWaitingSensorUpdate = false;
				}				
			}
			else
			//consume
			{
				//RTB backtrack
				//set current position as source waypoint
				//set last source as target waypoint
				guidance.targetLat = gWPoint.latStart[fsm.upperState()-1];
				guidance.targetLon = gWPoint.lonStart[fsm.upperState()-1];
				guidance.sourceLat = sensor.IMU.latitude;
				guidance.sourceLon = sensor.IMU.longitude;
				
				//notify producer
				fsm.isWaitingSensorUpdate = true;
				glider.pose = gliderConstants.descending;
				fsm.moveTo(RTBOPERATION);
			}

			break;
		case RTBOPERATION:
			if (fsm.isWaitingSensorUpdate)
			{
				//wait producer
				if (glider.sensorIsUpdated)
				{
					glider.sensorIsUpdated = false;
					fsm.isWaitingSensorUpdate = false;
				}				
			} 
			else
			//consume
			{
				//do normal glide operation
				glideOperation(0);
				//ask producer

				if (guidance.isFinished)
				{
					guidance.isFinished = false;
					fsm.moveTo(MISSIONABORTED);
				} else
				{
					fsm.isWaitingSensorUpdate = true;
				}
			}
			
			break;
		case MISSIONABORTED:
			/*
			backtracking algorithm
			situation : 
				1) all waypoints backtracked
				2) not all waypoints backtracked
			outcome :
				A) move to SETUPRTB
				B) move to WAITING
			*/

			if (fsm.upperState()==1)
			{
				//all waypoints backtracked
				//fsm state should be BASE + MISSIONABORTED

				//set to waiting
				fsm.moveTo(WAITING);
			}else
			{
				//not all waypoints backtracked

				//continue backtrack
				fsm.state -= BASE;
				fsm.moveTo(SETUPRTB);

			}

			break;
		case HALT:
			{
				//disable further state
				//caused by internal system fail
				//(AR and or BB not responding)
				//log all
				//exit program
				//return 0;
			}
			break;
		case SINGLETARGETFINISH:
			//check if another waypoint exist
			if (fsm.state == fsm.targetCount)
			{
				//all waypoint complete
				//fsm state should be ((number of waypoints) * BASE)+SINGLETARGETFINISH
				
				//set new state
				fsm.moveTo(WAITING);

			} else
			{
				//fsm state should ((number of waypoints travelled) * BASE)+SINGLETARGETFINISH
				
				//move to next operation target
				fsm.state += BASE;
				//set new state
				fsm.moveTo(SINGLETARGETSETUP);
			}
			break;
		default:
			break;
		}
		sleep(1);
	}

	dcout ("\n");
	return 0;
}

void FSM::moveTo(int _target)
{
	bool _isNegative = false;
	if (state<0) _isNegative = true;

	int newstate = (int) (state/BASE) * BASE;
	lastState = state;
	state = newstate + _target;

	if (_isNegative) state *= -1;

	dcout("Change FSM from "<<fsm.lastState<<" to "<<fsm.state<<"\n");
	if (!fsm.logIsCreated)
	{
		//clear operational log
		ofstream datalog(workPath + "logs/Operational-FSM.txt",std::ios::out | std::ios::trunc);
		datalog.close();
		fsm.logIsCreated = true;
	}
	if (fsm.logIsCreated)
	{
		//log
		char message[100];
		sprintf(message,"Change FSM from %d to %d",fsm.lastState,fsm.state);
		Utils::write2LOG("Operational-FSM",message,false);
		Utils::write2LOG("List-FSM",message,false);
	}
}

void FSM::Print()
{
	dwatch(state);
	dwatch(lastState);
}

int FSM::upperState()
{
	return state / BASE;
}
int FSM::lowerState()
{
	return abs(state % BASE);
}

//added december
int counterguidance = 0;

int glideOperation(int override = 0)
{
	//get lock for multithread
	threadmutex.lock();

	//calculate guidance and navigation
	navigation = calculateNavigation(navigation, sensor);
    guidance = calculateGuidance(guidance, navigation.latitude, navigation.longitude, guidance.targetLat, guidance.targetLon);

	//added december
	counterguidance++;
	if (counterguidance>400)
	{
		guidance.isFinished = true;
		
		counterguidance = 0;
	}
	dcout(counterguidance)

	//logging
	char logMessageGlide[300];
	sprintf(logMessageGlide,"%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f",	navigation.latitude,navigation.longitude,
																		guidance.distanceFromTarget,guidance.psiRef,
																		guidance.sourceLat,guidance.sourceLon,
																		guidance.targetLat,guidance.targetLon);
	Utils::write2LOG("Glide-NavigationGuidance",logMessageGlide,false);
	

	//ascending
	if (glider.pose == gliderConstants.ascending)
	{
		if (sensor.ALTI.depth>gliderConstants.surfaceDepth || sensor.ALTI.depth>gliderConstants.surfaceThreshold)
		{
			//check guidance
			if (guidance.isFinished)
			{
				//cout <<"STAYSTILL"<<endl;
				glider.pose = gliderConstants.staystill;
				PIDOperation(gliderConstants.staystill);
				//stop glide operation
			}
			else
			{
				//cout <<"STAYSTILL"<<endl;
				//change to descending
				glider.pose = gliderConstants.descending;
				PIDOperation(gliderConstants.staystill);
			}
		} else
		{
			//continue ascending
			PIDOperation(gliderConstants.ascending);							
		}
	}
	else
	//descending
	if (glider.pose == gliderConstants.descending)
	{
		if (sensor.ALTI.depth>gParam.depthOperation && sensor.ALTI.depth>gliderConstants.depthThreshold)
		{
			//continue descending
			PIDOperation(gliderConstants.descending);			
		} 
		else
		{
			//cout <<"STAYSTILL"<<endl;
			//change to ascending
			glider.pose = gliderConstants.ascending;
			PIDOperation(gliderConstants.staystill);
		}
	}
	threadmutex.unlock();
	return 0;
}


void PIDOperation(int glidestate)
{
	//data for arduino :
		//float bladder
		//float ballast
		//float teta
		//float rudder
		//float pose
		//float posn
		//int strobe
	
	//bladder <- PID
	//ballast <- PID
	//rudder <- PID
	//teta <- surfacepitch/ascendpitch/descendpitch
	//pos_e <- data BB
	//pos_n <- data BB
	
	//yawref	<-  nav/guidance (?)
	//global heading <- nav/guidance (?)
	
	//pitchref <- teta / user
	//global pitch <- sensor BB
	
	//depthref <- GCS / user
	//globaldepth <- sensor BB
	
	//.:.ondevelopment
	float yawRefference = 135;//guidance.psiRef;
	float pitchRefference;
	//:.:ondevelopment
	
	//ascending
	if (glidestate == 1 )
	{
		//propeller off

		//teta <- ascendpitch
		pitchRefference = gliderConstants.ascendPitch;

		//strobe on
		glider.strobe = gliderConstants.strobe[INNACTIVE];
		
		//use antiwindup
		heading = CalculatePID(heading,rudder,yawRefference,sensor.IMU.omegaHeading,true);		
		pitch = CalculatePID(pitch,ballast,gliderConstants.ascendPitch,sensor.IMU.omegaPitch,false);
		buoyancy = CalculatePID(buoyancy,bladder,gliderConstants.surfaceDepth,sensor.ALTI.depth,false);

		//pos_e ??
		//pos_n ??
	}
	else 
	//descending
	if (glidestate == 0)
	{
		//propeller off

		//teta <- descendpitch
		pitchRefference = gliderConstants.descendPitch;
		
		//strobe off
		glider.strobe = gliderConstants.strobe[INNACTIVE];
		
		heading = CalculatePID(heading,rudder,yawRefference,sensor.IMU.omegaHeading,true);		
		pitch = CalculatePID(pitch,ballast,gliderConstants.descendPitch,sensor.IMU.omegaPitch,false);
		buoyancy = CalculatePID(buoyancy,bladder,gParam.depthOperation,sensor.ALTI.depth,false);

		//pos_e ??
		//pos_n ??

	} else
	//stay still
	if (glidestate == 2)
	{
		//propeller off
		//teta <- surfacepitch
		pitchRefference = gliderConstants.surfacePitch;
		
		//reset actuator
	//	heading.Reset();		
		pitch.Reset();
		buoyancy.Reset();
		
		//pos_e ??
		//pos_n ??
		
		//strobe on		
		glider.strobe = gliderConstants.strobe[ACTIVE];
		this_thread::sleep_for(chrono::milliseconds(2000));
	}
	char ARbuffer[100];
	sprintf(ARbuffer,"#TS> %.6f %.6f %.6f %d\r\n",heading.output, pitch.output, buoyancy.output,glider.strobe);
	Utils::serWrite(AR.port,ARbuffer);

	//logging
	char logMessagePID[300];
	//rudder
	sprintf(logMessagePID,"%.6f %.6f %.6f %.6f",heading.refference,heading.current,heading.error,heading.output);
	Utils::write2LOG("PID-heading",logMessagePID,false);
	//ballast
	sprintf(logMessagePID,"%.6f %.6f %.6f %.6f",pitch.refference,pitch.current,pitch.error,pitch.output);
	Utils::write2LOG("PID-pitch",logMessagePID,false);
	//bladder
	sprintf(logMessagePID,"%.6f %.6f %.6f %.6f",buoyancy.refference,buoyancy.current,buoyancy.error,buoyancy.output);
	Utils::write2LOG("PID-buoyancy",logMessagePID,false);
}

/*
	change values of :
		gliderwaypoint
		gliderparameter
		communication data
	read values of :
		glider
		sensor
*/
void GCS_task()
{
	//wait and skip handling if task_flag is not raised
	//handle incoming communication from GCS
	while(1)
	{
		if(!GS.thread) {
		}
		else
		{
			// blocking read serial
			char GCSinbound[300];
			memset(&GCSinbound,'\0',sizeof(GCSinbound));
			int bytes_read = read(GS.port,&GCSinbound,sizeof(GCSinbound));

			if (echoSerial)
				cout << "GS RAW : " << GS.rawdata << endl;

			//check if thread is still active
			if (!GS.thread){
			} else
			{
				GS.rawdata = Utils::convertToString(GCSinbound);

				//parse data based on rawdata
				//if valid, change values of 
				//datasender, datacommand, dataargs, and datacode
				//return -1 on dataCode if data is not found within defaultmessage
				Utils::Parse(GS);
				dwatch(GS.rawdata);
				// dwatch(GS.dataSender);
				// dwatch(GS.dataCommand);
				// dwatch(GS.dataArgs);
				// dwatch(GS.dataCode);

				switch (GS.dataCode)
				{
				case AT : 

					break;
				case RTB :
					GS.sendACK();
					fsm.moveTo(SETUPRTB);
					break;
				case START :
						//dcout("STARTING WITH : \n");
						//drun(gParam.Print());
						//drun(gWPoint.Print());
						//trigger FSM
						fsm.state = SETUP;
					break;
				case PARAM : 
					if (gParam.isValid(GS.rawdata,17))
					{
						gParam.Set(GS.rawdata);
						//drun(gParam.Print());

						//use contiguous storage of std::string to convert to char *
						//rewrite operational log
						Utils::write2LOG("Operational-Parameter",&*GS.rawdata.begin(),true);
						//append log
						Utils::write2LOG("List-Parameter",&*GS.dataArgs.begin(),false);

						GS.sendACK();
					} else
					{
						//invalid data arguments
						//log error
						Utils::write2LOG("Error-Parameter",&*GS.dataArgs.begin(),false);
					}
					break;
				case WAYPOINT : 
					if (gWPoint.isValid(GS.rawdata))
					{
						gWPoint.Set(GS.rawdata);
						//drun(gWPoint.Print());

						//use contiguous storage of std::string to convert to char *
						//rewrite operational log
						Utils::write2LOG("Operational-Waypoint",&*GS.rawdata.begin(),true);
						//append log
						Utils::write2LOG("List-Waypoint",&*GS.dataArgs.begin(),false);

						GS.sendACK();
					} else
					{
						//invalid data arguments
						//log error
						Utils::write2LOG("Error-Waypoint",&*GS.dataArgs.begin(),false);
					}
					break;
				case PING : //PING
						timeNow = chrono::steady_clock::now();
						GS.sendACK();
					break;
				case DATA : //DATA
					if (glider.datalogReady || datalogAlwaysReady)
					{
						sendDatalog();
						//let GCS know that all data is sent
						Utils::serWrite(GS.port,"#TS> ENDDATA\r\n");

						//wait for next glide operation
						glider.datalogReady = false;
					}
					break;
				case STOP : //STOP
					//change glider fsm to waiting
					GS.sendACK();
					fsm.moveTo(WAITING);

					break;
				case OPERATION : //OPERATION
					//null
					break;
				case RTO : //RTO OR CTM
				case CTM : 
					GS.sendACK();
					if (fsm.state!=SINGLETARGETFINISH)
						fsm.moveTo(SINGLETARGETFINISH);
					break;
					
				default: //invalid data
					//try to flush I/O (not tested)
					tcflush(GS.port,TCIOFLUSH);
					break;
				}// switch case based on datacommand
			}
		}
	}
}

/*
	change values of :
		sensor
		fsm (sensorupdate)
	read values of :
		fsm (sensorupdate)
*/

void Sensor_task()
{
	//wait and skip handling if task_flag is not raised
	//handle incoming communication from GCS
	while(1)
	{
		if(!BB.thread) {
		}
		else
		{
			// blocking read serial
			char BBinbound[1000];
			memset(&BBinbound,'\0',sizeof(BBinbound));
			int bytes_read = read(BB.port,&BBinbound,sizeof(BBinbound));
			BB.rawdata = Utils::convertToString(BBinbound);

			if (echoSerial)
				cout << "BB RAW : " << BB.rawdata << endl;
			//check if thread is still active
			if (!BB.thread){
			} else
			{

				//parse data based on rawdata
				//if valid, change values of 
				//datasender, datacommand, dataargs, and datacode
				//return -1 on dataCode if data is not found within defaultmessage
				Utils::Parse(BB);
				//dwatch(BB.rawdata);
				// dwatch(BB.dataSender);
				// dwatch(BB.dataCommand);
				// dwatch(BB.dataArgs);
				// dwatch(BB.dataCode);
				// dwatch(BB.isACK);

				switch (BB.dataCode)
				{
				case AT : //AT

					break;
				case RTB : //RTB

					break;
				case START : //START

					break;
				case PARAM : //PARAM

					break;
				case WAYPOINT : //WAYPOINT

					break;
				case PING : //PING

					break;
				case DATA : //DATA
					//get lock to avoid data update while gliding
					threadmutex.lock();
					//separate args
					sensor.Parse(BB.dataArgs);	
					//sensor.Print(sensorIMU | sensorDVL | sensorALTI);
					
					//write sensor to log
					//create new datalog if not exist
					if (!glider.datalogIsCreated)
					{
						ofstream datalog(workPath + "logs/"+glider.datalogPath+".txt",std::ios::out | std::ios::app);
						datalog.close();
						glider.datalogIsCreated = true;
					}
					//datalog exist
					//append sensor data
					if (glider.datalogIsCreated)
					{
						char buffer[300];
						//todo (get board temperature)
						glider.boardTemp = 30;
						//measure speed
						glider.speed = sqrt(pow(sensor.DVL.veloX_F,2)+pow(sensor.DVL.veloY_F,2)+pow(sensor.DVL.veloZ_F,2));
						
						//write to log
						sprintf(buffer,"%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d %.6f %.6f %.6f %.6f",
							sensor.MINICT.temperature,
							sensor.MINICT.conductivity,
							sensor.ALTI.depth,
							sensor.IMU.latitude,
							sensor.IMU.longitude,
							sensor.IMU.omegaRoll,
							sensor.IMU.omegaPitch,
							sensor.IMU.omegaHeading,
							glider.pose,
							sensor.analog.volt,
							sensor.analog.leak,
							glider.speed,
							glider.boardTemp
						);
						Utils::write2LOG(&*glider.datalogPath.begin(),buffer,false);
					}
					
					//feed consumer
					if (fsm.isWaitingSensorUpdate) glider.sensorIsUpdated = true; 
					//release lock
					threadmutex.unlock();
					break;
				case STOP : //STOP

					break;
				case OPERATION : //OPERATION
					break;
					
				default: //invalid data
					//try to flush I/O (not tested)
					tcflush(BB.port,TCIOFLUSH);
					break;
				}// switch case based on datacommand
			}
		}
	}
}

void Actuator_task()
{
	//wait and skip handling if task_flag is not raised
	//handle incoming communication from GCS
	while(1)
	{
		if(!AR.thread) {
		}
		else
		{
			// blocking read serial
			char ARinbound[1000];
			memset(&ARinbound,'\0',sizeof(ARinbound));
			int bytes_read = read(AR.port,&ARinbound,sizeof(ARinbound));
			AR.rawdata = Utils::convertToString(ARinbound);

			if (echoSerial)
				cout << "AR RAW : " << AR.rawdata << endl;

			//check if thread is still active
			if (!AR.thread){
			} else
			{

				//parse data based on rawdata
				//if valid, change values of 
				//datasender, datacommand, dataargs, and datacode
				//return -1 on dataCode if data is not found within defaultmessage
				Utils::Parse(AR);
				dwatch(AR.dataSender);
				dwatch(AR.dataCommand);
				dwatch(AR.dataArgs);
				dwatch(AR.dataCode);

				switch (AR.dataCode)
				{
				case AT : //AT

					break;
				case RTB : //RTB

					break;
				case START : //START

					break;
				case PARAM : //PARAM

					break;
				case WAYPOINT : //WAYPOINT

					break;
				case PING : //PING

					break;
				case DATA : //DATA

					break;
				case STOP : //STOP

					break;
				case OPERATION : //OPERATION

					break;
					
				default: //invalid data
					//try to flush I/O (not tested)
					tcflush(AR.port,TCIOFLUSH);
					break;
				}// switch case based on datacommand
			}
		}
	}
}

void sendDatalog()
{
	//deactivate threads
	GS.deactivateThread();
	BB.deactivateThread();
	AR.deactivateThread();

	//open log
	ifstream datalog(workPath + "logs/"+glider.datalogPath+".txt");

	//skip sent lines
	for (int i = 0; i<glider.datalogCount; i++)
	{
		string buffer;
		getline(datalog,buffer);
	}

	//send while condition is good and not EOF
	while((datalog.good()) && (datalog.peek() != EOF))
	{ 
		/*
		line example :
		01-10-2020 09:05:34	16.000000 17.000000 15.000000 10.000000 11.000000 6.000000 4.000000 5.000000 0 1.000000 3.000000 22.561028 30.000000
		*/
		string __date;
		string __time;
		string __temperature;
		string __salinity;
		string __depth; 
		string __latitude;
		string __longitude;
		string __roll;
		string __pitch;
		string __yaw;
		string __motion;
		string __battery;
		string __leakage;
		string __speed;
		string __boardTemp;
		
		getline(datalog,__date,' ');
		getline(datalog,__time,'\t');
			
		getline(datalog,__temperature,' ');
		getline(datalog,__salinity,' ');
		getline(datalog,__depth,' '); 
		getline(datalog,__latitude,' ');
		getline(datalog,__longitude,' ');
		getline(datalog,__roll,' ');
		getline(datalog,__pitch,' ');
		getline(datalog,__yaw,' ');
		getline(datalog,__motion,' ');
		getline(datalog,__battery,' ');
		getline(datalog,__leakage,' ');
		getline(datalog,__speed,' ');
		getline(datalog,__boardTemp,'\n');
		
		char buffer[300];

		strcpy (buffer,("#TS> DATA "+
				__date+" "+
				__time+" "+
				__temperature+" "+
				__salinity+" "+
				__depth+" "+
				__latitude+" "+
				__longitude+" "+
				__roll+" "+
				__pitch+" "+
				__yaw+" "+
				__motion+" "+
				__battery+" "+
				__leakage+" "+
				__speed+" "+
				__boardTemp+
				"\r\n").c_str());
		
		//output example
		//#TS> DATA 01-10-2020 09:05:34 16.000000 17.000000 15.000000 10.000000 11.000000 6.000000 4.000000 5.000000 0 1.000000 3.000000 22.561028 30.000000
		Utils::serWrite(GS.port,buffer);
		glider.datalogCount++;
	}

	//close file
	datalog.close();
	
	//reactivate threads
	GS.activateThread();
	AR.activateThread();
	BB.activateThread();
}

void debug_override_task()
{
	//thread is destroyed if DEBUGTOOLS is OFF
	if(!DEBUGTOOLS)
	{}
	else
	//wait and skip handling if task_flag is not raised
	//handle incoming communication from GCS
	while(1)
	{
		if(!debugOverride.thread) {
		}
		else
		{
			// blocking read serial
			char DebInbound[1000];
			memset(&DebInbound,'\0',sizeof(DebInbound));
//			int bytes_read = read(debugOverride.port,&DebInbound,sizeof(DebInbound));
//			debugOverride.rawdata = Utils::convertToString(DebInbound);
			getline(cin,debugOverride.rawdata);
			if (echoSerial)
				cout << "DP RAW : " << debugOverride.rawdata << endl;

			//check if thread is still active
			if (!debugOverride.thread){
			} else
			{
				dwatch(debugOverride.rawdata);
				stringstream sentence(debugOverride.rawdata);
				string variable;
				sentence >> variable;
				string value;
				sentence >> value;
				cout << "var : " << variable <<"  val : " << value << endl;
			
				if (variable == "guidanceisfinished")
				{
					if (value == "true" || value == "t" || value == "1")
						guidance.isFinished = true;
					else
					if (value == "false" || value == "f" || value == "0")
						guidance.isFinished = false;
				} else
				if (variable == "bbalwaysack")
				{
					if (value == "true" || value == "t" || value == "1")
						overrideBBACK = true;
					else
					if (value == "false" || value == "f" || value == "0")
						overrideBBACK = false;
				} else
				if (variable == "aralwaysack")
				{
					if (value == "true" || value == "t" || value == "1")
						overrideARACK = true;
					else
					if (value == "false" || value == "f" || value == "0")
						overrideARACK = false;
				} else
				if (variable == "datalogalwaysready")
				{
					if (value == "true" || value == "t" || value == "1")
						datalogAlwaysReady = true;
					else
					if (value == "false" || value == "f" || value == "0")
						datalogAlwaysReady = false;
				} else
				if (variable == "setdatalog")
				{
					glider.datalogPath = (workPath + "sensor/" + value);
					glider.datalogIsCreated = true;
				} else
				if(variable == "bbinterval")
				{
					overrideInterval = true;
					overrideIntervalValue = stoi(value);
				} else
				if(variable == "datalogcount")
				{
					glider.datalogCount = stoi(value);
				} else
				if(variable == "arthread")
				{
					if (value == "true" || value == "t" || value == "1")
						AR.activateThread();
					else
					if (value == "false" || value == "f" || value == "0")
						AR.deactivateThread();
				} else
				if(variable == "bbthread")
				{
					if (value == "true" || value == "t" || value == "1")
						BB.activateThread();
					else
					if (value == "false" || value == "f" || value == "0")
						BB.deactivateThread();
				}
				else
				{
					cout << "unknown override" << endl;
				}
				
			}
		}
	}
}
