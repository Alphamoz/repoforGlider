#pragma GCC diagnostic ignored "-Wwrite-strings"

#include <moduleGlider/libSensor.hpp>
#include <moduleGlider/libGlider.hpp>
// #include <moduleGlider/libActuator.hpp>

#include <Utils.hpp>
#include <DebugTools.hpp>
#include <main.hpp>
#include <vector>

#include <moduleControl/NavGdn.hpp>
#include <moduleControl/PID.hpp>

#include <moduleCommunication/OperationParameter.hpp>
#include <moduleCommunication/OperationWaypoint.hpp>

#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>
#include <istream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h>	 // Contains file controls like O_RDWR
#include <errno.h>	 // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>	 // write(), read(), close()
#include <sys/time.h>
#include <sys/socket.h>
#include <limits>
#include <poll.h>

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
// Actuator actuator;

Guidance guidance;
Navigation navigation;

PID heading{"heading"};
PID pitch{"pitch"};
PID buoyancy{"buoyancy"};
PID depth{"depth"};
PID veloDepth{"veloDepth"};

PIDconst rudder{"rudder"};
PIDconst ballast{"ballast"};
PIDconst bladder{"bladder"};

gliderParam gParam;
gliderWP gWPoint;

Utils::Communication GS, AR1, AR2, debugOverride;
Utils::TCPCommunication BB;
mutex threadmutex;

FSM fsm;

std::string get_current_dir()
{
	char buff[FILENAME_MAX]; // create string buffer to hold path
	getcwd(buff, FILENAME_MAX);
	string current_working_dir(buff);
	return current_working_dir;
}

chrono::steady_clock::time_point timeStart, timeNow; // buffer (global)

//.:.override debugging variables
// WARNING OVERRIDE
bool overrideBBACK = true;
bool overrideARACK = true;

bool overrideInterval = false;
int overrideIntervalValue;
bool datalogAlwaysReady = true;

bool echoSerial = true;
float desYaw = 0.1;
float desPitch = 0.0;
float desDepth = 0.0;

//:.:override debugging variables
bool first = true;
int state = -1;
// this global variable may moved to the glider structure next to make code readable
int offsetIMU;
int MMvarHigh;
int MMvarLow;
int BEvarMid;
int MMvarMid;
int rudderMid;
int BE_input;
int MM_input;
int yaw_input;
int main_input;
int rudder_input = rudderMid;
int FSM_status;
float lastDepth = 0;
float depthNow = 0;
float lastTime = 0;
bool manualInput = true;
char _buff[300];
bool withYawControl = false;
bool askingInput = true;
bool withMainThruster = false;

void readDataArduino()
{
	char ARinbound1[100];
	char ARinbound2[100];
	// std::string receivedData;
	memset(&ARinbound1, '\0', sizeof(ARinbound1));
	memset(&ARinbound2, '\0', sizeof(ARinbound2));
	// check whether the if its success to read
	int bytes_readAR1 = read(AR1.port, &ARinbound1, sizeof(ARinbound1));
	AR1.rawdata = Utils::convertToString(ARinbound1);
	int bytes_readAR2 = read(AR2.port, &ARinbound2, sizeof(ARinbound2));
	AR2.rawdata = Utils::convertToString(ARinbound2);
	//
	dcout("Recieved Data AR1: " << AR1.rawdata << " --Read status: " << bytes_readAR1 ? "Success" : "failed");
	dcout("Recieved Data AR2: " << AR2.rawdata << " --Read status: " << bytes_readAR2 ? "Success" : "failed");

	std::string statusText;
	// double BE_now;
	// int MM_now;
	double vardump;
	std::stringstream dataStream(AR1.rawdata);
	dataStream >> statusText >> BE_input >> vardump;
	std::stringstream dataStream2(AR2.rawdata);
	dataStream2 >> statusText >> MM_input >> glider.hullTemp >> glider.humidity;
	// MM_input = MM_now;
	tcflush(AR1.port, TCIOFLUSH);
	tcflush(AR2.port, TCIOFLUSH);
}

void resetInputVariable()
{
	dcout("Resetting input variables \n");
	BE_input = BEvarMid;
	MM_input = MMvarMid;
	yaw_input = 1500;
	main_input = 1200;
	rudder_input = rudderMid;
	dcout("Reset input variable complete \n");
}

void moveActuator(int BE, int MM, int rudder, int yawThruster, int mainThruster)
{
	dcout("INPUTTING SIGNAL TO ACTUATOR \n");
	char ARbuffer1[100];
	char ARbuffer2[100];
	dcout("INPUT SIGNAL: --BE:" << BE << " --MM:" << MM << " --Rudder:" << rudder << " --yawThruster:" << yawThruster << " --mainThruster" << mainThruster << "\n");
	sprintf(ARbuffer1, "#TS %d %d %d %d %d %d %d\r\n", BE, 0, 0, rudder, yawThruster, mainThruster, 0);
	sprintf(ARbuffer2, "#TS %d\r\n", MM);
	Utils::serWrite(AR1.port, ARbuffer1);
	Utils::serWrite(AR2.port, ARbuffer2);
}

void resetActuator()
{
	dcout("RESETTING ACTUATOR \n");
	resetInputVariable();
	moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
}
bool firstDataLog = true;
void getDataLog()
{
	char logMessageGlide[300];
	// char logHeader[300];
	if (firstDataLog)
	{
		sprintf(logMessageGlide, "Status_GPS Lat_init Long_Init Lat Long Vx Vy Vz E N D distanceFromTarget psiRef sourceLat sourceLong targetLat targetLong");
		std::string navigationPath = ("navigation/" + Utils::convertToString(_buff));
		Utils::write2LOG(&*navigationPath.begin(), logMessageGlide, false);
	}
	sprintf(logMessageGlide, "%d %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f",
			navigation.Status_GPS, navigation.latitude_init, navigation.longitude_init,
			navigation.latitude, navigation.longitude,
			navigation.vX_DVL, navigation.vY_DVL, navigation.vZ_DVL,
			navigation.E_aug, navigation.N_aug, navigation.Depth_aug,
			guidance.distanceFromTarget, guidance.psiRef,
			guidance.sourceLat, guidance.sourceLon,
			guidance.targetLat, guidance.targetLon);
	std::string navigationPath = ("navigation/" + Utils::convertToString(_buff));
	Utils::write2LOG(&*navigationPath.begin(), logMessageGlide, false);

	char logMessagePID[300];
	if (firstDataLog)
	{
		sprintf(logMessagePID, "Heading_ref Heading_curr Heading_err Heading_Out Pitch_ref Pitch_curr Pitch_err Pitch_out Buoy_ref Buoy_curr Buoy_err Buoy_out");
		std::string controlPath = ("control/" + Utils::convertToString(_buff));
		Utils::write2LOG(&*controlPath.begin(), logMessagePID, false);
	}
	sprintf(logMessagePID, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d %.6f %.6f",
			heading.refference, heading.current, heading.error, heading.output,
			desPitch, sensor.IMU.omegaPitch, pitch.error, pitch.output,
			buoyancy.refference, buoyancy.current, buoyancy.error, buoyancy.output,
			glider.strobe, desDepth, desPitch);
	std::string controlPath = ("control/" + Utils::convertToString(_buff));
	Utils::write2LOG(&*controlPath.begin(), logMessagePID, false);

	char logMessageInput[300];
	if (firstDataLog)
	{
		sprintf(logMessageInput, "BE statusResetBE  MM rudder mainThruster bow _");
		std::string inputPath = ("input/" + Utils::convertToString(_buff));
		Utils::write2LOG(&*inputPath.begin(), logMessageInput, false);
	}
	sprintf(logMessageInput, "%d %d %d %d %d %d %d",
			BE_input, 0, MM_input, rudder_input, yaw_input, main_input, 0);
	std::string inputPath = ("input/" + Utils::convertToString(_buff));
	Utils::write2LOG(&*inputPath.begin(), logMessageInput, false);

	char buffer[300];
	// todo (get board temperature)
	glider.boardTemp = 30;
	// measure speed
	glider.speed = sqrt(pow(sensor.DVL.veloX_F, 2) + pow(sensor.DVL.veloY_F, 2) + pow(sensor.DVL.veloZ_F, 2));
	if (firstDataLog)
	{
		sprintf(buffer, "Temp Conductivity Depth Vx(cal) Vy(cal) Vz(cal) Lat Long Roll Pitch Heading Pose Volt Leak Speed HullTemperature Humidity");
		Utils::write2LOG(&*glider.datalogPath.begin(), buffer, false);
		firstDataLog = false;
	}
	// write to log
	sprintf(buffer, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d %.6f %.6f %.6f %.6f %.6f",
			sensor.MINICT.temperature,
			sensor.MINICT.conductivity,
			depthNow,
			sensor.IMU.latitude,
			sensor.IMU.longitude,
			sensor.IMU.omegaRoll,
			sensor.IMU.omegaPitch,
			sensor.IMU.omegaHeading,
			glider.pose,
			sensor.analog.volt,
			sensor.analog.leak,
			glider.speed,
			glider.hullTemp,
			glider.humidity);

	Utils::write2LOG(&*glider.datalogPath.begin(), buffer, false);
	// dcout("--Depth: " << sensor.ALTI.depth << "--Pitch: " << sensor.IMU.omegaPitch << "--Heading: " << sensor.IMU.omegaHeading);
}

void calculateNavGuide()
{
	navigation = calculateNavigation(navigation, sensor);
	// guidance = calculateGuidance(guidance, -6.88652107576, 107.610694559, -6.88664152084, 107.610554876);
	guidance = calculateGuidance(guidance, navigation.longitude, navigation.latitude, guidance.targetLat, guidance.targetLon);
	dcout("Status GPS: " << navigation.Status_GPS << "\n");
	dcout("Covariance GPS: " << sensor.IMU.posCov << "\n");
	dcout("East Now: " << navigation.E_aug << "\n");
	dcout("North Now: " << navigation.N_aug << "\n");
	// dcout("Distance From Target: " << guidance.distanceFromTarget);
	// dcout("Yaw Reference" << guidance.psiRef);
	// dcout("Calculating Navigation process Done\n");
	// dcout("Calculating index \n ");
	// dcout("Index" << (fsm.upperState() - 1) << "\n");
	// dcout(gWPoint.latEnd.size());
	// dcout("gWPoint.latEnd: ");
	// for (int i= 0; i < gWPoint.latEnd.size(); i++){
	// 	dcout(gWPoint.latEnd.at(i));
	// }
	// gWPoint.latEnd

	// guidance.targetLat = gWPoint.latEnd[fsm.upperState() - 1];
	// guidance.sourceLat = gWPoint.latStart[fsm.upperState() - 1];
	// guidance.sourceLon = gWPoint.lonStart[fsm.upperState() - 1];
	// guidance.targetLat = gWPoint.latEnd[fsm.upperState() - 1];
	// guidance.targetLon = gWPoint.lonEnd[fsm.upperState() - 1];
}

void ambilDataGlider()
{
	getDataLog();
	sleep(1);
}

int calcControlYaw(bool forward = false)
{

	dcout("Forward Value is: " << forward << "\n");
	// if(fabs(desYaw-sensor.IMU.omegaHeading) < 5 && fsm.lowerState() == MANUALGLIDE)
	// {return 1500;
	// }
	heading = CalculatePID(heading, rudder, desYaw, sensor.IMU.omegaHeading, false);
	// pitch = CalculatePID(pitch, ballast, pitchRefJason, sensor.IMU.omegaPitch, false);
	dcout("Desired Yaw: " << desYaw << "\n");
	dcout("Calculating PID done\n ");
	dcout("INPUT SIGNAL: ---Yaw: " << heading.output << "\n");
	if (forward == true)
	{
		main_input = 1400;
	}
	else
	{
		main_input = 1200;
	}
	return heading.output;
}

void param2constants(gliderParam gp, GliderConstants &gc)
{
	double PI = 3.1415926536;

	gc.descendPitch = -1 * gp.glidingAngle * PI / 180;
	gc.ascendPitch = gp.glidingAngle * PI / 180;
	gc.depthThreshold = gp.depthOperation + 1;
}

double rad2deg(double f)
{
	return (f * 180) / 3.14159265359;
}

void selfTest()
{
	dcout("MASUK KE SELFTEST \n");
	dcout("dengan param:" << BE_input << " " << MM_input << " " << rudder_input << " " << yaw_input << " " << main_input << "\n");
	moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
}

int controlPitch()
{
	float setPoint = desPitch;
	dcout("SETPOINT PITCH: " << setPoint << "\n");
	float pitchError = sensor.IMU.omegaPitch - setPoint;
	dcout("PitchNow: " << sensor.IMU.omegaPitch << "\n");
	dcout("PITCH ERROR: " << pitchError << "\n");
	if (fabs(pitchError) > 1)
	{
		if (pitchError == 0)
		{

			return MM_input;
		}
		MM_input -= 1 * (fabs(pitchError) / pitchError);
		if (MM_input > MMvarHigh)
		{
			MM_input = MMvarHigh;
		}
		if (MM_input < MMvarLow)
		{
			MM_input = MMvarLow;
		}
	}
	dcout("INPUT SIGNAL: ---Moving Mass: " << MM_input << "\n");
	return MM_input;
}
float depth1 = 0;
float depth2 = 0;
float depth3 = 0;
float depth4 = 0;
float depth5 = 0;

float getMeanVelocity(float depth)
{
	float diff = 0;
	depth5 = depth4;
	depth4 = depth3;
	depth3 = depth2;
	depth2 = depth1;
	depth1 = depth;
	diff = depth1 - depth5;
	float meanVelo = diff / 2;
	return meanVelo;
}

int controlDepth()
{
	// Modified 18 Jan 2023
	float setPoint = desDepth;
	float output = 0;
	depthNow = sensor.ALTI.depth - 1.1 * sin((sensor.IMU.omegaPitch / 180) * 3.14);
	float timeDiff = 0.5;

	float velocityDepth = getMeanVelocity(depthNow);
	float veloDes = 0;
	dcout("Velocity Depth: " << velocityDepth << "\n");
	dcout("SET POINT Depth: " << setPoint << "\n");
	dcout("Depth Now: " << depthNow << "\n");

	int velocityPID = 0;

	int veloError = velocityDepth - veloDes;

	float maxVeloDes = 0.06;
	float minVeloDes = 0;
	float maxError = 1;
	float minError = 0.4;
	float minmaxErrorDiff = maxError - minError;
	float errorDepth = setPoint - depthNow;
	if (fabs(errorDepth) > maxError)
	{
		veloDes = maxVeloDes * (errorDepth) / abs(errorDepth);
		dcout("Velo menuju " << veloDes << "\n");
	}
	else if (abs(errorDepth) < minError)
	{
		veloDes = 0;
		dcout("Menuju nol velonya");
	}
	else
	{
		veloDes = maxVeloDes / minmaxErrorDiff * (errorDepth) - ((minError * maxVeloDes / minmaxErrorDiff) * (errorDepth) / fabs(errorDepth));
		dcout("Transient velo");
	}

	veloDepth = CalculatePID(veloDepth, bladder, veloDes, velocityDepth, false);
	velocityPID = veloDepth.output;
	dcout("Velocity PID: " << velocityPID << "\n");

	if (velocityPID > 10)
	{
		output = 10;
	}
	else if (velocityPID < -10)
	{
		output = -10;
	}
	else if (velocityPID == 0)
	{
		output = 0;
	}
	else
	{
		output = velocityPID;
	}

	dcout("Output to BE!!" << output << "\n");
	BE_input += output;
	dcout("BEINPUT!!" << BE_input);
	if (BE_input > bladder.saturation_upper)
	{
		BE_input = bladder.saturation_upper;
	}
	else if (BE_input < bladder.saturation_lower)
	{
		BE_input = bladder.saturation_lower;
	}
	dcout("INPUT SIGNAL: ---Bouyancy Engine: " << BE_input << "\n");
	lastDepth = depthNow;
	return BE_input;
}

void manualGliding()
{
	int Pinit = 25; // initialize
	int P0 = 100;	// pitchdown, go down 3m
	int P1 = 60;	// pitch down, hold 3m
	int P2 = 30;	// hold
	int P3 = 20;	// pitch up still 3m
	int P4 = 60;	// go up hold 1m
	int P5 = 60;	// hold 1m
	int P6 = 60;	// still holding still 1m
	if (first)
	{
		timeStart = chrono::steady_clock::now();
		first = false;
		desPitch = -15;
		desDepth = 0;
		dcout("Normal Position");
		dcout("Preparing stage. StayStill, Pitching down 5deg \n");
	}
	timeNow = chrono::steady_clock::now();
	// cout << "Start Time: " << timeStart << "\n";
	// cout << "Now: " << timeNow << "\n";
	cout << "Time Diff: " << std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() << "\n";
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > Pinit && state == -1)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desPitch = gliderConstants.descendPitch;
		desDepth = gliderConstants.depthThreshold;
		dcout("Pitching Down, 15deg, go down 3m \n");
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P0 && state == 0)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desDepth = gliderConstants.depthThreshold;
		desPitch = gliderConstants.descendPitch;
		dcout("Staying still 3m \n");
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P1 && state == 1)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desPitch = 0;
		desDepth = gliderConstants.depthThreshold;
		dcout("still hold 3m \n");
		// staystill
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P2 && state == 2)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desPitch = gliderConstants.ascendPitch;
		desDepth = 3;
		dcout("Pitch up 3m \n");
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P3 && state == 3)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		// desPitch = gliderConstants.descendPitch;
		desDepth = 1;
		dcout("go up 1 m \n");
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P4 && state == 4)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desDepth = 1;
		dcout("staystill \n");
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P5 && state == 5)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desDepth = 1;
		desPitch = 0;
		// desPitch = gliderConstants.ascendPitch;
		dcout("up to 1m \n");
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P6 && state == 6)
	{
		state = -1;
		timeStart = chrono::steady_clock::now();
		dcout("End Of ManualGlide");
		fsm.moveTo(STOP);
		return;
	}
	readDataArduino();
	MM_input = controlPitch();
	BE_input = controlDepth();
	yaw_input = withYawControl ? calcControlYaw() : 1500;

	moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
	calculateNavGuide();
	getDataLog();
	sleep(1);
}
void guidanceTestZigZag()
{
	int Pinit = 10;
	int P0 = 10;
	int P1 = 10;
	int P2 = 10;
	int P3 = 30;
	if (first)
	{
		timeStart = chrono::steady_clock::now();
		first = false;
		desYaw = 0;
		dcout("Starting The Zig Zag\n");
	}
	timeNow = chrono::steady_clock::now();
	// cout << "Start Time: " << timeStart << "\n";
	// cout << "Now: " << timeNow << "\n";
	cout << "Time Diff: " << std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() << "\n";
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > Pinit && state == -1)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desYaw = 315 - offsetIMU;
		dcout("Going left \n");
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P0 && state == 0)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desYaw = 45 - offsetIMU;
		dcout("Going right \n");
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P1 && state == 1)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desYaw = 0 - offsetIMU;
		dcout("Going north \n");
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P2 && state == 2)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desYaw = 180 - offsetIMU;
		dcout("Going South \n");
	}

	yaw_input = calcControlYaw(true);
	// MM_input = controlPitch();
	moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P3 && state == 3)
	{
		state++;
		resetActuator();
		fsm.moveTo(WAITING);
		return;
	}
	calculateNavGuide();
	getDataLog();
}
void guidanceTest()
{
	int Pinit = 17;
	int P0 = 43;
	int P1 = 48;
	int P2 = 43;
	int P3 = 17;
	if (first)
	{
		timeStart = chrono::steady_clock::now();
		first = false;
		desYaw = 90 - offsetIMU;
		dcout("Starting The Guidance Rectangle \n");
	}
	timeNow = chrono::steady_clock::now();
	// cout << "Start Time: " << timeStart << "\n";
	// cout << "Now: " << timeNow << "\n";
	cout << "Time Diff: " << std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() << "\n";
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > Pinit && state == -1)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desYaw = 360 - offsetIMU;
		dcout("Going North \n");
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P0 && state == 0)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desYaw = 270 - offsetIMU;
		dcout("Going West \n");
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P1 && state == 1)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desYaw = 180 - offsetIMU;
		dcout("Going South \n");
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P2 && state == 2)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desYaw = 90 - offsetIMU;
		dcout("Going West \n");
	}
	yaw_input = calcControlYaw(true);
	// MM_input = controlPitch();
	moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P3 && state == 3)
	{
		state++;
		resetActuator();
		fsm.moveTo(WAITING);
		return;
	}
	calculateNavGuide();
	getDataLog();
	usleep(500000);
}

void yawControl(bool surge = true)
{
	yaw_input = calcControlYaw(surge);
	moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
	sleep(1);
}

void userInputFSM()
{
	std::cout << "Options:" << std::endl;
	std::cout << "0. Lewat Radio" << std::endl;
	std::cout << "8. STOP" << std::endl;
	std::cout << "14. PITCHCONTROL" << std::endl;
	std::cout << "15. DEPTHCONTROL" << std::endl;
	std::cout << "17. MANUALGLIDE" << std::endl;
	std::cout << "19. WAITINSTRUCTION" << std::endl;
	std::cout << "21. YAW CONTROL" << std::endl;
	std::cout << "22. GUIDANCE (NGOTAK)" << std::endl;
	std::cout << "94. HALT" << std::endl;
	std::vector<int> values = {WAITING, STOP, PITCHCONTROL, DEPTHCONTROL, MANUALGLIDING, WAITINSTRUCTION, CONTROL, GUIDANCE, HALT, ARDUDATACHECK};

	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	int FSM_status;
	bool timeoutOccurred = false;

	struct pollfd fds[1];
	fds[0].fd = STDIN_FILENO;
	fds[0].events = POLLIN;

	while (true)
	{
		std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
		std::chrono::duration<double> elapsed_seconds = now - start;

		if (elapsed_seconds.count() >= 4.0)
		{
			timeoutOccurred = true;
			break;
		}

		int pollResult = poll(fds, 1, 0);

		if (pollResult == -1)
		{
			std::cout << "Error occurred while polling input. Exiting...\n";
			return;
		}
		else if (pollResult > 0 && fds[0].revents & POLLIN)
		{
			if (!(std::cin >> FSM_status))
			{
				std::cin.clear();
				std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
				std::cout << "Masukkin angka dong!.\n";
			}
			else
			{
				if (std::find(values.begin(), values.end(), FSM_status) != values.end())
				{
					if (FSM_status == 0)
					{
						std::cout << "Monggo lewat radio\n";
						// Perform desired actions for status 0
						fsm.moveTo(WAITING);
						manualInput = false;
					}
					else
					{
						// Move FSM to the corresponding state based on FSM_status
						fsm.moveTo(FSM_status);
					}
					return;
				}
				else
				{
					std::cout << "Belum bisa ganti FSM " << FSM_status << " Coba Lagi!\n\n";
					sleep(1);
				}
			}
		}
	}

	if (timeoutOccurred)
	{
		std::cout << "Input timeout! Exiting...\n";
	}
}

void userInputFSMold()
{
	cout << "Lewat Radio		=   0,\nSTOP			=   8,\nMANUALGLIDING		=  17,	\nWAITINSTRUCTION		=  19,\nYAW CONTROL		=  21,\nGUIDANCE		=  22,\nHALT			=  94" << endl;

	while (std::cout << "Mau FSM berapa bos?\n" && !(std::cin >> FSM_status))
	{
		std::cin.clear();													// clear bad input flag
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard input
		std::cout << "Masukkin angka dong!.\n";
	}
	std::vector<int> values = {0, 8, 17, 19, 21, 22, 94, 15, 18, 25, 101};
	if (std::find(values.begin(), values.end(), FSM_status) != values.end())
	{
		if (FSM_status == 0)
		{
			dcout("Monggo lewat radio \n");
			fsm.moveTo(WAITING);
			manualInput = false;
			return;
		}
		fsm.moveTo(FSM_status);
		return;
	}
	dcout("Belum bisa ganti FSM " << FSM_status << " Coba Lagi! \n\n");
	sleep(1);
}

float userInputDesired(std::string name = "")
{
	cout << "Masukkan Des " << name
		 << endl;
	float desiredVal = 0;
	while (std::cout << "\n" && !(std::cin >> desiredVal))
	{
		std::cin.clear();													// clear bad input flag
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard input
		std::cout << "Masukkin angka dong!.\n";
	}
	dcout("Desired " << name << " jadi " << desYaw << "\n\n");
	askingInput = false;
	sleep(1);
	return desiredVal;
}
void userInputActuator()
{
	cout << "BE MM Rudder BowT MainT\n"
		 << endl;
	string str;
	while (std::cout << "Masukkin mang" && !(std::cin >> str))
	{
		std::cin.clear();													// clear bad input flag
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard input
		std::cout << "Masukkin angka dong!.\n";
	}
	Utils::GliderUtility Gliderutil;
	Gliderutil.parsingData(str, &BE_input, &MM_input, &rudder_input, &yaw_input, &main_input);

	moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
	sleep(1);
}

void timeThresholdGlide()
{
	// setting available states
	int ROA = 0.5;
	int P0 = 20;
	int P1 = 20;
	int P2 = 20;

	// Going Pitch Down
	if (first)
	{
		first = false;
		desPitch = -15;
		desDepth = 0;
		dcout("Going Pitch Down");
	}
	timeNow = chrono::steady_clock::now();
	cout << "Time Diff: " << std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() << "\n";
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P0 && state == -1)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desPitch = gliderConstants.descendPitch;
		desDepth = gliderConstants.depthThreshold;
		dcout("Pitching Down, 15deg, go down 3m \n");
	}
	if (fabs(sensor.ALTI.depth - gliderConstants.depthThreshold) < ROA && state == 0)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desPitch = gliderConstants.ascendPitch;
		desDepth = gliderConstants.depthThreshold;
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P1 && state == 1)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desPitch = gliderConstants.ascendPitch;
		desDepth = gliderConstants.surfaceDepth;
		dcout("Pitching Down, 15deg, go down 3m \n");
	}
	if (fabs(sensor.ALTI.depth - gliderConstants.depthThreshold) < ROA && state == 2)
	{
		state = -1;
		fsm.moveTo(WAITING);
	}
	readDataArduino();
	MM_input = controlPitch();
	BE_input = controlDepth();
	yaw_input = withYawControl ? calcControlYaw() : 1500;

	moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
	calculateNavGuide();
	getDataLog();
	sleep(1);
}
bool guidanceFinished = false;
void surfacing()
{
	if (!guidanceFinished)
	{
		switch (state)
		{
		case -1:
			guidance.targetLat = -8;
			guidance.targetLon = -8;
			dcout("Going to target 1");
			break;
		case 0:
			guidance.targetLat = -16;
			guidance.targetLon = 0;
			dcout("Going to target 2");
			break;
		case 1:
			guidance.targetLat = -8;
			guidance.targetLon = 8;
			dcout("Going to target 3");
			break;
		case 2:
			guidance.targetLat = -2;
			guidance.targetLon = 2;
			dcout("Going to target 4");
			break;
		default:
			dcout("Invalid case");
			break;
		}
	}
	calculateNavGuide();
	getDataLog();
	desYaw = guidance.psiRef;
	yawControl(1);
	dcout("Distance From Target: " << guidance.distanceFromTarget << "\n");
	if (guidance.isFinished)
	{
		// going to next target
		state++;
		if (state == 3)
		{
			state = -1;
			fsm.moveTo(WAITING);
		}
		// gWPoint.status[fsm.upperState() - 1] = 1;
	}
	sleep(1);
}

void yawMechanism()
{
}

int main(int argc, char *argv[])
{
	// Checking debugtools, if not on, then printed warning message
	if (!DEBUGTOOLS)
	{
		cout << "DEBUGTOOLS is off, no output will be printed" << endl;
		cout << "remake with DEBUGTOOLS on to see output" << endl;
	}

	// set all PID output to 0: heading, pitch and bouyancy
	heading.Reset();
	pitch.Reset();
	buoyancy.Reset();

	// reseting glider, sensor, guidance and navigation values
	glider.Reset();
	sensor.Reset();
	guidance.Reset();
	navigation.Reset();

	// pays attention to logs path
	// change path on utils if required
	dwatch(get_current_dir());

	//:.: variables initialization
	// load configuration file of rudder, ballast, bladder and gliderConstant
	rudder.loadConfig();
	ballast.loadConfig();
	bladder.loadConfig();
	gliderConstants.loadConfig();

	// Modified 18 Jan 2023
	MMvarHigh = gliderConstants.MMvarHigh;
	MMvarLow = gliderConstants.MMvarLow;
	BEvarMid = gliderConstants.BEvarMid;
	offsetIMU = gliderConstants.offset;
	MMvarMid = (MMvarHigh + MMvarLow) / 2;
	rudderMid = gliderConstants.rudderMid;
	resetInputVariable();

	//.:.serial initialization
	// loading configuration port for GCS, BB and Arduino
	dcout(":Opening Serial port ... \n");
	Utils::loadConfig(GS, "GS.cfg");
	Utils::loadConfig(BB, "BB.cfg");
	Utils::loadConfig(AR1, "AR1.cfg");
	Utils::loadConfig(AR2, "AR2.cfg");

	// testbench
	// ***************testing for the port location??????????????????????
	if (argc >= 2)
	{
		// testbench location
		GS.loc = "/dev/ttyS5";
		BB.loc = "/dev/ttyS7";
		AR1.loc = "/dev/ttyS12";

		// //debugport
		// Utils::loadConfig(debugOverride, "OverridePort.cfg");
		// debugOverride.port = Utils::openPort(debugOverride.loc, debugOverride.baudrate);
		// debugOverride.activateThread();
	}

	//.:.added on December
	// getting the overidePort, for ttyS0 and baudrate 115200, but for what??
	// Utils::loadConfig(debugOverride, "OverridePort.cfg");
	// debugOverride.port = Utils::openPort(debugOverride.loc, debugOverride.baudrate);
	// drun(debugOverride.printConnectionStatus());
	//:.:added on December

	// commented out to disable override thread and reserve cpu usage
	// thread override_thread(debug_override_task);
	// if (DEBUGTOOLS)
	// 	override_thread.detach();
	// else
	// 	override_thread.join();

	// opening port GCS, BB, Arduino
	dcout("BB Loc: " << BB.loc);
	GS.port = Utils::openPort(GS.loc, GS.baudrate);
	BB.port = Utils::openPort(BB.loc, BB.baudrate, "TCP");
	dcout("BB port: " << BB.port);
	AR1.port = Utils::openPort(AR1.loc, AR1.baudrate);
	AR2.port = Utils::openPort(AR2.loc, AR2.baudrate);
	drun(GS.printConnectionStatus());
	drun(BB.printConnectionStatus());
	drun(AR1.printConnectionStatus());
	drun(AR2.printConnectionStatus());
	dcout("done\n");
	//:.:serial initialization

	//.:.multithread initialization
	dcout(":Create then detach sub-thread \n");
	int harwareConcurency = std::thread::hardware_concurrency();
	dwatch(harwareConcurency);

	// thread Actuator_thread(Actuator_task);
	thread Sensor_thread(Sensor_task);
	thread GCS_thread(GCS_task);
	// thread ambilData_thread(ambilDataGlider);
	// thread Arudino_thread(readDataArduino);

	// Actuator_thread.detach();
	// ambilData_thread.detach();
	Sensor_thread.detach();
	GCS_thread.detach();
	dcout("done\n");
	//:.:multithread initialization

	// get last operational parameter (failsafe)
	dcout(":Loading last parameters ... \n");
	ifstream lastOPParam(workPath + "logs/Operational-Parameter.txt");
	string _lastOPParam;
	getline(lastOPParam, _lastOPParam);
	lastOPParam.close();

	// set initial param
	gParam.Set(_lastOPParam);
	drun(gParam.Print());
	dcout("Setting initial parameter done\n");

	// get last operational waypoint (failsafe)
	dcout(":Loading last waypoints ... \n");
	ifstream lastOPWPoint(workPath + "logs/Operational-Waypoint.txt");
	string _lastOPWPoint;
	getline(lastOPWPoint, _lastOPWPoint);
	lastOPWPoint.close();

	// make new datalog for sensor based on date and time
	{
		time_t _time;
		struct tm *_timestamp;
		time(&_time);
		_timestamp = localtime(&_time);
		strftime(_buff, sizeof(_buff), "%d%m%Y-%H%M%S", _timestamp);
	}
	glider.datalogIsCreated = false;
	glider.datalogPath = ("sensor/" + Utils::convertToString(_buff));
	glider.isTransmitingData = false;

	dwatch(glider.datalogPath);

	// set initial waypoint
	gWPoint.Set(_lastOPWPoint);
	drun(gWPoint.Print());
	dcout("Setting initial Way point done\n");

	// clear buffer (temporary storage area)
	tcflush(GS.port, TCIOFLUSH);
	tcflush(BB.port, TCIOFLUSH);
	tcflush(AR1.port, TCIOFLUSH);
	tcflush(AR2.port, TCIOFLUSH);

	// activate multithread task
	GS.activateThread();
	BB.activateThread();
	// AR.activateThread();

	// set initial FSM state
	fsm.state = 0;
	fsm.lastState = 0;
	fsm.logIsCreated = false;

	int PINGcounterBB = 0; // buffer
	int PINGcounterAR = 0; // buffer
	int PINGcounterGS = 0; // buffer

	int PINGcounterLimit = 20; // constants
	int batteryThreshold = 0;  // constants

	// put BB AR on standby to get data from BB
	Utils::tcpWrite(BB.port, "#TS> OPERATION 2\r\n");
	// Utils::serWrite(AR.port, "#TS> OPERATION 2\r\n");

	long long int timeElapsed;	 // function
	long long int timeThreshold; // constants

	dcout("initialization completed, going to main loop\n");
	// glider pose = 2 because staystill is 2
	glider.pose = gliderConstants.staystill;

	//......................................................................................
	// main loop
	while (1)
	{
		// check for battery
		if (sensor.analog.volt < batteryThreshold)
		{
			dcout("Failed, Battery Volt is not safe to run" << sensor.analog.volt << "lower than" << batteryThreshold << "\n");
			// if state is already triggered by GS
			if (fsm.lowerState() < FAILSAFE && fsm.lowerState() > SETUP)
			{
				fsm.moveTo(LOWBATTERYFAILSAFE);
				// consumption request
				fsm.isWaitingSensorUpdate = true;
			}
			// else state is not yet triggered, failsafe is ignored
		}
		// checking FSM lowerState()
		dcout("FSM " << fsm.lowerState() << "\n");
		switch (fsm.lowerState())
		{
		// initial fsm state is 0, waiting is triggered first
		case WAITING:
			// Modified 18 Jan 2023
			// Reset state and first variable when Task Finished
			// calculateNavGuide();
			// getDataLog();
			if (state != -1 || first != true)
			{
				first = true;
				state = -1;
			}
			dcout("Masuk WAITING \n");
			BB.activateThread();
			if (manualInput == true)
			{
				userInputFSM();
			}
			break;

		case SETUP:
			dcout("Masuk SETUP \n");
			glider.datalogReady = false;

			// default GSdeactivate
			// GS.deactivateThread();
			// GS.thread = INNACTIVE;

			// set target according to waypoint number
			// fsm.targetCount = gWPoint.status.size() * BASE;
			// dcout("FSM target count: " << fsm.targetCount);
			// target completion requirement
			// fsm.targetCount += SINGLETARGETFINISH;
			// initial operation state
			// fsm.state = BASE;
			// calculate finished waypoints
			if (gWPoint.status.size() == 0)
			{
				dcout("WayPointStatus False \n");
				fsm.moveTo(WAITING);
				break;
			}
			// elseif(gWPoint.status.size() == 1)
			// {
			// 	dcout
			// }

			// Multi Way point
			// increase base operation according to finished waypoint
			// for (int i = 0; i < gWPoint.status.size(); i++)
			// {
			// 	if (gWPoint.status[i] == 1)
			// 		fsm.baseTo(+BASE);
			// 	// fsm.state += BASE;
			// }

			BB.activateThread();
			if (!overrideInterval)
			{
				char BBparam[300];
				sprintf(BBparam, "#TS> PARAM %d\r\n", gParam.interv);
				Utils::tcpWrite(BB.port, BBparam);
			}
			else
			{
				char BBparam[300];
				sprintf(BBparam, "#TS> PARAM %d\r\n", overrideIntervalValue);
				Utils::tcpWrite(BB.port, BBparam);
			}
			// wait ack param
			// change to PING for bypass, default val : WAITING
			BB.isACK = PING;

			// todo : sendparam to AR
			// wait ack param
			// AR.activateThread();
			// change to PING for bypass, default val : WAITING
			// AR.isACK = PING;

			// fsm.moveTo(SINGLETARGETSETUP);
			first = true;
			state = -1;
			fsm.moveTo(SINGLETARGETSETUP);
			// fsm.Print();
			break;
		case SINGLETARGETSETUP:
			dcout("Masuk SINGLETARGETSETUP \n");
			// check for reply from BB
			if (BB.isACK == PING || BB.isACK == OPERATION || overrideBBACK)
			{
				// check for reply from AR
				if (AR1.isACK == PING || AR1.isACK == OPERATION || overrideARACK)
				{
					// send start signal (all sensor active)
					Utils::tcpWrite(BB.port, "#TS> OPERATION 3\r\n");

					// send ACK start/RTO
					GS.sendACK();

					// set waypoint
					guidance.sourceLat = gWPoint.latStart[0];
					guidance.sourceLon = gWPoint.lonStart[0];
					guidance.targetLat = gWPoint.latEnd[0];
					guidance.targetLon = gWPoint.lonEnd[0];
					dcout("TargetLat" << guidance.targetLat);

					// guidance.sourceLat = gWPoint.latStart[fsm.upperState() - 1];
					// guidance.sourceLon = gWPoint.lonStart[fsm.upperState() - 1];
					// guidance.targetLat = gWPoint.latEnd[fsm.upperState() - 1];
					// guidance.targetLon = gWPoint.lonEnd[fsm.upperState() - 1];
					fsm.isWaitingSensorUpdate = true;

					glider.pose = gliderConstants.descending;
					// GS.deactivateThread();
					fsm.moveTo(SURFACING);
				}
				else
				{
					// send PING to AR
					// Utils::serWrite(AR.port, "#TS> OPERATION 1\r\n");
					PINGcounterAR++;
					if (PINGcounterAR > PINGcounterLimit)
						fsm.moveTo(HALT);
				}
			}
			else
			{
				// send PING to BB
				Utils::tcpWrite(BB.port, "#TS> OPERATION 1\r\n");
				PINGcounterBB++;
				if (PINGcounterBB > PINGcounterLimit)
					fsm.moveTo(HALT);
			}
			break;
		case GLIDING:
			dcout("Masuk GLIDING \n");
			dcout("Is Waiting sensor Update " << fsm.isWaitingSensorUpdate << "\n");
			if (fsm.isWaitingSensorUpdate)
			{
				// wait producer
				if (glider.sensorIsUpdated)
				{
					glider.sensorIsUpdated = false;
					fsm.isWaitingSensorUpdate = false;
				}
			}
			else
			// consume
			{
				// do normal glide operation
				glideOperation(0);

				if (guidance.isFinished && glider.pose == gliderConstants.staystill)
				{
					guidance.isFinished = false;
					glider.datalogReady = true;

					fsm.moveTo(GLIDECOMPLETE);
				}
				else
				{
					// ask producer
					fsm.isWaitingSensorUpdate = true;
				}
			}
			break;
		case GLIDECOMPLETE:
			dcout("Masuk GLIDINGCOMPLETE \n");
			// activate GS handler
			tcflush(GS.port, TCIOFLUSH);
			GS.activateThread();
			// put BB AR to standby
			Utils::tcpWrite(BB.port, "#TS> OPERATION 2\r\n");
			// Utils::serWrite(AR.port, "#TS> OPERATION 2\r\n");
			//  start timer
			timeStart = chrono::steady_clock::now();
			// reset PID
			pitch.Reset();
			heading.Reset();
			buoyancy.Reset();

			fsm.moveTo(WAITINSTRUCTION);

			break;
		case WAITINSTRUCTION:
			dcout("Masuk WAITINSTRUCTION \n");
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

			if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > 120)
			{
				// situation 1
				// full BB operation mode is needed to acquire waypoint
				Utils::tcpWrite(BB.port, "#TS> OPERATION 3\r\n");
				fsm.isWaitingSensorUpdate = true;
				tcflush(GS.port, TCIOFLUSH);

				// notify producer
				fsm.isWaitingSensorUpdate = true;

				// GS.activateThread();
				fsm.moveTo(SETUPRTB);
			}
			// situation 2,3, and 4 detected by GS handler
			// situation 2 will reset timeStart
			// situation 3 cause outcome A
			// situation 4 cause outcome B
			break;
			// return to base state
		case SETUPRTB:
			dcout("Masuk SETUPRTB \n");
			if (fsm.isWaitingSensorUpdate)
			{
				// wait producer
				if (glider.sensorIsUpdated)
				{
					glider.sensorIsUpdated = false;
					fsm.isWaitingSensorUpdate = false;
				}
			}
			else
			// consume
			{
				// RTB backtrack
				// set current position as source waypoint
				// set last source as target waypoint
				guidance.targetLat = gWPoint.latStart[fsm.upperState() - 1];
				guidance.targetLon = gWPoint.lonStart[fsm.upperState() - 1];
				guidance.sourceLat = sensor.IMU.latitude;
				guidance.sourceLon = sensor.IMU.longitude;

				// notify producer
				fsm.isWaitingSensorUpdate = true;
				glider.pose = gliderConstants.descending;
				fsm.moveTo(RTBOPERATION);
			}

			break;
		case RTBOPERATION:
			dcout("Masuk RTBOPERATION \n");
			if (fsm.isWaitingSensorUpdate)
			{
				// wait producer
				if (glider.sensorIsUpdated)
				{
					glider.sensorIsUpdated = false;
					fsm.isWaitingSensorUpdate = false;
				}
			}
			else
			// consume
			{
				// do normal glide operation
				glideOperation(0);
				// ask producer

				if (guidance.isFinished && glider.pose == gliderConstants.ascending)
				{
					guidance.isFinished = false;
					fsm.moveTo(MISSIONABORTED);
				}
				else
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

			if (fsm.upperState() == 1)
			{
				// all waypoints backtracked
				// fsm state should be BASE + MISSIONABORTED

				// set to waiting
				fsm.moveTo(WAITING);
			}
			else
			{
				// not all waypoints backtracked

				// continue backtrack
				fsm.baseTo(-BASE);
				// fsm.state -= BASE;
				fsm.moveTo(SETUPRTB);
			}

			break;
		case HALT:
			// disable further state
			// caused by internal system fail
			//(AR and or BB not responding)
			// log all
			// exit program
			// return 0;

			// ADDED in 19 Dec 2022 for testing
			//  HALT FOR ambil Data
			fsm.moveTo(SETUP);
			break;
		case SINGLETARGETFINISH:
			// check if another waypoint exist
			if (fsm.state == fsm.targetCount)
			{
				// all waypoint complete
				// fsm state should be ((number of waypoints) * BASE)+SINGLETARGETFINISH
				// set new state
				fsm.moveTo(WAITING);
			}
			else
			{
				// fsm state should ((number of waypoints travelled) * BASE)+SINGLETARGETFINISH

				// move to next operation target
				fsm.baseTo(+BASE);
				// fsm.state += BASE;
				// set new state
				fsm.moveTo(SINGLETARGETSETUP);
			}
			break;
		case CONTROL:
			dcout("Masuk Yaw Control \n");
			dcout(askingInput);
			// modified 14 June
			desYaw = (askingInput ? userInputDesired("YAW") : desYaw);
			calculateNavGuide();
			getDataLog();
			yawControl(withMainThruster);
			break;
		case GUIDANCE:
			dcout("Masuk Guidance (Ngotak) \n");
			guidanceTest();
			break;
		case MANUALGLIDE:
			dcout("Masuk Manual Glide \n");
			manualGliding();
			break;
		case PITCHCONTROL:
			dcout("Masuk Pitch Control");
			desPitch = (askingInput ? userInputDesired("PITCH") : desPitch);
			MM_input = controlPitch();
			getDataLog();
			moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
			sleep(1);
			break;
		case DEPTHCONTROL:
			dcout("Masuk Depth Control");
			desDepth = (askingInput ? userInputDesired("Depth") : desDepth);
			BE_input = controlDepth();
			moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
			getDataLog();
			usleep(500000);
			break;
		case ZIGZAG:
			dcout("Masuk Zig Zag \n");
			guidanceTestZigZag();
			break;
		case SURFACING:
			dcout("Masuk ke surfacing");
			// buat function surfacing
			surfacing();
			// yang akan membuat input main thruster 1400 dan controlling yaw
			break;
		case STOP: // STOP
			// change glider fsm to waiting
			resetActuator();
			if (fsm.lowerState() != WAITING)
				fsm.moveTo(WAITING);
			break;
		case TIMETHRESHOLDGLIDE:
			dcout("Masuk ke TimeThresHoldGLIDE");
			timeThresholdGlide();
			break;
		case ARDUDATACHECK:
			dcout("Checking arduino data");
			readDataArduino();
			manualInput=true;
			getDataLog();
			fsm.moveTo(WAITING);
		default:
			break;
		}
		// usleep(100000);
	}
	dcout("\n");
	return 0;
}

void FSM::baseTo(int _target)
{
	lastState = state;
	state = state + _target;
	dcout("Change UPPER FSM from " << lastState << " to " << state << "\n");
	if (!logIsCreated)
	{
		dcout("If 1 is executed");
		// clear operational log
		ofstream datalog(workPath + "logs/Operational-FSM.txt", std::ios::out | std::ios::trunc);
		datalog.close();
		logIsCreated = true;
	}
	if (logIsCreated)
	{
		// log
		dcout("If 2 is executed");
		char message[100];
		sprintf(message, "Change FSM from %d to %d", lastState, state);
		Utils::write2LOG("Operational-FSM", message, false);
		Utils::write2LOG("List-FSM", message, false);
	}
}

void FSM::moveTo(int _target)
{
	bool _isNegative = false;
	if (state < 0)
		_isNegative = true;

	int newstate = (int)(state / BASE) * BASE;
	lastState = state;
	state = newstate + _target;

	if (_isNegative)
		state *= -1;

	dcout("Change LOWER FSM from " << lastState << " to " << state << "\n");
	if (!logIsCreated)
	{
		// clear operational log
		ofstream datalog(workPath + "logs/Operational-FSM.txt", std::ios::out | std::ios::trunc);
		datalog.close();
		logIsCreated = true;
	}
	if (logIsCreated)
	{
		// log
		char message[100];
		sprintf(message, "Change FSM from %d to %d", lastState, state);
		Utils::write2LOG("Operational-FSM", message, false);
		Utils::write2LOG("List-FSM", message, false);
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

// added on december
int counterguidance = 0;
int counterlimit = 150;

int glideOperation(int override = 0)
{
	// get lock for multithread
	threadmutex.lock();

	// calculate guidance and navigation
	navigation = calculateNavigation(navigation, sensor);
	guidance = calculateGuidance(guidance, navigation.latitude, navigation.longitude, guidance.targetLat, guidance.targetLon);
	dcout("Calculating Navigation process\n ");
	cout << "Velocity Data North : " << navigation.vN_IMU << " East : " << navigation.vE_IMU << " Down : " << navigation.vD_IMU << endl;

	cout << "DVL_X : " << navigation.vX_DVL << " DVL_Y : " << navigation.vY_DVL << " DVL_Z : " << navigation.vZ_DVL << endl;

	cout << "Position Data North : " << navigation.N_aug << " East : " << navigation.E_aug << " Down : " << navigation.Depth_aug << endl;

	// dcout("INPUT SIGNAL: ---North: " << navigation.vN_IMU << " ---East: " << navigation.vE_IMU << " ---Down: " <<navigation.vD_IMU "\n");

	// logging
	char logMessageGlide[300];
	sprintf(logMessageGlide, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f", navigation.latitude, navigation.longitude,
			guidance.distanceFromTarget, guidance.psiRef,
			guidance.sourceLat, guidance.sourceLon,
			guidance.targetLat, guidance.targetLon);
	Utils::write2LOG("Glide-NavigationGuidance", logMessageGlide, false);

	//.:.added on  december
	counterguidance++;
	if (counterguidance > counterlimit)
	{
		guidance.isFinished = true;
		counterguidance = 0;
		counterlimit += 150;
	}

	//:.:added on  december

	// ascending
	if (glider.pose == gliderConstants.ascending)
	{
		if (sensor.ALTI.depth > gliderConstants.surfaceDepth || sensor.ALTI.depth > gliderConstants.surfaceThreshold)
		{
			// check guidance
			if (guidance.isFinished)
			{
				// cout <<"STAYSTILL"<<endl;
				glider.pose = gliderConstants.staystill;
				PIDOperation(gliderConstants.staystill);
				// stop glide operation
				threadmutex.unlock();
				return 1;
			}
			else
			{
				// cout <<"STAYSTILL"<<endl;
				// change to descending
				glider.pose = gliderConstants.descending;
				PIDOperation(gliderConstants.staystill);
			}
		}
		else
		{
			// continue ascending
			PIDOperation(gliderConstants.ascending);
		}
	}
	else
		// descending
		if (glider.pose == gliderConstants.descending)
		{
			if (sensor.ALTI.depth > gParam.depthOperation && sensor.ALTI.depth > gliderConstants.depthThreshold)
			{
				// continue descending
				PIDOperation(gliderConstants.descending);
				// check if glider have reached destination
				// change to ascending if glider if completed
				if (guidance.isFinished)
				{
					glider.pose = gliderConstants.ascending;
					PIDOperation(gliderConstants.staystill);
				}
			}
			else
			{
				// cout <<"STAYSTILL"<<endl;
				// change to ascending
				glider.pose = gliderConstants.ascending;
				PIDOperation(gliderConstants.staystill);
			}
		}
	threadmutex.unlock();
	return 0;
}

void PIDOperation(int glidestate)
{
	// data for arduino :
	// float bladder (Bouyancy Engine)
	// float none
	// float ballast (Moving Mass)
	// float teta (rudder)
	// float left right thruster
	// float main thruster
	// int strobe

	// bladder <- PID
	// ballast <- PID
	// rudder <- PID
	// teta <- surfacepitch/ascendpitch/descendpitch
	// pos_e <- data BB
	// pos_n <- data BB

	// yawref	<-  nav/guidance (?)
	// global heading <- nav/guidance (?)

	// pitchref <- teta / user
	// global pitch <- sensor BB

	// depthref <- GCS / user
	// globaldepth <- sensor BB

	//.:.ondevelopment
	float yawRefference = 135; // guidance.psiRef;
	float pitchRefference;
	//:.:ondevelopment

	// ascending
	if (glidestate == 1)
	{
		dcout("Going Ascend \n");
		// propeller off

		// teta <- ascendpitch
		pitchRefference = gliderConstants.ascendPitch;

		// strobe on
		glider.strobe = gliderConstants.strobe[INNACTIVE];

		// calculate PID for input signal
		heading = CalculatePID(heading, rudder, yawRefference, sensor.IMU.omegaHeading, true);
		pitch = CalculatePID(pitch, ballast, gliderConstants.ascendPitch, sensor.IMU.omegaPitch, false);
		buoyancy = CalculatePID(buoyancy, bladder, gliderConstants.surfaceDepth, sensor.ALTI.depth, false);
		dcout("Calculating PID done\n ");
		dcout("INPUT SIGNAL: ---Pitch: " << pitch.output << "---Heading: " << heading.output << "---Bouyancy: " << buoyancy.output << "\n");
	}
	else
		// descending
		if (glidestate == 0)
		{
			dcout("Going Descend\n");
			// propeller off

			// teta <- descendpitch
			pitchRefference = gliderConstants.descendPitch;

			// strobe off
			glider.strobe = gliderConstants.strobe[INNACTIVE];

			heading = CalculatePID(heading, rudder, yawRefference, sensor.IMU.omegaHeading, true);
			pitch = CalculatePID(pitch, ballast, gliderConstants.descendPitch, sensor.IMU.omegaPitch, false);
			buoyancy = CalculatePID(buoyancy, bladder, gParam.depthOperation, sensor.ALTI.depth, false);

			dcout("Calculating PID done\n");
			dcout("INPUT SIGNAL: ---Pitch: " << pitch.output << "---Heading: " << heading.output << "---Bouyancy: " << buoyancy.output << "\n");

			// pos_e ??
			// pos_n ??
		}
		else
			// stay still
			if (glidestate == 2)
			{
				dcout("Glider Stay Still\n");
				// propeller off
				// teta <- surfacepitch
				pitchRefference = gliderConstants.surfacePitch;

				// reset actuator
				//	heading.Reset();
				pitch.Reset();
				buoyancy.Reset();

				// pos_e ??
				// pos_n ??

				// strobe on
				glider.strobe = gliderConstants.strobe[ACTIVE];
				this_thread::sleep_for(chrono::milliseconds(2000));
			}
	char ARbuffer[100];
	sprintf(ARbuffer, "#TS %.6f %d %.6f %d %.6f %d %d\r\n", buoyancy.output, 0, pitch.output, 60, heading.output, 1200, 0);
	Utils::serWrite(AR1.port, ARbuffer);

	dcout(ARbuffer);
	dcout("\n");
	// logging
	char logMessagePID[300];
	// all actuator
	// header : yawRef yawNow yawErr yawOut ptcRef ptcNow ptcErr ptcOut buoRef buoNow buoErr buoOut strobe
	sprintf(logMessagePID, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d",
			heading.refference, heading.current, heading.error, heading.output,
			pitch.refference, pitch.current, pitch.error, pitch.output,
			buoyancy.refference, buoyancy.current, buoyancy.error, buoyancy.output,
			glider.strobe);
	Utils::write2LOG("PID-actuator", logMessagePID, false);
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
	// wait and skip handling if task_flag is not raised
	// handle incoming communication from GCS
	while (1)
	{
		if (!GS.thread)
		{
		}
		else
		{
			// blocking read serial
			char GCSinbound[300];
			memset(&GCSinbound, '\0', sizeof(GCSinbound));
			int bytes_read = read(GS.port, &GCSinbound, sizeof(GCSinbound));

			if (echoSerial)
				// cout << "GS RAW : " << GS.rawdata << endl;

				// check if thread is still active
				if (!GS.thread)
				{
				}
				else
				{
					GS.rawdata = Utils::convertToString(GCSinbound);

					// parse data based on rawdata
					// if valid, change values of
					// datasender, datacommand, dataargs, and datacode
					// return -1 on dataCode if data is not found within defaultmessage
					Utils::Parse(GS);
					// dwatch(GS.rawdata);
					// dwatch(GS.dataSender);
					// dwatch(GS.dataCommand);
					// dwatch(GS.dataArgs);
					// dwatch(GS.dataCode);

					switch (GS.dataCode)
					{
					case AT:

						break;
					case RTB:
						GS.sendACK();
						if (fsm.lowerState() != SETUPRTB)
							fsm.moveTo(SETUPRTB);
						break;
					case START:
						// dcout("STARTING WITH : \n");
						// drun(gParam.Print());
						// drun(gWPoint.Print());
						// trigger FSM
						fsm.state = SETUP;
						break;
					case PARAM:
						if (gParam.isValid(GS.rawdata, 17))
						{
							dcout("Go to if statement param");
							gParam.Set(GS.rawdata);
							dcout(GS.rawdata);
							// manipulates glider const according to param
							// param2constants(gParam, gliderConstants);

							// drun(gParam.Print());

							// use contiguous storage of std::string to convert to char *
							// rewrite operational log
							// Utils::write2LOG("Operational-Parameter", &*GS.rawdata.begin(), true);
							// append log
							// Utils::write2LOG("List-Parameter", &*GS.dataArgs.begin(), false);
							// dcout("Sini yokkk\n");
							// GS.sendACK();
						}
						else
						{
							// invalid data arguments
							// log error
							dcout("Going to Else");
							dcout(GS.rawdata);
							std::string _str = GS.rawdata.erase(0, 11);
							desYaw = std::stof(_str);
							dcout("Des Yaw string: " << _str << "\n");
							dcout("Des Yaw float: " << desYaw << "\n");
							fsm.moveTo(CONTROL);
							// Utils::write2LOG("Error-Parameter", &*GS.dataArgs.begin(), false);
						}
						break;
					case WAYPOINT:
						if (gWPoint.isValid(GS.rawdata))
						{
							gWPoint.Set(GS.rawdata);
							// drun(gWPoint.Print());

							// use contiguous storage of std::string to convert to char *
							// rewrite operational log
							Utils::write2LOG("Operational-Waypoint", &*GS.rawdata.begin(), true);
							// append log
							Utils::write2LOG("List-Waypoint", &*GS.dataArgs.begin(), false);

							GS.sendACK();
						}
						else
						{
							// invalid data arguments
							// log error
							Utils::write2LOG("Error-Waypoint", &*GS.dataArgs.begin(), false);
						}
						break;
					case PING: // PING
						timeNow = chrono::steady_clock::now();
						GS.sendACK();
						dcout("PING dari GCS Masuk\n");
						break;
					case DATA: // DATA
						if (glider.datalogReady || datalogAlwaysReady)
						{
							threadmutex.lock();
							// glider.isTransmitingData = true;
							sendDatalog();
							// let GCS know that all data is sent
							Utils::serWrite(GS.port, "#TS> DATA END\r\n");
							// wait for next glide operation
							glider.datalogReady = false;
							// glider.isTransmitingData = false;
							threadmutex.unlock();
						}
						break;
					case STOP: // STOP
						// change glider fsm to waiting
						resetActuator();
						GS.sendACK();
						if (fsm.lowerState() != WAITING)
							fsm.moveTo(WAITING);

						break;
					case OPERATION:
						break;
					case RTO: // RTO OR CTM
					case CTM:
						GS.sendACK();
						if (fsm.lowerState() == WAITINSTRUCTION)
							fsm.moveTo(SINGLETARGETFINISH);
						break;
					case SELFTEST:
						dcout("Masuk ke Self Test \n");
						if (gParam.isValid(GS.rawdata, 5))
						{
							Utils::GliderUtility gliderUtils;
							gliderUtils.parsingData(GS.rawdata, &BE_input, &MM_input, &rudder_input, &yaw_input, &main_input);
							selfTest();
						}
						else
						{
							dcout("Masukkin 5 Data Bos");
						}
						break;
					case PITCHCONTROL:
						dcout("Masuk ke PITCHCONTROL \n");
						try
						{
							dcout(GS.rawdata);
							std::string _str = GS.rawdata.erase(0, 18);
							desPitch = std::stof(_str);
							dcout("Des pitch float: " << desPitch << "\n");
						}
						catch (std::invalid_argument)
						{
							dcout("Failed to make set desPitch, going to default descend pitch");
							desPitch = gliderConstants.descendPitch;
						}
						fsm.moveTo(PITCHCONTROL);
						askingInput = false;
						break;
					case DEPTHCONTROL:
						dcout("Masuk ke DepthControl \n");
						try
						{
							dcout(GS.rawdata);
							std::string _str = GS.rawdata.erase(0, 18);
							desDepth = std::stof(_str);
							dcout("Des pitch float: " << desDepth << "\n");
						}
						catch (std::invalid_argument)
						{
							dcout("Failed to make set desPitch, going to default depth: 0");
							desDepth = 0;
						}
						askingInput = false;
						fsm.moveTo(DEPTHCONTROL);
						break;
					case NGOTAK:
						dcout("MASUK NGOTAK \n");
						fsm.moveTo(GUIDANCE);
						break;
					case MANUALGLIDE:
						if (true)
						{
							dcout("MASUK Manual Glide \n");
							try
							{
								dcout(GS.rawdata);
								std::string _str = GS.rawdata.erase(0, 17);
								desYaw = std::stof(_str);
								dcout("Des Yaw string: " << _str << "\n");
								dcout("Des Yaw float: " << desYaw << "\n");
								withYawControl = true;
							}
							catch (std::invalid_argument)
							{
								dcout("Failed to make set DesYaw");
								withYawControl = false;
								first = 0;
							}
							fsm.moveTo(MANUALGLIDE);
						}
					case YAWCONTROL:
						if (true)
						{
							dcout("MASUK YAWCONTROL \n");
							try
							{
								dcout(GS.rawdata);
								std::string _str = GS.rawdata.erase(0, 16);
								std::stringstream ss(_str);
								if (!(ss >> desYaw))
								{
									desYaw = 0;
								}
								if (!(ss >> withMainThruster))
								{
									withMainThruster = false;
								}
								dcout("Des Yaw float: " << desYaw << "\n");
								dcout("With Main Thruster? " << withMainThruster << "\n");
								withYawControl = true;
							}
							catch (std::invalid_argument)
							{
								dcout("Failed to make set DesYaw");
								withYawControl = false;
							}
							askingInput = false;
							fsm.moveTo(CONTROL);
						}
					case TIMETHRESHOLDGLIDE:
						dcout("MASUK GLIDING TIME THRESHOLD \n");
						fsm.moveTo(TIMETHRESHOLDGLIDE);
						break;
					default: // invalid data
						tcflush(GS.port, TCIOFLUSH);
						break;
					} // switch case based on datacommand
				}
		}
		usleep(10000);
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
	// cout << "Masuk Sensor Task " << endl;
	// wait and skip handling if task_flag is not raised
	// handle incoming communication from GCS
	while (1)
	{
		// cout << "Masuk Sensor Task " << endl;
		if (!BB.thread)
		{
		}
		else
		{
			// blocking read serial
			char BBinbound[1000];
			memset(&BBinbound, '\0', sizeof(BBinbound));
			int bytes_read = recv(BB.port, &BBinbound, sizeof(BBinbound), 0);
			// handling recieved data
			if (bytes_read > 0)
			{
				std::string receivedData(BBinbound, bytes_read);
				std::cout << "Recieved Data: " << receivedData << std::endl;
				BB.rawdata = receivedData;
			}
			if (echoSerial)
				// cout << "BB RAW : " << BB.rawdata << endl;
			// check if thread is still active
			if (!BB.thread)
			{
			}
			else
			{
				// cout << "BB RAW : " << BB.rawdata << endl;
				// parse data based on rawdata
				// if valid, change values of
				// datasender, datacommand, dataargs, and datacode
				// return -1 on dataCode if data is not found within defaultmessage
				Utils::Parse(BB);
				// dwatch(BB.rawdata);
				//  dwatch(BB.dataSender);
				//  dwatch(BB.dataCommand);
				//  dwatch(BB.dataArgs);
				//  dwatch(BB.dataCode);
				//  dwatch(BB.isACK);

				switch (BB.dataCode)
				{
				case AT: // AT

					break;
				case RTB: // RTB

					break;
				case START: // START

					break;
				case PARAM: // PARAM

					break;
				case WAYPOINT: // WAYPOINT

					break;
				case PING: // PING
					BB.sendACK();
					break;
				case DATA: // DATA
					// get lock to avoid data update while gliding
					threadmutex.lock();
					// separate args
					// dcout(BB.dataArgs);
					sensor.Parse(BB.dataArgs);
					// sensor.Print(sensorIMU | sensorDVL | sensorALTI | sensorMINICT);
					// | sensorDVL | sensorALTI

					// write sensor to log
					// create new datalog if not exist
					if (!glider.datalogIsCreated)
					{
						ofstream datalog(workPath + "logs/" + glider.datalogPath + ".txt", std::ios::out | std::ios::app);
						datalog.close();
						glider.datalogIsCreated = true;
					}
					// datalog exist
					// append sensor data
					if (glider.datalogIsCreated && glider.pose != gliderConstants.staystill) //&& !glider.isTransmitingData)
					{
						char buffer[300];
						// todo (get board temperature)
						glider.boardTemp = 30;
						// measure speed
						glider.speed = sqrt(pow(sensor.DVL.veloX_F, 2) + pow(sensor.DVL.veloY_F, 2) + pow(sensor.DVL.veloZ_F, 2));

						// write to log
						sprintf(buffer, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d %.6f %.6f %.6f %.6f",
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
								glider.boardTemp);
						Utils::write2LOG(&*glider.datalogPath.begin(), buffer, false);
					}

					// feed consumer
					if (fsm.isWaitingSensorUpdate)
						glider.sensorIsUpdated = true;
					// release lock
					threadmutex.unlock();
					break;
				case STOP: // STOP

					break;
				case OPERATION: // OPERATION
					break;

				default: // invalid data
					// try to flush I/O (not tested)
					tcflush(BB.port, TCIOFLUSH);
					break;
				} // switch case based on datacommand
			}
		}
		// sleep(1);
		usleep(500000);
		tcflush(BB.port, TCIOFLUSH);
	}
}

void Actuator_task()
{
	// wait and skip handling if task_flag is not raised
	// handle incoming communication from GCS
	/*
	while (1)
	{
		if (!AR1.thread)
		{
			dcout("AR THREAD NOT RUNNING");
		}
		else
		{
			// dcout("AR THREAD RUNNING");
			// blocking read serial
			char ARinbound[1000];
			memset(&ARinbound, '\0', sizeof(ARinbound));
			int bytes_read = read(AR.port, &ARinbound, sizeof(ARinbound));
			// AR.rawdata = Utils::convertToString(ARinbound);
			// cout << "AR RAW : " << AR.rawdata << endl;
			// if (echoSerial)
			// 	cout << "AR RAW : " << AR.rawdata << endl;

			if (!AR.thread)
			{
				dcout("AR thread is not active");
			}
			else
			{
				AR.rawdata = Utils::convertToString(ARinbound);
				// parseData(AR.rawdata, BE_input, MM_input);
				// dcout(BE_input);
				// dcout(MM_input);
				// parse data based on rawdata

				// if valid, change values of
				// datasender, datacommand, dataargs, and datacode
				// return -1 on dataCode if data is not found within defaultmessage
				// Utils::Parse(AR);
				// dwatch(AR.dataSender);
				// dwatch(AR.dataCommand);
				// dwatch(AR.dataArgs);
				// dwatch(AR.dataCode);

				switch (AR.dataCode)
				{
				case AT: // AT

					break;
				case RTB: // RTB

					break;
				case START: // START

					break;
				case PARAM: // PARAM

					break;
				case WAYPOINT: // WAYPOINT

					break;
				case PING: // PING

					break;
				case DATA: // DATA

					break;
				case STOP: // STOP

					break;
				case OPERATION: // OPERATION

					break;

				default: // invalid data
					// try to flush I/O (not tested)
					tcflush(AR1.port, TCIOFLUSH);
					break;
				} // switch case based on datacommand
			}
		}
		usleep(100000);
	}
	*/
}

void sendDatalog()
{
	// deactivate threads
	GS.deactivateThread();
	BB.deactivateThread();
	// AR1.deactivateThread();

	// open log
	ifstream datalog(workPath + "logs/" + glider.datalogPath + ".txt");

	// skip sent lines
	for (int i = 0; i < glider.datalogCount; i++)
	{
		string buffer;
		getline(datalog, buffer);
	}
	chrono::steady_clock::time_point begin = chrono::steady_clock::now();
	// send while condition is good and not EOF
	while ((datalog.good()) && (datalog.peek() != EOF))
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

		getline(datalog, __date, ' ');
		getline(datalog, __time, '\t');

		getline(datalog, __temperature, ' ');
		getline(datalog, __salinity, ' ');
		getline(datalog, __depth, ' ');
		getline(datalog, __latitude, ' ');
		getline(datalog, __longitude, ' ');
		getline(datalog, __roll, ' ');
		getline(datalog, __pitch, ' ');
		getline(datalog, __yaw, ' ');
		getline(datalog, __motion, ' ');
		getline(datalog, __battery, ' ');
		getline(datalog, __leakage, ' ');
		getline(datalog, __speed, ' ');
		getline(datalog, __boardTemp, '\n');

		char buffer[300];

		strcpy(buffer, ("#TS> DATA " +
						__date + " " +
						__time + " " +
						__temperature + " " +
						__salinity + " " +
						__depth + " " +
						__latitude + " " +
						__longitude + " " +
						__roll + " " +
						__pitch + " " +
						__yaw + " " +
						__motion + " " +
						__battery + " " +
						__leakage + " " +
						__speed + " " +
						__boardTemp +
						"\r\n")
						   .c_str());
		dcout("Isi buffernya: " << buffer);
		// output example
		// #TS> DATA 01-10-2020 09:05:34 16.000000 17.000000 15.000000 10.000000 11.000000 6.000000 4.000000 5.000000 0 1.000000 3.000000 22.561028 30.000000
		Utils::serWrite(GS.port, buffer);
		glider.datalogCount++;
	}

	// make sure output buffer is empty
	tcdrain(GS.port);
	cout << "DATALOGCOUNT " << glider.datalogCount;
	long long int timeElapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(chrono::steady_clock::now() - begin).count();
	cout << " COMPLETIONTIME(ns): " << timeElapsed << endl;
	// close file
	datalog.close();

	// reactivate threads
	GS.activateThread();
	// Ardu is not using a routine thread
	// AR.activateThread();
	BB.activateThread();
}

/*
commented out to minimized cpu usage
void debug_override_task()
{
	// thread is destroyed if DEBUGTOOLS is OFF
	if (!DEBUGTOOLS)
	{
	}
	else
		// wait and skip handling if task_flag is not raised
		// handle incoming communication from GCS
		while (1)
		{
			if (!debugOverride.thread)
			{
			}
			else
			{
				// blocking read serial
				char DebInbound[1000];
				memset(&DebInbound, '\0', sizeof(DebInbound));
				int bytes_read = read(debugOverride.port, &DebInbound, sizeof(DebInbound));
				debugOverride.rawdata = Utils::convertToString(DebInbound);

				if (echoSerial)
					cout << "DP RAW : " << debugOverride.rawdata << endl;

				// check if thread is still active
				if (!debugOverride.thread)
				{
				}
				else
				{
					dwatch(debugOverride.rawdata);
					stringstream sentence(debugOverride.rawdata);
					string variable;
					sentence >> variable;
					string value;
					sentence >> value;
					cout << "override : " << variable << "  val : " << value << endl;

					if (variable == "guidanceisfinished")
					{
						if (value == "true" || value == "t" || value == "1")
							guidance.isFinished = true;
						else if (value == "false" || value == "f" || value == "0")
							guidance.isFinished = false;
					}
					else if (variable == "bbalwaysack")
					{
						if (value == "true" || value == "t" || value == "1")
							overrideBBACK = true;
						else if (value == "false" || value == "f" || value == "0")
							overrideBBACK = false;
					}
					else if (variable == "aralwaysack")
					{
						if (value == "true" || value == "t" || value == "1")
							overrideARACK = true;
						else if (value == "false" || value == "f" || value == "0")
							overrideARACK = false;
					}
					else if (variable == "datalogalwaysready")
					{
						if (value == "true" || value == "t" || value == "1")
							datalogAlwaysReady = true;
						else if (value == "false" || value == "f" || value == "0")
							datalogAlwaysReady = false;
					}
					else if (variable == "setdatalog")
					{
						glider.datalogPath = (workPath + "sensor/" + value);
						glider.datalogIsCreated = true;
					}
					else if (variable == "bbinterval")
					{
						overrideInterval = true;
						overrideIntervalValue = stoi(value);
					}
					else if (variable == "datalogcount")
					{
						glider.datalogCount = stoi(value);
					}
					else if (variable == "arthread")
					{
						if (value == "true" || value == "t" || value == "1")
							AR.activateThread();
						else if (value == "false" || value == "f" || value == "0")
							AR.deactivateThread();
					}
					else if (variable == "bbthread")
					{
						if (value == "true" || value == "t" || value == "1")
							BB.activateThread();
						else if (value == "false" || value == "f" || value == "0")
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
*/