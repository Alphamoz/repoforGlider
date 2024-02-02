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
#include <fcntl.h>	 // Contains file controls like O_RDWR
#include <errno.h>	 // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>	 // write(), read(), close()
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
PID depth{"depth"};
PID veloDepth{"veloDepth"};

PIDconst rudder{"rudder"};
PIDconst ballast{"ballast"};
PIDconst bladder{"bladder"};

gliderParam gParam;
gliderWP gWPoint;

Utils::Communication GS, AR, BB, debugOverride;

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
bool datalogAlwaysReady = false;

bool echoSerial = false;
float desYaw = 0.0;
float desPitch = 0.0;
float desDepth = 0.0;

//:.:override debugging variables
bool first = true;
int state = -1;
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
	char ARbuffer[100];
	dcout("INPUT SIGNAL: --BE:" << BE << " --MM:" << MM << " --Rudder:" << rudder << " --yawThruster:" << yawThruster << " --mainThruster" << mainThruster << "\n");
	sprintf(ARbuffer, "#TS %d %d %d %d %d %d %d\r\n", BE, 0, MM, rudder, yawThruster, mainThruster, 0);
	Utils::serWrite(AR.port, ARbuffer);
}

void resetActuator()
{
	dcout("RESETTING ACTUATOR \n");
	resetInputVariable();
	moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
}

void getDataLog()
{
	char logMessageGlide[300];
	sprintf(logMessageGlide, "%d %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f",
			navigation.Status_GPS, navigation.latitude_init, navigation.longitude_init,
			navigation.latitude, navigation.longitude,
			navigation.vX_DVL, navigation.vY_DVL, navigation.vZ_DVL,
			navigation.E_aug, navigation.N_aug, navigation.Depth_aug,
			guidance.distanceFromTarget, guidance.psiRef,
			guidance.sourceLat, guidance.sourceLon,
			guidance.targetLat, guidance.targetLon);
	Utils::write2LOG("Glide-NavigationGuidance", logMessageGlide, false);

	char logMessagePID[300];
	sprintf(logMessagePID, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d",
			heading.refference, heading.current, heading.error, heading.output,
			pitch.refference, pitch.current, pitch.error, pitch.output,
			buoyancy.refference, buoyancy.current, buoyancy.error, buoyancy.output,
			glider.strobe);
	Utils::write2LOG("PID-actuator", logMessagePID, false);

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

void printDataSensor()
{
	navigation = calculateNavigation(navigation, sensor);
	guidance = calculateGuidance(guidance, navigation.latitude, navigation.longitude, guidance.targetLat, guidance.targetLon);
	desYaw = guidance.psiRef;

	dcout("Calculating Navigation process\n ");

	cout << "GPS_Status : " << navigation.Status_GPS << " Latitude : " << navigation.latitude << " Longitude : " << navigation.longitude << " Heading_IMU : " << navigation.Heading_IMU << endl;

	cout << "DVL_X : " << navigation.vX_DVL << " DVL_Y : " << navigation.vY_DVL << " DVL_Z : " << navigation.vZ_DVL << endl;

	cout << "Position Data North : " << navigation.N_aug << " East : " << navigation.E_aug << " Depth : " << navigation.Depth_aug << endl;

	cout << "Velocity Data North : " << navigation.vN_IMU << " East : " << navigation.vE_IMU << " Down : " << navigation.vD_IMU << endl;

	cout << "DVL_X : " << navigation.vX_DVL << " DVL_Y : " << navigation.vY_DVL << " DVL_Z : " << navigation.vZ_DVL << endl;

	cout << "Position Data North : " << navigation.N_aug << " East : " << navigation.E_aug << " Down : " << navigation.Depth_aug << endl;
	dcout("SENSOR DATA IMU: ---Pitch: " << sensor.IMU.omegaPitch << "\n");
	dcout("SENSOR DATA IMU: ---Lat: " << sensor.IMU.latitude << "\n");
}
int controlYaw(bool forward = false)
{
	dcout("Forward Value is: " << forward << "\n");
	heading = CalculatePID(heading, rudder, desYaw, sensor.IMU.omegaHeading, false);
	dcout("Desired Yaw: " << desYaw << "\n");
	dcout("Calculating PID done\n ");
	// dcout("SENSOR DATA IMU: ---Pitch: " << sensor.IMU.omegaPitch << "---Heading: " << sensor.IMU.omegaHeading << "---ROLL: " << sensor.IMU.omegaRoll << "\n");
	// dcout("SENSOR DATA IMU: ---Lat: " << sensor.IMU.latitude << "---Long: " << sensor.IMU.longitude << "---Status: " << sensor.IMU.Status << "\n");
	// dcout("SENSOR DATA DVL: ---Vx: " << sensor.DVL.veloX_F << "---Vy: " << sensor.DVL.veloY_F << "---Vz: " << sensor.DVL.veloZ_F << "\n");
	dcout("INPUT SIGNAL: ---Yaw: " << heading.output << "\n");
	if (forward == true)
	{
		main_input = 1400;
	}
	else
	{
		main_input = 1200;
	}
	printDataSensor();
	return heading.output;

	//	cout << "DVL_X : " << navigation.vX_DVL << " DVL_Y : " << navigation.vY_DVL << " DVL_Z : " << navigation.vZ_DVL << endl;

	//	cout << "Position Data North : " << navigation.N_aug << " East : " << navigation.E_aug << " Depth : " << navigation.Depth_aug << endl;

	//	cout << "Velocity Data North : " << navigation.vN_IMU << " East : " << navigation.vE_IMU << " Down : " << navigation.vD_IMU << endl;

	//	cout << "DVL_X : " << navigation.vX_DVL << " DVL_Y : " << navigation.vY_DVL << " DVL_Z : " << navigation.vZ_DVL << endl;

	//	cout << "Position Data North : " << navigation.N_aug << " East : " << navigation.E_aug << " Down : " << navigation.Depth_aug << endl;
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

// added 25 Des 2023
// *selfTest
void selfTest()
{
	dcout("MASUK KE SELFTEST \n");
	dcout("dengan param:" << BE_input << " " << MM_input << " " << rudder_input << " " << yaw_input << " " << main_input << "\n");
	moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);

	// // Rudder Test
	// dcout("Test Rudder \n");
	// resetInputVariable();
	// moveActuator(BE_input, MM_input, rudder_input + 20, yaw_input, main_input);
	// sleep(1);
	// moveActuator(BE_input, MM_input, rudder_input - 20, yaw_input, main_input);
	// sleep(1);
	// dcout("Test Rudder Finished \n");

	// sleep(1);

	// dcout("Test Thruster Main \n");
	// moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input + 50);
	// sleep(2);
	// dcout("Test Thruster Main Finished \n");

	// sleep(2);

	// dcout("Test Side Thruster \n");
	// moveActuator(BE_input, MM_input, rudder_input, yaw_input + 50, main_input);
	// sleep(2);
	// dcout("Test Thruster Side Finished \n");

	// sleep(2);

	// dcout("Test Moving Mass \n Please watch the glider movement should be pitch down or listen the stepper sound \n");
	// moveActuator(BE_input, MM_input + 20, rudder_input, yaw_input, main_input);
	// sleep(20);

	// dcout("Test Bouyancy Engine \n Please watch the glider movement or listen the BE sound \n");
	// moveActuator(BE_input + 200, MM_input, rudder_input, yaw_input, main_input);
	// sleep(20);

	// dcout("Going to Normal Position \n");
	// moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
	// // sleep(20);

	// dcout("SelfTest Finished");
	// sleep(2);
}

float pitchNow = 0;
int lastInputPitch = MM_input;
void pitchMechanism()
{
	if (MM_input > lastInputPitch)
	{
		// pitchUp
		pitchNow += 0.5;
	}
	else if (MM_input < lastInputPitch)
	{
		pitchNow -= 0.5;
	}
	lastInputPitch = MM_input;
}

int controlPitch()
{
	float setPoint = desPitch;
	dcout("SETPOINT PITCH: " << setPoint << "\n");
	float pitchError = pitchNow - setPoint;
	dcout("PitchNow: " << sensor.IMU.omegaPitch << "\n");
	dcout("PitchNow: " << pitchNow << "\n");
	dcout("PITCH ERROR: " << pitchError << "\n");
	if (fabs(pitchError) > 2)
	{
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
	pitchMechanism();
	return MM_input;
}

float percepatan;
float kecepatan;
float vLast;
int t = 1;

void depthMechanism()
{
	// asumsi 400 ml normal.
	// jika bertambah volumenya
	// F akan negatif, yaitu ke atas.
	int BE_Now = BE_input;
	while (true)
	{
		/* code */
		if (BE_Now > BE_input)
		{
			BE_Now -= 10;
		}
		else if (BE_Now < BE_input)
		{
			BE_Now += 10;
		}
		float F = 1 * 9.8 * (BE_Now - 400) / 1000;
		float a = F / 100;
		dcout("Acceleration Now: " << a << "\n");
		float velo = vLast + a * t;
		dcout("Velo Now: " << velo << "\n");
		vLast = velo;
		// after t time
		depthNow = lastDepth + velo * t;
		lastDepth = depthNow;
		// if(depthNow<0){
		// 	depthNow = 0;
		// 	velo = 0;
		// 	vLast = 0;
		// }
		sleep(1);
		dcout("Depth Mechanism is working \n");
		dcout("BE_Now: " << BE_Now << "\n");
		dcout("DepthNow: " << depthNow << "\n");
	}
}
float veloDes = 0.05;
float lastCalcDepth = depthNow;
float lastCalcVelo = 0;
int controlDepth()
{
	// Modified 18 Jan 2023
	float setPoint = desDepth;
	float output = 0;
	// float depthNow = sensor.ALTI.depth;
	int timeDiff = 3;
	dcout("Depth Now: " << depthNow << "\n");
	dcout("Last Depth: " << lastCalcDepth << "\n");
	float velocityDepth = (depthNow - lastCalcDepth) / timeDiff;
	float calcAcceleration = (velocityDepth - lastCalcVelo) / timeDiff;
	dcout("Velocity Depth: " << velocityDepth << "\n");
	dcout("SET POINT Depth: " << setPoint << "\n");

	int velocityPID = 0;
	int veloError = velocityDepth - veloDes;
	// kalau
	// if (fabs(depthNow-setPoint) < 1 || depthNow > setPoint)
	// {
	// 	veloDepth = CalculatePID(veloDepth, bladder, 0, velocityDepth, false);
	// 	velocityPID = veloDepth.output * 30;
	// 	dcout("Velocity PID: " << velocityPID << "\n");
	// }
	// else if (fabs(depthNow-setPoint) > 1)
	// {
	// 	veloDepth = CalculatePID(veloDepth, bladder, veloDes, velocityDepth, false);
	// 	velocityPID = veloDepth.output * 30;
	// 	dcout("Velocity PID: " << velocityPID << "\n");
	// }

	if ((desDepth - depthNow) > 0)
	{
		veloDes = 0.02;
	}
	if ((desDepth - depthNow) < 0)
	{
		veloDes = -0.02;
	}
	if (fabs(desDepth - depthNow) < 1)
	{
		veloDes = 0;
	}

	veloDepth = CalculatePID(veloDepth, bladder, veloDes, velocityDepth, false);
	output = veloDepth.output;

	// dcout("Positional PID: " << positionalPID << "\n");
	dcout("Calculating PID for Depth done \n ");
	dcout("output dari depthControl: " << output << "\n");
	BE_input = BE_input + output;
	if (BE_input > bladder.saturation_upper)
	{
		BE_input = bladder.saturation_upper;
	}
	else if (BE_input < bladder.saturation_lower)
	{
		BE_input = bladder.saturation_lower;
	}
	dcout("INPUT SIGNAL: ---Bouyancy Engine: " << BE_input << "\n");
	lastCalcDepth = depthNow;
	lastCalcVelo = velocityDepth;
	dcout("DepthNow for Last Depth:" << lastCalcDepth << "\n");
	return output;
}

void manualGliding()
{
	// std::cin>>depthNow;
	int Pinit = 1; // slightly pitchdown
	int P0 = 2;	   // pitchdown
	int P1 = 1000; // go down
	int P2 = 1;	   // pitchup
	int P3 = 200;  // go up
	int P4 = 30;   // slightly pitchdown
	int P5 = 1;
	int P6;
	if (first)
	{
		timeStart = chrono::steady_clock::now();
		first = false;
		desPitch = -5;
		desDepth = 0;
		dcout("Normal Position");
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
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P0 && state == 0)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desDepth = gliderConstants.depthThreshold;
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P1 && state == 1)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desPitch = gliderConstants.ascendPitch;
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P2 && state == 2)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desDepth = gliderConstants.surfaceDepth;
		// MM_input = 80;
		// BE_input = 200;
		// yaw_input = 1500;
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P3 && state == 3)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		desPitch = gliderConstants.descendPitch;
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P4 && state == 4)
	{
		state++;
		timeStart = chrono::steady_clock::now();
	}
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P5 && state == 5)
	{
		state++;
		timeStart = chrono::steady_clock::now();
		dcout("End Of ManualGlide");
		fsm.moveTo(WAITING);
		return;
	}
	if (state == -1)
	{
		dcout("Preparing stage. StayStill, Pitching down 5deg \n");
	}
	else if (state == 0)
	{
		dcout("Pitching Down, 15deg \n");
	}
	else if (state == 1)
	{
		dcout("Going Down \n");
	}
	else if (state == 2)
	{
		dcout("Staying Still, pitching up 15deg \n");
	}
	else if (state == 3)
	{
		dcout("Going up \n");
	}
	else if (state == 4)
	{
		dcout("Gliding Finished, pitch to 5 deg.");
	}
	MM_input = controlPitch();
	BE_input = BE_input + controlDepth();
	if (BE_input > bladder.saturation_upper)
	{
		BE_input = bladder.saturation_upper;
	}
	else if (BE_input < bladder.saturation_lower)
	{
		BE_input = bladder.saturation_lower;
	}
	yaw_input = controlYaw();
	moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
	// getDataLog();
	sleep(3);
}
void guidanceTest()
{
	int Pinit = 18;
	int P0 = 43;
	int P1 = 48;
	int P2 = 48;
	int P3 = 23;
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
	yaw_input = controlYaw(true);
	MM_input = controlPitch();
	moveActuator(BE_input, MM_input, rudder_input, yaw_input, main_input);
	if (std::chrono::duration_cast<std::chrono::seconds>(timeNow - timeStart).count() > P3 && state == 3)
	{
		state++;
		resetActuator();
		fsm.moveTo(WAITING);
		return;
	}
}

void userInputGuidanceParam()
{
	cout << "Setting Param for Guidance" << endl;
	while (std::cout << "Masukkin Lat Target\n" && !(std::cin >> guidance.targetLat))
	{
		std::cin.clear();													// clear bad input flag
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard input
		std::cout << "Masukkin angka dong!.\n";
		std::cout << "tadi masukkinnya" << guidance.targetLat;
	}
	while (std::cout << "Masukkin Long Target\n" && !(std::cin >> guidance.targetLon))
	{
		std::cin.clear();													// clear bad input flag
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard input
		std::cout << "Masukkin angka dong!.\n";
		std::cout << "tadi masukkinnya" << guidance.targetLat;
	}

	dcout("Setting guidance Lat to " << guidance.targetLat << " \n\n");
	dcout("Setting guidance Lon to " << guidance.targetLon << " \n\n");
	sleep(1);
}

void userInputFSM()
{
	cout << "Lewat Radio		=   0,\nSTOP			=   8,\nMANUALGLIDING		=  17,	\nWAITINSTRUCTION		=  19,\nYAW CONTROL		=  21,\nGUIDANCE		=  22,\nHALT			=  94,\nSetting Param Lat Long	=999" << endl;
	while (std::cout << "Mau FSM berapa bos?\n" && !(std::cin >> FSM_status))
	{
		std::cin.clear();													// clear bad input flag
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard input
		std::cout << "Masukkin angka dong!.\n";
		// std::cout << "tadi masukkinnya" << FSM_status;
	}
	if ((FSM_status == 0 || FSM_status == 8 || FSM_status == 17 || FSM_status == 19 || FSM_status == 21 || FSM_status == 22 || FSM_status == 94 || FSM_status == 15 || FSM_status == 999))
	{
		if (FSM_status == 0)
		{
			dcout("Monggo lewat radio \n");
			fsm.moveTo(WAITING);
			manualInput = false;
			return;
		}
		if (FSM_status == 999)
		{
			// Setting Guidance Param
			userInputGuidanceParam();
			return;
		}
		fsm.moveTo(FSM_status);
		return;
	}
	dcout("Belum bisa ganti FSM " << FSM_status << " Coba Lagi! \n\n");
	sleep(1);
	userInputFSM();
}

void Gliding()
{
	// int isPitch = true;
	// int isMovingMassActive = false;

	// // glidingDown
	// while (sensor.ALTI.depth < 3.5)
	// {
	// 	// controlPitch
	// 	while (isPitch == true)
	// 	{
	// 		float pitchError = gliderConstants.descendPitch - sensor.IMU.omegaPitch;
	// 		controlPitch(pitchError);
	// 		if (fabs(pitchError < 2))
	// 		{
	// 			// flag
	// 			isPitch = false;
	// 		}
	// 	}
	// 	// controlDepth
	// 	controlDepth();
	// 	sleep(1);
	// }
	// isPitch = true;
	// // gliding transition
	// while (isPitch == true)
	// {
	// 	float pitchError = gliderConstants.ascendPitch - sensor.IMU.omegaPitch;
	// 	controlPitch(pitchError);

	// 	if (fabs(pitchError < 2))
	// 	{
	// 		// flag
	// 		isPitch = false;
	// 	}
	// }
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
	dcout(MMvarMid);
	dcout(BEvarMid);
	dcout(rudderMid);

	//.:.serial initialization
	// loading configuration port for GCS, BB and Arduino
	dcout(":Opening Serial port ... \n");
	Utils::loadConfig(GS, "GS.cfg");
	Utils::loadConfig(BB, "BB.cfg");
	Utils::loadConfig(AR, "AR.cfg");

	// testbench
	// ***************testing for the port location??????????????????????
	if (argc >= 2)
	{
		// testbench location
		GS.loc = "/dev/ttyS5";
		BB.loc = "/dev/ttyS7";
		AR.loc = "/dev/ttyS11";

		// //debugport
		Utils::loadConfig(debugOverride, "OverridePort.cfg");
		debugOverride.port = Utils::openPort(debugOverride.loc, debugOverride.baudrate);
		debugOverride.activateThread();
	}

	//.:.added on December
	// getting the overidePort, for ttyS0 and baudrate 115200, but for what??
	Utils::loadConfig(debugOverride, "OverridePort.cfg");
	debugOverride.port = Utils::openPort(debugOverride.loc, debugOverride.baudrate);
	drun(debugOverride.printConnectionStatus());
	//:.:added on December

	thread override_thread(debug_override_task);
	if (DEBUGTOOLS)
		override_thread.detach();
	else
		override_thread.join();

	// opening port GCS, BB, Arduino
	GS.port = Utils::openPort(GS.loc, GS.baudrate);
	BB.port = Utils::openPort(BB.loc, BB.baudrate);
	AR.port = Utils::openPort(AR.loc, AR.baudrate);
	drun(GS.printConnectionStatus());
	drun(BB.printConnectionStatus());
	drun(AR.printConnectionStatus());
	dcout("done\n");
	//:.:serial initialization

	//.:.multithread initialization
	dcout(":Create then detach sub-thread \n");
	int harwareConcurency = std::thread::hardware_concurrency();
	dwatch(harwareConcurency);

	thread Actuator_thread(Actuator_task);
	thread Sensor_thread(Sensor_task);
	thread GCS_thread(GCS_task);
	thread depthMechanism_thread(depthMechanism);

	Actuator_thread.detach();
	Sensor_thread.detach();
	GCS_thread.detach();
	depthMechanism_thread.detach();
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
	char _buff[300];
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
	tcflush(AR.port, TCIOFLUSH);

	// activate multithread task
	GS.activateThread();
	BB.deactivateThread();
	AR.deactivateThread();

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
	Utils::serWrite(BB.port, "#TS> OPERATION 2\r\n");
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
			fsm.targetCount = gWPoint.status.size() * BASE;
			// target completion requirement
			fsm.targetCount += SINGLETARGETFINISH;
			// initial operation state
			fsm.state = BASE;
			// calculate finished waypoints

			if (gWPoint.status.size() == 0)
			{
				dcout("WayPointStatus False \n");
			}
			// elseif(gWPoint.status.size() == 1)
			// {
			// 	dcout
			// }

			// increase base operation according to finished waypoint
			for (int i = 0; i < gWPoint.status.size(); i++)
			{
				if (gWPoint.status[i] == 1)
					fsm.baseTo(+BASE);
				// fsm.state += BASE;
			}

			BB.activateThread();
			if (!overrideInterval)
			{
				char BBparam[300];
				sprintf(BBparam, "#TS> PARAM %d\r\n", gParam.interv);
				Utils::serWrite(BB.port, BBparam);
			}
			else
			{
				char BBparam[300];
				sprintf(BBparam, "#TS> PARAM %d\r\n", overrideIntervalValue);
				Utils::serWrite(BB.port, BBparam);
			}
			// wait ack param
			// change to PING for bypass, default val : WAITING
			BB.isACK = PING;

			// todo : sendparam to AR
			// wait ack param
			AR.activateThread();
			// change to PING for bypass, default val : WAITING
			AR.isACK = PING;

			// fsm.moveTo(SINGLETARGETSETUP);
			first = true;
			state = -1;
			fsm.moveTo(HALT);
			// fsm.Print();
			break;
		case SINGLETARGETSETUP:
			dcout("Masuk SINGLETARGEWTSETUP \n");
			// check for reply from BB
			if (BB.isACK == PING || BB.isACK == OPERATION || overrideBBACK)
			{
				// check for reply from AR
				if (AR.isACK == PING || AR.isACK == OPERATION || overrideARACK)
				{
					// send start signal (all sensor active)
					Utils::serWrite(BB.port, "#TS> OPERATION 3\r\n");

					// send ACK start/RTO
					GS.sendACK();

					// set waypoint
					guidance.sourceLat = gWPoint.latStart[fsm.upperState() - 1];
					guidance.sourceLon = gWPoint.lonStart[fsm.upperState() - 1];
					guidance.targetLat = gWPoint.latEnd[fsm.upperState() - 1];
					guidance.targetLon = gWPoint.lonEnd[fsm.upperState() - 1];
					fsm.isWaitingSensorUpdate = true;
					glider.pose = gliderConstants.descending;
					// GS.deactivateThread();
					fsm.moveTo(GLIDING);
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
				Utils::serWrite(BB.port, "#TS> OPERATION 1\r\n");
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
			Utils::serWrite(BB.port, "#TS> OPERATION 2\r\n");
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
				Utils::serWrite(BB.port, "#TS> OPERATION 3\r\n");
				fsm.isWaitingSensorUpdate = true;
				tcflush(GS.port, TCIOFLUSH);

				// notify producer
				fsm.isWaitingSensorUpdate = true;

				//				GS.activateThread();
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
			//  HALT FOR GLIDE TEST
			if (gParam.isValid("#NICE ONE 1 2 3 4 5", 5))
			{
				Utils::GliderUtility gliderUtils;
				gliderUtils.parsingData("#NICE ONE 1 2 3 4 5", &BE_input, &MM_input, &rudder_input, &yaw_input, &main_input);
				selfTest();
			}
			// manualGliding();
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
			controlYaw();
			break;
		case GUIDANCE:
			dcout("Masuk Guidance (Ngotak) \n");
			guidanceTest();
			break;
		case MANUALGLIDING:
			dcout("Masuk Manual Glide \n");
			manualGliding();
			break;
		case DEPTHCONTROL:
			dcout("Masuk Depth Control");
			desDepth = 4;
			controlDepth();
			break;
		case STOP: // STOP
				   // change glider fsm to waiting
			resetActuator();
			if (fsm.lowerState() != WAITING)
				fsm.moveTo(WAITING);
			break;
		default:
			break;
		}
		usleep(100000);
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
	Utils::serWrite(AR.port, ARbuffer);

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
	/* //di comment
	//rudder
	sprintf(logMessagePID,"%.6f %.6f %.6f %.6f",heading.refference,heading.current,heading.error,heading.output);
	Utils::write2LOG("PID-heading",logMessagePID,false);
	//ballast
	sprintf(logMessagePID,"%.6f %.6f %.6f %.6f",pitch.refference,pitch.current,pitch.error,pitch.output);
	Utils::write2LOG("PID-pitch",logMessagePID,false);
	//bladder
	sprintf(logMessagePID,"%.6f %.6f %.6f %.6f",buoyancy.refference,buoyancy.current,buoyancy.error,buoyancy.output);
	Utils::write2LOG("PID-buoyancy",logMessagePID,false);
	*/
	// sampai sini
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
				cout << "GS RAW : " << GS.rawdata << endl;

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
						//						glider.isTransmitingData = true;
						sendDatalog();
						// let GCS know that all data is sent
						Utils::serWrite(GS.port, "#TS> ENDDATA\r\n");
						// wait for next glide operation
						glider.datalogReady = false;
						//						glider.isTransmitingData = false;
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

					// OPERATION
					// Disini untuk selftest					//null
					// dcout("Masuk ke Self Test \n");
					// selfTest();
					break;
				case RTO: // RTO OR CTM
				case CTM:
					GS.sendACK();
					if (fsm.lowerState() == WAITINSTRUCTION)
						fsm.moveTo(SINGLETARGETFINISH);
					break;
				case SELFTEST:
					// added 25 Des 2023
					// *selfTest
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
					controlPitch();
					break;
				case DEPTHCONTROL:
					dcout("MASUK DEPTHCONTROL \n");
					dcout("Offset IMU: " << offsetIMU << "\n");
					break;
				case NGOTAK:
					dcout("MASUK NGOTAK \n");
					first = true;
					state = -1;
					fsm.moveTo(GUIDANCE);
					break;
				case MANUALGLIDING:
					if (true)
					{
						dcout("MASUK Manual Glide \n");
						dcout(GS.rawdata);
						std::string _str = GS.rawdata.erase(0, 17);
						desYaw = std::stof(_str);
						dcout("Des Yaw string: " << _str << "\n");
						dcout("Des Yaw float: " << desYaw << "\n");
						fsm.moveTo(MANUALGLIDING);
					}
				default: // invalid data
					// try to flush I/O (not tested)
					tcflush(GS.port, TCIOFLUSH);
					break;
				} // switch case based on datacommand
			}
		}
		usleep(1000);
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
			int bytes_read = read(BB.port, &BBinbound, sizeof(BBinbound));
			BB.rawdata = Utils::convertToString(BBinbound);

			if (echoSerial)
				cout << "BB RAW : " << BB.rawdata << endl;
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

					break;
				case DATA: // DATA
					// get lock to avoid data update while gliding
					threadmutex.lock();
					// separate args
					sensor.Parse(BB.dataArgs);
					// sensor.Print(sensorIMU | sensorDVL | sensorALTI);

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
		usleep(1000);
	}
}

void Actuator_task()
{
	// wait and skip handling if task_flag is not raised
	// handle incoming communication from GCS
	while (1)
	{
		if (!AR.thread)
		{
		}
		else
		{
			// blocking read serial
			char ARinbound[1000];
			memset(&ARinbound, '\0', sizeof(ARinbound));
			int bytes_read = read(AR.port, &ARinbound, sizeof(ARinbound));
			AR.rawdata = Utils::convertToString(ARinbound);

			if (echoSerial)
				cout << "AR RAW : " << AR.rawdata << endl;

			// check if thread is still active
			if (!AR.thread)
			{
			}
			else
			{
				// parse data based on rawdata
				// if valid, change values of
				// datasender, datacommand, dataargs, and datacode
				// return -1 on dataCode if data is not found within defaultmessage
				Utils::Parse(AR);
				dwatch(AR.dataSender);
				dwatch(AR.dataCommand);
				dwatch(AR.dataArgs);
				dwatch(AR.dataCode);

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
					tcflush(AR.port, TCIOFLUSH);
					break;
				} // switch case based on datacommand
			}
		}
	}
}

void sendDatalog()
{
	// deactivate threads
	GS.deactivateThread();
	BB.deactivateThread();
	AR.deactivateThread();

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
	AR.activateThread();
	BB.activateThread();
}

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
