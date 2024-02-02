//second commit

#include <cstring>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <time.h>
#include <stdio.h>
#include <pthread.h>
#include <cstdlib>
#include <ctime>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include "sys/select.h"
#include "ExtendedLib/_param.h"
#include "ExtendedLib/_gcsCommandParser.h"
#include "ExtendedLib/_wPoint.h"

//nextdebugging
bool nextdebugging = 0;

//extended lib declaration
gliderParam gParam;
gliderWP gWPoint;

using namespace std;

//dpwdebug
	bool printstatFSM = false;
//dpwdebug


//dpw komunikasi
int openport(char* portname,int BAUDRATE){
	/*note:
		Serial config is 9600 8n1 / 115200 8n1
		no software flow control
		canonical mode (read until \n (newline))
	*/
	
    struct termios oldtio,newtio;

	int sport = open(portname, O_RDWR | O_NOCTTY | O_SYNC);	

    if (tcgetattr(sport, &oldtio) == -1) {
        cout << "tcgetattr failed" << endl;
        return -1;
    }

    cfmakeraw(&newtio); // Clean all settings

	if (BAUDRATE == 115200)
		newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B115200; 
	else
		newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B9600; 
	
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~(PARENB | PARODD); // No parity
    newtio.c_cflag &= ~CRTSCTS; // No hardware handshake
    newtio.c_cflag &= ~CSTOPB; // 1 stopbit
    newtio.c_iflag = IGNBRK;
    newtio.c_iflag &= ~(INPCK| IUCLC | IMAXBEL); 
	newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // No software handshake

	newtio.c_cc[VEOL] = '\n';
	newtio.c_iflag &= ~(INLCR | IGNCR | ICRNL);

  //newtio.c_iflag &= ~(IXANY); // No software handshake
  //newtio.c_iflag |= (IXON | IXOFF ); // software handshake

    newtio.c_oflag = 0;
	newtio.c_oflag &= ~OPOST;

    newtio.c_lflag = 0;	
	
  //newtio.c_cc[VTIME] = 0;
  //newtio.c_cc[VMIN] = 1;
	newtio.c_lflag |= ICANON; //Enable canonical mode

    tcflush(sport, TCIFLUSH); // Clear IO buffer
    tcflush(sport, TCIOFLUSH); // Clear IO buffer


    if (tcsetattr(sport, TCSANOW, &newtio) == -1) {
        cout << "tcsetattr failed" << endl;
        return -1;
    }

    //tcflush(sport, TCIOFLUSH); // Clear IO buffer

	cout << "port open : " << sport << endl;
	if(sport < 0) {
		printf("Error opening %s: \n ",strerror(errno));
		return -1;
	}

	return sport;
}

struct Serial{
	char* loc;
	int port;
	bool isSet()
	{
		if (port>0) return true;
		return false;
	}
	
}BB,AR,GS;
//dpw komunikasi

int write2LOG(char *logname, char* message)
{
	//call example : write2LOG("test.log","first commit")

	time_t timeNow;
	struct tm * timestamp;
	time(&timeNow);
	timestamp = localtime(&timeNow);
		
	char time_buffer[300];	
	strftime(time_buffer, sizeof(time_buffer), "%d-%m-%Y %H:%M:%S\t", timestamp); 

	char buffer_logname[50];
	sprintf(buffer_logname,"Log/%s.txt",logname);

	ofstream log(buffer_logname, ios::out | ios::app);
	if(!log.is_open()) {
		cout << "fail to write on "<<logname<<endl;
		return 0;
	} else
	{
		log << time_buffer << message << endl;
	    log.close();
	}
	return 1;
}

void *control_task (void *threadid);
void *sensor_task (void *threadid);
void *fsm_task (void *threadid);

bool sensor_task_flag;
bool control_task_flag;
bool fsm_task_flag;
bool comm_task_flag;

double shared_x;
pthread_mutex_t lock_flag;

bool stream_sensor;
bool stream_control;
bool stream_fsm;
bool handler_fsm;
bool handler_protocol;
bool handler_sensor;
bool handler_control;

bool ascent_flag;
bool descent_flag;
bool pitch_down_flag;
bool pitch_up_flag;

//handler fungsi sistem glider
void handlingBB(int port);
void handlingPID();
void handlingFSM();
void tempValue(); //redundant

void protocol_operation(int port);
void command_line();
void PID_heading_rudder();
void PID_pitch_ballast();
void PID_buoyancy_bladder();

//void FSM
void startOperation();
void stopoperation();
void glideOeration();
void descentState();
void ascentState();
void surfaceValue();
void bowthrusterOperation();
void bowthrusterOn();
void bowthrusterOff();
void propeller_operation();
void manual_mode();


string fsm_input;
int buf_str_fsm; //redundant
int mode;

//variabel gliding & data sensor
int BBcount = 0, ngliding = 0, nglided = 15,
	data_entry, data_limit = 1000, sampling_time = 1;

//value PID
/*
char rudder_val[6];
char ballast_val[6];
char bladder_val[6];
char char_prop[4];
char char_bow[4];
char char_strobe[1];
*/

char rd_val[6];
char bld_val[6];
char blst_val[6];
char char_prop[4];
char char_bow[4];
char char_strobe[1];

float rd_PID_val, blst_PID_val, bld_PID_val,
	  man_rd_val, man_blst_val, man_bld_val,
	  man_bld_des = 0, man_bld_asc = 0, man_bld_net = 0,
	  man_blst_des = 0, man_blst_asc = 0, man_blst_net = 0,
	  temp_yaw, temp_pitch, temp_depth;
float new_heading;

//variable sensor, teta = pitch, z = depth, yaw = heading, pos_e = x, pos_n = y
float teta_terukur, z_terukur, yawref, yaw_terukur, pos_e, pos_n, alt, sal,
	  temp, spd, lat, lon, roll;

float global_heading, global_pitch, global_depth;

float surface_depth = 0, depth_threshold = 17, depth_target = 20,
	  surface_pitch = 0, threshold_pitch_d = -0.26, threshold_pitch_a = 0.26;

float motion = 1, battery = 100.0, leakage = 0, strobo = 1, csum = 0, batCurrent;
float guidancestatus = 0;
// variable PID rudder
float sat_rd_u = 0.5, sat_rd_d = -0.5,
	  heading_ref, heading_fb,
	  heading_err = 0, heading_sumerr = 0,
	  heading_laster, PID_rd_out,
	  PID_rd_cal, PID_rd_unsat,
	  rd_AW = 0;

float PID_rd_KP = 70,
	  PID_rd_KI = 1,
	  PID_rd_KD = 0.1,
	  PID_rd_AW = 1;

float PID_rd_pro, PID_rd_int, PID_rd_dev;

// variable PID ballast
float sat_blst_u = 0.5, sat_blst_d = -0.5, 
	  pitch_ref, pitch_fb, 
	  ref_pitch_d = -0.26, ref_pitch_a = 0.26,
	  ref_pitch, pitch_err = 0, pitch_sumerr = 0,
	  pitch_lasterr, PID_blst_out,
	  PID_blst_cal, PID_blst_unsat,
	  blst_AW = 0;

float PID_blst_KP = -0.5,
	  PID_blst_KI = -0.05,
	  PID_blst_KD = -0.03,
	  PID_blst_AW = 0;

float PID_blst_pro, PID_blst_int, PID_blst_dev;

// variable PID bladder
float sat_bld_u = -0.5, sat_bld_d = 0.5,
	  depth_ref, depth_fb, 
	  ref_depth_d, ref_depth_a,
	  ref_depth, depth_err = 0, depth_sumerr = 0,
	  depth_lasterr, PID_bld_out,
	  PID_bld_cal, PID_bld_unsat,
	  bld_AW = 0;

float PID_bld_KP = 20,
	  PID_bld_KI = 0,
	  PID_bld_KD = 0.5,
	  PID_bld_AW = 1;

float PID_bld_pro, PID_bld_int, PID_bld_dev;

// variable propeller, bowthrusters, strobe
int prop_val_on = 1600, prop_val_off = 0,
	bow_val_on = 1800, bow_val_off = 0, bowCount = 0,
    strobe_on = 1, strobe_off = 0, rd_surface = 0,
	bld_surface = 0, blst_surface = 0,
	prop_val, bow_val, strobe_val;

// datalog variables
string datalog_day;
string datalog_mon;
string datalog_date;
string datalog_year;
string datalog_hour;
string datalog_minute;
string datalog_second;
string datalog_temperature;
string datalog_salinity;
string datalog_altitude;
string datalog_z_terukur;
string datalog_latitude;
string datalog_longitude;
string datalog_roll;
string datalog_pitch;
string datalog_yawref;
string datalog_yaw_terukur;
string datalog_pos_e;
string datalog_pos_n;
string datalog_motion;
string datalog_battery;
string datalog_leakage;
string datalog_speed;
string datalog_strobo;
string datalog_csum;
string datalog_boardTemp;
string datalog_alti;

string string_bld;
string string_blst;
string string_teta;
string string_rd;
string string_pose;
string string_posn;
string string_strobe;

stringstream stream_bld;
stringstream stream_blst;
stringstream stream_teta;
stringstream stream_rd;
stringstream stream_pose;
stringstream stream_posn;
stringstream stream_strobe;

// array parsing sensor
char comBBheader[10];
char comBB_1[10];
char comBB_2[10];
char comBB_3[10];
char comBB_4[10];
char comBB_5[10];
char comBB_6[10];
char comBB_7[10];
char comBB_8[10];
char comBB_9[10];
char comBB_10[10];
char comBB_11[10];
char comBB_12[10];
char comBB_13[10];

char char_teta[10];
char char_pos_e[10];
char char_pos_n[10];
char state_hils[2];

void serWrite(int _port, char _msg[]){
	char* p = _msg;

	for (; *p != '\0'; ++p){
         // if '\0' happens to be valid data for your app, 
         // then you can (maybe) use some other value as sentinel
    }
	int arraySize = p - _msg;	
	write(_port,_msg,arraySize);
	usleep(5000);
}

string convertToString(char* a) { 
    string s(a); 
    // int i; 
    // for (i = 0; i < size; i++) { 
    //     s = s + a[i]; 
    // } 
    return s; 
} 

#define default 0
#define ACK 1
#define fromAR 2 
#define fromBB 3
void sendPING(int target,int status = 0){
	time_t timeNow;
	struct tm * timestamp;
	time(&timeNow);
	timestamp = localtime(&timeNow);
		
	char buffer[300],log_buffer[300];
	
	if (status!=default){
		if (status == ACK)
			strftime(buffer, sizeof(buffer), "#TS> ack PING %d%m%Y%H%M%S\r\n", timestamp);  else
		if (status == fromAR)
			strftime(buffer, sizeof(buffer), "#AR> ack PING %d%m%Y%H%M%S\r\n", timestamp); else
		if (status == fromBB)
			strftime(buffer, sizeof(buffer), "#BB> ack PING %d%m%Y%H%M%S\r\n", timestamp); 
		strncpy(log_buffer,buffer,sizeof(buffer));
	}
	else{
		//data to be sent
		strftime(buffer, sizeof(buffer), "#TS> PING %d%m%Y%H%M%S\r\n", timestamp); 
		//data for log
		if (target == BB.port)
			strftime(log_buffer, sizeof(log_buffer), "#TS> PING BB %d%m%Y%H%M%S\r\n", timestamp); 
		else
		if (target == AR.port)
			strftime(log_buffer, sizeof(log_buffer), "#TS> PING AR %d%m%Y%H%M%S\r\n", timestamp); 
		else
		if (target == GS.port)
			strftime(log_buffer, sizeof(log_buffer), "#TS> PING GS %d%m%Y%H%M%S\r\n", timestamp); 
	}

	serWrite(target,buffer);
	write2LOG("PING",log_buffer);
}

int main(void) {
	pthread_t sensor_read;
	pthread_t control_PID;
	pthread_t fsm_handle;

	cout << "PROGRAM START" << endl;

	pthread_mutex_init(&lock_flag, NULL);

	GS.loc = "/dev/ttyxuart4";
	GS.port = openport(GS.loc,115200);
	if (!GS.isSet()) cout <<GS.loc<<" OPEN ERROR"<<endl;
	sendPING(GS.port);

	BB.loc = "/dev/ttyxuart0";
	BB.port = openport(BB.loc,9600);
	if (!BB.isSet()) cout <<BB.loc<<" OPEN ERROR"<<endl;
	sendPING(BB.port);
	
	AR.loc = "/dev/ttyxuart1";
	AR.port = openport(AR.loc,9600);
	if (!AR.isSet()) cout <<AR.loc<<" OPEN ERROR"<<endl;
	sendPING(AR.port);
	//:.:communication

	//Sub thread init
	int rc = pthread_create(&fsm_handle, NULL, fsm_task, NULL);
	cout << "Create thread fsm" << endl;
	if(rc) {
		cout << "Error : unable to create thread" << endl;
		exit(-1);
	}
	else {
		cout << "FSM thread created, ID : " << fsm_handle << endl;
	}

	rc = pthread_create(&control_PID, NULL, control_task, NULL);
	cout << "Create thread control" << endl;
	if(rc) {
		cout << "Error : unable to create thread" << endl;
		exit(-1);
	}
	else {
		cout << "Control thread created, ID : " << control_PID << endl;
	}

	rc = pthread_create(&sensor_read, NULL, sensor_task, NULL);
	cout << "Create thread sensor" << endl;
	if(rc) {
		cout << "Error : unable to create thread" << endl;
		exit(-1);
	}
	else{
		cout << "Sensor thread created, ID : " << sensor_read << endl;
	}

	sensor_task_flag = 1;
	control_task_flag = 1;
	fsm_task_flag = 1;
	comm_task_flag = 1;
	
	//immediately activate sensor readings
	handler_sensor = 1;
	
	while(comm_task_flag == true) {
		command_line();

		if(handler_sensor == true) {
			stream_sensor = 1;
		}

//		if(handler_control == true) {
	//		stream_control = 1;
		//}

		if(handler_fsm == true) {
			stream_fsm = 1;
		}

		if(mode = 0) {
			//debug
			cout <<"mode reset to 0 " << endl;
			handler_sensor = 0, handler_control = 0, handler_fsm = 0;
			stream_sensor = 0, stream_control = 0, stream_fsm = 0;
		}

		sleep(1);
	}
	cout << "END PROGRAM" << endl;
	cout << "Main Thread Exit" << endl;
	pthread_exit(NULL);
	return 0;
}

void *fsm_task(void *threadid) {
	if(!fsm_task_flag) {
		cout << "FSM Flag false" << endl;
		sleep(1);
	}
	while(fsm_task_flag == true) {
		handlingFSM();
		sleep(1);
	}
	pthread_exit(NULL);
}

void *control_task(void *threadid) {
	if(!control_task_flag) {
		cout << "Control Flag false" << endl;
		sleep(1);
	}
	while(control_task_flag == true) {
		handlingPID();
		sleep(1);
	}
	pthread_exit(NULL);
}

void *sensor_task(void *threadid) {
	if(!sensor_task_flag) {
		cout << "Sensor Flag false" << endl;
		sleep(1);
	}
	while(sensor_task_flag == true) {
		handlingBB(BB.port);
		sleep(1);
	}
	pthread_exit(NULL);
}

void PID_heading_rudder() {
	if(nextdebugging)	cout << "PID rudder running" << endl;
	heading_ref = yawref;
	heading_fb = temp_yaw;
	heading_ref = (heading_ref*3.1416)/180.0;
	heading_fb = (heading_fb*3.1416)/180.0;
	heading_laster = heading_err;
	heading_err = heading_err - heading_fb;
	heading_sumerr = heading_sumerr + heading_err + rd_AW;

	//nilai proporsional
	PID_rd_pro = heading_err * PID_rd_KP;
	//nilai integral
	PID_rd_int = heading_sumerr * PID_rd_KI * sampling_time;
	//nilai derivative
	PID_rd_dev = (heading_err - heading_laster) * PID_rd_KD / sampling_time;

	//nilai total P+I+D
	PID_rd_cal = PID_rd_pro + PID_rd_int + PID_rd_dev;

	//upper limit 0.5 
	if(PID_rd_cal > sat_rd_u) {
		PID_rd_out = sat_rd_u;
	}
	//lower limit -0.5 
	else if(PID_rd_cal < sat_rd_d) {
		PID_rd_out = sat_rd_d;
	}
	//-0.5 -- 0.5
	else {
		PID_rd_out = PID_rd_cal;
	}

	rd_AW = (PID_rd_out - PID_rd_cal) * PID_rd_AW;
	
	//sementara nilainya rentang -0.5 sampai 0.5
	//rubah ke dalam bentuk -100% sampai 100%
	//0.5	== 50%
	//1		== 100%
	//-0.25	== -25%

	rd_PID_val = PID_rd_out;

	if(nextdebugging)
	if(PID_rd_out != 0) {
		cout << "PID rudder : " << PID_rd_out << endl;
	}
	else if(PID_rd_out == 0) {
		cout << "PID rudder : 0" << endl;
	}
}

void PID_pitch_ballast() {
	if(nextdebugging)	cout << "PID ballast running" << endl;
	pitch_ref = ref_pitch;
	if(nextdebugging)	printf("pitch_ref : %f\n", pitch_ref);
	pitch_fb = temp_pitch;
	pitch_lasterr = pitch_err;
	pitch_err = pitch_ref - pitch_fb;
	if(nextdebugging)	printf("pitch_err : %f\n", pitch_err);
	pitch_sumerr = pitch_sumerr + pitch_err + blst_AW;

	PID_blst_pro = pitch_err * PID_blst_KP;
	PID_blst_int = pitch_sumerr * PID_blst_KI * sampling_time;
	PID_blst_dev = (pitch_err - pitch_lasterr) * PID_blst_KD / sampling_time;

	PID_blst_cal = PID_blst_pro + PID_blst_int + PID_blst_dev;

	if(nextdebugging)	printf("blst_cal : %f\n", PID_blst_cal);

	if(PID_blst_cal > sat_blst_u) {
		PID_blst_out = sat_blst_u;
	}
	else if(PID_blst_cal < sat_blst_d) {
		PID_blst_out = sat_blst_d;
	}
	else {
		PID_blst_out = PID_blst_cal;
	}
	// blst_AW = (PID_blst_out - PID_blst_cal) * PID_blst_AW;
	blst_PID_val = PID_blst_out;

	if(nextdebugging)
	if(PID_blst_out != 0) {
		cout << "PID ballast : " << PID_blst_out << endl;
	}
	else if(PID_blst_out == 0) {
		cout << "PID ballast : 0" << endl;
	}
}

void PID_buoyancy_bladder() {
	if(nextdebugging)	cout << "PID bladder running" << endl;
	depth_ref = ref_depth;
	if(nextdebugging)	printf("depth_ref %f\n", depth_ref);
	depth_fb = temp_depth;
	depth_lasterr = depth_err;
	depth_err = depth_ref + depth_fb;
	if(nextdebugging)	printf("depth_err %f\n", depth_err);
	depth_sumerr = depth_sumerr - depth_err + bld_AW;

	PID_bld_pro = depth_err * PID_bld_KP;
	PID_bld_int = depth_sumerr * PID_bld_KI * sampling_time;
	PID_bld_dev = (depth_err - depth_lasterr) * PID_bld_KD / sampling_time;

	PID_bld_cal = PID_bld_pro + PID_bld_int + PID_bld_dev;

	if(nextdebugging)	printf("bld_cal %f\n", PID_bld_cal);
	
	if(PID_bld_cal < sat_bld_u) {
		PID_bld_out = sat_bld_u;
	}
	else if(PID_bld_cal > sat_bld_d) {
		PID_bld_out = sat_bld_d;
	}
	else {
		PID_bld_out = PID_bld_cal;
	}
	// bld_AW = (PID_bld_out - PID_bld_cal) * PID_bld_AW;
	bld_PID_val = PID_bld_out;

	if(nextdebugging)
	if(PID_bld_out != 0) {
		cout << "PID bld out thread BB : " << PID_bld_out << endl;
		depth_lasterr = depth_err;
	}
	else if(PID_bld_out == 0) {
		cout << "PID bld thread BB : 0" << endl;
	}
}

void parseBBData(string str){
	const char _teta_terukur	= 2;
	const char _z_terukur		= 3;
	const char _yawref			= 4;
	const char _yaw_terukur		= 5;
	const char _pos_e			= 6;
	const char _pos_n			= 7;
	const char _alt				= 8;
	const char _sal				= 9;
	const char _temp			= 10;
	const char _spd				= 11;
	const char _lat				= 12;
	const char _lon				= 13;
	const char _roll			= 14;
	const char _battery			= 15;
	const char _bat_current		= 16;
	const char _leakage			= 17;
	const char _finishguidance	= 18;


	std::istringstream _sentence (str);
	char _count = 0;
	do
	{
		std::string _word;
		_sentence >> _word;
		switch (_count)
		{
			case _teta_terukur : std::istringstream (_word) >> 
								teta_terukur; 
								break;
			case _z_terukur : std::istringstream (_word) >> 
								z_terukur; 
								break;
			case _yawref : std::istringstream (_word) >> 
								yawref; 
								break;
			case _yaw_terukur : std::istringstream (_word) >> 
								yaw_terukur; 
								break;
			case _pos_e : std::istringstream (_word) >> 
								pos_e; 
								break;
			case _pos_n : std::istringstream (_word) >> 
								pos_n; 
								break;
			case _alt : std::istringstream (_word) >> 
								alt; 
								break;
			case _sal : std::istringstream (_word) >> 
								sal; 
								break;
			case _temp : std::istringstream (_word) >> 
								temp; 
								break;
			case _spd : std::istringstream (_word) >> 
								spd; 
								break;
			case _lat : std::istringstream (_word) >> 
								lat; 
								break;
			case _lon : std::istringstream (_word) >> 
								lon; 
								break;
			case _roll : std::istringstream (_word) >> 
								roll; 
								break;
			case _battery : std::istringstream (_word) >> 
								battery; 
								break;
			case _bat_current : std::istringstream (_word) >> 
								batCurrent; 
								break;
			case _leakage : std::istringstream (_word) >> 
								leakage; 
								break;
			case _finishguidance : std::istringstream (_word) >> 
								guidancestatus; 
								break;
		}
				_count++;
	} 
	while (_sentence);
}

void handlingBB(int port) {	
	while(stream_sensor == true) {
		pthread_mutex_lock(&lock_flag);

	if(nextdebugging)		printf("Mutex locked by sensor\n");
		
		time_t timeBB;
	    struct tm * timestamp;
	    time(&timeBB);
	    timestamp = localtime(&timeBB);
	    char buftimeBB[50];
	    char * timebufBB;
	    strftime(buftimeBB, 50, "%a %b %d %Y %H:%M:%S", timestamp);
	    ofstream logstateBB("state_function_thread.txt", ios::out | ios::app);
	    if(logstateBB.is_open()) {
	        logstateBB << buftimeBB << " | " << "Sensor Running" << endl;
	        logstateBB.close();
	    }

		char BBinbound[1000];
		memset(&BBinbound,'\0',sizeof(BBinbound));
		int bytes_read = read(port,&BBinbound,sizeof(BBinbound));
		string BBinboundstr = convertToString(BBinbound);


		cout << "debug : BB received : " << bytes_read <<" char - "<<BBinboundstr;
		cout << "-------------------------BBDATAEND"<<endl;

		//determine data
		if (BBinboundstr.find("#BB> ack PING") != std::string::npos)
		{
			cout << "receive PING from BB"<<endl;
			//savetolog
			//forward to GCS
			sendPING(GS.port,fromBB);
//			serWrite(GS.port,"#BB> PING \r\n");
		} else
		if (BBinboundstr.find("#BB> DATA") != std::string::npos)
		{
			//example data format
			//#BB> DATA 4.6037669 7209.0000 282.62532 345.64226 0.0000000 0.0000000 77.400001 98.000000 36.000000 2209.8898 -33.86820599 151.20369600 -90.37915
			cout << "receive DATA from BB"<<endl;			
			
			parseBBData(BBinboundstr);
			
			global_depth = z_terukur;
			global_pitch = teta_terukur;
			global_heading = yaw_terukur;

			if(global_depth <= -1*depth_threshold || global_depth == -1*depth_target) 
			{
				ascent_flag = 1;
				descent_flag = 0;
				pitch_up_flag = 1;
				pitch_down_flag = 0;
			}
			else 
			if(global_depth >= surface_depth) 
			{
				descent_flag = 1;
				ascent_flag = 0;
				pitch_up_flag = 0;
				pitch_down_flag = 1;
			}


			//printf("HANDLING BB : ascent_flag : %d, descent_flag : %d\n", ascent_flag, descent_flag);
			ofstream logflag("flag_logger.txt", ios::out | ios::app);
			if(logflag.is_open())
			{
				logflag << buftimeBB << "ascent_flag : " << ascent_flag << ", descent_flag : " << descent_flag
				<< ", pitch_up_flag : " << pitch_up_flag << ", pitch_down_flag : " << pitch_down_flag
				<< ", global_depth : " << global_depth << ", global_pitch : " << global_pitch << endl;
			}
							
			ofstream savedatalog("datalog_sensor.txt", ios::out | ios::app);
			
	//			savedatalog << std::fixed << std::setprecision(6)

			if(savedatalog.is_open())
			{
				//.:.debug : dummy value
				float boardTemp = 0.0; //ambil dari TS
				float alti = 1.0; //cek status alti on/off
				//:.:debug : dummy value
/*
				savedatalog << std::fixed << std::setprecision(6) << buftimeBB << ' ' << teta_terukur << ' ' << z_terukur << ' ' << yawref << ' ' << yaw_terukur << ' ' << pos_e << ' ' <<
				pos_n << ' ' << alt << ' ' << sal << ' ' << temp << ' ' << spd << ' ' << lat << ' ' << lon << ' ' << roll << ' ' << motion << ' ' << 
				leakage << ' ' << strobo << ' ' << boardTemp << ' ' << alti << ' ' << csum << ' ' << endl;
*/
//format penyesuaian 08072020
				savedatalog << std::fixed << std::setprecision(6) << buftimeBB << ' ' 
																  << temp 			<< ' ' << sal 		<< ' ' << alt		<< ' ' << z_terukur << ' ' 
																  << pos_e 			<< ' ' << pos_n 	<< ' ' << roll 		<< ' ' << teta_terukur << ' ' 
																  << yaw_terukur 	<< ' ' << motion 	<< ' ' << battery 	<< ' ' << leakage << ' '
																  << spd 			<< ' ' << strobo 	<< ' ' << boardTemp << ' ' << alti << ' ' 
																  << csum 			<< ' ' << endl;

			//	cout << "save success" << endl;
				savedatalog.close();
				BBcount = 0;
								
			}
		}
	    //tcflush(port, TCIOFLUSH);
	   // tcdrain(port);
		
		pthread_mutex_unlock(&lock_flag);
		if(nextdebugging)		printf("Mutex unlocked by sensor\n");
		
	    usleep(500000);
		handler_control = 1; 
		if(nextdebugging)		cout << "changing stream control in handlingBB" << endl;
		//stream_control = 1;
		// sleep(2);   
	    ofstream logparsingBB("state_function_thread.txt", ios::out | ios::app);
	    if(logparsingBB.is_open()) {
	        logparsingBB << buftimeBB << " | " << "Parsing Finished" << endl;
	        logparsingBB.close();
	    }
	}
	usleep(500000);
}

void tempValue() {
	temp_depth = global_depth;
	temp_pitch = global_pitch;
	temp_yaw = global_heading;
}

void handlingPID() {
	time_t tm_control;
    struct tm * control_systime;
    time(&tm_control);
    control_systime = localtime(&tm_control);
    char control_tm_info[50];
    strftime(control_tm_info, 50, "%a %b %d %Y %H:%M:%S", control_systime);
	//if (printstatFSM) printf("stream_sensor %d, stream_control %d\n", stream_sensor, stream_control);
	if(handler_control == true) {
		ofstream logstatecontrol("state_function_thread.txt", ios::out | ios::app);
	    if(logstatecontrol.is_open()) {
	        logstatecontrol << control_tm_info << " | " << "Control Running" << endl;
	        logstatecontrol.close();
	    }
		if (printstatFSM) cout << "Control System Start" << endl;
	if(nextdebugging)		printf("HANDLING PID : ascent_flag : %d, descent_flag : %d\n", ascent_flag, descent_flag);
		// handler_fsm = 1;
		//pthread_mutex_lock(&lock_flag);
	if(nextdebugging)		printf("Mutex locked by control\n");
		tempValue();
		if(ascent_flag == true && descent_flag == false) {
            ref_depth = surface_depth;
        }
        else if(descent_flag == true && ascent_flag == false) {
            ref_depth = depth_target;
        }

        if(pitch_down_flag == true && pitch_up_flag == false) {
        	ref_pitch = threshold_pitch_d;
        }
        else if(pitch_up_flag == true && pitch_down_flag == false) {
        	ref_pitch = threshold_pitch_a;
        }
		
		
		PID_heading_rudder();
		PID_pitch_ballast();
		PID_buoyancy_bladder();
		//pthread_mutex_unlock(&lock_flag);
	if(nextdebugging)		printf("Mutex unlocked by control\n");

		//unknown
		//stream_fsm = 1;
		
		ofstream controlvaluelog("value_control.txt", ios ::out | ios::app);
		if(controlvaluelog.is_open()) {
			controlvaluelog << control_tm_info << "| Error rudder : " << heading_err << ", PID rudder : " << rd_PID_val <<
             "| Error bladder : " << depth_err << ", PID bladder : " << bld_PID_val << 
             "| Error ballast : " << pitch_err << ", PID ballast : " << blst_PID_val << endl;
            controlvaluelog.close();
		}
		ofstream logref("error_reference_data.txt", ios::out | ios :: app);
        if(logref.is_open()) {
            logref << control_tm_info << " Ref Pitch/from FSM : " << pitch_ref << "/" << ref_pitch << ", Pitch Now : " << pitch_fb << ", Pitch Error : " << pitch_err << endl;
            logref << control_tm_info << " Ref Depth/from FSM : " << depth_ref << "/" << ref_depth << ", Depth Now : " << depth_fb << ", Depth Error : " << depth_err << endl;
            logref << control_tm_info << " Ref Heading/from FSM : " << heading_ref << ", Heading Now : " << heading_fb << ", Heading Error : " << heading_err << endl;
            depth_err = 0;
            logref.close();
        }
        ofstream logpitch("pitch_data.txt", ios::out | ios :: app);
        if(logpitch.is_open()) {
        	logpitch << control_tm_info << " Ref pitch : " << pitch_ref;
        }
		//usleep(500000);
		//stream_control = 0;
		handler_control = 0;
	}
	else {
		//if (printstatFSM) cout << "Control System OFF" << endl;
		ofstream logcontroloff("state_function_thread.txt", ios::out | ios::app);
	    if(logcontroloff.is_open()) {
	        logcontroloff << control_tm_info << " | " << "Control OFF" << endl;
	        logcontroloff.close();
	    }
	}
	usleep(500000);
}

void protocol_operation(int port) {
	//send data to AR
	time_t tm_op;
    struct tm * op_systime;
    time(&tm_op);
    op_systime = localtime(&tm_op);
    char op_sysinfo[50];
    strftime(op_sysinfo, 50, "%a %b %d %Y %H:%M:%S", op_systime);
	
	cout << "protocol operation"<<endl;
	
	if(stream_fsm == true) {
		stream_bld << fixed << setprecision(5) << bld_PID_val;
		stream_blst << fixed << setprecision(5) << blst_PID_val;
		stream_teta << fixed << setprecision(10) << ref_pitch-teta_terukur;
		stream_rd << fixed << setprecision(5) << rd_PID_val;
		stream_pose << fixed << setprecision(10) << pos_e;
		stream_posn << fixed << setprecision(10) << pos_n;
		stream_strobe << fixed << setprecision(1) << strobe_val;

		string_bld = stream_bld.str();
		string_blst = stream_blst.str();
		string_teta = stream_teta.str();
		string_rd = stream_rd.str();
		string_pose = stream_pose.str();
		string_posn = stream_posn.str();
		string_strobe = stream_strobe.str();

		stream_bld.str("");
		stream_blst.str("");
		stream_teta.str("");
		stream_rd.str("");
		stream_pose.str("");
		stream_posn.str("");
		stream_strobe.str("");

		stream_bld.clear();
		stream_blst.clear();
		stream_teta.clear();
		stream_rd.clear();
		stream_pose.clear();
		stream_posn.clear();
		stream_strobe.clear();

		strcpy(bld_val, string_bld.c_str());
		strcpy(blst_val, string_blst.c_str());
		strcpy(char_teta, string_teta.c_str());
		strcpy(rd_val, string_rd.c_str());
		strcpy(char_pos_e, string_pose.c_str());
		strcpy(char_pos_n, string_posn.c_str());
		strcpy(char_strobe, string_strobe.c_str());

		if(bld_PID_val<0){
			bld_val[0]='-';
		}

		else {
			bld_val[0]='0';
		}

	if(nextdebugging)		printf("bld_val %s, blst_val %s, char_teta %s, rd_val %s, char_pos_e %s, char_pos_n %s, char_strobe %s\n", bld_val, blst_val, char_teta, rd_val, char_pos_e, char_pos_n, char_strobe);
	    // sleep(1);	    
	if(nextdebugging)	    cout << "Sending data to AR, full control" << endl;
	    usleep(250000);

		//.:. EDIT
		/*
			data yg harus dikirim :
				bladder --> -100% - 100%
				ballast --> -100% - 100%
				propeller --> -100% - 100%
				rudder --> -45 45
				bowthruster --> -100% - 100%
				strobe --> 0 mati 1 nyala
				
				semua nilai % dikali 100 untuk menghindari conversi float
		*/
		//:.:



	    write(port, "$#TS ", 5);
        write(port, bld_val, 5);  
        write(port, " ", 1);
        write(port, blst_val, 5);
        write(port, " ", 1);

        //hapus
		write(port, char_teta, 10);
        write(port, " ", 1);
        
		write(port, rd_val, 5);
        write(port, " ", 1);
        write(port, char_pos_e, 10);
        write(port, " ", 1);
        write(port, char_pos_n, 10);
        write(port, " ", 1);
        write(port, char_strobe, 1);
        write(port, " \r\n", 2);
        handler_protocol = 1;

	    ofstream protocoldata("protocol_data.txt", ios::out | ios::app);
	    if(protocoldata.is_open()) {
	    	protocoldata << op_sysinfo << "$#TS " << bld_val << " " << blst_val << " " << char_teta << " " << rd_val << " " << char_pos_e << " " << char_pos_n << " " << char_strobe << endl;
 	    	protocoldata.close();
	   	}

	   	ofstream tetaval("teta_error.txt", ios::out | ios::app);
	   	if(tetaval.is_open()) {
	   		tetaval << op_sysinfo << "Nilai ballast : " << blst_val << ", Nilai error teta : " << char_teta << 
	   		", Error pitch : " << pitch_err << endl;
	   		tetaval.close();
	   	}
	   	tcdrain(port);
	}
	usleep(500000);
}

void startOperation() {
    time_t tm_sys_start;
    struct tm * start_systime;
    time(&tm_sys_start);
    start_systime = localtime(&tm_sys_start);
    char start_tm_info[50];
    strftime(start_tm_info, 50, "%a %b %d %Y %H:%M:%S", start_systime);

   	// printf("handler_sensor %d, handler_control %d\n", handler_sensor, handler_control);
   	printf("Operation Init\n");
	if(stream_sensor == true) {
		//stream_fsm == 1;
		// printf("stream_fsm : %d\n", stream_fsm);
	    ofstream comprotocol("com_log.txt", ios::out | ios::app);
	    if(comprotocol.is_open()) {
	        // write(fd2, "$#O1", 4);
	        cout << '\n';
    		comprotocol << start_tm_info << " " << "BB start | " << "$#O1" << endl;
	        comprotocol.close();
	    }

	    //start operation
		if(stream_fsm == true) {
		    ofstream start_fsm("log_state.txt", ios::out | ios::app);
		    if(start_fsm.is_open()) {
		        start_fsm << start_tm_info << " " << "State : Start Ops" << endl;
		        start_fsm.close();
		    }
		    cout << "Operation Start" << endl;
		}
	}
}

void glideOperation() {
	if(ascent_flag == true && descent_flag == false) {
		if(global_depth <= -1*depth_threshold || global_depth == -1*depth_target) {
			printf("Start Ascent\n");
			ascentState();
			// sleep(1);
			protocol_operation(AR.port);
			printf("depth now : %02.2f, depth target : %02.2f, depth threshold : %02.2f\n", global_depth, depth_target, depth_threshold);
			printf("pitch now : %02.2f\n", teta_terukur);
		}
		// descent_flag = 1;
		if(ascent_flag == true && global_depth >= surface_depth) {
			ngliding++;
			ascent_flag = 0;
			descent_flag = 1;
		}
		// sleep(1);
	}

	else if(descent_flag == true && ascent_flag == false) {
		if(global_depth >= surface_depth) {
			printf("Start Descent\n");
			descentState();
			// sleep(1);
			protocol_operation(AR.port);
			printf("depth now : %02.2f, depth target : %02.2f, depth threshold : %02.2f\n", global_depth, depth_target, depth_threshold);
			printf("pitch now : %02.2f\n", teta_terukur);
		}
		// ascent_flag = 1;
		if(descent_flag == true && global_depth <= -1*depth_threshold) {
			ascent_flag = 1;
			descent_flag = 0;
		}
		// sleep(1);
	}
	usleep(500000);
}

void stopOperation() {
    time_t tm_sys_stop;
    struct tm * stop_systime;
    time(&tm_sys_stop);
    stop_systime = localtime(&tm_sys_stop);
    char stop_tm_info[50];
    strftime(stop_tm_info, 50, "%a %b %d %Y %H:%M:%S", stop_systime);

    bld_PID_val = bld_surface;
    blst_PID_val = blst_surface;
    rd_PID_val = rd_surface; //ini nanti jadi rd_val untuk dikirim ke arduino
    prop_val = prop_val_off;
    bow_val = bow_val_off;
    strobe_val = strobe_on;
    
    while(global_depth < surface_depth) {
    	pitch_up_flag = 0;
    	pitch_down_flag = 0;
    	ref_pitch = surface_pitch;
        protocol_operation(AR.port);
    }

    stream_control = 0;
    stream_sensor = 0;

    descent_flag = 0;
    ascent_flag = 0;
    ngliding = 0;

	//stop operation
    if(stream_fsm == true) {
	    ofstream com_stop("com_log.txt", ios::out | ios::app);
	    if(com_stop.is_open()) {
	    	write(BB.port, "$#O0", 4);
    		cout << '\n';
    		com_stop << stop_tm_info << " " << "BB Stop | " << "$#O0" << endl;
	    	com_stop.close();
	    }
	    
	    ofstream stop_fsm("log_state.txt", ios::out | ios::app);
	    if(stop_fsm.is_open()) {
	        stop_fsm << stop_tm_info << " " << "State : Stop Ops" << endl;
	        stop_fsm.close();
	    }
	    buf_str_fsm = 0; handler_control = 0; handler_sensor = 0;
	    handler_fsm = 0; stream_fsm = 0;
	    cout << "Operation Stop" << endl;
	}
}

void descentState() {
    time_t tm_sys_des;
    struct tm * des_systime;
    time(&tm_sys_des);
    des_systime = localtime(&tm_sys_des);
    char des_tm_info[50];
    strftime(des_tm_info, 50, "%a %b %d %Y %H:%M:%S", des_systime);

    prop_val = prop_val_off;
    bow_val = bow_val_off;
    strobe_val = strobe_on;
	
	ofstream descent_fsm("log_state.txt", ios::out | ios::app);
    if(descent_fsm.is_open()) {
        descent_fsm << des_tm_info << " " << "State : Descent" << endl;
        descent_fsm.close();
    }
    cout << "Descent Start" << endl;
}

void ascentState() {
    time_t tm_sys_asc;
    struct tm * asc_systime;
    time(&tm_sys_asc);
    asc_systime = localtime(&tm_sys_asc);
    char asc_tm_info[50];
    strftime(asc_tm_info, 50, "%a %b %d %Y %H:%M:%S", asc_systime);

    prop_val = prop_val_off;
    bow_val = bow_val_off;
    strobe_val = strobe_on;

    ofstream ascent_fsm("log_state.txt", ios::out | ios::app);
    if(ascent_fsm.is_open()) {
        ascent_fsm << asc_tm_info << " " << "State : Ascent" << endl;
        ascent_fsm.close();
    }
    cout << "Ascent Start" << endl;
}

void surfaceValue() {
    time_t tm_sys_surface;
    struct tm * surface_systime;
    time(&tm_sys_surface);
    surface_systime = localtime(&tm_sys_surface);
    char surface_tm_info[50];
    strftime(surface_tm_info, 50, "%a %b %d %Y %H:%M:%S", surface_systime);

    bld_PID_val = bld_surface;
    blst_PID_val = blst_surface;
    rd_PID_val = rd_surface;
    prop_val = prop_val_off;
    bow_val = bow_val_off;
    strobe_val = strobe_on;
    while(global_depth != surface_depth) {
    	protocol_operation(AR.port);
    	ofstream surface_fsm("log_state.txt", ios::out | ios::app);
	    if(surface_fsm.is_open()) {
	        surface_fsm << surface_tm_info << " " << "State : Surfacing" << endl;
	        surface_fsm.close();
	    }
	    cout << "Surfacing" << endl;
    }
}

void bowthrusterOn() {
    bld_PID_val = bld_surface;
    blst_PID_val = blst_surface;
    rd_PID_val = rd_surface;
    prop_val = prop_val_off;
    bow_val = bow_val_on;
    strobe_val = strobe_on;
    protocol_operation(AR.port);

    cout << "Bowthrusters ON" << endl;
}

void bowthrusterOff() {
    bld_PID_val = bld_surface;
    blst_PID_val = blst_surface;
    rd_PID_val = rd_surface;
    prop_val = prop_val_off;
    bow_val = bow_val_off;
    strobe_val = strobe_on;
    protocol_operation(AR.port);
	
    cout << "Bowthrusters OFF" << endl;
}

void bowthrusterOperation() {
    time_t tm_sys_bow;
    struct tm * bow_systime;
    time(&tm_sys_bow);
    bow_systime = localtime(&tm_sys_bow);
    char bow_tm_info[50];
    strftime(bow_tm_info, 50, "%a %b %d %Y %H:%M:%S", bow_systime);

    if(handler_protocol == true) {
	    new_heading = yaw_terukur - 180.0;
	    if(new_heading != yawref) {
	        bowthrusterOn();
	        ofstream bow_on_fsm("log_state.txt", ios::out | ios::app);
	        if(bow_on_fsm.is_open()) {
	            bow_on_fsm << bow_tm_info << " " << "State : Bowthrusters ON" << endl;
	            bow_on_fsm.close();
	        }
	    }
	    else if(new_heading == yawref) {
	        bowthrusterOff();
	        ofstream bow_off_fsm("log_state.txt", ios::out | ios::app);
	        if(bow_off_fsm.is_open()) {
	            bow_off_fsm << bow_tm_info << " " << "State : Bowthrusters OFF" << endl;
	            bow_off_fsm.close();
	        }
	    }
	}
}

void propellerOperation() {
    time_t tm_sys_prop;
    struct tm * prop_systime;
    time(&tm_sys_prop);
    prop_systime = localtime(&tm_sys_prop);
    char prop_tm_info[50];
    strftime(prop_tm_info, 50, "%a %b %d %Y %H:%M:%S", prop_systime);

    bld_PID_val = bld_surface;
    blst_PID_val = blst_surface;
    rd_PID_val = rd_surface;
    prop_val = prop_val_on;
    bow_val = bow_val_off;
    strobe_val = strobe_on;
    protocol_operation(AR.port);

    ofstream prop_on_fsm("log_state.txt", ios::out | ios::app);
    if(prop_on_fsm.is_open()) {
        prop_on_fsm << prop_tm_info << " " << "State : Propeller ON" << endl;
        prop_on_fsm.close();
    }
    cout << "Propeller On" << endl;

	sleep(20);
    prop_val = prop_val_off;
    protocol_operation(AR.port);

    ofstream prop_off_fsm("log_state.txt", ios::out | ios::app);
    if(prop_off_fsm.is_open()) {
        prop_off_fsm << prop_tm_info << " " << "State : Propeller OFF" << endl;
        prop_off_fsm.close();
    }
    cout << "Propeller OFF" << endl;
}

void handlingFSM() {
	time_t tm_sys_fsm;
    struct tm * fsm_systime;
    time(&tm_sys_fsm);
    fsm_systime = localtime(&tm_sys_fsm);
    char fsm_tm_info[50];
    strftime(fsm_tm_info, 50, "%a %b %d %Y %H:%M:%S", fsm_systime);
    mode = buf_str_fsm;

    //if (printstatFSM) printf("FSM handling mode: %d\n", mode);

	switch(mode) {
		case 1 : //operation
		{		ofstream opstate("log_state.txt", ios ::out | ios::app);
				if(opstate.is_open()) {
					opstate << fsm_tm_info << ' ' << "State : Operation" << endl;
					opstate.close();
				}
				startOperation();
				printf("stream_fsm : %d\n", stream_fsm);
				// sleep(1);
				if(stream_control == true) {
					glideOperation();
					if(ngliding >= nglided) {
						surfaceValue();
						bowthrusterOperation();
						propellerOperation();
						stopOperation();
						//ngliding = 0;
					}
					printf("ngliding : %d\n", ngliding);
				}
				// sleep(1);
		}
		break;

		case 2 : //gliding
		{	
			ofstream glstate("log_state.txt", ios ::out | ios::app);
			if(glstate.is_open()) {
				glstate << fsm_tm_info << ' ' << "State : Gliding" << endl;
				glstate.close();
			}
			startOperation();
			printf("stream_fsm : %d\n", stream_fsm);
			// sleep(1);
			if(stream_control == true) {
				glideOperation();
				if(ngliding >= nglided) {
					surfaceValue();
					stopOperation();
					//ngliding = 0;
				}
				printf("ngliding : %d\n", ngliding);
			}
			// sleep(1);
		}
		break;

		case 3 : //propeller test
		{	
			startOperation();
			propellerOperation();
			stopOperation();
		}
		break;

		case 4 : //bowthruster test
		{	
			startOperation();
			while(bowCount < 10) {
				bowthrusterOn();
				bowCount++;
				usleep(500000);
			}
			bowthrusterOff();
			usleep(500000);
			// bowthrusterOperation();
			stopOperation();
			bowCount = 0;
		}
		break;

		case 5 : //manual op
		{
			startOperation();
			sleep(5);
			stopOperation();
		}
		break;
	}
}

void printcurrentstat(){
	cout << "handler sensor : "<<handler_sensor<<endl;
	cout << "handler control : "<<handler_control<<endl;
	cout << "handler fsm : "<<handler_fsm<<endl;

	cout << "stream sensor : "<<stream_sensor<<endl;
	cout << "stream control : "<<stream_control<<endl;
	cout << "stream fsm : "<<stream_fsm<<endl;
	
	cout << "stat - mode : "<<mode<<endl;
}

void sendtoAR(float _bld,
				  float _blst,
				  float _teta,
				  float _rd,
				  float _posE,
				  float _posN,
				  int _strobe){
					  
		char rd_val[6],bld_val[6], blst_val[6];
		char char_prop[4],char_bow[4],char_strobe[1];

		cout << "send data to AR : ";
		
		stringstream ss_bld,ss_blst,ss_teta,ss_rd,ss_pose,ss_posn,ss_strobe;

		ss_bld << fixed << setprecision(5) << _bld;
		ss_blst << fixed << setprecision(5) << _blst;
		ss_teta << fixed << setprecision(10) << _teta;		
		ss_rd << fixed << setprecision(5) << _rd;
		ss_pose << fixed << setprecision(10) << _posE;
		ss_posn << fixed << setprecision(10) << _posN;
		ss_strobe << fixed << setprecision(1) << _strobe;

		string str_bld = ss_bld.str();
		string str_blst = ss_blst.str();
		string str_teta = ss_teta.str();
		string str_rd = ss_rd.str();
		string str_pose = ss_pose.str();
		string str_posn = ss_posn.str();
		string str_strobe = ss_strobe.str();

		ss_bld.str("");
		ss_blst.str("");
		ss_teta.str("");
		ss_rd.str("");
		ss_pose.str("");
		ss_posn.str("");
		ss_strobe.str("");

		ss_bld.clear();
		ss_blst.clear();
		ss_teta.clear();
		ss_rd.clear();
		ss_pose.clear();
		ss_posn.clear();
		ss_strobe.clear();

		strcpy(bld_val, str_bld.c_str());
		strcpy(blst_val, str_blst.c_str());
		strcpy(char_teta, str_teta.c_str());
		strcpy(rd_val, str_rd.c_str());
		strcpy(char_pos_e, str_pose.c_str());
		strcpy(char_pos_n, str_posn.c_str());
		strcpy(char_strobe, str_strobe.c_str());
		
		
		write(AR.port, "$#TS ", 5);
        write(AR.port, bld_val, 5);  
        write(AR.port, " ", 1);
        write(AR.port, blst_val, 5);
        write(AR.port, " ", 1);
        write(AR.port, char_teta, 10);
        write(AR.port, " ", 1);
        write(AR.port, rd_val, 5);
        write(AR.port, " ", 1);
        write(AR.port, char_pos_e, 10);
        write(AR.port, " ", 1);
        write(AR.port, char_pos_n, 10);
        write(AR.port, " ", 1);
        write(AR.port, char_strobe, 1);
        write(AR.port, " \r\n", 2);;	
	
		cout << "done"<<endl;

}

void testActuator(){
	//bld blst teta rd posE posN strobe
	sendtoAR(0,0,0,0,0,0,0);	
	sleep(1);
	sendtoAR(0,0,0,0,0,0,0);	
}

void handlerGCS(int dat,string data, bool _debug = false){
	if (dat == AT)
	{
		printcurrentstat();
		
		serWrite(GS.port,"#TS> ack AT \r\n");
	} else
	//-------------------------------------------------------------------------
	if (dat == START)
	{
/*
		//check for RTB/RTO/test propeller/test bowthruster
		if (int c = gParam.subProcessExist())
		{
			if (_debug) cout << "unable to start : " << "pending task code "<<c<<endl;
			if (c == 12) //test prop 
			{
				//handler_fsm = 1;
				//buf_str_fsm = 3;
				testActuator();
				gParam.propeller = 0;
			}
			else
			if (c == 13) //test bow
			{
	//				handler_fsm = 1;
	//				buf_str_fsm = 4;
				testActuator();
				gParam.bow = 0;
			}
		}else
		{ */
			cout <<"starting mission"<<endl;
			{

					serWrite(BB.port,"#TS> OPERATION 1\r\n");
					//nextwork (kalau start, trus apa)

			}

			if (_debug)gParam.print();
			if (_debug)gWPoint.print();
//		}
		serWrite(GS.port,"#TS> ack START \r\n");
	}else
	//-------------------------------------------------------------------------
	if (dat == STOP)
	{
		//nextwork (kalau stop, trus apa)
		serWrite(BB.port,"#TS> OPERATION 0\r\n");
		serWrite(GS.port,"#TS> ack STOP \r\n");
	} else
	//--------------------------------------
	if (dat == RTB)
	{
		if (_debug) cout <<"RTB to "<<gParam.latRTB<< " "<<gParam.lonRTB<<endl;
		serWrite(GS.port,"#TS> ack RTB \r\n");
		
	} else 
	//-------------------------------------------------------------------------
	if (dat == PARAM)
	{
		gParam.set(data);
		if (_debug) gParam.print();
		//send data to BB
		
		char _parambuffer[200];
		int retVal = sprintf(_parambuffer,"#TS> PARAM %d %d %d %d %f %f \r\n",
								gParam.depthOperation, gParam.glidingAngle, 
								gParam.miniCT, gParam.oseRate,
								gParam.latRTB, gParam.lonRTB);
		
		serWrite(BB.port,_parambuffer);
		usleep(10000);
		serWrite(GS.port,"#TS> ack PARAM \r\n");
		usleep(10000);
		
	} else 
	//-------------------------------------------------------------------------
	if (dat == WAYPOINT)
	{
		gWPoint.set(data);
		if (_debug) gWPoint.print();
		//send data to BB
		
		char _waypointbuffer[200];
		int retVal = sprintf(_waypointbuffer,"#TS> WAYPOINT %f %f %f %f \r\n",
								gWPoint.latStart, gWPoint.lonStart,
								gWPoint.latEnd, gWPoint.lonEnd);
		
		serWrite(BB.port,_waypointbuffer);
		usleep(10000);
		serWrite(GS.port,"#TS> ack WAYPOINT \r\n");
		usleep(10000);
	} else
	//-------------------------------------------------------------------------
	if (dat == PING)
	{
		//cout << "debug : receive PING"<<endl;
		write2LOG("PING",const_cast<char*>(data.c_str()));
		sendPING(BB.port);
		usleep(10000);
		sendPING(AR.port);
		usleep(10000);
		sendPING(GS.port,ACK);
		usleep(10000);
	} else
	if (dat == DATA)
	{
    	handler_sensor = 0;
		handler_control = 0;
		handler_fsm = 0;
		stream_fsm = 0;
		stream_control = 0;
		stream_sensor = 0;
        cout << "Sending Datalog" << endl;
        ifstream readdatalog("datalog_sensor.txt");
        if(!readdatalog.is_open()) {
            cout << "Error : file is open" << endl;
        }

        while(readdatalog.good()) {
			/*revisi 08072020*/
            getline(readdatalog,datalog_day,' ');
            getline(readdatalog,datalog_mon,' ');
            getline(readdatalog,datalog_date,' ');
            getline(readdatalog,datalog_year,' ');
            getline(readdatalog,datalog_hour,':');
            getline(readdatalog,datalog_minute,':');
            getline(readdatalog,datalog_second,' ');
			
            getline(readdatalog,datalog_temperature,' ');
            getline(readdatalog,datalog_salinity,' ');
            getline(readdatalog,datalog_altitude,' ');
            getline(readdatalog,datalog_z_terukur,' '); //depth

            getline(readdatalog,datalog_latitude,' ');
            getline(readdatalog,datalog_longitude,' ');
            getline(readdatalog,datalog_roll,' ');
            getline(readdatalog,datalog_pitch,' ');

            getline(readdatalog,datalog_yaw_terukur,' ');
            getline(readdatalog,datalog_motion,' ');
            getline(readdatalog,datalog_battery,' ');
            getline(readdatalog,datalog_leakage,' ');

            getline(readdatalog,datalog_speed,' ');
            getline(readdatalog,datalog_strobo,' ');
            getline(readdatalog,datalog_boardTemp,' ');
            getline(readdatalog,datalog_alti,' ');

            getline(readdatalog,datalog_csum,'\n');
			char buffer[3000];
	/*			int n = sprintf (buffer,"ack DATA %s %s %s %s %s:%s:%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s \r\n",
				datalog_day,datalog_mon,datalog_date,datalog_year,datalog_hour,datalog_minute,datalog_second,datalog_temperature,datalog_salinity,
				datalog_altitude,datalog_z_terukur,datalog_latitude,datalog_longitude,datalog_roll,datalog_pitch,datalog_yaw_terukur,datalog_motion,
				datalog_battery,datalog_leakage,datalog_speed,datalog_strobo,datalog_csum);
	*/

//old
/*
			strcpy (buffer,("#TS> DATA "+datalog_day+" "+datalog_mon+" "+datalog_date+" "+datalog_year+" "+datalog_hour+":"+datalog_minute+":"+datalog_second+" "+
					datalog_pitch+" "+datalog_z_terukur+" "+datalog_yawref+" "+datalog_yaw_terukur+" "+datalog_pos_e+" "+datalog_pos_n+" "+datalog_altitude+" "+
					datalog_salinity+" "+datalog_temperature+" "+datalog_speed+" "+datalog_latitude+" "+datalog_longitude+" "+datalog_roll+" "+datalog_motion+" "+
					datalog_leakage+" "+datalog_strobo+" "+datalog_csum+" \r\n").c_str());
----------------------
			strcpy (buffer,("#TS> DATA "+datalog_day+" "+datalog_mon+" "+datalog_date+" "+datalog_year+" "+datalog_hour+":"+datalog_minute+":"+datalog_second+" "+
					datalog_pitch+" "+datalog_z_terukur+" "+datalog_yawref+" "+datalog_yaw_terukur+" "+datalog_pos_e+" "+datalog_pos_n+" "+datalog_altitude+" "+
					datalog_salinity+" "+datalog_temperature+" "+datalog_speed+" "+datalog_latitude+" "+datalog_longitude+" "+datalog_roll+" "+datalog_motion+" "+
					datalog_leakage+" "+datalog_strobo+" "+datalog_boardTemp+" "+datalog_alti+" "+datalog_csum+"\r\n").c_str());
*/
			strcpy (buffer,("#TS> DATA "+datalog_day+" "+datalog_mon+" "+datalog_date+" "+datalog_year+" "+datalog_hour+":"+datalog_minute+":"+datalog_second+" "+
					datalog_temperature+" "+datalog_salinity+" "+datalog_altitude+" "+datalog_z_terukur+" "+
					datalog_latitude+" "+datalog_longitude+" "+datalog_roll+" "+datalog_pitch+" "+
					datalog_yaw_terukur+" "+datalog_motion+" "+datalog_battery+" "+datalog_leakage+" "+
					datalog_speed+" "+datalog_strobo+" "+datalog_boardTemp+" "+datalog_alti+" "+
					datalog_csum+"\r\n").c_str());

			serWrite(GS.port,buffer);
            // cek kapabilitas peek
            if(readdatalog.peek() == EOF) {
                cout << "End of file reached" << endl;
                readdatalog.close();
            }
        }

		time_t tm_sys_com;
    	struct tm * com_systime;
		time(&tm_sys_com);
		com_systime = localtime(&tm_sys_com);
		char com_tm_info[50];
		strftime(com_tm_info, 50, "%a %b %d %Y %H:%M:%S", com_systime);
	

        ofstream datalog_fsm("log_state.txt", ios::out | ios::app);
        if(datalog_fsm.is_open()) {
            datalog_fsm << com_tm_info << " " << "State : Sending Datalog" << endl;
            datalog_fsm.close();
        }
        cout << "Sending Datalog Finished" << endl;	
	}
	
	
	//debug
	else
	if (dat == 101)
	{
		handler_sensor = !handler_sensor;
	}	
	else
	if (dat == 102)
	{
		handler_control = !handler_control;
	}	
	else
	if (dat == 103)
	{
		handler_fsm = !handler_fsm;
	}	
	else
	if (dat == 111)
	{
		stream_sensor = !stream_sensor;
	}	
	else
	if (dat == 112)
	{
		stream_control = !stream_control;
	}	
	else
	if (dat == 113)
	{
		stream_fsm = !stream_fsm;
	}	
	//debug
}

void command_line() {

	bool printaction = true;

	time_t tm_sys_com;
    struct tm * com_systime;
    time(&tm_sys_com);
    com_systime = localtime(&tm_sys_com);
    char com_tm_info[50];

	usleep(5000);
	//cout << "Waiting new command" << endl;
	
	char read_buffer[1000];
	memset(&read_buffer,'\0',sizeof(read_buffer));
	int bytes_read = read(GS.port,&read_buffer,sizeof(read_buffer));
	
	string mystr = convertToString(read_buffer);
	
	//	getline(cin, mystr);
	usleep(5000);
  	strftime(com_tm_info, 50, "%a %b %d %Y %H:%M:%S", com_systime);
	
	//.:.dpwdebug
	// add true to end of parameter to activate debug mode.
	//debug
//	cout << "debug : GCS : "<<mystr;
//	cout << "-------------------------GCSDATAEND"<<endl;
	handlerGCS(parsing_GCS(mystr),mystr,true);


	//:.:dpwdebug
	/*
	if(mystr == "#GB> AT") {
		cout << "ack OK" << endl;
	} else
	if(mystr == "#GB> RTB null") {

		cout << "ack RTB null" << endl;
	} else
		
	//dpwdebug
	if(mystr == "#GB> atc operation") {
		handler_sensor = 1;
		buf_str_fsm = 1;
	}
	else if(mystr == "#GB> atc glide") {
		handler_sensor = 1;
		buf_str_fsm = 2;
	}

	else if(mystr == "#GB> atc test prop") {
		// handler_fsm = 1;
		buf_str_fsm = 3;
	}

	else if(mystr == "#GB> atc test bow") {
		// handler_fsm = 1;
		buf_str_fsm = 4;
	}

	else if(mystr == "#GB> atc stop") {
		handler_sensor = 0;
		handler_control = 0;
		handler_fsm = 0;
		stream_fsm = 0;
		stream_control = 0;
		stream_sensor = 0;
	}

	else if(mystr == "#GB> test thread") {
		cout << "Check FSM thread, ID : " << fsm_task_flag<< endl;
		cout << "Check Control thread, ID : " <<control_task_flag << endl;
		cout << "Check Sensor thread, ID : " << sensor_task_flag<< endl;
	}
  else if(mystr == "#GB> atc senddata") {
    	handler_sensor = 0;
		handler_control = 0;
		handler_fsm = 0;
		stream_fsm = 0;
		stream_control = 0;
		stream_sensor = 0;
        cout << "Sending Datalog" << endl;
        ifstream readdatalog("datalog_sensor.txt");
        if(!readdatalog.is_open()) {
            cout << "Error : file is open" << endl;
        }

        while(readdatalog.good()) {
            getline(readdatalog,datalog_day,' ');
            getline(readdatalog,datalog_mon,' ');
            getline(readdatalog,datalog_date,' ');
            getline(readdatalog,datalog_year,' ');
            getline(readdatalog,datalog_hour,':');
            getline(readdatalog,datalog_minute,':');
            getline(readdatalog,datalog_second,' ');
            getline(readdatalog,datalog_pitch,' ');
            getline(readdatalog,datalog_z_terukur,' ');
            getline(readdatalog,datalog_yawref,' ');
            getline(readdatalog,datalog_yaw_terukur,' ');
            getline(readdatalog,datalog_pos_e,' ');
            getline(readdatalog,datalog_pos_n,' ');
            getline(readdatalog,datalog_altitude,' ');
            getline(readdatalog,datalog_salinity,' ');
            getline(readdatalog,datalog_temperature,' ');
            getline(readdatalog,datalog_speed,' ');
            getline(readdatalog,datalog_latitude,' ');
            getline(readdatalog,datalog_longitude,' ');
            getline(readdatalog,datalog_roll,' ');
            getline(readdatalog,datalog_motion,' ');
            getline(readdatalog,datalog_battery,' ');
            getline(readdatalog,datalog_leakage,' ');
            getline(readdatalog,datalog_strobo,' ');
            getline(readdatalog,datalog_csum,'\n');
            cout<<"#GB DAT "<<datalog_day<<" "<<datalog_mon<<" "<<datalog_date<<" "<<datalog_year<<" "<<datalog_hour<<":"<<datalog_minute<<":"<<datalog_second<<" "
            <<datalog_temperature<<" "<<datalog_salinity<<" "<<datalog_altitude<<" "<<datalog_z_terukur<<" "<<datalog_latitude<<" "<<datalog_longitude<<" "
            <<datalog_roll<<" "<<datalog_pitch<<" "<<datalog_yaw_terukur<<" "<<datalog_motion<<" "<<datalog_battery<<" "<<datalog_leakage<<" "<<datalog_speed<<" "
            <<datalog_strobo<<" "<<datalog_csum<<endl;

            // cek kapabilitas peek
            if(readdatalog.peek() == EOF) {
                cout << "End of file reached" << endl;
                readdatalog.close();
            }
        }

        ofstream datalog_fsm("log_state.txt", ios::out | ios::app);
        if(datalog_fsm.is_open()) {
            datalog_fsm << com_tm_info << " " << "State : Sending Datalog" << endl;
            datalog_fsm.close();
        }
        cout << "Sending Datalog Finished" << endl;
    }*/
}
