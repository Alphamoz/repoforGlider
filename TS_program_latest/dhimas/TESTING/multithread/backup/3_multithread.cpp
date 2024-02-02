/*
	backup 3
	CHECK ttg multithread sensor + control
	canonical serial
	
*/

/*
Program khusus operasional glider BMKG
*/

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

//extended lib declaration
gliderParam gParam;
gliderWP gWPoint;


using namespace std;

//dpwdebug
	bool printstatFSM = true;
//dpwdebug


//dpw komunikasi
int openport(char* portname)
{
	/*note:
		Serial config is 9600 8n1
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
    newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B9600; // 8 databits
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~(PARENB | PARODD); // No parity
    newtio.c_cflag &= ~CRTSCTS; // No hardware handshake
    newtio.c_cflag &= ~CSTOPB; // 1 stopbit
    newtio.c_iflag = IGNBRK;
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // No software handshake

  //newtio.c_iflag &= ~(IXANY); // No software handshake
  //newtio.c_iflag |= (IXON | IXOFF ); // software handshake

    newtio.c_lflag = 0;
    newtio.c_oflag = 0;
	
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
	
}BB,Arduino;
//dpw komunikasi

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
void handlingBB(int _fd);
void handling_PID();
void handling_FSM();
void tempValue(); //redundant

void protocol_operation(int fd);
void command_line();
void PID_heading_rudder();
void PID_pitch_ballast();
void PID_buoyancy_bladder();

//void FSM
void start_operation();
void stop_operation();
void glide_operation();
void descent_state();
void ascent_state();
void surface_state();
void bowthruster_operation();
void bowthruster_on();
void bowthruster_off();
void propeller_operation();
void manual_mode();

//inisialisasi port komunikasi
int fd,fd1, fd2;
const char *portname1 = "/dev/ttyxuart0";
const char *portname2 = "/dev/ttyxuart1";
int wlen1, wlen2;

int set_interface_attribs1();
int set_interface_attribs2();
void set_mincount(int fd, int mcount);
int open_port();

string mystr;
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

float motion = 1, battery = 70.0, leakage = 0, strobo = 1, csum = 0;

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


int main(void) {
	pthread_t sensor_read;
	pthread_t control_PID;
	pthread_t fsm_handle;

	cout << "PROGRAM START" << endl;

	int rc;
	int i = 7;

	pthread_mutex_init(&lock_flag, NULL);

/*	
	fd1 = open(portname1, O_RDWR | O_NOCTTY | O_SYNC);
	fd = fd1;
	cout << "fd1 open" << endl;
	if(fd1 < 0) {
		printf("Error opening %s: %s\n", portname1, strerror(errno));
		return -1;
	}

	fd2 = open(portname2, O_RDWR | O_NOCTTY | O_SYNC);
	cout << "fd2 open" << endl;
	if(fd2 < 0) {
		printf("Error opening %s: %s\n", portname2, strerror(errno));
		return -1;
	}

	//properties communication (baud, timing)
	set_interface_attribs1();
	set_interface_attribs2();

*/
	//.:.dpw comm
	BB.loc = "/dev/ttyxuart0";
	BB.port = openport(BB.loc);
	if (!BB.isSet()) cout <<BB.loc<<" OPEN ERROR"<<endl;
	fd2 = BB.port;
	
	Arduino.loc = "/dev/ttyxuart1";
	Arduino.port = openport(Arduino.loc);
	if (!Arduino.isSet()) cout <<Arduino.loc<<" OPEN ERROR"<<endl;
	fd = Arduino.port;
	//:.:dpw comm

	//Sub thread init
	rc = pthread_create(&fsm_handle, NULL, fsm_task, NULL);
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

	while(comm_task_flag == true) {
		command_line();

		if(handler_sensor == true) {
			stream_sensor = 1;
		}

		if(handler_control == true) {
			stream_control = 1;
		}

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
		handling_FSM();
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
		handling_PID();
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
		handlingBB(fd2);
		sleep(1);
	}
	pthread_exit(NULL);
}

int set_interface_attribs1()
{
    struct termios oldtio,newtio;
    int serial_fd;

    if ((serial_fd = open("/dev/ttyxuart1", O_RDWR | O_EXCL | O_NDELAY)) == -1) 
    {
        cout << "unable to open" << endl;
        return -1;
    }
    if (tcgetattr(serial_fd, &oldtio) == -1) {
        cout << "tcgetattr failed" << endl;
        return -1;
    }

    cfmakeraw(&newtio); // Clean all settings
    newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B9600; // 8 databits
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~(PARENB | PARODD); // No parity
    newtio.c_cflag &= ~CRTSCTS; // No hardware handshake
    newtio.c_cflag &= ~CSTOPB; // 1 stopbit
    newtio.c_iflag = IGNBRK;
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // No software handshake
    newtio.c_lflag = 0;
    newtio.c_oflag = 0;
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN] = 60;

    if (tcsetattr(serial_fd, TCSANOW, &newtio) == -1) {
        cout << "tcsetattr failed" << endl;
        return -1;
    }

    tcflush(serial_fd, TCIOFLUSH); // Clear IO buffer
    return serial_fd;
}

int set_interface_attribs2()
{
    struct termios oldtio,newtio;
    int serial_fd;
    if ((serial_fd = open("/dev/ttyxuart0", O_RDWR | O_EXCL | O_NDELAY)) == -1) 
    {
        cout << "unable to open" << endl;
        return -1;
    }
    if (tcgetattr(serial_fd, &oldtio) == -1) {
        cout << "tcgetattr failed" << endl;
        return -1;
    }

    cfmakeraw(&newtio); // Clean all settings
    newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B9600; // 8 databits
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~(PARENB | PARODD); // No parity
    newtio.c_cflag &= ~CRTSCTS; // No hardware handshake
    newtio.c_cflag &= ~CSTOPB; // 1 stopbit
    newtio.c_iflag = IGNBRK;
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // No software handshake
    newtio.c_lflag = 0;
    newtio.c_oflag = 0;
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN] = 60;
    
    if (tcsetattr(serial_fd, TCSANOW, &newtio) == -1) {
        cout << "tcsetattr failed" << endl;
        return -1;
    }

    tcflush(serial_fd, TCIOFLUSH); // Clear IO buffer
    return serial_fd;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}

// Buka Port serial
int open_port()
{
    struct termios oldtio,newtio;
    int serial_fd;
    if ((serial_fd = open("/dev/ttyxuart1", O_RDWR | O_EXCL | O_NDELAY)) == -1) {
            cout << "unable to open" << endl;
            return -1;
    }
    if (tcgetattr(serial_fd, &oldtio) == -1) {
            cout << "tcgetattr failed" << endl;
            return -1;
    }
    cfmakeraw(&newtio); // Clean all settings
    newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B9600; // 8 databits
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~(PARENB | PARODD); // No parity
    newtio.c_cflag &= ~CRTSCTS; // No hardware handshake
    newtio.c_cflag &= ~CSTOPB; // 1 stopbit
    newtio.c_iflag = IGNBRK;
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // No software handshake
    newtio.c_lflag = 0;
    newtio.c_oflag = 0;
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN] = 60;
    if (tcsetattr(serial_fd, TCSANOW, &newtio) == -1) {
            cout << "tcsetattr failed" << endl;
            return -1;
    }
    tcflush(serial_fd, TCIOFLUSH); // Clear IO buffer
    return serial_fd;
}

void PID_heading_rudder() {
	cout << "PID rudder running" << endl;
	heading_ref = yawref;
	heading_fb = temp_yaw;
	heading_ref = (heading_ref*3.1416)/180.0;
	heading_fb = (heading_fb*3.1416)/180.0;
	heading_laster = heading_err;
	heading_err = heading_err - heading_fb;
	heading_sumerr = heading_sumerr + heading_err + rd_AW;

	PID_rd_pro = heading_err * PID_rd_KP;
	PID_rd_int = heading_sumerr * PID_rd_KI * sampling_time;
	PID_rd_dev = (heading_err - heading_laster) * PID_rd_KD / sampling_time;
	PID_rd_cal = PID_rd_pro + PID_rd_int + PID_rd_dev;

	if(PID_rd_cal > sat_rd_u) {
		PID_rd_out = sat_rd_u;
	}
	else if(PID_rd_cal < sat_rd_d) {
		PID_rd_out = sat_rd_d;
	}
	else {
		PID_rd_out = PID_rd_cal;
	}

	rd_AW = (PID_rd_out - PID_rd_cal) * PID_rd_AW;
	rd_PID_val = PID_rd_out;

	if(PID_rd_out != 0) {
		cout << "PID rudder : " << PID_rd_out << endl;
	}
	else if(PID_rd_out == 0) {
		cout << "PID rudder : 0" << endl;
	}
}

void PID_pitch_ballast() {
	cout << "PID ballast running" << endl;
	pitch_ref = ref_pitch;
	printf("pitch_ref : %f\n", pitch_ref);
	pitch_fb = temp_pitch;
	pitch_lasterr = pitch_err;
	pitch_err = pitch_ref - pitch_fb;
	printf("pitch_err : %f\n", pitch_err);
	pitch_sumerr = pitch_sumerr + pitch_err + blst_AW;

	PID_blst_pro = pitch_err * PID_blst_KP;
	PID_blst_int = pitch_sumerr * PID_blst_KI * sampling_time;
	PID_blst_dev = (pitch_err - pitch_lasterr) * PID_blst_KD / sampling_time;
	PID_blst_cal = PID_blst_pro + PID_blst_int + PID_blst_dev;
	printf("blst_cal : %f\n", PID_blst_cal);

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

	if(PID_blst_out != 0) {
		cout << "PID ballast : " << PID_blst_out << endl;
	}
	else if(PID_blst_out == 0) {
		cout << "PID ballast : 0" << endl;
	}
}

void PID_buoyancy_bladder() {
	cout << "PID bladder running" << endl;
	depth_ref = ref_depth;
	printf("depth_ref %f\n", depth_ref);
	depth_fb = temp_depth;
	depth_lasterr = depth_err;
	depth_err = depth_ref + depth_fb;
	printf("depth_err %f\n", depth_err);
	depth_sumerr = depth_sumerr - depth_err + bld_AW;

	PID_bld_pro = depth_err * PID_bld_KP;
	PID_bld_int = depth_sumerr * PID_bld_KI * sampling_time;
	PID_bld_dev = (depth_err - depth_lasterr) * PID_bld_KD / sampling_time;
	PID_bld_cal = PID_bld_pro + PID_bld_int + PID_bld_dev;
	printf("bld_cal %f\n", PID_bld_cal);
	
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

	if(PID_bld_out != 0) {
		cout << "PID bld out thread BB : " << PID_bld_out << endl;
		depth_lasterr = depth_err;
	}
	else if(PID_bld_out == 0) {
		cout << "PID bld thread BB : 0" << endl;
	}
}

void handlingBB(int _fd) {
	while(stream_sensor == true) {
		pthread_mutex_lock(&lock_flag);
		printf("Mutex locked by sensor\n");
		cout << "Parsing BB start" << endl;
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

	    fd_set readfs;
	    timeval tv;
	    tv.tv_sec = 0;
	    tv.tv_usec = 500000;
	    char * BBinbound = new char[2000];
		usleep(500000);
		cout << "START READING BB" << endl;
		write(fd2, "$#TS ", 5);
	    int r = read(fd2, BBinbound, 2000);
		cout << "FINISH READING BB with :" << r <<" characters"<<endl;
	    BBinbound[r] = '\0';
	    cout << BBinbound << endl;

	    string my_string (BBinbound, BBinbound+r);
	    if(my_string[0] == '$')
	    {
	        if(my_string[1] == '#')
	        {
	            if(my_string[2] == 'N')
	            {
	                if(my_string[3] == 'G')
	                {
	                    char * comBBin;
	                    comBBin = strtok(BBinbound, " ");
	                    cout << "Start Parsing" << endl;
	                    cout << comBBin << endl;
	                    BBcount = 0;
	                    while(comBBin != NULL)
	                    {
	                        switch(BBcount) { //untuk demo 7 Des, printf parsing harus hilang
	                            default:

	                            case 0 :
	                            {
	                                strncpy(comBBheader, comBBin,4);
	                                cout << "Masuk" << endl;
	                                printf("Header : %s\n", comBBheader);
	                            }
	                            BBcount++;
	                            break;

	                            case 1 :
	                            {
	                                strcpy(comBB_1, comBBin);
	                                teta_terukur = atof(comBB_1);
	                                printf("Teta terukur : %09.6f\n", teta_terukur);
	                            }
	                            BBcount++;
	                            break;

	                            case 2 :
	                            {
	                                strcpy(comBB_2, comBBin);
	                                z_terukur = atof(comBB_2);
	                                printf("Z terukur : %05.2f\n", z_terukur);
	                            }
	                            BBcount++;
	                            break;

	                            case 3 :
	                            {
	                                strcpy(comBB_3, comBBin);
	                                yawref = atof(comBB_3);
	                                printf("Yaw ref : %09.6f\n", yawref);
	                            }
	                            BBcount++;
	                            break;

	                            case 4 :
	                            {
	                                strcpy(comBB_4, comBBin);
	                                yaw_terukur = atof(comBB_4);
	                                printf("Yaw terukur : %09.6f\n", yaw_terukur);
	                            }
	                            BBcount++;
	                            break;

	                            case 5 :
	                            {
	                                strcpy(comBB_5, comBBin);
	                                pos_e = atof(comBB_5);
	                                printf("Pos N : %9.6f\n", pos_n);
	                            }
	                            BBcount++;
	                            break;

	                            case 6 :
	                            {
	                                strcpy(comBB_6, comBBin);
	                                pos_n = atof(comBB_6);
	                                printf("Pos E : %9.6f\n", pos_e);
	                            }
	                            BBcount++;
	                            break;

	                            case 7 :
	                            {
	                                strcpy(comBB_7, comBBin);
	                                alt = atof(comBB_7);
	                                printf("Altitude : %05.2f\n", alt);
	                            }
	                            BBcount++;
	                            break;

	                            case 8 :
	                            {
	                                strcpy(comBB_8, comBBin);
	                                sal = atof(comBB_8);
	                                printf("Salinity : %05.2f\n", sal);
	                            }
	                            BBcount++;
	                            break;

	                            case 9 :
	                            {
	                                strcpy(comBB_9, comBBin);
	                                temp = atof(comBB_9);
	                                printf("Temperature : %05.2f\n", temp);
	                            }
	                            BBcount++;
	                            break;

	                            case 10 :
	                            {
	                                strcpy(comBB_10, comBBin);
	                                spd = atof(comBB_10);
	                                printf("Speed : %04.1f\n", spd);
	                            }
	                            BBcount++;
	                            break;

	                            case 11 :
	                            {
	                                strcpy(comBB_11, comBBin);
	                                lat = atof(comBB_11);
	                                printf("Latitude : %010.7f\n", lat);
	                            }
	                            BBcount++;
                            	break;

	                            case 12 :
	                            {
	                                strcpy(comBB_12, comBBin);
	                                lon = atof(comBB_12);
	                                printf("Longitude : %010.7f\n", lon);
	                            }
	                            BBcount++;
                            	break;
	                                                            
	                            case 13 :
	                            {
	                                strcpy(comBB_13, comBBin);
	                                roll =  atof(comBB_13);
	                                printf("Rolls : %09.6f\n", roll);
	                            }
	                            BBcount++;
	                            break;
	                                                                
	                            // default :
	                            //     break;
	                        }
							comBBin = strtok(NULL, " ");
	                    }
						global_depth = z_terukur;
						global_pitch = teta_terukur;
						global_heading = yaw_terukur;

						if(global_depth <= -1*depth_threshold || global_depth == -1*depth_target) {
							ascent_flag = 1;
							descent_flag = 0;
							pitch_up_flag = 1;
							pitch_down_flag = 0;
						}

						else if(global_depth >= surface_depth) {
							descent_flag = 1;
							ascent_flag = 0;
							pitch_up_flag = 0;
							pitch_down_flag = 1;
						}

						printf("HANDLING BB : ascent_flag : %d, descent_flag : %d\n", ascent_flag, descent_flag);
						ofstream logflag("flag_logger.txt", ios::out | ios::app);
							if(logflag.is_open())
							{
								logflag << buftimeBB << "ascent_flag : " << ascent_flag << ", descent_flag : " << descent_flag
								<< ", pitch_up_flag : " << pitch_up_flag << ", pitch_down_flag : " << pitch_down_flag
								<< ", global_depth : " << global_depth << ", global_pitch : " << global_pitch << endl;
							}
							
						if(BBcount >= 13)
						{
							ofstream savedatalog("datalog_sensor.txt", ios::out | ios::app);
							if(savedatalog.is_open())
							{
								savedatalog << buftimeBB << ' ' << teta_terukur << ' ' << z_terukur << ' ' << yawref << ' ' << yaw_terukur << ' ' << pos_e << ' ' <<
								pos_n << ' ' << alt << ' ' << sal << ' ' << temp << ' ' << spd << ' ' << lat << ' ' << lon << ' ' << roll << ' ' << motion << ' ' << 
								leakage << ' ' << strobo << ' ' << csum << ' ' << endl;
								cout << "save success" << endl;
								savedatalog.close();
								BBcount = 0;
								
							}
							usleep(500000);
							// BBcount = 0;
						}
	                }
	            }
	        }
	    }
		
		delete [] BBinbound;
	    tcflush(fd2, TCIOFLUSH);
	    tcdrain(fd2);
		
		pthread_mutex_unlock(&lock_flag);
		printf("Mutex unlocked by sensor\n");
		
	    usleep(500000);

	    cout << "Parsing BB Finished" << endl;
		//handler_control = 1; 
		cout << "changing stream control in handlingBB" << endl;
		stream_control = 1;
		// sleep(2);   
	    ofstream logparsingBB("state_function_thread.txt", ios::out | ios::app);
	    if(logparsingBB.is_open()) {
	        logparsingBB << buftimeBB << " | " << "Parsing Finished" << endl;
	        logparsingBB.close();
	    }
	    cout << "BBcount final : " << BBcount << endl;
	}
	usleep(500000);
}

void tempValue() {
	temp_depth = global_depth;
	temp_pitch = global_pitch;
	temp_yaw = global_heading;
}

void handling_PID() {
	time_t tm_control;
    struct tm * control_systime;
    time(&tm_control);
    control_systime = localtime(&tm_control);
    char control_tm_info[50];
    strftime(control_tm_info, 50, "%a %b %d %Y %H:%M:%S", control_systime);
	if (printstatFSM) printf("stream_sensor %d, stream_control %d\n", stream_sensor, stream_control);
	if(stream_control == true) {
		ofstream logstatecontrol("state_function_thread.txt", ios::out | ios::app);
	    if(logstatecontrol.is_open()) {
	        logstatecontrol << control_tm_info << " | " << "Control Running" << endl;
	        logstatecontrol.close();
	    }
		if (printstatFSM) cout << "Control System Start" << endl;
		printf("HANDLING PID : ascent_flag : %d, descent_flag : %d\n", ascent_flag, descent_flag);
		// handler_fsm = 1;
		//pthread_mutex_lock(&lock_flag);
		printf("Mutex locked by control\n");
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
		printf("Mutex unlocked by control\n");

		stream_fsm = 1;
		
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
		stream_control = 0;
	}
	else {
		if (printstatFSM) cout << "Control System OFF" << endl;
		ofstream logcontroloff("state_function_thread.txt", ios::out | ios::app);
	    if(logcontroloff.is_open()) {
	        logcontroloff << control_tm_info << " | " << "Control OFF" << endl;
	        logcontroloff.close();
	    }
	}
	usleep(500000);
}

void protocol_operation(int fd) {
	time_t tm_op;
    struct tm * op_systime;
    time(&tm_op);
    op_systime = localtime(&tm_op);
    char op_sysinfo[50];
    strftime(op_sysinfo, 50, "%a %b %d %Y %H:%M:%S", op_systime);

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

		printf("bld_val %s, blst_val %s, char_teta %s, rd_val %s, char_pos_e %s, char_pos_n %s, char_strobe %s\n", bld_val, blst_val, char_teta, rd_val, char_pos_e, char_pos_n, char_strobe);
	    // sleep(1);	    
	    cout << "Sending data to Arduino, full control" << endl;
	    usleep(250000);

	    write(fd, "$#TS ", 5);
        write(fd, bld_val, 5);  
        write(fd, " ", 1);
        write(fd, blst_val, 5);
        write(fd, " ", 1);
        write(fd, char_teta, 10);
        write(fd, " ", 1);
        write(fd, rd_val, 5);
        write(fd, " ", 1);
        write(fd, char_pos_e, 10);
        write(fd, " ", 1);
        write(fd, char_pos_n, 10);
        write(fd, " ", 1);
        write(fd, char_strobe, 1);
        write(fd, "\r\n", 2);
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
	   	tcdrain(fd);
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
			ascent_state();
			// sleep(1);
			protocol_operation(fd);
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
			descent_state();
			// sleep(1);
			protocol_operation(fd);
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

void stop_operation() {
    time_t tm_sys_stop;
    struct tm * stop_systime;
    time(&tm_sys_stop);
    stop_systime = localtime(&tm_sys_stop);
    char stop_tm_info[50];
    strftime(stop_tm_info, 50, "%a %b %d %Y %H:%M:%S", stop_systime);

    bld_PID_val = bld_surface;
    blst_PID_val = blst_surface;
    rd_PID_val = rd_surface;
    prop_val = prop_val_off;
    bow_val = bow_val_off;
    strobe_val = strobe_on;
    
    while(global_depth < surface_depth) {
    	pitch_up_flag = 0;
    	pitch_down_flag = 0;
    	ref_pitch = surface_pitch;
        protocol_operation(fd);
    }

    stream_control = 0;
    stream_sensor = 0;

    descent_flag = 0;
    ascent_flag = 0;
    ngliding = 0;

    if(stream_fsm == true) {
	    ofstream com_stop("com_log.txt", ios::out | ios::app);
	    if(com_stop.is_open()) {
	    	write(fd2, "$#O0", 4);
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

void descent_state() {
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

void ascent_state() {
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
    	protocol_operation(fd);
    	ofstream surface_fsm("log_state.txt", ios::out | ios::app);
	    if(surface_fsm.is_open()) {
	        surface_fsm << surface_tm_info << " " << "State : Surfacing" << endl;
	        surface_fsm.close();
	    }
	    cout << "Surfacing" << endl;
    }
}

void bowthrusters_on() {
    bld_PID_val = bld_surface;
    blst_PID_val = blst_surface;
    rd_PID_val = rd_surface;
    prop_val = prop_val_off;
    bow_val = bow_val_on;
    strobe_val = strobe_on;
    protocol_operation(fd);

    cout << "Bowthrusters ON" << endl;
}

void bowthrusters_off() {
    bld_PID_val = bld_surface;
    blst_PID_val = blst_surface;
    rd_PID_val = rd_surface;
    prop_val = prop_val_off;
    bow_val = bow_val_off;
    strobe_val = strobe_on;
    protocol_operation(fd);
	
    cout << "Bowthrusters OFF" << endl;
}

void bowthrustersOperation() {
    time_t tm_sys_bow;
    struct tm * bow_systime;
    time(&tm_sys_bow);
    bow_systime = localtime(&tm_sys_bow);
    char bow_tm_info[50];
    strftime(bow_tm_info, 50, "%a %b %d %Y %H:%M:%S", bow_systime);

    if(handler_protocol == true) {
	    new_heading = yaw_terukur - 180.0;
	    if(new_heading != yawref) {
	        bowthrusters_on();
	        ofstream bow_on_fsm("log_state.txt", ios::out | ios::app);
	        if(bow_on_fsm.is_open()) {
	            bow_on_fsm << bow_tm_info << " " << "State : Bowthrusters ON" << endl;
	            bow_on_fsm.close();
	        }
	    }
	    else if(new_heading == yawref) {
	        bowthrusters_off();
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
    protocol_operation(fd);

    ofstream prop_on_fsm("log_state.txt", ios::out | ios::app);
    if(prop_on_fsm.is_open()) {
        prop_on_fsm << prop_tm_info << " " << "State : Propeller ON" << endl;
        prop_on_fsm.close();
    }
    cout << "Propeller On" << endl;

	sleep(20);
    prop_val = prop_val_off;
    protocol_operation(fd);

    ofstream prop_off_fsm("log_state.txt", ios::out | ios::app);
    if(prop_off_fsm.is_open()) {
        prop_off_fsm << prop_tm_info << " " << "State : Propeller OFF" << endl;
        prop_off_fsm.close();
    }
    cout << "Propeller OFF" << endl;
}

// Protokol berubah dari TS
// $#TS vBd mBall errteta dRud 
void handling_FSM() {
	time_t tm_sys_fsm;
    struct tm * fsm_systime;
    time(&tm_sys_fsm);
    fsm_systime = localtime(&tm_sys_fsm);
    char fsm_tm_info[50];
    strftime(fsm_tm_info, 50, "%a %b %d %Y %H:%M:%S", fsm_systime);
    mode = buf_str_fsm;

    if (printstatFSM) printf("mode : %d\n", mode);

	switch(mode) {
		case 1 : //operation
			{
				ofstream opstate("log_state.txt", ios ::out | ios::app);
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
						bowthrustersOperation();
						propellerOperation();
						stop_operation();
						ngliding = 0;
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
						stop_operation();
						ngliding = 0;
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
				stop_operation();
			}
			break;

		case 4 : //bowthruster test
			{
				startOperation();
                while(bowCount < 10) {
                    bowthrusters_on();
                    bowCount++;
                    usleep(500000);
                }
                // handlingBB();
                // sleep(30);
                bowthrusters_off();
                usleep(500000);
                // bowthrustersOperation();
                stop_operation();
				bowCount = 0;
			}
			break;

		case 5 : //manual op
			{
				startOperation();
				sleep(5);
				stop_operation();
			}
			break;

		default :
			break;
	}
}

void printcurrentstat()
{
	cout << "handler sensor : "<<handler_sensor<<endl;
	cout << "handler control : "<<handler_control<<endl;
	cout << "handler fsm : "<<handler_fsm<<endl;

	cout << "stream sensor : "<<stream_sensor<<endl;
	cout << "stream control : "<<stream_control<<endl;
	cout << "stream fsm : "<<stream_fsm<<endl;
	
	cout << "mode : "<<mode<<endl;
}

void handlerGCS(int dat,string data, bool _debug = false)
{
	if (dat == AT)
	{
		printcurrentstat();
	} else
	//-------------------------------------------------------------------------
	if (dat == START)
	{
		//check for RTB/RTO/test propeller/test bowthruster
		if (int c = gParam.subProcessExist())
		{
			cout << "unable to start : " << "pending task code "<<c<<endl;
			if (c == 14) //test prop 
			{
				handler_fsm = 1;
				buf_str_fsm = 3;
			}
			else
			if (c == 15) //test bow
			{
				handler_fsm = 1;
				buf_str_fsm = 4;
			}
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
	
	getline(cin, mystr);
	usleep(5000);
    strftime(com_tm_info, 50, "%a %b %d %Y %H:%M:%S", com_systime);
	
	//.:.dpwdebug
	// add true to end of parameter to activate debug mode.
	handlerGCS(parsing_GCS(mystr,true),mystr,true);





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
            cout<<"$#GB "<<datalog_day<<" "<<datalog_mon<<" "<<datalog_date<<" "<<datalog_year<<" "<<datalog_hour<<":"<<datalog_minute<<":"<<datalog_second<<" "
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
