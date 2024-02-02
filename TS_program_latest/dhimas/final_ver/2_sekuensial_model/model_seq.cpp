/*
Program khusus HILS, 
*/

#include <cstring>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <time.h>
#include <stdio.h>
// #include <pthread.h>
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

using namespace std;

bool comm_task_flag;

bool descent_flag;
bool ascent_flag;

bool stream_sensor;

void handlingBB(int _fd);
void handlingPID();
void handlingFSM();
void tempValue();

bool *buf_c;
bool *buf_s;

int fd, fd2;
char *portname = "/dev/ttyxuart1";
char *portname2 = "/dev/ttyxuart0";
int wlen, wlen2;

string mystr;
string fsm_input;
int buf_str_fsm;
int mode;
int BBcount=0;

int set_interface_attribs();
int set_interface_attribs2();
void set_mincount(int fd, int mcount);
int open_port();

void protocol_operation(int _fd);

void commandLine();
void PID_rd();
void PID_bld();
void PID_blst();

void startOperation();
void descentValue();
void ascentValue();
void surfaceValue();
void bowthrustersOperation();
void bowthrusters_on();
void bowthrusters_off();
void propellerOperation();
void stopOperation();

char rd_val[6];
char bld_val[6];
char blst_val[6];
char char_prop[4];
char char_bow[4];
char char_strobe[1];

float rd_PID_val, bld_PID_val, blst_PID_val;
float man_bld_val, man_blst_val, man_rd_val;
float man_bld_des, man_bld_asc, man_bld_net; 
float man_blst_des, man_blst_asc, man_blst_net;
float new_heading;
int ngliding, nglided;

// teta_terukur=pitch , z_terukur=depth, pos_n=reference north, pos_e=reference east
float teta_terukur, z_terukur, yawref, yaw_terukur, pos_e, pos_n, alt, sal,
	  temp, spd, lat, lon, roll;

float global_heading, global_pitch, global_depth;

float surface_depth = 0, depth_threshold = 17, depth_target = 20;

float motion=1, battery=70.0, leakage=0, strobo=1, csum=0;

float temp_yaw, temp_pitch, temp_depth;

int sampling_time = 1;

int data_entry, data_limit = 1000;

// variable PID rudder
float sat_rd_u = 0.5, sat_rd_d = -0.5,
	  heading_ref, heading_fb,
	  heading_err = 0, heading_sumerr = 0,
	  heading_laster, PID_rd_out,
	  PID_rd_cal, PID_rd_unsat,
	  rd_AW = 0;

float PID_rd_KP = 3,
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

// float PID_blst_KP = 3,
// 	  PID_blst_KI = 1,
// 	  PID_blst_KD = 0.1,
// 	  PID_blst_AW = 1;

float PID_blst_KP = -2.5,
	  PID_blst_KI = 0,
	  PID_blst_KD = -0.7,
	  PID_blst_AW = 1;

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
	bow_val_on = 1800, bow_val_off = 0,
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

	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    cout << "fd open" << endl;
	if(fd < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		return -1;
	}

	fd2 = open(portname2, O_RDWR | O_NOCTTY | O_SYNC);
    cout << "fd2 open" << endl;
	if(fd2 < 0) {
		printf("Error opening %s: %s\n", portname2, strerror(errno));
		return -1;
	}

	// Comm properties
	set_interface_attribs();
	set_interface_attribs2();

	comm_task_flag = 1;

	while(comm_task_flag == true) {
    	commandLine();
		sleep(1);
    }

	sleep(1);
    // cout << "sleep done" << endl;
    cout << "END PROGRAM" << endl;
    // control_task_flag = 0;
	pthread_exit(NULL);
    cout << "Main thread exit" << endl;
    return 0;
}

int set_interface_attribs()
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

void PID_rd() {
	cout << "PID rudder running" << endl;
	heading_ref = yawref;
	heading_fb = temp_yaw;
	heading_ref = heading_ref * 3.1416 / 180.0;
	heading_fb = heading_fb * 3.1416 / 180.0;
	heading_laster = heading_err;
	heading_err = heading_ref - heading_fb;
	heading_sumerr = heading_sumerr + heading_err + rd_AW;

	PID_rd_pro = heading_err * PID_rd_KP;
	PID_rd_int = heading_sumerr * PID_rd_KI * sampling_time;
	PID_rd_dev = (heading_err - heading_laster) * PID_rd_KD / sampling_time;
	PID_rd_cal = PID_rd_pro + PID_rd_int + PID_rd_dev;

	if (PID_rd_cal > sat_rd_u) {
		PID_rd_out = sat_rd_u;
	}	

	else if (PID_rd_cal < sat_rd_d) {
		PID_rd_out = sat_rd_d;		
	}

	else {
		PID_rd_out = PID_rd_cal;
	}

	rd_AW = (PID_rd_out - PID_rd_cal) * PID_rd_AW;
	rd_PID_val = PID_rd_out;
	
	if(PID_rd_out != 0) {
		cout << "PID rd out thread BB : " << PID_rd_out << endl;
	}

	else if(PID_rd_out == 0) {
		cout << "PID rd thread BB : 0" << endl;
	}
}

void PID_blst() {
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

	blst_AW = (PID_blst_out - PID_blst_cal) * PID_blst_AW;
	blst_PID_val = PID_blst_out;
	
	if(PID_blst_out != 0) {
		cout << "PID blst out thread BB : " << PID_blst_out << endl;
	}

	else if(PID_blst_out == 0) {
		cout << "PID blst thread BB : 0" << endl;
	}
}

void PID_bld() {
	cout << "PID bladder running" << endl;
	depth_ref = ref_depth;
	printf("depth_ref %f\n", depth_ref);
	depth_fb = temp_depth;
	depth_lasterr = depth_err;
	depth_err = depth_ref + depth_fb;
	printf("depth_err %f\n", depth_err);
	depth_sumerr = depth_sumerr + depth_err + bld_AW;

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

	bld_AW = (PID_bld_out - PID_bld_cal) * PID_bld_AW;
	bld_PID_val = PID_bld_out;
	
	if(PID_bld_out != 0) {
		cout << "PID bld out thread BB : " << PID_bld_out << endl;
	}

	else if(PID_bld_out == 0) {
		cout << "PID bld thread BB : 0" << endl;
	}
}

void handlingBB(int _fd) {
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
    char * BBinbound = new char[900];
    
    usleep(500000);
    int r = read(fd2, BBinbound, 900);
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
                            // default:

                            case 0 :
                            {
                                strncpy(comBBheader, comBBin,4);
                                cout << "Masuk" << endl;
                                printf("Header : %s\n", comBBheader);
                            	BBcount++;
                            }
                            cout << "BBcount : " << BBcount << endl;
                            break;

                            case 1 :
                            {
                                strcpy(comBB_1, comBBin);
                                teta_terukur = atof(comBB_1);
                                printf("Teta terukur : %09.6f\n", teta_terukur);
                            }
                            BBcount++;
                            cout << "BBcount : " << BBcount << endl;
                            break;

                            case 2 :
                            {
                                strcpy(comBB_2, comBBin);
                                z_terukur = atof(comBB_2);
                                printf("Z terukur : %05.2f\n", z_terukur);
                            }
                            BBcount++;
                            cout << "BBcount : " << BBcount << endl;
                            break;

                            case 3 :
                            {
                                strcpy(comBB_3, comBBin);
                                yawref = atof(comBB_3);
                                printf("Yaw ref : %09.6f\n", yawref);
                            }
                            BBcount++;
                            cout << "BBcount : " << BBcount << endl;
                            break;

                            case 4 :
                            {
                                strcpy(comBB_4, comBBin);
                                yaw_terukur = atof(comBB_4);
                                printf("Yaw terukur : %09.6f\n", yaw_terukur);
                            }
                            BBcount++;
                            cout << "BBcount : " << BBcount << endl;
                            break;

                            case 5 :
                            {
                                strcpy(comBB_5, comBBin);
                                pos_e = atof(comBB_5);
                                printf("Pos N : %9.6f\n", pos_n);
                            }
                            BBcount++;
                            cout << "BBcount : " << BBcount << endl;
                            break;

                            case 6 :
                            {
                                strcpy(comBB_6, comBBin);
                                pos_n = atof(comBB_6);
                                printf("Pos E : %9.6f\n", pos_e);
                            }
                            BBcount++;
                            cout << "BBcount : " << BBcount << endl;
                            break;

                            case 7 :
                            {
                                strcpy(comBB_7, comBBin);
                                alt = atof(comBB_7);
                                printf("Altitude : %05.2f\n", alt);
                            }
                            BBcount++;
                            cout << "BBcount : " << BBcount << endl;
                            break;

                            case 8 :
                            {
                                strcpy(comBB_8, comBBin);
                                sal = atof(comBB_8);
                                printf("Salinity : %05.2f\n", sal);
                            }
                            BBcount++;
                            cout << "BBcount : " << BBcount << endl;
                            break;

                            case 9 :
                            {
                                strcpy(comBB_9, comBBin);
                                temp = atof(comBB_9);
                                printf("Temperature : %05.2f\n", temp);
                            }
                            BBcount++;
                            cout << "BBcount : " << BBcount << endl;
                            break;

                            case 10 :
                            {
                                strcpy(comBB_10, comBBin);
                                spd = atof(comBB_10);
                                printf("Speed : %04.1f\n", spd);
                            }
                            BBcount++;
                            cout << "BBcount : " << BBcount << endl;
                            break;

                            case 11 :
                            {
                                strcpy(comBB_11, comBBin);
                                lat = atof(comBB_11);
                                printf("Latitude : %010.7f\n", lat);
                            }
                            BBcount++;
                            cout << "BBcount : " << BBcount << endl;
                        	break;

                            case 12 :
                            {
                                strcpy(comBB_12, comBBin);
                                lon = atof(comBB_12);
                                printf("Longitude : %010.7f\n", lon);
                            }
                            BBcount++;
                            cout << "BBcount : " << BBcount << endl;
                        	break;
                                                            
                            case 13 :
                            {
                                strcpy(comBB_13, comBBin);
                                roll =  atof(comBB_13);
                                printf("Rolls : %09.6f\n", roll);
                            }
                            BBcount++;
                            cout << "BBcount : " << BBcount << endl;
                            break;
                                                                
                            default :
                            break;
                        }

                        cout << BBinbound << endl;
                        comBBin = strtok(NULL, " ");
                        global_depth = z_terukur;
                        global_pitch = teta_terukur;
                        global_heading = yaw_terukur;

                       if(global_depth <= -1*depth_threshold || global_depth == -1*depth_target) {
					            ascent_flag = 1;
				        }

				        else if(global_depth >= surface_depth) {
				            descent_flag = 1;
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
                            sleep(1);
						}
						// BBcount = 0;
                    }
                }
            }
        }
    }

    delete[] BBinbound;
    tcflush(fd2, TCIOFLUSH);
    tcdrain(fd2);
    cout << "Parsing BB Finished" << endl;
    
	sleep(2);   
    ofstream logparsingBB("state_function_thread.txt", ios::out | ios::app);
    if(logparsingBB.is_open()) {
        logparsingBB << buftimeBB << " | " << "Parsing Finished" << endl;
        logparsingBB.close();
    }
    sleep(1);
}

void tempValue() {
	temp_depth = z_terukur;
	temp_pitch = teta_terukur;
	temp_yaw = yaw_terukur;
}

void handlingPID() {
	time_t tm_control;
    struct tm * control_systime;
    time(&tm_control);
    control_systime = localtime(&tm_control);
    char control_tm_info[50];
    strftime(control_tm_info, 50, "%a %b %d %Y %H:%M:%S", control_systime);

	ofstream logstatecontrol("state_function_thread.txt", ios::out | ios::app);
    if(logstatecontrol.is_open()) {
        logstatecontrol << control_tm_info << " | " << "Control Running" << endl;
        logstatecontrol.close();
    }

	cout << "Control System Start" << endl;
	
	tempValue();
	PID_rd();
	PID_bld();
	PID_blst();

	ofstream controlvaluelog("value_control.txt", ios ::out | ios::app);
	if(controlvaluelog.is_open()) {
		controlvaluelog << control_tm_info << "<< Error rudder : " << heading_err << ", PID rudder : " << rd_PID_val << endl;
		controlvaluelog << control_tm_info << "|| Error bladder : " << pitch_err << ", PID bladder : " << bld_PID_val << endl;
		controlvaluelog << control_tm_info << ">> Error ballast : " << depth_err << ", PID ballast : " << blst_PID_val << endl;
		controlvaluelog.close();
	}
	usleep(500000);

	cout << "Control System Stop" << endl;
	ofstream logcontroloff("state_function_thread.txt", ios::out | ios::app);
    if(logcontroloff.is_open()) {
        logcontroloff << control_tm_info << " | " << "Control OFF" << endl;
        logcontroloff.close();
    }

	sleep(2);
}

void protocol_operation(int _fd) {
	stream_bld << fixed << setprecision(5) << bld_PID_val;
	stream_blst << fixed << setprecision(5) << blst_PID_val;
	stream_teta << fixed << setprecision(10) << teta_terukur;
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

	printf("bld_PID_val %2.2f, bld_val %c\n", bld_PID_val, bld_val);
	
    sleep(1);	    
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
    write(fd, "\r\n", 2);
    usleep(250000);

    ofstream protocoldata("protocol_data.txt", ios::out | ios::app);
    if(protocoldata.is_open()) {
        protocoldata << "$#TS " << bld_val << ' ' << blst_val << ' ' << char_teta << ' ' << rd_val << ' ' << char_pos_e << ' ' << char_pos_n << ' ' << char_strobe << endl;
        protocoldata.close();
    
    }
	cout << "Data sent" << endl;
	tcdrain(fd);
}

void startOperation() {
    time_t tm_sys_start;
    struct tm * start_systime;
    time(&tm_sys_start);
    start_systime = localtime(&tm_sys_start);
    char start_tm_info[50];
    strftime(start_tm_info, 50, "%a %b %d %Y %H:%M:%S", start_systime);

   	ofstream comprotocol("com_log.txt", ios::out | ios::app);
    if(comprotocol.is_open()) {
        write(fd2, "$#O1", 4);
        cout << '\n';
		comprotocol << start_tm_info << " " << "BB start | " << "$#O1" << endl;
        comprotocol.close();
    }

    ofstream start_fsm("log_state.txt", ios::out | ios::app);
    if(start_fsm.is_open()) {
        start_fsm << start_tm_info << " " << "State : Start Ops" << endl;
        start_fsm.close();
    }
    cout << "Operation Start" << endl;
	sleep(2);
}

void glideOperation() {
	if(ascent_flag == true) {
		printf("descent flag : %d, ascent flag : %d\n", descent_flag, ascent_flag);
		if(global_depth <= -1*depth_threshold || global_depth == -1*depth_target) {
			printf("Start Ascent\n");
            ascentValue();
			handlingPID();
			protocol_operation(fd);
			printf("depth now : %05.2f, depth target : %05.2f, depth threshold : %05.2f\n", global_depth, depth_target, depth_threshold);
			printf("pitch now : %05.2f, pitch_ref : %05.2f\n", teta_terukur, pitch_ref);
		}
		ascent_flag = 0;
	}

	else if(descent_flag == true) {
		printf("descent flag : %d, ascent flag : %d\n", descent_flag, ascent_flag);
        if(global_depth >= surface_depth) {
			printf("Start Descent\n");
            descentValue();
			handlingPID();
			protocol_operation(fd);
			printf("depth now : %05.2f, depth target : %05.2f, depth threshold : %05.2f\n", global_depth, depth_target, depth_threshold);
			printf("pitch now : %05.2f, pitch_ref : %05.2f\n", teta_terukur, pitch_ref);
		}
		// ascent_flag = 1;
		descent_flag = 0;
	}
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
    rd_PID_val = rd_surface;
    prop_val = prop_val_off;
    bow_val = bow_val_off;
    strobe_val = strobe_on;
    protocol_operation(fd);

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
    buf_str_fsm = 0; stream_sensor = 0;
    cout << "Operation Stop" << endl;

	sleep(2);
}

void descentValue() {
    time_t tm_sys_des;
    struct tm * des_systime;
    time(&tm_sys_des);
    des_systime = localtime(&tm_sys_des);
    char des_tm_info[50];
    strftime(des_tm_info, 50, "%a %b %d %Y %H:%M:%S", des_systime);

    ref_depth_d = depth_target;
    ref_pitch = ref_pitch_d;
    ref_depth = ref_depth_d;
    
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

void ascentValue() {
    time_t tm_sys_asc;
    struct tm * asc_systime;
    time(&tm_sys_asc);
    asc_systime = localtime(&tm_sys_asc);
    char asc_tm_info[50];
    strftime(asc_tm_info, 50, "%a %b %d %Y %H:%M:%S", asc_systime);

    ref_depth_a = surface_depth;
    ref_pitch = ref_pitch_a;
    ref_depth = ref_depth_a;
    
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
    protocol_operation(fd);

	ofstream surface_fsm("log_state.txt", ios::out | ios::app);
    if(surface_fsm.is_open()) {
        surface_fsm << surface_tm_info << " " << "State : Surfacing" << endl;
        surface_fsm.close();
    }
    cout << "Surface Operation Start" << endl;
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
void handlingFSM() {
	time_t tm_sys_fsm;
    struct tm * fsm_systime;
    time(&tm_sys_fsm);
    fsm_systime = localtime(&tm_sys_fsm);
    char fsm_tm_info[50];
    strftime(fsm_tm_info, 50, "%a %b %d %Y %H:%M:%S", fsm_systime);
    mode = buf_str_fsm;

    printf("mode : %d\n", mode);

    ngliding = 0;
	nglided = 50;				

	switch(mode) {
		case 1 : //operation
			{
				while(buf_str_fsm != 0) {
					ofstream opstate("log_state.txt", ios ::out | ios::app);
					if(opstate.is_open()) {
						opstate << fsm_tm_info << ' ' << "State : Operation" << endl;
						opstate.close();
					}
					startOperation();
					sleep(1);
					if(ngliding < nglided) {
						handlingBB(fd2);
						glideOperation();
						printf("ngliding : %d\n", ngliding);
						ngliding++;
					}
					else if(ngliding >= nglided) {
						surfaceValue();
						bowthrustersOperation();
						propellerOperation();
						stopOperation();
					}
					sleep(1);
				}
			}
			break;

		case 2 : //gliding
			{
				while(buf_str_fsm != 0) {
					ofstream glstate("log_state.txt", ios ::out | ios::app);
					if(glstate.is_open()) {
						glstate << fsm_tm_info << ' ' << "State : Gliding" << endl;
						glstate.close();
					}
					startOperation();
					sleep(1);
					if(ngliding < nglided) {
						handlingBB(fd2);
						glideOperation();
						printf("ngliding : %d\n", ngliding);
						ngliding++;
					}
					else if(ngliding >= nglided) {
						surfaceValue();
						stopOperation();
					}
				
					sleep(1);
				}
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
				bowthrusters_on();
				sleep(5);
				bowthrusters_off();
				stopOperation();
			}
			break;

		case 5 : //manual op
			{
				startOperation();
				sleep(5);
				stopOperation();
			}
			break;

		default :
			break;
	}
}

void commandLine() {
	time_t tm_sys_com;
    struct tm * com_systime;
    time(&tm_sys_com);
    com_systime = localtime(&tm_sys_com);
    char com_tm_info[50];
    strftime(com_tm_info, 50, "%a %b %d %Y %H:%M:%S", com_systime);

	cout << "Waiting new command" << endl;
	getline(cin, mystr);
	if(mystr == "#GB> atc operation") {
		printf("Operation Mode\n");
		stream_sensor = 1;
		buf_str_fsm = 1;
		handlingFSM();
	}

	else if(mystr == "#GB> atc glide") {
		printf("Gliding Only Mode\n");
		stream_sensor = 1;
		buf_str_fsm = 2;
		handlingFSM();
	}

	else if(mystr == "#GB> atc test prop") {
		printf("Propeller Test Mode\n");
		buf_str_fsm = 3;
		handlingFSM();
	}

	else if(mystr == "#GB> atc test bow") {
		printf("Bowthrusters Test Mode\n");
		buf_str_fsm = 4;
		handlingFSM();
	}

	else if(mystr == "#GB> atc stop") {
		printf("All Stop\n");
		buf_str_fsm = 0;
		handlingFSM();
	}

    else if(mystr == "#GB> atc senddata") {
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
    }
}
