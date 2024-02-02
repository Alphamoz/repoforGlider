//libraries
#include <stddef.h>                   
#include <iostream>
#include "stdio.h"
#include "termios.h"
#include "errno.h"
#include "fcntl.h"
#include "string.h"
#include "time.h"
#include <stdlib.h> 
#include "sys/select.h"
#include <cstring>
#include <sstream>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <iomanip> 
#include <sstream> 
#include <time.h>
#include "imu.h"
#include <stdio.h>                     // This ert_main.c example uses printf/fflush 
#include "navigasinonlinirham.h"                           // Model's header file
#include "rtwtypes.h"
#include <math.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace Eigen;
static navigasinonlinirhamModelClass rtObj;

/*
experimental logger
#include <cstdlib>
#include <string>
#include <ctime>
#include <string.h>
*/

/* Deklarasi Variabel dan Fungsi */
//main function variable
int program_running = 0; 
int program_running_firsttime_in_the_water=0; //for navigational system initial position 
int ReadDVL(int _fd);
void ReadIMU(void);
void ReadAlti(int _fd);
void ReadMiniCT(int _fd);
void TriggerAlti(int _fd);
void TriggerDVL(void);
void Navigasi(void);
void GuidanceLOS(void);
void LogData(void);
void SendTS(void);
void Scenario_1(void);
double distance_two_coordinates(double lat1,double lon1,double lat2,double lon2);
void RestartRelay(void);

/* deklarasi waktu
struct timeval start, end;
long mtime, seconds, useconds;
*/

// Variabel Komunikasi Serial
int fd;
int fd2;
int fd1;
int fd5;
int datacount=1;
int valid = 0;
char* chr;
char* tmp;
char digits[10];
double testfloat = 989.898;
double floattemp;
int datafull = 0;
string to_string(double x);
int enabletestoutput=1;
int closed_once = 0;
int intA;
char *portTS = "/dev/ttyO2"; //OK
char *portAlti = "/dev/ttyO5";
char *portDVL = "/dev/ttyO1";
char *portCT = "/dev/ttyO4"; //OK


//Variabel DVL
//Variabel pembacaan
bool DVLValid = 0;
//Variabel string DVL
char header[10];
char error_code[8];
char beamresult1[2],beamresult2[2],beamresult3[2],beamresult4[2];
char altitude1[8],altitude2[8],altitude3[8],altitude4[8];
char velo_rad1[10],velo_rad2[10],velo_rad3[10],velo_rad4[10];
char wvelo_rad1[10],wvelo_rad2[10],wvelo_rad3[10],wvelo_rad4[10];
char cwvelo_rad1[5],cwvelo_rad2[5],cwvelo_rad3[5],cwvelo_rad4[5];
char veloX[10],veloY[10],veloZ[10],veloFLAG[3];
char wveloX[10],wveloY[10],wveloZ[10],wveloFLAG[3];
char altitudeMin[8],Temperature[6];
char salinity[6],soundspeed[8];
char ceksum[4];


//Variabel float DVL
float altitude1_F=0.0,altitude2_F=0.0,altitude3_F=0.0,altitude4_F=0.0;
float velo_rad1_F=0.0,velo_rad2_F=0.0,velo_rad3_F=0.0,velo_rad4_F=0.0;
float wvelo_rad1_F=0.0,wvelo_rad2_F=0.0,wvelo_rad3_F=0.0,wvelo_rad4_F=0.0;
float cwvelo_rad1_F=0.0,cwvelo_rad2_F=0.0,cwvelo_rad3_F=0.0,cwvelo_rad4_F=0.0;
float veloX_F=0.0,veloY_F=0.0,veloZ_F=0.0,veloFLAG_F=0.0;
float wveloX_F=0.0,wveloY_F=0.0,wveloZ_F=0.0,wveloFLAG_F=0.0;
float altitudeMin_F=0.0,Temperature_F=0.0;
float salinity_F=0.0,soundspeed_F=0.0;
float ceksum_F=0.0;
float Speed=0.0;
/* Variabel yang di Log
float altitude1_F,altitude2_F,altitude3_F,altitude4_F;
float veloX_F,veloY_F,veloZ_F,veloFLAG_F;
float wveloX_F,wveloY_F,wveloZ_F,wveloFLAG_F;
float altitudeMin_F,
*/

//Variabel IMU
double Latitude=0.0;
double Longitude=0.0;
double Height=0.0;
double Roll=0.0;
double Pitch=0.0;
double Heading=0.0;
double accel_X=0.0;
double accel_Y=0.0;
double accel_Z=0.0;
double Omega_Roll=1.0;		
double Omega_Pitch=0.0;		
double Omega_Heading=0.0;
double Pitch_offset=0.0;

//Variabel MiniCT
//T=<tab>nn.nnn<tab>C=<tab>nn.nnn<cr><lf>
char c_CT_conductivity[8], c_CT_temperature[8];
bool MiniCTValid = 0;
double CT_conductivity=0.0;
double CT_temperature=0.0;

//Variabel Altisounder
char c_Sounder_altitude[8], c_Sounder_altitude_unit[3],c_Sounder_pressure[10],c_Sounder_pressure_unit[5];
bool AltiValid =0;
float Sounder_depth=0.0;
float Sounder_pressure=0.0;
float Sounder_altitude=0.0;
double earth_gravity = 9.81;
double water_density=1000;
static int Sounder_period=0;

//Fungsi dan Variabel C Code
void rt_OneStep(void);
void waitFor (unsigned int secs);

//Variabel Send to TS
//Dummy Tester
// $#NG teta_terukur z_terukur yawref yaw_terukur posx posy altitude depth  salinity temperature speed latitude longitude
//char header[4]  = {'$','#','N','G'}; 
double dum_teta	=	1;
double dum_z	=	2;
double dum_yawr	=	3;
double dum_yawt	=	-4;
//double dum_posx	=	5;
//double dum_posy	=	-6;
double dum_alti =	5;
//double dum_dept	=	6;
double dum_sali	=	6;
double dum_temp	=	7;
double dum_sped	=	8;
double dum_lati	=	9;
double dum_long	=	10;
//strings
string string_teta;
string string_z;
string string_yawr;
string string_yawt;
string string_posx;
string string_posy;
string string_alti;
string string_dept;
string string_sali;
string string_temp;
string string_sped;
string string_lati;
string string_long;
string string_roll;
//stream
stringstream stream_teta;
stringstream stream_z;
stringstream stream_yawr;
stringstream stream_yawt;
stringstream stream_posx;
stringstream stream_posy;
stringstream stream_alti;
stringstream stream_dept;
stringstream stream_sali;
stringstream stream_temp;
stringstream stream_sped;
stringstream stream_lati;
stringstream stream_long;
stringstream stream_roll;
//output chars
char char_teta [10];
char char_z    [10];
char char_yawr [10];
char char_yawt [10];
char char_posx [10];
char char_posy [10];
char char_alti [10];
char char_dept [10];
char char_sali [10];
char char_temp [10];
char char_sped [10];
char char_lati [13];
char char_long [13];
char char_roll [10];

//Variabel Data Log
static int datalog_entrynumber=0;
int datalog_entrylimit=-1;
//Dummy tester
float log_a = 10;
float log_b = 20;
float log_c = 30;
float log_d = 40;
float log_e = 50;
int newlogfile = 1;

//Deklarasi GPIO
//DVL Trigger GPIO Configuration
int GPIOPin=61, /* P8 26 */ times=10;
FILE *myOutputHandle = NULL;
char setValue[4], GPIOString[4], GPIOValue[64], GPIODirection[64];

//Relay Setting GPIO Configuration
int GPIOPin_1=46, /* P8 16 */ times_1=10;
FILE *myOutputHandle_1 = NULL;
char setValue_1[4], GPIOString_1[4], GPIOValue_1[64], GPIODirection_1[64];

//Variabel Guidance
//High Level Variable
double home_lat=0.0,home_long=0.0; //long bujur, lat lintang
double target_lat=0.0, target_long=0.0;
double wpX[10]={0,0,0,0,0,0,0,0,0,0},wpY[10]={0,0,0,0,0,0,0,0,0,0};
int waypoint_step=0,waypoint_number=0; //step between home and target, total number of wp including home and target
int guidance_wp_target=0; //target waypoint
int guidance_finish=0;
double psiref=0.0;

//High Level internal setting variable
float distance_step=1.0;//in km dividing between waypoint
double acceptance_radius=0.00009; //10 meter roughly

//Internal static guidance function
double distancefromtarget=100.0;

//Guidance communication protocol variable
int guidance_setting=0; // 0 idle, 1 set home, 2 set target, 3 return home
char home_lat_S[15],home_long_S[15];
char target_lat_S[15],target_long_S[15];
double PI = 3.1415926536;
double temp_wp=0;//temp file for swapping
int gcount=1;
int g_iteration=1; //iteration purpose
double distance_home_target=0; 

//Variabel Navigasi
int ReadNav(int _fd);
void SendNav(void);
int kalman_reset = 0;
VectorXd meas(3);
VectorXd xhat = VectorXd(5);
MatrixXd P = MatrixXd(5, 5);
VectorXd Resid(3);
VectorXd yhatOut(3);
VectorXd xhatOut(5);
MatrixXd A(5, 5);
MatrixXd Atrans(5, 5);
MatrixXd C(3, 5);
MatrixXd Q = MatrixXd::Zero(5, 5);
MatrixXd matrixR(3, 3);
MatrixXd K(3, 3);
MatrixXd Ctrans(5, 3);
MatrixXd PCtrans(5, 3);
MatrixXd CPCtransR(3, 3);
MatrixXd identitysizeK = MatrixXd::Identity(5, 5);
//Initial position
double lat_gps=0,lng_gps=0,depth_gps=0;
//Output
double Nav_zKF=0;
double Nav_lat=0,Nav_long=0,Nav_z=0;
double Nav_kecN=0,Nav_kecE=0,Nav_kecD=0;
//Reading
int NavValid=0;
//Static Var
double posisilat_aug = 0;
double posisilng_aug = 0;
double posisidepth_aug = 0;
double Nav_N=0, Nav_E=0;

static double vN_IMU = 0, vE_IMU = 0, vD_IMU = 0;
static double Z_KF=0,Z_KF_m=0;
static double Z_Complement=0;

//Variabel Skenario
int skenario=0;
static int State_Scenario_1=0;
int V_propeller=0;
double yawref_Scenario1;

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
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


void Com_mainboard(int _fd)
{
	fd_set readfs;
	timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 20000;
	char * buffer = new char[60];
	FD_ZERO(&readfs);
	FD_SET(_fd, &readfs);
	select(_fd+1, &readfs, NULL, NULL, &tv /* no timeout */);
	
	if (FD_ISSET(_fd, &readfs))
	{
		printf("Receiving from TS\n");
		//gettimeofday(&start, NULL);
		usleep(10000);
		int r = read(_fd, buffer, 60);
		if(r == -1)
		{
			cout << strerror(errno) << endl;
		}
		string my_string(buffer,buffer+r);
		printf("%s \n",buffer);
		if (my_string[0] == '$')
		{
			
			if (my_string[1] == '#')
			{
				if (my_string[2] == 'O')
				{
					if (my_string[3] == '1')
					{

						valid = 1;
						program_running = 1;
						program_running_firsttime_in_the_water=1;
						//write(_fd,buffer,r);
						//cout << "header keterima cuy" << endl;
						printf("Data received - Activation\n");
						write(_fd,"Beagle Activated/r/n",18);
						tcdrain(_fd);
					}
					else if(my_string[3] == '0')
					{
						//closed_once = 1;
						valid = 1;
						program_running = 0;
						//write(_fd,buffer,r);
						//cout << "header keterima cuy" << endl;
						printf("Data received - DeActivation\n");
						write(_fd,"Beagle Deactivated/r/n",20);
						tcdrain(_fd);
					}
					else if(my_string[3] == '9')
					{
						closed_once = 1;
						valid = 1;
						program_running = 0;
						//write(_fd,buffer,r);
						//cout << "header keterima cuy" << endl;
						printf("Data received\n");
						write(_fd,"Beagle Deactivated/r/n",20);
						tcdrain(_fd);
						close(fd);
						close(fd2);
					}
				}
				
				//Guidance Command
				if (my_string[2] == 'G')
				{
					if (my_string[3] == '1')
					{
						guidance_setting = 1;
						printf("Receiving Home Setting\n");
						write(_fd,"$#G1 OK/r/n",9);
						tcdrain(_fd);
					}
					else if(my_string[3] == '2')
					{
						guidance_setting = 2;
						printf("Receiving Target Setting\n");
						write(_fd,"$#G2 OK/r/n",9);
						tcdrain(_fd);
					}
					else if(my_string[3] == '3')
					{
						guidance_setting = 3;
						printf("Receiving Return Setting\n");
						write(_fd,"$#G3 OK/r/n",9);
						tcdrain(_fd);
					}
				}
				
				
				if (my_string[2] == 'S')
				{
					if (my_string[3] == '1')
					{
						skenario = 1;
						State_Scenario_1=1;
						printf("Receiving Scenario Setting\n");
						write(_fd,"S1 OK/r/n",7);
						tcdrain(_fd);
					}
					else if(my_string[3] == '0')
					{
						skenario = 0;
						State_Scenario_1=0;
						printf("Receiving Scenario Setting\n");
						write(_fd,"S0 OK/r/n",7);
						tcdrain(_fd);
					}
				}
				
				if (my_string[2] == 'R')
				{
					if (my_string[3] == '3')
					{
						write(_fd,"Restarting\r\n",12);
						tcdrain(_fd);
						RestartRelay();
						write(_fd,"The relay was restarted\r\n",25);
						tcdrain(_fd);
						
					}
					
					if (my_string[3] == '0')
					{
						if ((myOutputHandle_1 = fopen(GPIOValue_1, "rb+")) == NULL){
							printf("GPIO: Unable to open value handle\n");
							//return 1;
						}
						strcpy(setValue_1, "0"); // Set value low
						fwrite(&setValue_1, sizeof(char), 1, myOutputHandle_1);
						fclose(myOutputHandle_1);
						printf("Relay OFF\n");
						write(_fd,"Relay OFF\r\n",11);
					}	
					
					
					if (my_string[3] == '1')
					{
						if ((myOutputHandle_1 = fopen(GPIOValue_1, "rb+")) == NULL){
							printf("GPIO: Unable to open value handle\n");
							//return 1;
						}
						strcpy(setValue_1, "1"); // Set value high
						fwrite(&setValue_1, sizeof(char), 1, myOutputHandle_1);
						fclose(myOutputHandle_1);
						printf("Relay ON\n");
						write(_fd,"Relay ON\r\n",10);
						
					}
				}
				
				
				
			}
		}
		
		
		//usleep(10000);
		datacount = 1;
		
		//Guidance Setting
		if(guidance_setting ==1)
		{
			
			char * pchG;
		    pchG = strtok (buffer," ");
			while (pchG != NULL)
		    {
				switch ( gcount ) 
				{

				case 1 : 
					strcpy(header, pchG); /* BANG!!! */
					strncpy(header, pchG, sizeof header - 1); /* OK ... but `dst` needs to be NUL terminated */
					//printf ("Header %s\n\r",pch);
					//write(_fd,header,sizeof(header));
					//write(_fd,"\r\n",sizeof(pch));
					gcount++;
					break;
				
				case 2 : 
					//printf ("aImuX %s\n",pch);
					strcpy(home_lat_S, pchG); /* BANG!!! */
					strncpy(home_lat_S, pchG, sizeof home_lat_S - 1); /* OK ... but `dst` needs to be NUL terminated */
					home_lat = atof (home_lat_S);
					printf("Home: %f\t",home_lat);
					gcount++;
					break;
					
				case 3 : 
					//printf ("aImuX %s\n",pch);
					strcpy(home_long_S, pchG); /* BANG!!! */
					strncpy(home_long_S, pchG, sizeof home_long_S - 1); /* OK ... but `dst` needs to be NUL terminated */
					home_long = atof (home_long_S);
					printf("%f\n",home_long);
					gcount++;
					
					wpX[0]=home_long;
					wpY[0]=home_lat;
					wpX[2]=home_long;
					wpY[2]=home_lat;
					break;
				}
				pchG = strtok (NULL, " ");		
			}
			gcount = 1;
			printf("Receiving Home Setting DONE\n");
			for(g_iteration=0;g_iteration<=9;g_iteration++){
					printf("%d \t %f \t %f\n",g_iteration,wpX[g_iteration],wpY[g_iteration]); 
			}
		}
		else if(guidance_setting==2)
		{
			char * pchG;
		    pchG = strtok (buffer," ");
			while (pchG != NULL)
		    {
				switch ( gcount ) 
				{

				case 1 : 
					strcpy(header, pchG); /* BANG!!! */
					strncpy(header, pchG, sizeof header - 1); /* OK ... but `dst` needs to be NUL terminated */
					//printf ("Header %s\n\r",pch);
					//write(_fd,header,sizeof(header));
					//write(_fd,"\r\n",sizeof(pch));
					gcount++;
					break;
				
				case 2 : 
					//printf ("aImuX %s\n",pch);
					strcpy(target_lat_S, pchG); /* BANG!!! */
					strncpy(target_lat_S, pchG, sizeof target_lat_S - 1); /* OK ... but `dst` needs to be NUL terminated */
					target_lat = atof (target_lat_S);
					printf("Target: %f\t",target_lat);
					gcount++;
					break;
					
				case 3 : 
					//printf ("aImuX %s\n",pch);
					strcpy(target_long_S, pchG); /* BANG!!! */
					strncpy(target_long_S, pchG, sizeof target_long_S - 1); /* OK ... but `dst` needs to be NUL terminated */
					target_long = atof (target_long_S);
					printf("%f\n",target_long);
					gcount++;
					distance_home_target = distance_two_coordinates(home_lat,home_long,target_lat,target_long);
					printf("Distance_home_target = %f\n",distance_home_target);
					wpX[1]=target_long;
					wpY[1]=target_lat;
					//Print All waypoint
					for(g_iteration=0;g_iteration<=9;g_iteration++){
					printf("%d \t %f \t %f\n",g_iteration,wpX[g_iteration],wpY[g_iteration]); 
					}
					guidance_wp_target=1;
					waypoint_number=2;
					break;
				}
				pchG = strtok (NULL, " ");		
			}
			gcount = 1;
			printf("Receiving Target Setting DONE\n");
		}
		else if(guidance_setting==3)
		{
			//brute force dulu :)
			guidance_wp_target=1;
			if(waypoint_number==2){
				wpX[1]=home_long;
				wpY[1]=home_lat;
				wpX[2]=home_long;
				wpY[2]=home_lat;
			}
			printf("Receiving Return Setting DONE\n");
			for(g_iteration=0;g_iteration<=9;g_iteration++){
					printf("%d \t %f \t %f\n",g_iteration,wpX[g_iteration],wpY[g_iteration]); 
			}
		}
		guidance_setting=0;
	}
	else
	{
		//cout << "data not available" << endl;
		 // Clear IO buffer
		 //tcflush(_fd, TCIOFLUSH);
		if (program_running == 0)
		{
			write(_fd,"Beagle ON\r\n",11);
			// printf("Hey TS\n");
			tcdrain(_fd);
			//sleep(1);
		}	
	}
	datafull = 0;
	
}


int main(int argc, const char *argv[])
{
	// Unused arguments
	(void)(argc);
	(void)(argv);

	
	rtObj.initialize();
	sleep(2);
	//open port komunikasi
	fd = open(portTS, O_RDWR | O_NOCTTY | O_SYNC);
	usleep(100000);
	fd1 = open(portAlti, O_RDWR | O_NOCTTY | O_SYNC);
	usleep(100000);
	fd2 = open(portDVL, O_RDWR | O_NOCTTY | O_SYNC);
	usleep(100000);
	fd5 = open(portCT, O_RDWR | O_NOCTTY | O_SYNC);
	usleep(100000);
	/*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B9600);
	set_interface_attribs(fd1, B115200);
	set_interface_attribs(fd2, B115200);
	set_interface_attribs(fd5, B115200);
    //set_mincount(fd, 0);                /* set to pure timed read */
	
	/* Init IMU *
	init_imu();
	sleep(1);
	read_imu(Cport, &Latitude, &Longitude, &Height, &Roll, &Pitch, &Heading, &accel_X, &accel_Y, &accel_Z, &Omega_Roll, &Omega_Pitch, &Omega_Heading);
	/*End Init IMU*/
	
	//File Log Name
	
	
	printf("\nStarting GPIO output program\n");
	//GPIO Configuration for DVL
	sprintf(GPIOString, "%d", GPIOPin);
    sprintf(GPIOValue, "/sys/class/gpio/gpio%d/value", GPIOPin);
    sprintf(GPIODirection, "/sys/class/gpio/gpio%d/direction", GPIOPin);
    // Export the pin
    if ((myOutputHandle = fopen("/sys/class/gpio/export", "ab")) == NULL){
        printf("Unable to export GPIO pin\n");
        return 1;
    }
    strcpy(setValue, GPIOString);
    fwrite(&setValue, sizeof(char), 2, myOutputHandle);
    fclose(myOutputHandle);
    // Set direction of the pin to an output
    if ((myOutputHandle = fopen(GPIODirection, "rb+")) == NULL){
        printf("Unable to open direction handle\n");
        return 1;
    }
    strcpy(setValue,"out");
    fwrite(&setValue, sizeof(char), 3, myOutputHandle);
    fclose(myOutputHandle);
	
	//GPIO Configuration for Relay
	sprintf(GPIOString_1, "%d", GPIOPin_1);
    sprintf(GPIOValue_1, "/sys/class/gpio/gpio%d/value", GPIOPin_1);
    sprintf(GPIODirection_1, "/sys/class/gpio/gpio%d/direction", GPIOPin_1);
    // Export the pin
    if ((myOutputHandle_1 = fopen("/sys/class/gpio/export", "ab")) == NULL){
        printf("Unable to export GPIO pin\n");
        return 1;
    }
    strcpy(setValue_1, GPIOString_1);
    fwrite(&setValue_1, sizeof(char), 2, myOutputHandle_1);
    fclose(myOutputHandle_1);
    // Set direction of the pin to an output
    if ((myOutputHandle_1 = fopen(GPIODirection_1, "rb+")) == NULL){
        printf("Unable to open direction handle\n");
        return 1;
    }
    strcpy(setValue_1,"out");
    fwrite(&setValue_1, sizeof(char), 3, myOutputHandle_1);
    fclose(myOutputHandle_1);
	//END GPIO Conf
	
	// Set Relay output to high
        if ((myOutputHandle_1 = fopen(GPIOValue_1, "rb+")) == NULL){
            printf("GPIO: Unable to open value handle\n");
            //return 1;
        }
        strcpy(setValue_1, "1"); // Set value high
        fwrite(&setValue_1, sizeof(char), 1, myOutputHandle_1);
        fclose(myOutputHandle_1);
		printf("Relay ON\n");
	
	//loop
	while (1) {
	//gettimeofday(&start, NULL);
		
		Com_mainboard(fd);
		
		if (program_running == 0)
		{
			printf("Hei\n");
			//Trigger DVL
			TriggerDVL();
			
			/* Trigger Alti */
			if(Sounder_period==1){
			TriggerAlti(fd1);
			Sounder_period=0;
			}
			else
			{
			Sounder_period=1;
			}
			/* End Trigger Alti */
			
			//Read IMU, MiniCT
			ReadMiniCT(fd5);
			//read_imu(Cport, &Latitude, &Longitude, &Height, &Roll, &Pitch, &Heading, &accel_X, &accel_Y, &accel_Z, &Omega_Roll, &Omega_Pitch, &Omega_Heading);
			Pitch=Pitch-Pitch_offset;
			printf("Reading IMU\n");
			/*End Sensor Reading*/
			
			//printf("\tLatitude = %f, Longitude = %f, Height = %f\n", Latitude, Longitude, Height);
			
			//printf("\tAccelerometers X: %f Y: %f Z: %f\n", accel_X, accel_Y, accel_Z);
			//printf("\tGyroscopes X: %f Y: %f Z: %f\n", Omega_Roll, Omega_Pitch, Omega_Heading);
			/*end read IMU*/
			
			//Tuning Jeda untuk DVL dan Alti
			usleep(700000);
			
			
			//Baca DVL dan Alti
			ReadAlti(fd1);
			intA = ReadDVL(fd2);
			
			//Navigasi Guidance
			Navigasi();
			GuidanceLOS();
			if (skenario == 1) Scenario_1();
			//DataLog
			LogData();
			//SendTS
			SendTS();
			if(program_running_firsttime_in_the_water==1)
			{
			rtObj.rtU.Inpawalx = Longitude;
			rtObj.rtU.Inpawaly = Latitude;
			rtObj.rtU.Inpawalz = Height;
			Pitch_offset = Pitch;
			wpX[0]=107.611221;
			wpY[0]=-6.890702;
			//45 derajat -6.884356, 107.618340
			wpX[1]=107.618340;
			wpY[1]=-6.884356;
			//-45 -6.898282, 107.617977
			/*End */
			program_running_firsttime_in_the_water=0;
			printf("The navigation system is initialized\n");
			}
			
		}
		else
		{
			sleep(1);
			
		}
		 /*  
	gettimeofday(&end, NULL);		
	seconds  = end.tv_sec  - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;
	mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
	printf("Elapsed time: %ld milliseconds\n", mtime);
	*/
		
		
	}

	// Disable rt_OneStep() here
	return 0;
}

string to_string(double x)
{
  std::ostringstream ss;
  ss << x;
  return ss.str();
}

void ReadAlti(int _fd)
{
	printf("Reading Alti\r\n");
	fd_set readfs;
	timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 10000;
	char * buffer = new char[200];
	FD_ZERO(&readfs);
	FD_SET(_fd, &readfs);
	select(_fd+1, &readfs, NULL, NULL, &tv /* no timeout */);
	
	if (FD_ISSET(_fd, &readfs))
	{
		printf("Hei This Alti\r\n");
		//gettimeofday(&start, NULL);
		usleep(50000);
		int r = read(_fd, buffer, 3);
		r = read(_fd, buffer, 200);
		printf("%s\n",buffer);
		if(r == -1)
		{
			cout << strerror(errno) << endl;
		}
		string my_string(buffer,buffer+r);
		if (my_string[0] == '$')
		{
			AltiValid = 1;
			printf("Alti Data received\n");
			//write(_fd,"Alti Received/r/n",15);
			//tcdrain(_fd);
		}
		if (AltiValid == 1)
		{
			char * pch;
		    pch = strtok (buffer,",");
			printf("pch\n%s\n",pch);
		    
			while (pch != NULL)
		    {
				switch ( datacount ) 
				{
				case 1 : 
					//strcpy(header, pch); 
					strncpy(header, pch, sizeof header - 1); 
					//printf("header %s\t",header);
					datacount++;
					break;
				case 2 : 
					//strcpy(error_code, pch); 
					strncpy(c_Sounder_altitude, pch, sizeof c_Sounder_altitude - 1); 
					datacount++;
					Sounder_altitude = atof (c_Sounder_altitude);
					printf ("AltiSounder altitude %f\n",Sounder_altitude);
					break;
				case 3 :  
					datacount++;
					//printf ("float Dm1 %f\n",Dm1);
					break;
				case 4 : 
					//strcpy(error_code, pch); 
					strncpy(c_Sounder_pressure, pch, sizeof c_Sounder_pressure - 1); 
					datacount++;
					Sounder_depth = atof (c_Sounder_pressure);
					//Sounder_depth=Sounder_pressure*10000/(water_density*earth_gravity);
					//printf ("AltiSounder pressure %f\n",Sounder_pressure);
					printf ("AltiSounder depth %f\n",Sounder_depth);
					break;
				case 5 : 
					//strcpy(error_code, pch); 
					datacount++;
					
					//printf ("float Dm1 %f\n",Dm1);
					break;
				 default : 
					// Process for all other cases.
					printf ("Default/n");
					break;
				}
			  pch = strtok (NULL, ",");
			  
		    }
			datacount = 1;
			AltiValid = 0;
		}
		usleep(100000);
		
	}
	else
	{
		//cout << "data not available" << endl;
		 // Clear IO buffer
		 //tcflush(_fd, TCIOFLUSH);
		//write(_fd,"no alti  !\r\n",12);
		//tcdrain(_fd);
	}
	datafull = 0;
}

int ReadDVL(int _fd)
{
	printf("Reading DVL\r\n");
	fd_set readfs;
	timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 100000;
	char * buffer = new char[900];
	FD_ZERO(&readfs);
	FD_SET(_fd, &readfs);
	select(_fd+1, &readfs, NULL, NULL, &tv /* no timeout */);
	
	if (FD_ISSET(_fd, &readfs))
	{
		//gettimeofday(&start, NULL);
		usleep(50000);
		int r = read(_fd, buffer, 900);
		if(r == -1)
		{
			cout << strerror(errno) << endl;
		}
		string my_string(buffer,buffer+r);
		if (my_string[0] == '$')
		{
			DVLValid = 1;
			printf("DVL Data received\n");
			//write(_fd,"DVL Received/r/n",14);
		}
		if (DVLValid == 1)
		{
		    char * pch;
		    pch = strtok (buffer," ");
			printf("%s\n",pch);
		    
			while (pch != NULL)
		    {
				switch ( datacount ) 
				{
				case 1 : 
					//strcpy(header, pch); 
					strncpy(header, pch, sizeof header - 1); 
					//printf("%s\t",header);
					datacount++;
					break;
				case 2 : 
					//strcpy(error_code, pch); 
					strncpy(error_code, pch, sizeof error_code - 1); 
					datacount++;
					//Vbd = atof (vbd);
					//printf("%s\n",error_code);
					//printf ("float Dm1 %f\n",Dm1);
					break;
				case 3 : 
					//strcpy(error_code, pch); 
					strncpy(beamresult1, pch, sizeof beamresult1 - 1); 
					datacount++;
					//Vbd = atof (vbd);
					printf("%s\t",beamresult1);
					//printf ("float Dm1 %f\n",Dm1);
					break;
				case 4 : 
					//strcpy(error_code, pch); 
					strncpy(beamresult2, pch, sizeof beamresult2 - 1); 
					datacount++;
					//Vbd = atof (vbd);
					printf("%s\t",beamresult2);
					//printf ("float Dm1 %f\n",Dm1);
					break;
				case 5 : 
					//strcpy(error_code, pch); 
					strncpy(beamresult3, pch, sizeof beamresult3 - 1); 
					datacount++;
					//Vbd = atof (vbd);
					printf("%s\t",beamresult3);
					//printf ("float Dm1 %f\n",Dm1);
					break;
				case 6 : 
					//strcpy(error_code, pch); 
					strncpy(beamresult4, pch, sizeof beamresult4 - 1); 
					datacount++;
					//Vbd = atof (vbd);
					printf("%s\n",beamresult4);
					//printf ("float Dm1 %f\n",Dm1);
					break;
				case 7 : 
					//strcpy(error_code, pch); 
					strncpy(altitude1, pch, sizeof altitude1 - 1); 
					datacount++;
					altitude1_F = atof (altitude1);
					printf ("float altitude 1 %f\t",altitude1_F);
					break;
				case 8 : 
					//strcpy(error_code, pch); 
					strncpy(altitude2, pch, sizeof altitude2 - 1); 
					datacount++;
					altitude2_F = atof (altitude2);
					printf ("altitude 2 %f\t",altitude2_F);
					break;
				case 9 : 
					//strcpy(error_code, pch); 
					strncpy(altitude3, pch, sizeof altitude3 - 1); 
					datacount++;
					altitude3_F = atof (altitude3);
					printf ("faltitude 3 %f\t",altitude3_F);
					break;
				case 10 : 
					//strcpy(error_code, pch); 
					strncpy(altitude4, pch, sizeof altitude4 - 1); 
					datacount++;
					altitude4_F = atof (altitude4);
					printf ("altitude 4 %f\n",altitude4_F);
					break;
				case 11 : 
					//strcpy(error_code, pch); 
					strncpy(velo_rad1, pch, sizeof velo_rad1 - 1); 
					datacount++;
					velo_rad1_F = atof (velo_rad1);
					//printf ("float velo_rad1 %f\t",velo_rad1_F);
					break;
				case 12 : 
					//strcpy(error_code, pch); 
					strncpy(velo_rad2, pch, sizeof velo_rad2 - 1); 
					datacount++;
					velo_rad2_F = atof (velo_rad2);
					//printf ("velo_rad2 %f\t",velo_rad2_F);
					break;
				case 13 : 
					//strcpy(error_code, pch); 
					strncpy(velo_rad3, pch, sizeof velo_rad3 - 1); 
					datacount++;
					velo_rad3_F = atof (velo_rad3);
					//printf ("velo_rad3 %f\t",velo_rad3_F);
					break;
				case 14 : 
					//strcpy(error_code, pch); 
					strncpy(velo_rad4, pch, sizeof velo_rad4 - 1); 
					datacount++;
					velo_rad4_F = atof (velo_rad4);
					//printf ("velo_rad4 %f\n",velo_rad4_F);
					break;
				case 15 : 
					//strcpy(error_code, pch); 
					strncpy(wvelo_rad1, pch, sizeof wvelo_rad1 - 1); 
					datacount++;
					wvelo_rad1_F = atof (wvelo_rad1);
					//printf ("float wvelo_rad1 %f\t",wvelo_rad1_F);
					break;
				case 16 : 
					//strcpy(error_code, pch); 
					strncpy(wvelo_rad2, pch, sizeof wvelo_rad2 - 1); 
					datacount++;
					wvelo_rad2_F = atof (wvelo_rad2);
					//printf ("wvelo_rad2 %f\t",wvelo_rad2_F);
					break;
				case 17 : 
					//strcpy(error_code, pch); 
					strncpy(wvelo_rad3, pch, sizeof wvelo_rad3 - 1); 
					datacount++;
					wvelo_rad3_F = atof (wvelo_rad3);
					//printf ("wvelo_rad3 %f\t",wvelo_rad3_F);
					break;
				case 18 : 
					//strcpy(error_code, pch); 
					strncpy(wvelo_rad4, pch, sizeof wvelo_rad4 - 1); 
					datacount++;
					wvelo_rad4_F = atof (wvelo_rad4);
					//printf ("wvelo_rad4 %f\n",wvelo_rad4_F);
					break;
				case 19 : 
					//strcpy(error_code, pch); 
					strncpy(cwvelo_rad1, pch, sizeof cwvelo_rad1 - 1); 
					datacount++;
					cwvelo_rad1_F = atof (cwvelo_rad1);
					//printf ("float cwvelo_rad1 %f\t",cwvelo_rad1_F);
					break;
				case 20 : 
					//strcpy(error_code, pch); 
					strncpy(cwvelo_rad2, pch, sizeof cwvelo_rad2 - 1); 
					datacount++;
					cwvelo_rad2_F = atof (cwvelo_rad2);
					//printf ("cwvelo_rad2 %f\t",cwvelo_rad2_F);
					break;
				case 21 : 
					//strcpy(error_code, pch); 
					strncpy(cwvelo_rad3, pch, sizeof cwvelo_rad3 - 1); 
					datacount++;
					cwvelo_rad3_F = atof (cwvelo_rad3);
					//printf ("cwvelo_rad3 %f\t",cwvelo_rad3_F);
					break;
				case 22 : 
					//strcpy(error_code, pch); 
					strncpy(cwvelo_rad4, pch, sizeof cwvelo_rad4 - 1); 
					datacount++;
					cwvelo_rad4_F = atof (cwvelo_rad4);
					//printf ("cwvelo_rad4 %f\n",cwvelo_rad4_F);
					break;
				case 23 : 
					//strcpy(error_code, pch); 
					strncpy(veloX, pch, sizeof veloX - 1); 
					datacount++;
					veloX_F = atof (veloX);
					//printf ("veloX %f\t",veloX_F);
					break;
				case 24 : 
					//strcpy(error_code, pch); 
					strncpy(veloY, pch, sizeof veloY - 1); 
					datacount++;
					veloY_F = atof (veloY);
					//printf ("veloY %f\t",veloY_F);
					break;
				case 25 : 
					//strcpy(error_code, pch); 
					strncpy(veloZ, pch, sizeof veloZ - 1); 
					datacount++;
					veloZ_F = atof (veloZ);
					//printf ("veloZ %f\t",veloZ_F);
					break;
				case 26 : 
					//strcpy(error_code, pch); 
					strncpy(veloFLAG, pch, sizeof veloFLAG - 1); 
					datacount++;
					veloFLAG_F = atof (veloFLAG);
					//printf ("veloFLAG %f\n",veloFLAG_F);
					break;
				case 27 :
					datacount++;
					break;
				case 28 :
					datacount++;
					break;
				case 29 :
					datacount++;
					break;
				case 30 :
					datacount++;
					break;
				case 31 : 
					//strcpy(error_code, pch); 
					strncpy(wveloX, pch, sizeof wveloX - 1); 
					datacount++;
					wveloX_F = atof (wveloX);
					//printf ("wveloX %f\t",wveloX_F);
					break;
				case 32 : 
					//strcpy(error_code, pch); 
					strncpy(wveloY, pch, sizeof wveloY - 1); 
					datacount++;
					wveloY_F = atof (wveloY);
					//printf ("wveloY %f\t",wveloY_F);
					break;
				case 33 : 
					//strcpy(error_code, pch); 
					strncpy(wveloZ, pch, sizeof wveloZ - 1); 
					datacount++;
					wveloZ_F = atof (wveloZ);
					//printf ("wveloZ %f\t",wveloZ_F);
					break;
				case 34 : 
					//strcpy(error_code, pch); 
					strncpy(wveloFLAG, pch, sizeof wveloFLAG - 1); 
					datacount++;
					wveloFLAG_F = atof (wveloFLAG);
					//printf ("wveloFLAG %f\n",wveloFLAG_F);
					break;
				case 35 :
					datacount++;
					break;
				case 36 :
					datacount++;
					break;
				case 37 :
					datacount++;
					break;
				case 38 :
					datacount++;
					break;
				case 39 : //roll pitch yaw n/a
					datacount++;
					break;
				case 40 :
					datacount++;
					break;
				case 41 :
					datacount++;
					break;
				case 42 :
					strncpy(altitudeMin, pch, sizeof altitudeMin - 1); 
					datacount++;
					altitudeMin_F = atof (altitudeMin);
					printf ("float altitude Min %f\n",altitudeMin_F);
					break;
				case 43 :
					strncpy(Temperature, pch, sizeof Temperature - 1); 
					datacount++;
					Temperature_F = atof (Temperature);
					//printf ("float Temperature %f\t",Temperature_F);
					break;
				case 44 : //Press n/a
					datacount++;
					break;	
				case 45 :
					strncpy(salinity, pch, sizeof salinity - 1); 
					datacount++;
					salinity_F = atof (salinity);
					//printf ("User defined salinity %f\t",salinity_F);
					break;	
				case 46 :
					strncpy(soundspeed, pch, sizeof soundspeed - 1); 
					datacount++;
					soundspeed_F = atof (soundspeed);
					//printf ("User defined soundspeed %f\t",soundspeed_F);
					break;
				case 47 :
					strncpy(ceksum, pch, sizeof ceksum - 1); 
					datacount++;
					ceksum_F = atof (ceksum);
					//printf ("User defined ceksum %f\t",ceksum_F);
					
					Speed = sqrt((veloX_F*veloX_F)+(veloY_F*veloY_F)+(veloZ_F*veloZ_F));
					break;
				 default : 
					// Process for all other cases.
					//printf ("Default/n");
					break;
				}

			  pch = strtok (NULL, " ");
			  
		    }
			datacount = 1;
			DVLValid = 0;
		}
		usleep(1000);
	}
	else
	{
		//cout << "data not available" << endl;
		 // Clear IO buffer
		 //tcflush(_fd, TCIOFLUSH);
		//write(_fd,"no dvl  !\r\n",11);
		//tcdrain(_fd);
	}
	datafull = 0;
	return 0;
}

void ReadMiniCT(int _fd)
{
	printf("Reading MiniCT\r\n");
	fd_set readfs;
	timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 10000;
	char * buffer = new char[300];
	FD_ZERO(&readfs);
	FD_SET(_fd, &readfs);
	select(_fd+1, &readfs, NULL, NULL, &tv /* no timeout */);
	
	if (FD_ISSET(_fd, &readfs))
	{
		//gettimeofday(&start, NULL);
		usleep(10000);
		int r = read(_fd, buffer, 300);
		printf("read %s\n",buffer);
		if(r == -1)
		{
			cout << strerror(errno) << endl;
		}
		string my_string(buffer,buffer+r);
		
		//MiniCTValid = 1;
		//printf("MiniCT Data received\n");
		if (my_string[0] == '1'|| my_string[0] == '2'|| my_string[0] == '3'|| my_string[0] == '4'|| my_string[0] == '5')/*Sorry :)*/
		{
			MiniCTValid = 1;
			printf("MiniCT Data received\n");
			//write(_fd,"DVL Received/r/n",14);
		}
		if (MiniCTValid == 1)
		{
		    char * pch;
			//printf("Mini CT Hi 1\n");
		    pch = strtok (buffer,"\t");
			//printf("%s\n",pch);
			while (pch != NULL && datacount<3)
		    {
				switch ( datacount ) 
				{
				case 1 : 
					/*  //strcpy(header, pch); 
					strncpy(header, pch, sizeof header - 1); 
					printf("%s\t",header)
					datacount++;
					break; */
				//case 2 :  
					//strcpy(error_code, pch); 
					strncpy(c_CT_temperature, pch, sizeof c_CT_temperature - 1); 
					
					CT_temperature = atof (c_CT_temperature);
					printf ("CT_temperature %f\n",CT_temperature);
					datacount++;	
					break;
				case 2 : 
					//strcpy(error_code, pch); 
					//printf ("Hei\n");
					strncpy(c_CT_conductivity, pch, sizeof c_CT_conductivity - 1); 
					datacount++;
					CT_conductivity = atof (c_CT_conductivity);
					printf ("CT_conductivity %f\n",CT_conductivity);
					break;
				 default : 
					// Process for all other cases.
					//printf ("Default/n");
					break;
				}
			  pch = strtok (NULL,"\t");
			  
		    }
			datacount = 1;
			MiniCTValid = 0;
		}
		usleep(1000);
	}
	else
	{
		//cout << "data not available" << endl;
		 // Clear IO buffer
		 //tcflush(_fd, TCIOFLUSH);
		//write(_fd,"no miniCT !\r\n",13);
		//tcdrain(_fd);
	}
	datafull = 0;
	//printf ("Hei\n");
}

void TriggerAlti(int _fd)
{
	//Trigger and waitFor
	write(_fd,"S\r\n",3);
	tcdrain(_fd);
	printf("Trigger Alti\n");
	//usleep(1000000);
}

void TriggerDVL(void)
{
	//Trigger 20ms
	// Set output to high
        if ((myOutputHandle = fopen(GPIOValue, "rb+")) == NULL){
            printf("GPIO: Unable to open value handle\n");
            //return 1;
        }
        strcpy(setValue, "1"); // Set value high
        fwrite(&setValue, sizeof(char), 1, myOutputHandle);
        fclose(myOutputHandle);
		//printf("HIGH\n");
        usleep(30000);
		//sleep(2);
	
 
    // Set output to low
        if ((myOutputHandle = fopen(GPIOValue, "rb+")) == NULL){
            printf("GPIO: Unable to open value handle\n");
            //return 1;
        }
        strcpy(setValue, "0"); // Set value low
        fwrite(&setValue, sizeof(char), 1, myOutputHandle);
        fclose(myOutputHandle);
		//printf("LOW\n");
		//sleep(2);
	printf("DVL Triggered\n");
}

void Navigasi(void)
{
	
	//Input
	double Omega_Pitch_rad = Omega_Pitch * PI / 180.0;
	double Omega_Heading_rad = Omega_Heading * PI / 180.0;
	double Omega_Roll_rad = Omega_Roll * PI / 180.0;
	/*
	double Nav_long=0;
	double Nav_z=0;
	double Nav_zKF=0;
	*/
	
	
	
	//Transformasi koordinat IMU
	double udot_ned = (cos(Omega_Pitch_rad)*cos(Omega_Heading_rad)*accel_X)+(cos(Omega_Pitch_rad)*sin(Omega_Heading_rad)*accel_Y)+(-1*sin(Omega_Pitch_rad)*accel_Z);
	double vdot_ned = sin(Omega_Roll_rad)*sin(Omega_Pitch_rad)*cos(Omega_Heading_rad)*accel_X + (sin(Omega_Roll_rad)*sin(Omega_Pitch_rad)*sin(Omega_Heading_rad)+cos(Omega_Roll_rad)*cos(Omega_Heading_rad))*accel_Y + sin(Omega_Roll_rad)*cos(Omega_Pitch_rad)*accel_Z;
	double wdot_ned = (cos(Omega_Roll_rad)*sin(Omega_Pitch_rad)*cos(Omega_Heading_rad)+sin(Omega_Roll_rad)*sin(Omega_Heading_rad))*accel_X + (cos(Omega_Roll_rad)*sin(Omega_Pitch_rad)*sin(Omega_Heading_rad)*-1*sin(Omega_Roll_rad)*cos(Omega_Heading_rad))*accel_Y + cos(Omega_Roll_rad)*cos(Omega_Pitch_rad)*accel_Z; 
	
	//Transformasi Koordinat DVL
	double vN_DVL = veloX_F*(cos(Omega_Heading_rad)*cos(Omega_Pitch_rad))+veloY_F*(cos(Omega_Heading_rad)*sin(Omega_Pitch_rad)*sin(Omega_Roll_rad)-sin(Omega_Heading_rad)*cos(Omega_Roll_rad))+veloZ_F*(cos(Omega_Heading_rad)*sin(Omega_Pitch_rad)*cos(Omega_Roll_rad)+sin(Omega_Heading_rad)*sin(Omega_Roll_rad));
	double vE_DVL = veloX_F*(sin(Omega_Heading_rad)*cos(Omega_Pitch_rad)) + veloY_F*(sin(Omega_Heading_rad)*sin(Omega_Pitch_rad)*sin(Omega_Roll_rad)+ cos(Omega_Heading_rad)*cos(Omega_Roll_rad))+ veloZ_F*(sin(Omega_Heading_rad)*sin(Omega_Pitch_rad)*cos(Omega_Roll_rad)*-1*cos(Omega_Heading_rad)*sin(Omega_Roll_rad));
	double vD_DVL = veloX_F*(-1*sin(Omega_Pitch_rad))+veloY_F*(cos(Omega_Pitch_rad)*sin(Omega_Roll_rad))+veloZ_F*(cos(Omega_Pitch_rad)*cos(Omega_Roll_rad));
	
	//printf("VN VE VD DVL %f %f %f\n",vN_DVL,vE_DVL,vD_DVL);
	//Hitung kecepatan IMU
	if(Nav_lat==0)Nav_lat=lat_gps;
	
	
	double omegaE = 7.292115 * 0.00001;
	double R = 	6378137;
	
	double aIMU_N = udot_ned-2*omegaE*sin(Nav_lat)*vE_IMU-(vE_IMU*vE_IMU)/R*tan(Nav_lat)+vN_IMU*vD_IMU/R;
	double aIMU_E = vdot_ned-2*omegaE*sin(Nav_lat)*vN_IMU-vE_IMU*vN_IMU/R*tan(Nav_lat)-2*omegaE*cos(Nav_lat)*vD_IMU-vE_IMU*vD_IMU/R;
	double aIMU_D = wdot_ned-2*omegaE*cos(Nav_lat)*vE_IMU-(vE_IMU*vE_IMU)/R-(vN_IMU*vN_IMU)/R;
	
	vN_IMU = vN_IMU + aIMU_N * 1;//kali selang waktu
	vE_IMU = vE_IMU + aIMU_E * 1;//kali selang waktu
	vD_IMU = vD_IMU + aIMU_D * 1;//kali selang waktu
	
	//Galat Kecepatan
	double dvN = vN_IMU-vN_DVL;
	double dvE = vE_IMU-vE_DVL;
	double dvD = vD_IMU-vD_DVL;
	//printf("dVN dVE dVD DVL %f %f %f\n",dvN,dvE,dvD);
	
	
	//Kalman Filter
		meas << dvN,
				dvE,
				dvD;
		
		if(kalman_reset==0){
			xhat = VectorXd::Zero(5);
			//inisialisasi measurement
			P = MatrixXd::Identity(5,5);
			A << 1, 9.7803, 0, 0, 0,
				-0.00000015679, 1, 0, 0, 0,
				0, 0, 1, 9.7803, 0,
				0, 0, 0.00000015679, 1, 0,
				0, 0, 0, 0, 1;
			Atrans = A.transpose();
			C << 1, 0, 0, 0, 0,
				0, 0, 1, 0, 0,
				0, 0, 0, 0, 1;
			Ctrans = C.transpose();
			matrixR << 0.0303, 0, 0,
					0, 0.0297, 0,
					0, 0, 0.0211;
			//matrixR = matrixR.sqrt();
			kalman_reset=1;
		}
		
		
				
		
		xhat = A*xhat;
		//Covariance matrix
		P = A*P*Atrans + Q;
		//Calculate Kalman Gain
		PCtrans = P*Ctrans;
		CPCtransR = C*P*Ctrans + matrixR;
		K = PCtrans * CPCtransR.inverse();
		//calculate the measurement residual
		Resid = meas - C*xhat;
		//Update the state and error covariance estimate
		xhat = xhat + K*Resid;
		//printf("xhat %f %f %f %f %f\n",xhat(0),xhat(1),xhat(2),xhat(3),xhat(4));
		P = (identitysizeK - K*C)*P;

		//output Kalman
		xhatOut = xhat;
		yhatOut = C*xhatOut;
		double vN_kalman = yhatOut(0);
		double vE_kalman = yhatOut(1);
		double vD_kalman = yhatOut(2);
		//printf("vN vE vD kalman %f %f %f\n",vN_kalman,vE_kalman,vD_kalman);
		
	
	//Estimasi Kecepatan
		double vN_est = vN_IMU-vN_kalman;
		double vE_est = vE_IMU-vE_kalman;
		double vD_est = vD_IMU-vD_kalman;
		printf("Vest %f/n",vE_est	);
		Nav_kecN = vN_est;
		Nav_kecE = vE_est;
		Nav_kecD = vD_est;
		
		double lat_aug = vN_est / R;
		double lng_aug = vE_est / R*cos(lat_aug * PI / 180.0);
		double depth_aug = vD_est;
	
	//integrator
		posisilat_aug += lat_aug;
		posisilng_aug += lng_aug;
		posisidepth_aug += depth_aug;
		Nav_N += Nav_kecN;
		Nav_E += Nav_kecE;

	//hasil integrator ditambah posisi awal
		
		Nav_lat = posisilat_aug + lat_gps;
		Nav_long = posisilng_aug + lng_gps;
		Nav_z = posisidepth_aug + depth_gps;
	
	//Perhitungan Posisi NED
	
	//Complementary Filter
	Z_KF = Nav_z;
	printf("Sounder %f\n",Sounder_depth);
	if(Z_KF==0.0)
	{
		Z_Complement = Sounder_depth;
		printf("IF Z_Complement %f\n",Z_Complement);
	
	}
	else{
		
		Z_Complement = (Sounder_depth*1/1.01) + (Z_Complement + Z_KF - Z_KF_m)*(1.01-1/1.01);
		printf("ELSE Z_Complement %f\n",Z_Complement);
	}
	Z_KF_m = Z_KF;
}

void GuidanceLOS(void)
{
	double WpXnow,WpYnow;
	//Decide target waypoint, using previous distance
	//Decide wp X and Decide wp Y
	
	if(distancefromtarget<=acceptance_radius)
    {
        guidance_wp_target=guidance_wp_target+1;
        if(guidance_wp_target==waypoint_number)
        {
            guidance_finish=1;
            //guidance_wp_target=guidance_wp_target-1;
        }
		else
		{
			guidance_finish=0;
		}
		
    }
	else
	{
		guidance_finish=0;
	}
	WpXnow = wpX[1]; //wpX[guidance_wp_target];
	WpYnow = wpY[1];//wpY[guidance_wp_target];
	
	//Ambil data posisi: 1st priority navigasi NED 2nd priority IMU
	double Current_positionY = wpY[0];//Nav_lat;
	double Current_positionX = wpX[0];//Nav_long;
	
	//LOS
	psiref =  atan2((WpYnow-Current_positionY),(WpXnow-Current_positionX));
	distancefromtarget = sqrt( pow((WpXnow-Current_positionX),2) + pow((WpYnow-Current_positionY),2));
	psiref = psiref*180.0/PI;
	printf("psiref = %f\n",psiref);
}

void LogData(void)
{
	//printf("Reading LogData \n");
	
	datalog_entrynumber++;
	printf("%d entries \n",datalog_entrynumber);
	
	//string filename = "buffer.txt";
	string filename2 = "logger.txt";
	
	//ofstream buffer(filename.c_str());  // default mode is ios::out | ios::trunc
    ofstream log( filename2.c_str(), ios::out | ios::app );

    timeval curTime;
    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec / 1000;

    char buffers [80];
    strftime(buffers, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));

    char currentTime[84] = "";
    char datatulis[1000] = "";


    // Data yang ditulis ke logger
    /*
    DVL
    float altitude1_F,altitude2_F,altitude3_F,altitude4_F;
	float veloX_F,veloY_F,veloZ_F,veloFLAG_F;
	float wveloX_F,wveloY_F,wveloZ_F,wveloFLAG_F;
	float altitudeMin_F,
	float Speed
    
	IMU
	double Latitude;
	double Longitude;
	double Height;
	double Roll;
	double Pitch;
	double Heading;
	double accel_X;
	double accel_Y;
	double accel_Z;
	double Omega_Roll;		
	double Omega_Pitch;		
	double Omega_Heading;

	MiniCT
	double CT_conductivity;
	double CT_temperature

	ALti
	double Sounder_depth;
	double Sounder_altitude;
	double Sounder_pressure;
	Navigasi
	Guidance

    */
    //printf("current time: %s \n", currentTime);
    //buffer << currentTime << "\n";
    if (newlogfile==1)
	{
		log<< "Log Data "<< currentTime << "\n";
		printf("Begin data log\n");
		newlogfile = 0;
		sprintf(datatulis, "altitude1, altitude2, altitude3, altitude4, veloX, veloY, veloZ, veloFLAG, wveloX, wveloY, wveloZ, wveloFLAG, altitudeMin, Speed, Latitude, Longitude, Height, Roll, Pitch, Heading, accel_X, accel_Y, accel_Z,Omega_Roll, Omega_Pitch, Omega_Heading,CT_conductivity, CT_temperature,Sounder_depth,Sounder_pressure,Sounder_altitude, Nav_Lat, Nav_Long, Nav_Z,kecepatanX, kecepatanY, kecepatanZ, ZComp, Nav_N, Nav_E, psiref, guidestat\n");
		sprintf(currentTime, "%s:%d", buffers, milli);
		log << datatulis << ",";
		log << currentTime << "\n";
	}
	
	if(datalog_entrynumber>=datalog_entrylimit)
	{
    //versi simple data dummy
    //sprintf(datatulis, "%5f, %5f, %5f, %5f, %5f", log_a, log_b, log_c, log_d, log_e);

    //versi simple logging (data penting aja yang ditulis)
    //sprintf(datatulis, "%5f, %5f, %5f, %5f, %5f, %5f, %5f, %5f, %5f, %5f", Roll, Pitch, Yaw, Latitude, Longitude, Height, altitude1_F, veloX_F, wveloX_F , altitudeMin_F);

    //versi full data log (semua di tulis, boros memori) DVL IMU MiniCT Alti Navigasi Guidance
	
    sprintf(datatulis, "%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%d", 
	altitude1_F,altitude2_F,altitude3_F,altitude4_F,
	veloX_F,veloY_F,veloZ_F,veloFLAG_F,
	wveloX_F,wveloY_F,wveloZ_F,wveloFLAG_F,
	altitudeMin_F, Speed,
	Latitude, Longitude, Height, Roll, Pitch, Heading, 
	accel_X, accel_Y, accel_Z,
	Omega_Roll, Omega_Pitch, Omega_Heading,
	CT_conductivity, CT_temperature,
	Sounder_depth,Sounder_pressure,Sounder_altitude,
	Nav_lat, Nav_long, Nav_z,
	Nav_kecN, Nav_kecE, Nav_kecD,
	Z_Complement, Nav_N, Nav_E,
	psiref, guidance_wp_target
	);
	log << datatulis << ",";
    sprintf(currentTime, "%s:%d", buffers, milli);
	log << currentTime << "\n";
	}
    //buffer.close();
    log.close();
}

void SendTS(void)
{
	//Protokol data yang dikirimkan ke TS
	// $#NG teta_terukur z_terukur yawref yaw_terukur altitude roll salinity temperature speed latitude longitude
	 
	// needed data

	/*
	temp
	altitude
	depth
	lat
	lon
	roll
	pitch
	yaw (heading)
	motion status
	battery
	leakage
	speed
	strobo
	
	$#NG teta_terukur(IMU) z_terukur(Nav) yawref(Guidance) yaw_terukur(IMU) altitude(DVL) conductivity(CT) temperature(CT) speed(DVL) latitude(Nav) longitude(Nav)  roll(IMU) guidestat(guidancewaypointtracking) Propellerspeed(Scenario1) Scenario1_state(Scenario1)
	
	*/
	printf("SendTS\r\n");
	/*Dummy Test*
	// VARIABEL DUM_ itu dummy dhil, ganti sama yang beneran output Navigasi dan Guidance
	stream_teta << fixed << setprecision(10) << dum_teta;         
	stream_z    << fixed << setprecision(10) << dum_z;
	stream_yawr << fixed << setprecision(10) << dum_yawr;
	stream_yawt << fixed << setprecision(10) << dum_yawt;
	//stream_posx << fixed << setprecision(10) << dum_posx;
	//stream_posy << fixed << setprecision(10) << dum_posy;
	stream_alti << fixed << setprecision(10) << dum_alti;
	//stream_dept << fixed << setprecision(10) << dum_dept;
	stream_sali << fixed << setprecision(10) << dum_sali;
	stream_temp << fixed << setprecision(10) << dum_temp;
	stream_sped << fixed << setprecision(10) <<	dum_sped;
	stream_lati << fixed << setprecision(10) << dum_lati;
	stream_long << fixed << setprecision(10) << dum_long;
	End Dummy Test*/
	
	//Mapp variable
	stream_teta << fixed << setprecision(10) << Pitch;         
	stream_z    << fixed << setprecision(10) << altitudeMin_F;//Depth???? 1. Alti Depth 2. Navigasi Z topi?
	if(State_Scenario_1 == 5)
	{
		psiref=yawref_Scenario1;
	}
	else if(State_Scenario_1>0)
	{
		psiref = 0.0;
	}
	stream_yawr << fixed << setprecision(10) << psiref;
	stream_yawt << fixed << setprecision(10) << Heading;
	stream_posx << fixed << setprecision(10) << Nav_N;
	stream_posy << fixed << setprecision(10) << Nav_E;
	stream_alti << fixed << setprecision(10) << Z_Complement;//Sounder_depth;
	stream_sali << fixed << setprecision(10) << CT_conductivity; //N/A yet
	stream_temp << fixed << setprecision(10) << CT_temperature;//N/A yet
	stream_sped << fixed << setprecision(10) <<	Speed;
	stream_lati << fixed << setprecision(13) << Nav_lat;//Latitude;
	stream_long << fixed << setprecision(13) << Nav_long;//Longitude;
	stream_roll << fixed << setprecision(10) << Roll;

	string_teta = stream_teta.str();
	string_z    =    stream_z.str();
	string_yawr = stream_yawr.str();
	string_yawt = stream_yawt.str();
	string_posx = stream_posx.str();
	string_posy = stream_posy.str();
	string_alti = stream_alti.str();
	//string_dept = stream_dept.str();
	string_sali = stream_sali.str();
	string_temp = stream_temp.str();
	string_sped = stream_sped.str();
	string_lati = stream_lati.str();
	string_long = stream_long.str();
	string_roll = stream_roll.str();

	stream_teta.str("");  
	stream_z.str("");     
	stream_yawr.str("");  
	stream_yawt.str("");  
	stream_posx.str("");  
	stream_posy.str("");  
	stream_alti.str("");  
	//stream_dept.str("");  
	stream_sali.str("");  
	stream_temp.str("");  
	stream_sped.str("");  
	stream_lati.str("");  
	stream_long.str("");  
	stream_roll.str(""); 

	stream_teta.clear();
	stream_z.clear();
	stream_yawr.clear();
	stream_yawt.clear();
	stream_posx.clear();
	stream_posy.clear();
	stream_alti.clear();
	//stream_dept.clear();
	stream_sali.clear();
	stream_temp.clear();
	stream_sped.clear();
	stream_lati.clear();
	stream_long.clear();
	stream_roll.clear();

	strcpy( char_teta, string_teta.c_str() );
	strcpy( char_z   ,    string_z.c_str() );
	strcpy( char_yawr, string_yawr.c_str() );
	strcpy( char_yawt, string_yawt.c_str() );
	strcpy( char_posx, string_posx.c_str() );
	strcpy( char_posy, string_posy.c_str() );
	strcpy( char_alti, string_alti.c_str() );
	//strcpy( char_dept, string_dept.c_str() );
	strcpy( char_sali, string_sali.c_str() );
	strcpy( char_temp, string_temp.c_str() );
	strcpy( char_sped, string_sped.c_str() );
	strcpy( char_lati, string_lati.c_str() );
	strcpy( char_long, string_long.c_str() );
	strcpy( char_roll, string_roll.c_str() );
	
	write(fd,"$#NG",4);
	write(fd," ",1);
	write(fd,char_teta,9);
	write(fd," ",1);
	write(fd,char_z   ,9);
	write(fd," ",1);
	write(fd,char_yawr,9);
	write(fd," ",1);
	write(fd,char_yawt,9);
	write(fd," ",1);
	write(fd,char_posx,9);
	write(fd," ",1);
	write(fd,char_posy,9);
	write(fd," ",1);
	write(fd,char_alti,9);
	write(fd," ",1);
	//write(fd,char_dept,9);
	//write(fd," ",1);
	write(fd,char_sali,9);
	write(fd," ",1);
	write(fd,char_temp,9);
	write(fd," ",1);
	write(fd,char_sped,9);
	write(fd," ",1);
	write(fd,char_lati,12);
	write(fd," ",1);
	write(fd,char_long,12);
	write(fd," ",1);
	write(fd,char_roll,9);
	/*
	write(fd," ",1);
	if(guidance_wp_target==0)
	{
		write(fd,"0 ",2);
	}
	else if(guidance_wp_target==1)
	{
		write(fd,"1 ",2);
	}
	else if(guidance_wp_target==2)
	{
		write(fd,"2 ",2);
	}
	else if(guidance_wp_target==3)
	{
		write(fd,"3 ",2);
	}
	//V propeller
	if(V_propeller==0)
	{
		write(fd,"0",1);
	}
	else if(V_propeller==1)
	{
		write(fd,"25",2);
	}
	else if(V_propeller==2)
	{
		write(fd,"50",2);
	}
	else if(V_propeller==3)
	{
		write(fd,"75",2);
	}
	else if(V_propeller==4)
	{
		write(fd,"100",3);
	}
	write(fd," ",1);
	//Scenario Status
	if(State_Scenario_1==0)
	{
		write(fd,"0",1);
	}
	else if(State_Scenario_1==1)
	{
		write(fd,"1",1);
	}
	else if(State_Scenario_1==2)
	{
		write(fd,"2",1);
	}
	else if(State_Scenario_1==3)
	{
		write(fd,"3",1);
	}
	else if(State_Scenario_1==4)
	{
		write(fd,"4",1);
	}
	else if(State_Scenario_1==5)
	{
		write(fd,"5",1);
	}
	*/
	write(fd,"\r\n",2);
	tcdrain(fd);
	/*
	write(fd,"$#NG",4);
	write(fd," ",1);
	write(fd,char_teta,9);
	write(fd," Alti ",6);
	write(fd,char_z   ,9);
	write(fd," \r\n",3);
	write(fd,char_yawr,9);
	write(fd," ",1);
	write(fd,char_yawt,9);
	write(fd," ",1);
	//write(fd,char_posx,9);
	//write(fd," ",1);
	//write(fd,char_posy,9);
	//write(fd," ",1);
	write(fd,char_alti,9);
	write(fd," \r\nCT ",6);
	//write(fd,char_dept,9);
	//write(fd," ",1);
	write(fd,char_sali,9);
	write(fd," ",1);
	write(fd,char_temp,9);
	write(fd," \r\nDVL ",7);
	write(fd,char_sped,9);
	write(fd," \r\n",3);
	write(fd,char_lati,12);
	write(fd," ",1);
	write(fd,char_long,12);
	write(fd," \r\n",3);
	write(fd,char_roll,9);
	write(fd," ",1);
	if(guidance_finish==1)
	{
		write(fd,"1 ",2);
	}
	else
	{
		write(fd,"0 ",2);
	}
	write(fd,"\r\n",2);
	tcdrain(fd);
	*/
	
}

void rt_OneStep(void)
{
  static boolean_T OverrunFlag = false;

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    rtmSetErrorStatus(rtObj.getRTM(), "Overrun");
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here

  // Step the model for base rate
  rtObj.step();

  // Get model outputs here

  // Indicate task complete
  OverrunFlag = false;

  // Disable interrupts here
  // Restore FPU context here (if necessary)
  // Enable interrupts here
}

void Scenario_1(void){
	printf("Scenario 1\n");
	
	//State_Scenario 1
	// 0 skenario non aktif
	// 1 start sampai 50 meter
	// 2 gliding sampai pitch down
	// 3 pitch down sampai pitch up ke permukaan
	// 4 50 meter ke akhir
	// 5 selesai skenario
	
	//State 1
	//ambil koordinat home, ambil koordinat posisi wahana dari GPS, hitung jarak, hitung mapping.
	// Jika jarak >50m ke state 2
	
	//State 2 
	//tunggu sampai depthnya cukup dalam, pindah state 3
	
	//State 3
	//tunggu sampai depthnya cukup dangkal, pindah state 4 dengan mengambil gps
	
	//State 4
	// ambil koordinat posisi wahana dari GPS, hitung jarak terhadap GPS yang diambil di state 3, mapping
	// Jika jarak >50m motor 0, kasih flag 5 ke TS, ambil koordinat gps, Glider muter 180, 
	
	//State 5
	
	
	//Output
	/*
		1. Sequence scenario 1
		2.  kecepatan propeller
	*/
	double current_Lat, current_Long;
	double distance;
	static double surfacepoint_Lat, surfacepoint_Long;
	static int returnSequence=0;
	
	if(State_Scenario_1 == 1)
	{
	//ambil koordinat home, setting dengan $#G1 lat long, atau ketika pertama di air
	
	//ambil koordinat posisi wahana dari GPS,
	current_Lat= Latitude;
	current_Long= Longitude;
	
	//hitung jarak, hitung mapping.(
	if (returnSequence==0)
	{
	distance = 1000*distance_two_coordinates(wpY[0],wpX[0],current_Lat,current_Long);
	}
	else
	{
	distance = 1000*distance_two_coordinates(surfacepoint_Lat,surfacepoint_Long,current_Lat,current_Long);
	}
	if (distance<13)
	{
		V_propeller=4;
	}
	else if (distance<25)
	{
		V_propeller=3;
	}
	else if (distance<38)
	{
		V_propeller=2;
	}
	else if (distance<50)
	{
		V_propeller=1;
	}
	else
	{
		State_Scenario_1 = 2;
	}
	
	// Jika jarak >50m ke state 2
	
	
	}
	
	//State 2 
	//tunggu sampai depthnya cukup dalam, pindah state 3
	else if(State_Scenario_1 == 2)
	{
		if(Sounder_depth>12)
		{
			State_Scenario_1 = 3;
		}
	}
	//State 3
	//tunggu sampai depthnya cukup dangkal, pindah state 4 dengan mengambil gps
	else if(State_Scenario_1 == 3)
	{	
		if(Sounder_depth<10)
		{
			State_Scenario_1 = 4;
			surfacepoint_Lat = Latitude; 
			surfacepoint_Long = Longitude;
		}
	}
	else if(State_Scenario_1 == 4)
	{
		//ambil koordinat posisi wahana dari GPS,
		current_Lat= Latitude;
		current_Long= Longitude;
		
		//hitung jarak, hitung mapping.
		distance = 1000*distance_two_coordinates(surfacepoint_Lat,surfacepoint_Long,current_Lat,current_Long);
		if (distance<13)
		{
			V_propeller=4;
		}
		else if (distance<25)
		{
			V_propeller=3;
		}
		else if (distance<38)
		{
			V_propeller=2;
		}
		else if (distance<50)
		{
			V_propeller=1;
		}
		else
		{
			State_Scenario_1 = 5;
			surfacepoint_Lat = Latitude; 
			surfacepoint_Long = Longitude;
			yawref_Scenario1 = Heading-180.0;
			if(yawref_Scenario1<0)yawref_Scenario1+=360;
		}
	
	}
	else if(State_Scenario_1 == 5)
	{
		//Kalau nunggu command baru sequence balik di comment aja
		if ((yawref_Scenario1-Heading)<5 || (Heading-yawref_Scenario1)<5)
		{
			State_Scenario_1 = 1;
		}
		
		
	}
}

void RestartRelay(void){
	// Set output to low
        if ((myOutputHandle_1 = fopen(GPIOValue_1, "rb+")) == NULL){
            printf("GPIO: Unable to open value handle\n");
            //return 1;
        }
        strcpy(setValue_1, "0"); // Set value low
        fwrite(&setValue_1, sizeof(char), 1, myOutputHandle_1);
        fclose(myOutputHandle_1);
		printf("Relay OFF\n");
		sleep(3);
	
	// Set output to high
        if ((myOutputHandle_1 = fopen(GPIOValue_1, "rb+")) == NULL){
            printf("GPIO: Unable to open value handle\n");
            //return 1;
        }
        strcpy(setValue_1, "1"); // Set value high
        fwrite(&setValue_1, sizeof(char), 1, myOutputHandle_1);
        fclose(myOutputHandle_1);
		printf("Relay ON\n");
}


double distance_two_coordinates(double lat1,double lon1,double lat2,double lon2){  // generally used geo measurement function
    double R = 6378.137; // Radius of earth in KM
    double dLat = (lat2 * PI/ 180.0) - (lat1 *PI / 180.0);
    double dLon = (lon2 * PI / 180.0) - (lon1 * PI / 180.0);
    double a = sin(dLat/2.0) * sin(dLat/2.0) +
    cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
    sin(dLon/2.0) * sin(dLon/2.0);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = R * c;
    return d;//Kilometers
}
//1. Comm Testing
//--Activate the BB -> $#O1
//-- Stop the BB -> $#O0
//-- Stop the BB experimental -> $#O9

//2. DVL Data
/*
$#NQ.RES 0X0000 1 1 1 1 4.3 4.5 4.3 4.2 11.1 49.9 52.4 42.4 0 0 0 0 0 0 0 0 -55.1 10.0 42.0 1 -55.1 10.0 42.0 1 0 0 0 0 0 0 0 0 0 0 0 4.2 15.0 65.882 35.0 1500 74
$#NQ.RES 0X0000 
1 1 1 1 beam quality
4.3 4.5 4.3 4.2 Altitude
11.1 49.9 52.4 42.4 velo rad
0 0 0 0 wvelo
0 0 0 0 cwvelo
-55.1 10.0 42.0 1 bottom
-55.1 10.0 42.0 1 bottom earth
0 0 0 0 water
0 0 0 0 water earth
0 0 0 rph
4.2 Altitude
15.0 Temp
65.882 pressure INVALID
35.0 salinity?
1500 soundspeed
74 cek sum
,00000000000000,35.0,+15.0,0000.0,1500.0,000
:WI,+00000,+00000,+00000,+00000,I
:BI,+00000,-02345,-00439,+00000,A
:BD,+00000000.00,+00000000.00,+00000000.00,0000.40,000.00
$VDDPT,0.400,0


*/

/*
data protocol to GCS
head day mon da year    time  temp  sali   alt   dept     lat       lon        roll         pit         yaw    mo  bat lea spd  s CS   
$#GB Sat Aug 13 2016 17:34:01 29.16 29.05 26.15	10.53 -6.897908	107.596415	-4.998506	 21.047638	 24.021191 01 75.5 000 10.3 1 109
total length

data protocol to TS
$#NG teta_terukur altitude yawref yaw_terukur z_terukur salinity temperature speed latitude longitude

$#NG 1.0000000 2.0000000 3.0000000 -4.000000 5.0000000 6.0000000 7.0000000 8.0000000 9.0000000 10.000000 11.000000

*/
//3. Altimeter
// $PRVAT, xx.xxx,M, xxxx.xxx, dBar*hh<CR><LF>
//$PRVAT,00.115,M,0010.073,dBar*39
/*
S
$PRVAT,00.210,M,0007.497,Metres*3a
 P823 */


//4. MiniCT
//T=<tab>nn.nnn<tab>C=<tab>nn.nnn<cr><lf>
//dr arduino -> 25.0\t20.0\t10.0\t

//Catatan Fadhil
// 20 August:	1. latitude longitude panjang stringnya perlu lebih-> cek google Map untuk akurasi 10m berapa digit berapa koma -XXX.XXXXXX(sesuai spek)
//				2. Comman Guidance perlu ditambah
//				3. Speed skalar? Dibuat Skalar dulu

//Setting Guidance
// $#G1 lat long -> set home with lat, long
// $#G1 -6.890671 107.611378
// $#G2 lat long -> set target with lat, long, the algorithm will add an intermediary waypoint every float distance step in km
// $#G2 -6.858523 107.632477
// $#G3 -> return to home coordinate, if there are intermediary waypoint, it goes in reverse order.

//Improvement
//1. DVL dua kali