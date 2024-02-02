// Kode Program Superloop
// tanpa multithread
// Thread Komunikasi ke BBB (Xuart0)

// Protokol 7 Oktober 2017
// $#NG 1.0000000 2.0000000 3.0000000 -4.000000 5.0000000 6.0000000 7.0000000 8.0000000 9.0000000 10.000000

// $#NG teta_terukur z_terukur yawref yaw_terukur altitude salinity temperature speed latitude longitude

// $#Ox operation
// $#Sx surface system
// $#R1 relay off 2 detik, on
// $#Gx guidance

// compiling g++ blabla.cpp -lpthread -o output

#include <ctime>
#include <time.h>
#include <pthread.h>
#include <fstream>
#include <iostream>
#include <sys/time.h>
#include <cstdlib>
#include <string>
#include <stdio.h>      
#include <ctime>
#include <errno.h>
#include <fcntl.h> 
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "string.h"
#include "time.h"
#include "sys/select.h"
#include <iomanip> // setprecision
#include <sstream> // stringstream

using namespace std;y

//======================================================Thread=======================================================// 
void *write_read_sensor_task(void *threadid);
void *control_actuator_task(void *threadid);
void *write_datalog_task(void *threadid);

int set_interface_attribs();
int set_interface_attribs2();
void set_mincount(int fd, int mcount);
int open_port();
void handlingGCS();
// void handlingFSM(int _fd);
// void PID_blst();
// void PID_bld();
// void PID_rd();
void handlingBBB(int _fd);
void handlingDatalog();

//TASK FLAG
int command_task_flag = 0;
int sensor_task_flag = 0;
int control_actuator_task_flag = 0;
int datalog_task_flag = 0;

//===================================================================================================================//
  
//Variabel Port Komunikasi
int fd;
int fd2;
char *portname = "/dev/ttyxuart1";
int wlen;
char *portname2 = "/dev/ttyxuart0";
int wlen2;

// timers
struct timeval start, end;
long mtime, seconds, useconds;


//==================================================== GCS ==========================================================//
//--------------------------------------------------com GCS----------------------------------------------------------//

// Data variable
// int state = 0;
char GCSinput[100];
int end2 = 0;
int GCSsend= 0;
int counterdataGCS = 0;
int headervalid = 0;

string GCSString0;
string GCSString1;
string GCSString2;
string GCSString3;
string GCSString4;
string GCSString5;

char com_header[15];
char com_GCS1[15];
char com_GCS2[15];
char com_GCS3[15];
char com_GCS4[15];
char com_GCS5[15];

//--------------------------------------------------var GCS----------------------------------------------------------//

char datapacket[140];
///cout<<"$#GB Sat Aug 13 2016 17:34:01 29.16 29.05 26.15 10.53 -6.897908 -107.596415 -4.998506 -21.047638 -24.021191 01 75.5 000 10.3 1 109A"<<endl;
//data variables
int date = 13;
int year = 2016;
int hour = 17;
int minute = 34;
int second = 1;
float temperature = -29.16;
float salinity = -29.05;
float alt = -26.15;
float depth = -10.53;
float latitude =  -6.897908;
float longitude = -107.596415;
float roll = -4.998506;
float pitch = -21.047638;
float yaw = -24.021191;
string motion = "01";
float battery = -75.9;
string leakage = "000";
float speed = -30.3;
int strobo = 0;
//char checksum = 'a';
int counter = 0;
int checksum = 0;
unsigned char xor1 = 0;

// Reference variable settings
double set_coordx       = 0;
double set_coordy       = 0;
double set_strobcount   = 0;
double set_glidemode    = 0;
double set_glidecount   = 3;
double set_glidedisance = 300;
double set_samplingrate = 1;
double set_startcoordx  = 1;
double set_startcoordy  = 1;
double set_endcoordx    = 1;
double set_endcoordy    = 1;
double set_maxdepth     = 20;
double set_glideangle   = 15;
double set_propelspeed  = 10;

//================================================ BB ===============================================================//

//variabel input dari BB
char BBinbound[150];
int BBreceive = 0;
int counterdataBB = 0;

char com_BBheader[10];
char com_BBin1[10];
char com_BBin2[10];
char com_BBin3[10];
char com_BBin4[10];
char com_BBin5[10];
char com_BBin6[10];
char com_BBin7[10];
char com_BBin8[10];
char com_BBin9[10];
char com_BBin10[10];
char com_BBin11[10];

float teta_terukur;
float z_terukur;
float yawref;
float yaw_terukur;
float altitude;
float rolls;
float salin;
float temps;
float spd;
float lat;
float lon;

//------------------------------------------------var datalog--------------------------------------------------------//

//strings
string string_header;
string string_date;
string string_time;
string string_temp;
string string_sali;
string string_alti;
string string_dept;
string string_lati;
string string_long;
string string_roll;
string string_pitc;
string string_head;
string string_moti;
string string_batt;
string string_leak;
string string_sped;
string string_stro;
string string_csum;

//stream
stringstream stream_header;
stringstream stream_date;
stringstream stream_time;
stringstream stream_temp;
stringstream stream_sali;
stringstream stream_alti;
stringstream stream_dept;
stringstream stream_lati;
stringstream stream_long;
stringstream stream_roll;
stringstream stream_pitc;
stringstream stream_head;
stringstream stream_moti;
stringstream stream_batt;
stringstream stream_leak;
stringstream stream_sped;
stringstream stream_stro;
stringstream stream_csum;

//output chars
char char_header[10];
char char_date  [10];
char char_time  [10];
char char_temp  [10];
char char_sali  [10];
char char_alti  [10];
char char_dept  [10];
char char_lati  [10];
char char_long  [10];
char char_roll  [10];
char char_pitc  [10];
char char_head  [10];
char char_moti  [10];
char char_batt  [10];
char char_leak  [10];
char char_sped  [10];
char char_stro  [10];
char char_csum  [10];

//============================================== FSM ================================================================//
//variable PID blst
float sat_blst_u = 0.5,
    sat_blst_d = -0.5,
    pitch_ref, 
    pitch_fb, // BB
    pitch_err = 0, 
    pitch_sumerr = 0, 
    pitch_lasterr, 
    PID_blst_out,
    PID_blst_cal,
    PID_blst_unsat, 
    blst_AW = 0;
    
float PID_blst_KP = 3, 
    PID_blst_KI = 1, 
    PID_blst_KD = 0.1,
    PID_blst_AW = 1;
    
float PID_blst_pro, PID_blst_int, PID_blst_dev;

//variable PID bld
float sat_bld_u = 0.5,
    sat_bld_d = -0.5,
    depth_ref, 
    depth_fb, //BB
    depth_err = 0, 
    depth_sumerr = 0, 
    depth_lasterr, 
    PID_bld_out,
    PID_bld_cal,
    PID_bld_unsat, 
    bld_AW = 0;
    
float PID_bld_KP = 20, 
    PID_bld_KI = 0.1, 
    PID_bld_KD = 0.5,
    PID_bld_AW = 1;
    
float PID_bld_pro, PID_bld_int, PID_bld_dev;

//variable PID rd
float sat_rd_u = 0.5,
    sat_rd_d = -0.5,
    heading_ref, //BB
    heading_fb,  //BB
    heading_err = 0, 
    heading_sumerr = 0, 
    heading_lasterr, 
    PID_rd_out,
    PID_rd_cal,
    PID_rd_unsat, 
    rd_AW = 0;
    
float PID_rd_KP = 3, 
    PID_rd_KI = 1, 
    PID_rd_KD = 0.1,
    PID_rd_AW = 1;
    
float PID_rd_pro, PID_rd_int, PID_rd_dev;

//variable FSM
//depth
float ref_depth_ini = 0,
      ref_depth_u = -0.5,
      ref_depth_d = -20,
      ref_bld;
//pitch
float ref_pitch_ini = 0,
      ref_pitch_u = 20,
      ref_pitch_d = -20,
      ref_blst;

float rotate_ref;

int rot, bld, blst, toleransi_deg;
    
//output ke matlab
int stop = 0,
  state; 

int guide = 0; //BB
    
int mode = 11, full = 1, half = 0;
//11 = test pitch
//12 = test heave

//variabel waktu timer
int sampling_time = 1;
struct timeval t1, t2;
//

int i = 0;

//===================================================================================================================//
	
int main(void){
    
    pthread_t comm_beagle;
    pthread_t comm_control;
    pthread_t datalog_thread;

    int rc;

/**************************** ALGORITHM ****************************/
	cout << "Program start" <<  endl;
	
	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
	
	fd2 = open(portname2, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd2 < 0) {
        printf("Error opening %s: %s\n", portname2, strerror(errno));
        return -1;
    }
	
	//Setting properti komunikasi serial
	set_interface_attribs();
	set_interface_attribs2();
  
  //Start Threads
  // rc = pthread_create(&antenna_command, NULL, read_command_task, NULL);
  // if(rc){
  //   cout << "Error:unable to create command thread, " << rc << endl;
  //   exit(-1);
  // }
  
  rc = pthread_create(&comm_beagle, NULL, write_read_sensor_task, NULL);
  if(rc){
    cout << "Error:unable to create sensor thread, " << rc << endl;
    exit(-1);
  }
  
  rc = pthread_create(&comm_control, NULL, control_actuator_task, NULL);
  if(rc){
    cout << "Error:unable to create control thread, " << rc << endl;
    exit(-1);
  }
  
  rc = pthread_create(&datalog_thread, NULL, write_datalog_task, NULL);
  if(rc){
    cout << "Error:unable to create datalog thread, " << rc << endl;
    exit(-1);
  }

  sensor_task_flag = 1;
  control_actuator_task_flag = 1;
  datalog_task_flag = 1;

  char * comGCS;
  int g;

  while(end2 == 0) {
    fd_set readGCS;
    FD_ZERO(&readGCS);
    FD_SET(STDIN_FILENO, &readGCS);
    struct timeval tv = {10, 0};
    select(STDIN_FILENO+1, &readGCS, NULL, NULL, &tv);

    if (FD_ISSET(STDIN_FILENO, &readGCS)) {
      cin.getline(GCSinput);
      comGCS = strtok(GCSinput, " ");

      while(comGCS != NULL) {
        switch (GCSsend) {
          case 0 :
                strncpy(com_header, comGCS, sizeof com_header);
                printf("GCS0 %s\n", com_header);
                GCSString0 = com_header;
                GCSsend++;
                break;

          case 1 :
                strncpy(com_GCS1, comGCS, sizeof com_GCS1);
                printf("GCS1 %s\n", com_GCS1);
                GCSString1 = com_GCS1;
                GCSsend++;
                break;

          case 2 :
                strncpy(com_GCS2, comGCS, sizeof com_GCS2);
                printf("GCS2 %s\n", com_GCS2);
                GCSString2 = com_GCS2;
                GCSsend++;
                break;

          case 3 :
                strncpy(com_GCS3, comGCS, sizeof com_GCS3);
                printf("GCS3 %s\n", com_GCS3);
                GCSString3 = com_GCS3;
                GCSsend++;
                break;

          case 4 :
                strncpy(com_GCS4, comGCS, sizeof com_GCS4);
                printf("GCS4 %s\n", com_GCS4);
                GCSString4 = com_GCS4;
                GCSsend++;
                break;

          case 5 :
                strncpy(com_GCS5, comGCS, sizeof com_GCS5);
                printf("GCS5 %s\n", com_GCS5);
                GCSString5 = com_GCS5;
                GCSsend++;
                break;

          default :
                break;

        }

        comGCS = strtok (NULL, " ");

      }
      counterdataGCS = GCSsend;
      
      if (GCSString1 == "#GB>") {
          cout << "header valid" << endl;
          headervalid = 1;
      }

      if ((counterdataGCS > 2) && (headervalid == 1)) {
        if (GCSString1 == "atc") {
          cout << "command received" << endl;

          write(fd2, "$#O1", 4); //start program beagle
          //$#G1 latitude longitude //set home point
          //$#G2 latitude longitude //set target point
          //$#G3 latitude longitude //override heading to home point

          if (GCSString2 == "rtb") {
            cout << "returning to base" << endl;
            //control_actuator_task_flag = 1;
            //sensor_task_flag = 1;
            //write(fd2, "$#O1", 4);
            write(fd2, "$#G3", 4);
            //write(fd2, " ", 1);
            //write(fd2, com_GCS4, sizeof (com_GCS4)); //latitude
            //write(fd2, " ", 1);
            //write(fd2, com_GCS3, sizeof (com_GCS3)); //longitude
            //mode = 7? //surface action
          }

          else if (GCSString2 == "ctc") {
            if (counterdataGCS > 4) {
              set_coordx = atof (com_GCS3);
              set_coordy = atof (com_GCS4);
              //write(fd2, "$#G2", 4);
              //write(fd2, " ", 1);
              //write(fd2, com_GCS4, sizeof (com_GCS4)); //latitude
              //write(fd2, " ", 1);
              //write(fd2, com_GCS3, sizeof (com_GCS3)); //longitude
              cout << "input coordinate success" << endl;
            }

            else {
              cout << "not enough input" << endl;
            }
          }

          else if (GCSString2 == "kss") {
            cout << "staying at surface" << endl;
            // handlingFSM();
            // mode = 8; //surfacing net
          }

          else if (GCSString2 == "bsl") {
            if (counterdataGCS > 3) {
              set_strobcount = atof (com_GCS3);
              cout << "strobo on count" << endl;
            }

            else {
              cout << "not enough input" << endl;
            }
          }

          else if (GCSString2 == "rto") {
            cout << "continue mission" << endl;
            write(fd2,"$#O1",4);
          }

          else if (GCSString2 == "mode") {
            if (counterdataGCS > 5) {
              set_glidemode = atof (com_GCS3);
              set_glidecount = atof (com_GCS4);
              set_glidedisance = atof (com_GCS5);
              cout << "glide mode set" << endl;
            }

            else {
              cout << "not enough input" << endl;
            }
          }

          else if (GCSString2 == "pitchd") {
                //tes pitch down
                cout<<"tes pitch down"<<endl;
                pitch_ref = ref_pitch_d;
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, "\n", 1);
                sleep(30);
                pitch_ref = ref_pitch_ini;
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, "\n", 1);
              }

              else if (GCSString2 == "pitchu") {
                //tes pitch up
                cout<<"pitch up"<<endl;
                pitch_ref = ref_pitch_u;
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, "\n", 1);
                sleep(30);
                pitch_ref = ref_pitch_ini;
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, "\n", 1);
              }

              else if (GCSString2 == "heaved") {
                //tes heave down
                cout<<"heave down"<<endl;
                depth_ref = ref_depth_d;
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, "\n", 1);
                sleep(30);
                depth_ref = ref_depth_ini;
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, "\n", 1);
              }

              else if (GCSString2 == "heaveu") {
                //tes heave up
                cout<<"heave up"<<endl;
                depth_ref = ref_depth_u;
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, "\n", 1);
                sleep(30);
                depth_ref = ref_depth_ini;
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, "\n", 1);
              }

              else if (GCSString2 == "topen") {
                //tes glide
                cout<<"glide test"<<endl;
                pitch_ref = ref_pitch_d;
                depth_ref = ref_depth_d;
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "-0.3", 4); //bladder
                write(fd2, " ", 1); write(fd2, "0.1", 3); //ballast
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1); //bow
                write(fd2, " ", 1); write(fd2, "0", 1); //prop
                write(fd2, "\n", 1);
                sleep(30);
                pitch_ref = ref_pitch_u;
                depth_ref = ref_depth_u;
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0.3", 3); //bladder
                write(fd2, " ", 1); write(fd2, "-0.1", 4); //ballast
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, "\n", 1);
                sleep(30);
                pitch_ref = ref_pitch_ini;
                depth_ref = ref_depth_ini;
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1); //bladder
                write(fd2, " ", 1); write(fd2, "0", 1); //ballast
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, "\n", 1);
              }

              else if (GCSString2 == "tglide") {
                //tes glide
                cout<<"glide test"<<endl;
                pitch_ref = ref_pitch_d;
                depth_ref = ref_depth_d;
                write(fd2, "$#TS", 4);
                //propeler 50m, conditioning pitch up
                write(fd2, " ", 1); write(fd2, "0", 1); //bladder
                write(fd2, " ", 1); write(fd2, "0.1", 3); //ballast
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1); //bow
                write(fd2, " ", 1); write(fd2, "0", 1); //prop
                write(fd2, "\n", 1);
                sleep(30);
                //propeler off, pitch init
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1); //bladder
                write(fd2, " ", 1); write(fd2, "0", 1); //ballast
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1); //bow
                write(fd2, " ", 1); write(fd2, "0", 1); //prop
                write(fd2, "\n", 1);
                sleep(30);
                //descent
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "-0.3", 4); //bladder
                write(fd2, " ", 1); write(fd2, "0.1", 3); //ballast
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1); //bow
                write(fd2, " ", 1); write(fd2, "0", 1); //prop
                write(fd2, "\n", 1);
                sleep(30);
                pitch_ref = ref_pitch_u;
                depth_ref = ref_depth_u;
                //ascent
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0.3", 3); //bladder
                write(fd2, " ", 1); write(fd2, "-0.1", 4); //ballast
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1); //bow
                write(fd2, " ", 1); write(fd2, "0", 1); //prop
                write(fd2, "\n", 1);
                sleep(30);
                //bladder ballast init
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1); //bladder
                write(fd2, " ", 1); write(fd2, "0", 1); //ballast
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1); //bow
                write(fd2, " ", 1); write(fd2, "0", 1); //prop
                write(fd2, "\n", 1);
                sleep(30);
                //bow 180
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1); //bladder
                write(fd2, " ", 1); write(fd2, "0", 1); //ballast
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1); //bow
                write(fd2, " ", 1); write(fd2, "0", 1); //prop
                write(fd2, "\n", 1);
                sleep(30);
                //bow init
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1); //bladder
                write(fd2, " ", 1); write(fd2, "0", 1); //ballast
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1); //bow
                write(fd2, " ", 1); write(fd2, "0", 1); //prop
                write(fd2, "\n", 1);
                sleep(30);
                //propeler 100m, pitch up
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1); //bladder
                write(fd2, " ", 1); write(fd2, "0.1", 3); //ballast
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1); //bow
                write(fd2, " ", 1); write(fd2, "0", 1); //prop
                write(fd2, "\n", 1);
                sleep(30);
                pitch_ref = ref_pitch_ini;
                depth_ref = ref_depth_ini;
                //propeler off, pitch init
                write(fd2, "$#TS", 4);
                write(fd2, " ", 1); write(fd2, "0", 1); //bladder
                write(fd2, " ", 1); write(fd2, "0", 1); //ballast
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1);
                write(fd2, " ", 1); write(fd2, "0", 1); //bow
                write(fd2, " ", 1); write(fd2, "0", 1); //prop
                write(fd2, "\n", 1);
              }

              else if (GCSString2 == "tclose") {
                //tes glide
                cout<<"glide test"<<endl;
                pitch_ref = ref_pitch_d;
                depth_ref = ref_depth_d;
                int loop = 1;
                if(loop == 1) {
                  loop = 0;
                  write(fd2, "$#TS", 4);
                  write(fd2, " ", 1); write(fd2, "0", 1);
                  write(fd2, " ", 1); write(fd2, "0", 1);
                  write(fd2, " ", 1); write(fd2, "0", 1);
                  write(fd2, " ", 1); write(fd2, "0", 1);
                  write(fd2, " ", 1); write(fd2, "0", 1);
                  write(fd2, " ", 1); write(fd2, "0", 1);
                  write(fd2, " ", 1); write(fd2, "0", 1);
                  write(fd2, " ", 1); write(fd2, "0", 1);
                  write(fd2, " ", 1); write(fd2, "0", 1);
                  write(fd2, " ", 1); write(fd2, "0", 1);
                  write(fd2, "\n", 1);
                  if(altitude >= 2) {
                    pitch_ref = ref_pitch_u;
                    depth_ref = ref_depth_u;
                    write(fd2, "$#TS", 4);
                    write(fd2, " ", 1); write(fd2, "0", 1);
                    write(fd2, " ", 1); write(fd2, "0", 1);
                    write(fd2, " ", 1); write(fd2, "0", 1);
                    write(fd2, " ", 1); write(fd2, "0", 1);
                    write(fd2, " ", 1); write(fd2, "0", 1);
                    write(fd2, " ", 1); write(fd2, "0", 1);
                    write(fd2, " ", 1); write(fd2, "0", 1);
                    write(fd2, " ", 1); write(fd2, "0", 1);
                    write(fd2, " ", 1); write(fd2, "0", 1);
                    write(fd2, " ", 1); write(fd2, "0", 1);
                    write(fd2, "\n", 1);
                    if(altitude <= 0.1) {
                      pitch_ref = ref_pitch_ini;
                      depth_ref = ref_depth_ini;
                      write(fd2, "$#TS", 4);
                      write(fd2, " ", 1); write(fd2, "0", 1);
                      write(fd2, " ", 1); write(fd2, "0", 1);
                      write(fd2, " ", 1); write(fd2, "0", 1);
                      write(fd2, " ", 1); write(fd2, "0", 1);
                      write(fd2, " ", 1); write(fd2, "0", 1);
                      write(fd2, " ", 1); write(fd2, "0", 1);
                      write(fd2, " ", 1); write(fd2, "0", 1);
                      write(fd2, " ", 1); write(fd2, "0", 1);
                      write(fd2, " ", 1); write(fd2, "0", 1);
                      write(fd2, " ", 1); write(fd2, "0", 1);
                      write(fd2, "\n", 1);
                      //loop = 1;
                    }
                  }
                }
              }

          else if (GCSString2 == "testdata") {
              second +=5;
              if (second > 59) {
                second = 1;
                minute += 1;
              }
              
              if (minute > 59) {
                minute = 0;
                hour +=1;
              }
              
              if (hour > 23) {
                hour = 1;
              }

              temperature = temperature + (rand() % 10 + 1);
              //salinity = fRand (3.49,3.51);

              if (i < 50) {
                alt = alt - 4;
                depth = depth- 4;
              }
              
              else {
                alt = alt + 4;
                depth = depth + 4;
              }
              
              //temperature, salinity, alt, depth, latitude, longitude, roll, pitch, yaw, battery, speed, strobo

              if (temperature < 0) {
                temperature = -temperature;
              }
              
              if (salinity < 0) {
                salinity = -salinity;
              }
              
              if (alt < 0) {
                alt = -alt;
              }
              
              if (depth < 0) {
                depth = -temperature;
              }
              
              if (battery < 0) {
                battery = -battery;
              }

              if (speed < 0) {
                speed = -speed;
              }
              
              if (battery < 0) {
                strobo = -strobo;
              }

              memset(datapacket, 0, strlen(datapacket));
                    
              sprintf(datapacket,"$#GB SAT AUG %d %d %02d:%02d:%02d %05.2f %05.2f %05.2f %05.2f %012.7f %012.7f %09.6f %+09.6f %09.6f 01 %04.1f 000 %04.1f %d 1234", date, year, hour, minute, second, temperature, salinity, altitude, depth, latitude, longitude, roll, pitch, yaw_terukur, battery, speed, strobo);
              cout<<datapacket<<endl;
              // datalog_task_flag = 1;
              else {
                //unknown command
                cout << "unknown command" << endl;
              }

          }
          
          else if (GCSString1 == "ats") {
              cout << "setting received" << endl;
              if (GCSString2 == "oserate") {
                  //set sampling rate
                  if (counterdataGCS > 3) {
                      set_samplingrate = atof(com_GCS3);
                      cout << "sampling rate set" << endl;
                  }
                  
                  else {
                      cout << "not enough input" << endl; 
                  }
              } 
              
              else if (GCSString2 == "bpos") {
                  //start coord
                  if (counterdataGCS > 4) {
                      set_startcoordx = atof(com_GCS3);
                      set_startcoordy = atof(com_GCS4);
                      cout << "start coord set" << endl;
                  }
                  
                  else {
                      cout << "not enough input" << endl; 
                  }
              }
              
              else if (GCSString2 == "endpos") {
                  //end coord
                  if (counterdataGCS > 4) {
                      set_endcoordx = atof(com_GCS3);
                      set_endcoordy = atof(com_GCS4);
                      cout << "end coord set" << endl;
                  }
                  
                  else {
                      cout << "not enough input" << endl; 
                  }
              }
                
              else if (GCSString2 == "do") {
                  //set depth
                  if (counterdataGCS > 3) {
                      set_maxdepth = atof(com_GCS3);
                      cout << "max depth set" << endl;
                  }
                    
                  else {
                      cout << "not enough input" << endl; 
                  }
              }
                
              else if (GCSString2 == "gad") {
                  //set glide angle
                  cout << "glide angle set" << endl;
              }
              
              else if (GCSString2 == "prop") {
                  //set propeller speed
                  cout << "propeller speed set" << endl;
              }

              

              else if (GCSString2 == "review") {
                  //see all setting value
                  cout << "Reviewing Settings" << endl;
                  cout << "gliding mode     " << set_glidemode << endl;
                  cout << "gliding count    " << set_glidecount << " times"<<endl;
                  cout << "glide distance   " << set_glidedisance << " meter"<< endl;
                  cout << "sampling rate    " << set_samplingrate << " Hz"<< endl;
                  cout << "start coordinatex " << set_startcoordx << endl;
                  cout << "start coordinatey " << set_startcoordy << endl;
                  cout << "end coordinatex   " << set_endcoordx<< endl;
                  cout << "end coordinatey   " << set_endcoordy<< endl;
                  cout << "max depth        " << set_maxdepth << " meter" <<endl;
                  cout << "gliding angle    " << set_glideangle << " Degree"<<endl;
                  cout << "propeller speed  " << set_propelspeed << " cm/s"<<endl;
              }
                
              else {
                  //unknown command
              }  
          }
        }
      }
    }
  }
  pthread_exit(NULL);
}		

void *write_read_sensor_task(void *threadid)
{
   
  while(!sensor_task_flag)
  {
    usleep(1000000);
  }
  while(1)
  {
    cout << "task 2" <<  endl;
    handlingBBB(fd);
    usleep(500000);
  }
  pthread_exit(NULL);
}

void *control_actuator_task(void *threadid)
{
   
  while(!control_actuator_task)
  {
    usleep(1000000);
  }
  while(1)
  {
    cout << "task 3" <<  endl;
    handlingFSM(fd2);
    usleep(500000);
  }
  pthread_exit(NULL);
}

void *write_datalog_task(void *threadid){

  while(!datalog_task_flag)
  {
    sleep(1);
  }
  
  while(1)
  {     
    cout << "task 4" <<  endl;
    handlingDatalog();
    usleep(500000);
    
  }
  
  pthread_exit(NULL);
}

// void PID_blst(){
//   // pitch_fb = pitch_fb * 3.1416 / 180.0;
//   pitch_fb = teta_terukur;
//   pitch_lasterr = pitch_err;
//   pitch_err = pitch_ref - pitch_fb;
//   pitch_sumerr = pitch_sumerr + pitch_err + blst_AW;

//   PID_blst_pro = pitch_err * PID_blst_KP;
//   PID_blst_int = pitch_sumerr * PID_blst_KI * sampling_time;
//   PID_blst_dev = (pitch_err - pitch_lasterr) * PID_blst_KD / sampling_time;
//   PID_blst_cal = PID_blst_pro + PID_blst_int + PID_blst_dev;
  

//   if (PID_blst_cal > sat_blst_u){
//   PID_blst_out = sat_blst_u;
//   }
//   else if (PID_blst_cal < sat_blst_d){
//   PID_blst_out = sat_blst_d;
//   }
//   else{
//   PID_blst_out = PID_blst_cal;
//   }
  
//   blst_AW = (PID_blst_out - PID_blst_cal) * PID_blst_AW;
// }


// void PID_bld(){
//   // depth_ref = depth_ref * 3.1416 / 180.0;
//   depth_lasterr = depth_err;
//   depth_err = depth_ref - depth_fb;
//   depth_sumerr = depth_sumerr + depth_err + bld_AW;

//   PID_bld_pro = depth_err * PID_bld_KP;
//   PID_bld_int = depth_sumerr * PID_bld_KI * sampling_time;
//   PID_bld_dev = (depth_err - depth_lasterr) * PID_bld_KD / sampling_time;
//   PID_bld_cal = PID_bld_pro + PID_bld_int + PID_bld_dev;
  
//   if (PID_bld_cal > sat_bld_u){
//     PID_bld_out = sat_bld_u;
//   }
  
//   else if (PID_bld_cal < sat_bld_d){
//     PID_bld_out = sat_bld_d;
//   }
  
//   else{
//     PID_bld_out = PID_bld_cal;
//   }
//   bld_AW = (PID_bld_out - PID_bld_cal) * PID_bld_AW;
// }

// void PID_rd() {
//   heading_ref = yawref;
//   heading_fb = yaw_terukur;
//   heading_ref = heading_ref * 3.1416 / 180.0;
//   heading_lasterr = heading_err;
//   heading_err = heading_ref - heading_fb;
//   heading_sumerr = heading_sumerr + heading_err + rd_AW;

//   PID_rd_pro = heading_err * PID_rd_KP;
//   PID_rd_int = heading_sumerr * PID_rd_KI * sampling_time;
//   PID_rd_dev = (heading_err - heading_lasterr) * PID_rd_KD / sampling_time;
//   PID_rd_cal = PID_rd_pro + PID_rd_int + PID_rd_dev;
  
//   if (PID_rd_cal > sat_rd_u){
//     PID_rd_out = sat_rd_u;
//   }
  
//   else if (PID_rd_cal < sat_rd_d){
//     PID_rd_out = sat_rd_d;
//   }
  
//   else{
//     PID_rd_out = PID_rd_cal;
//   }
//   rd_AW = (PID_rd_out - PID_rd_cal) * PID_rd_AW;
// }

// void handlingFSM(int _fd) {
//   while(!control_actuator_task_flag) {
//     sleep(1);
//   }

//   PID_bld();
//   PID_blst();
//   PID_rd();
//   switch (mode) {
//     case 1 :// off
    
//       // switch (half){
//       // case 1 ://cek guidance
//       // state = 1.1;
//       // if (guide != 1){
//       // half = 2;
//       // }
//       // else(){
//       // mode = 6;
//       // }
//       // break;
//       break;      
      
//     case 2 :// full control
//       switch (full){
//         case 1 :// cek guidance
//           state = 2.1;
//           if (guide == 1){
//             mode = 50;
//           }
//           else {
//             full = 2;
//           }
//           break;
  
//         case 2 ://descent
//           state = 2.2;
//           pitch_ref = ref_pitch_d;
//           depth_ref = ref_depth_d;
//           if (depth_fb <= depth_ref){
//             full = 3;
//           }
//           else if (guide == 1){
//             mode = 50;
//           }
//           else{
//             full = 2;
//           }
//           break;  
  
//         case 3 ://ascent
//           state = 2.3;
//           pitch_ref = ref_pitch_u;
//           depth_ref = ref_depth_u;
//           if (depth_fb >= depth_ref){
//             full = 1;
//           }
//           else if(guide == 1){
//             mode = 50;
//           }
//           else{
//             full = 3;
//           }
//           break;
  
//         default :
//           break;
//       }
      
//     case 3 ://on-off control
//         break;
      
//     case 4 ://surface maju
//         break;
      
//     case 5 ://serface rotate
//       int toleransi_deg = 3;
//       printf ("rotate %f\n",heading_ref);
//       rot = 1;
//       switch (rot) {
//         case 1 :
//           // tambah protokol untuk menghidupkan bow
//           rotate_ref = heading_ref;
//             if(heading_fb < heading_ref + toleransi_deg || heading_fb > heading_ref + toleransi_deg){
//               rot = 2;//off bow
//             }
//             else{
//               rot = 1;//onbow
//             }
//           break;
   
//         case 2 :
//           // tambah protokol untuk mematikan bow      
//           break;
//         default :
//           break;
//       }

//     case 11://blst test
//       printf ("pitch_TEST \n");
//       blst = 1;
//       switch (blst) {
//         case 1 :
//           pitch_ref = ref_pitch_d;
//           printf ("pitch_DOWN \n");
//             if(pitch_fb <= ref_pitch_d){
//               blst = 2;
//             }
//             else{
//               blst = 1;
//             }
//           break;
    
//         case 2 :
//           pitch_ref = ref_pitch_u;
//           printf ("pitch_UP \n");
//             if(pitch_fb >= ref_pitch_u){
//               mode = 50;
//             }
//             else{
//               blst = 2;
//             }
//           break;
    
//         default :
//           break;
//       }

//     case 12: //bld test
//       printf ("heave_TEST \n");
//       bld = 1;
//         switch(bld) {
//           case 1 :
//             depth_ref = ref_depth_d;
//             printf ("heave_DOWN \n");
//               if(altitude <= 2 || depth_fb <= ref_depth_d){
//                 bld = 2;
//               }
//               else{
//                 bld = 1;  
//               }
//             break;
    
//           case 2 :
//             depth_ref = surface;
//             printf ("heave_UP");
//               if(altitude >= ref_depth_ini || depth_fb >= ref_depth_ini){
//                 mode = 50;
//               }
//               else{
//                 bld = 2;
//               }
//             break;
    
//           default :
//             break;
//         }
    
//     case 50 ://mission complete
//       pitch_ref = ref_pitch_ini;
//       depth_ref = ref_depth_ini;
//       stop = 1;
//       // mode = 1;
//       break;
//   } 
// }

void handlingBBB(int _fd) {
  char * comBBin;
  int h;
  
  while(!sensor_task_flag) {
    sleep(1);
  }

  fd_set readBB;
  FD_ZERO(&readBB);
  FD_SET(_fd, &readBB);
  // struct timeval tv = {5, 0};
  select(_fd+1, &readBB, NULL, NULL, &tv);

  if (FD_ISSET(_fd, &readBB)) {
    usleep(300000);
    int r = read(_fd, BBinbound, 300);
    if (r == -1) {
      cout << strerror(errno) << endl;
    }
    comBBin = strtok (BBinbound, " ");

    //$#NG teta_terukur(IMU) z_terukur(Nav) yawref(Guidance) yaw_terukur(IMU) altitude(DVL) roll(IMU) salinity(CT) temperature(CT) speed(DVL) latitude(Nav) longitude(Nav)
    while(comBBin != NULL) {
      switch(BBreceive) {
        case 0 :
          strncpy(com_BBheader, comBBin, sizeof com_BBheader);
          printf("Header%s\n", com_BBheader);
          BBreceive++;
          break;

        case 1 :
          strncpy(com_BBin1, comBBin, sizeof com_BBin1);
          teta_terukur = atof (com_BBin1);
          printf("Teta terukur%s\n", teta_terukur);
          BBreceive++;
          break;

        case 2 :
          strncpy(com_BBin2, comBBin, sizeof com_BBin2);
          z_terukur = atof (com_BBin2);
          printf("Z terukur%s\n", z_terukur);
          BBreceive++;
          break;

        case 3 :
          strncpy(com_BBin3, comBBin, sizeof com_BBin3);
          yawref = atof (com_BBin3);
          printf("Yaw ref%s\n", yawref);
          BBreceive++;
          break;

        case 4 :
          strncpy(com_BBin4, comBBin, sizeof com_BBin4);
          yaw_terukur = atof (com_BBin4);
          printf("Yaw terukur%s\n", yaw_terukur);
          BBreceive++;
          break;

        case 5 :
          strncpy(com_BBin5, comBBin, sizeof com_BBin5);
          altitude = atof (com_BBin5);
          printf("Altitude%s\n", altitude);
          BBreceive++;
          break;

        case 6 :
          strncpy(com_BBin6, comBBin, sizeof com_BBin6);
          roll = atof (com_BBin6);
          printf("Roll%s\n", roll);
          BBreceive++;
          break;

        case 7 :
          strncpy(com_BBin7, comBBin, sizeof com_BBin7);
          salinity = atof (com_BBin7);
          printf("Salinity%s\n", salinity);
          BBreceive++;
          break;

        case 8 :
          strncpy(com_BBin8, comBBin, sizeof com_BBin8);
          temperature = atof (com_BBin8);
          printf("Temp%s\n", temperature);
          BBreceive++;
          break;

        case 9 :
          strncpy(com_BBin9, comBBin, sizeof com_BBin9);
          speed = atof (com_BBin9);
          printf("Speed%s\n", speed);
          BBreceive++;
          break;

        case 10 :
          strncpy(com_BBin10, comBBin, sizeof com_BBin10);
          latitude = atof (com_BBin10);
          printf("Lat%s\n", latitude);
          BBreceive++;
          break;

        case 11 :
          strncpy(com_BBin11, comBBin, sizeof com_BBin11);
          longitude = atof (com_BBin11);
          printf("Long%s\n", longitude);
          BBreceive++;
          break;
        
        default :
          break;
      }
      comBBin = strtok(NULL, " ");
    }
  }
}

void handlingDatalog() {
  char bn[140];
  int data_entry=0;
  int data_limit=1000;
  while(!datalog_task_flag) {
    sleep(1);
  }
  
  FILE * log_data;
  log_data = fopen("datalog_system.csv","a");
  if(data_entry<=data_limit) {
    if(log_data!=NULL) {
      fprintf(log_data,"$#GB SAT AUG %d,%d,%02d:%02d:%02d,%05.2f,%05.2f,%05.2f,%05.2f,%012.7f,%012.7f,%09.6f,%+09.6f,%09.6f,01,%04.1f,000,%04.1f,%d,1234", date, year, hour, minute, second, temperature, salinity, altitude, z_terukur, latitude, longitude, roll, pitch, yaw_terukur, battery, speed, strobo);
      }
    data_entry++;
    fclose(log_data); 
  } 

