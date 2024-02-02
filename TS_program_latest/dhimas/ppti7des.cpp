#include <cstring>
#include <sstream>
#include <fstream>
#include <iostream>
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
using namespace std;

// Thread
void *read_sensor_task(void *threadid);
void *read_command_task(void *threadid);
// void *write_datalog_task(void *threadid);
int set_interface_attribs();
int set_interface_attribs2();
void set_mincount(int fd, int mcount);
int open_port();
void handlingSimulink(int _fd);
void handlingBBB(int _fd);

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
string datalog_yaw_terukur;
string datalog_motion;
string datalog_battery;
string datalog_leakage;
string datalog_speed;
string datalog_strobo;
string datalog_csum;

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

float teta_terukur, z_terukur, yawref, yaw_terukur, posn, pose, alt, sal, temp, spd, lat, lon, roll;
int data_entry;
int data_limit = 1000;

//TASK FLAG
int command_task_flag = 0;
int sensor_task_flag = 0;
int datalog_task_flag = 0;

//UART STATUS
int uart_status_imu =0;
int uart_status_altisounder =0;
int uart_status_oseanometer =0;
int uart_status_command =0;
int uart_status_pwmgenerator =0;

//DATALOG
int datalog_filenumber;

string string_teta;
stringstream stream_teta;
char char_teta [100];

//Variabel Port Komunikasi
int fd;
int fd2;
char *portname = "/dev/ttyxuart1";
int wlen;
char *portname2 = "/dev/ttyxuart0";
int wlen2;
char input[100];
int sentinel;

string myCppString0;
string myCppString1;

int main(void){

        pthread_t pwm_generator;
        pthread_t command_command;
        pthread_t datalog_thread;

        int rc;
        int i = 7;


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

        rc = pthread_create(&pwm_generator, NULL, read_sensor_task, (void *)i);
        if(rc){
                cout << "Error:unable to create sensor thread, " << rc << endl;
                exit(-1);
        }

        rc = pthread_create(&command_command, NULL, read_command_task, NULL);
        if(rc){
                cout << "Error:unable to create command thread, " << rc << endl;
                exit(-1);
        }


        // rc = pthread_create(&datalog_thread, NULL, write_datalog_task, NULL);
        // if(rc){
        //         cout << "Error:unable to create datalog thread, " << rc << endl;
        //  }

        double x;

        /*************************** INISIALISASI ******************************
*********/

        //task activation
        command_task_flag = 1;
        sensor_task_flag = 1;
        datalog_task_flag = 1;

        //conditioning awal
        sleep(1);
        char * pch;
        int n;
//$#TS 0 0 1600 1600
//
        while(1){
                //cout << "Main Loop Working" <<  endl;
                // char * strcoba
                int mode;
                switch(mode)
                {
                    case 1 : //DESCENT
                        write(fd, "$#TS", 4); write(fd, " ", 1); 
                        write(fd, "0.1", 3);  write(fd, " ", 1); //bladder
                        write(fd, "-0.05", 5); write(fd, " ", 1); //ballast
                        write(fd, "0", 1);    write(fd, " ", 1); //rudder
                        write(fd, "0", 1);    write(fd, " ", 1); //propeler
                        write(fd, "0", 1);    write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1);
                        break;

                    case 2 : //ASCENT
                        write(fd, "$#TS", 4); write(fd, " ", 1); 
                        write(fd, "0.3", 3);  write(fd, " ", 1); //bladder
                        write(fd, "-0.15", 5); write(fd, " ", 1); //ballast
                        write(fd, "0", 1);    write(fd, " ", 1); //rudder
                        write(fd, "0", 1);    write(fd, " ", 1); //propeler
                        write(fd, "0", 1);    write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1);
                        break;

                    case 3 : //BOWTHRUSTER
                        write(fd, "$#TS", 4); write(fd, " ", 1); 
                        write(fd, "0", 1);  write(fd, " ", 1); //bladder
                        write(fd, "0", 1); write(fd, " ", 1); //ballast
                        write(fd, "0", 1); write(fd, " ", 1); //rudder
                        write(fd, "0", 1); write(fd, " ", 1); //propeler
                        write(fd, "1800", 4); write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1);
                        break;

                    case 4 : //PROPELLER
                        write(fd, "$#TS", 4); write(fd, " ", 1); 
                        write(fd, "0", 1);  write(fd, " ", 1); //bladder
                        write(fd, "0", 1); write(fd, " ", 1); //ballast
                        write(fd, "0", 1); write(fd, " ", 1); //rudder
                        write(fd, "1600", 4); write(fd, " ", 1); //propeler
                        write(fd, "0", 1); write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1);
                        break;

                    case 5 : //NETT
                        write(fd, "$#TS", 4); write(fd, " ", 1); 
                        write(fd, "0", 1);  write(fd, " ", 1); //bladder
                        write(fd, "0", 1); write(fd, " ", 1); //ballast
                        write(fd, "0", 1); write(fd, " ", 1); //rudder
                        write(fd, "0", 1); write(fd, " ", 1); //propeler
                        write(fd, "0", 1); write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1);
                        break;
                        
                    case 6 : {
                        ifstream readdatalog("datalog_system.txt");
                        if(!readdatalog.is_open()) {
                        cout << "Error, file is open" << endl;
                        }

                        while(readdatalog.good()) 
                        {
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
                            getline(readdatalog,datalog_z_terukur,' ');
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
                            getline(readdatalog,datalog_csum,'\n');

                            cout<<"$#GB "<<datalog_day<<" "<<datalog_mon<<" "<<datalog_date<<" "<<datalog_year<<" "<<datalog_hour<<":"<<datalog_minute<<":"<<datalog_second<<" "<<datalog_temperature<<" "<<datalog_salinity<<" "<<datalog_altitude<<" "<<datalog_z_terukur<<" "<<datalog_latitude<<" "<<datalog_longitude<<" "<<datalog_roll<<" "<<datalog_pitch<<" "<<datalog_yaw_terukur<<" "<<datalog_motion<<" "<<datalog_battery<<" "<<datalog_leakage<<" "<<datalog_speed<<" "<<datalog_strobo<<" "<<datalog_csum<<endl;
                            cout<<"data day"<<datalog_day<<endl;

                        }
                        readdatalog.close();
                        break;
                    }

                    default :
                        break;

                }
                string mystr;
                cout << "Waiting new command !";
                getline (cin, mystr);
                // strcoba = strtok(mystr, " ");
                //myCppString0 = input;
                string_teta = mystr;
                if (mystr == "#GB> atc rto\n")
                {
                        cout << "tes return operation" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, "\n", 1);
                        //do sequence
                }

                else if(mystr == "#GB> atc rtb\n")
                {
                        cout << "return to base" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, "\n", 1);
                }

                else if(mystr == "#GB> atc kss\n")
                {
                        cout << "stay surface" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, "\n", 1);
                }

                else if(mystr == "#GB> atc blst")
                {
                        cout << "ballast test" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "1", 1);
                        sleep(15);
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1);
                        write(fd, "-0.15", 5); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);
                        sleep(15);
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1);
                        write(fd, "-0.15", 5); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "1", 1);
                        sleep(15);
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                else if(mystr == "#GB> atc bld")
                {
                        cout << "bladder test" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.1", 3);  write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "1", 1);
                        sleep(15);
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.4", 3);  write(fd, " ", 1);
                        write(fd, "-0.15", 5); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);
                        sleep(15);
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1);
                        write(fd, "-0.15", 5); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "1", 1);
                        sleep(15);
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1);
                        write(fd, "-0.15", 5); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                else if(mystr == "#GB> atc surface")
                {
                        cout << "surface" << endl;
                        //propeller
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1); //bladder
                        write(fd, "-0.15", 5); write(fd, " ", 1); //ballast
                        write(fd, "0", 1);    write(fd, " ", 1); //rudder
                        write(fd, "1600", 4); write(fd, " ", 1); //propeller
                        write(fd, "0", 1);    write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1); //strobo
                        sleep(30);
                        //bowthruster
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1);
                        write(fd, "-0.15", 5); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "1800", 4); write(fd, " ", 1); //bow CW
                        write(fd, "1", 1);
                        sleep(10);
                        //propeller
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1);
                        write(fd, "-0.15", 4); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "1600", 4); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "1", 1);
                        sleep(30);
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1);
                        write(fd, "-0.15", 5); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);
                        cout << "mission complete" << endl;
                }

                else if(mystr == "#GB> atc glide")
                {
                        cout << "gliding" << endl;
                        //propeller
                        // write(fd, "$#TS", 4);
                        // write(fd, " ", 1); write(fd, "0.3", 3); //bladder
                        // write(fd, " ", 1); write(fd, "-0.15", 5); //ballast
                        // write(fd, " ", 1); write(fd, "0", 1); //rudder
                        // write(fd, " ", 1); write(fd, "1600", 4); //propeller
                        // write(fd, " ", 1); write(fd, "0", 1); //bowthruster
                        // write(fd, " ", 1); write(fd, "1", 1); //strobo
                        // sleep(20);
                        // cout << " glide" << endl;
                        //descent
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1); write(fd, "0.1", 3);
                        write(fd, " ", 1); write(fd, "-0.05", 5);
                        write(fd, " ", 1); write(fd, "0", 1);
                        write(fd, " ", 1); write(fd, "0", 1);
                        write(fd, " ", 1); write(fd, "0", 1);
                        write(fd, " ", 1); write(fd, "1", 1);
                        sleep(20);
                        //ascent
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1); write(fd, "0.3", 3);
                        write(fd, " ", 1); write(fd, "-0.15", 5);
                        write(fd, " ", 1); write(fd, "0", 1);
                        write(fd, " ", 1); write(fd, "0", 1);
                        write(fd, " ", 1); write(fd, "0", 1);
                        write(fd, " ", 1); write(fd, "1", 1);
                        sleep(90);
                        cout << "propeller" << endl;
                        //propeller
                        // write(fd, "$#TS", 4);
                        // write(fd, " ", 1); write(fd, "0.3", 3);
                        // write(fd, " ", 1); write(fd, "-0.15", 5);
                        // write(fd, " ", 1); write(fd, "0", 1);
                        // write(fd, " ", 1); write(fd, "1600", 4);
                        // write(fd, " ", 1); write(fd, "0", 1);
                        // write(fd, " ", 1); write(fd, "1", 1);
                        // sleep(25);
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1); write(fd, "0.3", 3);
                        write(fd, " ", 1); write(fd, "-0.15", 5);
                        write(fd, " ", 1); write(fd, "0", 1);
                        write(fd, " ", 1); write(fd, "0", 1);
                        write(fd, " ", 1); write(fd, "0", 1);
                        write(fd, " ", 1); write(fd, "1", 1);
                        sleep(5);
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1); write(fd, "0.3", 3);
                        write(fd, " ", 1); write(fd, "-0.15", 5);
                        write(fd, " ", 1); write(fd, "0", 1);
                        write(fd, " ", 1); write(fd, "0", 1);
                        write(fd, " ", 1); write(fd, "0", 1);
                        write(fd, " ", 1); write(fd, "0", 1);
                        cout << "mission complete" << endl;
                }

                else if(mystr == "#GB> atc demo")
                {
                        cout << "start demo" << endl;
                        //descent
                        if(alt > 2.0)
                        {
                            mode = 1;
                        }
                        //ascent
                        else if(alt <= 2.0)
                        {
                            mode = 2;
                        }

                        //bowthruster
                        cout << "bowthruster" << endl;
                        mode = 3;
                        sleep(25);

                        //propeller
                        cout << "propeller" << endl;
                        mode = 4;
                        sleep(25);
                        
                        //nett
                        mode = 5;
                        sleep(25);
                        
                        mode = 6;
                        cout << "mission complete" << endl;
                        mode = 0;
                }

                else if(mystr == "#GB> atc test net")
                {
                        cout << "test nett" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1); //bladder
                        write(fd, "-0.15", 5); write(fd, " ", 1); //ballast
                        write(fd, "0", 1);    write(fd, " ", 1); //rudder
                        write(fd, "0", 1);    write(fd, " ", 1); //propeler
                        write(fd, "0", 1);    write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1); //strobo
                }

                else if(mystr == "#GB> atc test pbld")
                {
                        cout << "test bladder out" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.4", 3);  write(fd, " ", 1); //bladder
                        write(fd, "-0.15", 5); write(fd, " ", 1); //ballast
                        write(fd, "0", 1);    write(fd, " ", 1); //rudder
                        write(fd, "0", 1);    write(fd, " ", 1); //propeler
                        write(fd, "0", 1);    write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1); //strobo
                }

                else if(mystr == "#GB> atc test nbld")
                {
                        cout << "test bladder in" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.1", 3);  write(fd, " ", 1); //bladder
                        write(fd, "-0.15", 5); write(fd, " ", 1); //ballast
                        write(fd, "0", 1);    write(fd, " ", 1); //rudder
                        write(fd, "0", 1);    write(fd, " ", 1); //propeler
                        write(fd, "0", 1);    write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1); //strobo
                }

                else if(mystr == "#GB> atc test pblst")
                {
                        cout << "test ballast front" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1); //bladder
                        write(fd, "0.1", 3);  write(fd, " ", 1); //ballast
                        write(fd, "0", 1);    write(fd, " ", 1); //rudder
                        write(fd, "0", 1);    write(fd, " ", 1); //propeler
                        write(fd, "0", 1);    write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1); //strobo
                }

                else if(mystr == "#GB> atc test nblst")
                {
                        cout << "test ballast back" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1); //bladder
                        write(fd, "-0.1", 4); write(fd, " ", 1); //ballast
                        write(fd, "0", 1);    write(fd, " ", 1); //rudder
                        write(fd, "0", 1);    write(fd, " ", 1); //propeler
                        write(fd, "0", 1);    write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1); //strobo
                }

                else if(mystr == "#GB> atc test prop")
                {
                        cout << "test propeller" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1); //bladder
                        write(fd, "-0.15", 5); write(fd, " ", 1); //ballast
                        write(fd, "0", 1);    write(fd, " ", 1); //rudder
                        write(fd, "1600", 4); write(fd, " ", 1); //propeler
                        write(fd, "0", 1);    write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1); //strobo
                        sleep(10);
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1); //bladder
                        write(fd, "-0.15", 5); write(fd, " ", 1); //ballast
                        write(fd, "0", 1);    write(fd, " ", 1); //rudder
                        write(fd, "0", 1);    write(fd, " ", 1); //propeler
                        write(fd, "0", 1);    write(fd, " ", 1); //bowthruster
                        write(fd, "0", 1); //strobo
                }

                else if(mystr == "#GB> atc test bow")
                {
                        cout << "test bowthruster" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1); //bladder
                        write(fd, "-0.15", 5); write(fd, " ", 1); //ballast
                        write(fd, "0", 1);    write(fd, " ", 1); //rudder
                        write(fd, "0", 1);    write(fd, " ", 1); //propeler
                        write(fd, "1800", 4); write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1); //strobo
                        sleep(8);
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1); //bladder
                        write(fd, "-0.15", 5); write(fd, " ", 1);  //ballast
                        write(fd, "0", 1);    write(fd, " ", 1); //rudder
                        write(fd, "0", 1);    write(fd, " ", 1); //propeler
                        write(fd, "0", 1);    write(fd, " ", 1); //bowthruster
                        write(fd, "0", 1); //strobo
                }

                else if(mystr == "#GB> atc senddata")
				{
					cout << "tes send data" << endl;
					mode = 6;
				}

                else if(mystr == "#GB> atc deldata")
                {
                    cout << "delete all datalog" << endl;
                    ofstream deletedatalog("datalog_system.txt");
                    if(deletedatalog.is_open()) {
                        deletedatalog << "" << endl;
                        cout << "delete success" << endl;
                        deletedatalog.close();
                    }
                    data_entry = 0;
                    cout << "data count : " << data_entry << endl;
                }

                else if(mystr == "#GB> atc test strob")
                {
                        cout << "test strobo on" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1); //bladder
                        write(fd, "-0.15", 5); write(fd, " ", 1); //ballast
                        write(fd, "0", 1);    write(fd, " ", 1); //rudder
                        write(fd, "0", 1);    write(fd, " ", 1); //propeler
                        write(fd, "0", 1);    write(fd, " ", 1); //bowthruster
                        write(fd, "1", 1); //strobo
                }

                else if(mystr == "#GB> atc test off")
                {
                        cout << "test off" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1);
                        write(fd, "-0.15", 5); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                else if(mystr == "#GB> atc BB on")
                {
                    cout << "log BB" << endl;
                    write(fd2, "$#O1", 4);
                }

                else if(mystr == "#GB> atc BB off")
                {
                    cout << "log BB off" << endl;
                    write(fd2, "$#O0", 4);
                }

                else if(mystr == "#GB> atc bld200")
                {
                        cout << "bld 0.2" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.2", 3);  write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);
                }
                
                else if(mystr == "#GB> atc bld300")
                {
                        cout << "bld 0.3" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.3", 3);  write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                else if(mystr == "#GB> atc bld400")
                {
                        cout << "bld 0.4" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0.4", 3);  write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                else if(mystr == "#GB> ats prop 2000")
                {
                        cout << "setting propeller 2000" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "2000", 4); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                else if(mystr == "#GB> ats prop 0")
                {
                        cout << "setting propeller 0" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                else if(mystr == "#GB> ats prop 700")
                {
                        cout << "setting propeller 700" << endl;
                        write(fd, "$#TS", 4); write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "700", 3);  write(fd, " ", 1);
                        write(fd, "0", 1);    write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                stream_teta.str("");
                stream_teta.clear();
                strcpy( char_teta, mystr.c_str() );


                usleep(200000);
                //write(fd2,"bbb audi!\r\n",11);
                // write(fd2,char_teta,sizeof(char_teta));
                // write(fd2,"\n",1);
                mystr = "";
        }

        cout << "END_PROGRAM " << endl;

        usleep(10000);
        pthread_exit(NULL);
        return 0;

}

void *read_command_task(void *threadid)
{

        while(!sensor_task_flag)
        {
                usleep(1000000);
        }
        while(1)
        {
                //cout << "task 1" <<  endl;
                handlingSimulink(fd);
                usleep(500000);
        }
        pthread_exit(NULL);
}
                                       
void *read_sensor_task(void *threadid)
{
        while(!command_task_flag)
        {
                sleep(1);
        }
        sleep(1);

        while(1)
        {
                //cout << "task 2" <<  endl;
                handlingBBB(fd2);
                usleep(500000);

        }
         pthread_exit(NULL);
}

int set_interface_attribs()
{
    struct termios oldtio,newtio;
        int serial_fd;
        if ((serial_fd = open("/dev/ttyxuart1", O_RDWR | O_EXCL | O_NDELAY)) ==
-1) {
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
        if ((serial_fd = open("/dev/ttyxuart0", O_RDWR | O_EXCL | O_NDELAY)) ==
-1) {
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

// Melakukan read serial non bloking, checking dan timeout buat read
void handlingSimulink(int _fd)
{
        fd_set readfs;
        timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        char * buffer = new char[240];
        //int _fd = open_port();
        FD_ZERO(&readfs);
        FD_SET(_fd, &readfs);
        select(_fd+1, &readfs, NULL, NULL, &tv /* no timeout */);
        if (FD_ISSET(_fd, &readfs))
        {
                usleep(100000);
                int r = read(_fd, buffer, 240);
                if(r == -1){
                        cout << strerror(errno) << endl;
                }

                write(_fd,"sim data!\r\n",11);
                //write(_fd,buffer,r);


                string my_string(buffer,buffer+r);

                if (my_string[0] == '$')
                {
                        if (my_string[1] == '#')
                        {
                                if (my_string[2] == 'T')
                                {
                                        if (my_string[3] == 'S')
                                        {
                                                //valid = 1;
                                                write(fd2,buffer,r);
                                        }
                                }
                        }
                }
        }
        else{
                write(_fd,"no data from simulink!#\r\n",25);
        }
        //close(_fd);
        //sleep(1);
}

void handlingBBB(int _fd)
{
        fd_set readfs;
        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 500000;
        char * BBinbound = new char[900];
        int BBcount=0;
        //int _fd = open_port();
        // FD_ZERO(&readfs);
        // FD_SET(_fd, &readfs);
        // select(_fd+1, &readfs, NULL, NULL, &tv /* no timeout */);
        // if (FD_ISSET(_fd, &readfs))
        // {
        //         int r = read(_fd, BBinbound, 400);
        //         if(r == -1){
        //                 cout << strerror(errno) << endl;
        //         }
                //write(_fd,"bbb data!\r\n",11);

                //write(_fd,buffer,r);
                // BBinbound[r]='\0';
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
                // cout << comBBin << endl;
                while(comBBin != NULL)
                {
                    
                    switch(BBcount) {
                        case 0 :
                        {
                            strncpy(comBBheader, comBBin,4);
                            cout << "Masuk" << endl;
                            printf("Header : %s\n", comBBheader);
                        }
                        BBcount++;
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
                            posn = atof(comBB_5);
                            printf("Pos N : %9.6f\n", posn);
                        }
                        BBcount++;
                        cout << "BBcount : " << BBcount << endl;
                        break;

                        case 6 :
                        {
                            strcpy(comBB_6, comBBin);
                            pose = atof(comBB_6);
                            printf("Pos E : %9.6f\n", pose);
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
                        // BBcount++;
                        cout << "BBcount : " << BBcount << endl;
                        break;
                                                            
                        default :
                            break;
                    }
                    comBBin = strtok(NULL, " ");

                    if(BBcount >= 13)
                    {
                        if(data_entry <= data_limit)
                        {
                            ofstream savedatalog("datalog_system.txt", ios::out | ios::app);
                            if(savedatalog.is_open())
                            {
                                savedatalog << "$#NG " << teta_terukur << ' ' << z_terukur << ' ' << yawref << ' ' << yaw_terukur << ' ' << posn << ' ' << pose << ' ' << alt << ' ' << sal << ' ' << temp << ' ' << spd << ' ' << lat << ' ' << lon << ' ' << roll << endl;
                                cout << "save success" << endl;
                                savedatalog.close();
                            }
                            data_entry++;
                            cout << "data count : " << data_entry << endl;
                        }

                        else
                        {
                            cout << "Error, datalog full" << endl;
                            
                        }
                    }
                }
                }
            }
        }
    }
                BBcount = 0;
                delete[] BBinbound;
        // }
        // else
        // {
        //         //write(_fd,"no data from beaglebone!#\r\n",27);
        // }
        // close(_fd);
        // sleep(1);
}

