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
void *write_datalog_task(void *threadid);
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


        rc = pthread_create(&datalog_thread, NULL, write_datalog_task, NULL);
        if(rc){
                cout << "Error:unable to create datalog thread, " << rc << endl;
         }

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
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "1", 1);
                        sleep(15);
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3);
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        sleep(15);
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3);
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "1", 1);
                        sleep(15);
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                else if(mystr == "#GB> atc bld")
                {
                        cout << "bladder test" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.1", 3);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "1", 1);
                        sleep(15);
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.4", 3);
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        sleep(15);
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3);
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "1", 1);
                        sleep(15);
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3);
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                else if(mystr == "#GB> atc surface")
                {
                        cout << "surface" << endl;
                        //propeller
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3); //bladder
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5); //ballast
                        write(fd, " ", 1);
                        write(fd, "0", 1); //rudder
                        write(fd, " ", 1);
                        write(fd, "1600", 4); //propeller
                        write(fd, " ", 1);
                        write(fd, "0", 1); //bowthruster
                        write(fd, " ", 1);
                        write(fd, "1", 1); //strobo
                        sleep(30);
                        //bowthruster
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3);
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "1800", 4); //bow CW
                        write(fd, " ", 1);
                        write(fd, "1", 1);
                        sleep(10);
                        //propeller
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3);
                        write(fd, " ", 1);
                        write(fd, "-0.15", 4);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "1600", 4);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "1", 1);
                        sleep(30);
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3);
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
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

                else if(mystr == "#GB> atc test net")
                {
                        cout << "test nett" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3); //bladder
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5); //ballast
                        write(fd, " ", 1);
                        write(fd, "0", 1); //rudder
                        write(fd, " ", 1);
                        write(fd, "0", 1); //propeler
                        write(fd, " ", 1);
                        write(fd, "0", 1); //bowthruster
                        write(fd, " ", 1);
                        write(fd, "1", 1); //strobo
                }

                else if(mystr == "#GB> atc test pbld")
                {
                        cout << "test bladder out" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.4", 3); //bladder
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5); //ballast
                        write(fd, " ", 1);
                        write(fd, "0", 1); //rudder
                        write(fd, " ", 1);
                        write(fd, "0", 1); //propeler
                        write(fd, " ", 1);
                        write(fd, "0", 1); //bowthruster
                        write(fd, " ", 1);
                        write(fd, "1", 1); //strobo
                }

                else if(mystr == "#GB> atc test nbld")
                {
                        cout << "test bladder in" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.1", 3); //bladder
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5); //ballast
                        write(fd, " ", 1);
                        write(fd, "0", 1); //rudder
                        write(fd, " ", 1);
                        write(fd, "0", 1); //propeler
                        write(fd, " ", 1);
                        write(fd, "0", 1); //bowthruster
                        write(fd, " ", 1);
                        write(fd, "1", 1); //strobo
                }

                else if(mystr == "#GB> atc test pblst")
                {
                        cout << "test ballast front" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3); //bladder
                        write(fd, " ", 1);
                        write(fd, "0.1", 3); //ballast
                        write(fd, " ", 1);
                        write(fd, "0", 1); //rudder
                        write(fd, " ", 1);
                        write(fd, "0", 1); //propeler
                        write(fd, " ", 1);
                        write(fd, "0", 1); //bowthruster
                        write(fd, " ", 1);
                        write(fd, "1", 1); //strobo
                }

                else if(mystr == "#GB> atc test nblst")
                {
                        cout << "test ballast back" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3); //bladder
                        write(fd, " ", 1);
                        write(fd, "-0.1", 4); //ballast
                        write(fd, " ", 1);
                        write(fd, "0", 1); //rudder
                        write(fd, " ", 1);
                        write(fd, "0", 1); //propeler
                        write(fd, " ", 1);
                        write(fd, "0", 1); //bowthruster
                        write(fd, " ", 1);
                        write(fd, "1", 1); //strobo
                }

                else if(mystr == "#GB> atc test prop")
                {
                        cout << "test propeller" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3); //bladder
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5); //ballast
                        write(fd, " ", 1);
                        write(fd, "0", 1); //rudder
                        write(fd, " ", 1);
                        write(fd, "1600", 4); //propeler
                        write(fd, " ", 1);
                        write(fd, "0", 1); //bowthruster
                        write(fd, " ", 1);
                        write(fd, "1", 1); //strobo
                        sleep(10);
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3); //bladder
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5); //ballast
                        write(fd, " ", 1);
                        write(fd, "0", 1); //rudder
                        write(fd, " ", 1);
                        write(fd, "0", 1); //propeler
                        write(fd, " ", 1);
                        write(fd, "0", 1); //bowthruster
                        write(fd, " ", 1);
                        write(fd, "0", 1); //strobo
                }

                else if(mystr == "#GB> atc test bow")
                {
                        cout << "test bowthruster" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3); //bladder
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5); //ballast
                        write(fd, " ", 1);
                        write(fd, "0", 1); //rudder
                        write(fd, " ", 1);
                        write(fd, "0", 1); //propeler
                        write(fd, " ", 1);
                        write(fd, "1800", 4); //bowthruster
                        write(fd, " ", 1);
                        write(fd, "1", 1); //strobo
                        sleep(8);
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3); //bladder
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5); //ballast
                        write(fd, " ", 1);
                        write(fd, "0", 1); //rudder
                        write(fd, " ", 1);
                        write(fd, "0", 1); //propeler
                        write(fd, " ", 1);
                        write(fd, "0", 1); //bowthruster
                        write(fd, " ", 1);
                        write(fd, "0", 1); //strobo
                }

                else if(mystr == "#GB> atc senddata")
				{
					cout << "tes send data" << endl;
					ifstream readdatalog("datalog_system.txt");
					printf("Hello\n");
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
				}

                else if(mystr == "#GB> atc test strob")
                {
                        cout << "test strobo on" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3); //bladder
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5); //ballast
                        write(fd, " ", 1);
                        write(fd, "0", 1); //rudder
                        write(fd, " ", 1);
                        write(fd, "0", 1); //propeler
                        write(fd, " ", 1);
                        write(fd, "0", 1); //bowthruster
                        write(fd, " ", 1);
                        write(fd, "1", 1); //strobo
                }

                else if(mystr == "#GB> atc test off")
                {
                        cout << "test off" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3);
                        write(fd, " ", 1);
                        write(fd, "-0.15", 5);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
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
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.2", 3);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                }
                
                else if(mystr == "#GB> atc bld300")
                {
                        cout << "bld 0.3" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.3", 3);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                else if(mystr == "#GB> atc bld400")
                {
                        cout << "bld 0.4" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0.4", 3);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                else if(mystr == "#GB> ats prop 2000")
                {
                        cout << "setting propeller 2000" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "2000", 4);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                else if(mystr == "#GB> ats prop 0")
                {
                        cout << "setting propeller 0" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                }

                else if(mystr == "#GB> ats prop 700")
                {
                        cout << "setting propeller 700" << endl;
                        write(fd, "$#TS", 4);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
                        write(fd, "700", 3);
                        write(fd, " ", 1);
                        write(fd, "0", 1);
                        write(fd, " ", 1);
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

void *read_sensor_task(void *threadid)
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
                                       
void *read_command_task(void *threadid)
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

void *write_datalog_task(void *threadid)
{
        while(!datalog_task_flag)
        {
                sleep(1);
        }

        while(1)
        {
                //cout << "task 3" <<  endl;
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
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        char * buffer = new char[240];
        //int _fd = open_port();
        FD_ZERO(&readfs);
        FD_SET(_fd, &readfs);
        select(_fd+1, &readfs, NULL, NULL, &tv /* no timeout */);
        if (FD_ISSET(_fd, &readfs))
        {
                int r = read(_fd, buffer, 240);
                if(r == -1){
                        cout << strerror(errno) << endl;
                }
                //write(_fd,"bbb data!\r\n",11);

                //write(_fd,buffer,r);
                buffer[r]='\0';
                cout << buffer << endl;
                string my_string(buffer,buffer+r);
                if (my_string[0] == '$')
                {
                        if (my_string[1] == '#')
                        {
                                if (my_string[2] == 'N')
                                {
                                        if (my_string[3] == 'G')
                                        {
                                                //valid = 1;
                                                //write(fd,"$#NG dispu dispv dispw dispx dispy dispz dispp dispq dispr dispphi dispteta disppsi guidestat psiref psiest\r\n",109);
                                                cout << "data bisa terparsing" << endl;
                                        }
                                }
                        }
                }
        }
        else
        {
                //write(_fd,"no data from beaglebone!#\r\n",27);
        }
        //close(_fd);
        //sleep(1);
}

