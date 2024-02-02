// Kode komunikasi untuk HILS
// 3 Thread, Thread Komunikasi dari PC Simulink (Xuart1)
// Thread Komunikasi ke BBB (Xuart0)

// protokol data dari TS, bypass langsung ke BBB
// $#TS 100 200 300 400 500 600 700 800 900 1000 1100 1200 1300 1400 1500 1600 1700 1800 1900 2000 2100

// protokol dari BBB hasil navigasi guidance, bypass ke simulink + rekam datalog
// $#NG dispu dispv dispw dispx dispy dispz dispp dispq dispr dispphi dispteta disppsi guidestat psiref psiest


// HIlS dimulai dari data dari PC simulink (Xuart1) yang diterima dan di cek headernya, jika valid dikirim ke BBB
// Sistem selanjutnya menunggu input dari BBB hasil navigasi guidance, jika sudah diterima di cek headernya dan di kirim ke simulink PC


// compiling g++ threadingv2.cpp -lpthread -o output

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

//
char BBinbound[150];
int BBreceive = 0;

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
float roll;
float salinity;
float temperature;
float speed;
float latitude;
float longitude;

int BBok = 0;

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
		exit(-1);
	}
	
	double x;
	
	/*************************** INISIALISASI ***************************************/
	
	//task activation
	command_task_flag = 1;
	sensor_task_flag = 0;
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
			write(fd, " ", 1);
			write(fd, "0", 1);
			write(fd, "\n", 1);	
		}
		
		else if(mystr == "#GB> atc blst")
		{
			cout << "ballast test" << endl;
			write(fd, "$#TS", 4);
			write(fd, " ", 1);
			write(fd, "0", 1);
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
			sleep(30);
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
			sleep(30);
			write(fd, "$#TS", 4);
			write(fd, " ", 1);
			write(fd, "0", 1);
			write(fd, " ", 1);
			write(fd, "-0.1", 4);
			write(fd, " ", 1);
			write(fd, "0", 1);
			write(fd, " ", 1);
			write(fd, "0", 1);
			write(fd, " ", 1);
			write(fd, "0", 1);
			write(fd, " ", 1);
			write(fd, "0", 1);
			sleep(30);
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
			write(fd, "0", 1);
			sleep(30);
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
			sleep(30);
			write(fd, "$#TS", 4);
			write(fd, " ", 1);
			write(fd, "-0.1", 4);
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
			sleep(30);
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

		else if(mystr == "#GB> atc glide\n")
		{
			cout << "gliding" << endl;
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

		else if(mystr == "#GB> atc test pbld")
		{
			cout << "test bladder out" << endl;
			write(fd, "$#TS", 4);
			write(fd, " ", 1);
			write(fd, "0.05", 4); //bladder
			write(fd, " ", 1);
			write(fd, "0", 1); //ballast
			write(fd, " ", 1);
			write(fd, "0", 1); //rudder
			write(fd, " ", 1);
			write(fd, "0", 1); //propeler
			write(fd, " ", 1);
			write(fd, "0", 1); //bowthruster
			write(fd, " ", 1);
			write(fd, "0", 1); //strobo
		}

		else if(mystr == "#GB> atc test nbld")
		{
			cout << "test bladder in" << endl;
			write(fd, "$#TS", 4);
			write(fd, " ", 1);
			write(fd, "-0.05", 5); //bladder
			write(fd, " ", 1);
			write(fd, "0", 1); //ballast
			write(fd, " ", 1);
			write(fd, "0", 1); //rudder
			write(fd, " ", 1);
			write(fd, "0", 1); //propeler
			write(fd, " ", 1);
			write(fd, "0", 1); //bowthruster
			write(fd, " ", 1);
			write(fd, "0", 1); //strobo
		}

		else if(mystr == "#GB> atc test pblst")
		{
			cout << "test ballast front" << endl;
			write(fd, "$#TS", 4);
			write(fd, " ", 1);
			write(fd, "0", 1); //bladder
			write(fd, " ", 1);
			write(fd, "0.1", 3); //ballast
			write(fd, " ", 1);
			write(fd, "0", 1); //rudder
			write(fd, " ", 1);
			write(fd, "0", 1); //propeler
			write(fd, " ", 1);
			write(fd, "0", 1); //bowthruster
			write(fd, " ", 1);
			write(fd, "0", 1); //strobo
		}
		
		else if(mystr == "#GB> atc test nblst")
		{
			cout << "test ballast back" << endl;
			write(fd, "$#TS", 4);
			write(fd, " ", 1);
			write(fd, "0", 1); //bladder
			write(fd, " ", 1);
			write(fd, "-0.1", 4); //ballast
			write(fd, " ", 1);
			write(fd, "0", 1); //rudder
			write(fd, " ", 1);
			write(fd, "0", 1); //propeler
			write(fd, " ", 1);
			write(fd, "0", 1); //bowthruster
			write(fd, " ", 1);
			write(fd, "0", 1); //strobo
		}

		else if(mystr == "#GB> atc test strob")
		{
			cout << "test strobo on" << endl;
			write(fd, "$#TS", 4);
			write(fd, " ", 1);
			write(fd, "0", 1); //bladder
			write(fd, " ", 1);
			write(fd, "0", 1); //ballast
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
			write(fd2, "$#O0\n", 5);
			sensor_task_flag = 0;
			BBok = 0;
		}

		else if(mystr == "#GB> atc test BB")
		{
			cout << "test BB" << endl;
			sensor_task_flag = 1;
			write(fd2, "$#O1\n", 5);
		}

		// else
		// {
		// 	write(fd, mystr, );
		// }

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
		handlingBBB(fd2);
		usleep(500000);
	}
	pthread_exit(NULL);
}

void *read_command_task(void *threadid){

	while(!command_task_flag)
	{
		sleep(1);
	}
	sleep(1);
	
	while(1)
	{			
		//cout << "task 2" <<  endl;
		// handlingBBB(fd2);
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
		//cout << "task 3" <<  endl;
		usleep(500000);
		
	}
	
	pthread_exit(NULL);
}

int set_interface_attribs()
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

int set_interface_attribs2()
{
    struct termios oldtio,newtio;
	int serial_fd;
	if ((serial_fd = open("/dev/ttyxuart0", O_RDWR | O_EXCL | O_NDELAY)) == -1) {
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
	char * comBBin;
  int h;
  
  while(!sensor_task_flag) {
    sleep(1);
  }

  fd_set readBB;
  FD_ZERO(&readBB);
  FD_SET(_fd, &readBB);
  struct timeval tv = {1, 0};
  select(_fd+1, &readBB, NULL, NULL, &tv);

  if (FD_ISSET(_fd, &readBB)) {
    usleep(300000);
    int r = read(_fd, BBinbound, 300);
    printf("BBinbound: %s\n",BBinbound);
    if (r == -1) {
      cout << strerror(errno) << endl;
    }

    string BBin(BBinbound,BBinbound+r);
		// cek header
		if (BBin[0] == '$')
		{
			if (BBin[1] == '#')
			{
				if (BBin[2] == 'N')
				{
					if (BBin[3] == 'G')
					{
						BBok = 1;
					}
				}
			}
		}
		
    comBBin = strtok (BBinbound, " ");

    //$#NG teta_terukur(IMU) z_terukur(Nav) yawref(Guidance) yaw_terukur(IMU) altitude(DVL) roll(IMU) salinity(CT) temperature(CT) speed(DVL) latitude(Nav) longitude(Nav)
    if(BBok = 1) {
    	while(comBBin != NULL) {
    		// printf("Hello BB\n");
    		
    		// printf("comBBin:\n",comBBin);
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
		          altitude = atof (com_BBin2);
		          printf("altitude%s\n", altitude);
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
		          z_terukur = atof (com_BBin5);
		          printf("Z_terukur%s\n", z_terukur);
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
      		
      }
     	
      	comBBin = strtok(NULL, " ");
    }
    	BBreceive = 0;
		BBok = 0;
  }
}

