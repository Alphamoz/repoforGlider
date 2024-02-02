#include <iostream>
#include <stdio.h>
#include <string.h>

#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

int sport = 0;

int initserial(char param[])
{
	//return -1 : fail get port
	//return -2 : fail set attrib
	//return -3 :

	//def : return port
	char deb = 0;
	int _port = 0;
	struct termios SerSettings;

	if (deb) cout << "opening serial port" << endl;
//	_port = open("/dev/ttyS0",O_RDWR | O_NOCTTY);
	_port = open(param,O_RDWR | O_NOCTTY);
	if (_port == 0) return -1;

	if (deb) cout << "try to set attribs" << endl;
/*
	tcgetattr(_port,&SerSettings);
	cfsetispeed(&SerSettings,B115200);
	cfsetospeed(&SerSettings,B115200);

	SerSettings.c_cflag &= ~PARENB;

	SerSettings.c_cflag &= ~CSTOPB;

	SerSettings.c_cflag &= ~CSIZE;
	SerSettings.c_cflag |= CS8;

	SerSettings.c_cflag &= ~CRTSCTS;

	SerSettings.c_cflag |= CREAD | CLOCAL;

	SerSettings.c_cflag &= ~(IXON | IXOFF | IXANY);

	SerSettings.c_lflag = 0;
	SerSettings.c_oflag = 0;

	tcflush(_port,TCIFLUSH);
	if ((tcsetattr(sport,TCSANOW,&SerSettings))!= 0) 
		return -2; else */
		return _port;

}

void SerWrite(char _msg[],int _port){
	char* p = _msg;

	for (; *p != '\0'; ++p)
    {
         // if '\0' happens to be valid data for your app,
         // then you can (maybe) use some other value as
         // sentinel
    }
	int arraySize = p - _msg;

	write(_port,_msg,arraySize);
}

int main(){

	sport = initserial("/dev/ttyxuart1");
	cout << "init val : "<< sport << endl;
	SerWrite("hello\r\n",sport);
	SerWrite("world",sport);
	SerWrite("hehe\r\n",sport);
	close (sport);
	return 0;
}
