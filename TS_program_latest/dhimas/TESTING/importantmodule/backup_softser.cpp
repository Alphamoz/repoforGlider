#include <iostream>
#include <stdio.h>
#include <string.h>

#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

int sport = 0;

int setattribs()
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

int initserial(char param[])
{
	//return -1 : fail get port
	//return -2 : fail set attrib
	//return -3 : 
	
	//def : return port
	char deb = 1;
	int _port = 0;
	struct termios SerSettings;
	
	if (deb) cout << "opening serial port" << endl;
//	_port = open("/dev/ttyS0",O_RDWR | O_NOCTTY);
	_port = open(param,O_RDWR | O_NOCTTY);
	if (_port == 0) return -1;
	
	if (deb) cout << "try to set attribs" << endl;

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
	{
		cout << "fail set attribs" <<endl;
		return -2; 
	}
	return _port;
	cout << "success set attribs" <<endl;

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

//	sport = initserial("/dev/ttyS0");
//	sport = initserial("/dev/ttyxuart0");
	int fd2 = open("/dev/ttyxuart1", O_RDWR | O_NOCTTY | O_SYNC);
	cout << "fd2 open" << endl;
	if(fd2 < 0) {
		printf("Error opening %s: %s\n", "/dev/ttyxuart1", strerror(errno));
		return -1;
	}

setattribs();
	cout <<"writing"<<endl;
	write(fd2, "$#TS ", 5);
//	SerWrite("hello\r\n",sport);
//	SerWrite("world",sport);
//	SerWrite("hehe\r\n",sport);
	return 0;
}
