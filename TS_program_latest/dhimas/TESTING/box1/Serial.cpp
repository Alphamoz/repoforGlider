#include <iostream>
#include <stdio.h>
#include <string.h>

#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

int sport = 0;

void Serial(){
	cout << "opening" << endl;
//	sport = open("/dev/ttyxuart4",O_RDWR | O_NOCTTY);
	sport = open("/dev/ttyxuart0",O_RDWR | O_NOCTTY);
	cout << "serial port status : "<< sport << endl;
}

void set_interface()
{
	cout << "try to set attribs" << endl;
	struct termios SerSettings;

	tcgetattr(sport,&SerSettings);
	cfsetospeed(&SerSettings,B9600);

	SerSettings.c_cflag &= ~PARENB;

	SerSettings.c_cflag &= ~CSTOPB;

	SerSettings.c_cflag &= ~CSIZE;
	SerSettings.c_cflag |= CS8;

	SerSettings.c_cflag &= ~CRTSCTS;

	SerSettings.c_cflag |= CREAD | CLOCAL;

	SerSettings.c_cflag &= ~(IXON | IXOFF | IXANY);

	SerSettings.c_lflag = 0;
	SerSettings.c_oflag = 0;

	tcflush(sport,TCIFLUSH);
	if ((tcsetattr(sport,TCSANOW,&SerSettings))!= 0)
		cout << "attribs setting error" << endl; else
		cout << "attribs set"<<endl;


}
/*
int set_interface_attribs()
{
    struct termios oldtio,newtio;
    int serial_fd = sport;

    if (sport == -1)
    {
        cout << "unable to open" << endl;
        return -1;
    }

    if (tcgetattr(serial_fd, &oldtio) == -1) {
        cout << "tcgetattr failed" << endl;
        return -1;
    }

    cfmakeraw(&newtio); // Clean all settings
    newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B115200; // 8 databits
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
    cout << "set interface attribs clear" << endl;
    return serial_fd;
}
*/
void SerWrite(){
	char write_buf[] = "hello";
	int bytes_size = 0;
	bytes_size = write(sport,"+++",sizeof("+++"));
	cout << "bytes written : "<< bytes_size << endl;
}


int main(){
	cout << "hello"<<endl;
	Serial();
//	set_interface_attribs();
	set_interface();
	while (1){
		SerWrite();
		cout << "loop";
		printf("looop \n");
		usleep (500000);
		usleep (500000);
	}
	return 0;
}
