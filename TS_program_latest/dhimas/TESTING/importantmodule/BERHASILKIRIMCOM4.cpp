#include <iostream>
#include <stdio.h>
#include <string.h>

#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

int sport = 0;
string port;
int setattribs()
{
//    char* _port = port.c_str();
    struct termios oldtio,newtio;
    int serial_fd;

    if ((serial_fd = open("/dev/ttyxuart4", O_RDWR | O_EXCL | O_NDELAY)) == -1) 
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
    return serial_fd;

}

void SerWrite(int _port, char _msg[]){
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
	int sport = open("/dev/ttyxuart4", O_RDWR | O_NOCTTY | O_SYNC);
	cout << "port opened" << endl;
	if(sport < 0) {
		printf("Error opening %s: %s\n", "/dev/ttyxuart4", strerror(errno));
		return -1;
	}

setattribs();


	while(1){
	// cout <<"Echo : ";
	// //.:.read data
	// char read_buffer[1000];
	// memset(&read_buffer,'\0',sizeof(read_buffer));
	// int bytes_read = read(sport,&read_buffer,sizeof(read_buffer));
	// //:.:read data

	// cout <<read_buffer<<endl;
	// SerWrite(sport,read_buffer);
    cout <<"writing"<<endl;
	SerWrite(sport, "data sent through DB9\r\n");
	sleep(1);
	SerWrite(sport, "ttyxuart4 - baudrate 115200\r\n");
	sleep(1);
	}
	return 0;
}
