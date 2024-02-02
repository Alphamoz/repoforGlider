//working version to handle serial communication (simplified)

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

void SerWrite(int _port, char _msg[]){
	char* p = _msg;

	for (; *p != '\0'; ++p)
    {
         // if '\0' happens to be valid data for your app, 
         // then you can (maybe) use some other value as
         // sentinel
    }
	int arraySize = p - _msg;
	
//	cout << arraySize << endl;;
	write(_port,_msg,arraySize);
}

char* portname = "/dev/ttyxuart0";

int openport(char* portname)
{
	/*note:
		Serial config is 9600 8n1
		no software flow control
		canonical mode (read until \n (newline))
	*/
	
    struct termios oldtio,newtio;

	int sport = open(portname, O_RDWR | O_NOCTTY | O_SYNC);	

    if (tcgetattr(sport, &oldtio) == -1) {
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

//    newtio.c_iflag &= ~(IXANY); // No software handshake
 //   newtio.c_iflag |= (IXON | IXOFF ); // software handshake

    newtio.c_lflag = 0;
    newtio.c_oflag = 0;
	
//    newtio.c_cc[VTIME] = 0;
//    newtio.c_cc[VMIN] = 1;
	newtio.c_lflag |= ICANON;

    tcflush(sport, TCIFLUSH); // Clear IO buffer
    tcflush(sport, TCIOFLUSH); // Clear IO buffer


    if (tcsetattr(sport, TCSANOW, &newtio) == -1) {
        cout << "tcsetattr failed" << endl;
        return -1;
    }

    //tcflush(sport, TCIOFLUSH); // Clear IO buffer

	cout << "port open : " << sport << endl;
	if(sport < 0) {
		printf("Error opening %s: \n ",strerror(errno));
		return -1;
	}

	return sport;
}

struct Serial{
	char* loc;
	int port;
	bool isSet()
	{
		if (port>0) return true;
		return false;
	}
	
}BB;

int main(){
	BB.loc = "/dev/ttyxuart0";
	BB.port = openport(BB.loc);
	if (BB.isSet()) cout << "SUCCESS OPEN"<<endl;
while(1){
	cout <<"writing to : ";
	cout << BB.port<<endl;
	SerWrite(BB.port, "$#TS ");
	SerWrite(BB.port, "hello ");
	SerWrite(BB.port, "test \r\n"); 
	sleep(1);
	
	cout <<"TRY TO READ "<<endl;
/*	
	char * BBinbound = new char[900];
	int r = read(BB.port, BBinbound, 900);
	
	cout << "FINISH READING BB" << endl;
	BBinbound[r] = '\0';
	cout << BBinbound << endl;
*/
	char read_buf [256];
	memset(read_buf, 0, sizeof(read_buf));
	//tcflush(BB.port,TCIOFLUSH);
	int n = 0;
	n=read(BB.port, &read_buf, sizeof(read_buf)); 
	
	//for (int i=0; i<n; i++) cout << read_buf[i];
	//cout << endl;
	printf("read : %s",read_buf);
	cout << "FINISH READING BB : " <<n<< endl;
}
	close (BB.port);
	return 0;
}
