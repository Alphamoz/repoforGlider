#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctime>
#include <sys/time.h>

using namespace std;

// datalog variables
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
string datalog_yawref;
string datalog_yaw_terukur;
string datalog_pos_e;
string datalog_pos_n;
string datalog_motion;
string datalog_battery;
string datalog_leakage;
string datalog_speed;
string datalog_strobo;
string datalog_csum;


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

string getLastLine(char filename[])
{
	std::string lastline;

	ifstream readdatalog(filename);
    if(!readdatalog.is_open())
    	cout << "Error : file is open" << endl;
    else
	{
		readdatalog.seekg(-1,std::ios_base::end);
		if (readdatalog.peek() == '\n')
		{
			//Start searching for \n occurrences
			readdatalog.seekg(-1, std::ios_base::cur);
			int i = readdatalog.tellg();
			for(i;i > 0; i--)
			{
				if(readdatalog.peek() == '\n')
				{
				//Found
				readdatalog.get();
				break;
				}
				//Move one character back
				readdatalog.seekg(i, std::ios_base::beg);
			}
		}
		getline(readdatalog, lastline);
	}
	readdatalog.close();
	return lastline;
}


int write2LOG(char *logname, char*message)
{
	//call example : write2LOG("test.log","first commit")

	time_t timeNow;
	struct tm * timestamp;
	time(&timeNow);
	timestamp = localtime(&timeNow);
		
	char time_buffer[300];	
	strftime(time_buffer, sizeof(time_buffer), "%d-%m-%Y %H:%M:%S\t", timestamp); 

	char buffer_logname[50];
	sprintf(buffer_logname,"Log/%s.txt",logname);
	cout << buffer_logname << endl;

	ofstream log(buffer_logname, ios::out | ios::app);
	if(!log.is_open()) {
		cout << "fail to write on "<<logname<<endl;
		return 0;
	} else
	{
		log << time_buffer << message << endl;
	    log.close();
	}
	return 1;
}

int main(){
	write2LOG("test","initial commit");
// 	int sport = open("/dev/ttyxuart4", O_RDWR | O_NOCTTY | O_SYNC);
// 	cout << "port opened" << endl;
// 	if(sport < 0) {
// 		printf("Error opening %s: %s\n", "/dev/ttyxuart4", strerror(errno));
// 		return -1;
// 	}

// 	setattribs();

// 	//.:.time 
// 	time_t timeBB;
// 	struct tm * timestamp;
// 	time(&timeBB);
// 	timestamp = localtime(&timeBB);
		
// 	char buffer[30];
//     strftime(buffer, 30, "#TS> PING %d%m%Y%H%M%S\r\n", timestamp);

// 	cout<<buffer<<endl;
// 	SerWrite(sport,buffer);
// 	//:.:time 
  
// 	/*while(1){
// 	cout <<"Echo : ";
// 	//.:.read data
// 	char read_buffer[1000];
// 	memset(&read_buffer,'\0',sizeof(read_buffer));
// 	int bytes_read = read(sport,&read_buffer,sizeof(read_buffer));
// 	//:.:read data

// 	cout <<read_buffer<<endl;
// 	SerWrite(sport,read_buffer);
// 	}*/

// //	cout << getLastLine("datalog_sensor.txt");

// 	string tes = "tes 123 43";
// 	char buf[300];
// 	char chars [tes.size() + 1];
// 	strcpy(chars,tes.c_str());

// 	sprintf (buf,"string data :%s",chars,sizeof(buf));
// 	cout<< buf << endl;
	return 0;
}