#include <Utils.hpp>

#include <time.h>

#include <iostream>
#include <fstream>
#include <sstream>

// C library headers
#include <cstdio>
#include <stdio.h>
#include <string>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

int totalDefaultMessage = 9;
defaultMessage dm[9] {
    {"> AT",1},
    {"> RTB",2},
    {"> START",3},
    {"> PARAM",4},
    {"> WAYPOINT",5},
    {"> PING",6},
    {"> DATA",7},
    {"> STOP",8},
    {"> OPERATION",9}
};

void Utils::foo()
{
    printf("program::foo()\n");
}

void Utils::Communication::printConnectionStatus()
{
    switch (Utils::Communication::port)
    {
    case -1 : 
        std::cout<<"\t tcgetattr FAILED"<<std::endl;
        break;
    case -2 : 
        std::cout<<"\t tcsetattr FAILED"<<std::endl;
        break;
    case -3 : 
        std::cout<<"\t port open FAILED"<<std::endl;
        break;
    default :
        std::cout<<"\t port "<<Utils::Communication::port<<" Opened"<<std::endl;
        break;
    }
}


//check if incoming data contain default message
//if found, change values of dataSender, dataCommand, dataArgs, dataCode accordingly.
//if not found, set dataCode as -1
void Utils::Communication::Parse(std::string data)
{   
    bool found = false;
    int i = 0;
    while (!found && i<totalDefaultMessage)
    {
        if (data.find(dm[i].message) != std::string::npos)
        {
            int foundidx = 0,wcount = 0;
            char c;
            std::string word = "";
            while (foundidx<3 && wcount<=data.length())
            {
                char c = data[wcount];
                if (c == ' ' && foundidx <2)
                {
                    if (foundidx == 0)
                    {
                        dataSender = word;
                    }
                    else if (foundidx == 1)
                    {
                        dataCommand = word;                        
                    }
                    word = "";
                    foundidx++;
                }
                else
                {                                        
                    word = word + data[wcount];
                }                
                wcount++;
            }
            dataArgs = word;
            dataCode = dm[i].code;
            found = true;
        }
        i++;
    }
    if (!found)
        dataCode = -1;
}

int Utils::openPort(char* portname,int BAUDRATE)
{
    /*note:
        Serial config is 9600 8n1 / 115200 8n1
        no software flow control
        canonical mode (read until \n (newline))
    */
    struct termios oldtio,newtio;
    int sport = open(portname, O_RDWR | O_NOCTTY | O_SYNC);	
    if (tcgetattr(sport, &oldtio) == -1) {
        return -1;
    }
    cfmakeraw(&newtio); // Clean all settings
    if (BAUDRATE == 115200)
        newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B115200; 
    else
        newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B9600; 
    
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~(PARENB | PARODD); // No parity
    newtio.c_cflag &= ~CRTSCTS; // No hardware handshake
    newtio.c_cflag &= ~CSTOPB; // 1 stopbit
    newtio.c_iflag = IGNBRK;
    newtio.c_iflag &= ~(INPCK| IUCLC | IMAXBEL); 
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // No software handshake
    newtio.c_cc[VEOL] = '\n';
    newtio.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    newtio.c_oflag = 0;
    newtio.c_oflag &= ~OPOST;
    newtio.c_lflag = 0;	
    
    newtio.c_lflag |= ICANON; //Enable canonical mode
    tcflush(sport, TCIFLUSH); // Clear IO buffer
    tcflush(sport, TCIOFLUSH); // Clear IO buffer
    if (tcsetattr(sport, TCSANOW, &newtio) == -1) {
        return -2;
    }
    if(sport < 0) {
        return -3;
    }
    return sport;
}

void Utils::getTime(char* _timeDest)
{
    char _buff[300];
    time_t _time;
    struct tm * _timestamp;
    time(&_time);
    _timestamp = localtime(&_time);
    strftime(_buff, sizeof(_buff), "%d-%m-%Y %H:%M:%S\t", _timestamp);
    strcpy(_timeDest,_buff);
}

//call example : write2LOG("test","message 2 commit")
int Utils::write2LOG(char *logname,char* message,bool _trunc = false)
{
    char time_buffer[300];	
    getTime(time_buffer);
    
    char buffer_logname[50];
    sprintf(buffer_logname,"../logs/%s.txt",logname);
    if (_trunc)
    { 
        std::ofstream log(buffer_logname, std::ios::out | std::ios::trunc);
        if(!log.is_open()) {
            std::cout << "fail to write on "<<logname<<std::endl;
            return 0;
        } else
        {
            log << message << std::endl << time_buffer <<std::endl;
            log.close();
        }
    }
    else
    {
        std::ofstream log(buffer_logname, std::ios::out | std::ios::app);
        if(!log.is_open()) {
            std::cout << "fail to write on "<<logname<<std::endl;
            return 0;
        } else
        {
            log << time_buffer << message << std::endl;
            log.close();
        }
    }
    return 1;
}

void Utils::serWrite(int _port, char _msg[]){
    char* p = _msg;
    for (; *p != '\0'; ++p){
        // if '\0' happens to be valid data for your app, 
        // then you can (maybe) use some other value as sentinel
    }
    int arraySize = p - _msg;	
    write(_port,_msg,arraySize);
    usleep(5000);
}

std::string Utils::convertToString(char* a) { 
    std::string s(a); return s; 
} 

/*
int sendDatalog(bool deb = false)
{
	// datalog variables
	std::string datalog_day;
	std::string datalog_mon;
	std::string datalog_date;
	std::string datalog_year;
	std::string datalog_hour;
	std::string datalog_minute;
	std::string datalog_second;
	std::string datalog_temperature;
	std::string datalog_salinity;
	std::string datalog_altitude;
	std::string datalog_z_terukur;
	std::string datalog_latitude;
	std::string datalog_longitude;
	std::string datalog_roll;
	std::string datalog_pitch;
	std::string datalog_yawref;
	std::string datalog_yaw_terukur;
	std::string datalog_pos_e;
	std::string datalog_pos_n;
	std::string datalog_motion;
	std::string datalog_battery;
	std::string datalog_leakage;
	std::string datalog_speed;
	std::string datalog_strobo;
	std::string datalog_csum;
	std::string datalog_boardTemp;
	std::string datalog_alti;

    if (deb) std::cout << "opening datalog ... ";
    
    std::ifstream readdatalog("datalog_sensor.txt");
    if(!readdatalog.is_open()) {
        if (deb) std::cout << "Error : file is open" << std::endl;
        return 0;
    }
    if (deb) std::cout << "success" << std::endl;

    //skip lines to avoid duplicates being sent
		for (int i = 0; i<glider.datalogCount; i++)
		{
					string buffer;
					getline(readdatalog,buffer);
                    if (deb) std::cout << "skip line " << i+1 << std::endl;
		}

	//check file condition & EOF
    while((readdatalog.good()) && (readdatalog.peek() != EOF)) {	
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
        getline(readdatalog,datalog_boardTemp,' ');
        getline(readdatalog,datalog_alti,' ');
        getline(readdatalog,datalog_csum,'\n');
		
        char buffer[3000];

		strcpy (buffer,("#TS> DATA "+datalog_day+" "+datalog_mon+" "+datalog_date+" "+datalog_year+" "+datalog_hour+":"+datalog_minute+":"+datalog_second+" "+
				datalog_temperature+" "+datalog_salinity+" "+datalog_altitude+" "+datalog_z_terukur+" "+
				datalog_latitude+" "+datalog_longitude+" "+datalog_roll+" "+datalog_pitch+" "+
				datalog_yaw_terukur+" "+datalog_motion+" "+datalog_battery+" "+datalog_leakage+" "+
				datalog_speed+" "+datalog_strobo+" "+datalog_boardTemp+" "+datalog_alti+" "+
				datalog_csum+"\r\n").c_str());

		if (deb) std::cout << "sending data" << std::endl;
		serWrite(GS.port,buffer);
		
        //mark sent lines
        glider.datalogCount++;

        // check EOF
        if(readdatalog.peek() == EOF) 
            if (deb) std::cout << "End of file reached" << std::endl;
    }
    
    readdatalog.close();
    if (deb) std::cout << "Sending Datalog Finished" << std::endl;	
    
    return 1;
}
*/