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
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

std::string workPath = "";

int totalDefaultMessage = 19;
defaultMessage dm[19]{
    {" AT", 1},
    {" RTB", 2},
    {" START", 3},
    {" PARAM", 4},
    {" WAYPOINT", 5},
    {" PING", 6},
    {" DATA", 7},
    {" STOP", 8},
    {" OPERATION", 9},
    {" RTO", 10},
    {" CTM", 11},
    {" SELFTEST", 12},
    {" MANUALGLIDE", 17},
    {" PITCHCONTROL", 14},
    {" DEPTHCONTROL", 15},
    {" NGOTAK", 16},
    {" ZIGZAG", 18},
    {" YAWCONTROL", 19},
    {" TIMETHRESHOLDGLIDE", 24}};

void Utils::foo()
{
    printf("program::foo()\n");
}

int Utils::loadConfig(Communication &c, std::string name)
{
    std::ifstream file("configurations/" + name);
    std::stringstream buffer;

    if (file)
    {
        // copy to buffer
        buffer << file.rdbuf();
        file.close();
    }
    // file not found
    else
    {
        return 0;
    }

    std::string line;
    while (std::getline(buffer, line))
    {
        std::istringstream is_line(line);
        if (line[0] == '#')
        {
            // comment line
        }
        else
        {
            std::string key;
            if (std::getline(is_line, key, '='))
            {
                std::string value;
                if (std::getline(is_line, value))
                {
                    if (key == "location")
                    {
                        c.loc = value;
                    }
                    else if (key == "baudrate")
                    {
                        std::istringstream(value) >> c.baudrate;
                    }
                }
            }
        }
    }
    return 1;
}

void Utils::Communication::printConnectionStatus()
{
    switch (Utils::Communication::port)
    {
    case -1:
        std::cout << "\t tcgetattr FAILED" << std::endl;
        break;
    case -2:
        std::cout << "\t tcsetattr FAILED" << std::endl;
        break;
    case -3:
        std::cout << "\t port open FAILED" << std::endl;
        break;
    default:
        std::cout << "\t port " << Utils::Communication::port << " Opened" << std::endl;
        break;
    }
}

// check if incoming data contain default message
// if command found, change values of dataSender, dataCommand, dataArgs, dataCode accordingly.
// if ack found, change values of isACK according to messagecode
// if not found, set dataCode as -1
void Utils::Parse(Utils::Communication &com)
{
    std::string data = com.rawdata;
    com.dataCommand = "";
    com.dataArgs = "";
    com.dataSender = "";
    bool found = false;
    int i = 0;
    // making sure the message starts with # and ends with /r/n
    if (data.find("#BB") != std::string::npos)
    {
        int first = data.find("#");
        int last = data.rfind("#");
        data = data.substr(first, last-first);
        std::cout << "index of # " << first << "index of rn " << last << std::endl;
        std::cout<< "cropped data " << data <<std::endl;
        first = data.rfind("#");
        last = data.rfind("\r\n");
        data = data.substr(first, last-first);
        std::cout << "index of # " << first << "index of rn " << last << std::endl;
    }

    // #DATA Sender dataCommand dataArgs until new line
    while (!found && i < totalDefaultMessage)
    {
        // message match
        if (data.find(dm[i].message) != std::string::npos)
        {
            // ack (acknowledge) is found
            if (data.find("ack") != std::string::npos)
            {
                com.isACK = dm[i].code;
                com.dataCode = 0;
                found = true;
            }
            // command
            else
            {
                int foundidx = 0, wcount = 0;
                char c;
                std::string word = "";
                // parse data into sender, command, args
                while (foundidx < 3 && wcount <= data.length())
                {
                    char c = data[wcount];
                    if (c == ' ' && foundidx < 2)
                    {
                        if (foundidx == 0)
                        {
                            com.dataSender = word;
                            std::cout << "com Data Sender is: " << com.dataSender << std::endl;
                        }
                        else if (foundidx == 1)
                        {
                            com.dataCommand = word;
                            std::cout << "com Data Command is: " << com.dataCommand << std::endl;
                        }
                        word = "";
                        foundidx++;
                    }
                    else
                    {
                        if (c != '\n' && c != '\r' && c != 0)
                            word = word + data[wcount];
                    }
                    wcount++;
                }

                // check if incoming data has args,
                if (com.dataCommand == "")
                {
                    com.dataCommand = word;
                    com.dataArgs = "null";
                }
                else
                    com.dataArgs = word;

                com.dataCode = dm[i].code;

                com.isACK = 0;
                found = true;
            }
        }

        i++;
    }
    if (!found)
    {
        com.dataCode = -1;
    }
    // std::cout << com.dataCode << std::endl;
}

int Utils::openPort(std::string portname, int BAUDRATE, std::string connectionType="Serial")
{
    /*note:
        Serial config is 9600 8n1 / 115200 8n1
        no software flow control
        canonical mode (read until \n (newline))
    */
    if (connectionType == "Serial")
    {
        struct termios oldtio, newtio;
        int sport = open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (tcgetattr(sport, &oldtio) == -1)
        {
            return -1;
        }
        cfmakeraw(&newtio); // Clean all settings
        if (BAUDRATE == 115200)
            newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B115200;
        else
            newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B9600;

        newtio.c_cflag |= (CLOCAL | CREAD);
        newtio.c_cflag &= ~(PARENB | PARODD); // No parity
        newtio.c_cflag &= ~CRTSCTS;           // No hardware handshake
        newtio.c_cflag &= ~CSTOPB;            // 1 stopbit
        newtio.c_iflag = IGNBRK;
        newtio.c_iflag &= ~(INPCK | IUCLC | IMAXBEL);
        newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // No software handshake
        newtio.c_cc[VEOL] = '\n';
        newtio.c_iflag &= ~(INLCR | IGNCR | ICRNL);
        newtio.c_oflag = 0;
        newtio.c_oflag &= ~OPOST;
        newtio.c_lflag = 0;

        newtio.c_lflag |= ICANON;  // Enable canonical mode
        tcflush(sport, TCIFLUSH);  // Clear IO buffer
        tcflush(sport, TCIOFLUSH); // Clear IO buffer
        if (tcsetattr(sport, TCSANOW, &newtio) == -1)
        {
            return -2;
        }
        if (sport < 0)
        {
            return -3;
        }
        return sport;
    }
    else if (connectionType == "TCP")
    {
        std::cout << "Starting TCP connection" << std::endl;
        int serverPort = BAUDRATE;
        std::string serverIP = portname;
        int socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);
        if (socketDescriptor < 0)
        {
            // Handle error
            return -1;
        }

        struct sockaddr_in serverAddress;
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_port = htons(serverPort);                  // Specify the server's port
        serverAddress.sin_addr.s_addr = inet_addr(serverIP.c_str()); // Specify the server's IP address

        if (connect(socketDescriptor, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0)
        {
            // Handle connection error
            close(socketDescriptor);
            return -2;
        }
        return socketDescriptor;
    }
    // handling invalid connection type
    return -3;
}

void Utils::Communication::sendACK()
{
    std::string message = "#TS> ack ";
    message = message.append(dataCommand);
    message = message.append("\r\n");
    int messagelength = message.length();
    char _message[messagelength + 1];
    strcpy(_message, message.c_str());
    Utils::serWrite(port, _message);
}

void Utils::TCPCommunication::sendACK()
{
    std::string message = "#TS> ack ";
    message = message.append(dataCommand);
    message = message.append("\r\n");
    int messagelength = message.length();
    char _message[messagelength + 1];
    strcpy(_message, message.c_str());
    Utils::tcpWrite(port, _message);
}

void Utils::getTime(char *_timeDest)
{
    char _buff[300];
    time_t _time;
    struct tm *_timestamp;
    time(&_time);
    _timestamp = localtime(&_time);
    strftime(_buff, sizeof(_buff), "%Y-%m-%d %H:%M:%S\t", _timestamp);
    strcpy(_timeDest, _buff);
}

int Utils::write2LOG(char *logname, char *message, bool _trunc)
{
    char time_buffer[300];
    getTime(time_buffer);

    char buffer_logname[50];
    sprintf(buffer_logname, "%slogs/%s.txt", &*workPath.begin(), logname);
    if (_trunc)
    {
        std::ofstream log(buffer_logname, std::ios::out | std::ios::trunc);
        if (!log.is_open())
        {
            std::cout << "fail to write on " << logname << std::endl;
            return 0;
        }
        else
        {
            log << message << std::endl
                << time_buffer << std::endl;
            log.close();
        }
    }
    else
    {
        std::ofstream log(buffer_logname, std::ios::out | std::ios::app);
        if (!log.is_open())
        {
            std::cout << "fail to write on " << logname << std::endl;
            return 0;
        }
        else
        {
            log << time_buffer << message;
            // check message already contains newline
            if (!(message[strlen(message) - 1] == '\n'))
                log << std::endl;
            log.close();
        }
    }
    return 1;
}

void Utils::serWrite(int _port, char _msg[])
{
    char *p = _msg;
    for (; *p != '\0'; ++p)
    {
        // if '\0' happens to be valid data for your app,
        // then you can (maybe) use some other value as sentinel
    }
    int arraySize = p - _msg;
    write(_port, _msg, arraySize);
    tcdrain(_port);
}

int Utils::tcpWrite(int _socketDescriptor, char _data[])
{
    if (_socketDescriptor < 0)
    {
        // Handle invalid socket descriptor
        return -1;
    };

    if (_data == nullptr)
    {
        // Handle invalid data
        return -2;
    };

    // Calculate the length of the data (excluding null terminator)
    int dataSize = strlen(_data);

    // Send the data over the TCP/IP connection
    int bytesSent = send(_socketDescriptor, _data, dataSize, 0);
    if (bytesSent < 0)
    {
        // Handle the error (e.g., failed to send data)
        return -3;
    };

    // Optionally, you can tcdrain or flush here if needed
    tcdrain(_socketDescriptor);
    return bytesSent;
}

std::string Utils::convertToString(char *a)
{
    std::string s(a);
    return s;
}

// Parsing Data For SelfTest
void Utils::GliderUtility::parsingData(std::string str, int *BE_input, int *MM_input, int *rudder_input, int *yaw_input, int *main_input)
{
    std::istringstream _sentence(str);
    char _count = 0;
    do
    {
        std::string _word;
        _sentence >> _word;
        switch (_count)
        {
        case 2:
            std::istringstream(_word) >>
                *BE_input;
            // std::cout << *BE_input << std::endl;
            break;
        case 3:
            std::istringstream(_word) >>
                *MM_input;
            break;
        case 4:
            std::istringstream(_word) >>
                *rudder_input;
            break;
        case 5:
            std::istringstream(_word) >>
                *yaw_input;
            break;
        case 6:
            std::istringstream(_word) >>
                *main_input;
            break;
        }
        _count++;
    } while (_sentence);
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
