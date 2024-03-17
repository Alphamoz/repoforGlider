#pragma GCC diagnostic ignored "-Wwrite-strings"
#pragma GCC diagnostic ignored "-Wformat="
// libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stddef.h>
#include <iostream>
#include "stdio.h"
#include "termios.h"
#include "errno.h"
#include "fcntl.h"
#include "string.h"
#include "time.h"
#include <stdlib.h>
#include "sys/select.h"
#include <cstring>
#include <sstream>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <iomanip>
#include <sstream>
#include <time.h>
#include <stdio.h> // This ert_main.c example uses printf/fflush
#include <typeinfo>
#include <vector>
#include <signal.h>
#include <thread>
#include <mutex>
#include <string>
#include <glider_msg/miniCTmsg.h>

using namespace std;

// All function declared here! ============================================

// Read -------------------------------------------------------------------
string ReadSerial(int ports, string head, int line);
float ReadAnalog(int adc);

// Parse ------------------------------------------------------------------
void ParseMiniCT(string Parse_MiniCT);

// Other ------------------------------------------------------------------
int openport(char *portname, int baudrate);
void SerWrite(int _port, char _msg[]);
void process_mem_usage();
void samplingInterupt(int signnum);
inline string getCurrentDateTime(string s);
inline void Logger(string logMsg);
// ------------------------------------------------------------------------

// Struct for Flags =======================================================

struct mark
{
    int BeagleActive = 0; // Set 1 jika rutin superloop(baca sensor, navigasi, guidance, dan send string ke TS) langsung jalan tanpa $#O1 dari command BB
    int BeagleMatlab = 1;
    int Sampling = 0;   // Sampling flag when the time cal
    int SendDataTS = 1; // Flag for always send data instead of ping response by default
    int State = 0;      // State operation of BB : 0
    int CT = 0;         // MiniCT on/off
} Flag;

// Struct for Time =======================================================

struct time
{

    string DateTime[6]; // Store date from PING command (DD-MM-YYYY-hh-mm-ss)
    int counter = 0;
    struct timespec start, finish;                   //
    struct timespec stopwatchStart, stopwatchFinish; //
    double stopwatch = 0.0;

    int valSampling = 0; // Sampling Rate value (in second)

    double times = 0.0;
    double Sampling = 0.0;
    double ElapsedTime = 0.0;
    char *Months[12] = {"Januari", "Februari", "Maret", "April",
                        "Mei", "Juni", "Juli", "Agustus", "September",
                        "Oktober", "November", "Desember"};

} Times;

// Struct for Logging ====================================================

struct log
{

    int newFile = 1;
    int entryNumber = 0;

} Log;

// Struct for serial purpose =============================================

struct Serial
{

    char *loc;
    int port;
    bool isSet()
    {
        if (port > 0)
            return true;
        return false;
    }

} MiniCT;

// Struct for Sensor Value ================================================

struct Sensor
{

    string Condition = "0000000000000000";
    char *buff = new char[100];
    int maxBuff = 20;

    struct _MiniCT
    {
        double conductivity = 0.0;
        double temperature = 0.0;
        char *valid = "0";
    };

    _MiniCT valMiniCT;

} Sensors;

// Main Program ===========================================================
int main(int argc, char **argv)
{

    ros::init(argc, argv, "node_miniCT");
    ros::NodeHandle n;
    ros::Publisher miniCT_message = n.advertise<glider_msg::miniCTmsg>("messageMiniCT", 1000);
    ros::Rate loop_rate(10);
    int count = 0;

    // initializing port and baud
    int Baud_Sensor = 115200;
    MiniCT.loc = "/dev/ttyACM0";
    MiniCT.port = openport(MiniCT.loc, Baud_Sensor);
    if (!MiniCT.isSet())
        cout << MiniCT.loc << "Open Error" << endl;

    // Error Serial in String
    string gagal = "gagal";
    string time_out = "timeout";

    clock_gettime(CLOCK_MONOTONIC, &Times.stopwatchStart);

    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */

        Flag.Sampling = 0;
        signal(SIGALRM, samplingInterupt);
        alarm(Times.valSampling);

        clock_gettime(CLOCK_MONOTONIC, &Times.stopwatchFinish);
        Times.stopwatch = (Times.stopwatchFinish.tv_sec - Times.stopwatchStart.tv_sec);
        Times.stopwatch += (Times.stopwatchFinish.tv_nsec - Times.stopwatchStart.tv_nsec) / 1000000000.0;
        // Logger(to_string(Times.stopwatch));
        clock_gettime(CLOCK_MONOTONIC, &Times.stopwatchStart);

        int MiniCT_read_attempt = 0;
        int Alti_read_attempt = 0;
        int DVL_read_attempt = 0;
        int sensor_limit_attempt = 3;
        ROS_INFO("Reading Sensor Data");
        // getting MiniCT data
        while (MiniCT_read_attempt < sensor_limit_attempt)
        {
            MiniCT_read_attempt += 1;
            string MiniCT_Data = ReadSerial(MiniCT.port, "T=", 1);
            ROS_INFO("The Data is: %s", MiniCT_Data);
            if (MiniCT_Data != gagal && MiniCT_Data != time_out)
            {
                Sensors.valMiniCT.valid = "1";
                ParseMiniCT(MiniCT_Data);
                break;
            }
            else
            {
                ROS_INFO("Failed, %s", MiniCT_Data);
            }
        }

        glider_msg::miniCTmsg msg;

        // std::stringstream ss("20.0  0.22");
        // float temperature, conductivity;
        // // removing tab delimiter
        // ss >> temperature >> conductivity;

        msg.temperature = Sensors.valMiniCT.temperature;
        msg.conductivity = Sensors.valMiniCT.conductivity;

        ROS_INFO("%s", msg);

        miniCT_message.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;

}

// Universal function for reading from serial port (TS & Sesnsor) ------------------
string ReadSerial(int ports, string head, int line)
{

    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 5000;

    int j = 0;
    string data;

    fd_set readfs;
    FD_ZERO(&readfs);
    FD_SET(ports, &readfs);
    int status = select(ports + 1, &readfs, NULL, NULL, &tv);
    ROS_INFO("Status is: %d", status);
    if (status == -1)
    {
        perror("status");
        return "gagal";
    }
    else if (status == 0)
    {
        return "timeout";
    }
    else
    {
        while (j < line)
        {
            char read_buffer[1000];
            memset(&read_buffer, '\0', sizeof(read_buffer));
            int bytes_read = read(ports, &read_buffer, sizeof(read_buffer));
            // cout << "\nReceived Serial Data : \r\n" << read_buffer << endl;
            // cout << bytes_read << "bytes" << endl;
            data += read_buffer;
            j += 1;
        }
        size_t found = data.find(head);
        if (found != string::npos)
        {
            data.erase(0, found);
            return data;
        }
        else if (head == "T=" && data.find("\r\n") && data.find("\t"))
        {
            return data;
        }
        else
        {
            return "gagal";
        }
    }
    tcflush(ports, TCIOFLUSH); // Clear IO buffer
}

// Reading analog -------------------------------------------------------------------
float ReadAnalog(int adc)
{
    string str;
    ifstream AdcFile("/sys/bus/iio/devices/iio:device0/in_voltage" + std::to_string(adc) + "_raw");
    getline(AdcFile, str);
    AdcFile.close();
    return std::stof(str);
}

// Parse Data MinicT -----------------------------------------------------------------
void ParseMiniCT(string Parse_MiniCT)
{
    int i = 1;
    char *pch;
    char *data = new char[50];
    strcpy(data, Parse_MiniCT.c_str());
    pch = strtok(data, "\t");

    while (pch != NULL)
    {
        if (i == 1)
        {
            strncpy(Sensors.buff, pch, Sensors.maxBuff);
            Sensors.valMiniCT.temperature = atof(Sensors.buff);
        }
        else if (i == 2)
        {
            strncpy(Sensors.buff, pch, Sensors.maxBuff);
            Sensors.valMiniCT.conductivity = atof(Sensors.buff);
        }
        pch = strtok(NULL, "\t");
        i += 1;
    }
    // print these lines if you want -------
    // printf ("CT_temperature %f\n",Sensors.valMiniCT.temperature);
    // printf ("CT_conductivity %f\n\n",Sensors.valMiniCT.conductivity);
}

// For opening serial port -------------------------------------------------------
int openport(char *portname, int baudrate)
{
    struct termios oldtio, newtio;

    int sport = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (tcgetattr(sport, &oldtio) == -1)
    {
        cout << "tcgetattr failed" << endl;
        return -1;
    }

    cfmakeraw(&newtio); // Clean all settings
    if (baudrate == 9600)
    {
        newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B9600; // 8 databits
    }
    else if (baudrate == 115200)
    {
        newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B115200; // 8 databits
    }
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~(PARENB | PARODD); // No parity
    newtio.c_cflag &= ~CRTSCTS;           // No hardware handshake
    newtio.c_cflag &= ~CSTOPB;            // 1 stopbit
    newtio.c_iflag = IGNBRK;
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // No software handshake
    newtio.c_lflag = 0;
    newtio.c_oflag = 0;
    newtio.c_lflag |= ICANON; // Enable canonical mode
    newtio.c_cc[VEOL] = '\n';
    newtio.c_iflag &= ~(INLCR | IGNCR | ICRNL);

    tcflush(sport, TCIFLUSH);  // Clear IO buffer
    tcflush(sport, TCIOFLUSH); // Clear IO buffer

    if (tcsetattr(sport, TCSANOW, &newtio) == -1)
    {
        cout << "tcsetattr failed" << endl;
        return -1;
    }

    tcflush(sport, TCIOFLUSH); // Clear IO buffer

    cout << "port open : " << sport << endl;
    if (sport < 0)
    {
        printf("Error opening %s: \n ", strerror(errno));
        return -1;
    }
    return sport;
}

// For write serial without count the chars, save your time -----------------------
void SerWrite(int _port, char _msg[])
{
    char *p = _msg;

    for (; *p != '\0'; ++p)
    {
        // if '\0' happens to be valid data for your app,
        // then you can (maybe) use some other value as
        // sentinel
    }
    int arraySize = p - _msg;

    write(_port, _msg, arraySize);
}

// Calculate Memory Usage -----------------------------------------------------------
void process_mem_usage()
{
    float vm_usage = 0.0;
    float resident_set = 0.0;

    // the two fields we want
    unsigned long vsize;
    double vm;
    long rss;
    {
        std::string ignore;
        std::ifstream ifs("/proc/self/stat", std::ios_base::in);
        ifs >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> vsize >> rss;
    }

    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
    vm_usage = vsize / 1024.0;
    resident_set = rss * page_size_kb;
    cout << "VM\t: " << vm << "\nRSS\t: " << rss << endl;
}

// Get time and log ---------------------------------------------------------------------

void samplingInterupt(int signnum)
{
    printf("Inside handler function\n");
    Flag.Sampling = 1;
    // Times.Sampling = 0;
}

string getCurrentDateTime(string s)
{

    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    if (s == "now")
        strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);
    else if (s == "date")
        strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);
    return string(buf);
};

inline void Logger(string logMsg)
{
    // cout << "Creating stopwatch 3 log" << endl;
    string filePath = "/home/ubuntu/Baru/115200/1000ms/stopwatch3.txt";
    // string now = getCurrentDateTime("now");
    ofstream ofs(filePath.c_str(), std::ios_base::out | std::ios_base::app);
    ofs << logMsg << '\n';
    ofs.close();
};
// ------------------------------------------------------------------------------------------
