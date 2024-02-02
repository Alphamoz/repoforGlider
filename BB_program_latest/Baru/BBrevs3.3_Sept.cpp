// supress warning Wwrite-string
#pragma GCC diagnostic ignored "-Wwrite-strings"
#pragma GCC diagnostic ignored "-Wformat="

// libraries
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
#include "imu.h"
#include <vector>
#include <signal.h>
#include <thread>
#include <mutex>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace std;

// All function declared here! ============================================

// Read -------------------------------------------------------------------
void ReadSetting(char *configtxt);
void ReadCommand(int _fd, string command);
string ReadSerial(int ports, string head, int line);
float ReadAnalog(int adc);
// Send -------------------------------------------------------------------
void SendTS(int ports, vector<float> data, int size);
void SendStatusTS(int ports);
void LogData(void);
// Parse ------------------------------------------------------------------
void ParsePing(string command);
void ParseParam(string command);
void ParseAlti(string Parse_Alti);
void ParseMiniCT(string Parse_MiniCT);
void ParseDVL(string Parse_DVL);
// Other ------------------------------------------------------------------
int openport(char *portname, int baudrate, std::string connectionType = "Serial");
void SerWrite(int _port, char _msg[], std::string connectionType = "TCP");
// void gpioActivation();
int TriggerDVL();
int Relay_On();
int Relay_Off();
void process_mem_usage();
void samplingInterupt(int signnum);
inline string getCurrentDateTime(string s);
inline void Logger(string logMsg);
void getDVLdata();
void tcp_listener(int serverSocket);
bool tcp_is_connected = false;
std::vector<int> clientSockets;
int clientSocket;
// ------------------------------------------------------------------------

// Struct for Flags =======================================================

struct mark
{

    int IMUActive = 0;    // Set 0 Jika tidak dikoneksikan ke IMU via USB, 1 jika dikoneksikan. Program Exit jika tidak mendeteksi IMU
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

} MiniCT, Alti, DVL, TS, USB;

// Struct for GPIO purpose ================================================

struct gpio
{

    int Pin;
    FILE *Handler = NULL;
    char Set[4], String[4], Value[64], Direction[64];

} DVL_GPIO, Relay_GPIO;

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

    struct _Alti
    {
        float depth = 0.0;
        float pressure = 0.0;
        char *valid = "0";
    };

    struct _DVL
    {
        float altitude1 = 0.0, altitude2 = 0.0, altitude3 = 0.0, altitude4 = 0.0;
        float velo_rad1 = 0.0, velo_rad2 = 0.0, velo_rad3 = 0.0, velo_rad4 = 0.0;
        float wvelo_rad1 = 0.0, wvelo_rad2 = 0.0, wvelo_rad3 = 0.0, wvelo_rad4 = 0.0;
        float cwvelo_rad1 = 0.0, cwvelo_rad2 = 0.0, cwvelo_rad3 = 0.0, cwvelo_rad4 = 0.0;
        float veloX = 0.0, veloY = 0.0, veloZ = 0.0, veloFLAG = 0.0;
        float wveloX = 0.0, wveloY = 0.0, wveloZ = 0.0, wveloFLAG = 0.0;
        float altitudeMin = 0.0, Temperature = 0.0, salinity = 0.0, soundspeed = 0.0;
        float ceksum = 0.0;
        float Speed = 0.0;
        char *valid = "0";
    };

    struct _IMU
    {
        double Latitude;
        double Longitude;
        double Height;
        double Roll;
        double Pitch;
        double Heading;
        double accel_X;
        double accel_Y;
        double accel_Z;
        double Omega_Roll;    // 7
        double Omega_Pitch;   // 5
        double Omega_Heading; // 6
        double Pitch_offset;
        char *valid = "0";
        int Status;
        double posCov;
    };

    struct _Analog
    {
        float Volt = 0;    // 1
        float Current = 0; // 2
        float Leak = 0;    // 3
        char *Volt_Valid = "0";
        char *Current_Valid = "0";
        char *Leak_Valid = "0";
    };

    _MiniCT valMiniCT;
    _Alti valAlti;
    _DVL valDVL;
    _IMU valIMU;
    _Analog valAnalog;

    // SENSOR 1-3 : primary
    // sensor 4-15 : keperluan navigasi
    // sensor 16-xx : payload

} Sensors;

// Main Program ===========================================================
int main()
{
    // gpioActivation();
    // Relay_On();
    // sleep(2);
    // Relay_Off();
    // sleep(4);
    // Relay_On();
    // sleep(2);

    //    gpioActivation();

    //    Relay_Off();
    //    sleep(2);
    //    Relay_On();
    //    sleep(2);
    //    Relay_Off();

    //    int fd = open("/sys/class/gpio/gpio46/value", O_RDWR | O_CLOEXEC ); // Relay 1
    //    if( fd < 0 ) {
    //      fprintf( stderr, "open: %m\n" );
    //       exit(1);
    //    }

    //    char c = '0'; // 0 = off, 1 == on
    //    write( fd, &c, 1 );
    //    usleep(1000000);
    //    c = '1';
    //    write( fd, &c, 1 );
    //    close( fd );
    //    usleep(5000000);

    ReadSetting("Config.txt");

    if (Flag.IMUActive)
    {
        init_imu();
        sleep(1);
    }

    // Serial Setting ---------------------------------------------------------
    // Set Baudrate
    int Baud_Sensor = 115200;
    int Baud_TS = 9600;

    if (Flag.BeagleMatlab == 1)
    {
        USB.loc = "/dev/ttyUSB0";
        USB.port = openport(USB.loc, Baud_TS);
        if (!USB.isSet())
            cout << USB.loc << "Open Error" << endl;
    }

    // Port MiniCT
    MiniCT.loc = "/dev/ttyMCT";
    MiniCT.port = openport(MiniCT.loc, Baud_Sensor);
    if (!MiniCT.isSet())
        cout << MiniCT.loc << "Open Error" << endl;

    // Port Alti
    Alti.loc = "/dev/ttyALTI";
    Alti.port = openport(Alti.loc, Baud_Sensor);
    if (!Alti.isSet())
        cout << Alti.loc << "Open Error" << endl;

    // Port DVL
    DVL.loc = "/dev/ttyDVL";
    DVL.port = openport(DVL.loc, Baud_Sensor);
    if (!DVL.isSet())
        cout << DVL.loc << "Open Error" << endl;

    // Port TS
    // TS.loc = "/dev/ttyS2";
    TS.loc = "192.168.2.123";
    int TSport = 5001;
    TS.port = openport(TS.loc, TSport, "TCP");
    if (!TS.isSet())
        cout << TS.loc << "Open Error" << endl;

    // Error Serial in String
    string gagal = "gagal";
    string time_out = "timeout";

    // Initialize GPIO --------------------------------------------------------

    //    printf("\nStarting GPIO output program\n");
    //    gpioActivation();

    // Loop Goes Here ---------------------------------------------------------

    clock_gettime(CLOCK_MONOTONIC, &Times.stopwatchStart);
    // activating DVL thread
    thread DVL_thread(getDVLdata);
    thread TCP_listener(tcp_listener, TS.port);
    // cout.precision(6);

    while (1)
    {
        if (Flag.BeagleMatlab == 0)
        {
            string TS_Command = ReadSerial(TS.port, "#TS>", 1);
            // cout << "DATA : "<<TS_Command<<endl; //Know the data from TS
            // cout << TS_Command << endl;
            ReadCommand(TS.port, TS_Command);

            if ((Times.valSampling != 0 || Flag.BeagleActive) && (Flag.Sampling || Flag.BeagleActive))
            {

                Flag.Sampling = 0;
                signal(SIGALRM, samplingInterupt);
                alarm(Times.valSampling);

                clock_gettime(CLOCK_MONOTONIC, &Times.stopwatchFinish);
                Times.stopwatch = (Times.stopwatchFinish.tv_sec - Times.stopwatchStart.tv_sec);
                Times.stopwatch += (Times.stopwatchFinish.tv_nsec - Times.stopwatchStart.tv_nsec) / 1000000000.0;
                // Logger(to_string(Times.stopwatch));
                clock_gettime(CLOCK_MONOTONIC, &Times.stopwatchStart);

                if (Flag.State > 0 || Flag.BeagleActive)
                {

                    int MiniCT_read_attempt = 0;
                    int Alti_read_attempt = 0;
                    int DVL_read_attempt = 0;
                    int sensor_limit_attempt = 3;

                    while (MiniCT_read_attempt < sensor_limit_attempt)
                    {
                        MiniCT_read_attempt += 1;
                        string MiniCT_Data = ReadSerial(MiniCT.port, "T=", 1);
                        // cout << "\nReading MiniCT" << " attempt " << MiniCT_read_attempt << endl;
                        // cout << MiniCT_Data << endl;
                        if (MiniCT_Data != gagal && MiniCT_Data != time_out)
                        {
                            Sensors.valMiniCT.valid = "1";
                            ParseMiniCT(MiniCT_Data);
                            break;
                        }
                        else
                        {
                            // Sensors.valMiniCT.conductivity = 0;
                            // Sensors.valMiniCT.temperature = 0;
                        }
                    }

                    if (Flag.IMUActive)
                    {
                        tcflush(Cport, TCIOFLUSH);
                        usleep(50000);
                        read_imu(Cport,
                                 &Sensors.valIMU.Latitude, &Sensors.valIMU.Longitude, &Sensors.valIMU.Height,
                                 &Sensors.valIMU.Roll, &Sensors.valIMU.Pitch, &Sensors.valIMU.Heading,
                                 &Sensors.valIMU.accel_X, &Sensors.valIMU.accel_Y, &Sensors.valIMU.accel_Z,
                                 &Sensors.valIMU.Omega_Roll, &Sensors.valIMU.Omega_Pitch, &Sensors.valIMU.Omega_Heading, &Sensors.valIMU.Status, &Sensors.valIMU.posCov);
                        // printf("Reading IMU\n");
                        // printf("Reading GPS Status = %f\n",Sensors.valIMU.Status);
                        // printf("\tLatitude = %f, Longitude = %f, Height = %f\n", Sensors.valIMU.Latitude, Sensors.valIMU.Longitude, Sensors.valIMU.Height);
                        // printf("\tAccelerometers X: %f Y: %f Z: %f\n", Sensors.valIMU.accel_X, Sensors.valIMU.accel_Y, Sensors.valIMU.accel_Z);
                        // printf("\tGyroscopes Roll: %f Pitch: %f Heading: %f\n", Sensors.valIMU.Roll, Sensors.valIMU.Pitch, Sensors.valIMU.Heading);
                        // printf("\tOmega Roll: %f Pitch: %f Heading: %f\n", Sensors.valIMU.Omega_Roll, Sensors.valIMU.Omega_Pitch, Sensors.valIMU.Omega_Heading);
                        // printf("\tStatusGPS : %f\n", Sensors.valIMU.Status);
                        Sensors.valIMU.valid = "1";
                        /*end read IMU*/
                    }
                    while (Alti_read_attempt < sensor_limit_attempt)
                    {
                        Alti_read_attempt += 1;
                        SerWrite(Alti.port, "S\r\n", "Serial");
                        // tcdrain(sport2);
                        string Alti_Data = ReadSerial(Alti.port, "$PRVAT", 1);
                        // cout << "Reading Alti" << " attempt " << Alti_read_attempt << endl;
                        // cout << Alti_Data << endl;
                        if (Alti_Data != gagal && Alti_Data != time_out)
                        {
                            Sensors.valAlti.valid = "1";
                            ParseAlti(Alti_Data);
                            break;
                        }
                        else
                        {
                            // Sensors.valAlti.depth = 0;
                            // Sensors.valAlti.pressure = 0;
                        }
                        // cout << "ALTI_Depth :" <<Sensors.valAlti.depth<<endl;
                        // cout << "ALTI_Pressure :" <<Sensors.valAlti.pressure<<endl;
                    }
                    // tcflush(DVL.port, TCIOFLUSH);
                    // usleep(100);

                    // priorities -------
                    // Sensors.valAnalog.Volt = ReadAnalog(0);
                    // cout << "\nAnalog channel 0: " << Sensors.valAnalog.Volt << endl;
                    if (Sensors.valAnalog.Volt != 0)
                    {
                        Sensors.valAnalog.Volt_Valid = "1";
                    }

                    // Sensors.valAnalog.Current = ReadAnalog(1);
                    // cout << "Analog channel 1: " << Sensors.valAnalog.Current << endl;
                    if (Sensors.valAnalog.Current != 0)
                    {
                        Sensors.valAnalog.Current_Valid = "1";
                    }

                    // Sensors.valAnalog.Leak = ReadAnalog(2);
                    // cout << "Analog channel 2: " << Sensors.valAnalog.Leak << endl;
                    if (Sensors.valAnalog.Leak != 0)
                    {
                        Sensors.valAnalog.Leak_Valid = "1";
                    }
                    // -----------------

                    if (Flag.State == 1)
                    {
                        SendStatusTS(TS.port);
                        Flag.State = 0;
                    }
                    else
                    {
                        cout << "Sending Data to TS" << endl;
                        // DUMMYLATLONG
                        Sensors.valIMU.Latitude = 123.4567890123456789;
                        Sensors.valIMU.Longitude = 12.34567890123456789;
                        std::vector<float> dataSend;
                        if (Flag.State > 1 || Flag.BeagleActive)
                        {
                            // From Analog
                            dataSend.push_back(Sensors.valAnalog.Volt);
                            dataSend.push_back(Sensors.valAnalog.Current);
                            dataSend.push_back(Sensors.valAnalog.Leak);

                            if (Flag.State > 2 || Flag.BeagleActive)
                            {
                                // From IMU
                                dataSend.push_back(Sensors.valIMU.Pitch);   // Sensors.valIMU.Omega_Pitch
                                dataSend.push_back(Sensors.valIMU.Heading); // Sensors.valIMU.Omega_Heading
                                dataSend.push_back(Sensors.valIMU.Roll);    // Sensors.valIMU.Omega_Roll
                                dataSend.push_back(Sensors.valIMU.accel_X);
                                dataSend.push_back(Sensors.valIMU.accel_Y);
                                dataSend.push_back(Sensors.valIMU.accel_Z);
                                dataSend.push_back(Sensors.valIMU.Latitude);
                                dataSend.push_back(Sensors.valIMU.Longitude);
                                dataSend.push_back(Sensors.valIMU.Status);
                                // From DVL
                                cout << "Sending DVL data" << endl;
                                cout << Sensors.valDVL.veloX << endl;
                                cout << Sensors.valDVL.veloX << endl;
                                dataSend.push_back(Sensors.valDVL.veloX);
                                dataSend.push_back(Sensors.valDVL.veloY);
                                dataSend.push_back(Sensors.valDVL.veloZ);
                                // From Alti
                                dataSend.push_back(Sensors.valAlti.depth);

                                if (Flag.State > 3 || Flag.BeagleActive)
                                {
                                    dataSend.push_back(Sensors.valMiniCT.temperature);
                                    dataSend.push_back(Sensors.valMiniCT.conductivity);
                                }
                                cout << "Sensor_GPS_IMU :" << endl;
                                cout << "Status_GPS :" << Sensors.valIMU.Status << " Latutude :" << Sensors.valIMU.Latitude << " Longitude :" << Sensors.valIMU.Longitude << endl;
                                cout << "Pitch :" << Sensors.valIMU.Pitch << " Heading :" << Sensors.valIMU.Heading << " Roll :" << Sensors.valIMU.Roll << endl;
                                cout << "accel_X :" << Sensors.valIMU.accel_X << " accel_Y :" << Sensors.valIMU.accel_Y << " accel_Z :" << Sensors.valIMU.accel_Z << endl;
                                cout << "Sensor_DVL :" << endl;
                                cout << "velo_X :" << Sensors.valDVL.veloX << " velo_Y :" << Sensors.valDVL.veloY << " velo_Z :" << Sensors.valDVL.veloZ << endl;
                                cout << "Sensor_ALTI :" << endl;
                                cout << "ALTI_Depth :" << Sensors.valAlti.depth << " Pressure : " << Sensors.valAlti.pressure << endl;
                                cout << "Sensor_MINICT :" << endl;
                                cout << "Conductivity : " << Sensors.valMiniCT.conductivity << " Temperature : " << Sensors.valMiniCT.temperature << endl;
                            }
                        }
                        // dataSend.push_back(--Add on your own--);
                        int arrSize = dataSend.size(); // Calculate the number of data that will be sent
                        // cout << "array length : " << arrSize << endl;
                        SendTS(TS.port, dataSend, arrSize);
                        dataSend.clear();
                        LogData();
                    }
                }
            }
        }
        else
        {
            char *char_Matlab = new char[1000];
            string dataMatlab = ReadSerial(USB.port, "#BB>", 1);
            // cout << dataMatlab << endl;
            if (dataMatlab != gagal && dataMatlab != time_out)
            {
                dataMatlab += "\r\n";
                strcpy(char_Matlab, dataMatlab.c_str());
                SerWrite(TS.port, char_Matlab);
            }
            delete[] char_Matlab;
        }
        usleep(10000);
    }
}

// All Read Purposes =============================================================

// Read Setting from a config.txt ------------------------------------------------

void getDVLdata()
{
    while (true)
    {
        // cout << "Threading DVL Active : \n\n\n\n\n\n\n"
        //      << endl;
        string gagal = "gagal";
        string time_out = "timeout";
        SerWrite(DVL.port, "#&!LQNQ.COMD2828#&!LQNQ.COMD5959 0\r\n", "Serial");
        string DVL_Data = ReadSerial(DVL.port, "$#NQ.RES", 8);

        if (DVL_Data != gagal && DVL_Data != time_out)
        {
            Sensors.valDVL.valid = "1";
            ParseDVL(DVL_Data);
            tcflush(DVL.port, TCIOFLUSH);
            cout << "Selesai Parsing DVL data" << endl;
        }
        usleep(20000);
    }
}

void ReadSetting(char *configtxt)
{

    const string Str_IMUActive = "IMUActive";
    const string Str_BeagleActive = "BeagleActive";
    const string Str_BeagleMatlab = "BeagleMatlab";

    string str;
    ifstream data(configtxt);
    cout << "\nsetting is: " << endl;
    while (getline(data, str))
    {
        if (str.find(Str_IMUActive) != std::string::npos)
        {
            str = str.erase(0, str.length() - 2);
            str = str.erase(1);
            printf("IMU Active\t= %d\n", stoi(str));
            Flag.IMUActive = stoi(str);
        }
        else if (str.find(Str_BeagleActive) != std::string::npos)
        {
            str = str.erase(0, str.length() - 2);
            str = str.erase(1);
            printf("BB debug mode\t= %d\n", stoi(str));
            Flag.BeagleActive = stoi(str);
        }
        else if (str.find(Str_BeagleMatlab) != std::string::npos)
        {
            str = str.erase(0, str.length() - 2);
            str = str.erase(1);
            printf("BB Matlab mode\t= %d\n", stoi(str));
            Flag.BeagleMatlab = stoi(str);
        }
    }
}

// Read Command from TS -----------------------------------------------------------
void ReadCommand(int _fd, string command)
{

    const string TS_BB_PARAM = "#TS> PARAM";
    const string TS_BB_OPERATE = "#TS> OPERATION";

    if (command.find(TS_BB_PARAM) != std::string::npos)
    {
        printf("Param received from TS\r\n");
        SerWrite(_fd, "#BB> ack PARAM\r\n");
        command.erase(0, TS_BB_PARAM.length() + 1); // Remove header of command
        cout << command << endl;
        ParseParam(command);
    }
    else if (command.find(TS_BB_OPERATE) != std::string::npos)
    {
        command.erase(0, TS_BB_OPERATE.length() + 1); // Remove header of command
        if (command.substr(0, 1) == "0")
        {
            SerWrite(_fd, "#BB> ack OPERATION\r\n");
            printf("Standby state\r\n");
            Flag.State = 0;
            Flag.Sampling = 0;
        }
        else if (command.substr(0, 1) == "1")
        {
            printf("Ping state\r\n");
            command.erase(command.end() - 2, command.end()); // Remove header of command
            cout << command << endl;
            ParsePing(command);
            Flag.State = 1;
            Flag.Sampling = 1;
        }
        else if (command.substr(0, 1) == "2")
        {
            SerWrite(_fd, "#BB> ack OPERATION\r\n");
            printf("Standby state\r\n");
            Flag.State = 2;
            Flag.Sampling = 1;
        }
        else if (command.substr(0, 1) == "3")
        {
            SerWrite(_fd, "#BB> ack OPERATION\r\n");
            printf("Read Sensor state\r\n");
            Flag.State = 3;
            Flag.Sampling = 1;
        }
        else if (command.substr(0, 1) == "4")
        {
            SerWrite(_fd, "#BB> ack OPERATION\r\n");
            printf("User Sensor state\r\n");
            Flag.State = 4;
            Flag.Sampling = 1;
        }
    }
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
    try
    {
        string str;
        ifstream AdcFile("/sys/bus/iio/devices/iio:device0/in_voltage" + std::to_string(adc) + "_raw");
        getline(AdcFile, str);
        AdcFile.close();
        return std::stof(str);
    }
    catch (const std::invalid_argument &e)
    {
        // return 0;
    };
}

// All Send Purposes =================================================================

// Send status (reply of the PING command from TS) -----------------------------------
void SendStatusTS(int ports)
{
    char PingReply[100];
    long ConditionStatus;
    // Condition.replace(13,3,State);
    Sensors.Condition.replace(15, 1, Sensors.valAnalog.Volt_Valid);
    Sensors.Condition.replace(14, 1, Sensors.valAnalog.Current_Valid);
    Sensors.Condition.replace(13, 1, Sensors.valAnalog.Leak_Valid);
    Sensors.Condition.replace(12, 1, Sensors.valMiniCT.valid);
    Sensors.Condition.replace(11, 1, Sensors.valAlti.valid);
    Sensors.Condition.replace(10, 1, Sensors.valDVL.valid);
    Sensors.Condition.replace(9, 1, Sensors.valIMU.valid);
    ConditionStatus = strtol(Sensors.Condition.c_str(), 0, 2);
    // cout << Sensors.Condition << endl;
    // cout << ConditionStatus << endl;

    // Send ping reply without status ------------
    // sprintf(PingReply,"#BB> ack PING %s%s%s%s%s%s\r\n",Times.DateTime[0].c_str(),
    //         Times.DateTime[1].c_str(),Times.DateTime[2].c_str(),Times.DateTime[3].c_str(),
    //         Times.DateTime[4].c_str(),Times.DateTime[5].c_str());

    // Send ping reply with status    ------------
    sprintf(PingReply, "#BB> ack PING %s%s%s%s%s%s %ld\r\n", Times.DateTime[0].c_str(),
            Times.DateTime[1].c_str(), Times.DateTime[2].c_str(), Times.DateTime[3].c_str(),
            Times.DateTime[4].c_str(), Times.DateTime[5].c_str(), ConditionStatus);

    SerWrite(ports, PingReply);
    Sensors.Condition = "0000000000000000";
}

// Send Data Required for TS ---------------------------------------------------------
void SendTS(int ports, vector<float> data, int size)
{
    stringstream stream_SendTS;
    string string_SendTS;
    char *char_SendTS = new char[3000];

    // SerWrite(ports, "#BB> DATA 0 0 0 0 0");

    SerWrite(ports, "#BB> DATA ");

    for (int i = -1; i < size; i++)
    {
        if (i < 0)
        {
            stream_SendTS << size;
            string_SendTS += stream_SendTS.str();
        }
        else
        {
            if (i == 9 || i == 10)
            {
                stream_SendTS << fixed <<setprecision(8) << data[i];
            }
            else
            {
                stream_SendTS << defaultfloat << data[i];
            }
            string_SendTS += " " + stream_SendTS.str();
        }
        stream_SendTS.str("");
        stream_SendTS.clear();
    }

    strcpy(char_SendTS, string_SendTS.c_str());
    SerWrite(ports, char_SendTS);
    SerWrite(ports, "\r\n");

    tcdrain(ports);
    cout << "Data sent: " << string_SendTS << endl;
    memset(char_SendTS, 0, sizeof(char_SendTS));
    // avoid memory leak
    delete[] char_SendTS;
}

// Send All Data to Log --------------------------------------------------------------
void LogData()
{
    return;
    // printf("Reading LogData \n");
    Log.entryNumber++;
    printf("%d entries \n", Log.entryNumber);

    stringstream fileName;
    fileName << "BB_LOG_" << Times.DateTime[0] << "-" << Times.DateTime[1] << "-" << Times.DateTime[2] << ".txt";
    // ofstream buffer(Log.fileName.c_str());  // default mode is ios::out | ios::trunc
    ofstream log(fileName.str(), ios::out | ios::app);

    timeval curTime;
    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec / 1000;

    char buffers[80];
    strftime(buffers, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));

    char currentTime[84] = "";
    char datatulis[1000] = "";

    // //printf("current time: %s \n", currentTime);
    // //buffer << currentTime << "\n";
    if (Log.newFile = 1)
    {
        log << "Log Data " << currentTime << "\n";
        printf("Begin data log\n");
        Log.newFile = 0;
        sprintf(datatulis, "altitude1, altitude2, altitude3, altitude4, veloX, veloY, veloZ, veloFLAG, wveloX, wveloY, wveloZ, wveloFLAG, altitudeMin, Speed, Latitude, Longitude, Height, Roll, Pitch, Heading, accel_X, accel_Y, accel_Z,Omega_Roll, Omega_Pitch, Omega_Heading,CT_conductivity, CT_temperature,Sounder_depth,Sounder_pressure,Sounder_altitude, Nav_Lat, Nav_Long, Nav_Z,kecepatanX, kecepatanY, kecepatanZ, ZComp, posisiN_aug, posisiE_aug, psiref, guidestat\n");
        sprintf(currentTime, "%s:%d", buffers, milli);
        log << datatulis << ",";
        log << currentTime << "\n";
    }

    sprintf(datatulis, "%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f",
            Sensors.valMiniCT.conductivity, Sensors.valMiniCT.temperature,
            Sensors.valAlti.pressure, Sensors.valAlti.depth,
            Sensors.valIMU.Latitude, Sensors.valIMU.Longitude, Sensors.valIMU.Height,
            Sensors.valIMU.Roll, Sensors.valIMU.Pitch, Sensors.valIMU.Heading,
            Sensors.valIMU.accel_X, Sensors.valIMU.accel_Y, Sensors.valIMU.accel_Z,
            Sensors.valIMU.Omega_Roll, Sensors.valIMU.Omega_Pitch, Sensors.valIMU.Omega_Heading,
            Sensors.valDVL.altitude1, Sensors.valDVL.altitude2, Sensors.valDVL.altitude3, Sensors.valDVL.altitude4,
            Sensors.valDVL.velo_rad1, Sensors.valDVL.velo_rad2, Sensors.valDVL.velo_rad3, Sensors.valDVL.velo_rad4,
            Sensors.valDVL.wvelo_rad1, Sensors.valDVL.wvelo_rad2, Sensors.valDVL.wvelo_rad3, Sensors.valDVL.wvelo_rad4,
            Sensors.valDVL.cwvelo_rad1, Sensors.valDVL.cwvelo_rad2, Sensors.valDVL.cwvelo_rad3, Sensors.valDVL.cwvelo_rad4,
            Sensors.valDVL.veloX, Sensors.valDVL.veloY, Sensors.valDVL.veloZ, Sensors.valDVL.veloFLAG,
            Sensors.valDVL.altitudeMin, Sensors.valDVL.Temperature, Sensors.valDVL.salinity, Sensors.valDVL.soundspeed,
            Sensors.valAnalog.Volt, Sensors.valAnalog.Current, Sensors.valAnalog.Leak);
    log << datatulis << ",";
    sprintf(currentTime, "%s:%d", buffers, milli);
    log << currentTime << "\n";
    // buffer.close();
    log.close();
}

// All Parsing Purposes ==============================================================

// Parse PING command from TS --------------------------------------------------------
void ParsePing(string command)
{
    int a = 0;
    string data;
    for (unsigned i = 0; i < command.length(); ++i)
    {
        data += command.at(i);
        if (i == 1 || i == 3 || i == 7 || i == 9 || i == 11 || i == 13)
        {
            // Times.DateTime[a] = std::stoi(data);
            Times.DateTime[a] = std::string(data);
            data.clear();
            a++;
        }
    }
    cout << "Date   : " << Times.DateTime[0] << endl;
    cout << "Month  : " << Times.DateTime[1] << endl;
    cout << "Year   : " << Times.DateTime[2] << endl;
    cout << "Hour   : " << Times.DateTime[3] << endl;
    cout << "Minute : " << Times.DateTime[4] << endl;
    cout << "Second : " << Times.DateTime[5] << endl;
}

// Parse PARAM command from TS -------------------------------------------------------
void ParseParam(string command)
{
    std::istringstream _sentence(command);
    char _count = 0;
    do
    {
        std::string _word;
        _sentence >> _word;
        switch (_count)
        {
        case 0:
            std::istringstream(_word) >>
                Times.valSampling;
            break;
            // case 1: std::istringstream (_word) >>
            //                              GA;
            //                              break;
            // case 2: std::istringstream (_word) >>
            //                              CT;
            //                              break;
            // case 3: std::istringstream (_word) >>
            //                              SR;
            //                              break;
            // case 4: std::istringstream (_word) >>
            //                              lat_RTB;
            //                              break;
            // case 5: std::istringstream (_word) >>
            //                              lon_RTB;
            //                              break;
        }
        _count++;
    } while (_sentence);
    printf("Sampling Time : %d second\r\n", Times.valSampling);
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
    delete[] data;
    // print these lines if you want -------
    // printf ("CT_temperature %f\n",Sensors.valMiniCT.temperature);
    // printf ("CT_conductivity %f\n\n",Sensors.valMiniCT.conductivity);
}

// Parse Data Alti ---------------------------------------------------------------------
void ParseAlti(string Parse_Alti)
{
    int i = 1;
    char *pch;
    char *data = new char[50];
    strcpy(data, Parse_Alti.c_str());
    pch = strtok(data, ",");

    while (pch != NULL)
    {
        pch = strtok(NULL, ",");
        if (i == 1)
        {
            strncpy(Sensors.buff, pch, Sensors.maxBuff);
            Sensors.valAlti.pressure = atof(Sensors.buff);
        }
        else if (i == 3)
        {
            strncpy(Sensors.buff, pch, Sensors.maxBuff);
            Sensors.valAlti.depth = atof(Sensors.buff);
        }
        i += 1;
    }
    delete[] data;
    // print these lines if you want -------
    // printf ("AltiSounder altitude %f\n",Sensors.valAlti.pressure);
    // printf ("AltiSounder depth %f\n\n",Sensors.valAlti.depth);
}

// Parse Data DVL ----------------------------------------------------------------------

void ParseDVL(string Parse_DVL)
{
    istringstream iss(Parse_DVL);

    string header;
    string hex_value;
    int b1, b2, b3, b4;
    float alti1, alti2, alti3, alti4;
    float velo_rad1, velo_rad2, velo_rad3, velo_rad4;
    int wvelo_rad1, wvelo_rad2, wvelo_rad3, wvelo_rad4;
    int wvelo_credit1, wvelo_credit2, wvelo_credit3, wvelo_credit4;
    float velo_instrument1, velo_instrument2, velo_instrument3;
    int flag_velo_instrument;
    float velo_earth1, velo_earth2, velo_earth3;
    int flag_velo_earth;
    int water_velo_instrument1, water_velo_instrument2, water_velo_instrument3;
    int flag_water_velo_instrument;
    int water_velo_earth1, water_velo_earth2, water_velo_earth3;
    int flag_water_velo_earth;
    float rph1, rph2, rph3;
    float alti, temperature, pressure, salinity;
    int sound_speed, checksum;

    iss >> header >> hex_value >> b1 >> b2 >> b3 >> b4 >> alti1 >> alti2 >> alti3 >> alti4 >> velo_rad1 >> velo_rad2 >> velo_rad3 >> velo_rad4 >> wvelo_rad1 >> wvelo_rad2 >> wvelo_rad3 >> wvelo_rad4 >> wvelo_credit1 >> wvelo_credit2 >> wvelo_credit3 >> wvelo_credit4 >> velo_instrument1 >> velo_instrument2 >> velo_instrument3 >> flag_velo_instrument >> velo_earth1 >> velo_earth2 >> velo_earth3 >> flag_velo_earth >> water_velo_instrument1 >> water_velo_instrument2 >> water_velo_instrument3 >> flag_water_velo_instrument >> water_velo_earth1 >> water_velo_earth2 >> water_velo_earth3 >> flag_water_velo_earth >> rph1 >> rph2 >> rph3 >> alti >> temperature >> pressure >> salinity >> sound_speed >> checksum;

    Sensors.valDVL.altitude1 = alti1;
    Sensors.valDVL.altitude2 = alti2;
    Sensors.valDVL.altitude3 = alti3;
    Sensors.valDVL.altitude4 = alti4;
    Sensors.valDVL.velo_rad1 = velo_rad1;
    Sensors.valDVL.velo_rad2 = velo_rad2;
    Sensors.valDVL.velo_rad3 = velo_rad3;
    Sensors.valDVL.velo_rad4 = velo_rad4;
    Sensors.valDVL.wvelo_rad1 = wvelo_rad1;
    Sensors.valDVL.wvelo_rad2 = wvelo_rad2;
    Sensors.valDVL.wvelo_rad3 = wvelo_rad3;
    Sensors.valDVL.wvelo_rad4 = wvelo_rad4;
    Sensors.valDVL.cwvelo_rad1 = wvelo_credit1;
    Sensors.valDVL.cwvelo_rad2 = wvelo_credit2;
    Sensors.valDVL.cwvelo_rad3 = wvelo_credit3;
    Sensors.valDVL.cwvelo_rad4 = wvelo_credit4;
    Sensors.valDVL.veloFLAG = flag_velo_instrument;
    if (flag_velo_instrument)
    {
        Sensors.valDVL.veloX = (float)velo_instrument1 / 1000;
        Sensors.valDVL.veloY = (float)velo_instrument2 / 1000;
        Sensors.valDVL.veloZ = (float)velo_instrument3 / 1000;
        Sensors.valDVL.Speed = sqrt((Sensors.valDVL.veloX * Sensors.valDVL.veloX) + (Sensors.valDVL.veloY * Sensors.valDVL.veloY) + (Sensors.valDVL.veloZ * Sensors.valDVL.veloZ));
    }
    Sensors.valDVL.wveloFLAG = flag_water_velo_instrument;
    if (flag_velo_instrument)
    {
        Sensors.valDVL.wveloX = water_velo_instrument1;
        Sensors.valDVL.wveloY = water_velo_instrument2;
        Sensors.valDVL.wveloZ = water_velo_instrument3;
    }
    Sensors.valDVL.altitudeMin = alti;
    Sensors.valDVL.Temperature = temperature;
    Sensors.valDVL.salinity = salinity;
    Sensors.valDVL.soundspeed = sound_speed;
    Sensors.valDVL.ceksum = checksum;
}

// Other Useful Function and Procedure ===========================================

// For opening serial port -------------------------------------------------------
int openport(char *portname, int baudrate, std::string connectionType)
{
    if (connectionType == "Serial")
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
    else if (connectionType == "TCP")
    {
        int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (serverSocket == -1)
        {
            perror("Socket creation failed");
            return 1;
        }

        sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(baudrate); // Change to your desired port
        serverAddr.sin_addr.s_addr = INADDR_ANY;

        if (bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1)
        {
            perror("Binding failed");
            close(serverSocket);
            return 1;
        }

        if (listen(serverSocket, 5) == -1)
        {
            perror("Listening failed");
            close(serverSocket);
            return 1;
        }
        return serverSocket;
        std::cout << "Server is listening on port " << baudrate << " ... " << std::endl;
    }
}
void tcp_listener(int serverSocket)
{

    while (true)
    {
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(serverSocket, &readSet);
        int maxFd = serverSocket;

        for (int clientSocket : clientSockets)
        {
            FD_SET(clientSocket, &readSet);
            maxFd = std::max(maxFd, clientSocket);
        }

        int readySockets = select(maxFd + 1, &readSet, nullptr, nullptr, nullptr);
        if (readySockets == -1)
        {
            perror("Select error");
            break;
        }

        if (FD_ISSET(serverSocket, &readSet))
        {
            sockaddr_in clientAddr;
            socklen_t clientAddrLen = sizeof(clientAddr);
            clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddr, &clientAddrLen);
            if (clientSocket == -1)
            {
                perror("Accepting connection failed");
                continue;
            }
            tcp_is_connected = true;
            std::cout << "Client connected!" << std::endl;
            clientSockets.push_back(clientSocket);
        }

        for (auto it = clientSockets.begin(); it != clientSockets.end();)
        {
            clientSocket = *it;
            if (FD_ISSET(clientSocket, &readSet))
            {
                char buffer[256];
                ssize_t bytesRead = recv(clientSocket, buffer, sizeof(buffer), 0);
                if (bytesRead <= 0)
                {
                    tcp_is_connected = false;
                    std::cout << "Client disconnected!" << std::endl;
                    close(clientSocket);
                    it = clientSockets.erase(it);
                    continue;
                }

                buffer[bytesRead] = '\0'; // Null-terminate the received data
                std::cout << "Received from client: " << buffer << std::endl;

                // Send a response back to the client
                // const char* response = "Message received!";
                // send(clientSocket, response, strlen(response), 0);
            }
            ++it;
        }
        usleep(10000);
    }
}
// For write serial without count the chars, save your time -----------------------
void SerWrite(int _port, char _msg[], std::string connectionType)
{
    char *p = _msg;

    for (; *p != '\0'; ++p)
    {
        // if '\0' happens to be valid data for your app,
        // then you can (maybe) use some other value as
        // sentinel
    }
    int arraySize = p - _msg;
    if (connectionType == "Serial")
    {
        write(_port, _msg, arraySize);
    }
    else if (connectionType == "TCP")
    {
        send(clientSocket, _msg, arraySize, 0);
    }
}

// Procedure for GPIO usage --------------------------------------------------------
void gpioActivation()
{
    // DVL
    DVL_GPIO.Pin = 61;
    sprintf(DVL_GPIO.String, "%d", DVL_GPIO.Pin);
    sprintf(DVL_GPIO.Value, "/sys/class/gpio/gpio%d/value", DVL_GPIO.Pin);
    sprintf(DVL_GPIO.Direction, "/sys/class/gpio/gpio%d/direction", DVL_GPIO.Pin);

    DVL_GPIO.Handler = fopen(DVL_GPIO.Direction, "rb+");
    if (DVL_GPIO.Handler == NULL)
    {
        cout << "error set DVL trigger pin direction" << endl;
        // return 1;
    }

    strcpy(DVL_GPIO.Set, "out");
    fwrite(&DVL_GPIO.Set, sizeof(char), 3, DVL_GPIO.Handler);
    fclose(DVL_GPIO.Handler);

    // Relay
    Relay_GPIO.Pin = 46;
    sprintf(Relay_GPIO.String, "%d", Relay_GPIO.Pin);
    sprintf(Relay_GPIO.Value, "/sys/class/gpio/gpio%d/value", Relay_GPIO.Pin);
    sprintf(Relay_GPIO.Direction, "/sys/class/gpio/gpio%d/direction", Relay_GPIO.Pin);

    Relay_GPIO.Handler = fopen(Relay_GPIO.Direction, "rb+");
    if (Relay_GPIO.Handler == NULL)
    {
        cout << "error set Relay trigger pin direction" << endl;
        // return 1;
    }

    strcpy(Relay_GPIO.Set, "out");
    fwrite(&Relay_GPIO.Set, sizeof(char), 3, Relay_GPIO.Handler);
    fclose(Relay_GPIO.Handler);

    Relay_On();
}

// Function for Relay ON -----------------------------------------------------------
int Relay_On()
{
    Relay_GPIO.Handler = fopen(Relay_GPIO.Value, "rb+");
    if (Relay_GPIO.Handler == NULL)
    {
        cout << "error set Relay trigger pin value" << endl;
        return 1;
    }

    strcpy(Relay_GPIO.Set, "1");
    fwrite(&Relay_GPIO.Set, sizeof(char), 1, Relay_GPIO.Handler);
    fclose(Relay_GPIO.Handler);
    return 0;
}

// Function for Relay OFF -----------------------------------------------------------
int Relay_Off()
{
    Relay_GPIO.Handler = fopen(Relay_GPIO.Value, "rb+");
    if (Relay_GPIO.Handler == NULL)
    {
        cout << "error set Relay trigger pin value" << endl;
        return 1;
    }

    strcpy(Relay_GPIO.Set, "0");
    fwrite(&Relay_GPIO.Set, sizeof(char), 1, Relay_GPIO.Handler);
    fclose(Relay_GPIO.Handler);
    return 0;
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

// DEPRECATED CODE
// ------------------------------------------------------------------------------------------
// void ParseDVL(string Parse_DVL)
// {
//     int datacount = 1;
//     char *pch;
//     char *data = new char[600];
//     strcpy(data, Parse_DVL.c_str());
//     pch = strtok(data, " ");

//     while (pch != NULL)
//     {
//         switch (datacount)
//         {
//         case 1:
//             // Header
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             printf("%s\t", Sensors.buff);
//             datacount++;
//             break;
//         case 2:
//             // Error Codes
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             printf("%s\n", Sensors.buff);
//             datacount++;
//             break;
//         case 3:
//             // Beam Result 1
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             printf("%s\t", Sensors.buff);
//             datacount++;
//             break;
//         case 4:
//             // Beam Result 2
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             printf("%s\t", Sensors.buff);
//             datacount++;
//             break;
//         case 5:
//             // Beam Result 3
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             printf("%s\t", Sensors.buff);
//             datacount++;
//             break;
//         case 6:
//             // Beam Result 4
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             printf("%s\n", Sensors.buff);
//             datacount++;
//             break;
//         case 7:
//             // Altitude 1
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.altitude1 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 8:
//             // Altitude 2
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.altitude2 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 9:
//             // Altitude 3
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.altitude3 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 10:
//             // Altitude 4
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.altitude4 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 11:
//             // Velocity Rad 1
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.velo_rad1 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 12:
//             // Velocity Rad 2
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.velo_rad2 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 13:
//             // Velocity Rad 3
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.velo_rad3 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 14:
//             // Velocity Rad 4
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.velo_rad4 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 15:
//             // W Velocity Rad 1
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.wvelo_rad1 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 16:
//             // W Velocity Rad 2
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.wvelo_rad2 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 17:
//             // W Velocity Rad 3
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.wvelo_rad3 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 18:
//             // W Velocity Rad 4
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.wvelo_rad4 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 19:
//             // CW Velocity Rad 1
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.cwvelo_rad1 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 20:
//             // CW Velocity Rad 2
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.cwvelo_rad2 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 21:
//             // CW Velocity Rad 3
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.cwvelo_rad3 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 22:
//             // CW Velocity Rad 4
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.cwvelo_rad4 = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 23:
//             // Velocity X
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.veloX = atof(Sensors.buff) / 1000;
//             datacount++;
//             break;
//         case 24:
//             // Velocity Y
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.veloY = atof(Sensors.buff) / 1000;
//             datacount++;
//             break;
//         case 25:
//             // Velocity Z
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.veloZ = atof(Sensors.buff) / 1000;
//             datacount++;
//             break;
//         case 26:
//             // Velocity Flag
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.veloFLAG = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 27:
//             datacount++;
//             break;
//         case 28:
//             datacount++;
//             break;
//         case 29:
//             datacount++;
//             break;
//         case 30:
//             datacount++;
//             break;
//         case 31:
//             // W Velocity X
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.wveloX = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 32:
//             // W Velocity Y
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.wveloY = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 33:
//             // W Velocity Z
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.wveloZ = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 34:
//             // W Velocity Flag
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.wveloFLAG = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 35:
//             datacount++;
//             break;
//         case 36:
//             datacount++;
//             break;
//         case 37:
//             datacount++;
//             break;
//         case 38:
//             datacount++;
//             break;
//         case 39: // roll pitch yaw n/a
//             datacount++;
//             break;
//         case 40:
//             datacount++;
//             break;
//         case 41:
//             datacount++;
//             break;
//         case 42:
//             // Altitude Min
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.altitudeMin = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 43:
//             // Temperature
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.Temperature = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 44: // Press n/a
//             datacount++;
//             break;
//         case 45:
//             // Salinity
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.salinity = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 46:
//             // Soundspeed
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.soundspeed = atof(Sensors.buff);
//             datacount++;
//             break;
//         case 47:
//             // Ceksum
//             strncpy(Sensors.buff, pch, Sensors.maxBuff);
//             Sensors.valDVL.ceksum = atof(Sensors.buff);
//             datacount++;

//             Sensors.valDVL.Speed = sqrt((Sensors.valDVL.veloX * Sensors.valDVL.veloX) + (Sensors.valDVL.veloY * Sensors.valDVL.veloY) + (Sensors.valDVL.veloZ * Sensors.valDVL.veloZ));
//             break;
//         default:
//             // Process for all other cases.
//             // printf ("Default/n");
//             break;
//         }
//         pch = strtok(NULL, " ");
//     }
//
//     // print these line if you want -------
//     // printf ("float altitude 1 %f\t",Sensors.valDVL.altitude1);
//     // printf ("altitude 2 %f\t",Sensors.valDVL.altitude2);
//     // printf ("faltitude 3 %f\t",Sensors.valDVL.altitude3);
//     // printf ("altitude 4 %f\n",Sensors.valDVL.altitude4);
//     // printf ("float velo_rad1 %f\t",Sensors.valDVL.velo_rad1);
//     // printf ("velo_rad2 %f\t",Sensors.valDVL.velo_rad2);
//     // printf ("velo_rad3 %f\t",Sensors.valDVL.velo_rad3);
//     // printf ("velo_rad4 %f\n",Sensors.valDVL.velo_rad4);
//     // printf ("float wvelo_rad1 %f\t",Sensors.valDVL.wvelo_rad1);
//     // printf ("wvelo_rad2 %f\t",Sensors.valDVL.wvelo_rad2);
//     // printf ("wvelo_rad3 %f\t",Sensors.valDVL.wvelo_rad3);
//     // printf ("wvelo_rad4 %f\n",Sensors.valDVL.wvelo_rad4);
//     // printf ("float cwvelo_rad1 %f\t",Sensors.valDVL.cwvelo_rad1);
//     // printf ("cwvelo_rad2 %f\t",Sensors.valDVL.cwvelo_rad2);
//     // printf ("cwvelo_rad3 %f\t",Sensors.valDVL.cwvelo_rad3);
//     // printf ("cwvelo_rad4 %f\n",Sensors.valDVL.cwvelo_rad4);
//     // printf ("veloX %f\t",Sensors.valDVL.veloX);
//     // printf ("veloY %f\t",Sensors.valDVL.veloY);
//     // printf ("veloZ %f\t",Sensors.valDVL.veloZ);
//     // printf ("veloFLAG %f\n",Sensors.valDVL.veloFLAG);
//     // printf ("wveloX %f\t",Sensors.valDVL.wveloX);
//     // printf ("wveloY %f\t",Sensors.valDVL.wveloY);
//     // printf ("wveloZ %f\t",Sensors.valDVL.wveloZ);
//     // printf ("wveloFLAG %f\n",Sensors.valDVL.wveloFLAG);
//     // printf ("float altitude Min %f\n",Sensors.valDVL.altitudeMin);
//     // printf ("float Temperature %f\t",Sensors.valDVL.Temperature);
//     // printf ("User defined salinity %f\t",Sensors.valDVL.salinity);
//     // printf ("User defined soundspeed %f\t",Sensors.valDVL.soundspeed);
//     // printf ("User defined ceksum %f\t",Sensors.valDVL.ceksum);
// }

// Function for DVL's Trigger -------------------------------------------------------
// int TriggerDVL()
// {
//     DVL_GPIO.Handler = fopen(DVL_GPIO.Value, "rb+");
//     if (DVL_GPIO.Handler == NULL)
//     {
//         cout << "error set DVL trigger pin value" << endl;
//         return 1;
//     }

//     strcpy(DVL_GPIO.Set, "1");
//     fwrite(&DVL_GPIO.Set, sizeof(char), 1, DVL_GPIO.Handler);
//     fclose(DVL_GPIO.Handler);

//     usleep(300000);

//     DVL_GPIO.Handler = fopen(DVL_GPIO.Value, "rb+");
//     if (DVL_GPIO.Handler == NULL)
//     {
//         cout << "error set DVL trigger pin value" << endl;
//         return 1;
//     }

//     strcpy(DVL_GPIO.Set, "0");
//     fwrite(&DVL_GPIO.Set, sizeof(char), 1, DVL_GPIO.Handler);
//     fclose(DVL_GPIO.Handler);

//     cout << "DVL triggered" << endl;
// }
