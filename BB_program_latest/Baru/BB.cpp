//libraries
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
#include "imu.h"
#include <stdio.h>                     // This ert_main.c example uses printf/fflush 
#include "navigasinonlinirham.h"                           // Model's header file
#include "rtwtypes.h"
#include <math.h>
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/MatrixFunctions"
#include <typeinfo>

using namespace std;
using namespace Eigen;
static navigasinonlinirhamModelClass rtObj;

// Program Setting Definition =============================================
    int IMUActive; //Set 0 Jika tidak dikoneksikan ke IMU via USB, 1 jika dikoneksikan. Program Exit jika tidak mendeteksi IMU
    int BeagleActive; //Set 1 jika rutin superloop(baca sensor, navigasi, guidance, dan send string ke TS) langsung jalan tanpa $#O1 dari command BB
    int SurfaceMode; //Set 1 jika navigasi deadreckoning tidak ingin diaktifkan dan hanya bergantung data GPS IMU(Misal pada mode surfacing saja)
    int SurfaceDepth; //Threshold(dalam meter) menentukan kapan hanya menggunakan data GPS(Perlu dikalibrasi). Asumsi permukaan, Jika Sounder_depth(Dari AltiMeter) > SurfaceDepth
    int DebugGuidance; //Set 1 untuk printf di terminal BB parameter2 Guidance untuk keperluan Debug

// Com_Mainboard variables ================================================
    const string TS_BB_PING          = "#TS> PING BB";
    const string TS_BB_DATA          = "#TS> DATA";
    const string TS_BB_OPERATION     = "#TS> OPERATION";
    const string TS_BB_PARAM         = "#TS> PARAM";
    const string TS_BB_WAYPOINT      = "#TS> WAYPOINT";

// Config.txt setting var ================================================
    const string Str_IMUActive       = "IMUActive";
    const string Str_BeagleActive    = "BeagleActive";
    const string Str_SurfaceMode     = "SurfaceMode";
    const string Str_SurfaceDepth    = "SurfaceDepth";
    const string Str_DebugGuidance   = "DebugGuidance";

// Param Var =============================================================

    int DO = 0; // Depth_Operation
    int GA = 0; // Glide Angle
    int CT = 0; // MiniCT on/off
    int SR = 0; // Sampling Rate
    double lat_RTB = 0.0;
    double lon_RTB = 0.0;

// Waypoint var ==========================================================

    double lat_start = 0.0;
    double lon_start = 0.0;
    double lat_end = 0.0;
    double lon_end = 0.0;

// FLag ==================================================================
    // For status return purpose------------------------------------------
    char * MiniCT_Valid = "0";
    char * Alti_Valid   = "0";
    char * DVL_Valid    = "0";
    char * IMU_Valid    = "0";
    char * Volt_Valid   = "0";
    char * Crnt_Valid   = "0";
    char * Leak_Valid   = "0";
    char * State        = "000";
    string Condition    = "0000000000000000";

    // For operational loop purpose --------------------------------------
    int Ping            = 0;
    int SensorRead      = 0;
    int ProcessRunning  = 0; 
    int SendDataTS      = 0;
    int FLag            = 0;
    int program_running_firsttime_in_the_water =0; //for navigational system initial position 

    // Time --------------------------------------------------------------
    int DateTime[6];
    char * Months[12]   = {"Januari", "Februari", "Maret", "April",
                            "Mei", "Juni", "Juli", "Agustus", "September",
                            "Oktober", "November", "Desember"};
    double times        = 0.0;
    double Sampling     = 0;
    double ElapsedTime  = 0;

// All function declared here! ============================================
    // All these function placed under the main function ------------------
    void ReadSetting(char* configtxt);
    int openport(char* portname, int baudrate);
    string ReadSerial(int ports, string head, int line);
    float ReadAnalog(int adc);
    void SerWrite(int _port, char _msg[]);
    void Com_Mainboard(int _fd, string command);
    void ParsePing(string command);
    void ParseParam(string command);
    void ParseWaypoint(string command);
    void Navigasi(void);
    void GuidanceLOS(void);
    void LogData(void);
    void SendTS(int ports);
    void SendStatusTS(int ports);
    void rt_OneStep(void);
    void Scenario_1(void);
    double distance_two_coordinates(double lat1,double lon1,double lat2,double lon2);
    char * ParseAlti(string Parse_Alti);
    char * ParseMiniCT(string Parse_MiniCT);
    char * ParseDVL(string Parse_DVL);
    int TriggerDVL();
    int Relay_On();
    int Relay_Off();
    void process_mem_usage(double& vm_usage, double& resident_set);

// Variabel Analog =======================================================
    float Volt      = 0;
    float Current   = 0;
    float Leak      = 0;

// Variabel MiniCT =======================================================
    char c_CT_conductivity[8], c_CT_temperature[8];
    double CT_conductivity = 0.0;
    double CT_temperature  = 0.0;

// Variabel Alti Meter ===================================================
    char c_Sounder_altitude[8], c_Sounder_altitude_unit[3],c_Sounder_pressure[10],c_Sounder_pressure_unit[5];
    float Sounder_depth     = 0.0;
    float Sounder_pressure  = 0.0;
    float Sounder_altitude  = 0.0;

// Variabel DVL ==========================================================
// string DVL ------------------------------------------------------------
    char header[10];
    char error_codes[8];
    char beamresult1[2],beamresult2[2],beamresult3[2],beamresult4[2];
    char altitude1[8],altitude2[8],altitude3[8],altitude4[8];
    char velo_rad1[10],velo_rad2[10],velo_rad3[10],velo_rad4[10];
    char wvelo_rad1[10],wvelo_rad2[10],wvelo_rad3[10],wvelo_rad4[10];
    char cwvelo_rad1[5],cwvelo_rad2[5],cwvelo_rad3[5],cwvelo_rad4[5];
    char veloX[10],veloY[10],veloZ[10],veloFLAG[3];
    char wveloX[10],wveloY[10],wveloZ[10],wveloFLAG[3];
    char altitudeMin[8],Temperature[6];
    char salinity[6],soundspeed[8];
    char ceksum[4];

// float DVL -------------------------------------------------------------
    float altitude1_F=0.0,altitude2_F=0.0,altitude3_F=0.0,altitude4_F=0.0;
    float velo_rad1_F=0.0,velo_rad2_F=0.0,velo_rad3_F=0.0,velo_rad4_F=0.0;
    float wvelo_rad1_F=0.0,wvelo_rad2_F=0.0,wvelo_rad3_F=0.0,wvelo_rad4_F=0.0;
    float cwvelo_rad1_F=0.0,cwvelo_rad2_F=0.0,cwvelo_rad3_F=0.0,cwvelo_rad4_F=0.0;
    float veloX_F=0.0,veloY_F=0.0,veloZ_F=0.0,veloFLAG_F=0.0;
    float wveloX_F=0.0,wveloY_F=0.0,wveloZ_F=0.0,wveloFLAG_F=0.0;
    float altitudeMin_F=0.0,Temperature_F=0.0;
    float salinity_F=0.0,soundspeed_F=0.0;
    float ceksum_F=0.0;
    float Speed=0.0;

//Variabel IMU ===========================================================
    double Latitude      =0.0;
    double Longitude     =0.0;
    double Height        =0.0;
    double Roll          =0.0;
    double Pitch         =0.0;
    double Heading       =0.0;
    double accel_X       =0.0;
    double accel_Y       =0.0;
    double accel_Z       =0.0;
    double Omega_Roll    =1.0;      
    double Omega_Pitch   =0.0;     
    double Omega_Heading =0.0;
    double Pitch_offset  =0.0;

//Variabel Send to TS ====================================================
//Dummy Tester -----------------------------------------------------------
// $#NG teta_terukur z_terukur yawref yaw_terukur posx posy altitude depth  salinity temperature speed latitude longitude
//char header[4]  = {'$','#','N','G'}; 
    double dum_teta =   1;
    double dum_z    =   2;
    double dum_yawr =   3;
    double dum_yawt =   -4;
    //double dum_posx   =   5;
    //double dum_posy   =   -6;
    double dum_alti =   5;
    //double dum_dept   =   6;
    double dum_sali =   6;
    double dum_temp =   7;
    double dum_sped =   8;
    double dum_lati =   9;
    double dum_long =   10;
//strings ----------------------------------------------------------------
    string string_teta;
    string string_z;
    string string_yawr;
    string string_yawt;
    string string_posx;
    string string_posy;
    string string_alti;
    string string_dept;
    string string_sali;
    string string_temp;
    string string_sped;
    string string_lati;
    string string_long;
    string string_roll;
    string string_volt;
    string string_crnt;
    string string_leak;
    string string_fnsh;
//stream ------------------------------------------------------------------
    stringstream stream_teta;
    stringstream stream_z;
    stringstream stream_yawr;
    stringstream stream_yawt;
    stringstream stream_posx;
    stringstream stream_posy;
    stringstream stream_alti;
    stringstream stream_dept;
    stringstream stream_sali;
    stringstream stream_temp;
    stringstream stream_sped;
    stringstream stream_lati;
    stringstream stream_long;
    stringstream stream_roll;
    stringstream stream_volt;
    stringstream stream_crnt;
    stringstream stream_leak;
    stringstream stream_fnsh;
//output chars ------------------------------------------------------------
    char char_teta [10];
    char char_z    [10];
    char char_yawr [10];
    char char_yawt [10];
    char char_posx [10];
    char char_posy [10];
    char char_alti [10];
    char char_dept [10];
    char char_sali [10];
    char char_temp [10];
    char char_sped [10];
    char char_lati [13];
    char char_long [13];
    char char_roll [10];
    char char_volt [10];
    char char_crnt [10];
    char char_leak [10];
    char char_fnsh [1];

//Variabel Guidance ======================================================
//High Level Variable ----------------------------------------------------
    double home_lat=0.0,home_long=0.0; //long bujur, lat lintang
    double target_lat=0.0, target_long=0.0;
    double wpX[10]={0,0,0,0,0,0,0,0,0,0},wpY[10]={0,0,0,0,0,0,0,0,0,0};
    int waypoint_step=0,waypoint_number=0; //step between home and target, total number of wp including home and target
    int guidance_wp_target=0; //target waypoint
    int guidance_finish=0;
    double psiref=0.0;

//High Level internal setting variable -----------------------------------
    float distance_step=1.0;//in km dividing between waypoint
    double acceptance_radius=0.00009; //10 meter roughly

//Internal static guidance function --------------------------------------
    double distancefromtarget=100.0;    

//Guidance communication protocol variable -------------------------------
    int guidance_setting=0; // 0 idle, 1 set home, 2 set target, 3 return home
    char home_lat_S[15],home_long_S[15];
    char target_lat_S[15],target_long_S[15];
    double PI = 3.1415926536;
    double temp_wp=0;//temp file for swapping
    int gcount=1;
    int g_iteration=1; //iteration purpose
    double distance_home_target=0; 

//Variabel Navigasi =======================================================
    int ReadNav(int _fd);
    void SendNav(void);
    int kalman_reset = 0;
    VectorXd meas(3);
    VectorXd xhat = VectorXd(5);
    MatrixXd P = MatrixXd(5, 5);
    VectorXd Resid(3);
    VectorXd yhatOut(3);
    VectorXd xhatOut(5);
    MatrixXd A(5, 5);
    MatrixXd Atrans(5, 5);
    MatrixXd C(3, 5);
    MatrixXd Q = MatrixXd::Zero(5, 5);
    MatrixXd matrixR(3, 3);
    MatrixXd K(3, 3);
    MatrixXd Ctrans(5, 3);
    MatrixXd PCtrans(5, 3);
    MatrixXd CPCtransR(3, 3);
    MatrixXd identitysizeK = MatrixXd::Identity(5, 5);
//Initial position --------------------------------------------------------
    double lat_gps=0,lng_gps=0,depth_gps=0;
    double posisiN_aug,posisiE_aug;
//Output ------------------------------------------------------------------
    double Nav_zKF=0;
    double Nav_lat=0,Nav_long=0,Nav_z=0;
    double Nav_kecN=0,Nav_kecE=0,Nav_kecD=0;
//Reading -----------------------------------------------------------------
    int NavValid=0; 
//Static Var --------------------------------------------------------------
    double posisilat_aug = 0;
    double posisilng_aug = 0;
    double posisidepth_aug = 0;
    static double vN_IMU = 0, vE_IMU = 0, vD_IMU = 0;
    static double Z_KF=0,Z_KF_m=0;
    static double Z_Complement=0;
    static int resetNavigasi = 0;

//Variabel Data Log ======================================================
    static int datalog_entrynumber=0;
    int datalog_entrylimit=-1;
    //Dummy tester
    float log_a = 10;
    float log_b = 20;
    float log_c = 30;
    float log_d = 40;
    float log_e = 50;
    int newlogfile = 1;
//Variabel Skenario =======================================================
    int skenario=0;
    static int State_Scenario_1=0;
    int V_propeller=0;
    double yawref_Scenario1;

//Fungsi dan Variabel C Code ==============================================
    void rt_OneStep(void);
    void waitFor (unsigned int secs);

    double vm, rss;

// Struct for serial purpose =============================================
struct Serial{
    
    char* loc;
    int port;
    bool isSet(){
        if (port>0) return true;
        return false;
    }  

}MiniCT,Alti,DVL,TS;

// Struct for GPIO purpose ================================================
struct gpio{
    
    int Pin;
    FILE* Handler = NULL;
    char Set[4],String[4],Value[64],Direction[64];

}DVL_GPIO,Relay_GPIO;

// Main Program ===========================================================
int main(){

    ReadSetting("Config.txt");
    if(IMUActive){
        // Init IMU
        init_imu();
        sleep(1);
        // read_imu(Cport, &Latitude, &Longitude, &Height, &Roll, &Pitch, &Heading, &accel_X, &accel_Y, &accel_Z, &Omega_Roll, &Omega_Pitch, &Omega_Heading);
        /*End Init IMU*/
    }
// Serial Setting ---------------------------------------------------------
    // Set Baudrate
    int Baud_Sensor = 115200;
    int Baud_TS = 9600;

    // Port MiniCT
    MiniCT.loc = "/dev/ttyS4";
    MiniCT.port = openport(MiniCT.loc, Baud_Sensor);
    if(!MiniCT.isSet()) cout << MiniCT.loc << "Open Error" << endl;

    // Port Alti
    Alti.loc = "/dev/ttyO5";
    Alti.port = openport(Alti.loc, Baud_Sensor);
    if(!Alti.isSet()) cout << Alti.loc << "Open Error" << endl;

    // Port DVL
    DVL.loc = "/dev/ttyS1";
    DVL.port = openport(DVL.loc, Baud_Sensor);
    if(!DVL.isSet()) cout << DVL.loc << "Open Error" << endl;

    // Port TS
    TS.loc = "/dev/ttyS2";
    TS.port = openport(TS.loc, Baud_TS);
    if(!TS.isSet()) cout << TS.loc << "Open Error" << endl;

    // Error Serial Variable
    string gagal = "gagal";
    string time_out = "timeout";

// Initialize GPIO --------------------------------------------------------
    printf("\nStarting GPIO output program\n");

    // DVL
    DVL_GPIO.Pin = 61;
    sprintf(DVL_GPIO.String, "%d", DVL_GPIO.Pin);
    sprintf(DVL_GPIO.Value, "/sys/class/gpio/gpio%d/value", DVL_GPIO.Pin);
    sprintf(DVL_GPIO.Direction, "/sys/class/gpio/gpio%d/direction", DVL_GPIO.Pin);
    
    DVL_GPIO.Handler = fopen(DVL_GPIO.Direction,"rb+");
    if (DVL_GPIO.Handler == NULL){
        cout << "error set DVL trigger pin direction" << endl;
        return 1;
    }
    
    strcpy(DVL_GPIO.Set,"out");
    fwrite(&DVL_GPIO.Set, sizeof(char), 3, DVL_GPIO.Handler);
    fclose(DVL_GPIO.Handler);

    // Relay
    Relay_GPIO.Pin = 46;
    sprintf(Relay_GPIO.String, "%d", Relay_GPIO.Pin);
    sprintf(Relay_GPIO.Value, "/sys/class/gpio/gpio%d/value", Relay_GPIO.Pin);
    sprintf(Relay_GPIO.Direction, "/sys/class/gpio/gpio%d/direction", Relay_GPIO.Pin);
    
    Relay_GPIO.Handler = fopen(Relay_GPIO.Direction,"rb+");
    if (Relay_GPIO.Handler == NULL){
        cout << "error set Relay trigger pin direction" << endl;
        return 1;
    }
    
    strcpy(Relay_GPIO.Set,"out");
    fwrite(&Relay_GPIO.Set, sizeof(char), 3, Relay_GPIO.Handler);
    fclose(Relay_GPIO.Handler);

    Relay_On();

// Whlie loop here! ----------------------------------------------------

    while(1){

        string TS_Command = ReadSerial(TS.port,"#TS>",1);
        // cout << "DATA : "<<TS_Command<<endl; //Know the data from TS
        // cout << TS_Command << endl;
        Com_Mainboard(TS.port,TS_Command);

        clock_t start = clock();

        if(FLag || BeagleActive){
            if (Ping == 1 || SensorRead == 1 || ProcessRunning == 1 || BeagleActive == 1){
                
                int MiniCT_read_attempt = 0;
                int Alti_read_attempt = 0;
                int DVL_read_attempt = 0;
                int sensor_limit_attempt = 3;

                while(MiniCT_read_attempt < sensor_limit_attempt){
                    cout << "\nReading MiniCT" << endl;
                    string MiniCT_Data = ReadSerial(MiniCT.port,"T=",1);
                    cout << MiniCT_Data << endl;
                    MiniCT_read_attempt += 1;
                    cout << "attempt " << MiniCT_read_attempt << endl;
                    if(MiniCT_Data != gagal && MiniCT_Data != time_out){
                        MiniCT_Valid = "1";
                        ParseMiniCT(MiniCT_Data);
                        MiniCT_read_attempt = 0;
                        break;
                    }
                }

                if(IMUActive){
                    read_imu(Cport, &Latitude, &Longitude, &Height, &Roll, &Pitch, &Heading, &accel_X, &accel_Y, &accel_Z, &Omega_Roll, &Omega_Pitch, &Omega_Heading);
                    Pitch=Pitch-Pitch_offset;
                    printf("Reading IMU\n");
                    printf("\tLatitude = %f, Longitude = %f, Height = %f\n", Latitude, Longitude, Height);
                    printf("\tAccelerometers X: %f Y: %f Z: %f\n", accel_X, accel_Y, accel_Z);
                    printf("\tGyroscopes Roll: %f Pitch: %f Heading: %f\n", Roll, Pitch, Heading);
                    /*end read IMU*/
                }
        
                while(Alti_read_attempt < sensor_limit_attempt){
                    cout << "Reading Alti" << endl;
                    SerWrite(Alti.port, "S\r\n");
                    // tcdrain(sport2);
                    string Alti_Data = ReadSerial(Alti.port,"$PRVAT",1);
                    cout << Alti_Data << endl;
                    Alti_read_attempt += 1;
                    cout << "attempt " << Alti_read_attempt << endl;
                    if(Alti_Data != gagal && Alti_Data != time_out){
                        Alti_Valid = "1";
                        ParseAlti(Alti_Data);
                        Alti_read_attempt = 0;
                        break;
                    }
                }    
        
                while(DVL_read_attempt < sensor_limit_attempt){
                    cout << "Reading DVL" << endl;
                    TriggerDVL();
                    string DVL_Data = ReadSerial(DVL.port,"$#NQ.RES",1);
                    cout << DVL_Data << endl;
                    DVL_read_attempt += 1;
                    cout << "attempt " << DVL_read_attempt << endl;
                    if(DVL_Data != gagal && DVL_Data != time_out){
                        DVL_Valid = "1";
                        ParseDVL(DVL_Data);
                        DVL_read_attempt = 0;
                        break;
                    } 
                }

                Volt = ReadAnalog(0);
                cout << "\nAnalog channel 0: " << Volt << endl;

                Current = ReadAnalog(1);
                cout << "Analog channel 1: " << Current << endl;

                Leak = ReadAnalog(2);
                cout << "Analog channel 2: " << Leak << endl;

                if(Ping == 1){
                    // SerWrite(TS.port, "This should be status");
                    SendStatusTS(TS.port);
                    Ping = 0;
                }
                else{
                    SendDataTS = 1;
                }
            }

            if (ProcessRunning == 1){
                if(program_running_firsttime_in_the_water==1){
                    lng_gps = Longitude;
                    lat_gps = Latitude;
                    /*End */
                    
                    program_running_firsttime_in_the_water=0;
                    printf("\nThe navigation system is initialized\n");
                }

                Navigasi();
                if(DebugGuidance)printf("\nCurrent Heading \t= %f\n",Heading);
                GuidanceLOS();
                if (skenario == 1) Scenario_1();
                //DataLog & Send TS
                LogData();
            }

            if(SendDataTS == 1){
                SendTS(TS.port);
                SendDataTS = 0;
            }

            if(Ping == 1 || SensorRead == 1 || ProcessRunning == 1){
                times = ((double)clock() - start ) / CLOCKS_PER_SEC;
                printf ( "Processor's time elapsed : %f sec \r\n", times ); 
                process_mem_usage(vm, rss);
                cout << "VM\t: " << vm << "; RSS\t: " << rss << endl;
            }
        }

        if (SR != 0 && (SensorRead == 1 || ProcessRunning == 1)){
            if(ElapsedTime < (SR*1000000)){
                FLag = 0;
                Sampling = ((float)SR / 10) * 1000000;
                ElapsedTime = ElapsedTime + Sampling;
                cout << "Sampling time : " << Sampling << endl;
                cout << "Elapsed time  : " << ElapsedTime << endl;
                cout << "Time Left     : " << ((float)SR*1000000)-ElapsedTime << " usec" << endl;
                usleep((int)Sampling);
            }
            else{
                FLag = 1;
                ElapsedTime = 0;
                cout << "\n\rThe time has come!\r\n" << endl;
            }
        }
        else{
            FLag = 0;
        }

    }
    return 0;
}

// READ SETTING FROM TXT FILE ============================================
void ReadSetting(char* configtxt){
    string str;
    ifstream data(configtxt);
    cout << "\nsetting is: " << endl;
    while(getline(data,str)){
        
        if (str.find(Str_IMUActive) != std::string::npos){
            str = str.erase(0,str.length()-2);
            str = str.erase(1);
            printf("IMU Active\t= %d\n", stoi(str));
            IMUActive = stoi(str);
        } else
        if (str.find(Str_BeagleActive) != std::string::npos){
            str = str.erase(0,str.length()-2);
            str = str.erase(1);
            printf("BB debug mode\t= %d\n", stoi(str));
            BeagleActive = stoi(str);
        } else
        if (str.find(Str_SurfaceMode) != std::string::npos){
            str = str.erase(0,str.length()-2);
            str = str.erase(1);
            printf("Surface Mode\t= %d\n", stoi(str));
            SurfaceMode = stoi(str);
        } else
        if (str.find(Str_SurfaceDepth) != std::string::npos){
            str = str.erase(0,str.length()-2);
            str = str.erase(1);
            printf("Surface Depth\t= %d\n", stoi(str));
            SurfaceDepth = stoi(str);
        } else
        if (str.find(Str_DebugGuidance) != std::string::npos){
            str = str.erase(0,str.length()-2);
            str = str.erase(1);
            printf("Debug Guidance\t= %d\n", stoi(str));
            DebugGuidance = stoi(str);
        }

    }
}

// ALL ABOUT SERIAL ======================================================
// For opening serial port -----------------------------------------------
int openport(char* portname, int baudrate){
    
    struct termios oldtio,newtio;

    int sport = open(portname, O_RDWR | O_NOCTTY | O_SYNC); 

    if (tcgetattr(sport, &oldtio) == -1) {
        cout << "tcgetattr failed" << endl;
        return -1;
    }

    cfmakeraw(&newtio); // Clean all settings
    if (baudrate == 9600){
        newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B9600; // 8 databits   
    }else
    if (baudrate == 115200){
        newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8 | B115200; // 8 databits   
    }
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~(PARENB | PARODD); // No parity
    newtio.c_cflag &= ~CRTSCTS; // No hardware handshake
    newtio.c_cflag &= ~CSTOPB; // 1 stopbit
    newtio.c_iflag = IGNBRK;
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // No software handshake
    newtio.c_lflag = 0;
    newtio.c_oflag = 0;
    newtio.c_lflag |= ICANON; //Enable canonical mode
    newtio.c_cc[VEOL] = '\n';
    newtio.c_iflag &= ~(INLCR | IGNCR | ICRNL);

    tcflush(sport, TCIFLUSH); // Clear IO buffer
    tcflush(sport, TCIOFLUSH); // Clear IO buffer

    if (tcsetattr(sport, TCSANOW, &newtio) == -1) {
        cout << "tcsetattr failed" << endl;
        return -1;
    }

    tcflush(sport, TCIOFLUSH); // Clear IO buffer

    cout << "port open : " << sport << endl;
    if(sport < 0) {
        printf("Error opening %s: \n ",strerror(errno));
        return -1;
    }
    return sport;
}

// Universal for reading from serial port (TS & Sesnsor) -----------------
string ReadSerial(int ports, string head, int line){
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 5000;

    int j = 0;
    string data;

    fd_set readfs;
    FD_ZERO(&readfs);
    FD_SET(ports, &readfs);
    int status = select(ports+1, &readfs, NULL, NULL, &tv);

    if(status == -1){
        perror("status");
        return "gagal";
    }
    else if(status == 0){
        return "timeout";
    }
    else{
        while(j < line){
            char read_buffer[1000];
            memset(&read_buffer,'\0',sizeof(read_buffer));
            int bytes_read = read(ports,&read_buffer,sizeof(read_buffer));
            cout << "\nReceived Serial Data : \r\n" << read_buffer << endl;
            // cout << bytes_read << "bytes" << endl;
            data += read_buffer;
            j += 1;
        }
        size_t found = data.find(head);
        if (found != string::npos){
            data.erase(0,found);
            return data;
        } else
        if(head == "T=" && data.find("\r\n") && data.find("\t")){
            return data;
        }
        else{
            return "gagal";
        }
    }
    tcflush(ports, TCIOFLUSH); // Clear IO buffer
}

// For write serial without count the chars, save your time --------------
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

// READING ANALOG ========================================================
float ReadAnalog(int adc){
    string str;
    ifstream AdcFile("/sys/bus/iio/devices/iio:device0/in_voltage" + std::to_string(adc) + "_raw");
    getline (AdcFile, str);
    AdcFile.close();
    return std::stof(str);
}

// ALL ABOUT COMMAND PROCESSING ==========================================

// Process TS command ----------------------------------------------------
void Com_Mainboard(int _fd, string command){

    if (command.find(TS_BB_PING) != std::string::npos){
        printf("Ping received from TS\r\n");
        if (ProcessRunning == 1){
            State = "001";
        }
        else{
            State = "000";
        }
        Ping = 1;
        FLag = 1;
        command.erase(0,TS_BB_PING.length()+1); //Remove header of command
        ParsePing(command);
    } else
    if (command.find(TS_BB_DATA) != std::string::npos){
        SerWrite(_fd,"#BB> ack DATA\r\n");
        command.erase(0,TS_BB_DATA.length()+1); //Remove header of command
        command.erase(1,2); // Remove /r dan /r char
        if(command == "0"){
            SensorRead = 0;
        } else
        if(command == "1"){
            printf("Read Sensor Start\r\n");
            SensorRead = 1;
            FLag = 1;
        }
    } else
    if (command.find(TS_BB_PARAM) != std::string::npos){
        printf("Param received from TS\r\n");
        SerWrite(_fd,"#BB> ack PARAM\r\n");
        command.erase(0,TS_BB_PARAM.length()+1); //Remove header of command
        ParseParam(command);
    } else
    if (command.find(TS_BB_WAYPOINT) != std::string::npos){
        printf("Waypoint received from TS\n");
        SerWrite(_fd,"#BB> ack WAYPOINT\r\n");
        command.erase(0,TS_BB_WAYPOINT.length()+1); //Remove header of command
        ParseWaypoint(command);
    } else
    if (command.find(TS_BB_OPERATION) != std::string::npos){
        SerWrite(_fd,"#BB> ack OPERATION\r\n");
        command.erase(0,TS_BB_OPERATION.length()+1); //Remove header of command
        command.erase(1,2); // Remove /r dan /r char
        if(command == "0"){
            printf("Operation Stop\r\n");
            ProcessRunning = 0;
        }else
        if(command == "1"){
            printf("Operation Start\r\n");
            ProcessRunning = 1;
            FLag = 1;
        }
    }

}

// Parse TS's PING command -----------------------------------------------
void ParsePing(string command){
    int a = 0;
    string data;
    for (unsigned i=0; i<command.length(); ++i){
        data += command.at(i);
        if (i == 1 || i == 3 || i == 7 || i == 9 || i == 11 || i == 13){
            DateTime[a] = std::stoi(data);
            data.clear();
            a++;
        }
    }
    printf("Date    : %d\r\n", DateTime[0]);
    printf("Month   : %d\r\n", DateTime[1]);
    printf("Year    : %d\r\n", DateTime[2]);
    printf("Hour    : %d\r\n", DateTime[3]);
    printf("Minute  : %d\r\n", DateTime[4]);
    printf("Second  : %d\r\n", DateTime[5]);
}

// Parse TS's PARAM command ----------------------------------------------
void ParseParam(string command){
    std::istringstream _sentence (command);
    char _count = 0;
    do{
        std::string _word;
        _sentence >> _word;
        switch (_count){
            case 0: std::istringstream (_word) >> 
                                         DO; 
                                         break;
            case 1: std::istringstream (_word) >> 
                                         GA; 
                                         break;
            case 2: std::istringstream (_word) >> 
                                         CT; 
                                         break;
            case 3: std::istringstream (_word) >> 
                                         SR; 
                                         break;
            case 4: std::istringstream (_word) >> 
                                         lat_RTB; 
                                         break;
            case 5: std::istringstream (_word) >> 
                                         lon_RTB; 
                                         break;
        }
        _count++;
    }while(_sentence);
    printf("Depth Operation : %d\r\n", DO);
    printf("Glide Angle     : %d\r\n", GA);
    printf("MiniCT Relay    : %d\r\n", CT);
    printf("Sampling Rate   : %d\r\n", SR);
    printf("Lat RTB         : %f\r\n", lat_RTB);
    printf("Lon RTB         : %f\r\n", lon_RTB);
    wpX[0] = lat_RTB;
    wpY[0] = lon_RTB;
}

// Parse TS's WAYPOINT command -------------------------------------------
void ParseWaypoint(string command){
    std::istringstream _sentence (command);
    char _count = 0;
    do{
        std::string _word;
        _sentence >> _word;
        switch (_count){
            case 0: std::istringstream (_word) >> 
                                         lat_start; 
                                         break;
            case 1: std::istringstream (_word) >> 
                                         lon_start; 
                                         break;
            case 2: std::istringstream (_word) >> 
                                         lat_end; 
                                         break;
            case 3: std::istringstream (_word) >> 
                                         lon_end; 
                                         break;
        }
        _count++;
    }while(_sentence);
    printf("Lat Start  : %f\r\n",lat_start);
    printf("Lon Start  : %f\r\n",lon_start);
    printf("Lat End    : %f\r\n",lat_end);
    printf("Lon End    : %f\r\n",lon_end);
    wpX[1] = lat_start;
    wpY[1] = lon_start;
    wpX[2] = lat_end;
    wpY[2] = lon_end;
}

// NAVIGATION ============================================================
void Navigasi(void){
    if(SurfaceMode || Sounder_depth > SurfaceDepth)
    {
        Nav_lat = Latitude;
        Nav_long = Longitude;
        Nav_z = Sounder_depth;
        printf("\nSurfaceMode\n");
    }
    else
    {
    //Input
    double Omega_Pitch_rad = Omega_Pitch * PI / 180.0;
    double Omega_Heading_rad = Omega_Heading * PI / 180.0;
    double Omega_Roll_rad = Omega_Roll * PI / 180.0;
    printf("RollPitchYaw = %f %f %f\n",Omega_Roll_rad, Omega_Pitch_rad, Omega_Heading_rad);
    /*
    double Nav_long=0;
    double Nav_z=0;
    double Nav_zKF=0;
    */
    
    //Transformasi koordinat IMU
    double udot_ned = (cos(Omega_Pitch_rad)*cos(Omega_Heading_rad)*accel_X)+(cos(Omega_Pitch_rad)*sin(Omega_Heading_rad)*accel_Y)+(-1*sin(Omega_Pitch_rad)*accel_Z);
    double vdot_ned = sin(Omega_Roll_rad)*sin(Omega_Pitch_rad)*cos(Omega_Heading_rad)*accel_X + (sin(Omega_Roll_rad)*sin(Omega_Pitch_rad)*sin(Omega_Heading_rad)+cos(Omega_Roll_rad)*cos(Omega_Heading_rad))*accel_Y + sin(Omega_Roll_rad)*cos(Omega_Pitch_rad)*accel_Z;
    double wdot_ned = (cos(Omega_Roll_rad)*sin(Omega_Pitch_rad)*cos(Omega_Heading_rad)+sin(Omega_Roll_rad)*sin(Omega_Heading_rad))*accel_X + (cos(Omega_Roll_rad)*sin(Omega_Pitch_rad)*sin(Omega_Heading_rad)*-1*sin(Omega_Roll_rad)*cos(Omega_Heading_rad))*accel_Y + cos(Omega_Roll_rad)*cos(Omega_Pitch_rad)*accel_Z; 
    
    //Transformasi Koordinat DVL
    double vN_DVL = veloX_F*(cos(Omega_Heading_rad)*cos(Omega_Pitch_rad))+veloY_F*(cos(Omega_Heading_rad)*sin(Omega_Pitch_rad)*sin(Omega_Roll_rad)-sin(Omega_Heading_rad)*cos(Omega_Roll_rad))+veloZ_F*(cos(Omega_Heading_rad)*sin(Omega_Pitch_rad)*cos(Omega_Roll_rad)+sin(Omega_Heading_rad)*sin(Omega_Roll_rad));
    double vE_DVL = veloX_F*(sin(Omega_Heading_rad)*cos(Omega_Pitch_rad)) + veloY_F*(sin(Omega_Heading_rad)*sin(Omega_Pitch_rad)*sin(Omega_Roll_rad)+ cos(Omega_Heading_rad)*cos(Omega_Roll_rad))+ veloZ_F*(sin(Omega_Heading_rad)*sin(Omega_Pitch_rad)*cos(Omega_Roll_rad)*-1*cos(Omega_Heading_rad)*sin(Omega_Roll_rad));
    double vD_DVL = veloX_F*(-1*sin(Omega_Pitch_rad))+veloY_F*(cos(Omega_Pitch_rad)*sin(Omega_Roll_rad))+veloZ_F*(cos(Omega_Pitch_rad)*cos(Omega_Roll_rad));
    
    //printf("VN VE VD DVL %f %f %f\n",vN_DVL,vE_DVL,vD_DVL);
    //Hitung kecepatan IMU
    if(Nav_lat==0)Nav_lat=lat_gps;
    
    
    double omegaE = 7.292115 * 0.00001;
    double R =  6378137;
    
    double aIMU_N = udot_ned-2*omegaE*sin(Nav_lat)*vE_IMU-(vE_IMU*vE_IMU)/R*tan(Nav_lat)+vN_IMU*vD_IMU/R;
    double aIMU_E = vdot_ned-2*omegaE*sin(Nav_lat)*vN_IMU-vE_IMU*vN_IMU/R*tan(Nav_lat)-2*omegaE*cos(Nav_lat)*vD_IMU-vE_IMU*vD_IMU/R;
    double aIMU_D = wdot_ned-2*omegaE*cos(Nav_lat)*vE_IMU-(vE_IMU*vE_IMU)/R-(vN_IMU*vN_IMU)/R;

    //printf("Percepatan IMU N =\t%f\nPercepatan IMU E =\t%f\n",aIMU_N,aIMU_E);
    
    if(resetNavigasi == 1){
        vN_IMU = 0;
        vE_IMU = 0;//kali selang waktu
        vD_IMU = 0;//kali selang waktu
    }
    vN_IMU = vN_IMU + aIMU_N * 1;//kali selang waktu
    vE_IMU = vE_IMU + aIMU_E * 1;//kali selang waktu
    vD_IMU = vD_IMU + aIMU_D * 1;//kali selang waktu

    printf("V DVL N =\t%f\nV DVL E =\t%f\n",vN_DVL,vE_DVL);
    printf("V IMU N =\t%f\nV IMU E =\t%f\n",vN_IMU,vE_IMU);
    
    //Galat Kecepatan
    double dvN = vN_IMU-vN_DVL;
    double dvE = vE_IMU-vE_DVL;
    double dvD = vD_IMU-vD_DVL;
    //printf("dVN dVE dVD DVL %f %f %f\n",dvN,dvE,dvD);
    
    
    //Kalman Filter
        meas << dvN,
                dvE,
                dvD;
        
        if(kalman_reset==0 || resetNavigasi == 1){
            xhat = VectorXd::Zero(5);
            //inisialisasi measurement
            P = MatrixXd::Identity(5,5);
            A << 1, 9.7803, 0, 0, 0,
                -0.00000015679, 1, 0, 0, 0,
                0, 0, 1, 9.7803, 0,
                0, 0, 0.00000015679, 1, 0,
                0, 0, 0, 0, 1;
            Atrans = A.transpose();
            C << 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 0, 1;
            Ctrans = C.transpose();
            matrixR << 0.0303, 0, 0,
                    0, 0.0297, 0,
                    0, 0, 0.0211;
            //matrixR = matrixR.sqrt();
            kalman_reset=1;
        }
        
        xhat = A*xhat;
        //Covariance matrix
        P = A*P*Atrans + Q;
        //Calculate Kalman Gain
        PCtrans = P*Ctrans;
        CPCtransR = C*P*Ctrans + matrixR;
        K = PCtrans * CPCtransR.inverse();
        //calculate the measurement residual
        Resid = meas - C*xhat;
        //Update the state and error covariance estimate
        xhat = xhat + K*Resid;
        //printf("xhat %f %f %f %f %f\n",xhat(0),xhat(1),xhat(2),xhat(3),xhat(4));
        P = (identitysizeK - K*C)*P;

        //output Kalman
        xhatOut = xhat;
        yhatOut = C*xhatOut;
        double vN_kalman = yhatOut(0);
        double vE_kalman = yhatOut(1);
        double vD_kalman = yhatOut(2);
        //printf("vN vE vD kalman %f %f %f\n",vN_kalman,vE_kalman,vD_kalman);
        
    
    //Estimasi Kecepatan
        double vN_est = vN_IMU-vN_kalman;
        double vE_est = vE_IMU-vE_kalman;
        double vD_est = vD_IMU-vD_kalman;
        printf("VNest = %f \t VE_est = %f\n",vN_est,vE_est);
        Nav_kecN = vN_est;
        Nav_kecE = vE_est;
        Nav_kecD = vD_est;
        
    //integrator
    if(resetNavigasi == 1)
    {
        posisiN_aug = 0;
        posisiE_aug = 0;
        posisidepth_aug = 0;
    
    }
        posisiN_aug += vN_est;
        posisiE_aug += vE_est;
        posisidepth_aug += vD_est;
        

    //hasil integrator ditambah posisi awal
        
        Nav_lat = (posisiN_aug/R)*180/PI + lat_gps;
        Nav_long = posisiE_aug/(R*cos(Nav_lat * PI / 180.0))*180/PI + lng_gps;
        Nav_z = posisidepth_aug + depth_gps;
        printf("Latitude Nav=\t%f\nLongitude Nav=\t%f\n",Nav_lat,Nav_long);
    
    //Perhitungan Posisi NED
    
    //Complementary Filter
    Z_KF = Nav_z;
    printf("Sounder %f\n",Sounder_depth);
    if(Z_KF==0.0)
    {
        Z_Complement = Sounder_depth;
        //printf("IF Z_Complement %f\n",Z_Complement);
    
    }
    else{
        
        Z_Complement = (Sounder_depth*1/1.01) + (Z_Complement + Z_KF - Z_KF_m)*(1.01-1/1.01);
        //printf("ELSE Z_Complement %f\n",Z_Complement);
    }
    Z_KF_m = Z_KF;
    
    resetNavigasi = 0;

    }
}

// GUIDANCE LOS ==========================================================
void GuidanceLOS(void){
    double WpXnow,WpYnow;
    //Decide target waypoint, using previous distance
    //Decide wp X and Decide wp Y
    
    // Nanti diubah --------------------------------------------------
    if(distancefromtarget<=acceptance_radius && !guidance_finish){
        
        guidance_finish=1;

    }
    else{

        guidance_finish=0;

    }
    // ---------------------------------------------------------------
    
    if(guidance_finish){
        psiref = 0;
    }
    else{
        WpXnow = wpX[2]; // wpX[1]; //Debug 4Des
        WpYnow = wpY[2]; //wpY[1];//Debug 4Des
        
        //Ambil data posisi: 1st priority navigasi NED 2nd priority IMU
        double Current_positionY = Nav_lat; //wpY[0];Debug 4Des
        double Current_positionX = Nav_long; //wpX[0];//Debug 4Des
        
        //LOS
        psiref =  atan2((WpYnow-Current_positionY),(WpXnow-Current_positionX));
        distancefromtarget = sqrt( pow((WpXnow-Current_positionX),2) + pow((WpYnow-Current_positionY),2));
        psiref = psiref*180.0/PI;
        //transformasi
        if(psiref>90 && psiref<=180){
            psiref = 450 - psiref;
        }
        else{
            psiref = 90 - psiref;
        }
        
        printf("psiref           \t= %f\n",psiref);
        if(DebugGuidance){
            printf("Distance from target \t= %f \t %f \t %f\n", distancefromtarget, WpXnow-Current_positionX, WpYnow-Current_positionY );
            printf("acceptance_radius \t= %f\n",acceptance_radius);
            printf("guidance_wp_target \t= %d\n",guidance_wp_target);
            printf("guidance_finish \t= %d\n",guidance_finish);
        }
    }   
}

// LOGDATA ===============================================================
void LogData(void)
{
    //printf("Reading LogData \n");
    datalog_entrynumber++;
    printf("%d entries \n",datalog_entrynumber);
    
    //string filename = "buffer.txt";
    // string filename2 = "Logger_4Desember.txt";
    // char * filename;
    // sprintf(filename,"BB_LOG_%d-%d-%d.txt", DateTime[0], DateTime[1], DateTime[2]);
    
    stringstream filename;
    filename << "BB_LOG_" << DateTime[0] << DateTime[1] << DateTime[2] << ".txt";
    //ofstream buffer(filename.c_str());  // default mode is ios::out | ios::trunc
    ofstream log( filename.str(), ios::out | ios::app );

    timeval curTime;
    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec / 1000;

    char buffers [80];
    strftime(buffers, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));

    char currentTime[84] = "";
    char datatulis[1000] = "";


    // Data yang ditulis ke logger
    /*
    DVL
    float altitude1_F,altitude2_F,altitude3_F,altitude4_F;
    float veloX_F,veloY_F,veloZ_F,veloFLAG_F;
    float wveloX_F,wveloY_F,wveloZ_F,wveloFLAG_F;
    float altitudeMin_F,
    float Speed
    
    IMU
    double Latitude;
    double Longitude;
    double Height;
    double Roll;
    double Pitch;
    double Heading;
    double accel_X;
    double accel_Y;
    double accel_Z;
    double Omega_Roll;      
    double Omega_Pitch;     
    double Omega_Heading;

    MiniCT
    double CT_conductivity;
    double CT_temperature

    ALti
    double Sounder_depth;
    double Sounder_altitude;
    double Sounder_pressure;
    Navigasi
    Guidance

    */
    //printf("current time: %s \n", currentTime);
    //buffer << currentTime << "\n";
    if (newlogfile==1)
    {
        log<< "Log Data "<< currentTime << "\n";
        printf("Begin data log\n");
        newlogfile = 0;
        sprintf(datatulis, "altitude1, altitude2, altitude3, altitude4, veloX, veloY, veloZ, veloFLAG, wveloX, wveloY, wveloZ, wveloFLAG, altitudeMin, Speed, Latitude, Longitude, Height, Roll, Pitch, Heading, accel_X, accel_Y, accel_Z,Omega_Roll, Omega_Pitch, Omega_Heading,CT_conductivity, CT_temperature,Sounder_depth,Sounder_pressure,Sounder_altitude, Nav_Lat, Nav_Long, Nav_Z,kecepatanX, kecepatanY, kecepatanZ, ZComp, posisiN_aug, posisiE_aug, psiref, guidestat\n");
        sprintf(currentTime, "%s:%d", buffers, milli);
        log << datatulis << ",";
        log << currentTime << "\n";
        //write(ports,"Log in BB\r\n",11);
    }
    
    if(datalog_entrynumber>=datalog_entrylimit)
    {
    //versi simple data dummy
    //sprintf(datatulis, "%5f, %5f, %5f, %5f, %5f", log_a, log_b, log_c, log_d, log_e);

    //versi simple logging (data penting aja yang ditulis)
    //sprintf(datatulis, "%5f, %5f, %5f, %5f, %5f, %5f, %5f, %5f, %5f, %5f", Roll, Pitch, Yaw, Latitude, Longitude, Height, altitude1_F, veloX_F, wveloX_F , altitudeMin_F);

    //versi full data log (semua di tulis, boros memori) DVL IMU MiniCT Alti Navigasi Guidance
    
    sprintf(datatulis, "%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%d", 
    altitude1_F,altitude2_F,altitude3_F,altitude4_F,
    veloX_F,veloY_F,veloZ_F,veloFLAG_F,
    wveloX_F,wveloY_F,wveloZ_F,wveloFLAG_F,
    altitudeMin_F, Speed,
    Latitude, Longitude, Height, Roll, Pitch, Heading, 
    accel_X, accel_Y, accel_Z,
    Omega_Roll, Omega_Pitch, Omega_Heading,
    CT_conductivity, CT_temperature,
    Sounder_depth,Sounder_pressure,Sounder_altitude,
    Nav_lat, Nav_long, Nav_z,
    Nav_kecN, Nav_kecE, Nav_kecD,
    Z_Complement, posisiN_aug, posisiE_aug,
    psiref, guidance_wp_target
    );
    log << datatulis << ",";
    sprintf(currentTime, "%s:%d", buffers, milli);
    log << currentTime << "\n";
    }
    //buffer.close();
    log.close();
}

// SEND TS ===============================================================

// Send status as a reply for the PING command ---------------------------
void SendStatusTS(int ports){
    char PingReply[100];
    long ConditionStatus;
    Condition.replace(13,3,State);
    Condition.replace(12,1,MiniCT_Valid);
    Condition.replace(11,1,Alti_Valid);
    Condition.replace(10,1,DVL_Valid);
    Condition.replace(9,1,IMU_Valid);
    ConditionStatus = strtol(Condition.c_str(),0,2);
    cout << Condition << endl;
    cout << ConditionStatus << endl;
    sprintf(PingReply,"#BB> ack PING %d%d%d%d%d%d %ld\r\n",DateTime[0],
            DateTime[1],DateTime[2],DateTime[3],DateTime[4],
            DateTime[5],ConditionStatus);
    SerWrite(ports,PingReply);
    Condition = "0000000000000000";
}

// Send all sensors data -------------------------------------------------
void SendTS(int ports){
    // $#NG teta_terukur(IMU) z_terukur(Nav) yawref(Guidance) yaw_terukur(IMU) altitude(DVL) conductivity(CT) temperature(CT) speed(DVL) latitude(Nav) longitude(Nav)  roll(IMU) guidestat(guidancewaypointtracking) Propellerspeed(Scenario1) Scenario1_state(Scenario1)
    printf("SendTS\r\n");
    //Mapp variable
    stream_teta << fixed << setprecision(6) << Pitch;         
    stream_z    << fixed << setprecision(6) << Sounder_depth; //Z_Complement //Depth???? 1. Alti Depth 2. Navigasi Z topi?
    /*
    if(State_Scenario_1 == 5)
    {
        psiref=yawref_Scenario1;
    }
    else if(State_Scenario_1>0)
    {
        psiref = 0.0;
    }
    */
    stream_yawr << fixed << setprecision(6) << psiref;
    stream_yawt << fixed << setprecision(6) << Heading;
    stream_posx << fixed << setprecision(6) << posisiE_aug;
    stream_posy << fixed << setprecision(6) << posisiN_aug;
    stream_alti << fixed << setprecision(6) << altitudeMin_F;//Sounder_depth;
    stream_sali << fixed << setprecision(6) << CT_conductivity; //N/A yet
    stream_temp << fixed << setprecision(6) << CT_temperature;//N/A yet
    stream_sped << fixed << setprecision(6) << Speed;
    stream_lati << fixed << setprecision(6) << Nav_lat;//Latitude;
    stream_long << fixed << setprecision(6) << Nav_long;//Longitude;
    stream_roll << fixed << setprecision(6) << Roll;
    stream_volt << fixed << setprecision(6) << Volt;
    stream_crnt << fixed << setprecision(6) << Current;
    stream_leak << fixed << setprecision(6) << Leak;
    stream_fnsh << guidance_finish;

    string_teta = stream_teta.str();
    string_z    =    stream_z.str();
    string_yawr = stream_yawr.str();
    string_yawt = stream_yawt.str();
    string_posx = stream_posx.str();
    string_posy = stream_posy.str();
    string_alti = stream_alti.str();
    //string_dept = stream_dept.str();
    string_sali = stream_sali.str();
    string_temp = stream_temp.str();
    string_sped = stream_sped.str();
    string_lati = stream_lati.str();
    string_long = stream_long.str();
    string_roll = stream_roll.str();
    string_volt = stream_volt.str();
    string_crnt = stream_crnt.str();
    string_leak = stream_leak.str();
    string_fnsh = stream_fnsh.str();

    stream_teta.str("");  
    stream_z.str("");     
    stream_yawr.str("");  
    stream_yawt.str("");  
    stream_posx.str("");  
    stream_posy.str("");  
    stream_alti.str("");  
    //stream_dept.str("");  
    stream_sali.str("");  
    stream_temp.str("");  
    stream_sped.str("");  
    stream_lati.str("");  
    stream_long.str("");  
    stream_roll.str(""); 
    stream_volt.str(""); 
    stream_crnt.str(""); 
    stream_leak.str(""); 
    stream_fnsh.str("");

    stream_teta.clear();
    stream_z.clear();
    stream_yawr.clear();
    stream_yawt.clear();
    stream_posx.clear();
    stream_posy.clear();
    stream_alti.clear();
    //stream_dept.clear();
    stream_sali.clear();
    stream_temp.clear();
    stream_sped.clear();
    stream_lati.clear();
    stream_long.clear();
    stream_roll.clear();
    stream_volt.clear();
    stream_crnt.clear();
    stream_leak.clear();
    stream_fnsh.clear();

    strcpy( char_teta, string_teta.c_str() );
    strcpy( char_z   ,    string_z.c_str() );
    strcpy( char_yawr, string_yawr.c_str() );
    strcpy( char_yawt, string_yawt.c_str() );
    strcpy( char_posx, string_posx.c_str() );
    strcpy( char_posy, string_posy.c_str() );
    strcpy( char_alti, string_alti.c_str() );
    //strcpy( char_dept, string_dept.c_str() );
    strcpy( char_sali, string_sali.c_str() );
    strcpy( char_temp, string_temp.c_str() );
    strcpy( char_sped, string_sped.c_str() );
    strcpy( char_lati, string_lati.c_str() );
    strcpy( char_long, string_long.c_str() );
    strcpy( char_roll, string_roll.c_str() );
    strcpy( char_volt, string_volt.c_str() );
    strcpy( char_crnt, string_crnt.c_str() );
    strcpy( char_leak, string_leak.c_str() );
    strcpy( char_fnsh, string_fnsh.c_str() );
    
    SerWrite(ports,"#BB>");
    SerWrite(ports," ");
    SerWrite(ports,char_teta);
    SerWrite(ports," ");
    SerWrite(ports,char_z);
    SerWrite(ports," ");
    SerWrite(ports,char_yawr);
    SerWrite(ports," ");
    SerWrite(ports,char_yawt);
    SerWrite(ports," ");
    SerWrite(ports,char_posx);
    SerWrite(ports," ");
    SerWrite(ports,char_posy);
    SerWrite(ports," ");
    SerWrite(ports,char_alti);
    SerWrite(ports," ");
    //write(ports,char_dept,9);
    //write(ports," ",1);
    SerWrite(ports,char_sali);
    SerWrite(ports," ");
    SerWrite(ports,char_temp);
    SerWrite(ports," ");
    SerWrite(ports,char_sped);
    SerWrite(ports," ");
    SerWrite(ports,char_lati);
    SerWrite(ports," ");
    SerWrite(ports,char_long);
    SerWrite(ports," ");
    SerWrite(ports,char_roll);
    SerWrite(ports," ");
    SerWrite(ports,char_volt);
    SerWrite(ports," ");
    SerWrite(ports,char_crnt);
    SerWrite(ports," ");
    SerWrite(ports,char_leak);
    SerWrite(ports," ");
    SerWrite(ports,char_fnsh);
    /*
    write(ports," ",1);
    if(guidance_wp_target==0)
    {
        write(ports,"0 ",2);
    }
    else if(guidance_wp_target==1)
    {
        write(ports,"1 ",2);
    }
    else if(guidance_wp_target==2)
    {
        write(ports,"2 ",2);
    }
    else if(guidance_wp_target==3)
    {
        write(ports,"3 ",2);
    }
    //V propeller
    if(V_propeller==0)
    {
        write(ports,"0",1);
    }
    else if(V_propeller==1)
    {
        write(ports,"25",2);
    }
    else if(V_propeller==2)
    {
        write(ports,"50",2);
    }
    else if(V_propeller==3)
    {
        write(ports,"75",2);
    }
    else if(V_propeller==4)
    {
        write(ports,"100",3);
    }
    write(ports," ",1);
    //Scenario Status
    if(State_Scenario_1==0)
    {
        write(ports,"0",1);
    }
    else if(State_Scenario_1==1)
    {
        write(ports,"1",1);
    }
    else if(State_Scenario_1==2)
    {
        write(ports,"2",1);
    }
    else if(State_Scenario_1==3)
    {
        write(ports,"3",1);
    }
    else if(State_Scenario_1==4)
    {
        write(ports,"4",1);
    }
    else if(State_Scenario_1==5)
    {
        write(ports,"5",1);
    }
    */
    SerWrite(ports,"\r\n");
    tcdrain(ports);
    /*
    write(ports,"$#NG",4);
    write(ports," ",1);
    write(ports,char_teta,9);
    write(ports," Alti ",6);
    write(ports,char_z   ,9);
    write(ports," \r\n",3);
    write(ports,char_yawr,9);
    write(ports," ",1);
    write(ports,char_yawt,9);
    write(ports," ",1);
    //write(ports,char_posx,9);
    //write(ports," ",1);
    //write(ports,char_posy,9);
    //write(ports," ",1);
    write(ports,char_alti,9);
    write(ports," \r\nCT ",6);
    //write(ports,char_dept,9);
    //write(ports," ",1);
    write(ports,char_sali,9);
    write(ports," ",1);
    write(ports,char_temp,9);
    write(ports," \r\nDVL ",7);
    write(ports,char_sped,9);
    write(ports," \r\n",3);
    write(ports,char_lati,12);
    write(ports," ",1);
    write(ports,char_long,12);
    write(ports," \r\n",3);
    write(ports,char_roll,9);
    write(ports," ",1);
    if(guidance_finish==1)
    {
        write(ports,"1 ",2);
    }
    else
    {
        write(ports,"0 ",2);
    }
    write(ports,"\r\n",2);
    tcdrain(ports);
    */
    
}

void rt_OneStep(void)
{
  static boolean_T OverrunFlag = false;

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    rtmSetErrorStatus(rtObj.getRTM(), "Overrun");
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here

  // Step the model for base rate
  rtObj.step();

  // Get model outputs here

  // Indicate task complete
  OverrunFlag = false;

  // Disable interrupts here
  // Restore FPU context here (if necessary)
  // Enable interrupts here
}

void Scenario_1(void){
    printf("Scenario 1\n");
    
    //State_Scenario 1
    // 0 skenario non aktif
    // 1 start sampai 50 meter
    // 2 gliding sampai pitch down
    // 3 pitch down sampai pitch up ke permukaan
    // 4 50 meter ke akhir
    // 5 selesai skenario
    
    //State 1
    //ambil koordinat home, ambil koordinat posisi wahana dari GPS, hitung jarak, hitung mapping.
    // Jika jarak >50m ke state 2
    
    //State 2 
    //tunggu sampai depthnya cukup dalam, pindah state 3
    
    //State 3
    //tunggu sampai depthnya cukup dangkal, pindah state 4 dengan mengambil gps
    
    //State 4
    // ambil koordinat posisi wahana dari GPS, hitung jarak terhadap GPS yang diambil di state 3, mapping
    // Jika jarak >50m motor 0, kasih flag 5 ke TS, ambil koordinat gps, Glider muter 180, 
    
    //State 5
    
    
    //Output
    /*
        1. Sequence scenario 1
        2.  kecepatan propeller
    */
    double current_Lat, current_Long;
    double distance;
    static double surfacepoint_Lat, surfacepoint_Long;
    static int returnSequence=0;
    
    if(State_Scenario_1 == 1)
    {
    //ambil koordinat home, setting dengan $#G1 lat long, atau ketika pertama di air
    
    //ambil koordinat posisi wahana dari GPS,
    current_Lat= Latitude;
    current_Long= Longitude;
    
    //hitung jarak, hitung mapping.(
    if (returnSequence==0)
    {
    distance = 1000*distance_two_coordinates(wpY[0],wpX[0],current_Lat,current_Long);
    }
    else
    {
    distance = 1000*distance_two_coordinates(surfacepoint_Lat,surfacepoint_Long,current_Lat,current_Long);
    }
    if (distance<13)
    {
        V_propeller=4;
    }
    else if (distance<25)
    {
        V_propeller=3;
    }
    else if (distance<38)
    {
        V_propeller=2;
    }
    else if (distance<50)
    {
        V_propeller=1;
    }
    else
    {
        State_Scenario_1 = 2;
    }
    
    // Jika jarak >50m ke state 2
    
    
    }
    
    //State 2 
    //tunggu sampai depthnya cukup dalam, pindah state 3
    else if(State_Scenario_1 == 2)
    {
        if(Sounder_depth>12)
        {
            State_Scenario_1 = 3;
        }
    }
    //State 3
    //tunggu sampai depthnya cukup dangkal, pindah state 4 dengan mengambil gps
    else if(State_Scenario_1 == 3)
    {   
        if(Sounder_depth<10)
        {
            State_Scenario_1 = 4;
            surfacepoint_Lat = Latitude; 
            surfacepoint_Long = Longitude;
        }
    }
    else if(State_Scenario_1 == 4)
    {
        //ambil koordinat posisi wahana dari GPS,
        current_Lat= Latitude;
        current_Long= Longitude;
        
        //hitung jarak, hitung mapping.
        distance = 1000*distance_two_coordinates(surfacepoint_Lat,surfacepoint_Long,current_Lat,current_Long);
        if (distance<13)
        {
            V_propeller=4;
        }
        else if (distance<25)
        {
            V_propeller=3;
        }
        else if (distance<38)
        {
            V_propeller=2;
        }
        else if (distance<50)
        {
            V_propeller=1;
        }
        else
        {
            State_Scenario_1 = 5;
            surfacepoint_Lat = Latitude; 
            surfacepoint_Long = Longitude;
            yawref_Scenario1 = Heading-180.0;
            if(yawref_Scenario1<0)yawref_Scenario1+=360;
        }
    
    }
    else if(State_Scenario_1 == 5)
    {
        //Kalau nunggu command baru sequence balik di comment aja
        if ((yawref_Scenario1-Heading)<5 || (Heading-yawref_Scenario1)<5)
        {
            State_Scenario_1 = 1;
        }
        
        
    }
}

double distance_two_coordinates(double lat1,double lon1,double lat2,double lon2){  // generally used geo measurement function
    double R = 6378.137; // Radius of earth in KM
    double dLat = (lat2 * PI/ 180.0) - (lat1 *PI / 180.0);
    double dLon = (lon2 * PI / 180.0) - (lon1 * PI / 180.0);
    double a = sin(dLat/2.0) * sin(dLat/2.0) +
    cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
    sin(dLon/2.0) * sin(dLon/2.0);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = R * c;
    return d;//Kilometers
}

char * ParseAlti(string Parse_Alti){
    int i = 1;
    char * pch;
    char * data = new char[50];
    strcpy(data,Parse_Alti.c_str());
    pch = strtok (data,",");

    while (pch != NULL){
        pch = strtok(NULL,",");    
        if(i == 1){
            strncpy(c_Sounder_altitude, pch, sizeof c_Sounder_altitude - 1); 
            Sounder_altitude = atof (c_Sounder_altitude);
            printf ("AltiSounder altitude %f\n",Sounder_altitude);
        }
        else if (i == 3){
            strncpy(c_Sounder_pressure, pch, sizeof c_Sounder_pressure - 1);
            Sounder_depth = atof (c_Sounder_pressure);
            printf ("AltiSounder depth %f\n\n",Sounder_depth);
        }
        i += 1;
    }
}

char * ParseMiniCT(string Parse_MiniCT){
    int i = 1;
    char * pch;
    char * data = new char[50];
    strcpy(data,Parse_MiniCT.c_str());
    pch = strtok (data,"\t");

    while (pch != NULL){
        if(i == 1){
            strncpy(c_CT_temperature, pch, sizeof c_CT_temperature - 1);
            CT_temperature = atof (c_CT_temperature);
            printf ("CT_temperature %f\n",CT_temperature);
        }
        else if (i == 2){
            strncpy(c_CT_conductivity, pch, sizeof c_CT_conductivity - 1);
            CT_conductivity = atof (c_CT_conductivity);
            printf ("CT_conductivity %f\n\n",CT_conductivity);
        }
        pch = strtok(NULL,"\t");    
        i += 1;
    }
}

char * ParseDVL(string Parse_DVL){
    int datacount = 1;
    char * pch;
    char * data = new char[600];
    strcpy(data,Parse_DVL.c_str());
    pch = strtok (data," ");
    
    while (pch != NULL)
    {
        switch ( datacount ) 
        {
        case 1 : 
            //strcpy(header, pch); 
            strncpy(header, pch, sizeof header - 1); 
            //printf("%s\t",header);
            datacount++;
            break;
        case 2 : 
            //strcpy(error_codes, pch); 
            strncpy(error_codes, pch, sizeof error_codes - 1); 
            datacount++;
            //Vbd = atof (vbd);
            //printf("%s\n",error_codes);
            //printf ("float Dm1 %f\n",Dm1);
            break;
        case 3 : 
            //strcpy(error_codes, pch); 
            strncpy(beamresult1, pch, sizeof beamresult1 - 1); 
            datacount++;
            //Vbd = atof (vbd);
            printf("%s\t",beamresult1);
            //printf ("float Dm1 %f\n",Dm1);
            break;
        case 4 : 
            //strcpy(error_codes, pch); 
            strncpy(beamresult2, pch, sizeof beamresult2 - 1); 
            datacount++;
            //Vbd = atof (vbd);
            printf("%s\t",beamresult2);
            //printf ("float Dm1 %f\n",Dm1);
            break;
        case 5 : 
            //strcpy(error_codes, pch); 
            strncpy(beamresult3, pch, sizeof beamresult3 - 1); 
            datacount++;
            //Vbd = atof (vbd);
            printf("%s\t",beamresult3);
            //printf ("float Dm1 %f\n",Dm1);
            break;
        case 6 : 
            //strcpy(error_codes, pch); 
            strncpy(beamresult4, pch, sizeof beamresult4 - 1); 
            datacount++;
            //Vbd = atof (vbd);
            printf("%s\n",beamresult4);
            //printf ("float Dm1 %f\n",Dm1);
            break;
        case 7 : 
            //strcpy(error_codes, pch); 
            strncpy(altitude1, pch, sizeof altitude1 - 1); 
            datacount++;
            altitude1_F = atof (altitude1);
            // printf ("float altitude 1 %f\t",altitude1_F);
            break;
        case 8 : 
            //strcpy(error_codes, pch); 
            strncpy(altitude2, pch, sizeof altitude2 - 1); 
            datacount++;
            altitude2_F = atof (altitude2);
            // printf ("altitude 2 %f\t",altitude2_F);
            break;
        case 9 : 
            //strcpy(error_codes, pch); 
            strncpy(altitude3, pch, sizeof altitude3 - 1); 
            datacount++;
            altitude3_F = atof (altitude3);
            // printf ("faltitude 3 %f\t",altitude3_F);
            break;
        case 10 : 
            //strcpy(error_codes, pch); 
            strncpy(altitude4, pch, sizeof altitude4 - 1); 
            datacount++;
            altitude4_F = atof (altitude4);
            // printf ("altitude 4 %f\n",altitude4_F);
            break;
        case 11 : 
            //strcpy(error_codes, pch); 
            strncpy(velo_rad1, pch, sizeof velo_rad1 - 1); 
            datacount++;
            velo_rad1_F = atof (velo_rad1);
            // printf ("float velo_rad1 %f\t",velo_rad1_F);
            break;
        case 12 : 
            //strcpy(error_codes, pch); 
            strncpy(velo_rad2, pch, sizeof velo_rad2 - 1); 
            datacount++;
            velo_rad2_F = atof (velo_rad2);
            // printf ("velo_rad2 %f\t",velo_rad2_F);
            break;
        case 13 : 
            //strcpy(error_codes, pch); 
            strncpy(velo_rad3, pch, sizeof velo_rad3 - 1); 
            datacount++;
            velo_rad3_F = atof (velo_rad3);
            // printf ("velo_rad3 %f\t",velo_rad3_F);
            break;
        case 14 : 
            //strcpy(error_codes, pch); 
            strncpy(velo_rad4, pch, sizeof velo_rad4 - 1); 
            datacount++;
            velo_rad4_F = atof (velo_rad4);
            // printf ("velo_rad4 %f\n",velo_rad4_F);
            break;
        case 15 : 
            //strcpy(error_codes, pch); 
            strncpy(wvelo_rad1, pch, sizeof wvelo_rad1 - 1); 
            datacount++;
            wvelo_rad1_F = atof (wvelo_rad1);
            // printf ("float wvelo_rad1 %f\t",wvelo_rad1_F);
            break;
        case 16 : 
            //strcpy(error_codes, pch); 
            strncpy(wvelo_rad2, pch, sizeof wvelo_rad2 - 1); 
            datacount++;
            wvelo_rad2_F = atof (wvelo_rad2);
            // printf ("wvelo_rad2 %f\t",wvelo_rad2_F);
            break;
        case 17 : 
            //strcpy(error_codes, pch); 
            strncpy(wvelo_rad3, pch, sizeof wvelo_rad3 - 1); 
            datacount++;
            wvelo_rad3_F = atof (wvelo_rad3);
            // printf ("wvelo_rad3 %f\t",wvelo_rad3_F);
            break;
        case 18 : 
            //strcpy(error_codes, pch); 
            strncpy(wvelo_rad4, pch, sizeof wvelo_rad4 - 1); 
            datacount++;
            wvelo_rad4_F = atof (wvelo_rad4);
            // printf ("wvelo_rad4 %f\n",wvelo_rad4_F);
            break;
        case 19 : 
            //strcpy(error_codes, pch); 
            strncpy(cwvelo_rad1, pch, sizeof cwvelo_rad1 - 1); 
            datacount++;
            cwvelo_rad1_F = atof (cwvelo_rad1);
            // printf ("float cwvelo_rad1 %f\t",cwvelo_rad1_F);
            break;
        case 20 : 
            //strcpy(error_codes, pch); 
            strncpy(cwvelo_rad2, pch, sizeof cwvelo_rad2 - 1); 
            datacount++;
            cwvelo_rad2_F = atof (cwvelo_rad2);
            // printf ("cwvelo_rad2 %f\t",cwvelo_rad2_F);
            break;
        case 21 : 
            //strcpy(error_codes, pch); 
            strncpy(cwvelo_rad3, pch, sizeof cwvelo_rad3 - 1); 
            datacount++;
            cwvelo_rad3_F = atof (cwvelo_rad3);
            // printf ("cwvelo_rad3 %f\t",cwvelo_rad3_F);
            break;
        case 22 : 
            //strcpy(error_codes, pch); 
            strncpy(cwvelo_rad4, pch, sizeof cwvelo_rad4 - 1); 
            datacount++;
            cwvelo_rad4_F = atof (cwvelo_rad4);
            // printf ("cwvelo_rad4 %f\n",cwvelo_rad4_F);
            break;
        case 23 : 
            //strcpy(error_codes, pch); 
            strncpy(veloX, pch, sizeof veloX - 1); 
            datacount++;
            veloX_F = atof (veloX);
            // printf ("veloX %f\t",veloX_F);
            break;
        case 24 : 
            //strcpy(error_codes, pch); 
            strncpy(veloY, pch, sizeof veloY - 1); 
            datacount++;
            veloY_F = atof (veloY);
            // printf ("veloY %f\t",veloY_F);
            break;
        case 25 : 
            //strcpy(error_codes, pch); 
            strncpy(veloZ, pch, sizeof veloZ - 1); 
            datacount++;
            veloZ_F = atof (veloZ);
            // printf ("veloZ %f\t",veloZ_F);
            break;
        case 26 : 
            //strcpy(error_codes, pch); 
            strncpy(veloFLAG, pch, sizeof veloFLAG - 1); 
            datacount++;
            veloFLAG_F = atof (veloFLAG);
            // printf ("veloFLAG %f\n",veloFLAG_F);
            break;
        case 27 :
            datacount++;
            break;
        case 28 :
            datacount++;
            break;
        case 29 :
            datacount++;
            break;
        case 30 :
            datacount++;
            break;
        case 31 : 
            //strcpy(error_codes, pch); 
            strncpy(wveloX, pch, sizeof wveloX - 1); 
            datacount++;
            wveloX_F = atof (wveloX);
            //printf ("wveloX %f\t",wveloX_F);
            break;
        case 32 : 
            //strcpy(error_codes, pch); 
            strncpy(wveloY, pch, sizeof wveloY - 1); 
            datacount++;
            wveloY_F = atof (wveloY);
            //printf ("wveloY %f\t",wveloY_F);
            break;
        case 33 : 
            //strcpy(error_codes, pch); 
            strncpy(wveloZ, pch, sizeof wveloZ - 1); 
            datacount++;
            wveloZ_F = atof (wveloZ);
            //printf ("wveloZ %f\t",wveloZ_F);
            break;
        case 34 : 
            //strcpy(error_codes, pch); 
            strncpy(wveloFLAG, pch, sizeof wveloFLAG - 1); 
            datacount++;
            wveloFLAG_F = atof (wveloFLAG);
            //printf ("wveloFLAG %f\n",wveloFLAG_F);
            break;
        case 35 :
            datacount++;
            break;
        case 36 :
            datacount++;
            break;
        case 37 :
            datacount++;
            break;
        case 38 :
            datacount++;
            break;
        case 39 : //roll pitch yaw n/a
            datacount++;
            break;
        case 40 :
            datacount++;
            break;
        case 41 :
            datacount++;
            break;
        case 42 :
            strncpy(altitudeMin, pch, sizeof altitudeMin - 1); 
            datacount++;
            altitudeMin_F = atof (altitudeMin);
            // printf ("float altitude Min %f\n",altitudeMin_F);
            break;
        case 43 :
            strncpy(Temperature, pch, sizeof Temperature - 1); 
            datacount++;
            Temperature_F = atof (Temperature);
            //printf ("float Temperature %f\t",Temperature_F);
            break;
        case 44 : //Press n/a
            datacount++;
            break;  
        case 45 :
            strncpy(salinity, pch, sizeof salinity - 1); 
            datacount++;
            salinity_F = atof (salinity);
            //printf ("User defined salinity %f\t",salinity_F);
            break;  
        case 46 :
            strncpy(soundspeed, pch, sizeof soundspeed - 1); 
            datacount++;
            soundspeed_F = atof (soundspeed);
            //printf ("User defined soundspeed %f\t",soundspeed_F);
            break;
        case 47 :
            strncpy(ceksum, pch, sizeof ceksum - 1); 
            datacount++;
            ceksum_F = atof (ceksum);
            //printf ("User defined ceksum %f\t",ceksum_F);
            
            Speed = sqrt((veloX_F*veloX_F)+(veloY_F*veloY_F)+(veloZ_F*veloZ_F));
            break;
         default : 
            // Process for all other cases.
            //printf ("Default/n");
            break;
        }
      pch = strtok (NULL, " ");
    }
}

int TriggerDVL(){
    DVL_GPIO.Handler = fopen(DVL_GPIO.Value,"rb+");
    if (DVL_GPIO.Handler == NULL){
        cout << "error set DVL trigger pin value" << endl;
        return 1;
    }

    strcpy(DVL_GPIO.Set,"1");
    fwrite(&DVL_GPIO.Set, sizeof(char), 1, DVL_GPIO.Handler);
    fclose(DVL_GPIO.Handler);

    usleep(300000);

    DVL_GPIO.Handler = fopen(DVL_GPIO.Value,"rb+");
    if (DVL_GPIO.Handler == NULL){
        cout << "error set DVL trigger pin value" << endl;
        return 1;
    }

    strcpy(DVL_GPIO.Set,"0");
    fwrite(&DVL_GPIO.Set, sizeof(char), 1, DVL_GPIO.Handler);
    fclose(DVL_GPIO.Handler);

    cout << "DVL triggered" << endl;
}

int Relay_On(){
    Relay_GPIO.Handler = fopen(Relay_GPIO.Value,"rb+");
    if (Relay_GPIO.Handler == NULL){
        cout << "error set Relay trigger pin value" << endl;
        return 1;
    }

    strcpy(Relay_GPIO.Set,"1");
    fwrite(&Relay_GPIO.Set, sizeof(char), 1, Relay_GPIO.Handler);
    fclose(Relay_GPIO.Handler);
    return 0;
}

int Relay_Off(){
    Relay_GPIO.Handler = fopen(Relay_GPIO.Value,"rb+");
    if (Relay_GPIO.Handler == NULL){
        cout << "error set Relay trigger pin value" << endl;
        return 1;
    }

    strcpy(Relay_GPIO.Set,"0");
    fwrite(&Relay_GPIO.Set, sizeof(char), 1, Relay_GPIO.Handler);
    fclose(Relay_GPIO.Handler);
    return 0;
}

void process_mem_usage(double& vm_usage, double& resident_set){
    vm_usage     = 0.0;
    resident_set = 0.0;

    // the two fields we want
    unsigned long vsize;
    long rss;
    {
        std::string ignore;
        std::ifstream ifs("/proc/self/stat", std::ios_base::in);
        ifs >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore
                >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore
                >> ignore >> ignore >> vsize >> rss;
    }

    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
    vm_usage = vsize / 1024.0;
    resident_set = rss * page_size_kb;
}
