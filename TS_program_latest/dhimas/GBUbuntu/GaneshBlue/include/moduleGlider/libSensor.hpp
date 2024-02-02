#ifndef _libSensor
#define _libSensor

#include <string>
enum sensorIndex
{
    sensorIMU = 1,
    sensorDVL = 2,
    sensorALTI = 4,
    sensorMINICT = 8,
    sensorAnalog = 16,
    sensorALL = 31
};

inline sensorIndex operator|(sensorIndex a, sensorIndex b)
{
    return static_cast<sensorIndex>(static_cast<int>(a) | static_cast<int>(b));
}

struct Sensor
{
//nested struct
    struct _IMU
    {
        double omegaPitch;
        double omegaHeading;
        double omegaRoll;
        double accel_X;
        double accel_Y;
        double accel_Z;
        double latitude;
        double longitude;
	    int Status;
        double posCov;
        void Print();
    };
    struct _DVL
    {
        float veloX_F;
        float veloY_F;
        float veloZ_F;
        
        void Print();
    };
    struct _ALTI
    {
        float depth;
        void Print();
    };
    struct _MINICT
    {
        //payload
        double conductivity;
        double temperature;
        void Print();
    };
    struct _analog
    {
        float volt;
        float current;
        float leak;
        void Print();
    };

    _IMU IMU;
    _DVL DVL;
    _ALTI ALTI;
    _MINICT MINICT;
    _analog analog;

    void Print(sensorIndex par);
    void Reset();
    void Parse(std::string s);

};

#endif

/*notes
    nilai sensor untuk navigasi
    
    IMU
    {
        double omegaPitch;
        double omegaHeading;
        double omegaRoll;
        double accel_X;
        double accel_Y;
        double accel_Z;
        double latitude;
        double longitude;
    }
    DVL
    {
        float veloX_F;
        float veloY_F;
        float veloZ_F;
    }
    Alti
    {
        float depth;
    }
*/
