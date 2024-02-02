#include <moduleGlider/libSensor.hpp>

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <vector>

// char sensorIMU       = 1;
// char sensorDVL       = 2;
// char sensorALTI      = 4;
// char sensorMINICT    = 8;
// char sensorAnalog    =16;
// char sensorALL       =31;

void Sensor::_IMU::Print()
{
    std::cout<<"\tIMU>>>>>"<<std::endl;
    std::cout<<"\t" << omegaPitch<<std::endl;
    std::cout<<"\t" << omegaHeading<<std::endl;
    std::cout<<"\t" << omegaRoll<<std::endl;
    std::cout<<"\t" << accel_X<<std::endl;
    std::cout<<"\t" << accel_Y<<std::endl;
    std::cout<<"\t" << accel_Z<<std::endl;
    std::cout<<"\t" << latitude<<std::endl;
    std::cout<<"\t" << longitude<<std::endl;
    std::cout<<"\t" << Status<<std::endl;
    std::cout<<"\t<<<<<IMU"<<std::endl;

}
void Sensor::_DVL::Print()
{
    std::cout<<"\tDVL>>>>>"<<std::endl;
    std::cout<<"\t" << veloX_F << std:: endl;
    std::cout<<"\t" << veloY_F << std:: endl;
    std::cout<<"\t" << veloZ_F << std:: endl;
    std::cout<<"\t<<<<<DVL"<<std::endl;
}
void Sensor::_ALTI::Print()
{
    std::cout<<"\tALTI>>>>>"<<std::endl;
    std::cout<<"\t<<<<<ALTI"<<std::endl;    
}
void Sensor::_MINICT::Print()
{
    std::cout<<"\tMINICT>>>>>"<<std::endl;
    std::cout<<"\t<<<<<MINICT"<<std::endl;        
}
void Sensor::_analog::Print()
{
    std::cout<<"\tAnalog>>>>>"<<std::endl;
    std::cout<<"\t" << volt << std:: endl;
    std::cout<<"\t" << current << std:: endl;
    std::cout<<"\t" << leak << std:: endl;
    std::cout<<"\t<<<<<Analog"<<std::endl;
}

void Sensor::Print(sensorIndex par)
{
    if (par&sensorIMU)
        IMU.Print();
    if (par&sensorDVL)
        DVL.Print();
    if (par&sensorALTI)
        ALTI.Print();
    if (par&sensorMINICT)
        MINICT.Print();
    if (par&sensorAnalog)
        analog.Print();
}

void Sensor::Reset()
{

}

void Sensor::Parse(std::string s)
{
    //determine length
    std::stringstream _sentence(s);
    std::string _length;
    _sentence >> _length;
    int length = stoi(_length);

    std::vector<float> data;
    data.clear();
    //sanity check
    do{
        std::string _word;
        _sentence >> _word;
        if (_word != "")
        {
            float temp;
            std::istringstream (_word) >> std::fixed >> std::setprecision(12) >> temp;
            data.push_back(temp);
        }
    }while(_sentence);
    int datalength = data.size();
    
    //data is valid
    if (datalength == length)
    {
        //primary sensor
        if (length >= 3)
        {
            analog.volt = data[0];
            analog.current = data[1];
            analog.leak = data[2];
            //operational sensor
            if (length >= 15)
            {
                IMU.omegaPitch = data[3];
                IMU.omegaHeading = data[4];
                IMU.omegaRoll = data[5];
                IMU.accel_X = data[6];
                IMU.accel_Y = data[7];
                IMU.accel_Z = data[8];
                IMU.latitude = data[9];
                IMU.longitude = data[10];
		IMU.Status = data[11];

                DVL.veloX_F = data[12];
                DVL.veloY_F = data[13];
                DVL.veloZ_F = data[14];

                ALTI.depth = data[15];
            
                //payload
                if (length >= 18)
                {
                    MINICT.temperature = data[16];
                    MINICT.conductivity = data[17];
                }
            }       
        }

    } else
    {
        //data is not valid based on args count
    }
}
