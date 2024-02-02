#ifndef _libNavGdn
#define _libNavGdn

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/MatrixFunctions"

//#include <moduleGlider/libSensor.hpp>
#include <math.h>

struct  Navigation
{
    double latitude;
    double longitude;
    double z;

    double vE_IMU;
    double vN_IMU;
    double vD_IMU;

    double kecN;
    double kecE;
    double kecD;

    double N_aug;
    double E_aug;
    double Depth_aug;

    int reset;
    int kalmanReset;

//Matrix Navigasi =======================================================
//declare eigen::Matrix<datatype, rows, columns> name
    Eigen::Matrix<double, 3, 1> meas;
    Eigen::Matrix<double, 5, 1> xhat;
    Eigen::Matrix<double, 5, 5> P;
    Eigen::Matrix<double, 3, 1> Resid;
    Eigen::Matrix<double, 3, 1> yhatOut;
    Eigen::Matrix<double, 5, 1> xhatOut;
    Eigen::Matrix<double, 5, 5> A;
    Eigen::Matrix<double, 5, 5> Atrans;
    Eigen::Matrix<double, 3, 5> C;
    Eigen::Matrix<double, 3, 3> matrixR;
    Eigen::Matrix<double, 5, 3> K;
    Eigen::Matrix<double, 5, 3> Ctrans;
    Eigen::Matrix<double, 5, 3> PCtrans;
    Eigen::Matrix<double, 3, 3> CPCtransR;
//Matrix Navigasi =======================================================

    void Init();
    void Reset();
    void Print();
};

Navigation calculateNavigation(Navigation _nav, Sensor sensor);

struct Guidance
{
    double distanceFromTarget;
    double psiRef;
    bool isFinished;
    double acceptanceRadius;
    double targetLat;
    double targetLon;
    double sourceLat;
    double sourceLon;

    void Init();
    void Print();
    void Reset();
};

Guidance calculateGuidance(Guidance _temp, double wpXNow, double wpYNow, double wpXTarget, double wpYTarget);
#endif


