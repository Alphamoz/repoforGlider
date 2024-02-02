#include <moduleControl/NavGdn.hpp>
#include <chrono>
#include <iostream>
using namespace std;
/*
    //call example :
    //declare struct
    Navigation navigation;
    Sensor sensor;
    ...
    ...
    ...
    //calculate navigation
    navigation = calculateNavigation(navigation, sensor);
    //output
    cout << navigation.latitude << " " <<navigation.longitude << endl;
*/
double alpha = 0.075; //tambahan low pass
double threshold_pos_cov = 10; //tambahan low pass nanti dari sensor GPS
double pos_cov_Current;
double PI = 3.1415926536;
double R = 6378.137; //radius bumi dalam km
double R_m = (1/((2*PI/360)*R))/1000; // 1 meter in degree 
int pengali_lat = 119580;//kolam 110520;//119580 //karimun
int pengali_lon = 110740;//kolam 111600;//110740 //karimun

double g = 9.8335;
double init_Latitude[] = {-5.867675};//{-6.886641521}; // inisial pertama dari lat
double init_Longitude[] = {110.430031};//{107.6105549}; // inisial pertama dari long
int Cont = 0;
int GPSintFlag = 0;
int curStatus;
int lastStatus;
double lastgpsnorth;
double lastgpseast;
// chrono::steady_clock::time_point timeLast, timeCurrent;
// bool first = true;

Navigation calculateNavigation(Navigation nav, Sensor sensor)
{
    
    // Navigation nav = _nav;
    float timeElapsed;
    nav.timeCurrent = chrono::steady_clock::now();
    cout << "Cont : " << Cont << endl;
    if (Cont == 0)
    {
        // nav.first = false;
        Cont = 1;
        GPSintFlag = 0;
        timeElapsed = 0.0;
        nav.vX_DVL = 0; //inisial awal kecepatan DVL
        nav.vY_DVL = 0;
        nav.vZ_DVL = 0;
    }
    else
    {
        timeElapsed = float(chrono::duration_cast<chrono::microseconds>(nav.timeCurrent - nav.timeLast).count()) / 1000000;
    }
    nav.timeLast = nav.timeCurrent;
    cout << "Time Elapsed Navigation: " << timeElapsed << endl;
    nav.Status_GPS = sensor.IMU.Status;
    curStatus = nav.Status_GPS;
    nav.Depth_aug = sensor.ALTI.depth;
    pos_cov_Current = sensor.IMU.posCov;

    nav.Heading_IMU = sensor.IMU.omegaHeading * (PI / 180.0);
    double yaw_rad = nav.Heading_IMU;
    //nav.vX_DVL = sensor.DVL.veloX_F;
    //nav.vY_DVL = sensor.DVL.veloY_F;
    //nav.vZ_DVL = sensor.DVL.veloZ_F;

    double Omega_Pitch_rad = sensor.IMU.omegaPitch*(PI/180.0);
    double Omega_Heading_rad = sensor.IMU.omegaHeading*(PI/180.0);
    double Omega_Roll_rad = sensor.IMU.omegaRoll*(PI/180.0);
    //dengan low pass
    nav.vX_DVL = alpha*-sensor.DVL.veloX_F + (1-alpha)*nav.vX_DVL;
    nav.vY_DVL = alpha*-sensor.DVL.veloY_F + (1-alpha)*nav.vY_DVL;
    nav.vZ_DVL = alpha*sensor.DVL.veloZ_F + (1-alpha)*nav.vZ_DVL;

    //Transformasi Koordinat DVL

    double vN_DVL = nav.vX_DVL*(cos(Omega_Pitch_rad)*cos(Omega_Heading_rad))+
                    nav.vY_DVL*((sin(Omega_Roll_rad)*sin(Omega_Pitch_rad)*cos(Omega_Heading_rad))-(cos(Omega_Roll_rad)*sin(Omega_Heading_rad)))+
                    nav.vZ_DVL*((cos(Omega_Roll_rad)*sin(Omega_Pitch_rad)*cos(Omega_Heading_rad))+(sin(Omega_Roll_rad)*sin(Omega_Heading_rad)));

    double vE_DVL = nav.vX_DVL*(cos(Omega_Pitch_rad)*sin(Omega_Heading_rad))+
                    nav.vY_DVL*((sin(Omega_Roll_rad)*sin(Omega_Pitch_rad)*sin(Omega_Heading_rad))+(cos(Omega_Roll_rad)*cos(Omega_Heading_rad)))+
                    nav.vZ_DVL*((cos(Omega_Roll_rad)*sin(Omega_Pitch_rad)*sin(Omega_Heading_rad))-(sin(Omega_Roll_rad)*cos(Omega_Heading_rad)));

    double vD_DVL = nav.vX_DVL*(-1*sin(Omega_Pitch_rad))+
                    nav.vY_DVL*(sin(Omega_Roll_rad)*cos(Omega_Pitch_rad))+
                    nav.vZ_DVL*(cos(Omega_Roll_rad)*cos(Omega_Pitch_rad));

    
    // nav.latitude = sensor.IMU.latitude;
    // nav.longitude = sensor.IMU.longitude;
    //nav.latitude_init = init_Latitude[0];
    //nav.longitude_init = init_Longitude[0];
    // std::cout << "Latitude : " << nav.latitude << " Longitude : " << nav.longitude << std::endl;
    double velocity = pow(pow(nav.vX_DVL, 2) + pow(nav.vY_DVL, 2), 0.5); //(velx^2+vely^2)^0.5

    if (curStatus == 0 && pos_cov_Current < threshold_pos_cov)
    {
        cout << "Real GPS Data (0)" << endl;
        if (GPSintFlag == 0)
        {
            nav.latitude_init = sensor.IMU.latitude;
            nav.longitude_init = sensor.IMU.longitude;
            GPSintFlag = 1;
            //GPSintFlag = 1;//nav.latitude_init = nav.latitude;
            //nav.longitude_init = nav.longitude;
	    //nav.latitude_init = init_Latitude[0];
   	    //nav.longitude_init = init_Longitude[0];
        }
        // init_Latitude[Cont] = nav.latitude;     //inisial pertama dari lat
        // init_Longitude[Cont] = nav.longitude;   //inisial pertama dari long
        // nav.latitude_init = init_Latitude[0];
        // nav.longitude_init = init_Longitude[0];
        // cout << "Latitude_init " << nav.latitude_init << " Longitude_init : " << nav.longitude_init << endl;
        // std::cout << "Latitude_init  : "<<nav.latitude_init<<std::endl;
        // std::cout << "Longitude_init : "<<nav.longitude_init<<std::endl;

        /*
        nav.N_aug = 110520 * (nav.latitude - nav.latitude_init);
        nav.E_aug = 111600 * (nav.longitude - nav.longitude_init);

        lastgpsnorth = nav.N_aug;
        lastgpseast = nav.E_aug;
        */
        nav.latitude = sensor.IMU.latitude;
        nav.longitude = sensor.IMU.longitude;
	    nav.N_aug = (pengali_lat * (nav.latitude - nav.latitude_init));
        nav.E_aug = (pengali_lon * (nav.longitude - nav.longitude_init));
	    lastgpsnorth = nav.N_aug;
        lastgpseast = nav.E_aug;

        //Cont += 1;
    }
    else
    {
        if (GPSintFlag == 0)
        {
            //nav.latitude_init = sensor.IMU.latitude;
            //nav.longitude_init = sensor.IMU.longitude;
            GPSintFlag = 1;
            //GPSintFlag = 1;//nav.latitude_init = nav.latitude;
            //nav.longitude_init = nav.longitude;
	        nav.latitude_init = init_Latitude[0];
   	        nav.longitude_init = init_Longitude[0];
        }
        //integral velocity to position
        nav.N_aug += vN_DVL * timeElapsed;
        nav.E_aug += vE_DVL * timeElapsed;
        nav.Depth_aug += vD_DVL * timeElapsed;

        //conversion Position NED to latitude & longitude
        //nav.latitude = nav.latitude_init + (nav.N_aug*R_m); 
        //nav.longitude = nav.longitude_init + (nav.E_aug*R_m)/cos(nav.longitude_init*(PI/180));

        nav.latitude = (nav.N_aug / pengali_lat) + nav.latitude_init;
        nav.longitude = (nav.E_aug / pengali_lon) + nav.longitude_init;

        // if (lastStatus == 0)
        // {
        //     nav.N_aug = lastgpsnorth;
        //     nav.E_aug = lastgpseast;
        // }

        /*
        nav.N_aug += velocity * cos(yaw_rad) * timeElapsed;
        cout << "Nav N Aug" << nav.N_aug << endl;
        nav.E_aug += velocity * sin(yaw_rad) * timeElapsed;
        cout << "Nav E Aug" << nav.E_aug << endl;
        nav.latitude = (nav.N_aug / 110520) + nav.latitude_init;
        nav.longitude = (nav.E_aug / 111600) + nav.longitude_init;
        */

        // nav.N_aug = 110520 * (nav.latitude - nav.latitude_init);
        // nav.E_aug = 111600 * (nav.longitude - nav.longitude_init);
    }

    lastStatus = curStatus;

    /*
        double PI = 3.1415926536;
        double g = 9.8335;

        //Input
        //double Omega_Pitch_rad = sensor.IMU.omegaPitch * PI / 180.0;
        //diubah dalam bentuk radian
        int Status_GPS = sensor.IMU.Status;
        double Heading_IMU_rad = sensor.IMU.omegaHeading*(PI/180);


        double Omega_Pitch_rad = sensor.IMU.omegaPitch*(PI/180.0);
        double Omega_Heading_rad = sensor.IMU.omegaHeading*(PI/180.0);
        double Omega_Roll_rad = sensor.IMU.omegaRoll*(PI/180.0);

        //Transformasi koordinat IMU

        double udot_ned = sensor.IMU.accel_X*(cos(Omega_Pitch_rad)*cos(Omega_Heading_rad))+
                          sensor.IMU.accel_Y*((sin(Omega_Roll_rad)*sin(Omega_Pitch_rad)*cos(Omega_Heading_rad))-(cos(Omega_Roll_rad)*sin(Omega_Heading_rad)))+
                          sensor.IMU.accel_Z*((cos(Omega_Roll_rad)*sin(Omega_Pitch_rad)*cos(Omega_Heading_rad))+(sin(Omega_Roll_rad)*sin(Omega_Heading_rad)));

        double vdot_ned = sensor.IMU.accel_X*(cos(Omega_Pitch_rad)*sin(Omega_Heading_rad))+
                          sensor.IMU.accel_Y*((sin(Omega_Roll_rad)*sin(Omega_Pitch_rad)*sin(Omega_Heading_rad))+(cos(Omega_Roll_rad)*cos(Omega_Heading_rad)))+
                          sensor.IMU.accel_Z*((cos(Omega_Roll_rad)*sin(Omega_Pitch_rad)*sin(Omega_Heading_rad))-(sin(Omega_Roll_rad)*cos(Omega_Heading_rad)));

        double wdot_ned = sensor.IMU.accel_X*(-1*sin(Omega_Pitch_rad))+
                          sensor.IMU.accel_Y*(sin(Omega_Roll_rad)*cos(Omega_Pitch_rad))+
                          sensor.IMU.accel_Z*(cos(Omega_Roll_rad)*cos(Omega_Pitch_rad))+g;

        //Transformasi Koordinat DVL

        double vN_DVL = sensor.DVL.veloX_F*(cos(Omega_Pitch_rad)*cos(Omega_Heading_rad))+
                        sensor.DVL.veloY_F*((sin(Omega_Roll_rad)*sin(Omega_Pitch_rad)*cos(Omega_Heading_rad))-(cos(Omega_Roll_rad)*sin(Omega_Heading_rad)))+
                        sensor.DVL.veloZ_F*((cos(Omega_Roll_rad)*sin(Omega_Pitch_rad)*cos(Omega_Heading_rad))+(sin(Omega_Roll_rad)*sin(Omega_Heading_rad)));

        double vE_DVL = sensor.DVL.veloX_F*(cos(Omega_Pitch_rad)*sin(Omega_Heading_rad))+
                        sensor.DVL.veloY_F*((sin(Omega_Roll_rad)*sin(Omega_Pitch_rad)*sin(Omega_Heading_rad))+(cos(Omega_Roll_rad)*cos(Omega_Heading_rad)))+
                        sensor.DVL.veloZ_F*((cos(Omega_Roll_rad)*sin(Omega_Pitch_rad)*sin(Omega_Heading_rad))-(sin(Omega_Roll_rad)*cos(Omega_Heading_rad)));

        double vD_DVL = sensor.DVL.veloX_F*(-1*sin(Omega_Pitch_rad))+
                        sensor.DVL.veloY_F*(sin(Omega_Roll_rad)*cos(Omega_Pitch_rad))+
                        sensor.DVL.veloZ_F*(cos(Omega_Roll_rad)*cos(Omega_Pitch_rad))+g;

        //Hitung kecepatan IMU
        if (nav.latitude==0)
            nav.latitude=sensor.IMU.latitude;
        else
            nav.latitude = nav.latitude * PI / 180;

        double omegaE = 7.292115 * 0.00001;
        double R =  6378137;

        double aIMU_N = udot_ned-2*omegaE*sin(nav.latitude)*nav.vE_IMU-(nav.vE_IMU*nav.vE_IMU)/R*tan(nav.latitude)+nav.vN_IMU*nav.vD_IMU/R;
        double aIMU_E = vdot_ned-2*omegaE*sin(nav.latitude)*nav.vN_IMU-nav.vE_IMU*nav.vN_IMU/R*tan(nav.latitude)-2*omegaE*cos(nav.latitude)*nav.vD_IMU-nav.vE_IMU*nav.vD_IMU/R;
        double aIMU_D = wdot_ned-2*omegaE*cos(nav.latitude)*nav.vE_IMU-(nav.vE_IMU*nav.vE_IMU)/R-(nav.vN_IMU*nav.vN_IMU)/R;

        if(nav.reset == 1){
            nav.vN_IMU = 0;
            nav.vE_IMU = 0;//kali selang waktu
            nav.vD_IMU = 0;//kali selang waktu
        }
        nav.vN_IMU = aIMU_N;//nav.vN_IMU + aIMU_N * 1;//kali selang waktu
        nav.vE_IMU = aIMU_E;//nav.vE_IMU + aIMU_E * 1;//kali selang waktu
        nav.vD_IMU = aIMU_D;//nav.vD_IMU + aIMU_D * 1;//kali selang waktu

        //Galat Kecepatan
        double dvN = nav.vN_IMU-vN_DVL;
        double dvE = nav.vE_IMU-vE_DVL;
        double dvD = nav.vD_IMU-vD_DVL;

    //Kalman Filter
        nav.meas << dvN,
                dvE,
                dvD;

        if(nav.kalmanReset==0 || nav.reset == 1)
        {
            nav.xhat = Eigen::VectorXd::Zero(5);
            //inisialisasi measurement
            nav.P = Eigen::MatrixXd::Identity(5,5);
            nav.A <<    1, 9.7803, 0, 0, 0,
                    -0.00000015679, 1, 0, 0, 0,
                    0, 0, 1, 9.7803, 0,
                    0, 0, 0.00000015679, 1, 0,
                    0, 0, 0, 0, 1;
            nav.Atrans = nav.A.transpose();
            nav.C <<    1, 0, 0, 0, 0,
                    0, 0, 1, 0, 0,
                    0, 0, 0, 0, 1;
            nav.Ctrans = nav.C.transpose();
            nav.matrixR <<  0.0303, 0, 0,
                        0, 0.0297, 0,
                        0, 0, 0.0211;
            //matrixR = matrixR.sqrt();
            nav.kalmanReset=1;
        }

        nav.xhat = nav.A*nav.xhat;
        //Covariance matrix
        nav.P = nav.A*nav.P*nav.Atrans + Eigen::MatrixXd::Zero(5,5);
        //Calculate Kalman Gain
        nav.PCtrans = nav.P*nav.Ctrans;
        nav.CPCtransR = nav.C*nav.P*nav.Ctrans + nav.matrixR;
        nav.K = nav.PCtrans * nav.CPCtransR.inverse();
        //calculate the measurement residual
        nav.Resid = nav.meas - nav.C*nav.xhat;
        //Update the state and error covariance estimate
        nav.xhat = nav.xhat + nav.K*nav.Resid;
        nav.P = (Eigen::MatrixXd::Identity(5,5) - nav.K*nav.C)*nav.P;

        //output Kalman
        nav.xhatOut = nav.xhat;
        nav.yhatOut = nav.C*nav.xhatOut;

        double vN_kalman = nav.yhatOut(0);
        double vE_kalman = nav.yhatOut(1);
        double vD_kalman = nav.yhatOut(2);

        //Estimasi Kecepatan
        double vN_est = nav.vN_IMU-vN_kalman;
        double vE_est = nav.vE_IMU-vE_kalman;
        double vD_est = nav.vD_IMU-vD_kalman;

        nav.kecN = vN_est;
        nav.kecE = vE_est;
        nav.kecD = vD_est;

        //integrator
        if(nav.reset == 1)
        {
            nav.N_aug = 0;
            nav.E_aug = 0;
            nav.Depth_aug = 0;
        }
        nav.N_aug += vN_est;
        nav.E_aug += vE_est;
        nav.Depth_aug += vD_est;

        //hasil integrator ditambah posisi awal
    //   nav.latitude = (nav.N_aug/R)*180/PI + sensor.IMU.latitude;
    //   nav.longitude = nav.E_aug/(R*cos(nav.latitude * PI / 180.0))*180/PI + sensor.IMU.longitude;
        //hasil dalam satuan degree
        nav.latitude  = (nav.N_aug/R)*180/PI + sensor.IMU.latitude*180/PI;
        nav.longitude = (nav.E_aug/(R*cos(nav.latitude)))*180/PI + sensor.IMU.longitude*180/PI;
    */
    // need further information
    /*
        Nav_z = posisidepth_aug + depth_gps; //DEPTH GPS??? <<<----------- TAKE NOTE!
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
    */

    return nav;
}

void Navigation::Init()
{
}
void Navigation::Reset()
{
}
void Navigation::Print()
{
    std::cout << "Latitude  : " << latitude << std::endl;
    std::cout << "Longitude : " << longitude << std::endl;
    std::cout << "Latitude_init  : " << latitude_init << std::endl;
    std::cout << "Longitude_init : " << longitude_init << std::endl;
    std::cout << "Heading_IMU: " << Heading_IMU << std::endl;
    std::cout << "z         : " << z << std::endl;
    std::cout << "Status_GPS: " << Status_GPS << std::endl;
    std::cout << "vE_IMU    : " << vE_IMU << std::endl;
    std::cout << "vN_IMU    : " << vN_IMU << std::endl;
    std::cout << "vD_IMU    : " << vD_IMU << std::endl;
    std::cout << "vX_DVL    : " << vX_DVL << std::endl;
    std::cout << "vY_DVL    : " << vY_DVL << std::endl;
    std::cout << "vZ_DVL    : " << vZ_DVL << std::endl;
    std::cout << "kecN      : " << kecN << std::endl;
    std::cout << "kecE      : " << kecE << std::endl;
    std::cout << "kecD      : " << kecD << std::endl;
    std::cout << "N_aug     : " << N_aug << std::endl;
    std::cout << "E_aug     : " << E_aug << std::endl;
    std::cout << "Depth_aug : " << Depth_aug << std::endl;
}

/*
    //call example :
    //declare struct
    Guidance guidance;
    ...
    ...
    ...
    //calculate Guidance
    guidance = calculateGuidance(guidance, navigation.latitude, navigation,longiture, wpXTarget, wpYTarget);
    //output
    cout << guidance.distanceFromTarget << " " <<guidance.psiRef<< endl;
*/
Guidance calculateGuidance(Guidance _temp, double wpXNow, double wpYNow, double wpXTarget, double wpYTarget)
{
    Guidance temp = _temp;
    double PI = 3.1415926536;

    temp.acceptanceRadius = 1;

    if (temp.isFinished)
    {
        temp.psiRef = 0;
    }
    else
    {
        /*
        //.:.relic
        wpXTarget *= wpXTarget * 180 / PI;
        wpYTarget *= wpYTarget * 180 / PI;
        wpXNow *= wpXNow * 180 / PI;
        wpYNow *= wpYNow * 180 / PI;

        float _psiRef = atan2((wpYTarget - wpYNow),(wpXTarget - wpXNow));
        _psiRef = _psiRef*180.0/PI;
        temp.distanceFromTarget = sqrt(pow((wpYTarget-wpYNow),2)+pow((wpXTarget-wpXNow),2));

        //transformasi (??????)

        if (_psiRef > 90 && _psiRef <= 180)
            temp.psiRef = 450 - _psiRef;
        else
            temp.psiRef = 90 - _psiRef;

        //:.:relic
        */
        // calculate distance with haversine formula
        // or equirectangular approximation

        double _distance;
        // constant
        double R = 6371e3;
        double PI = 3.14159265358979323846;
        // convert to rad
        double psi1 = wpXNow * PI / 180;
        double psi2 = wpXTarget * PI / 180;

        // haversine formula (choose one)
        double deltaPsi = (wpXTarget - wpXNow) * PI / 180;
        double deltaLon = (wpYTarget - wpYNow) * PI / 180;

        double _a = sin(deltaPsi / 2) * sin(deltaPsi / 2) +
                    cos(psi1) * cos(psi2) *
                        sin(deltaLon / 2) * sin(deltaLon / 2);
        double _c = 2 * atan2(sqrt(_a), sqrt(1 - _a));
        _distance = R * _c; // in metres

        // equirectangular approximation (choose one)
        //  double _x = deltaLon * cos((psi1+psi2)/2);
        //  double _y = (psi2-psi1);
        //  distance = sqrt(pow(_x,2)+pow(_y,2)) * R ; //in metres

        // bearing (psiref)
        double __y = sin(deltaLon) * cos(psi2);
        double __x = cos(psi1) * sin(psi2) - sin(psi1) * cos(psi2) * cos(deltaLon);
        double __teta = atan2(__y, __x);
        // initial bearing
        double __bearing = __teta * 180 / PI + 360; // % 360; //in degrees
        __bearing = fmod(__bearing, 360);

        // normalize single direction
        // output will be in range of -180 to 180
        // fabs(x) is absolute function for float numbers
        // loop until value is acceptable
        while (fabs(__bearing) > 180.0)
        {
            // if value is negative
            if (__bearing < 0)
                __bearing += 360;
            // if value is positive
            else
                __bearing -= 360;
        }

        temp.distanceFromTarget = _distance;
        temp.psiRef = __bearing;

        if (temp.distanceFromTarget <= temp.acceptanceRadius)
            temp.isFinished = true;

        // for debugging purpose
        // temp.isFinished = false;
    }
    return temp;
}

void Guidance::Init()
{
}
void Guidance::Reset()
{
}
void Guidance::Print()
{
}
