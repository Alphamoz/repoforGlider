

#include "imu.h"
//#include "imu_acuan.h"
#include <stddef.h>
#include <stdio.h>

#include "stdio.h"
#include "termios.h"
#include "errno.h"
#include "fcntl.h"
#include "string.h"
#include "time.h"
#include <stdlib.h> 
#include "sys/select.h"

#include <unistd.h>

#include <time.h>

int main(int argc, char *argv[]){
	

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
	double posCov; 
	int Status;
	
	init_imu();
	
	while(1)
	{
		read_imu(Cport, &Latitude, &Longitude, &Height, &Roll, &Pitch, &Heading, &accel_X, &accel_Y, &accel_Z, &Omega_Roll, &Omega_Pitch, &Omega_Heading, &Status, &posCov);
		
		printf("\tStatus = %d, Latitude = %f, Longitude = %f, Height = %f\n", Status, Latitude, Longitude, Height);
		printf("\tPosition_covariance = %f\n", posCov);
		printf("\tRoll = %f, Pitch = %f, Heading = %f\n", Roll, Pitch, Heading);
		printf("\tAccelerometers X: %f Y: %f Z: %f\n", accel_X, accel_Y, accel_Z);
		printf("\tGyroscopes X: %f Y: %f Z: %f\n", Omega_Roll, Omega_Pitch, Omega_Heading);
				
		sleep (1);
	}	

	return 0;
}
