///supress warning Wwrite-string
#pragma GCC diagnostic ignored "-Wwrite-strings"
#pragma GCC diagnostic ignored "-Wformat="

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
#include <stdio.h>                     // This ert_main.c example uses printf/fflush 
#include <typeinfo>
#include "imu.h"
#include <vector>
#include<signal.h>
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/MatrixFunctions"
#include <math.h>

using namespace std;

// Struct for GPIO purpose ================================================

void    gpioActivation();
int     TriggerDVL();
int     Relay_On();
int     Relay_Off();


struct gpio{

    int Pin;
    FILE* Handler = NULL;
    char Set[4],String[4],Value[64],Direction[64];

}DVL_GPIO,Relay_GPIO;



int main()
{
   gpioActivation();
   Relay_On();
   sleep(2);
   Relay_Off();
   sleep(4);
   Relay_On();
   sleep(2);

}

// Procedure for GPIO usage --------------------------------------------------------
void gpioActivation(){
    // DVL
    DVL_GPIO.Pin = 61;
    sprintf(DVL_GPIO.String, "%d", DVL_GPIO.Pin);
    sprintf(DVL_GPIO.Value, "/sys/class/gpio/gpio%d/value", DVL_GPIO.Pin);
    sprintf(DVL_GPIO.Direction, "/sys/class/gpio/gpio%d/direction", DVL_GPIO.Pin);
    
    DVL_GPIO.Handler = fopen(DVL_GPIO.Direction,"rb+");
    if (DVL_GPIO.Handler == NULL){
        cout << "error set DVL trigger pin direction" << endl;
        // return 1;
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
        // return 1;
    }

    strcpy(Relay_GPIO.Set,"out");
    fwrite(&Relay_GPIO.Set, sizeof(char), 3, Relay_GPIO.Handler);
    fclose(Relay_GPIO.Handler);

    Relay_On();
}

// Function for Relay ON -----------------------------------------------------------
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

// Function for Relay OFF -----------------------------------------------------------
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

