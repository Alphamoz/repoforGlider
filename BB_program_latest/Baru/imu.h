#ifndef IMU_H
#define IMU_H

// #define COM_PORT1 "/dev/ttyUSB3"
#define COM_PORT1 "/dev/ttyIMU"
#define COM_PORT2 "/dev/ttyUSB3"
#define BAUD_RATE1 115200
#define BAUD_RATE2 115200
#define imu_print 0
#define uart_imu_send 0

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"
	
extern an_decoder_t an_decoder;
extern an_packet_t *an_packet;
extern system_state_packet_t system_state_packet;
extern raw_sensors_packet_t raw_sensors_packet;
extern int bytes_received;

void read_imu_always(void);
void init_imu(void);
void read_imu(int uart_object, 	double *Latitude, double *Longitude, double *Height, double *Roll, double *Pitch, double *Heading,
					double *accel_X, double *accel_Y, double *accel_Z, double *Omega_Roll, double *Omega_Pitch, double *Omega_Heading, int *Status, double *Covpos);
void assign_imu_variables(double input1, double input2, double input3, double input4, double input5, double input6,
									double input7, double input8, double input9, double input10, double input11, double input12,
									double *output1, double *output2, double *output3, double *output4, double*output5, double *output6,
									double *output7, double *output8, double *output9, double *output10, double *output11, double *output12);

#endif
