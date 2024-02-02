/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*          C Language Dynamic Spatial SDK, Version 4.0         */
/*   Copyright 2014, Xavier Orr, Advanced Navigation Pty Ltd    */
/*                                                              */
/****************************************************************/
/*
 * Copyright (C) 2014 Advanced Navigation Pty Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

/*
#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"*/
#include "imu_acuan.h"
/*#include <windows.h>*/

#define RADIANS_TO_DEGREES (180.0/M_PI)

an_decoder_t an_decoder;
an_packet_t *an_packet;
system_state_packet_t system_state_packet;
raw_sensors_packet_t raw_sensors_packet;
int bytes_received;

int an_packet_transmit(an_packet_t *an_packet)
{
	an_packet_encode(an_packet);
	return SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
}

/*
 * This is an example of sending a configuration packet to Spatial.
 *
 * 1. First declare the structure for the packet, in this case sensor_ranges_packet_t.
 * 2. Set all the fields of the packet structure
 * 3. Encode the packet structure into an an_packet_t using the appropriate helper function
 * 4. Send the packet
 * 5. Free the packet
 */
void set_sensor_ranges()
{
	an_packet_t *an_packet;
	sensor_ranges_packet_t sensor_ranges_packet;

	sensor_ranges_packet.permanent = TRUE;
	sensor_ranges_packet.accelerometers_range = accelerometer_range_4g;
	sensor_ranges_packet.gyroscopes_range = gyroscope_range_500dps;
	sensor_ranges_packet.magnetometers_range = magnetometer_range_2g;

	an_packet = encode_sensor_ranges_packet(&sensor_ranges_packet);

	an_packet_transmit(an_packet);

	an_packet_free(&an_packet);
}

void read_imu_always(void)
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
	double Omega_Roll;		
	double Omega_Pitch;		
	double Omega_Heading;	
//	double posCov;
	int Status;
	
	init_imu();
	while (1)
	{
		
		read_imu(Cport, &Latitude, &Longitude, &Height, &Roll, &Pitch, &Heading, &accel_X, &accel_Y, &accel_Z, &Omega_Roll, &Omega_Pitch, &Omega_Heading, &Status);
	
	}
}

void init_imu(void){

	if (OpenComport((char*)COM_PORT1, BAUD_RATE1))
	{
		printf("Could not open serial port\n");
		exit(EXIT_FAILURE);
	}

	#if uart_imu_send
	if (OpenComport2(COM_PORT2, BAUD_RATE2))
	{
		printf("Could not open serial port 2\n");
		exit(EXIT_FAILURE);
	}
	#endif

	an_decoder_initialise(&an_decoder);

}

void read_imu(int uart_object, 	double *Latitude, double *Longitude, double *Height, double *Roll, double *Pitch, double *Heading,
					double *accel_X, double *accel_Y, double *accel_Z, double *Omega_Roll, double *Omega_Pitch, double *Omega_Heading, int *Status){

	if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder), uart_object)) > 0)
		{
			/* increment the decode buffer length by the number of bytes received */
			an_decoder_increment(&an_decoder, bytes_received);
			
			/* decode all the packets in the buffer */
			while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{
				if (an_packet->id == packet_id_system_state) /* system state packet */
				{
					/* copy all the binary data into the typedef struct for the packet */
					/* this allows easy access to all the different values             */
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						*Latitude = system_state_packet.latitude * RADIANS_TO_DEGREES;
						*Longitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
						*Height = system_state_packet.height;
						*Roll = system_state_packet.orientation[0] * RADIANS_TO_DEGREES;
						*Pitch = system_state_packet.orientation[1] * RADIANS_TO_DEGREES;
						*Heading = system_state_packet.orientation[2] * RADIANS_TO_DEGREES;
						
                                                if ((system_state_packet.filter_status.b.gnss_fix_type == 1) ||
                                                        (system_state_packet.filter_status.b.gnss_fix_type == 2))
                                                {
                                                        *Status=0;
                                                }
                                                else if ((system_state_packet.filter_status.b.gnss_fix_type == 3) ||
                                                         (system_state_packet.filter_status.b.gnss_fix_type == 5))
                                                {
                                                        *Status=1;
                                                }
                                                else if ((system_state_packet.filter_status.b.gnss_fix_type == 4) ||
                                                         (system_state_packet.filter_status.b.gnss_fix_type == 6) ||
                                                         (system_state_packet.filter_status.b.gnss_fix_type == 7))
                                                {
                                                        *Status=2;
                                                }
                                                else
                                                {
                                                        *Status=-1;
                                                }
						
					//	*posCov = 10.00;
						// *posCov = (pow(system_state_packet.standard_deviation[1],2)+pow(system_state_packet.standard_deviation[0],2)+pow(system_state_packet.standard_deviation[2],2))/3;
						
                                                #if imu_print
                                                printf("System State Packet:\n");
                                                printf("\tLatitude = %f, Longitude = %f, Height = %f\n", *Latitude, *Longitude, *Height);
                                                printf("\tRoll = %f, Pitch = %f, Heading = %f\n", *Roll, *Pitch, *Heading);
                                                printf("GPS status = %f\n",*Status);
						//printf("\tPosition_covariance = %f\n", *posCov);
						#endif
					}
					
				}
				else if (an_packet->id == packet_id_raw_sensors) /* raw sensors packet */
				{
					/* copy all the binary data into the typedef struct for the packet */
					/* this allows easy access to all the different values             */
					if(decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0)
					{
						*accel_X = raw_sensors_packet.accelerometers[0];
						*accel_Y = raw_sensors_packet.accelerometers[1];
						*accel_Z = raw_sensors_packet.accelerometers[2];
						*Omega_Roll = raw_sensors_packet.gyroscopes[0] * RADIANS_TO_DEGREES;	
						*Omega_Pitch = raw_sensors_packet.gyroscopes[1] * RADIANS_TO_DEGREES;	
						*Omega_Heading = raw_sensors_packet.gyroscopes[2] * RADIANS_TO_DEGREES;
						
						#if imu_print
						printf("Raw Sensors Packet:\n");
						printf("\tAccelerometers X: %f Y: %f Z: %f\n", *accel_X, *accel_Y, *accel_Z);
						printf("\tGyroscopes X: %f Y: %f Z: %f\n", *Omega_Roll, *Omega_Pitch, *Omega_Heading);
						#endif
					}
				}
				else
				{
					printf("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);
				}
				
				/* Ensure that you free the an_packet when your done with it or you will leak memory */
				an_packet_free(&an_packet);
			}

			#if uart_imu_send
			SendAllData(*Latitude, *Longitude, *Height, *Roll, *Pitch, *Heading);		
			#endif

	}
}


void assign_imu_variables(double input1, double input2, double input3, double input4, double input5, double input6,
									double input7, double input8, double input9, double input10, double input11, double input12,
									double *output1, double *output2, double *output3, double *output4, double*output5, double *output6,
									double *output7, double *output8, double *output9, double *output10, double *output11, double *output12){
									
	*output1   = input1;
	*output2   = input2;
	*output3   = input3;
	*output4   = input4;
	*output5   = input5;
	*output6   = input6;
	*output7   = input7;
	*output8   = input8;
	*output9   = input9;
	*output10 = input10;
	*output11 = input11;
	*output12 = input12;
}
