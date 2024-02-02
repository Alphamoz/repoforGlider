/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*        ROS Driver, Packet to Published Message Example       */
/*          Copyright 2017, Advanced Navigation Pty Ltd         */
/*                                                              */
/****************************************************************/
/*
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

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <tf/tf.h>

#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

// ini msgny
#include <glider_msg/imumsg.h>
#include <geometry_msgs/Twist.h>

#define RADIANS_TO_DEGREES (180.0/M_PI)

int main(int argc, char *argv[]) {
	// Set up ROS node //
	ros::init(argc, argv, "node_IMUGPS");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	printf("\nYour Advanced Navigation ROS driver is currently running\nPress Ctrl-C to interrupt\n");

	// Set up the COM port
	std::string com_port;
	int baud_rate;
	std::string imu_frame_id;
	std::string nav_sat_frame_id;
	std::string topic_prefix;

	if (argc >= 3) {
		com_port = std::string(argv[1]);
		baud_rate = atoi(argv[2]);
	}
	else {
		pnh.param("port", com_port, std::string("/dev/ttyACM0"));
		pnh.param("baud_rate", baud_rate, 115200);
	}

	pnh.param("topic_prefix", topic_prefix, std::string("messageIMUGPS"));

	// Initialise Publishers and Topics //
	ros::Publisher IMUGPSpublisher=nh.advertise<glider_msg::imumsg>(topic_prefix,10);

	//initialise Spatial
	// Ned _ned(0.0, 0.0, 0.0);
	bool init_ned = false ;
	glider_msg::imumsg imumsg;

	imumsg.header.stamp.sec=0;
	imumsg.header.stamp.nsec=0;
	imumsg.header.frame_id='0';

	// get data from com port //
	an_decoder_t an_decoder;
	an_packet_t *an_packet;
	system_state_packet_t system_state_packet;
	euler_orientation_standard_deviation_packet_t euler_orientation_standard_deviation_packet;
	velocity_standard_deviation_packet_t velocity_standard_deviation_packet; 
	//quaternion_orientation_standard_deviation_packet_t quaternion_orientation_standard_deviation_packet;
	int bytes_received;

	if (OpenComport(const_cast<char*>(com_port.c_str()), baud_rate))
	{
		printf("Could not open serial port: %s \n",com_port.c_str());
		exit(EXIT_FAILURE);
	}

	an_decoder_initialise(&an_decoder);

	// Loop continuously, polling for packets
	while (ros::ok())
	{
		ros::spinOnce();
		if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{
			// increment the decode buffer length by the number of bytes received //
			an_decoder_increment(&an_decoder, bytes_received);

			// decode all the packets in the buffer //
			while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{
				// system state packet //
				if (an_packet->id == packet_id_system_state)
				{
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						// //NavSatFix
						// imumsg.header.stamp.sec=system_state_packet.unix_time_seconds;
						// imumsg.header.stamp.nsec=system_state_packet.microseconds*1000;
						// imumsg.header.frame_id=nav_sat_frame_id;

						// IMU
						imumsg.header.stamp.sec=system_state_packet.unix_time_seconds;
						imumsg.header.stamp.nsec=system_state_packet.microseconds*1000;
						imumsg.header.frame_id=imu_frame_id;
						std::cout<<imumsg.gpsStatus<<std::endl;
						if ((system_state_packet.filter_status.b.gnss_fix_type == 1) ||
							(system_state_packet.filter_status.b.gnss_fix_type == 2))
						{
							imumsg.gpsStatus=0;
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 3) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 5))
						{
							imumsg.gpsStatus=1;
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 4) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 6) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 7))
						{
							imumsg.gpsStatus=2;
						}
						else
						{
							imumsg.gpsStatus=-1;
						}
						
						imumsg.latitude=system_state_packet.latitude * RADIANS_TO_DEGREES;
						imumsg.longitude=system_state_packet.longitude * RADIANS_TO_DEGREES;
						imumsg.altitude=system_state_packet.height;


						// Twist
						imumsg.linearX=system_state_packet.velocity[0];
						imumsg.linearY=system_state_packet.velocity[1];
						imumsg.linearZ=system_state_packet.velocity[2];
						imumsg.angularX=system_state_packet.angular_velocity[0];
						imumsg.angularY=system_state_packet.angular_velocity[1];
						imumsg.angularZ=system_state_packet.angular_velocity[2];


						imumsg.angularVeloX=system_state_packet.angular_velocity[0]; // These the same as the TWIST msg values
						imumsg.angularVeloY=system_state_packet.angular_velocity[1];
						imumsg.angularVeloZ=system_state_packet.angular_velocity[2];
						imumsg.linearAccelerationX=system_state_packet.body_acceleration[0];
						imumsg.linearAccelerationY=system_state_packet.body_acceleration[1];
						imumsg.linearAccelerationZ=system_state_packet.body_acceleration[2];
						
						// Convert roll, pitch, yaw from radians to quaternion format //
						imumsg.RPY[0] = system_state_packet.orientation[0] ;
						imumsg.RPY[1] = system_state_packet.orientation[1]  ;
						imumsg.RPY[2] = system_state_packet.orientation[2]  ;

						geometry_msgs::Quaternion quat_msg = tf::createQuaternionMsgFromRollPitchYaw(
																imumsg.RPY[0],
																imumsg.RPY[1],
																imumsg.RPY[2] 	
															);
						imumsg.orientationX= quat_msg.x ; 
						imumsg.orientationY= quat_msg.y ;
						imumsg.orientationZ= quat_msg.z ;
						imumsg.orientationW= quat_msg.w ;

						// System Status
						imumsg.systemStatusMessage = "";
						imumsg.statusLevel = 0; // default OK state
						if (system_state_packet.system_status.b.system_failure) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "0. System Failure! ";
						}
						if (system_state_packet.system_status.b.accelerometer_sensor_failure) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "1. Accelerometer Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.gyroscope_sensor_failure) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "2. Gyroscope Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.magnetometer_sensor_failure) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "3. Magnetometer Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.pressure_sensor_failure) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "4. Pressure Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.gnss_failure) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "5. GNSS Failure! ";
						}
						if (system_state_packet.system_status.b.accelerometer_over_range) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "6. Accelerometer Over Range! ";
						}
						if (system_state_packet.system_status.b.gyroscope_over_range) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "7. Gyroscope Over Range! ";
						}
						if (system_state_packet.system_status.b.magnetometer_over_range) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "8. Magnetometer Over Range! ";
						}
						if (system_state_packet.system_status.b.pressure_over_range) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "9. Pressure Over Range! ";
						}
						if (system_state_packet.system_status.b.minimum_temperature_alarm) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "10. Minimum Temperature Alarm! ";
						}
						if (system_state_packet.system_status.b.maximum_temperature_alarm) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "11. Maximum Temperature Alarm! ";
						}
						if (system_state_packet.system_status.b.low_voltage_alarm) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "12. Low Voltage Alarm! ";
						}
						if (system_state_packet.system_status.b.high_voltage_alarm) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "13. High Voltage Alarm! ";
						}
						if (system_state_packet.system_status.b.gnss_antenna_disconnected) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "14. GNSS Antenna Disconnected! ";
						}
						if (system_state_packet.system_status.b.serial_port_overflow_alarm) {
							imumsg.statusLevel = 2; // ERROR state
							imumsg.systemStatusMessage = imumsg.systemStatusMessage + "15. Data Output Overflow Alarm! ";
						}

					// 	// Filter Status
					// 	filter_status_msg.message = "";
					// 	filter_status_msg.level = 0; // default OK state
					// 	if (system_state_packet.filter_status.b.orientation_filter_initialised) {
					// 		filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter Initialised. ";
					// 	}
					// 	else {
					// 		filter_status_msg.level = 1; // WARN state
					// 		filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter NOT Initialised. ";
					// 	}
					// 	if (system_state_packet.filter_status.b.ins_filter_initialised) {
					// 		filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter Initialised. ";
					// 	}
					// 	else {
					// 		filter_status_msg.level = 1; // WARN state
					// 		filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter NOT Initialised. ";
					// 	}
					// 	if (system_state_packet.filter_status.b.heading_initialised) {
					// 		filter_status_msg.message = filter_status_msg.message + "2. Heading Initialised. ";
					// 	}
					// 	else {
					// 		filter_status_msg.level = 1; // WARN state
					// 		filter_status_msg.message = filter_status_msg.message + "2. Heading NOT Initialised. ";
					// 	}
					// 	if (system_state_packet.filter_status.b.utc_time_initialised) {
					// 		filter_status_msg.message = filter_status_msg.message + "3. UTC Time Initialised. ";
					// 	}
					// 	else {
					// 		filter_status_msg.level = 1; // WARN state
					// 		filter_status_msg.message = filter_status_msg.message + "3. UTC Time NOT Initialised. ";
					// 	}
					// 	if (system_state_packet.filter_status.b.event1_flag) {
					// 		filter_status_msg.level = 1; // WARN state
					// 		filter_status_msg.message = filter_status_msg.message + "7. Event 1 Occured. ";
					// 	}
					// 	else {
					// 		filter_status_msg.message = filter_status_msg.message + "7. Event 1 NOT Occured. ";
					// 	}
					// 	if (system_state_packet.filter_status.b.event2_flag) {
					// 		filter_status_msg.level = 1; // WARN state
					// 		filter_status_msg.message = filter_status_msg.message + "8. Event 2 Occured. ";
					// 	}
					// 	else {
					// 		filter_status_msg.message = filter_status_msg.message + "8. Event 2 NOT Occured. ";
					// 	}
					// 	if (system_state_packet.filter_status.b.internal_gnss_enabled) {
					// 		filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS Enabled. ";
					// 	}
					// 	else {
					// 		filter_status_msg.level = 1; // WARN state
					// 		filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS NOT Enabled. ";
					// 	}
					// 	if (system_state_packet.filter_status.b.magnetic_heading_enabled) {
					// 		filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading Active. ";
					// 	}
					// 	else {
					// 		filter_status_msg.level = 1; // WARN state
					// 		filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading NOT Active. ";
					// 	}
					// 	if (system_state_packet.filter_status.b.velocity_heading_enabled) {
					// 		filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading Enabled. ";
					// 	}
					// 	else {
					// 		filter_status_msg.level = 1; // WARN state
					// 		filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading NOT Enabled. ";
					// 	}
					// 	if (system_state_packet.filter_status.b.atmospheric_altitude_enabled) {
					// 		filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude Enabled. ";
					// 	}
					// 	else {
					// 		filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude NOT Enabled. ";
					// 		filter_status_msg.level = 1; // WARN state
					// 	}
					// 	if (system_state_packet.filter_status.b.external_position_active) {
					// 		filter_status_msg.message = filter_status_msg.message + "13. External Position Active. ";
					// 	}
					// 	else {
					// 		filter_status_msg.level = 1; // WARN state
					// 		filter_status_msg.message = filter_status_msg.message + "13. External Position NOT Active. ";
					// 	}
					// 	if (system_state_packet.filter_status.b.external_velocity_active) {
					// 		filter_status_msg.message = filter_status_msg.message + "14. External Velocity Active. ";
					// 	}
					// 	else {
					// 		filter_status_msg.level = 1; // WARN state
					// 		filter_status_msg.message = filter_status_msg.message + "14. External Velocity NOT Active. ";
					// 	}
					// 	if (system_state_packet.filter_status.b.external_heading_active) {
					// 		filter_status_msg.message = filter_status_msg.message + "15. External Heading Active. ";
					// 	}
					// 	else {
					// 		filter_status_msg.level = 1; // WARN state
					// 		filter_status_msg.message = filter_status_msg.message + "15. External Heading NOT Active. ";
					// 	}
					}
				}

				imumsg.position_covariance={pow(system_state_packet.standard_deviation[1],2), 0.0, 0.0,
							0.0, pow(system_state_packet.standard_deviation[0],2), 0.0,
							0.0, 0.0, pow(system_state_packet.standard_deviation[2],2)};

				if (an_packet->id == packet_id_euler_orientation_standard_deviation)
				{
					if(decode_euler_orientation_standard_deviation_packet(&euler_orientation_standard_deviation_packet, an_packet) == 0)
					{
						// IMU
						imumsg.orientationCovariance[0] = euler_orientation_standard_deviation_packet.standard_deviation[0];
						imumsg.orientationCovariance[4] = euler_orientation_standard_deviation_packet.standard_deviation[1];
						imumsg.orientationCovariance[8] = euler_orientation_standard_deviation_packet.standard_deviation[2];
					}
				
				}

				//testing
				if (an_packet->id == packet_id_velocity_standard_deviation)
				{
					if(decode_velocity_standard_deviation_packet(&velocity_standard_deviation_packet, an_packet) == 0)
					{
						// IMU
						imumsg.linearAccelerationCovariance[0] = velocity_standard_deviation_packet.standard_deviation[0];
						imumsg.linearAccelerationCovariance[4] = velocity_standard_deviation_packet.standard_deviation[1];
						imumsg.linearAccelerationCovariance[8] = velocity_standard_deviation_packet.standard_deviation[2];
					}
				}







				// Ensure that you free the an_packet when your done with it //
				// or you will leak memory                                   //
				an_packet_free(&an_packet);

				// Publish messages //
				IMUGPSpublisher.publish(imumsg);
				// system_status_pub.publish(system_status_msg);
				// filter_status_pub.publish(filter_status_msg);
			}
			// IMUGPSpublisher.publish(imumsg);
		}
	}

}
