all: imu

imu: rs232/rs232.c rs232/rs232.h an_packet_protocol.c an_packet_protocol.h spatial_packets.c spatial_packets.h imu.c imu.h imu_main.cpp
	g++ -I /pengujiannavigasi/matrix -pedantic -Wall -o bb rs232/rs232.c an_packet_protocol.c spatial_packets.c imu.c BB.cpp navigasinonlinirham.cpp
clean:
	rm imu
	