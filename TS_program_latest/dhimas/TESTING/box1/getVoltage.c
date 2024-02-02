#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
	uint8_t data[19];
	uint16_t vin;
	float vinf;
	int fd, i;

	fd = open("/dev/i2c-1", O_RDWR);
	assert (fd != -1);
	
	if (ioctl(fd, 0x0706, 0x2a) < 0) {
		perror("MCU did not ACK 0x2a\n");
		return -1;
	}

	bzero(data, 19);
	read(fd, data, 19);
	vin = (data[0]<<8)|data[1];
	vin = (vin*2500/1024); /* Convert to ADC mV (2500 mV ref, 10b ADC) */
//	vinf = vin*(1./.0414); /* Voltage divider before uC: 4.14% */
	vinf = vin*(5.0/305.0); /* Voltage divider before uC: 4.14% */
	printf("vin=%3.2f\t\n", vinf);
//	printf("vin_mv=%d\t\n", (uint16_t)vinf);
}
