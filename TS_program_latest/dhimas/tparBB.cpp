#include <iostream>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <fstream>

using namespace std;

char comBBheader[10];
char comBB_1[10];
char comBB_2[10];
char comBB_3[10];
char comBB_4[10];
char comBB_5[10];
char comBB_6[10];
char comBB_7[10];
char comBB_8[10];
char comBB_9[10];
char comBB_10[10];
char comBB_11[10];

char bn[140];
int data_entry = 0;
int data_limit = 100;

float teta_terukur, z_terukur, yawref, yaw_terukur, alt, roll, sal, temp, spd, lat, lon;

int main(void) {
	int BBcount;
	char * comBBin;
	char BBinbound[] = "$#NG 1.0000000 2.0000000 3.0000000 -4.000000 5.0000000 6.0000000 7.0000000 8.0000000 9.0000000 10.000000 11.000000";
	printf("Dummy beagle %s\n", BBinbound);
	comBBin = strtok(BBinbound, " ");
	while(comBBin != NULL)
	{
		switch(BBcount) {
			case 0 :
				strcpy(comBBheader, comBBin);
				printf("Header : %s\n", comBBheader);
				BBcount++;
				break;

			case 1 :
				{
					strcpy(comBB_1, comBBin);
					teta_terukur = atof(comBB_1);
					printf("Teta terukur : %09.6f\n", teta_terukur);
				}
				BBcount++;
				break;

			case 2 :
				{
					strcpy(comBB_2, comBBin);
					z_terukur = atof(comBB_2);
					printf("Z terukur : %05.2f\n", z_terukur);
				}
				BBcount++;
				break;

			case 3 :
				{
					strcpy(comBB_3, comBBin);
					yawref = atof(comBB_3);
					printf("Yaw ref : %09.6f\n", yawref);
				}
				BBcount++;
				break;

			case 4 :
				{
					strcpy(comBB_4, comBBin);
					yaw_terukur = atof(comBB_4);
					printf("Yaw terukur : %09.6f\n", yaw_terukur);
				}
				BBcount++;
				break;

			case 5 :
				{
					strcpy(comBB_5, comBBin);
					alt = atof(comBB_5);
					printf("Altitude : %05.2f\n", alt);
				}
				BBcount++;
				break;

			case 6 :
				{
					strcpy(comBB_6, comBBin);
					roll = atof(comBB_6);
					printf("Rolls : %09.6f\n", roll);
				}
				BBcount++;
				break;

			case 7 :
				{
					strcpy(comBB_7, comBBin);
					sal = atof(comBB_7);
					printf("Salinity : %05.2f\n", sal);
				}
				BBcount++;
				break;

			case 8 :
				{
					strcpy(comBB_8, comBBin);
					temp = atof(comBB_8);
					printf("Temperature : %05.2f\n", temp);
				}
				BBcount++;
				break;

			case 9 :
				{
					strcpy(comBB_9, comBBin);
					spd = atof(comBB_9);
					printf("Speed : %04.1f\n", spd);
				}
				BBcount++;
				break;

			case 10 :
				{
					strcpy(comBB_10, comBBin);
					lat = atof(comBB_10);
					printf("Latitude : %012.7f\n", lat);
				}
				BBcount++;
				break;
			
			case 11 :
				{
					strcpy(comBB_11, comBBin);
					lon =  atof(comBB_11);
					printf("Longitude : %012.7f\n", lon);
				}
				BBcount++;
				break;
				
			default :
				break;
		}
		comBBin = strtok(NULL, " ");
	}

	if(data_entry <= data_limit)
	{
		ofstream savedatalog("tes data BB.txt", ios::out | ios::app);
		if(savedatalog.is_open())
		{
			savedatalog << "$#NG " << teta_terukur << ' ' << z_terukur << ' ' << yawref << ' ' << yaw_terukur << ' ' << alt << ' ' << roll << ' ' << sal << ' ' << temp << ' ' << spd << ' ' << lat << ' ' << lon << ' ' << endl;
			cout << "save success" << endl;
			savedatalog.close();
		}
		data_entry++;
		cout << data_entry << endl;
	}

	else
	{
		// ofstream savedatalog("tes data BB.txt", ios::out | ios::trunc);
		// if(savedatalog.is_open())
		// {
		// 	savedatalog << "" << endl;
		// 	cout << "delete success" << endl;
		// 	savedatalog.close();
		// }
		// data_entry = 0;
		// cout << data_entry << endl;
	}
	return 0;
}
