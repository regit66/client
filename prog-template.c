/* \file khepera4.c

 *
 * \brief
 *         This is the big application example for the Khepera4
 *
 *
 * \author   Julien Tharin (K-Team SA)
 *
 * \note     Copyright (C) 2013 K-TEAM SA
 * \bug      none discovered.
 * \todo     nothing.

 * compile with command (don't forget to source the env.sh of your development folder!):
 arm-angstrom-linux-gnueabi-gcc kh4test.c -o khepera4_test -I $INCPATH -L $LIBPATH -lkhepera


 */
#include <khepera/khepera.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <signal.h>
#include <ctype.h>
#include <arpa/inet.h>
#include <netdb.h>

#define ROTATE_HIGH_SPEED_FACT 0.5
#define PORT 20000
#define LENGTH 4096
//#define DEBUG 1

static knet_dev_t * dsPic; // robot pic microcontroller access

int maxsp, accinc, accdiv, minspacc, minspdec; // for speed profile

static int quitReq = 0; // quit variable for loop

void error(const char *msg) {
	perror(msg);
	exit(1);
}

/*--------------------------------------------------------------------*/
/*!
 * Make sure the program terminate properly on a ctrl-c
 */
static void ctrlc_handler(int sig) {
	quitReq = 1;

	kh4_set_speed(0, 0, dsPic); // stop robot
	kh4_SetMode(kh4RegIdle, dsPic);

	kh4_SetRGBLeds(0, 0, 0, 0, 0, 0, 0, 0, 0, dsPic); // clear rgb leds because consumes energy

	kb_change_term_mode(0); // revert to original terminal if called

	exit(0);
}
/*!
 * Compute time difference
 *

 * \param difference difference between the two times, in structure timeval type
 * \param end_time end time
 * \param start_time start time
 *
 * \return difference between the two times in [us]
 *
 */
long long timeval_diff(struct timeval *difference, struct timeval *end_time,
		struct timeval *start_time) {
	struct timeval temp_diff;

	if (difference == NULL) {
		difference = &temp_diff;
	}

	difference->tv_sec = end_time->tv_sec - start_time->tv_sec;
	difference->tv_usec = end_time->tv_usec - start_time->tv_usec;

	/* Using while instead of if below makes the code slightly more robust. */

	while (difference->tv_usec < 0) {
		difference->tv_usec += 1000000;
		difference->tv_sec -= 1;
	}

	return 1000000LL * difference->tv_sec + difference->tv_usec;

} /* timeval_diff() */
void proximitySensor(int i, char *Buffer, short *sensors, char* fs_name);
void uaSensor(int i, char *Buffer, short *usvalues, char* fs_name);
void ambientSensor(int i, char *Buffer, short *sensors, char* fs_name);
void mottorSensor(char *Buffer, char* fs_name);
void batterySensor(char *Buffer, char* fs_name);
void go(int num1, int num2, double rotate);
/*--------------------------------------------------------------------*/
/*!
 * Main
 */
int main(int argc, char * argv[]) {

#define IR_BAR_LEN 15 	// display bar length for IR sensor
#define US_BAR_LEN 23 	// display bar length for US sensor
#define ACGY_BAR_LEN 30 // display bar length for Accel/gyro sensor
#define MAX_US_DISTANCE 250.0 // max distance US
#define MAX_G 2 		// max acceleration in g

// convert US value to text comment
#define US_VAL(val) ((val)==KH4_US_DISABLED_SENSOR ? "Not activated" : ((val)==KH4_US_NO_OBJECT_IN_RANGE ? "No object in range" : ((val)==KH4_US_OBJECT_NEAR ? "Object at less than 25cm" : "Object in range 25..250cm")))

	double fpos, dval, dmean;
	long lpos, rpos;
	char Buffer[100], bar[12][64], revision, version;
	int i, n, type_of_test = 0, sl, sr, pl, pr;
	short index, value, sensors[12], usvalues[5];
	char c;
	int motorSpeed = 100;
	char line[80], l[9];
	int kp, ki, kd;
	int pmarg;
	char* fs_name = "data.csv";

	// initiate libkhepera and robot access
	if (kh4_init(argc, argv) != 0) {
		printf("\nERROR: could not initiate the libkhepera!\n\n");
		return -1;
	}

	/* open robot socket and store the handle in their respective pointers */
	dsPic = knet_open("Khepera4:dsPic", KNET_BUS_I2C, 0, NULL);

	if (dsPic == NULL) {
		printf("\nERROR: could not initiate communication with Kh4 dsPic\n\n");
		return -2;
	}

	/* initialize the motors controlers*/

	/* tuned parameters */
	pmarg = 20;
	kh4_SetPositionMargin(pmarg, dsPic); 			// position control margin
	kp = 10;
	ki = 5;
	kd = 1;
	kh4_ConfigurePID(kp, ki, kd, dsPic); 		// configure P,I,D

	accinc = 3; 		//3;
	accdiv = 0;
	minspacc = 20;
	minspdec = 1;
	maxsp = 400;
	// configure acceleration slope
	kh4_SetSpeedProfile(accinc, accdiv, minspacc, minspdec, maxsp, dsPic); // Acceleration increment ,  Acceleration divider, Minimum speed acc, Minimum speed dec, maximum speed

	kh4_SetMode(kh4RegIdle, dsPic);  			// Put in idle mode (no control)

	// get revision
	if (kh4_revision(Buffer, dsPic) == 0) {
		version = (Buffer[0] >> 4) + 'A';
		revision = Buffer[0] & 0x0F;
		printf("\r\nVersion = %c, Revision = %u\r\n", version, revision);
	}

	/* Variable Definition */
	int sockfd;
	struct sockaddr_in remote_addr;
	char message[1000], server_reply[2000];
	/* Get the Socket file descriptor */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		fprintf(stderr,
				"ERROR: Failed to obtain Socket Descriptor! (errno = %d)\n",
				errno);
		kh4_SetRGBLeds(1, 0, 0, 0, 0, 0, 0, 0, 0, dsPic); // clear rgb leds because consumes energy
		exit(1);
	}

	/* Fill the socket address struct */
	remote_addr.sin_family = AF_INET;
	remote_addr.sin_port = htons(PORT);
	inet_pton(AF_INET, "192.168.1.117", &remote_addr.sin_addr);
	bzero(&(remote_addr.sin_zero), 8);

	/* Try to connect the remote */
	if (connect(sockfd, (struct sockaddr *) &remote_addr,
			sizeof(struct sockaddr)) == -1) {
		fprintf(stderr, "ERROR: Failed to connect to the host! (errno = %d)\n",
		errno);
		exit(1);
	} else
		printf("[Client] Connected to server at port %d...ok!\n", PORT);
	kh4_SetRGBLeds(0, 0, 0, 0, 0, 0, 0, 1, 0, dsPic); // enable green diode when connect

	// Initialize camera
	system("./aa.sh &");

	//keep communicating with server
	while (1) {

		kh4_battery_status(Buffer, dsPic);

		int battery = Buffer[3];
		//printf("%3d",Buffer[3]);
		sprintf(message, "%d", battery);

		if (recv(sockfd, server_reply, 2000, 0) < 0) {
			puts("recv failed");
			break;
		}

		puts("Server reply :");
		puts(server_reply);

		if (strcmp(server_reply, "stop") == 0) {
			printf("stop");
			kh4_set_speed(0, 0, dsPic); // stop robot
			kh4_SetMode(kh4RegIdle, dsPic); // set motors to idle

		}

		if (strcmp(server_reply, "up") == 0) {
			printf("przod");
			go(motorSpeed, motorSpeed, 1);

		}
		if (strcmp(server_reply, "down") == 0) {
			printf("tyl");
			go(-motorSpeed, -motorSpeed, 1);

		}
		if (strcmp(server_reply, "left") == 0) {

			printf("lewo");
			go(-motorSpeed, motorSpeed, ROTATE_HIGH_SPEED_FACT);
		}
		if (strcmp(server_reply, "right") == 0) {
			printf("prawo");
			go(motorSpeed, -motorSpeed, ROTATE_HIGH_SPEED_FACT);
		}
		if (strcmp(server_reply, "speed") == 0) {
			printf("speed");
			memset(server_reply, 0, 255);
			//sET SPEED
			if (send(sockfd, message, strlen(message), 0) < 0) {
				puts("Send failed");
				return 1;
			}

			if (recv(sockfd, server_reply, 2000, 0) < 0) {
				puts("recv failed");
				break;
			}

			sscanf(server_reply, "%d", &motorSpeed);

			memset(server_reply, 0, 255);

		}


		if (strcmp(server_reply, "diode") == 0) {
			printf("diode");
			memset(server_reply, 0, 255);
			//sET SPEED
			if (send(sockfd, message, strlen(message), 0) < 0) {
				puts("Send failed");
				return 1;
			}

			if (recv(sockfd, server_reply, 2000, 0) < 0) {
				puts("recv failed");
				break;
			}

			if (strcmp(server_reply, "red") == 0) {
			//printf("set red diode");
			
			kh4_SetRGBLeds(1, 0, 0, 0, 0, 0, 0, 0, 0, dsPic);
			}
			if (strcmp(server_reply, "blue") == 0) {
			//printf("set red diode");
			
			kh4_SetRGBLeds(0, 0, 1, 0, 0, 0, 0, 0, 0, dsPic);
			}
			if (strcmp(server_reply, "yellow") == 0) {
			//printf("set red diode");
			
			kh4_SetRGBLeds(63, 63, 0, 0, 0, 0, 0, 0, 0, dsPic);
			}
			if (strcmp(server_reply, "pink") == 0) {
			//printf("set red diode");
			
			kh4_SetRGBLeds(30, 0, 10, 0, 0, 0, 0, 0, 0, dsPic);
			}
			if (strcmp(server_reply, "purple") == 0) {
			//printf("set red diode");
			
			kh4_SetRGBLeds(20, 0, 40, 0, 0, 0, 0, 0, 0, dsPic);
			}
			if (strcmp(server_reply, "orange") == 0) {
			//printf("set red diode");
			
			kh4_SetRGBLeds(63, 20, 0, 0, 0, 0, 0, 0, 0, dsPic);
			}
			if (strcmp(server_reply, "green") == 0) {
			//printf("set red diode");
			
			kh4_SetRGBLeds(0, 1, 0, 0, 0, 0, 0, 0, 0, dsPic);
			}
			if (strcmp(server_reply, "white") == 0) {
			//printf("set red diode");
			
			kh4_SetRGBLeds(60, 60, 60, 0, 0, 0, 0, 0, 0, dsPic);
			}

			if (send(sockfd, message, strlen(message), 0) < 0) {
				puts("Send failed");
				return 1;
			}
		}




		if (strcmp(server_reply, "file") == 0) {

			//send file

			proximitySensor(i, Buffer, sensors, fs_name);
			uaSensor(i, Buffer, usvalues, fs_name);
			ambientSensor(i, Buffer, sensors, fs_name);
			mottorSensor(Buffer, fs_name);
			batterySensor(Buffer, fs_name);

			char sdbuf[LENGTH];
			printf("[Client] Sending %s to the Server... ", fs_name);
			FILE *fs = fopen(fs_name, "r");
			if (fs == NULL) {
				printf("ERROR: File %s not found.\n", fs_name);
				break;
			}

			bzero(sdbuf, LENGTH);
			int fs_block_sz;
			while ((fs_block_sz = fread(sdbuf, sizeof(char), LENGTH, fs)) > 0) {
				if (send(sockfd, sdbuf, fs_block_sz, 0) < 0) {
					fprintf(stderr,
							"ERROR: Failed to send file %s. (errno = %d)\n",
							fs_name, errno);
					break;
				}
				bzero(sdbuf, LENGTH);
			}
			printf("Ok File %s from Client was Sent!\n", fs_name);

		}

		kb_clrscr();

//Send some data
		if (send(sockfd, message, strlen(message), 0) < 0) {
			puts("Send failed");
			return 1;
		}

		memset(server_reply, 0, 255);

	}

	close(sockfd);
	printf("[Client] Connection lost.\n");

	kh4_set_speed(0, 0, dsPic); // stop robot
	kh4_SetMode(kh4RegIdle, dsPic); // set motors to idle
	kh4_SetRGBLeds(0, 0, 0, 0, 0, 0, 1, 0, 0, dsPic); // clear rgb leds because consumes energy

	return 0;
}

void proximitySensor(int i, char *Buffer, short *sensors, char* fs_name) {

	FILE *file = fopen(fs_name, "w+");
	if (file == NULL) {
		printf("Error opening file!\n");
		exit(1);
	}
	kh4_proximity_ir(Buffer, dsPic);
	for (i = 0; i < 12; i++) {
		sensors[i] = (Buffer[i * 2] | Buffer[i * 2 + 1] << 8);

	}
	fprintf(file,
			"Proximity Sensors\ 
			\nback left :; %4u; \nleft :; %4u\
 \nfront left :; %4u; \nfront :; %4u\
 \nfront right :; %4u; \nright :; %4u\
 \nback right :; %4u; \nback :; %4u\
 \nground left :; %4u; \ngnd front left :; %4u\
 \ngnd front right:; %4u; \nground right :; %4u\n",
			sensors[0], sensors[1], sensors[2], sensors[3], sensors[4],
			sensors[5], sensors[6], sensors[7], sensors[8], sensors[9],
			sensors[10], sensors[11]);

	fclose(file);
}

void uaSensor(int i, char *Buffer, short *usvalues, char* fs_name) {

	FILE *file = fopen(fs_name, "a+");
	if (file == NULL) {
		printf("Error opening file!\n");
		exit(1);
	}
	kh4_measure_us(Buffer, dsPic);
	for (i = 0; i < 5; i++) {
		usvalues[i] = (short) (Buffer[i * 2] | Buffer[i * 2 + 1] << 8);

	}
	fprintf(file, "\n");
	fprintf(file,
			"\nUS sensors : distance [cm]\
							  \nleft 90:; %4d;\nleft 45:; %4d;\
							  \nfront:; %4d;\nright 45:; %4d;\nright 90:; %4d;\n",
			usvalues[0], usvalues[1], usvalues[2], usvalues[3], usvalues[4]);
	fclose(file);
}

void ambientSensor(int i, char *Buffer, short *sensors, char* fs_name) {

	FILE *file = fopen(fs_name, "a+");
	if (file == NULL) {
		printf("Error opening file!\n");
		exit(1);
	}
	kh4_ambiant_ir(Buffer, dsPic);
	for (i = 0; i < 12; i++) {
		sensors[i] = (Buffer[i * 2] | Buffer[i * 2 + 1] << 8);

	}
	fprintf(file, "\n");
	fprintf(file,
			"Ambiant Sensors\
						 \nback left      :; %4.4u;  \nleft           :; %4.4u\
						 \nfront left     :; %4.4u;  \nfront          :;%4.4u\
						 \nfront right    :; %4.4u;  \nright          :; %4.4u\
						 \nback right     :; %4.4u;  \nback           :; %4.4u\
						 \nground left    :; %4.4u;  \ngnd front left :; %4.4u\
						 \ngnd front right:; %4.4u;  \nground right   :; %4.4u\n",
			sensors[0], sensors[1], sensors[2], sensors[3], sensors[4],
			sensors[5], sensors[6], sensors[7], sensors[8], sensors[9],
			sensors[10], sensors[11]);
	fclose(file);
}

void mottorSensor(char *Buffer, char* fs_name) {

	FILE *file = fopen(fs_name, "a+");
	if (file == NULL) {
		printf("Error opening file!\n");
		exit(1);
	}

	int sl, sr, pl, pr;
	kh4_get_speed(&sl, &sr, dsPic);
	kh4_get_position(&pl, &pr, dsPic);
	fprintf(file, "\nmotor speed and position");
	fprintf(file,
			"motors speed [mm/s (pulse/)]:; left:; %7.1f;  (%5d)  | right:; %7.1f; (%5d)\n",
			sl * KH4_SPEED_TO_MM_S, sl, sr * KH4_SPEED_TO_MM_S, sr);
	fprintf(file,
			"motors position [mm (pulse)]:; left:; %7.1f; (%7d) | right:; %7.1f; (%7d)\n",
			pl * KH4_PULSE_TO_MM, pl, pr * KH4_PULSE_TO_MM, pr);

	fclose(file);
}

void batterySensor(char *Buffer, char* fs_name) {

	FILE *file = fopen(fs_name, "a+");
	if (file == NULL) {
		printf("Error opening file!\n");
		exit(1);
	}

	kh4_battery_status(Buffer, dsPic);
	fprintf(file, "\n");
	fprintf(file, "Battery:\n  status (DS2781)   :;  0x%x\n", Buffer[0]);
	fprintf(file, "  remaining capacity:;  %4.0f mAh\n",
			(Buffer[1] | Buffer[2] << 8) * 1.6);
	fprintf(file, "  remaining capacity:;   %3d %%\n", Buffer[3]);
	fprintf(file, "  current           :; %4.0f mA\n",
			(short) (Buffer[4] | Buffer[5] << 8) * 0.07813);
	fprintf(file, "  average current   :;  %4.0f mA\n",
			(short) (Buffer[6] | Buffer[7] << 8) * 0.07813);
	fprintf(file, "  temperature       :;  %3.1f C \n",
			(short) (Buffer[8] | Buffer[9] << 8) * 0.003906);
	fprintf(file, "  voltage           :;  %4.0f mV \n",
			(Buffer[10] | Buffer[11] << 8) * 9.76);
	fprintf(file, "  charger           :;  %s\n",
			kh4_battery_charge(dsPic) ? "plugged" : "unplugged");

	fclose(file);
}
void go(int num1, int num2, double rotate) {

	kh4_SetMode(kh4RegSpeed, dsPic);
	kh4_set_speed(num1 * rotate, num2 * rotate, dsPic);
	//usleep(100000);
	//kh4_set_speed(0, 0, dsPic); // stop robot
	//kh4_SetMode(kh4RegIdle, dsPic); // set motors to idle

}
