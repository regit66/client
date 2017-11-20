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

#define PORT 20000
#define LENGTH 512 
//#define DEBUG 1

static knet_dev_t * dsPic; // robot pic microcontroller access

int maxsp,accinc,accdiv,minspacc, minspdec; // for speed profile

static int quitReq = 0; // quit variable for loop


void error(const char *msg)
{
	perror(msg);
	exit(1);
}



/*--------------------------------------------------------------------*/
/*!
 * Make sure the program terminate properly on a ctrl-c
 */
static void ctrlc_handler( int sig ) 
{
  quitReq = 1;
  
  kh4_set_speed(0 ,0 ,dsPic); // stop robot
  kh4_SetMode( kh4RegIdle,dsPic );
  
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy
  
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
long long
timeval_diff(struct timeval *difference,
             struct timeval *end_time,
             struct timeval *start_time
            )
{
  struct timeval temp_diff;

  if(difference==NULL)
  {
    difference=&temp_diff;
  }

  difference->tv_sec =end_time->tv_sec -start_time->tv_sec ;
  difference->tv_usec=end_time->tv_usec-start_time->tv_usec;

  /* Using while instead of if below makes the code slightly more robust. */

  while(difference->tv_usec<0)
  {
    difference->tv_usec+=1000000;
    difference->tv_sec -=1;
  }

  return 1000000LL*difference->tv_sec+
                   difference->tv_usec;

} /* timeval_diff() */

/*!
/*--------------------------------------------------------------------*/

// #define for driver mode
#define BIG_SPEED_FACTOR 25
#define SPEED_FACTOR 1
#define MAX_SPEED 1500
#define MIN_SPEED 15
#define DEFAULT_SPEED 200
#define ROTATE_HIGH_SPEED_FACT 0.5
#define ROTATE_LOW_SPEED_FACT 0.75
#define ROT_SPEED_HIGH_TRESH 300
#define STOP_TIME 100000 // us

#define SIGN(x) ((x)>0?1:((x)<0?-1:0))  // sign or zero


/*!
 * Drive the robot with the keyboard
 *
 * \param none
 *
 * \return an error code:
 *         - <0  standard error code. See errno.h
 *         - >=0 on success
 *
 */
int drive_robot()
{
	int out=0,speed=DEFAULT_SPEED,vsl,vsr,anymove=0;
	char c;
	struct timeval startt,endt;

	
	kb_clrscr(); // erase screen
	
	printf("Drive the robot with the keyboard:\n  's' for stop\n  arrows (UP, DOWN, LEFT , RIGHT) for direction\n  PAGE UP/DOWN for changing speed  by small increments\n  Home/End for changing speed by big increments\n  'q' for going back to main menu\n");
	
	
	printf("\ndefault parameters:\n  robot speed %d  (%5.1f mm/s)  (min %d, max %d)\n\n",DEFAULT_SPEED,DEFAULT_SPEED*KH4_SPEED_TO_MM_S,MIN_SPEED,MAX_SPEED);
	
	kb_change_term_mode(1); // change terminal mode for kbhit and getchar to return immediately
	

	kh4_SetMode(kh4RegSpeed,dsPic );
	
	gettimeofday(&startt,0x0);
	
	// loop until 'q' is pushed
	while(!out)
	{
		if(kb_kbhit())
		{
			c=getchar();


			// get special keys
			if (c== 27  ) 
			{
			
			 if (c=getchar()==91) // escape with [
			 {
				 c = getchar(); 
			 
				 switch(c)
				 {
					case 65: // UP arrow = forward
							 kh4_set_speed(speed ,speed,dsPic );
							anymove=1;						
					break;
					case 66: // DOWN arrow = backward			
							 kh4_set_speed(-speed ,-speed,dsPic  );
							anymove=1;
					break;

					case 68: // LEFT arrow = left
							if (speed > ROT_SPEED_HIGH_TRESH) // at high speed, rotate too fast
								 kh4_set_speed(-speed*ROTATE_HIGH_SPEED_FACT ,speed*ROTATE_HIGH_SPEED_FACT ,dsPic );
							else
								 kh4_set_speed(-speed*ROTATE_LOW_SPEED_FACT ,speed*ROTATE_LOW_SPEED_FACT ,dsPic );
							anymove=1;	
					break;

					case 67: // RIGHT arrow = right
							if (speed > ROT_SPEED_HIGH_TRESH) // at high speed, rotate too fast
								 kh4_set_speed(speed*ROTATE_HIGH_SPEED_FACT ,-speed*ROTATE_HIGH_SPEED_FACT ,dsPic );
							else
								 kh4_set_speed(speed*ROTATE_LOW_SPEED_FACT ,-speed*ROTATE_LOW_SPEED_FACT ,dsPic );
							anymove=1;	
					break;

					case 53: // PAGE UP  = speed up
						speed+=SPEED_FACTOR;
				 		if (speed>MAX_SPEED)
				 		{
							speed=MAX_SPEED;
				 		};
				 		c = getchar(); // get last character
				 		
				 		 kh4_get_speed(&vsl,&vsr,dsPic );
				 		 kh4_set_speed(SIGN(vsl)*speed ,SIGN(vsr)*speed ,dsPic ); // set new speed, keeping direction with sign
				 		printf("\033[1`\033[Krobot speed: %d (%5.1f mm/s)",speed,speed*KH4_SPEED_TO_MM_S); // move cursor to first column, erase line and print info
				 		fflush(stdout); // make the display refresh
				 		anymove=1;
					break;

					case 54: // PAGE DOWN = speed down
						speed-=SPEED_FACTOR;
				 		if (speed<MIN_SPEED)
				 		{
							speed=MIN_SPEED;
				 		};
				 		c = getchar(); // get last character
				 		
				 		kh4_get_speed(&vsl,&vsr,dsPic );
				 		kh4_set_speed(SIGN(vsl)*speed ,SIGN(vsr)*speed,dsPic  ); // set new speed, keeping direction with sign
				 		printf("\033[1`\033[Krobot speed: %d (%5.1f mm/s)",speed,speed*KH4_SPEED_TO_MM_S); // move cursor to first column, erase line and print info
				 		fflush(stdout); // make the display refresh
				 		anymove=1;
					break;
			

					default:
					break;
					} // switch(c)
				} // escape with [
				else
				{ // other special key code
					
					 c = getchar(); 
					 
					switch(c){
				
						case 72: // Home  = speed up
							speed+=BIG_SPEED_FACTOR;
					 		if (speed>MAX_SPEED)
					 		{
								speed=MAX_SPEED;
					 		};
					 		//c = getchar(); // get last character
					 		
					 		 kh4_get_speed(&vsl,&vsr,dsPic );
					 		 kh4_set_speed(SIGN(vsl)*speed ,SIGN(vsr)*speed ,dsPic ); // set new speed, keeping direction with sign
					 		printf("\033[1`\033[Krobot speed: %d (%5.1f mm/s)",speed,speed*KH4_SPEED_TO_MM_S); // move cursor to first column, erase line and print info
					 		fflush(stdout); // make the display refresh
					 		anymove=1;
						break;

						case 70: // End = speed down
							speed-=BIG_SPEED_FACTOR;
					 		if (speed<MIN_SPEED)
					 		{
								speed=MIN_SPEED;
					 		};
					 		//c = getchar(); // get last character
					 		
					 		kh4_get_speed(&vsl,&vsr,dsPic );
					 		kh4_set_speed(SIGN(vsl)*speed ,SIGN(vsr)*speed,dsPic  ); // set new speed, keeping direction with sign
					 		printf("\033[1`\033[Krobot speed: %d (%5.1f mm/s)",speed,speed*KH4_SPEED_TO_MM_S); // move cursor to first column, erase line and print info
					 		fflush(stdout); // make the display refresh
					 		anymove=1;
						break;
						
						default:
						break	;	
						
					}  
			
				} // ether special key code
							
				
			} // if (c== '\027')	 
			else 
			{
				switch(c)
				{
				 	case 'q': // quit to main menu
				 		out=1;
				   	break;
					case 's': // stop motor
						 kh4_set_speed(0,0,dsPic);
					break;
				   
				 	default:
				   break;
				}
		  }
		  
		  gettimeofday(&startt,0x0);
		} else
		{
		
			gettimeofday(&endt,0x0);;
			// stop when no key is pushed after some time
			
			if (anymove &&  (timeval_diff(NULL,&endt,&startt)>STOP_TIME))
			{
				 kh4_set_speed(0 ,0,dsPic );
				anymove=0;
			}	
				
		}
		

		usleep(10000); // wait some ms
	} // while

	kb_change_term_mode(0); // switch to normal key input mode	
	kh4_set_speed(0,0,dsPic );	 // stop robot
	kh4_SetMode(kh4RegIdle,dsPic );
	return 0;
}



/*--------------------------------------------------------------------*/
/*!
 * Main
 */
int main(int argc , char * argv[]) 
{ 
 
#define IR_BAR_LEN 15 	// display bar length for IR sensor
#define US_BAR_LEN 23 	// display bar length for US sensor
#define ACGY_BAR_LEN 30 // display bar length for Accel/gyro sensor
#define MAX_US_DISTANCE 250.0 // max distance US
#define MAX_G 2 		// max acceleration in g

// convert US value to text comment
#define US_VAL(val) ((val)==KH4_US_DISABLED_SENSOR ? "Not activated" : ((val)==KH4_US_NO_OBJECT_IN_RANGE ? "No object in range" : ((val)==KH4_US_OBJECT_NEAR ? "Object at less than 25cm" : "Object in range 25..250cm")))

  double fpos,dval,dmean;
  long lpos,rpos;
  char Buffer[100],bar[12][64],revision,version;
  int i,n,type_of_test=0,sl,sr,pl,pr;
  short index, value,sensors[12],usvalues[5];
  char c;
  long motspeed;
  char line[80],l[9];
  int kp,ki,kd;
  int pmarg;
  
  // initiate libkhepera and robot access
  if ( kh4_init(argc ,argv)!=0)
  {
  	printf("\nERROR: could not initiate the libkhepera!\n\n");
  	return -1;
  }	

  /* open robot socket and store the handle in their respective pointers */
  dsPic  = knet_open( "Khepera4:dsPic" , KNET_BUS_I2C , 0 , NULL );

	if ( dsPic==NULL)
  {
  	printf("\nERROR: could not initiate communication with Kh4 dsPic\n\n");
  	return -2;
  }	

  /* initialize the motors controlers*/
   
  /* tuned parameters */
  pmarg=20;
  kh4_SetPositionMargin(pmarg,dsPic ); 				// position control margin
  kp=10;
  ki=5;
  kd=1;
  kh4_ConfigurePID( kp , ki , kd,dsPic  ); 		// configure P,I,D
  
  accinc=3;//3;
  accdiv=0;
  minspacc=20;
  minspdec=1;
  maxsp=400;
  // configure acceleration slope
  kh4_SetSpeedProfile(accinc,accdiv,minspacc, minspdec,maxsp,dsPic ); // Acceleration increment ,  Acceleration divider, Minimum speed acc, Minimum speed dec, maximum speed
  
	kh4_SetMode( kh4RegIdle,dsPic );  				// Put in idle mode (no control)

  // get revision
  if(kh4_revision(Buffer, dsPic)==0){
   	version=(Buffer[0]>>4) +'A';
  	revision=Buffer[0] & 0x0F; 
    printf("\r\nVersion = %c, Revision = %u\r\n",version,revision);        
  }
  

// drive_robot();
  /* Variable Definition */
	int sockfd; 
	int nsockfd;
	char revbuf[LENGTH]; 
	struct sockaddr_in remote_addr;
    	char message[1000] , server_reply[2000];
	/* Get the Socket file descriptor */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{
		fprintf(stderr, "ERROR: Failed to obtain Socket Descriptor! (errno = %d)\n",errno);
                kh4_SetRGBLeds(1,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy
		exit(1);
	}

	/* Fill the socket address struct */
	remote_addr.sin_family = AF_INET; 
	remote_addr.sin_port = htons(PORT); 
	inet_pton(AF_INET, "192.168.1.105", &remote_addr.sin_addr); 
	bzero(&(remote_addr.sin_zero), 8);

	/* Try to connect the remote */
	if (connect(sockfd, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr)) == -1)
	{
		fprintf(stderr, "ERROR: Failed to connect to the host! (errno = %d)\n",errno);
		exit(1);
	}
	else 
		printf("[Client] Connected to server at port %d...ok!\n", PORT);
kh4_SetRGBLeds(0,1,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy

	/* Send File to Server */
	//if(!fork())
	//{
    
 	if( recv(sockfd , server_reply , 2000 , 0) < 0)
        {
            puts("recv failed");
         
        }
        puts("Server reply :");
        puts(server_reply);


if (server_reply[0]=='t')

{
int status = system("./move");

}
///////////////
//Send some data
printf("Enter message : ");
        scanf("%s" , message);
        if( send(sockfd , message , strlen(message) , 0) < 0)
        {
            puts("Send failed");
            return 1;
        }
puts("Data Send");

if( recv(sockfd , server_reply , 2000 , 0) < 0)
        {
            puts("recv failed");
         
        }
        puts("Server reply :");
        puts(server_reply);




/////////
printf("Enter message : ");
        scanf("%s" , message);
        if( send(sockfd , message , strlen(message) , 0) < 0)
        {
            puts("Send failed");
            return 1;
        }
puts("Data Send");
/* 

		char* fs_name = "/home/user/Desktop/9.txt";
		char sdbuf[LENGTH]; 
		printf("[Client] Sending %s to the Server... ", fs_name);
		FILE *fs = fopen(fs_name, "r");
		if(fs == NULL)
		{
			printf("ERROR: File %s not found.\n", fs_name);
			exit(1);
		}

		bzero(sdbuf, LENGTH); 
		int fs_block_sz; 
		while((fs_block_sz = fread(sdbuf, sizeof(char), LENGTH, fs)) > 0)
		{
		    if(send(sockfd, sdbuf, fs_block_sz, 0) < 0)
		    {
		        fprintf(stderr, "ERROR: Failed to send file %s. (errno = %d)\n", fs_name, errno);
		        break;
		    }
		    bzero(sdbuf, LENGTH);
		}
		printf("Ok File %s from Client was Sent!\n", fs_name);
\*
	//}

	/* Receive File from Server 
	printf("[Client] Receiveing file from Server and saving it as final.txt...");
	char* fr_name = "/home/user/Desktop/final.txt";
	FILE *fr = fopen(fr_name, "a");
	if(fr == NULL)
		printf("File %s Cannot be opened.\n", fr_name);
	else
	{
		bzero(revbuf, LENGTH); 
		int fr_block_sz = 0;
	    while((fr_block_sz = recv(sockfd, revbuf, LENGTH, 0)) > 0)
	    {
			int write_sz = fwrite(revbuf, sizeof(char), fr_block_sz, fr);
	        if(write_sz < fr_block_sz)
			{
	            error("File write failed.\n");
	        }
			bzero(revbuf, LENGTH);
			if (fr_block_sz == 0 || fr_block_sz != 512) 
			{
				break;
			}
		}
		if(fr_block_sz < 0)
        {
			if (errno == EAGAIN)
			{
				printf("recv() timed out.\n");
			}
			else
			{
				fprintf(stderr, "recv() failed due to errno = %d\n", errno);
			}
		}
	    printf("Ok received from server!\n");
	    fclose(fr);

	}
*/
	close (sockfd);
	printf("[Client] Connection lost.\n");
	
	
  kh4_set_speed(0 ,0 ,dsPic); // stop robot
  kh4_SetMode( kh4RegIdle,dsPic ); // set motors to idle
  kh4_SetRGBLeds(1,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy
  
	return 0;
}
