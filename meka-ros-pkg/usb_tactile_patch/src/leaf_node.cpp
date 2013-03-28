#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include "ros/ros.h"
#include "usb_tactile_patch/UsbTactilePatch.h"
#include <sstream>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>

#define BUF_SIZE 200
#define NUM_SENSOR 12
//#define MSG_SIZE (NUM_SENSOR * 2 + 1)
#define MSG_SIZE 26

unsigned char shared_buf[BUF_SIZE];

int shared_tactiles[NUM_SENSOR];
int crc_errors;

//int shared_tactiles_last[NUM_SENSOR];
bool stop_thread;
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

/*
  * 'open_port()' - Open serial port 1.
  *
  * Returns the file descriptor on success or -1 on error.
  */

int
open_port(void)
{
  int fd; /* File descriptor for the port */


  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
  {
    /*
    * Could not open the port.
    */

    perror("open_port: Unable to open /dev/ttyS0 - ");
  }
  else
    fcntl(fd, F_SETFL, 0);

  return (fd);
}


void *read_serial_thread(void *threadid)
{   
   int fd = open_port();
   unsigned short tactiles[NUM_SENSOR];
   unsigned short their_crc;
   unsigned short my_crc;
   
   
   struct termios options;
   /*
     * Get the current options for the port...
     */
    tcgetattr(fd, &options);
    /*
     * Set the baud rates to 19200...
     */
    cfsetispeed(&options, B57600);
    cfsetospeed(&options, B57600);
    /*
     * Enable the receiver and set local mode...
     */
    options.c_cflag |= (CLOCAL | CREAD);
    /*  8N1 */
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    /*
     * Set the new options for the port...
     */        
    tcsetattr(fd, TCSANOW, &options);
            
    unsigned char buf[BUF_SIZE];
    unsigned char buf_msg[MSG_SIZE];    
    bool starting_read = false;
    bool finalizing_read = false;
    int bytes_read;
    int msg_buf_cnt = 0;
     int shared_buf_cnt = 0;
    crc_errors=0;
    // now read until told to stop
    while (!stop_thread)
    {
      bytes_read = read(fd, buf, BUF_SIZE);     	            
      if (bytes_read != -1)
      {
	  // test to just copy to shared buf..
	  /*pthread_mutex_lock( &mutex1 ); 	  	  
	  for (int i = 0; i < bytes_read; i++)
	  {
	    if (shared_buf_cnt < BUF_SIZE)
	    {
	      shared_buf[shared_buf_cnt] = buf[i];
	      shared_buf_cnt++;
	    }
	  }	  
	  if (shared_buf_cnt >= BUF_SIZE)	  
	  {
	    shared_buf_cnt = 0;
	     for (int i = 0; i < BUF_SIZE; i++)
	      shared_buf[i] = 77;
	  }
	  pthread_mutex_unlock( &mutex1 );*/	  			
	
	for (int i = 0; i < bytes_read; i++)
	{
	  if (!starting_read && !finalizing_read) // waiting for the start byte..
	  {
	    if (buf[i] == 115)
	    {
	      starting_read = true;
	    }
	  } else if (finalizing_read) { // we should now be on stop byte and then send it off if CRC checks
	    if (buf[i] == 102)
	    {	  
	      unsigned short value;
	      unsigned char * ptr;
	      ptr = (unsigned char*)(&value);
	      for (int i = 0; i < NUM_SENSOR; i++) // start parse the values...
	      {		  		  
		  ptr[0] = buf_msg[i*2];
		  ptr[1] = buf_msg[i*2+1];		  
		  tactiles[i] = value;			      	      
	      }
	      ptr[0] = buf_msg[NUM_SENSOR*2];
	      ptr[1] = buf_msg[NUM_SENSOR*2+1];	  
	      their_crc = value;	      
	      my_crc = 0;
	      for (int i = 0; i < NUM_SENSOR; i++)	      
		my_crc += tactiles[i];
	      //printf("%u %u\n", their_crc, my_crc);
	      // if passes CRC copy it over	      	      
	      if (my_crc == their_crc)
	      {
		pthread_mutex_lock( &mutex1 );
		for (int i = 0; i < NUM_SENSOR; i++)
		  shared_tactiles[i] = tactiles[i];
		pthread_mutex_unlock( &mutex1 );
	      }	    
	      else
		crc_errors++;
	    }
	    finalizing_read = false;
	  } else if (starting_read) {	    
	    buf_msg[msg_buf_cnt] = buf[i];
	    msg_buf_cnt++;	    
	    if (msg_buf_cnt == MSG_SIZE)
	    {
	      msg_buf_cnt = 0;
	      starting_read = false;
	      finalizing_read = true;
	    }
	  }
	}
      } else {
	printf("error reading usb\n");
      }
    }
    
    close(fd);
   
   pthread_exit(NULL);
}

void sigproc(int sig)
{ 		   
  stop_thread = true;
}

int main(int argc, char **argv)
{
//  int startup_cnt=100;
  for (int i = 0; i < NUM_SENSOR; i++)
  {
    shared_tactiles[i] = 0;
//    shared_tactiles_last[i] = 0;
  }
  
  stop_thread = false;
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  
  ros::Publisher chatter_pub = n.advertise<usb_tactile_patch::UsbTactilePatch>("usb_tactile_patch", 1000);
  
   int rc1;
   pthread_t thread1;
   
   printf("starting, press CTRL-C to quit.\n");
   if( (rc1=pthread_create( &thread1, NULL, read_serial_thread, NULL)) )
   {
      printf("Thread creation failed: %d\n", rc1);
   }
    
   usb_tactile_patch::UsbTactilePatch msg;
   ros::Rate r(111); //  hz
   
   signal(SIGINT, sigproc);
   unsigned char my_buf[BUF_SIZE];
   while (!stop_thread)
   {
     pthread_mutex_lock( &mutex1 ); 
     for (int i = 0; i < NUM_SENSOR; i++)       
     {
       //This is a temp. hack. Toss out very large deltas. These
       //show up very occaisionally due to a comm/CDC error.
       //int dtx = abs(shared_tactiles[i]-shared_tactiles_last[i]);
       //if (dtx<10000 || startup_cnt)
       //{
	  msg.tactile_value[i] = shared_tactiles[i];
	  msg.crc_errors=crc_errors;
	  //shared_tactiles_last[i]=shared_tactiles[i];
       //}
       //startup_cnt=startup_cnt-1>0?startup_cnt-1:0;
     }
     /*for (int i = 0; i < BUF_SIZE; i++)
       my_buf[i] = shared_buf[i];*/
     pthread_mutex_unlock( &mutex1 );
     /*printf("-------------------------\n");
     for (int i = 0; i < BUF_SIZE; i++)
       printf("%i ", (int)my_buf[i]);     
     printf("-------------------------\n");     */
     chatter_pub.publish(msg);     
     r.sleep();
    }    
  
     pthread_join( thread1, NULL);
     
    printf("exiting...\n");
 
  return 0;
}