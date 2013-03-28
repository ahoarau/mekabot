/*
 * Serial communication over RS232.
 * Copyright (C) 2011  Meka Robotics
 * Author <pierrelucbacon@mekabot.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "serial_port.h"

#include <iostream>
#include <algorithm>

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/select.h>
#include <linux/serial.h>

#define CRC_LEN 4
#define CRC_BEGIN (DATA_END + 1)
#define CRC_END (CRC_BEGIN + CRC_LEN)

#define CRC_CCITT_POLYNOMIAL 0x1021
#define CRC_CCITT_INITIAL_VALUE 0xFFFF

#define BUF_SIZE 200

namespace m3
{
	using namespace std;
	using namespace m3rt;

////////////////////////////////////////////////////////////////////
	
void M3SerialPort::send(unsigned char * data, std::size_t len) 
{	    
	unsigned char write_buff[2*MAX_SP_PDO_BUFFER_SIZE + 2*SP_SIZE_LENGTH + 2*SP_CRC_LENGTH + 2];
  
	//Build send data
	write_data_bytes=0;
	for (int i=0;i<len;i++)
	{
	  if (data[i]==SP_START_MARKER || data[i]==SP_ESC_MARKER)
	    write_data[write_data_bytes++]=SP_ESC_MARKER;
	  write_data[write_data_bytes++]=data[i];
	}
	//Build size
	unsigned char * pc=(unsigned char*)&write_data_bytes;
	write_size_bytes=0;
	for (int i=0;i<SP_SIZE_LENGTH;i++)
	{
	  if (pc[i]==SP_START_MARKER || pc[i]==SP_ESC_MARKER)
	    write_size[write_size_bytes++]=SP_ESC_MARKER;
	  write_size[write_size_bytes++]=pc[i];
	}
	// Build CRC16, over 32 bits
	/*uint32_t checksum = crc16((uint8_t*) data, len);
	unsigned char * pk=(unsigned char*)&checksum;
	write_crc_bytes=0;
	for (int i=0;i<SP_CRC_LENGTH;i++)
	{
	  if (pk[i]==SP_START_MARKER || pk[i]==SP_ESC_MARKER)
	    write_crc[write_crc_bytes++]=SP_ESC_MARKER;
	  write_crc[write_crc_bytes++]=pk[i];
	}*/
	
	//Send out data
	char c=SP_START_MARKER;
	// copy into one buffer so only 1 write() call
	int bytes_not_written;    
	write_buff[0] = (unsigned char) c;
	for (int i = 0; i < write_size_bytes; i++)
	  write_buff[1+i] = (unsigned char) write_size[i];
	for (int i = 0; i < write_data_bytes; i++)
	  write_buff[1+i+write_size_bytes] = (unsigned char) write_data[i];
	
	ioctl(fd, TIOCOUTQ, &bytes_not_written);
	if (bytes_not_written == 0)
	{
	  write((unsigned char*)write_buff,(size_t)(1+write_size_bytes+write_data_bytes));
	}
	//std::cout<<"Did Write"<<tmp_cnt++<<std::endl;
	//write((char*)write_crc,(size_t)write_crc_bytes);
}

//Called by the M3Component. Should return immediately.
void M3SerialPort::sendAsync(void *data, std::size_t len)
{
	
	if (len>MAX_DATA_BUFFER_SIZE)
	      m3rt::M3_WARN("M3SerialPort sendAsync size out of bounds %d:%d\n",(int)len,(int)MAX_DATA_BUFFER_SIZE);
	else
	{
	    if (!pendingBufferSize)
	    {
	    pthread_mutex_unlock(&pendingBufferMutex);
	    memcpy(pendingBuffer,data,len);
	    pthread_mutex_unlock(&pendingBufferMutex);
	    pendingBufferSize = len;
	    }
	    //std::cerr<<"Got PDO"<<std::endl;
	}
		
}

void M3SerialPort::sendPendingData()
{
    
	if (pendingBufferSize >0) 
	{
	    memcpy(sendBuffer,pendingBuffer,pendingBufferSize);
	    send(sendBuffer, pendingBufferSize);
            #ifdef DEBUG
	    std::cerr << "Flushed pending PDO." << std::endl;
            #endif
            pendingBufferSize = 0;
	}
	/*else
	  std::cerr<<"Waiting on pdo"<<tmp_cnt++<<std::endl;*/
    
}
////////////////////////////////////////////////////////////////////

//Decode ESCAPE encoder data packet into rawBuffer from Serial Port
//Called from port thread
void M3SerialPort::pumpData() 
{
   //std::cout<<"PumpData1"<<port<<std::endl;
    
   int bytesRead = ::read(fd, buffer, SERIAL_PORT_BUFFER_SIZE);
   //std::cout<<"PumpData2"<<port<<std::endl;
  
   //M3_DEBUG("br:%d\n",bytesRead);

   if (bytesRead < 0)
   {
      /*throw std::runtime_error(std::string("Failed to read because ") + std::string(strerror(errno)));*/
      
      //SetStateError();
      /*M3_ERR("Failure to read Serial Port: %s for component %s.: %s\n",port,GetName(),strerror(errno));*/
   } 
   //std::cout<<"PumpData3"<<port<<std::endl;

   for (int i = 0; i < bytesRead; i++)
   {
           //#ifdef DEBUG
			//std::cout << "Received " << (int)buffer[i] << std::endl;
			//printf("Received %#x\n",buffer[i]);
           //#endif
           //Reset parser any time get a start byte
           if (!read_esc_last && buffer[i]==SP_START_MARKER)
	   {
	      read_state=READ_STATE_SIZE;
	      n_bytes_read=0;
	   }
	   else
	   {
	      switch (read_state)
	      {
		case READ_STATE_WAITING:
		  break;	//toss
		case READ_STATE_SIZE:
		{
		  if (buffer[i]!=SP_ESC_MARKER || read_esc_last)
		  {
		      read_size[n_bytes_read]=buffer[i];
		      
		      n_bytes_read++;
		      if (n_bytes_read==SP_SIZE_LENGTH)
		      {
			n_bytes_data=*((int*)read_size);
			if (n_bytes_data>=MAX_SP_PDO_BUFFER_SIZE)//overflow
			{
			  read_state=READ_STATE_WAITING;
			}
			else
			{
			  read_state=READ_STATE_DATA;
			}
			n_bytes_read=0;
			n_bytes_skip=0;
		      }
		  }
		  break;
		}
		case READ_STATE_DATA:
		{
		  //printf("Data %#x at %i\n",buffer[i],i);
		  if (buffer[i]!=SP_ESC_MARKER || read_esc_last)
		  {
		      read_data[n_bytes_read++]=buffer[i];
		      if (n_bytes_read+n_bytes_skip==n_bytes_data)
		      {
			//read_state=READ_STATE_CRC;
			//std::cout<<"ReadData "<<n_bytes_read<<std::endl;
			//n_bytes_read=0;
			read_state=READ_STATE_WAITING;
			n_bytes_read=0;
			pthread_mutex_lock(&rawBufferMutex);
			memcpy(rawBuffer, read_data, (size_t)n_bytes_data);
			pthread_mutex_unlock(&rawBufferMutex); 
		      }
		  }
		  else
		    n_bytes_skip++;
		  break;
		}
		case READ_STATE_CRC:
		{
		  if (buffer[i]!=SP_ESC_MARKER || read_esc_last)
		  {
		      read_crc[n_bytes_read++]=buffer[i];
		      if (n_bytes_read==SP_CRC_LENGTH)
		      {
			read_state=READ_STATE_WAITING;
			n_bytes_read=0;
			uint32_t checksum = crc16(read_data, n_bytes_data);
			//if (checksum == *((int*)read_crc)) //got good packet
			{
			  std::cout<<"PacketDone "<<tmp_cnt<<std::endl;
			   pthread_mutex_lock(&rawBufferMutex);
			   memcpy(rawBuffer, read_data, (size_t)n_bytes_data);
			   pthread_mutex_unlock(&rawBufferMutex);
			}
		      }
		  }
		  break;
		}
		default:
		  read_state=READ_STATE_WAITING;
		  break;
	    }
	   read_esc_last=(buffer[i]==SP_ESC_MARKER);
    }
   }
}

//Called from Component Process
int M3SerialPort::getLastDataBuffer(void * buffer, std::size_t len)
{
  
  
	if (len < n_bytes_data || !n_bytes_data) {
		return -1;
	}

	pthread_mutex_lock(&rawBufferMutex);
	memcpy(buffer, rawBuffer, n_bytes_data);
	pthread_mutex_unlock(&rawBufferMutex);
	return n_bytes_data;
}


bool M3SerialPort::write(unsigned char* buffer, std::size_t len)
{
	
	if (fd == -1) {
	      return false;
		throw std::runtime_error("Port is currently closed.");
	}

	int ret = ::write(fd,buffer,len);

	if (ret < 1) {
	      return false;
		throw std::runtime_error(std::string("Write failed with : ")
				+ std::string(strerror(errno)));
	}	
	//M3_DEBUG("bw: %d\n", ret);
	//usleep(1000);
 	return true;
}

uint16_t M3SerialPort::crc16(uint16_t crc, char byte)
{
    uint8_t v = 0x80;
    int i;
    for (i = 0; i < 8; i++) {
        bool xor_p = crc & 0x8000;
        crc <<= 1;
        if (byte & v) {
            crc += 1;
        }
        if (xor_p) {
            crc ^= CRC_CCITT_POLYNOMIAL;
        }
        v >>= 1;
    }
    return crc;
}

uint16_t M3SerialPort::crc16(uint8_t* buffer, unsigned int len)
{
    uint16_t crc = CRC_CCITT_INITIAL_VALUE;
    unsigned int i;
    for (i = 0; i < len; i++) {
        crc = crc16(crc, buffer[i]);
    }
    return crc16Normalize(crc);
}

uint16_t M3SerialPort::crc16Normalize(uint16_t crc)
{
    int i;
    for (i = 0; i < 16; i++) {
        bool xor_p = crc & 0x8000;
        crc <<= 1;
        if (xor_p) {
            crc ^= CRC_CCITT_POLYNOMIAL;
        }
    }
    return crc;
}


void *read_serial_thread(void * arg)
{
  M3SerialPort * m3sp = (M3SerialPort *)arg;
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
   
   while (!m3sp->stop_thread)
   {
      m3sp->pumpData();
      m3sp->sendPendingData();
     usleep(1000);
      
    }    
}

bool M3SerialPort::open(const std::string& port_name)
{
  port =port_name;
 
  fd = ::open(port.c_str(), O_RDWR | O_NOCTTY| O_NDELAY);
  if (fd == -1)
  {
    /*
    * Could not open the port.
    */

    //perror("open_port: Unable to open /dev/ttyS0 - ");
    M3_WARN("Unable to open serial port %s\n",port.c_str());
    //string err = "Unable to open serial port: "+port;
    //throw std::runtime_error(err);
    return false;
  }
  
  fcntl(fd, F_SETFL, 0);
  //fcntl(fd, F_SETFL, FNDELAY);//make return 0 if no byts

  
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
    
    //options.c_cc[VTIME] = 2;   /* inter-character timer 0.2s timeout */
    //options.c_cc[VMIN]  = 1;   /* blocking read until 1 chars received */

    /*
     * Set the new options for the port...
     */        
    tcsetattr(fd, TCSANOW, &options);
    
		  
	
	this->port = port;
	#ifdef DEBUG
	std::cerr << "Starting notifier on " << fd << std::endl;
	#endif
	
	stop_thread = false;
	if( (rc1=pthread_create( &thread1, NULL, read_serial_thread, (void*)this)) )
	{
	    M3_ERR("Thread creation failed: %d\n", rc1);
	    return false;
	}
	
	return true;
 	
}

void M3SerialPort::close() {

  M3_INFO("Stopping M3SerialPort Thread %s\n",port.c_str());
  if (!rc1)
  {
    stop_thread = true;
    pthread_cancel(thread1);
    pthread_join( thread1, NULL);
  }
  if (fd!=-1)
    ::close(fd);
    // Reset variables
	fd = -1;
	pendingBufferSize = 0;
	completeRawBuffer.erase(completeRawBuffer.begin(), completeRawBuffer.end());
  M3_INFO("Closed M3SerialPort %s\n",port.c_str());
}


} //namespace



