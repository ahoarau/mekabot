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
#ifndef M3SerialPort_H_
#define M3SerialPort_H_

#include <m3rt/base/m3rt_def.h>
#include <m3rt/base/toolbox.h>
#include "m3rt/base/component.h"
#define SERIAL_PORT_BUFFER_SIZE 4096
#include <stack>
#include <vector>
#include <stdint.h>
#include <pthread.h>
#include <stdexcept>
#include <sys/termios.h>
#include <iostream>

/**setNewOptions
 * This class allows access to a serial port device for reading and writing.
 */



#define MAX_SP_PDO_BUFFER_SIZE 256
#define SP_START_MARKER                    1  // SOH byte
#define SP_ESC_MARKER                      27 // ESC
#define SP_SIZE_LENGTH                     4 //bytes
#define SP_CRC_LENGTH                      4 //bytes
 
#define READ_STATE_WAITING 0 //Waiting on a start byte
#define READ_STATE_SIZE 1 //Reading size bytes
#define READ_STATE_DATA 2 //Reading data bytes
#define READ_STATE_CRC 3  //Reading CRC bytes

namespace m3
{
	using namespace std;
	using namespace m3rt;
		

class M3SerialPort {
public:
	M3SerialPort():	
	restoreSettings(false),
	pendingBufferSize(0),
	state(READY),rc1(-1),
	sizeTmp(0),read_esc_last(0),read_state(0),n_bytes_read(0),n_bytes_data(0),n_bytes_skip(0),
	write_data_bytes(0),write_size_bytes(0),write_crc_bytes(0)
	{
		pendingBufferMutex = PTHREAD_MUTEX_INITIALIZER;
		rawBufferMutex = PTHREAD_MUTEX_INITIALIZER;
	}
	virtual ~M3SerialPort(){}
	bool stop_thread;
  private:
	int read_esc_last,read_state,n_bytes_read,n_bytes_data,n_bytes_skip;
	unsigned char read_size[SP_SIZE_LENGTH];
	unsigned char read_crc[SP_CRC_LENGTH];
	unsigned char read_data[MAX_SP_PDO_BUFFER_SIZE];
	
	unsigned char write_data[2*MAX_SP_PDO_BUFFER_SIZE];
	unsigned char write_size[2*SP_SIZE_LENGTH];
	unsigned char write_crc[2*SP_CRC_LENGTH];
	int write_data_bytes,write_size_bytes,write_crc_bytes;
	int tmp_cnt;

  public:	
	/**
	 * Open the given port.
	 * @param port The serial port to open : /dev/ttyS0 for example.
	 * 
	 */
	bool open(const std::string& port_name);
			

	/**
	 * Close the given port
	 */
	void close();

	

	/**
	 * Write a byte array to the serial port.
	 * @param buffer
	 * @param len The number of bytes in the buffer.
	 */
	bool write(unsigned char* buffer, std::size_t len);

	/**
	 * This function will write the data to the
	 * serial port using the following packet structure :
	 *
	 * START MARKER | LEN | DATA | CRC
	 *      3          4     N      4
	 *
	 * @param data A pointer to the raw data
	 * @param len The length of the data to write
	 */
	void send(unsigned char * data, std::size_t len);

	/**
	 * This function will write the data to the
	 * serial port just as M3SerialPort::send() does.
	 * However, a call to this function will be
	 * asynchronous. Note that "data" is not guaranteed to
	 * be sent. Only the latest data buffer set by this
	 * function will be sent when the serial port will
	 * become available for writing. In other words, this
	 * function does not make use of a queue internally
	 * and a dedicated thread for each call.
	 *
	 * @param data A pointer to the raw data
	 * @param len The length of the data to write
	 */
	void sendAsync(void * data, std::size_t len);

	
	/**
	 * @return The filedescriptor opened for the serial port.
	 */
	int getFd() { return fd; }

	/**
	 * @param buffer The input buffer in which to copy
	 * @param len The length of the input buffer
	 * @return The latest data buffer received. By data buffer,
	 * we mean a buffer that was transfered using the CRC-based protocol.
	 * This will *not* return the content of the input buffer, but rather
	 * the result of the parser on this input buffer. If no data buffer is available,
	 * the buffer parameter will be set to NULL, and len to 0. -1 failure, or the number
	 * of bytes copied into buffer otherwise.
	 */
	int getLastDataBuffer(void * buffer, std::size_t len);

	static const int MAX_DATA_BUFFER_SIZE = 512;
	
	/**
	 * Sent the pending data.
	 */
	void sendPendingData();

	/**
	 * Read data from the serial port and put it
	 * in a queue.
	 */
	void pumpData();
protected:
	

	typedef std::vector<uint8_t> Packet;

	/**
	 * Inspect the last packet received to see if it
	 * matches the state machine for receiving PDOs.
	 */
	void addByte(uint8_t byteReceived);
private:		
	std::string port;
	bool restoreSettings;

	char buffer[SERIAL_PORT_BUFFER_SIZE];

    uint16_t crc16(uint16_t crc, char byte);
    
    uint16_t crc16(uint8_t* buffer, unsigned int len);
    
    uint16_t crc16Normalize(uint16_t crc);	

	uint8_t pendingBuffer[MAX_DATA_BUFFER_SIZE];
	std::size_t pendingBufferSize;
	uint8_t sendBuffer[MAX_DATA_BUFFER_SIZE];
	pthread_mutex_t pendingBufferMutex;

	enum ReceivingState {
		READY,
		MARKER,
		SIZE,
		DATA,
		CRC
	};

	ReceivingState state;
	Packet completeRawBuffer;

	std::size_t size;
	std::size_t sizeTmp;
	int fd;
	
	uint32_t crc;
	uint8_t rawBufferTmp[MAX_DATA_BUFFER_SIZE];
	uint8_t rawBuffer[MAX_DATA_BUFFER_SIZE];
	pthread_mutex_t rawBufferMutex;	
	int rc1;
	pthread_t thread1;
};

} //namespace

#endif /* M3SerialPort_H_ */
