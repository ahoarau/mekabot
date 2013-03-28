/* 
M3 -- Meka Robotics Real-Time Control System
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef M3_BOOTLOADER_H
#define M3_BOOTLOADER_H


#include <m3rt/base/m3ec_def.h>
//#include <m3/hardware/m3ec_pdo_v1_def.h>
#include <m3rt/base/m3rt_def.h>


#define BUFFER_SIZE         4096
#define PM30F_ROW_SIZE 32
#define PM33F_ROW_SIZE 64*8
#define EE30F_ROW_SIZE 16
#define PM_SIZE 1536 /* Max: 144KB/3/32=1536 PM rows for 30F. */
#define EE_SIZE 128 /* 4KB/2/16=128 EE rows */
#define CM_SIZE 8

#define COMMAND_NACK			0x00
#define COMMAND_ACK				0x01
#define COMMAND_READ_PM			0x02
#define COMMAND_READ_PM_CON		0x03
#define COMMAND_WRITE_PM		0x04
#define COMMAND_WRITE_PM_CON	0x05
#define COMMAND_WRITE_CM		0x06
#define COMMAND_RESET			0x07
#define COMMAND_READ_ID			0x08
#define COMMAND_PING			0x09
#define COMLINK_ERR 			0x0A


#define BLD_BUF_SIZE 48 

typedef struct 
{
	int16_t config;						//Reserved
	int16_t nbytes;						//Number of data bytes
	char data[BLD_BUF_SIZE];			//Bootloader data buffer
	int16_t cmd_idx;					//Command counter
	unsigned char cmd;
	
}M3BldPdoV1Cmd;

typedef struct 
{
	int16_t nbytes;						//Number of data bytes
	char data[BLD_BUF_SIZE];			//Bootloader data buffer
	int16_t cmd_idx_ack;				//Acknowledege of command
	int16_t flags; 						//Reserved
}M3BldPdoV1Status;


enum eFamily
{
	dsPIC30F,
	dsPIC33F,
	PIC24H,
	PIC24F
};



class M3Bootloader{
public:
	M3Bootloader():shm(0),Family(dsPIC33F),debug(0),verbose(1){}
	virtual bool Ping(int id_slave);
	virtual bool ReadProgramMemory(int id_slave,char * address);
	virtual bool ReadConfigId(int id_slave);
	virtual bool WriteProgramMemory(int id_slave,char * filename);
	virtual bool Shutdown();
	virtual bool Startup(int id_slave);
	virtual void SetDebug(int on){debug=on;}
	virtual void SetVerbose(int on){verbose=on;}

private:
		M3EcSystemShm * shm;
		M3EcSlaveShm * slave_shm;
		char Buffer[BUFFER_SIZE];
		eFamily  Family;
		int debug;
		int verbose;
		int ping_success[100];
};



#endif
