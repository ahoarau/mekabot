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


#ifndef M3EC_H
#define M3EC_H

#ifdef __KERNEL__
#define int16_t short
#define int32_t int
#else
#ifndef EMBEDDED
#include <sys/types.h>
#include <stdint.h>
#endif
#endif

#ifdef EMBEDDED
#define int16_t short
#define int32_t long
#define uint16_t unsigned short
#define uint32_t unsigned long
#define uint64_t unsigned long long
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////
#define M3EC_ADC_TICKS_MAX 4096.0
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#define M3_CONFIG_START_BLD 0x0001			//Flag to enter the bootloader
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MEKA_VENDOR_ID  0x00000518
#define M3ECHUB_PRODUCT_CODE 	1000
#define M3SEA_PRODUCT_CODE 	1001 //Legacy
#define M3PWR_PRODUCT_CODE 	1002 
#define M3LOADX6_PRODUCT_CODE 	1003 
#define M3DIGIT_PRODUCT_CODE 	1004  //Legacy
#define M3DEV_PRODUCT_CODE 	1005  //Legacy
#define M3GMB_PRODUCT_CODE	1006  //Legacy
#define M3BLD_PRODUCT_CODE	1007 
#define M3HEX4_PRODUCT_CODE 1008	 //Legacy
#define M3LEDX2_PRODUCT_CODE 1009
#define M3ACTX1_PRODUCT_CODE 1010
#define M3ACTX2_PRODUCT_CODE 1011
#define M3ACTX3_PRODUCT_CODE 1012
#define M3ACTX4_PRODUCT_CODE 1013
#define M3TACTX2_PRODUCT_CODE 1014
#define M3AH_FB_PRODUCT_CODE 1015

#define M3_PRODUCT_CODE_START 1001 //m3ec.ko searches start-end to ID non-hub slaves
#define M3_PRODUCT_CODE_END 1015

#define M3EC_PDO_STATUS_INDEX 0x6000
#define M3EC_PDO_CMD_INDEX 0x7000
#define M3EC_PDO_STATUS_SUBINDEX 1
#define M3EC_PDO_CMD_SUBINDEX 1

///////////////////////////////////////////////////////////////////////////////////////////////////////////

//v0.1-v0.3
//Arbitrary size accomodates most applications, can be increased
//Total PDO size must be even. ET1200 EEPROM must be updated to match size.
//#define MAX_PDO_SIZE_BYTES 60

//v0.4. Now supports non-fixed PDO sizes. The size designated in the EEPROM def. file
// must be match the size of the PDO structure here.
// Max PDO entry size is 30 bytes (8 bit bit-size field, max bit-len=256 bits per entry)
//Each PDO size is rounded up to N*30 bytes so that ESC read/write sizes are matched to expectation.


//The ET1200 has 1K of physical memory for PDO exchange. Because PDOs are triple-buffered, this 
// give roughly a max of 160 bytes for Status and 160 bytes for Command data
//The EEPROM is configured to place Command at 0x1000 and Status at 0x1200. This can be adjusted
//if memory gets tight.

//v1.0 Moving all to 64bit timestamp

#define MAX_PDO_ENTRY_SIZE 30
#define MAX_PDO_SIZE_BYTES 150  //5 entries. Can be larger in theory.

//Shared-Memory Interface structs

///////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef EMBEDDED

#define MAX_NUM_SLAVE 50		//No reason for limit, but rtai_malloc has issue if go bigger. Fix.

/*****************************************************************************/
//Causes downsample too bus by this factor
//#define NUM_EC_DOMAIN 3  //Works for ISS full humanoid
#define NUM_EC_DOMAIN 3  //Works for ISS full humanoid
//Todo: With NUM_EC_DOMAIN=1, not all in RBL bot get processed
// With NUM_EC_DOMAIN=2, last in chain (MH3) doesn't get prcoessed if m3rt_bus_init port3
// With NUM_EC_DOMAIN=3, works. Need to figure out why customized per # in chain...
// Must change m3rt_def.h so RT_KMOD_FREQUENCY/NUM_EC_DOMAIN=1000 (Hz)

typedef struct	
{
	int active;
	int network_id;
	int product_code;
	int serial_number;
	int online;
	int operational;
	int al_state;
	unsigned char status[MAX_PDO_SIZE_BYTES];
	unsigned char cmd[MAX_PDO_SIZE_BYTES];
	int n_byte_status;
	int n_byte_cmd;
} M3EcSlaveShm;

typedef struct 
{
  int64_t t_ecat_wait_rx;
  int64_t t_ecat_rx;
  int64_t t_ecat_wait_shm;
  int64_t t_ecat_shm;
  int64_t t_ecat_wait_tx;
  int64_t t_ecat_tx;
} M3EcDomainMonitor;

typedef struct 
{
	int64_t timestamp_ns;
	int slaves_responding;
	int slaves_active;
	int slaves_dropped;
	int link_up;
	int watchdog;
	int counter;
	M3EcDomainMonitor monitor[NUM_EC_DOMAIN];
	M3EcSlaveShm slave[MAX_NUM_SLAVE];
} M3EcSystemShm;




#define MAX_SDS_SIZE_BYTES 10000

typedef struct	
{	
	unsigned char status[MAX_SDS_SIZE_BYTES]; // SDS = shared data structure
	unsigned char cmd[MAX_SDS_SIZE_BYTES];
	int n_byte_status;
	int n_byte_cmd;
} M3Sds;


#endif


/////////////////////////////////////////////////////////////////////////////////////////////////

#endif
