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


#ifndef __ETHERCAT_H__
#define __ETHERCAT_H__ 

#ifdef USE_ETHERCAT

#include "setup.h"

#define EC_WATCHDOG_US 5000		//Signal an EC error if no PDO activity for this period
#define EC_SYS_TIME_ADDR 0x0910   //RJK edit for DC
#define EC_LATCH0_POS_EDG_ADDR 0x09B0
#define EC_LATCH1_POS_EDG_ADDR 0x09C0

int step_ethercat(void);
void setup_ethercat(void);
int eeprom_loaded(void);
void setup_spi(void);

extern unsigned long long dc_timestamp;  //RJK edit for DC
extern int ec_active;
extern int ec_debug[];
extern int ec_flags[];
extern  int ec_wd_expired;
extern long ec_wd_timestamp;

//Move ActX1 to PDOV2 (smaller packet size)

#if defined HB2_H2R1_J0J1 || defined HB2_H2R2_J0J1 \
|| defined HB2_0_2_H2R3_J0J1 
typedef M3ActX2PdoV1Cmd ec_cmd_t;
typedef M3ActX2PdoV1Status ec_stat_t;
#define PDO_COMMAND_SIZE MAX_PDO_ENTRY_SIZE*3
#define PDO_STATUS_SIZE MAX_PDO_ENTRY_SIZE*2
#define NUM_DBG_CH 2
#endif 

#if defined HB2_H2R1_J2J3J4 || defined HB2_H2R2_J2J3J4 || defined HB2_0_2_H2R3_J2J3J4
typedef M3ActX3PdoV1Cmd ec_cmd_t;
typedef M3ActX3PdoV1Status ec_stat_t;
#define PDO_COMMAND_SIZE MAX_PDO_ENTRY_SIZE*4
#define PDO_STATUS_SIZE MAX_PDO_ENTRY_SIZE*3
#define NUM_DBG_CH 3
#endif 

//typedef M3AhFingerX1PdoV0Cmd ec_cmd_t;
//typedef M3AhFingerX1PdoV0Status ec_stat_t;
//#define PDO_COMMAND_SIZE MAX_PDO_ENTRY_SIZE*2
//#define PDO_STATUS_SIZE MAX_PDO_ENTRY_SIZE*2
//#define NUM_DBG_CH 1
//#endif

/////////////////////////////////////////////////////////////////
extern ec_cmd_t  ec_cmd;
extern ec_stat_t   ec_stat;
/////////////////////////////////////////////////////////////////

#endif
#endif
