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


#ifndef _ECATSLV_H_
#define _ECATSLV_H_

#ifdef USE_ETHERCAT

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/

#include "ethercat_hw.h"

/*-----------------------------------------------------------------------------------------
------	
------	Defines and Types
------	
-----------------------------------------------------------------------------------------*/

///////////////////////////////////////////////////////////////
//
//	General
//

#ifndef OBJGETNEXTSTR
	#define	OBJGETNEXTSTR(p)	( (OBJCONST CHAR OBJMEM * )( ((UINT32) p) + OBJSTRLEN( (OBJCONST CHAR OBJMEM *) p ) + 1 ) )
#endif

#if MC_MOTOROLA
#ifndef LO_BYTE
	#define	LO_BYTE							1
#endif

#ifndef HI_BYTE
	#define	HI_BYTE							0
#endif

#ifndef LOLO_BYTE
	#define	LOLO_BYTE  						3
#endif

#ifndef LOHI_BYTE
	#define	LOHI_BYTE  						2
#endif

#ifndef HILO_BYTE
	#define	HILO_BYTE 						1
#endif

#ifndef HIHI_BYTE
	#define	HIHI_BYTE  						0
#endif

#ifndef LO_WORD
	#define	LO_WORD							1
#endif

#ifndef HI_WORD
	#define	HI_WORD							0
#endif

#else
#ifndef LO_BYTE
	#define	LO_BYTE							0
#endif

#ifndef HI_BYTE
	#define	HI_BYTE							1
#endif

#ifndef LOLO_BYTE
	#define	LOLO_BYTE  						0
#endif

#ifndef LOHI_BYTE
	#define	LOHI_BYTE  						1
#endif

#ifndef HILO_BYTE
	#define	HILO_BYTE 						2
#endif

#ifndef HIHI_BYTE
	#define	HIHI_BYTE  						3
#endif

#ifndef LO_WORD
	#define	LO_WORD							0
#endif

#ifndef HI_WORD
	#define	HI_WORD							1
#endif
#endif

#ifndef LOBYTE
	#define	LOBYTE(x)						(x&0xFF)
#endif

#ifndef HIBYTE
	#define	HIBYTE(x)						((x&0xFF00)>>8)
#endif

#ifndef LOLOBYTE
	#define	LOLOBYTE(x)						(x&0xFF)
#endif

#ifndef LOHIBYTE
	#define	LOHIBYTE(x)						((x&0xFF00)>>8)
#endif

#ifndef HILOBYTE
	#define	HILOBYTE(x)						((x&0xFF0000)>>16)
#endif

#ifndef HIHIBYTE
	#define	HIHIBYTE(x)						((x&0xFF000000)>>24)
#endif

#ifndef LOWORD
	#define	LOWORD(x)						(x&0xFFFF)
#endif

#ifndef HIWORD
	#define	HIWORD(x)						((x&0xFFFF0000)>>16)
#endif

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// State defines
*/

#define 	STATE_INIT						((UINT8) 0x01)
#define 	STATE_PREOP 					((UINT8) 0x02)
#define 	STATE_BOOT						((UINT8) 0x03)
#define 	STATE_SAFEOP					((UINT8) 0x04)
#define 	STATE_OP							((UINT8) 0x08)

#define	STATE_MASK						((UINT8) 0x0F)
#define 	STATE_CHANGE					((UINT8) 0x10)

#define 	INIT_2_BOOT						((STATE_INIT << 4) | STATE_BOOT)
#define 	PREOP_2_BOOT					((STATE_PREOP << 4) | STATE_BOOT)
#define 	SAFEOP_2_BOOT					((STATE_SAFEOP << 4) | STATE_BOOT)
#define 	OP_2_BOOT						((STATE_OP << 4) | STATE_BOOT)

#define 	INIT_2_INIT						((STATE_INIT << 4) | STATE_INIT)
#define 	INIT_2_PREOP					((STATE_INIT << 4) | STATE_PREOP)
#define 	INIT_2_SAFEOP					((STATE_INIT << 4) | STATE_SAFEOP)
#define 	INIT_2_OP						((STATE_INIT << 4) | STATE_OP)

#define 	PREOP_2_INIT					((STATE_PREOP << 4) | STATE_INIT)
#define 	PREOP_2_PREOP					((STATE_PREOP << 4) | STATE_PREOP)
#define 	PREOP_2_SAFEOP					((STATE_PREOP << 4) | STATE_SAFEOP)
#define 	PREOP_2_OP						((STATE_PREOP << 4) | STATE_OP)

#define 	SAFEOP_2_INIT					((STATE_SAFEOP << 4) | STATE_INIT)
#define 	SAFEOP_2_PREOP	  				((STATE_SAFEOP << 4) | STATE_PREOP)
#define 	SAFEOP_2_SAFEOP				((STATE_SAFEOP << 4) | STATE_SAFEOP)
#define 	SAFEOP_2_OP						((STATE_SAFEOP << 4) | STATE_OP)

#define 	OP_2_INIT						((STATE_OP << 4) | STATE_INIT)
#define 	OP_2_PREOP						((STATE_OP << 4) | STATE_PREOP)
#define 	OP_2_SAFEOP						((STATE_OP << 4) | STATE_SAFEOP)
#define 	OP_2_OP							((STATE_OP << 4) | STATE_OP)

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// ESM transition error codes
*/

#define	SYNCMANCHODDADDRESS 									0x00
#define	SYNCMANCHADDRESS 										0x01
#define	SYNCMANCHSIZE											0x02
#define	SYNCMANCHSETTINGS										0x03
#define	ERROR_SYNCMANCH(code, channel)					(code+(channel<<2))
#define	ERROR_SYNCMANCHODDADDRESS(channel)				(SYNCMANCHODDADDRESS+(channel<<2))
#define	ERROR_SYNCMANCHADDRESS(channel)					(SYNCMANCHADDRESS+(channel<<2))
#define	ERROR_SYNCMANCHSIZE(channel)	  					(SYNCMANCHSIZE+(channel<<2))
#define	ERROR_SYNCMANCHSETTINGS(channel)					(SYNCMANCHSETTINGS+(channel<<2))
#define	ERROR_SYNCTYPES										0x80
#define	ERROR_DCSYNCCONTROL									0x81
#define	ERROR_DCSYNC0CYCLETIME								0x82
#define	ERROR_DCSYNC1CYCLETIME								0x83
#define	ERROR_DCCYCLEPARAMETER								0x84
#define	ERROR_DCLATCHCONTROL									0x85

#define	ERROR_INVALIDSTATE									0xF0
#define	ERROR_NOMEMORY											0xF1
#define	ERROR_OBJECTDICTIONARY								0xF2
#define	ERROR_NOSYNCMANACCESS								0xF3
#define	ERROR_NOOFRXPDOS										0xF4
#define	ERROR_NOOFTXPDOS										0xF5
#define	ERROR_STATECHANGE										0xF6

#define	NOERROR_NOSTATECHANGE								0xFE
#define	NOERROR_INWORK											0xFF

#if MC_MOTOROLA
#define	EMCY_SM_ERRORCODE										0x00A0
#else
#define	EMCY_SM_ERRORCODE										0xA000
#endif
/* ECAT_CHANGE_START V3.11 */
#define	EMCY_SM_DEVICESPECIFIC								0xFF00
/* ECAT_CHANGE_END V3.11 */

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// AL Status Codes
*/

#define	ALSTATUSCODE_NOERROR							0x0000
#define	ALSTATUSCODE_UNSPECIFIEDERROR				0x0001
#define	ALSTATUSCODE_INVALIDALCONTROL				0x0011
#define	ALSTATUSCODE_UNKNOWNALCONTROL				0x0012
#define	ALSTATUSCODE_BOOTNOTSUPP					0x0013
#define	ALSTATUSCODE_NOVALIDFIRMWARE				0x0014
#define	ALSTATUSCODE_INVALIDMBXCFGINBOOT			0x0015
#define	ALSTATUSCODE_INVALIDMBXCFGINPREOP		0x0016
#define	ALSTATUSCODE_INVALIDSMCFG					0x0017
#define	ALSTATUSCODE_NOVALIDINPUTS					0x0018
#define	ALSTATUSCODE_NOVALIDOUTPUTS				0x0019
#define	ALSTATUSCODE_SYNCERROR						0x001A
#define	ALSTATUSCODE_SMWATCHDOG						0x001B
#define	ALSTATUSCODE_SYNCTYPESNOTCOMPATIBLE		0x001C
#define	ALSTATUSCODE_INVALIDSMOUTCFG 				0x001D
#define	ALSTATUSCODE_INVALIDSMINCFG 				0x001E
/* V3.01 ECAT_CHANGE_BEGIN */
#define	ALSTATUSCODE_INVALIDWDCFG					0x001F
/* V3.01 ECAT_CHANGE_END */
#define	ALSTATUSCODE_WAITFORCOLDSTART				0x0020
#define	ALSTATUSCODE_WAITFORINIT					0x0021
#define	ALSTATUSCODE_WAITFORPREOP					0x0022
#define	ALSTATUSCODE_WAITFORSAFEOP					0x0023
/* ECAT_CHANGE_BEGIN V3.11 */
#define	ALSTATUSCODE_INVALIDINPUTMAPPING			0x0024
#define	ALSTATUSCODE_INVALIDOUTPUTMAPPING		0x0025
/* ECAT_CHANGE_BEGIN V3.20 */
#define	ALSTATUSCODE_INCONSISTENTSETTINGS		0x0026
/* ECAT_CHANGE_END V3.20 */
/* ECAT_CHANGE_END V3.11 */
#define	ALSTATUSCODE_DCINVALIDSYNCCFG				0x0030
#define	ALSTATUSCODE_DCINVALIDLATCHCFG 			0x0031
#define	ALSTATUSCODE_DCPLLSYNCERROR	 			0x0032
#define	ALSTATUSCODE_DCSYNCIOERROR	 				0x0033
#define	ALSTATUSCODE_DCSYNCMISSEDERROR	 		0x0034
/* ECAT_CHANGE_BEGIN V3.20 */
#define	ALSTATUSCODE_DCINVALIDSYNCCYCLETIME		0x0035
#define	ALSTATUSCODE_DCSYNC0CYCLETIME				0x0036
#define	ALSTATUSCODE_DCSYNC1CYCLETIME				0x0037
/* ECAT_CHANGE_END V3.20 */
#define  ALSTATUSCODE_MBX_AOE							0x0041
#define  ALSTATUSCODE_MBX_EOE							0x0042
#define  ALSTATUSCODE_MBX_COE							0x0043
#define  ALSTATUSCODE_MBX_FOE							0x0044
#define  ALSTATUSCODE_MBX_SOE							0x0045
#define  ALSTATUSCODE_MBX_VOE							0x004F

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// AL event masks
*/

#define 	AL_CONTROL_EVENT 				((UINT16) 0x01)
#define 	SYNC0_EVENT		 				((UINT16) 0x04)
#define 	SYNC1_EVENT		 				((UINT16) 0x08)
#define 	SM_CHANGE_EVENT 				((UINT16) 0x10)

#ifndef MAX_PD_SYNC_MAN_CHANNELS
	#define	MAX_PD_SYNC_MAN_CHANNELS	2
#endif
#define	MAX_NUMBER_OF_SYNCMAN		MAX_PD_SYNC_MAN_CHANNELS //(MAX_PD_SYNC_MAN_CHANNELS+2)

//////// No Mailboxes uses so reconfigure here for just 2 SyncManagers //////////////////////
//#define	MAILBOX_WRITE					0 
//#define	MAILBOX_READ					1
#define	PROCESS_DATA_OUT	 			0 //SyncM0 is for output data from Master to Slave
#define	PROCESS_DATA_IN					1 //SyncM1 is for input data from Slave to Master

//#define 	MAILBOX_WRITE_EVENT	 		((UINT16) 0x0100)		//Interrupt mask
//#define 	MAILBOX_READ_EVENT			((UINT16) 0x0200)		//Interrupt mask
#define 	PROCESS_OUTPUT_EVENT 		((UINT16) 0x0100)		//Interrupt mask, was 0x0400
#define 	PROCESS_INPUT_EVENT 			((UINT16) 0x0200)	//Interrupt mask, was 0x800

#define 	PROCESS_DATA_EVENT			((UINT16) 0x0C00)
#define	NOT_SUPPORTED_EVENTS			((UINT16) 0xF0FE)

#define	FIRST_PD_SYNC_MAN_CHANNEL	0//2
#define	FIRST_PD_SYNC_MAN_MASK		0x0100//0x0400

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// Addresses
*/

#define	ESC_ADDR_OUTPUTSM				(ESC_ADDR_SYNCMAN+(PROCESS_DATA_OUT)*SIZEOF(TSYNCMAN))
#define	ESC_ADDR_INPUTSM				(ESC_ADDR_SYNCMAN+(PROCESS_DATA_IN)*SIZEOF(TSYNCMAN))

#define	MEMORY_START_ADDRESS			0x1000

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// Data Types
*/

typedef struct
{
	UINT16  dataType;
	UINT16  bitLen;
} TTYPELENGTH;
 
#endif //_ECATSLV_H_

/*-----------------------------------------------------------------------------------------
------	
------	global variables
------	
-----------------------------------------------------------------------------------------*/

#if _ECATSLV_
	#define PROTO 
#else 
	#define PROTO extern
#endif

#if BOOTSTRAPMODE_SUPPORTED
PROTO	BOOL						bBootMode;
#endif
PROTO	BOOL						bEcatOutputUpdateRunning;
PROTO	BOOL						bEcatInputUpdateRunning;
PROTO	BOOL						bEcatFirstOutputsReceived;
/* V3.02 ECAT_CHANGE_BEGIN */
/* HBu 06.02.06: process data sizes should be UINT16-variables */
PROTO	UINT16					nPdInputSize;
PROTO	UINT16					nPdOutputSize;
/* V3.02 ECAT_CHANGE_END */
/* ECAT_CHANGE_BEGIN V3.20 */
PROTO	BOOL						bSynchronMode;
/* ECAT_CHANGE_END V3.20 */
PROTO UINT8						nMaxSyncMan;
PROTO	UINT8						nAlControl;
PROTO	UINT8						nAlStatus;
PROTO	UINT8						nAlStatusFailed;
PROTO	UINT8						nAlStatusCode;
PROTO	UINT16					u16WdValue;
PROTO	UINT16					u16WdCounter;
/* V3.01 ECAT_CHANGE_BEGIN */
PROTO	BOOL						bWdActive;
/* V3.01 ECAT_CHANGE_END */
PROTO UINT8						nStateTrans;
PROTO	UINT16 					nEscAddrOutputData;
PROTO	UINT16 					nEscAddrInputData;
PROTO	BOOL						bDcSyncActive;
#if DC_SUPPORTED
PROTO	UINT8						nDcSyncControl;
#endif
/*-----------------------------------------------------------------------------------------
------  
------	global functions
------				 
-----------------------------------------------------------------------------------------*/

PROTO UINT8	CheckSmChannelParameters(UINT8 channel, TSYNCMAN ESCMEM *pSyncMan);
//PROTO void 	SendSmFailedEmergency(UINT8 channel, UINT8 code);
PROTO UINT8	CheckSmSettings(UINT8 maxChannel);
PROTO UINT8	StartInputHandler(void);
/* ECAT_CHANGE_BEGIN V3.20 */
PROTO UINT8 StartOutputHandler(void);
/* ECAT_CHANGE_END V3.20 */
PROTO UINT8 StopOutputHandler(void);
PROTO UINT8	StopInputHandler(void);
PROTO	void 	AL_ControlInd(UINT8 alControl);
PROTO	void 	ECAT_Init(void);
PROTO	void 	ECAT_Main(void);

#undef PROTO

#endif
