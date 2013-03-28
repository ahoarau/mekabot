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

#ifndef _ETHERCAT_ESC_DEF_H_

#define _ETHERCAT_ESC_DEF_H_

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/

#include "ethercat_def.h"

/*-----------------------------------------------------------------------------------------
------	
------	Defines and Types
------	
-----------------------------------------------------------------------------------------*/

///////////////////////////////////////////////////////////////
//
//	Standard Data Types
//

#ifndef	UBYTEWORD
	typedef union
	{
		UINT8		b[2];
		UINT16	w;
	} UBYTEWORD;
#endif

#ifndef	UBYTEDWORD
	typedef union
	{
		UINT8		b[4];
		UINT16	w[2];
		UINT32	d;
	} UBYTEDWORD;
#endif

#if BYTE_NOT_SUPPORTED
	#define	SIZEOF(a)			(sizeof(a)<<1)
#else
	#define	SIZEOF(a)			sizeof(a)
#endif

/*---------------------------------------------------------------------------------
------																								------
------	Typdefinitionen																		------
------																								------
---------------------------------------------------------------------------------*/

/****************************************************************
**
** DLL-Information
*/

typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[5];
									#define	ESC_ADDR_SYNCMANCHANNELS		2
									#define	ESC_SHIFT_SYNCMANCHANNELS		8
#else
	UINT8			 				Byte[10];
									#define	ESC_ADDR_SYNCMANCHANNELS		5
									#define	ESC_SHIFT_SYNCMANCHANNELS		0
#endif
	UINT16		 				Word[5];
									#define	ESC_ADDR_DPRAMSIZE				6
} UDLLINFORMATION;

/****************************************************************
**
** DLL-Control
*/

typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
#else
	UINT8			 				Byte[2];
#endif
	UINT16		 				Word[1];
} UDLLCONTROL;

/****************************************************************
**
** DLL-Status
*/

typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
#else
	UINT8			 				Byte[2];
#endif
	UINT16		 				Word[1];
} UDLLSTATUS;

/****************************************************************
**
** AL-Control
*/

typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
									/* ECAT_CHANGE_BEGIN V3.11 */
									#define	ESC_OFFS_ALCONTROL		0
									/* ECAT_CHANGE_END V3.11 */
#else
	UINT8			 				Byte[2];
#if MOTOROLA_16BIT
									#define	ESC_OFFS_ALCONTROL		1
#else
									#define	ESC_OFFS_ALCONTROL		0
#endif
#endif
	UINT16		 				Word[1];
} UALCONTROL;
	
/****************************************************************
**
** AL-Status
*/

typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
#else
	UINT8			 				Byte[2];
#endif
	UINT16 		 				Word[1];
} UALSTATUS;
	
/****************************************************************
**
** PDI Control
*/

typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
#else
	UINT8			 				Byte[2];
#endif
	UINT16 		 				Word[1];
} UPDICONTROL;
	
/****************************************************************
**
** PDI-Configuration
*/

typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
#else
	UINT8			 				Byte[2];
#endif
	UINT16 		 				Word[1];
} UPDICONFIGURATIONMCI16;
	
/****************************************************************
**
** AL-Event
*/

typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[2];
#else
	UINT8			 				Byte[4];
#endif
	UINT16 		 				Word[2];
} UALEVENT;
	
/****************************************************************
**
** CRC-Fault-Counter
*/

typedef struct
{
  UINT16        				ChannelACrcFaultCounter;
  UINT16        				ChannelBCrcFaultCounter;
} TCHANNELCRCFAULTCOUNTER;

/****************************************************************
**
** Serial E²PROM access
*/

typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
#else
	UINT8			 				Byte[2];
#endif
	UINT16 		 				Word[1];
} USERIALEEPROMCONTROL;
	
/* ECAT_CHANGE_BEGIN V3.20 */
typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8					 				Byte[1];	
#else
	UINT8					 				Byte[2];	
#endif
	UINT16								Word[1];
} USERIALEEPROMCONFIG;
/* ECAT_CHANGE_END V3.20 */

/****************************************************************
**
** FMMU
*/

typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[8];
#else
	UINT8			 				Byte[16];
#endif
	UINT16 		 				Word[8];
} UFMMU;
	
/****************************************************************
**
** Sync Manager
*/

typedef union
{
#if BYTE_NOT_SUPPORTED
  	UINT8         				Byte[2];
									#define	ESC_OFFS_SMCTRL			0
/* ECAT_CHANGE_BEGIN V3.20 */
										#define	SM_PDINITMASK				0x0D
										#define	SM_INITMASK					0x0F
										#define	SM_WRITESETTINGS			0x04				
										#define	SM_READSETTINGS			0x00				
										#define	SM_PDIEVENT					0x20				
/* ECAT_CHANGE_END V3.20 */
										#define	THREE_BUFFER				0x00
										#define	ONE_BUFFER					0x02
										#define	SYNCMAN_READ				0x00
										#define	SYNCMAN_WRITE				0x01
										/* V3.01 ECAT_CHANGE_BEGIN */
										#define	WATCHDOG_TRIGGER			0x40
										/* V3.01 ECAT_CHANGE_END */
									#define	ESC_OFFS_SMSTATUS			0
										#define	BUFFER_READ					0x0800
									#define	ESC_OFFS_ECATENABLE		1
										#define	SM_ECATENABLE				0x01
										#define	SM_TOGGLEMASTER			0x02
									#define	ESC_OFFS_PDIDISABLE		1
										#define	SM_PDIDISABLE				0x0100
										#define	SM_TOGGLESLAVE				0x0200
#else
  	UINT8         				Byte[4];
#if MOTOROLA_16BIT
									#define	ESC_OFFS_SMCTRL			1
#else
									#define	ESC_OFFS_SMCTRL			0
#endif
/* ECAT_CHANGE_BEGIN V3.20 */
										#define	SM_PDINITMASK				0x0D
										#define	SM_INITMASK					0x0F
										#define	SM_WRITESETTINGS			0x04				
										#define	SM_READSETTINGS			0x00				
										#define	SM_PDIEVENT					0x20				
/* ECAT_CHANGE_END V3.20 */
										#define	THREE_BUFFER				0x00
										#define	ONE_BUFFER					0x02
										#define	SYNCMAN_READ				0x00
										#define	SYNCMAN_WRITE				0x01
										/* V3.01 ECAT_CHANGE_BEGIN */
										#define	WATCHDOG_TRIGGER			0x40
										/* V3.01 ECAT_CHANGE_END */
#if MOTOROLA_16BIT
									#define	ESC_OFFS_SMSTATUS			0
#else
										#define	ESC_OFFS_SMSTATUS			1
#endif
/* ECAT_CHANGE_BEGIN V3.20 */
										#define	SM_WRITEEVENT				0x01
										#define	SM_READEVENT				0x02
										#define	SM_BUFFERWRITTEN			0x08
/* ECAT_CHANGE_END V3.20 */
#if MOTOROLA_16BIT
									#define	ESC_OFFS_ECATENABLE		3
#else
										#define	ESC_OFFS_ECATENABLE		2
#endif
										#define	SM_ECATENABLE				0x01
										#define	SM_TOGGLEMASTER			0x02
#if MOTOROLA_16BIT
									#define	ESC_OFFS_PDIDISABLE		2
#else
										#define	ESC_OFFS_PDIDISABLE		3
#endif
										#define	SM_PDIDISABLE				0x01
										#define	SM_TOGGLESLAVE				0x02
#endif
  	UINT16        				Word[2];
} USMSETTINGS;  
  	
typedef struct
{
  UINT16        				PhysicalStartAddress;					
  UINT16        				Length;	
/* V3.10 ECAT_CHANGE_BEGIN */
#if BYTE_NOT_SUPPORTED
									#define	ESC_OFFS_SMSETTINGS		2
#else
									#define	ESC_OFFS_SMSETTINGS		4
#endif
/* V3.10 ECAT_CHANGE_END */
  USMSETTINGS   				Settings;
} TSYNCMAN;

typedef struct
{
  UINT32        				ReceiveTimeChannelA;				// 0x0900
  UINT32        				ReceiveTimeChannelB;				// 0x0904
  UINT16        				Reserved1[4];						
  UINT32        				SystemTime[2];						// 0x0910
  UINT16        				Reserved2[4];						
  UINT32        				SystemTimeOffset[2];				// 0x0920
  UINT32        				DelayTime;							// 0x0928
  UINT16        				Reserved3[2];						
} TDCTRANSMISSION;

typedef union
{
#if BYTE_NOT_SUPPORTED
  	UINT8         				Byte[1];
									#define	ESC_OFFS_DC_CONTROL	  		0x00
										#define	DC_CYCLIC_ACTIVE				0x0100
										#define	DC_SYNC0_ACTIVE				0x0200
										#define	DC_SYNC1_ACTIVE				0x0400
#else
  	UINT8         				Byte[2];
#if MOTOROLA_16BIT
									#define	ESC_OFFS_DC_CONTROL	  		0x00
#else
									#define	ESC_OFFS_DC_CONTROL	  		0x01
#endif
										#define	DC_CYCLIC_ACTIVE				0x01
										#define	DC_SYNC0_ACTIVE				0x02
										#define	DC_SYNC1_ACTIVE				0x04
#endif
  	UINT16        				Word[1];
} UDCSYNCCONTROL;  
  	
typedef union
{
#if BYTE_NOT_SUPPORTED
  	UINT8             		Byte[1];
#else
  	UINT8             		Byte[2];
#endif
  	UINT16             		Word[1];
} UDCSYNCSTATUS;  
  	
typedef union
{
#if BYTE_NOT_SUPPORTED
  	UINT8             		Byte[1];
#else
  	UINT8             		Byte[2];
#endif
  	UINT16             		Word[1];
} UDCLATCHCONTROL;  
  	
typedef union
{
#if BYTE_NOT_SUPPORTED
  	UINT8             		Byte[1];
#else
  	UINT8             		Byte[2];
#endif
  	UINT16             		Word[1];
} UDCLATCHSTATUS;  
  	
typedef struct
{
	UINT32						PosEdgeTime_L;
	UINT32						PosEdgeTime_H;	
	UINT32						NegEdgeTime_L;
	UINT32						NegEdgeTime_H;	
} TLATCHTIMES32;

typedef struct
{
	UINT16						PosEdgeTime_LL;
	UINT16						PosEdgeTime_LH;
	UINT16						PosEdgeTime_HL;
	UINT16						PosEdgeTime_HH;	
	UINT16						NegEdgeTime_LL;
	UINT16						NegEdgeTime_LH;
	UINT16						NegEdgeTime_HL;
	UINT16						NegEdgeTime_HH;	
} TLATCHTIMES16;

typedef union
{
	TLATCHTIMES32				Dword;
	TLATCHTIMES16				Word;
} ULATCHTIMES;

typedef struct
{
  UDCSYNCCONTROL   			SyncControl;           			// 0x0980
#define	ESC_ADDR_SYNCCONTROL								0x0980
  UINT16             		ImpulseLength;						// 0x0982
  UINT16             		Reserved4[5];						
#define	ESC_ADDR_SYNC_STATUS								0x098E
  UDCSYNCSTATUS    			SyncStatus;           			// 0x098e
  UINT32            			SyncStartTime[2][2];				// 0x0990
  UINT32            			SyncCycleTime[2];					// 0x09a0
#define	ESC_ADDR_SYNC_CYCLETIME							0x09A0
  UDCLATCHCONTROL  			LatchControl;           		// 0x09a8
  UINT16             		Reserved9[2];						
  UDCLATCHSTATUS    			LatchStatus;           			// 0x09ae
/* ECAT_CHANGE_BEGIN V3.20 */
  ULATCHTIMES					LatchTimes[2];						// 0x09b0
/* ECAT_CHANGE_END V3.20 */
} TDCINTERRUPT;

typedef struct
{																			// offset...
	UDLLINFORMATION			DllInformation;					// 0x0000
	
	UINT16						cRes00[0x0003];					// 0x000A
	
	UINT16          			FixedStationAddress;				// 0x0010
	UINT16						cRes01[0x0077];					// 0x0012

	UDLLCONTROL					DllControl;							// 0x0100
	UINT16						cRes02[0x0007];					// 0x0102

	UDLLSTATUS					DllStatus;							// 0x0110
	UINT16						cRes03[0x0007];					// 0x0112

	UALCONTROL					AlControl;							// 0x0120
#define		ESC_ADDR_ALCONTROL							0x0120
	UINT16						cRes04[0x0007];					// 0x0122	

	UALSTATUS 					AlStatus;							// 0x0130
#define		ESC_ADDR_ALSTATUS								0x0130
	UINT16						cRes05[0x0001];					// 0x0132
#define		ESC_ADDR_ALSTATUSCODE						0x0134
	UINT16						AlStatusCode;						// 0x0134	
	UINT16						cRes05a[0x0005];					// 0x0136	

/* ECAT_CHANGE_BEGIN V3.20 */
#define		ESC_ADDR_PDICTRL								0x0140
/* ECAT_CHANGE_END V3.20 */
	UPDICONTROL					PdiControl;							// 0x0140
	UINT16						cRes06[0x0007];					// 0x0142	

	UPDICONFIGURATIONMCI16	PdiConfigurationMci16;			// 0x0150
	UINT16						cRes07[0x0059];					// 0x0152	

	UALEVENT 					AlEventMask; 						// 0x0204
#define		ESC_ADDR_ALEVENTMASK							0x0204
	UINT16						cRes07a[0x000C];					// 0x0208	

	UALEVENT 					AlEvent;								// 0x0220
#define		ESC_ADDR_ALEVENT								0x0220
	UINT16						cRes08[0x006E];					// 0x0224

	TCHANNELCRCFAULTCOUNTER	CrcFaultCounter;					// 0x0300
	UINT16						cRes09[0x007E];					// 0x0304

	UINT16          			WatchdogDivider;					// 0x0400
#define		ESC_WATCHDOG_DIVIDER							0x400
	UINT16						cRes10[0x0007];					// 0x0402

	UINT16          			PdiWatchdog;						// 0x0410
	UINT16						cRes11[0x0007];					// 0x0412

	UINT16          			SyncManChannelWatchdog[16];	// 0x0420
#define		ESC_SM_WATCHDOG								0x420
	UINT16          			SyncManWatchdogStatus;			// 0x0440
	UINT16						cRes12[0x005F];					// 0x0442

/* ECAT_CHANGE_BEGIN V3.20 */
	USERIALEEPROMCONFIG		SerialEepromConfig;				// 0x0500
/* ECAT_CHANGE_END V3.20 */
	USERIALEEPROMCONTROL		SerialEepromControl;				// 0x0502
#define		ESC_EEPROM_CONTROL							0x502
  	UINT16          			SerialEepromAddress[2];			// 0x0504
#define		ESC_EEPROM_ADDRESS							0x504
  	UINT16          			SerialEepromData[2];				// 0x0508
#define		ESC_EEPROM_DATA								0x508
	UINT16						cRes15[0x007A];					// 0x050C

	UFMMU	 						Fmmu[16];							// 0x0600
	UINT16						cRes16[0x0080];					// 0x0700

	#define	MAX_NO_OF_SYNC_MAN	16
	TSYNCMAN						SyncMan[MAX_NO_OF_SYNC_MAN]; 	// 0x0800
#define		ESC_ADDR_SYNCMAN								0x0800
/* ECAT_CHANGE_BEGIN V3.20 */
#define		ESC_ADDR_SM_MBXWRITE						0x0800
#define		ESC_ADDR_SM_MBXREAD						0x0808
/* ECAT_CHANGE_END V3.20 */
#define		ESC_ADDR_SM_MBXREAD_ECATCTRL			0x080E
#define		ESC_ADDR_SM_MBXREAD_PDICTRL			0x080F
	UINT16						cRes17[0x0040];					// 0x0880

	TDCTRANSMISSION			DcTransmission;					// 0x0900
	UINT16						cRes18[0x0028];					// 0x0930

	TDCINTERRUPT				DcInterrupt;						// 0x0980
/* ECAT_CHANGE_BEGIN V3.20 */
	UINT16						cRes19[0x0318];					// 0x09D0
/* ECAT_CHANGE_END V3.20 */
} TESCREGS;

typedef struct
{
	TESCREGS						Regs;
#define		ESC_ADDR_MEMORY								0x1000
	UINT16						Memory[0x0800];
} TESC;
	
typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8 			   		Byte[3];
									#define	MBX_OFFS_TYPE		2
									#define	MBX_OFFS_COUNTER	2
									#define	MBX_MASK_TYPE		0x0F00
									#define	MBX_MASK_COUNTER	0xF000
									#define	MBX_SHIFT_TYPE		8
									#define	MBX_SHIFT_COUNTER	12
#else
	UINT8 			   		Byte[6];
#if MOTOROLA_16BIT
									#define	MBX_OFFS_TYPE		4
									#define	MBX_OFFS_COUNTER	4
#else
									#define	MBX_OFFS_TYPE		5
									#define	MBX_OFFS_COUNTER	5
#endif
									#define	MBX_MASK_TYPE		0x0F
									#define	MBX_MASK_COUNTER	0xF0
									#define	MBX_SHIFT_TYPE		0
									#define	MBX_SHIFT_COUNTER	4
#endif
	UINT16 			   		Word[3];
									#define	MBX_OFFS_LENGTH	0
									#define	MBX_OFFS_ADDRESS	1
									#define	MBX_OFFS_FLAGS		2
} UMBXHEADER;

#define	MAX_MBX_DATA_SIZE	(MAX_MBX_SIZE-SIZEOF(UMBXHEADER))

typedef struct
{
  UMBXHEADER      			MbxHeader;
  UINT16            			Data[MAX_MBX_DATA_SIZE >> 1];
} TMBX;

typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8		b[1];
				#define	COEHEADER_COESERVICEOFFSET				0
				#define	COEHEADER_COESERVICEMASK				0xF000		
				#define	COEHEADER_COESERVICESHIFT				12
#else
	UINT8		b[2];
#if MOTOROLA_16BIT
				#define	COEHEADER_COESERVICEOFFSET				0
#else
				#define	COEHEADER_COESERVICEOFFSET				1
#endif
				#define	COEHEADER_COESERVICEMASK				0xF0		
				#define	COEHEADER_COESERVICESHIFT				4
#endif
	UINT16 	w;
} TCOEHEADER;

typedef struct
{
	union
	{
		UINT16 Word;
/* ECAT_CHANGE_BEGIN V3.20 */
			#define	SOEFLAGS_OPCODE		0x0007	// 0 = unused, 1 = readReq, 2 = readRes, 3 = writeReq, 4 = writeRes
																// 5 = notification (command changed notification)
			#define	SOEFLAGS_INCOMPLETE	0x0008	// more follows
			#define	SOEFLAGS_ERROR			0x0010	// an error word follows
			#define	SOEFLAGS_DRIVENO		0x00E0	// drive number

			#define	SOEFLAGS_DATASTATE	0x0100	// follows or requested
			#define	SOEFLAGS_NAME			0x0200	// follows or requested
			#define	SOEFLAGS_ATTRIBUTE	0x0400	// follows or requested
			#define	SOEFLAGS_UNIT			0x0800	// follows or requested
			#define	SOEFLAGS_MIN			0x1000	// follows or requested
			#define	SOEFLAGS_MAX			0x2000	// follows or requested
			#define	SOEFLAGS_VALUE			0x4000	// follows or requested
			#define	SOEFLAGS_DEFAULT		0x8000	// 
/* ECAT_CHANGE_END V3.20 */
	} Flags;

	UINT16		IDN_Frag;			// if (InComplete==0) SERCOS IDN else FragmentsLeft
//	union
//	{
//		UINT8		Data[]			// rest of mailbox data		if (Error==0)
//		UINT16		ErrorCode		//									if (Error==1)
//	};
} TSOEHEADER;

//////////////////////////////////////////////////////////////////////

typedef union
{
#if BYTE_NOT_SUPPORTED
	UINT8 			   		Byte[4];
									#define	EMCY_OFFS_ERRORREGISTER		1
									#define	EMCY_MASK_ERRORREGISTER		0x00FF
									#define	EMCY_SHIFT_ERRORREGISTER	0
									#define	EMCY_OFFS_DIAGCODE			1
									#define	EMCY_MASK_DIAGCODE			0xFF00
									#define	EMCY_SHIFT_DIAGCODE			8
#else
	UINT8 			   		Byte[8];
#if MOTOROLA_16BIT
									#define	EMCY_OFFS_ERRORREGISTER		3
									#define	EMCY_OFFS_DIAGCODE			2
#else
									#define	EMCY_OFFS_ERRORREGISTER		2
									#define	EMCY_OFFS_DIAGCODE			3
#endif
#endif
	UINT16 			   		Word[4];
									#define	EMCY_OFFS_ERRORCODE			0
									#define	EMCY_OFFS_DIAGDATA			2
} UEMCY;

typedef struct
{
#if SOE_SUPPORTED
	UINT16						SoeHeader[2];
#endif
	UEMCY							Emcy;
} TEMCYMESSAGE;

#endif //_ESC_H_

