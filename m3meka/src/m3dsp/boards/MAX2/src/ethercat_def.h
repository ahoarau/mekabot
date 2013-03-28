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

#ifndef _ETHERCAT_DEF_H_
#define _ETHERCAT_DEF_H_

///////////////////////////////////////////////////////////////
//
//	Compiler defines
//
// The compiler defines are used to adapt the used microcontroller
// to the slave sample code. 
//

/*	COMPILERDEFINES_START */
#define	FALSE								0
#define	TRUE								1
#define	BOOL								unsigned char
#define	UINT8								unsigned char
#define	UINT16							unsigned short
#define	UINT32							unsigned long
#define	INT8								char
#define	INT16								short
#define	INT32								long
#define	CHAR								char
#define	UCHAR								unsigned char
#define	FAR								
#define	HUGE								
#define	HMEMCPY							memcpy
#define	ESCMEM							
#define	ESCMEMCPY						memcpy
#define	ESCMEMSET						memset
#define	ESCMBXMEMCPY					memcpy
#define	MBXMEM							
#define	MBXMEMCPY						memcpy
#define	MBXMEMCMP						memcmp
#define	MBXMEMSET						memset
#define	MBXSTRLEN						strlen
#define	MBXSTRCPY						memcpy
#define	OBJCONST							const
#define	VARCONST							
#define	OBJMEM							
#define	OBJTOMBXMEMCPY					memcpy
#define	OBJTOMBXSTRCPY					memcpy
#define	OBJMEMCPY						memcpy
#define	OBJSTRLEN						strlen
#define	OBJSTRCPY						memcpy
#define	MAKE_HUGE_PTR					
#define	MAKE_PTR_TO_ESC				
#define	EMCYMEMCPY						memcpy
#define	EMCYMEMSET						memset
#define	EMCYMEM							
#define	PD_BUFFER_TYPE					THREE_BUFFER
#define	ALLOCMEM							
#define	FREEMEM							
#define	PD_OUT_BUFFER_TYPE			THREE_BUFFER
#define	PD_IN_BUFFER_TYPE				THREE_BUFFER
#define	VARMEM							
#define	CONSTTYPE						
#define	APPL_AllocMailboxBuffer		
#define	APPL_FreeMailboxBuffer		
/*	COMPILERDEFINES_END */

///////////////////////////////////////////////////////////////
//
//	ECAT defines
//
// The ECAT defines are used independent of the used EtherCAT
// protocol:
//
// PRODUCT_CODE_LO: Lower 16 bits of the Product-Code (Object 0x1018, SI2 for CoE)
// PRODUCT_CODE_HI: Upper 16 bits of the Product-Code (Object 0x1018, SI2 for CoE)
// REVISION_LO: Lower 16 bits of the Revision (Object 0x1018, SI3 for CoE)
// REVISION_HI: Upper 16 bits of the Revision (Object 0x1018, SI3 for CoE)
// MAX_RX_PDOS: maximum number of RxPDOs supported by the slave, 
//              for NON-CoE devices this define shall set to 1 if the slave 
//					 has process output data (master view) for cyclic exchange
// MAX_TX_PDOS: maximum number of TxPDOs supported by the slave, 
//              for NON-CoE devices this define shall set to 1 if the slave 
//					 has process input data (master view) for cyclic exchange
// MIN_PD_WRITE_ADDRESS: minimum address for the process output data (Sync Manager 2)
//                       inside the application memory of the EtherCAT Slave Controller
//								 which could be set by the master
// MAX_PD_WRITE_ADDRESS: maximum address for the process output data (Sync Manager 2)
//                       inside the application memory of the EtherCAT Slave Controller
//								 which could be set by the master
// MIN_PD_READ_ADDRESS: minimum address for the process input data (Sync Manager 3)
//                      inside the application memory of the EtherCAT Slave Controller
//								which could be set by the master
// MAX_PD_READ_ADDRESS: maximum address for the process input data (Sync Manager 3)
//                      inside the application memory of the EtherCAT Slave Controller
//								which could be set by the master
// NO_OF_PD_INPUT_BUFFER: 1 or 3 depending on the mode of the sync manager input channel (2)
// NO_OF_PD_OUTPUT_BUFFER: 1 or 3 depending on the mode of the sync manager output channel (3)
// MIN_MBX_SIZE: minimum mailbox size (Sync Manager 0 and 1) which could be set by the master
// MAX_MBX_SIZE: maximum mailbox size (Sync Manager 0 and 1) which could be set by the master
// MIN_MBX_WRITE_ADDRESS: minimum address for the write (receive) mailbox (Sync Manager 0)
// MAX_MBX_WRITE_ADDRESS: maximum address for the write (receive) mailbox (Sync Manager 0)
// MIN_MBX_READ_ADDRESS: minimum address for the read (send) mailbox (Sync Manager 1)
// MAX_MBX_READ_ADDRESS: maximum address for the read (send) mailbox (Sync Manager 1)
// MAX_EMERGENCIES: number of emergencies supported in parallel
//
// the following defines are only needed if Distributed clocks are supported 
// (switch DC_SUPPORTED set to 1):
// DC_SYNC_INDEX: 0 - SYNC 0 is used for cyclic synchronization, 
//					   1 - SYNC 1 is used for cyclic synchronization
//	DC_SYNC_MODE: correct setting for DC control register (address 0x981 in ESC)
// DC_SYNC_TYPE: correct setting for Sync Manager parameter object (0x1C32, SI1 or 0x1C33, SI1)
//               in Distributed Clocks mode
// DC_SYNC_ACTIVE: active mask (SYNC 0 and/or SYNC 1 are supported) to be checked with the
//                 DC control register (address 0x981 in ESC)
// DC_SYNC_EVENT: event mask (SYNC 0 and/or SYNC 1 are supported) to be checked with the
//                AL event register
// MINIMUM_SYNC_CYCLE_TIME: minimum sync cycle time in timer ticks
// TIMER_TICKS_US: one micro second in number of timer ticks 
//

/*	ECATDEFINES_START */
#define	PRODUCT_CODE_LO				0x03E9 //MEKA code
#define	PRODUCT_CODE_HI				0x0000
#define	REVISION_LO						0x0002
#define	REVISION_HI						0x0000
#define	MAX_RX_PDOS						0x0001
#define	MAX_TX_PDOS						0x0001
#define	MIN_PD_WRITE_ADDRESS			0x1000
#define	MAX_PD_WRITE_ADDRESS			0x2000
#define	MIN_PD_READ_ADDRESS			0x1000
#define	MAX_PD_READ_ADDRESS			0x2000
#define	NO_OF_PD_INPUT_BUFFER		0x0003
#define	NO_OF_PD_OUTPUT_BUFFER		0x0003
#define	MIN_MBX_SIZE					0x0020
#define	MIN_MBX_WRITE_ADDRESS		0x1000
#define	MIN_MBX_READ_ADDRESS			0x1000
#define	MAX_MBX_SIZE					0x00C0
#define	MAX_MBX_WRITE_ADDRESS		0x2000
#define	MAX_MBX_READ_ADDRESS			0x2000
#define	MAX_EMERGENCIES				0x0005
#define	MIN_PD_CYCLE_TIME				100000L // 100 µs
#define	MAX_PD_CYCLE_TIME				10000000L // 10 ms
#define	PD_OUTPUT_SHIFT_TIME			0x0100
#define	PD_INPUT_SHIFT_TIME			0x0100
#define	DC_SYNC_INDEX					0x0000
#define	DC_SYNC_MODE					0x0003
#define	DC_SYNC_TYPE					0x0003
#define	DC_SYNC_ACTIVE					0x0002
#define	DC_SYNC_EVENT					0x0004
#define	MINIMUM_SYNC_CYCLE_TIME		0x0258
#define	TIMER_TICKS_US					5		//Meka
#define	MIN_CYCLE_TIME					0x012C
#define	MAX_CYCLE_TIME					0x1770
#define	DEFAULT_CYCLE_TIME32			0x30D40
#define	TIMER_TICKS_MS					5000	//Meka
#define	DC_HARDWARE_DELAY				0x0200
#define	HW_MONITOR_DELAY				0x0100
#define	CYCLE_OVERHEAD					0x0100
/*	ECATDEFINES_END */

///////////////////////////////////////////////////////////////
//
//	ECATAPPL defines
//
// The ECAT defines are used independent of the used EtherCAT
// protocol for the application:
//
// MAX_PD_OUTPUT_SIZE: maximum size of the process output data (Sync Manager 0) for cyclic exchange
// MAX_PD_INPUT_SIZE: maximum size of the process input data (Sync Manager 1) for cyclic exchange
//

/*	ECATAPPLDEFINES_START */
#define	MAX_PD_INPUT_SIZE				0x0040
#define	MAX_PD_OUTPUT_SIZE			0x0040
/*	ECATAPPLDEFINES_END */

///////////////////////////////////////////////////////////////
//
//	COE defines
//
// The CoE defines are only used for the CoE protocol
//
// VALUEINFOMASK: reserved for future use
// SUBINDEX0_SIZE: size of subindex 0 in case of a complete index access
// DEVICE_TYPE: device type of the slave (Object 0x1000)
// DEVICE_SERIAL_NUMBER: serial number of the slave (Object 0x1018, SI4)
// DEVICE_VENDOR_ID: serial number of the slave (Object 0x1018, SI1)
//

/*	COEDEFINES_START */
#define	VALUEINFOMASK					0x00000000
#define	SUBINDEX0_SIZE					0x00000001
#define	DEVICE_TYPE						0x00000000
#define	DEVICE_SERIAL_NUMBER			0x00000000
#define	DEVICE_VENDOR_ID				0x00000002
/*	COEDEFINES_END */

///////////////////////////////////////////////////////////////
//
//	COE String defines
//
// The CoE string defines are only used for the CoE protocol
//
// DEVICE_NAME: name of the slave (Object 0x1008)
// DEVICE_HW_VERSION: hardware version of the slave (Object 0x1009)
// DEVICE_HW_VERSION: software version of the slave (Object 0x100A)
//

/*	COESTRINGS_START */
/*	COESTRINGS_END */

///////////////////////////////////////////////////////////////
//
//	COEAPPL defines
//
// The COEAPPL defines are only used for the CoE protocol
//
// NO_OF_STANDARD_TYPES: number of standard data types
//

/*	COEAPPLDEFINES_START */
#define	NO_OF_STANDARD_TYPES				0x0020
/*	COEAPPLDEFINES_END */

///////////////////////////////////////////////////////////////
//
//	Terminal Profile defines
//
// The Terminal Profile defines are only used for the terminal profile
//
// MAX_CHANNELS: number of channels of the slave
// NO_OF_CHANNEL_OBJECTS: number of channel objects (d in terminal profile) of the slave
//

/*	TERMINALPROFILEDEFINES_START */
/*	TERMINALPROFILEDEFINES_END */

///////////////////////////////////////////////////////////////
//
//	FBM Profile defines
//
// The FBM Profile defines are only used for the fieldbus master profile
//
// MAX_NO_OF_BOXES: maximum number of fieldbus slaves
// NO_OF_SLAVE_OBJECTS: number of slave objects (d in FBM profile)
// MAX_SM_RX_PDOS: maximum number of Sync Manager 2 RxPDO Assigns supported by the slave
// MAX_SM_TX_PDOS: maximum number of Sync Manager 3 TxPDO Assigns supported by the slave
// MAX_RX_PDO_VARS: maximum number of mappable output varioables in a RxPDO
// MAX_RX_PDO_VARS: maximum number of mappable input varioables in a TxPDO
// FBUS_BOX_IDENT_SIZE: ident data size per fieldbus slave for the scan boxes command
// MAX_SIDX_CONFIG_GENERAL: maximum subindex in the general config object (object area 0x8000-0x8FFF)
// MAX_CMD_REQ_SIZE: maximum request size of a command object
//

/* FBMPROFILEDEFINES_START */
/* FBMPROFILEDEFINES_END */

///////////////////////////////////////////////////////////////
//
//	FOEAPPL defines
//
// The FOEAPPL defines are only used for the FoE protocol
//
// MAX_FILE_SIZE: maximum file size
//

/*	FOEAPPLDEFINES_START */
#define	MAX_FILE_SIZE					0x0100
/*	FOEAPPLDEFINES_END */

///////////////////////////////////////////////////////////////
//
//	Bootstrap-Mode defines
//

/*	BOOTSTRAPDEFINES_START */
/*	BOOTSTRAPDEFINES_END */

///////////////////////////////////////////////////////////////
//
//	Bootstrap-Mode expression defines
//

/*	BOOTSTRAPEXPRESSIONS_START */
/*	BOOTSTRAPEXPRESSIONS_END */

///////////////////////////////////////////////////////////////
//
//	project specific defines
//

/* ECATPROJECTDEFINES_START */
/* ECATPROJECTDEFINES_END */

///////////////////////////////////////////////////////////////
//
//	Software Switches to reduce code size
//

/*	ECATSWITCHES_START */

#define ECAT_PROCESS_OUTPUT_INT 1		//uC interrupted when Master writes to SyncM
#define ECAT_PROCESS_INPUT_INT 0		//uC interrupted when Master reads from SyncM
#define BOOTSTRAPMODE_SUPPORTED 0
#define MAILBOX_SUPPORTED 1
#define SOE_SUPPORTED 0
#define EOE_SUPPORTED 0			//Meka: Was 1
#define FOE_SUPPORTED 0
#define COE_SUPPORTED 0			//Meka: Was 1
#if COE_SUPPORTED
#define EMCY_PROTOCOL 3
#else
#define EMCY_PROTOCOL 5
#endif
#define COMPLETE_ACCESS_SUPPORTED 0
#define SEGMENTED_SDO_SUPPORTED 0
#define BYTE_NOT_SUPPORTED 0
#define DC_SUPPORTED 0			//Meka: turn back on for Distributed clocks
#define _PIC18 0				//Meka: Was 1
#define VOE_SUPPORTED 0
#define SM_CHANGE_SUPPORTED 0 	//Meka: Was 1
#define AL_EVENT_ENABLED 1		//Meka: Turn on for synchronous AL interrupts
#define MC_MOTOROLA 0
#define MOTOROLA_8BIT 0
#define MOTOROLA_16BIT 0
#define TEST_ON_EVA_BOARD 0		//Meka: Was 1
#define SMPAR_SUPPORTED 0		//Meka: Was 1
/*	ECATSWITCHES_END */

#endif // _ECATDEF_H_
