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

// Microcontroller specific settings for the EtherCAT Controller Interface
#ifndef _ETHERCAT_HW_H_
#define _ETHERCAT_HW_H_


#include "ethercat_def.h"
#include <string.h>
#include  "ethercat_esc.h"
//#include "p33fxxxx.h" //Configure for dsPIC 33FJ32MC204 M3 System

extern void ISR_EscReadAccess( UINT8 *pData, UINT16 Address, UINT16 Len );
extern void ISR_EscWriteAccess( UINT8 *pData, UINT16 Address, UINT16 Len );
//////////////////////////////////////////////////////////////////////////////////

#define SPI_DEACTIVE		1		//Active low
#define SPI_ACTIVE			0

#define ESC_RD					0x02			///< read acces to ESC
#define	ESC_WR					0x04			///< write acces to ESC

//////////////////////////////////////////////////////////////////////////////////

//	Microcontroller definitions
/*RC3	INPUT	PIN36	SYNC_1
RC4	INPUT	PIN37	SYNC_0
RC6	OUTPUT	PIN2	SPI_SEL
RC7	OUTPUT	PIN3	SPI_CLK
RC8	OUTPUT	PIN4	SPI_DO
RC9	INPUT	PIN5	SPI_DI
RA4	OUTPUT	PIN34	HEARTBEAT LED
RA10 OUTPUT	PIN12	DEBUG LED
RB7	INPUT	PIN43	SPI_IRQ
RA8 INPUT   PIN32   EEPROM_LOADED
*/
//////////////////////////////////////////////////////////////////////////////////
#define EEPROM_LOADED			PORTAbits.RA8
#define	SPI_SEL					LATCbits.LATC6
#define	SPI_CLK					LATCbits.LATC7
#define	SPI_DO					LATCbits.LATC8
#define	SPI_DI					LATCbits.LATC9

//dsPIC SPI Register Mapping to PIC18 code
#define SSPIF					_SPI1IF
#define SSPBUF					SPI1BUF

//////////////////////////////////////////////////////////////////////////////////

/* ESC interrupt */
#ifdef M3_BLD
//No interrupts
#define	DISABLE_ESC_INT				
#define	ENABLE_ESC_INT			
#define	ACK_ESC_INT				
#define	SET_ESC_INT				
#else
#define	DISABLE_ESC_INT			_INT0IE=0		
#define	ENABLE_ESC_INT			_INT0IE=1
#define	ACK_ESC_INT				_INT0IF=0
#define	SET_ESC_INT				_INT0IF=1
// RJK: For Sync0 Int
#define	DISABLE_SYNC_INT			_INT2IE=0		
#define	ENABLE_SYNC_INT				_INT2IE=1
#define	ACK_SYNC_INT				_INT2IF=0
#define	SET_SYNC_INT				_INT2IF=1
#endif


/* AL interrupt */
#define	DISABLE_GLOBAL_INT			DISABLE_ESC_INT 
#define	ENABLE_GLOBAL_INT			ENABLE_ESC_INT

#define	DISABLE_AL_EVENT_INT		DISABLE_GLOBAL_INT
#define	ENABLE_AL_EVENT_INT			ENABLE_GLOBAL_INT

//////////////////////////////////////////////////////////////////////////////////

/* Timer interrupt */
//Meka: This timer appears to be used to time the Slave running time
//Meka: We won't use it. 
#define	TIMER_RELOAD_REG			TMR0
#define	TIMER_RELOAD_REG_LO			TMR0L
#define	TIMER_RELOAD_REG_HI			TMR0H
#define	TIMER_CONFIG_REG			T0CON
#define	DISABLE_TIMER_INT					
#define	ENABLE_TIMER_INT			
#define	ACK_TIMER_INT				
#define	START_TIMER				
#define	STOP_TIMER					

//////////////////////////////////////////////////////////////////////////////////

// EtherCAT Timer (counts cyclically) 
// TMR3 with prescaler 1
// 
#define	ECAT_TIMER_REG				TMR3
#define	ECAT_TIMER_CONFIG_REG		T3CON
#define	ECAT_TIMER_REQ				_T3IF
#define	ACK_ECAT_TIMER_INT			_T3IF=0
#define	ENABLE_ECAT_TIMER_INT		_T3IE=1
#define	DISABLE_ECAT_TIMER_INT		_T3IE=0
#define	SET_ECAT_TIMER_INT_PRIO		_T3IP=0
#define	START_ECAT_TIMER			T3CONbits.TON=1
#define	STOP_ECAT_TIMER				T3CONbits.TON=0

//////////////////////////////////////////////////////////////////////////////////

/* Capture to filter the jitter of the DC-interrupt 
   CCP2 is connected to TMR3, capture mode with every falling edge */
#define	ESC_CAPTURE					0x04 //???
#define	ECAT_CAPTURE_REG			CCPR2
#define	ECAT_CAPTURE_CONFIG_REG	CCP2CON

//////////////////////////////////////////////////////////////////////////////////

typedef	union
{
	unsigned short	Word;
	unsigned char	Byte[2];
} UBYTETOWORD;

//////////////////////////////////////////////////////////////////////////////////

#if _ETHERCATHW_
	#define PROTO 
#else 
	#define PROTO extern
#endif

//PROTO	BOOL			bEcatWdExpired;
PROTO	BOOL			bAlEventEnabled;
//PROTO	UINT16		u16OldTimer;
PROTO	UALCONTROL	EscAlControl;
PROTO UALEVENT		EscAlEvent;

//////////////////////////////////////////////////////////////////////////////////
// Note: 
// The HW_* are called from the main cyclic loop
// The ISR_* are called from within the INT0 ISR

PROTO	void HW_EscReadAccess( UINT8 *pData, UINT16 Address, UINT16 Len );
PROTO	void HW_EscWriteAccess( UINT8 *pData, UINT16 Address, UINT16 Len );
PROTO	void HW_ResetIntMask(UINT16 intMask);
PROTO	void HW_SetIntMask(UINT16 intMask);
PROTO	void HW_SetAlStatus(UINT8 alStatus, UINT8 alStatusCode);
PROTO	void HW_DisableSyncManChannel(UINT8 channel);
PROTO	void HW_EnableSyncManChannel(UINT8 channel);
PROTO	TSYNCMAN * HW_GetSyncMan(UINT8 channel);
PROTO	void HW_Main(void);
PROTO   void ISR_GetInterruptRegister();


#undef	PROTO
//////////////////////////////////////////////////////////////////////////////////


#endif



