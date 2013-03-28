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

// This file contains the interface to the ESC via SPI\n
#ifdef USE_ETHERCAT

#define	_ETHERCATHW_ 1
#include "ethercat_hw.h"
#undef	_ETHERCATHW_
#define	_ETHERCATHW_ 0
#include "ethercat.h"
#include "ethercat_slave_fsm.h"
#include "ethercat_appl.h"
#include "setup.h"
#include "dio.h"
#include "spi1Drv.h"

/////////////////////////////////////////////////////////////////
TSYNCMAN		TmpSyncMan;
UINT8			EscMbxReadEcatCtrl;




int tx_idx;
int rx_idx;

unsigned char did_rx;
unsigned char did_tx;

unsigned char do_rx;
unsigned char do_tx;



void ResetIdx()
{
    tx_idx = 0;
    rx_idx = 0;

    did_rx = 0;
    did_tx = 0;

    do_tx = 0;
    do_rx = 0;
}

/////////////////////////////////////////////////////////////////////////////
void GetInterruptRegister( )
{
	#if AL_EVENT_ENABLED
	DISABLE_AL_EVENT_INT;
	#endif
if (!IEC0bits.DMA1IE)
{
	/* select the SPI */
	SPI_SEL = SPI_ACTIVE;


//#ifdef ECAT_DMA
#if 0
                DMA0CNT = 0;
                DMA1CNT = 0;
                ResetDMATx();

                SetTxBuf((UINT8) (ESC_ADDR_SM_MBXREAD_ECATCTRL >> 5));

                //dma_running = 1;
                DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

                //while (dma_running);
                while(!IFS0bits.DMA1IF);
                IFS0bits.DMA1IF = 0;

              EscAlEvent.Byte[0]  = GetRxBuf();

              DMA0CNT = 0;
                DMA1CNT = 0;
                ResetDMATx();

                SetTxBuf((UINT8) (((ESC_ADDR_SM_MBXREAD_ECATCTRL & 0x1F) << 3) | ESC_RD));

                //dma_running = 1;
                DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

                //while (dma_running);
                while(!IFS0bits.DMA1IF);
                IFS0bits.DMA1IF = 0;

              EscAlEvent.Byte[1]  = GetRxBuf();

               DMA0CNT = 0;
                DMA1CNT = 0;
                ResetDMATx();

                SetTxBuf(0xFF);

                //dma_running = 1;
                DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

                //while (dma_running);
                while(!IFS0bits.DMA1IF);
                IFS0bits.DMA1IF = 0;

              EscMbxReadEcatCtrl  = GetRxBuf();

#else

		/* reset transmission flag */
	SSPIF=0;
	/* there have to be at least 15 ns after the SPI_SEL signal was active (0) before
	   the transmission shall be started */
	/* write to SSPBUF register starts the SSI access,
	   read the sm mailbox read ecatenable byte
	   (has to be synchronous to the al event flags) */
	SSPBUF = (UINT8) (ESC_ADDR_SM_MBXREAD_ECATCTRL >> 5);

	/* SSI is busy */
	while( SSPIF == 0 );
	/* get first byte of AL Event register */

//	LATDbits.LATD11 = 0;


	EscAlEvent.Byte[0] = SSPBUF;

	/* reset SSI interrupt flag */
	SSPIF = 0;
	/* write to SSPBUF register starts the SSI access
	   read the sm mailbox read ecatenable byte */
	SSPBUF = (UINT8) (((ESC_ADDR_SM_MBXREAD_ECATCTRL & 0x1F) << 3) | ESC_RD);
	/* write to SSPBUF register starts the SSI access */
	while( SSPIF == 0 );
	/* get first byte of AL Event register */

	EscAlEvent.Byte[1] = SSPBUF;

	/* reset SSI interrupt flag */
	SSPIF = 0;
	/* if the SPI transmission rate is higher than 15 MBaud, the Busy detection shall be
	   done here */

	/* write to SSPBUF register starts the SSI access
	   read the sm mailbox read ecatenable byte (last byte) */
	SSPBUF = 0xFF;
	/* write to SSPBUF register starts the SSI access */
	while( SSPIF == 0 );
	/* get first byte of AL Event register */
	EscMbxReadEcatCtrl = SSPBUF;
	/* reset SSI interrupt flag */
	SSPIF = 0;						/* reset transmission flag */
#endif


	/* there has to be at least 15 ns + CLK/2 after the transmission is finished
	   before the SPI_SEL signal shall be 1 */

	SPI_SEL = SPI_DEACTIVE;
}
	#if AL_EVENT_ENABLED
//	ENABLE_AL_EVENT_INT;
	#endif


}

/////////////////////////////////////////////////////////////////////////////
void ISR_GetInterruptRegister( )
{
	/////////////////////////////////////////////////////////////////////////////////////////
	/**
	 \brief  The function operates a SPI access without addressing.

			 The first two bytes of an access to the EtherCAT ASIC always deliver the interupt
			 register. It will be saved in the global EscAlEvent
	*////////////////////////////////////////////////////////////////////////////////////////

	SPI_SEL = SPI_ACTIVE;				/* select the SPI */

#ifdef ECAT_DMA
        DMA0CNT = 0;
	DMA1CNT = 0;
	ResetDMATx();

	SetTxBuf(0);

        //dma_running = 1;
	DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

	//while (dma_running);
        while(!IFS0bits.DMA1IF);
        IFS0bits.DMA1IF = 0;

        EscAlEvent.Byte[0] = GetRxBuf();

        DMA0CNT = 0;
	DMA1CNT = 0;
	ResetDMATx();
        //dma_running = 1;

	SetTxBuf(ESC_RD);

   	DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

        //while (dma_running);
        while(!IFS0bits.DMA1IF);
        IFS0bits.DMA1IF = 0;

	EscAlEvent.Byte[1] =  GetRxBuf();

#else

	SSPIF=0;							/* reset transmission flag */

	/* there have to be at least 15 ns after the SPI_SEL signal was active (0) before
	   the transmission shall be started */
	/* write to SSPBUF register starts the SSI access,
	   read the sm mailbox read ecatenable byte
	   (has to be synchronous to the al event flags) */
	SSPBUF = 0;

	while( SSPIF == 0 );				/* SSI is busy */
	EscAlEvent.Byte[0] = SSPBUF;		/* get first byte of AL Event register */
	SSPIF = 0;							/* reset SSI interrupt flag */

	/* write to SSPBUF register starts the SSI access
	   read the sm mailbox read ecatenable byte */
	SSPBUF = ESC_RD;					/* write to SSPBUF register starts the SSI access */

	while( SSPIF == 0 );
	EscAlEvent.Byte[1] = SSPBUF;		/* get first byte of AL Event register */
	SSPIF = 0;
#endif						/* reset SSI interrupt flag */

	/* if the SPI transmission rate is higher than 15 MBaud, the Busy detection shall be
	   done here */

	SPI_SEL = SPI_DEACTIVE;
}
/////////////////////////////////////////////////////////////////////////////
void AddressingEsc( UINT16 Address, UINT8 Command )
{
	/////////////////////////////////////////////////////////////////////////////////////////
	/**
	 \param Address 	EtherCAT ASIC address ( upper limit is 0x1FFF )	for access.
	 \param Command	ESC_WR performs a write access; ESC_RD performs a read access.

	 \brief The function addresses the EtherCAT ASIC via SPI for a following SPI access.
	*////////////////////////////////////////////////////////////////////////////////////////

	UBYTETOWORD tmp;
	tmp.Word = ( Address << 3 ) | Command;

	SPI_SEL = SPI_ACTIVE;				/* select the SPI */

//#ifdef ECAT_DMA
#if 0
        DMA0CNT = 0;
	DMA1CNT = 0;
	ResetDMATx();

	//Spi1TxBuffA[0] = (unsigned int)tmp.Byte[1];
	SetTxBuf(tmp.Byte[1]);

        //dma_running = 1;
	DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

        while(!IFS0bits.DMA1IF);
        IFS0bits.DMA1IF = 0;
	//while (dma_running);
        //while(DMA0REQbits.FORCE==1);

	//EscAlEvent.Byte[0] = (unsigned char)Spi1RxBuffA[0];
        EscAlEvent.Byte[0] = GetRxBuf();

        DMA0CNT = 0;
	DMA1CNT = 0;
	ResetDMATx();
        //dma_running = 1;

	SetTxBuf(tmp.Byte[0]);

   	DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

        while(!IFS0bits.DMA1IF);
        IFS0bits.DMA1IF = 0;
        //while (dma_running);
        //while(DMA0REQbits.FORCE==1);

	EscAlEvent.Byte[1] =  GetRxBuf();

#else

	SSPIF=0;							/* reset transmission flag */

	/* there have to be at least 15 ns after the SPI_SEL signal was active (0) before
	   the transmission shall be started */
	/* send the first address/command byte to the ESC */
	SSPBUF = tmp.Byte[1];

	while( !SSPIF );					/* wait until the transmission of the byte is finished */
	EscAlEvent.Byte[0] = SSPBUF;		/* get first byte of AL Event register */
	SSPIF=0;							/* reset transmission flag */
	SSPBUF = tmp.Byte[0];				/* send the second address/command byte to the ESC */
	while( !SSPIF );					/* wait until the transmission of the byte is finished */
	EscAlEvent.Byte[1] = SSPBUF;		/* get second byte of AL Event register */
	SSPIF = 0;							/* reset transmission flag */

#endif

	/* if the SPI transmission rate is higher than 15 MBaud, the Busy detection shall be
	done here */
}
/////////////////////////////////////////////////////////////////////////////
void ISR_AddressingEsc( UINT16 Address, UINT8 Command )
{
	/////////////////////////////////////////////////////////////////////////////////////////
	/**
	 \param Address 	EtherCAT ASIC address ( upper limit is 0x1FFF )	for access.
	 \param Command	ESC_WR performs a write access; ESC_RD performs a read access.

	 \brief The function addresses the EtherCAT ASIC via SPI for a following SPI access.
	*////////////////////////////////////////////////////////////////////////////////////////

        UBYTETOWORD tmp;
	char read;
	tmp.Word = ( Address << 3 ) | Command;

	/* select the SPI */
	SPI_SEL = SPI_ACTIVE;

 #ifdef ECAT_DMA
        DMA0CNT = 0;
	DMA1CNT = 0;
	ResetDMATx();

	SetTxBuf(tmp.Byte[1]);

        //dma_running = 1;
	DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

	//while (dma_running);
        while(!IFS0bits.DMA1IF);
        IFS0bits.DMA1IF = 0;


        DMA0CNT = 0;
	DMA1CNT = 0;
	ResetDMATx();
        //dma_running = 1;

	SetTxBuf(tmp.Byte[0]);

   	DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

        //while (dma_running);
        while(!IFS0bits.DMA1IF);
        IFS0bits.DMA1IF = 0;

#else


	/* reset transmission flag */
	SSPIF=0;
	/* there have to be at least 15 ns after the SPI_SEL signal was active (0) before
	   the transmission shall be started */
	/* send the first address/command byte to the ESC */
	SSPBUF = tmp.Byte[1];
	/* wait until the transmission of the byte is finished */
	while( !SSPIF );
	read=SSPBUF; //Throw-away
	/* reset transmission flag */
	SSPIF=0;
	/* send the second address/command byte to the ESC */
	SSPBUF = tmp.Byte[0];
	/* wait until the transmission of the byte is finished */
	while( !SSPIF );
	read=SSPBUF; //Throw-away
	/* reset transmission flag */
	SSPIF = 0;

	/* if the SPI transmission rate is higher than 15 MBaud, the Busy detection shall be
	   done here */
#endif



}


/////////////////////////////////////////////////////////////////////////////
void ISR_EscReadAccess( UINT8 *pData, UINT16 Address, UINT16 Len )
{
	/////////////////////////////////////////////////////////////////////////////////////////
	/**
	 \param pData		Pointer to a byte array which holds data to write or saves read data.
	 \param Address 	EtherCAT ASIC address ( upper limit is 0x1FFF )	for access.
	 \param Len			Access size in Bytes.

	 \return	Indicates if the access was succesful ( TRUE ).

	 \brief  This function operates the SPI access to the EtherCAT ASIC.
	*////////////////////////////////////////////////////////////////////////////////////////
	UINT16 i = Len;
	UINT8 data = 0;

	/* send the address and command to the ESC */
 	ISR_AddressingEsc( Address, ESC_RD );


#ifdef BURST_DMA
        int idx = 0;
        while ( i-- > 0 )
	{
		if ( i == 0 )
		{
			/* when reading the last byte the DI pin shall be 1 */
			data = 0xFF;
		}
                SetTxBufIdx(idx, data);
                idx++;
        }

        DMA0CNT = Len-1;
        DMA1CNT = Len-1;
        ResetDMATx();

        //dma_running = 1;
        DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

        //while (dma_running);
        while(!IFS0bits.DMA1IF);
        IFS0bits.DMA1IF = 0;

        i = Len;
        idx = 0;
        while ( i-- > 0 )
	{

                *pData++ = GetRxBufIdx(idx);
                idx++;
        }
#else

	/* loop for all bytes to be read */
	while ( i-- > 0 )
	{
		if ( i == 0 )
		{
			/* when reading the last byte the DI pin shall be 1 */
			data = 0xFF;
		}

#ifdef ECAT_DMA
                DMA0CNT = 0;
                DMA1CNT = 0;
                ResetDMATx();

                SetTxBuf(data);

                //dma_running = 1;
                DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

                //while (dma_running);
                while(!IFS0bits.DMA1IF);
                IFS0bits.DMA1IF = 0;

               *pData++ = GetRxBuf();
        }

#else

		/* reset transmission flag */
		SSPIF = 0;
		/* start transmission */
		SSPBUF = data;
		/* wait until transmission finished */
		while ( !SSPIF );
		/* get data byte */
		*pData++ = SSPBUF;
	}
	/* reset transmission flag */
	SSPIF = 0;							/* reset transmission flag */
#endif
#endif
	/* there has to be at least 15 ns + CLK/2 after the transmission is finished
	   before the SPI_SEL signal shall be 1 */
	SPI_SEL = SPI_DEACTIVE;

	/* at this time the result of the transmission can be checked */

}


/////////////////////////////////////////////////////////////////////////////
void ISR_EscWriteAccess( UINT8 *pData, UINT16 Address, UINT16 Len )
{
	/////////////////////////////////////////////////////////////////////////////////////////
	/**
	 \param pData		Pointer to a byte array which holds data to write or saves read data.
	 \param Address 	EtherCAT ASIC address ( upper limit is 0x1FFF )	for access.
	 \param Len			Access size in Bytes.

	 \return	Indicates if the access was succesful ( TRUE ).

	 \brief  This function operates the SPI access to the EtherCAT ASIC.
	*////////////////////////////////////////////////////////////////////////////////////////

	UINT16 i = Len;
	unsigned char tmp;
	/* send the address and command to the ESC */
 	ISR_AddressingEsc( Address, ESC_WR );
	/* loop for all bytes to be written */
#ifdef BURST_DMA
        int idx = 0;


        while ( i-- > 0 )
	{
		SetTxBufIdx(idx, *pData);
                idx++;
                pData++;
        }



        DMA0CNT = Len-1;
        DMA1CNT = Len-1;
        ResetDMATx();

        //dma_running = 1;

        DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

        //while (dma_running);
        while(!IFS0bits.DMA1IF);
        IFS0bits.DMA1IF = 0;

#else

	while ( i-- > 0 )
	{
#ifdef ECAT_DMA
                DMA0CNT = 0;
                DMA1CNT = 0;
                ResetDMATx();

//                Spi1TxBuffA[0] = (unsigned int)0xFF;
                SetTxBuf(*pData);

//                dma_running = 1;
                DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

                //while (dma_running);
                while(!IFS0bits.DMA1IF);
                IFS0bits.DMA1IF = 0;

               pData++;
        }

#else

		SSPIF = 0;
		/* start transmission */
		SSPBUF = *pData;
		/* wait until transmission finished */
		while ( !SSPIF );
		tmp=SSPBUF;  //Throw away read
		/* increment data pointer */
		pData++;
                /* reset transmission flag */

	}

	/* reset transmission flag */
	SSPIF = 0;/* reset transmission flag */
#endif
#endif
	/* there has to be at least 15 ns + CLK/2 after the transmission is finished
	   before the SPI_SEL signal shall be 1 */
	SPI_SEL = SPI_DEACTIVE;

	/* at this time the result of the transmission can be checked */

}

/////////////////////////////////////////////////////////////////////////////
void ISR_AddressingEscDMA( UINT16 Address, UINT8 Command )
{
	/////////////////////////////////////////////////////////////////////////////////////////
	/**
	 \param Address 	EtherCAT ASIC address ( upper limit is 0x1FFF )	for access.
	 \param Command	ESC_WR performs a write access; ESC_RD performs a read access.

	 \brief The function addresses the EtherCAT ASIC via SPI for a following SPI access.
	*////////////////////////////////////////////////////////////////////////////////////////

        UBYTETOWORD tmp;
	char read;
	tmp.Word = ( Address << 3 ) | Command;



	SetTxBufIdx(tx_idx, tmp.Byte[1]);

        tx_idx++;

	SetTxBufIdx(tx_idx, tmp.Byte[0]);

   	tx_idx++;


}



/////////////////////////////////////////////////////////////////////////////
void ISR_EscReadAccessDMA( UINT8 *pData, UINT16 Address, UINT16 Len )
{
	/////////////////////////////////////////////////////////////////////////////////////////
	/**
	 \param pData		Pointer to a byte array which holds data to write or saves read data.
	 \param Address 	EtherCAT ASIC address ( upper limit is 0x1FFF )	for access.
	 \param Len			Access size in Bytes.

	 \return	Indicates if the access was succesful ( TRUE ).

	 \brief  This function operates the SPI access to the EtherCAT ASIC.
	*////////////////////////////////////////////////////////////////////////////////////////
	UINT16 i = Len;
	UINT8 data = 0;

        tx_idx = 0;
        rx_idx = 0;

	/* send the address and command to the ESC */
 	ISR_AddressingEscDMA( Address, ESC_RD );

        did_rx = 1;

        int idx = 0;
        while ( i-- > 0 )
	{
		if ( i == 0 )
		{
			/* when reading the last byte the DI pin shall be 1 */
			data = 0xFF;
		}
                SetTxBufIdx(tx_idx, data);
                tx_idx++;
                //idx++;
        }


}


/////////////////////////////////////////////////////////////////////////////
void ISR_EscWriteAccessDMA( UINT8 *pData, UINT16 Address, UINT16 Len )
{
	/////////////////////////////////////////////////////////////////////////////////////////
	/**
	 \param pData		Pointer to a byte array which holds data to write or saves read data.
	 \param Address 	EtherCAT ASIC address ( upper limit is 0x1FFF )	for access.
	 \param Len			Access size in Bytes.

	 \return	Indicates if the access was succesful ( TRUE ).

	 \brief  This function operates the SPI access to the EtherCAT ASIC.
	*////////////////////////////////////////////////////////////////////////////////////////

	UINT16 i = Len;
	unsigned char tmp;

        tx_idx = 0;
        rx_idx = 0;

	/* send the address and command to the ESC */
 	ISR_AddressingEscDMA( Address, ESC_WR );
	/* loop for all bytes to be written */

        did_tx = 1;


        int idx = 0;

        while ( i-- > 0 )
	{
		SetTxBufIdx(tx_idx, *pData);
                tx_idx++;
                //idx++;
                pData++;
        }



}

/////////////////////////////////////////////////////////////////////////////
void ISR_StartDMA(  )
{

        DMA0CNT = tx_idx-1;
        DMA1CNT = tx_idx-1;

        IEC0bits.DMA1IE  = 1;			// Enable DMA interrupt
        //dma_running = 1;
        ResetDMATx();

        dma_running = 1;
//        tx_cnt++;
        DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer


}

/////////////////////////////////////////////////////////////////////////////
void HW_Main(void)
{
	//Handle state changes and watchdog from main() cyclic loop
	GetInterruptRegister();
//	LATDbits.LATD11 = 0;
	//Handle Watchdog
	if (u16WdValue != 0 && bEcatOutputUpdateRunning && bEcatFirstOutputsReceived)
	{
		#ifdef EC_USE_WATCHDOG
		//check the watchdog only in OP
		//generate the ms-timer
		long ts=get_timestamp_us();
		if ( ts-ec_wd_timestamp> EC_WATCHDOG_US)
		{
			//watchdog expired
			ec_watchdog_expired=1;
			nAlStatusCode = ALSTATUSCODE_SMWATCHDOG;
			AL_ControlInd(STATE_SAFEOP);
		}
		#endif
	}

	// check AL_CONTROL event first
	if ( EscAlEvent.Byte[0] & ((UINT8) AL_CONTROL_EVENT) )
	{
		/* get the AL Control register sent by the Master to acknowledge the interrupt */
		HW_EscReadAccess( &EscAlControl.Byte[0], ESC_ADDR_ALCONTROL, sizeof(UALCONTROL) );


		/* ECAT_CHANGE_BEGIN V3.11 */
		/* acknowledge AL-Control-event and the SM-Change event (because it was handled too) */
		EscAlEvent.Byte[0] &= ~(((UINT8) AL_CONTROL_EVENT) | ((UINT8) SM_CHANGE_EVENT));


		/* ECAT_CHANGE_END V3.11 */
		AL_ControlInd(EscAlControl.Byte[0]);


	}

	#if SM_CHANGE_SUPPORTED
	if ( EscAlEvent.Byte[0] & SM_CHANGE_EVENT )
	{
		// call AL_Control with status unchanged
		AL_ControlInd(nAlStatus & STATE_MASK);
		EscAlEvent.Byte[0] &= ~((UINT8) SM_CHANGE_EVENT);
	}
	#endif
//	LATDbits.LATD11 = 0;
}
/////////////////////////////////////////////////////////////////////////////
void HW_EscReadAccess( UINT8 *pData, UINT16 Address, UINT16 Len )
{
	////////////////////////////////////////////////////////////////////////////////////////
	/**
	 \param pData		Pointer to a byte array which holds data to write or saves read data.
	 \param Address 	EtherCAT ASIC address ( upper limit is 0x1FFF )	for access.
	 \param Len			Access size in Bytes.

	 \return	Indicates if the access was succesful ( TRUE ).

	 \brief  This function operates the SPI access to the EtherCAT ASIC.
	*////////////////////////////////////////////////////////////////////////////////////////

	/* HBu 24.01.06: if the SPI will be read by an interrupt routine too the
     mailbox reading may be interrupted but an interrupted
     reading will remain in a SPI transmission fault that will
     reset the internal Sync Manager status. Therefore the reading
     will be divided in 1-byte reads with disabled interrupt */

	UINT16 i = Len;

	while ( i-- > 0 )						/* loop for all bytes to be read */
	{
		#if AL_EVENT_ENABLED
		/* the reading of data from the ESC can be interrupted by the
		   AL Event ISR, in that case the address has to be reinitialized,
		   in that case the status flag will indicate an error because
		   the reading operation was interrupted without setting the last
		   sent byte to 0xFF */
		DISABLE_AL_EVENT_INT;
		#endif

if (!IEC0bits.DMA1IE)
{
		AddressingEsc( Address, ESC_RD );

//#ifdef ECAT_DMA
#if 0
                DMA0CNT = 0;
                DMA1CNT = 0;
                ResetDMATx();

//                Spi1TxBuffA[0] = (unsigned int)0xFF;
                SetTxBuf(0xFF);

                //dma_running = 1;
                DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

                while(!IFS0bits.DMA1IF);
                IFS0bits.DMA1IF = 0;
                //while (dma_running);
                //while(DMA0REQbits.FORCE==1);

               *pData++ = GetRxBuf();

#else

		/* start transmission */
		SSPBUF = 0xFF;						/* when reading the last byte the DI pin shall be 1 */
		while ( !SSPIF );					/* wait until transmission finished */
		*pData++ = SSPBUF;					/* get data byte */

                SSPIF = 0;							/* reset transmission flag */
#endif
		/* enable the ESC interrupt to get the AL Event ISR the chance to interrupt,
		   if the next byte is the last the transmission shall not be interrupted,
		   otherwise a sync manager could unlock the buffer, because the last was
		   read internally */
}
//		ENABLE_AL_EVENT_INT;

		/* there has to be at least 15 ns + CLK/2 after the transmission is finished
		   before the SPI_SEL signal shall be 1 */
		SPI_SEL = SPI_DEACTIVE;

		Address++;							/* next address */

	}
}


/////////////////////////////////////////////////////////////////////////////
void HW_EscWriteAccess( UINT8 *pData, UINT16 Address, UINT16 Len )
{
	/////////////////////////////////////////////////////////////////////////////////////////
	/**
	 \param pData		Pointer to a byte array which holds data to write or saves read data.
	 \param Address 	EtherCAT ASIC address ( upper limit is 0x1FFF )	for access.
	 \param Len			Access size in Bytes.

	 \return	Indicates if the access was succesful ( TRUE ).

	 \brief  This function operates the SPI access to the EtherCAT ASIC.
	*////////////////////////////////////////////////////////////////////////////////////////

	UINT16 i = Len;
	unsigned char tmp;

	/* loop for all bytes to be written */
	while ( i-- > 0 )
	{
		/* the reading of data from the ESC can be interrupted by the
		   AL Event ISR, so evrey byte will be written seperate */
		DISABLE_AL_EVENT_INT;

if (!IEC0bits.DMA1IE)
{

	 	AddressingEsc( Address, ESC_WR );		/* HBu 24.01.06: wrong parameter ESC_RD */

//#ifdef ECAT_DMA
#if 0
                DMA0CNT = 0;
                DMA1CNT = 0;
                ResetDMATx();

                SetTxBuf(*pData++);
                //Spi1TxBuffA[0] = *pData++;

                //dma_running = 1;
                DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer

                while(!IFS0bits.DMA1IF);
                IFS0bits.DMA1IF = 0;
                //while (dma_running);
                //while(DMA0REQbits.FORCE==1);

               //tmp = (unsigned char)Spi1RxBuffA[0];

#else
		SSPIF = 0;
		SSPBUF = *pData++;						/* start transmission */
		while ( !SSPIF );						/* wait until transmission finished */
		tmp = SSPBUF;							//Throw away read
                SSPIF = 0;

#endif
		/* enable the ESC interrupt to get the AL Event ISR the chance to interrupt */
}
//		ENABLE_AL_EVENT_INT;
		Address++;								/* next address */

		/* reset transmission flag */
		/* there has to be at least 15 ns + CLK/2 after the transmission is finished
		   before the SPI_SEL signal shall be 1 */
		SPI_SEL = SPI_DEACTIVE;
  

	}
}
/////////////////////////////////////////////////////////////////////////////
void 	HW_ResetIntMask(UINT16 intMask)
{
	UINT16 mask;
	HW_EscReadAccess( (UINT8 *)(&mask), ESC_ADDR_ALEVENTMASK, 2 );
	mask &= intMask;

	#if AL_EVENT_ENABLED
	DISABLE_AL_EVENT_INT;
if (!IEC0bits.DMA1IE)
{
	if ( mask == 0 )
	{
		bAlEventEnabled = FALSE;
		START_TIMER;
	}
	#endif

	HW_EscWriteAccess( (UINT8 *)(&mask), ESC_ADDR_ALEVENTMASK, 2 );
}
	#if AL_EVENT_ENABLED
//	ENABLE_AL_EVENT_INT;
	#endif
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param	intMask		interrupt mask (enabled interrupt shall be one)

 \brief	This function makes a logical or with the AL Event Mask register
*////////////////////////////////////////////////////////////////////////////////////////

void 	HW_SetIntMask(UINT16 intMask)
{
	UINT16 mask;
	HW_EscReadAccess( (UINT8 *)(&mask), ESC_ADDR_ALEVENTMASK, 2 );
	mask |= intMask;

	#if AL_EVENT_ENABLED
	DISABLE_AL_EVENT_INT;
	STOP_TIMER;
	ACK_TIMER_INT;
	bAlEventEnabled = TRUE;
	#endif

if (!IEC0bits.DMA1IE)
{


	HW_EscWriteAccess( (UINT8 *)(&mask), ESC_ADDR_ALEVENTMASK, 2 );
}
	#if AL_EVENT_ENABLED
//	ENABLE_AL_EVENT_INT;
	#endif
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param alStatus	The new EtherCAT ASIC state.

 \brief  The function changes the state of the EtherCAT ASIC to the requested.
*////////////////////////////////////////////////////////////////////////////////////////

void HW_SetAlStatus(UINT8 alStatus, UINT8 alStatusCode)
{
	UINT16 state = alStatus;
	HW_EscWriteAccess( (UINT8 *)(&state), ESC_ADDR_ALSTATUS, 2 );
	if (alStatusCode != 0xFF)
	{
		state = alStatusCode;
		HW_EscWriteAccess( (UINT8 *)(&state), ESC_ADDR_ALSTATUSCODE, 2 );
	}
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	channel		Sync Manager channel

 \brief	This function disables a Sync Manager channel
*////////////////////////////////////////////////////////////////////////////////////////

void HW_DisableSyncManChannel(UINT8 channel)
{
	UINT8 value = 1;
	/* HBu 13.02.06: the offset of the Sync manager settings had to be added */
	HW_EscWriteAccess( &value, ESC_ADDR_SYNCMAN+sizeof(TSYNCMAN)*channel+ESC_OFFS_SMSETTINGS+ESC_OFFS_PDIDISABLE, 1 );
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	channel		Sync Manager channel

 \brief	This function enables a Sync Manager channel
*////////////////////////////////////////////////////////////////////////////////////////

void HW_EnableSyncManChannel(UINT8 channel)
{
	UINT8 value = 0;
	/* HBu 13.02.06: the offset of the Sync manager settings had to be added */
	HW_EscWriteAccess( &value, ESC_ADDR_SYNCMAN+sizeof(TSYNCMAN)*channel+ESC_OFFS_SMSETTINGS+ESC_OFFS_PDIDISABLE, 1 );
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pPdSyncMan	Pointer to a return pointer to the SYNC Manager channel describing
                     of two Sync Manager channels

 \return	Indicates if the SYNC Manager channel descriptions could be delivered completely
 			(0: finished).

 \brief  This function is called to read the SYNC Manager channel descriptions of the
 			process data SYNC Managers.
*////////////////////////////////////////////////////////////////////////////////////////

TSYNCMAN * HW_GetSyncMan(UINT8 channel)
{
	// get a temporary structure of the Sync Manager:
	HW_EscReadAccess( (UINT8 *) &TmpSyncMan, ESC_ADDR_SYNCMAN + (channel * sizeof(TSYNCMAN)), sizeof(TSYNCMAN) );
	return &TmpSyncMan;
}



#endif

