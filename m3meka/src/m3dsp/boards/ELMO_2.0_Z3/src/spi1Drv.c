/**********************************************************************
* ? 2005 Microchip Technology Inc.
*
* FileName:        spi1Drv.c
* Dependencies:    Header (.h) files if applicable, see below
* Processor:       dsPIC33Fxxxx/PIC24Hxxxx
* Compiler:        MPLAB? C30 v3.00 or higher
* Tested On:	   dsPIC33FJ256GP710
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author            Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RK	          04/06/06 	  First release of source file
*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* ADDITIONAL NOTES.
**********************************************************************/

#include "setup.h"
#include "ethercat_hw.h"
#include "ethercat.h"

void cfgDma0SpiTx(void);
void cfgDma1SpiRx(void);


unsigned char Spi1RxBuffA[128] __attribute__((space(dma)));
unsigned int Spi1TxBuffA[128] __attribute__((space(dma)));




//unsigned int Spi1TxBuffB[2] __attribute__((space(dma)));
/*=============================================================================
_DMA0Init(): Initialise DMA0 for SPI Data Transmission 
=============================================================================*/
// DMA0 configuration
// Direction: Read from DMA RAM and write to SPI buffer
// AMODE: Register Indirect with Post-Increment mode
// MODE: Continuous, Ping-Pong Enabled
// IRQ: SPI

void cfgDma0SpiTx(void)
{
			
	DMA0CON = 0x2001;					
	
	DMA0CNT = 0;
	DMA0REQ = 0x00A;					

	DMA0PAD = (volatile unsigned int) &SPI1BUF;
	DMA0STA= __builtin_dmaoffset(Spi1TxBuffA);
	
	DMA1CONbits.SIZE = 1;			// 8-bit
	IFS0bits.DMA0IF  = 0;			// Clear DMA interrupt
	IEC0bits.DMA0IE  = 1;			// Enable DMA interrupt
	DMA0CONbits.CHEN = 0;			// Enable DMA Channel
	
}

// DMA1 configuration
// Direction: Read from SPI buffer and write to DMA RAM 
// AMODE: Register Indirect with Post-Increment mode
// MODE: Continuous, Ping-Pong Enabled
// IRQ: SPI
void cfgDma1SpiRx(void)
{
    
	DMA1CON = 0x0001;				
	DMA1CNT = 0;
	DMA1REQ = 0x00A;					

	DMA1PAD = (volatile unsigned int) &SPI1BUF;
	DMA1STA= __builtin_dmaoffset(Spi1RxBuffA);	
	
	DMA1CONbits.SIZE = 1;			// 8-bit
	IFS0bits.DMA1IF  = 0;			// Clear DMA interrupt	
	DMA1CONbits.CHEN = 0;			// Enable DMA Channel
	IEC0bits.DMA1IE = 0;
}


void ResetDMATx(void)
{
	DMA0STA= __builtin_dmaoffset(Spi1TxBuffA);
	
	DMA1STA= __builtin_dmaoffset(Spi1RxBuffA);
	
	DMA1CONbits.CHEN = 1; // Enable DMA0 Channel
	DMA0CONbits.CHEN = 1; // Enable DMA0 Channel
}



/*=============================================================================
Interrupt Service Routines.
=============================================================================*/


void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
      DMA0CONbits.CHEN = 0; // Enable DMA0 Channel
      IFS0bits.DMA0IF = 0;		// Clear the DMA0 Interrupt Flag;

}



unsigned char GetRxBufIdx(int idx)
{

    return (unsigned char)Spi1RxBuffA[idx];

}



void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
{
    int i;

    DMA1CONbits.CHEN = 0; // Enable DMA0 Channel
    IFS0bits.DMA1IF = 0;
    IEC0bits.DMA1IE  = 0;			// Enable DMA interrupt
    
            
  
    if (did_rx)
    {

        did_rx = 0;
        
        SPI_SEL = SPI_DEACTIVE;
        //IFS0bits.DMA1IF = 0;
        //IEC0bits.DMA1IE  = 0;			// Enable DMA interrupt
        
        memcpy((unsigned char *)&ec_cmd_in, &Spi1RxBuffA[2],sizeof(ec_cmd_in_t));
        unpack_command_in();
        if (!do_tx)
        {
            //ClrHeartbeatLED;
            ENABLE_AL_EVENT_INT;
        }
        //step_control();
    }


    if (did_tx)
    {
        did_tx = 0;
        
        
        SPI_SEL = SPI_DEACTIVE;

        //IFS0bits.DMA1IF = 0;
        //IEC0bits.DMA1IE  = 0;			// Enable DMA interrupt

        //ClrHeartbeatLED;
        
        ENABLE_AL_EVENT_INT;
     
    }

  

    if (do_tx)
    {
        do_tx = 0;
        isr_update_inputs();
        SPI_SEL = SPI_ACTIVE;
        ISR_StartDMA();
        
    }
    
}
