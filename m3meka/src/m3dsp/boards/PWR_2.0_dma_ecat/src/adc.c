 /*
Copyright ? 2010, Meka Robotics
All rights reserved.
http://mekabot.com

Redistribution and use in source and binary forms, with or without
modification, are permitted.


THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef USE_ADC

#include "p33Fxxxx.h"
#include "setup.h"
#include "adc.h"


int adc_idx;
int adc_idx_fast;
unsigned int adc_raw[ADC_NUM_CH];
unsigned int volatile adc_buffer[ADC_NUM_CH][ADC_NUM_SMOOTH];


int  BufferA[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(128)));
int  BufferB[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(128)));


unsigned int get_avg_adc(int ch)
{
	long v;
	int i;
	v=0;
	for (i=0;i<ADC_NUM_SMOOTH;i++)
		v=v+adc_buffer[ch][i];
	return (unsigned int)(v>>ADC_SHIFT_SMOOTH);
}

void setup_adc(void) {
	adc_idx=0;
	adc_idx_fast=0;

        AD1CON2bits.VCFG = 0;           // AVdd/AVss
	AD1CON3bits.SAMC=10; 			// SLOW: Auto Sample Time = 3*Tad

	AD1CON1bits.FORM   = 0;		// Data Output Format: Signed Fraction (Q15 format) 3
	//AD1CON1bits.SSRC   = 2;		// Sample Clock Source: GP Timer starts conversion
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON1bits.ASAM   = 1;		// ADC Sample Control: Sampling begins immediately after conversion
	AD1CON1bits.AD12B  = 1;		// 10-bit ADC operation


	AD1CON2bits.CSCNA = 1;		// Scan Input Selections for CH0+ during Sample A bit
	AD1CON2bits.CHPS  = 0;		// Converts CH0

	AD1CON3bits.ADRC = 0;		// ADC Clock is derived from Systems Clock
	//AD1CON3bits.ADCS = 63;		// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
	AD1CON3bits.ADCS=40;			// SLOW: System clock divider TAD=(ADCS+1)*TCY=5*=50ns (As fast as works...)
								// ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us

	AD1CON1bits.ADDMABM = 0; 	// DMA buffers are built in scatter/gather mode
	AD1CON2bits.SMPI    = (ADC_NUM_CH-1);	// 4 ADC Channel is scanned
	AD1CON4bits.DMABL   = 3;	// Each buffer contains 8 words

	//AD1CSSH/AD1CSSL: A/D Input Scan Selection Register

	AD1CSSLbits.CSS0=1;			// Enable AN4 for channel scan
	AD1CSSLbits.CSS1=1;			// Enable AN5 for channel scan
	AD1CSSLbits.CSS2=1;		// Enable AN10 for channel scan


 	//AD1PCFGH/AD1PCFGL: Port Configuration Register
	AD1PCFGL=0xFFFF;

	AD1PCFGLbits.PCFG0 = 0;		// AN0 as Analog Input
	AD1PCFGLbits.PCFG1 = 0;		// AN1 as Analog Input
        AD1PCFGLbits.PCFG2 = 0;		// AN2 as Analog Input


	IFS0bits.AD1IF   = 0;		// Clear the A/D interrupt flag bit
	IEC0bits.AD1IE   = 0;		// Do Not Enable A/D interrupt
	AD1CON1bits.ADON = 1;		// Turn on the A/D converter

}

// DMA2 configuration
// Direction: Read from peripheral address 0-x300 (ADC1BUF0) and write to DMA RAM
// AMODE: Peripheral Indirect Addressing Mode
// MODE: Continuous, Ping-Pong Mode
// IRQ: ADC Interrupt

void initDma2(void)
{
	DMA2CONbits.AMODE = 2;			// Configure DMA for Peripheral indirect mode
	DMA2CONbits.MODE  = 2;			// Configure DMA for Continuous Ping-Pong mode
	DMA2PAD=(int)&ADC1BUF0;
	DMA2CNT = (SAMP_BUFF_SIZE*ADC_NUM_CH)-1;
	DMA2REQ = 13;					// Select ADC1 as DMA Request source

	DMA2STA = __builtin_dmaoffset(BufferA);
	DMA2STB = __builtin_dmaoffset(BufferB);

	IFS1bits.DMA2IF = 0;			//Clear the DMA interrupt flag bit
        IEC1bits.DMA2IE = 1;			//Set the DMA interrupt enable bit

	DMA2CONbits.CHEN=1;				// Enable DMA

}


/*=============================================================================
_DMA0Interrupt(): ISR name is chosen from the device linker script.
=============================================================================*/

unsigned int DmaBuffer = 0;



void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void)
{
    int j;
	if(DmaBuffer == 0)
	{
            for (j = 0; j < 8; j++)
            {

                adc_buffer[ADC_BUS_VOLTAGE][(adc_idx*8)+j] = BufferA[0][j];
		adc_buffer[ADC_CURRENT_DIGITAL][(adc_idx*8)+j] = BufferA[1][j];
		adc_buffer[ADC_EXT][(adc_idx*8)+j] = BufferA[2][j];

            }
	}
	else
	{
            for (j = 0; j < 8; j++)
            {

                adc_buffer[ADC_BUS_VOLTAGE][(adc_idx*8)+j] = BufferB[0][j];
		adc_buffer[ADC_CURRENT_DIGITAL][(adc_idx*8)+j] = BufferB[1][j];
		adc_buffer[ADC_EXT][(adc_idx*8)+j] = BufferB[2][j];

            }

	}

	DmaBuffer ^= 1;

        adc_idx=INC_MOD(adc_idx,4);


	IFS1bits.DMA2IF = 0;		// Clear the DMA0 Interrupt Flag
}


void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void)
{
	_AD1IF = 0;		//Clear the flag

}

#endif
