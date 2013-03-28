 /*
Copyright � 2010, Meka Robotics
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


int  BufferA[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
int  BufferB[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));

int  Buffer[MAX_CHNUM+1][SAMP_BUFF_SIZE];


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
	AD1CON3bits.SAMC=6; 			// SLOW: Auto Sample Time = 3*Tad

	AD1CON1bits.FORM   = 0;		// Data Output Format: Signed Fraction (Q15 format) 3
	//AD1CON1bits.SSRC   = 2;		// Sample Clock Source: GP Timer starts conversion
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON1bits.ASAM   = 1;		// ADC Sample Control: Sampling begins immediately after conversion
	AD1CON1bits.AD12B  = 1;		// 10-bit ADC operation


	AD1CON2bits.CSCNA = 1;		// Scan Input Selections for CH0+ during Sample A bit
	AD1CON2bits.CHPS  = 0;		// Converts CH0

	AD1CON3bits.ADRC = 0;		// ADC Clock is derived from Systems Clock
	//AD1CON3bits.ADCS = 63;		// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
	AD1CON3bits.ADCS=12;			// SLOW: System clock divider TAD=(ADCS+1)*TCY=5*=50ns (As fast as works...)
								// ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us

	AD1CON1bits.ADDMABM = 0; 	// DMA buffers are built in scatter/gather mode
	AD1CON2bits.SMPI    = (ADC_NUM_CH-1);	// 4 ADC Channel is scanned
	AD1CON4bits.DMABL   = 3;	// Each buffer contains 8 words

	//AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
#if defined HB2_H2R2_J0J1 || defined HB2_0_2_H2R3_J0J1
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;
	AD1CSSLbits.CSS2=1;
	AD1CSSLbits.CSS3=1;
	AD1CSSLbits.CSS4=1;
	AD1CSSLbits.CSS5=1;
        // Analog Inputs
      	AD1PCFGLbits.PCFG0 = 0;
	AD1PCFGLbits.PCFG1 = 0;
        AD1PCFGLbits.PCFG2 = 0;
        AD1PCFGLbits.PCFG3 = 0;
        AD1PCFGLbits.PCFG4 = 0;
        AD1PCFGLbits.PCFG5 = 0;

#endif

#if defined HB2_H2R2_J2J3J4 || defined HB2_0_2_H2R3_J2J3J4
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;
	AD1CSSLbits.CSS2=1;
	AD1CSSLbits.CSS3=1;
	AD1CSSLbits.CSS4=1;
	AD1CSSLbits.CSS5=1;
	AD1CSSLbits.CSS6=1;
        // Analog Inputs
      	AD1PCFGLbits.PCFG0 = 0;
	AD1PCFGLbits.PCFG1 = 0;
        AD1PCFGLbits.PCFG2 = 0;
        AD1PCFGLbits.PCFG3 = 0;
        AD1PCFGLbits.PCFG4 = 0;
        AD1PCFGLbits.PCFG5 = 0;
        AD1PCFGLbits.PCFG6 = 0;
#endif

 	//AD1PCFGH/AD1PCFGL: Port Configuration Register
	AD1PCFGL=0xFFFF;


	IFS0bits.AD1IF   = 0;		// Clear the A/D interrupt flag bit
	IEC0bits.AD1IE   = 0;		// Do Not Enable A/D interrupt
	AD1CON1bits.ADON = 1;		// Turn on the A/D converter

}

// DMA0 configuration
// Direction: Read from peripheral address 0-x300 (ADC1BUF0) and write to DMA RAM
// AMODE: Peripheral Indirect Addressing Mode
// MODE: Continuous, Ping-Pong Mode
// IRQ: ADC Interrupt

void initDma0(void)
{
	DMA0CONbits.AMODE = 2;			// Configure DMA for Peripheral indirect mode
	DMA0CONbits.MODE  = 2;			// Configure DMA for Continuous Ping-Pong mode
	DMA0PAD=(int)&ADC1BUF0;
	DMA0CNT = (SAMP_BUFF_SIZE*ADC_NUM_CH)-1;
	DMA0REQ = 13;					// Select ADC1 as DMA Request source

	DMA0STA = __builtin_dmaoffset(BufferA);
	DMA0STB = __builtin_dmaoffset(BufferB);

	IFS0bits.DMA0IF = 0;			//Clear the DMA interrupt flag bit
        IEC0bits.DMA0IE = 1;			//Set the DMA interrupt enable bit

	DMA0CONbits.CHEN=1;				// Enable DMA

}


/*=============================================================================
_DMA0Interrupt(): ISR name is chosen from the device linker script.
=============================================================================*/

unsigned int DmaBuffer = 0;

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
    int j;
    adc_idx=0;
    
	if(DmaBuffer == 0)
	{
            for (j = 0; j < 8; j++)
            {
#if defined HB2_H2R2_J0J1  || defined HB2_0_2_H2R3_J0J1
                adc_buffer[ADC_SEAS_B][(adc_idx*8)+j] = BufferA[ADC_SEAS_B][j];
                adc_buffer[ADC_CURRENT_B][(adc_idx*8)+j] = BufferA[ADC_CURRENT_B][j];
                adc_buffer[ADC_SEAS_A][(adc_idx*8)+j] = BufferA[ADC_SEAS_A][j];
                adc_buffer[ADC_CURRENT_A][(adc_idx*8)+j] = BufferA[ADC_CURRENT_A][j];
                adc_buffer[ADC_AMP_TEMP_B][(adc_idx*8)+j] = BufferA[ADC_AMP_TEMP_B][j];
                adc_buffer[ADC_AMP_TEMP_A][(adc_idx*8)+j] = BufferA[ADC_AMP_TEMP_A][j];
#endif
#if defined HB2_H2R2_J2J3J4 || defined HB2_0_2_H2R3_J2J3J4
                adc_buffer[ADC_SEAS_A][(adc_idx*8)+j] = BufferA[ADC_SEAS_A][j];
                adc_buffer[ADC_CURRENT_A][(adc_idx*8)+j] = BufferA[ADC_CURRENT_A][j];
                adc_buffer[ADC_SEAS_B][(adc_idx*8)+j] = BufferA[ADC_SEAS_B][j];
                adc_buffer[ADC_CURRENT_B][(adc_idx*8)+j] = BufferA[ADC_CURRENT_B][j];
                adc_buffer[ADC_SEAS_C][(adc_idx*8)+j] = BufferA[ADC_SEAS_C][j];
                adc_buffer[ADC_AMP_TEMP_A][(adc_idx*8)+j] = BufferA[ADC_AMP_TEMP_A][j];
#endif
            }

	}
	else
	{
            for (j = 0; j < 8; j++)
            {
#if defined HB2_H2R2_J0J1  || defined HB2_0_2_H2R3_J0J1
                adc_buffer[ADC_SEAS_B][(adc_idx*8)+j] = BufferB[ADC_SEAS_B][j];
                adc_buffer[ADC_CURRENT_B][(adc_idx*8)+j] = BufferB[ADC_CURRENT_B][j];
                adc_buffer[ADC_SEAS_A][(adc_idx*8)+j] = BufferB[ADC_SEAS_A][j];
                adc_buffer[ADC_CURRENT_A][(adc_idx*8)+j] = BufferB[ADC_CURRENT_A][j];
                adc_buffer[ADC_AMP_TEMP_B][(adc_idx*8)+j] = BufferB[ADC_AMP_TEMP_B][j];
                adc_buffer[ADC_AMP_TEMP_A][(adc_idx*8)+j] = BufferB[ADC_AMP_TEMP_A][j];
#endif

#if defined HB2_H2R2_J2J3J4 || defined HB2_0_2_H2R3_J2J3J4
                adc_buffer[ADC_SEAS_A][(adc_idx*8)+j] = BufferA[ADC_SEAS_A][j];
                adc_buffer[ADC_CURRENT_A][(adc_idx*8)+j] = BufferA[ADC_CURRENT_A][j];
                adc_buffer[ADC_SEAS_B][(adc_idx*8)+j] = BufferA[ADC_SEAS_B][j];
                adc_buffer[ADC_CURRENT_B][(adc_idx*8)+j] = BufferA[ADC_CURRENT_B][j];
                adc_buffer[ADC_SEAS_C][(adc_idx*8)+j] = BufferA[ADC_SEAS_C][j];
                adc_buffer[ADC_AMP_TEMP_A][(adc_idx*8)+j] = BufferA[ADC_AMP_TEMP_A][j];
#endif                
            }
	}

	DmaBuffer ^= 1;

        //adc_idx=INC_MOD(adc_idx,4);
        
	IFS0bits.DMA0IF = 0;		// Clear the DMA0 Interrupt Flag
}


void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void)
{
	_AD1IF = 0;		//Clear the flag

}

#endif
