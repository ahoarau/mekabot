 /*
Copyright (c) 2010, Meka Robotics
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


#include "p33Fxxxx.h"
#include "setup.h"
#include "adc.h"


int adc_idx;
int adc_idx_fast;
unsigned int adc_raw[ADC_NUM_CH];
unsigned int volatile adc_buffer[ADC_NUM_CH][ADC_NUM_SMOOTH];

unsigned int BufferA[DMA_BUF_DEPTH] __attribute__( (space(dma),aligned(4)) );
unsigned int BufferB[DMA_BUF_DEPTH] __attribute__( (space(dma),aligned(4)) );


unsigned int get_avg_adc(int ch)
{
	long v;
	int i;
	v=0;
	for (i=0;i<ADC_NUM_SMOOTH;i++)
		v=v+adc_buffer[ch][i];
	return (unsigned int)(v>>ADC_SHIFT_SMOOTH);
}



void setup_adc(void)
{

	adc_idx			= 0;
	adc_idx_fast	= 0;

	AD1CON1bits.ADON	= 0;					//Turn off ADC

	//Configure A/D to convert AN0-AN(ADC_NUM_CH-1) using CH0
	//Select port pins
	//AD1PCFGH/AD1PCFGL: Port Configuration Register
	ADPCFG	= 0;

	//Select voltage reference
	AD1CON2bits.VCFG = 0;           // AVdd/AVss
	//AD1CON2bits.VCFG = 3;           // Vref+/Vref-
	//AD1CON2bits.VCFG = 2;           // Avdd/Vref-
	//Select conversion clock to get around 100K sample cycles per second, or sample cycle period of 10us
	//For 12bit, conversion time = Tc=14TAD
	//For Sample time = 3*TAD, and 6 inputs, one sample cycle takes (14+3)*6*TAD=102TAD
	//TAD=10us/102 ~= 100ns = 10,000 per ms
	//ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*(ADCS+1)  = 150ns
	//ADCS = 5
	// 10,000ns PWM period. TCY is 25ns. TAD is 50ns.
	// After PWM trigger, wait for 2500ns (SEVTCMP=100)
	// Then do 4 samples, 17TAD each or 3400ns total.
	AD1CON3bits.ADRC	= 0;			// ADC Clock is derived from Systems Clock

	AD1CON1bits.ADDMABM	= 0;			// DMA in scatter/gather
	AD1CON1bits.AD12B	= 1;			// 12-bit ADC operation
	AD1CON1bits.FORM	= 0;			// Integer Output Format (0B 0000 dddd dddd dddd )
	AD1CON1bits.SSRC	= 0b111;		// Auto StartOfConversion
	AD1CON1bits.SIMSAM	= 0;			// No simultaneous sample for 1CH
	AD1CON1bits.ASAM	= 1;			// Sampling begins immediately after conversion is done

	AD1CON2bits.CSCNA	= 1;			// Enable channel scanning
	AD1CON2bits.CHPS	= 0;			// Only convert CH0	in 12-bit mode
	AD1CON2bits.SMPI	= ADC_NUM_CH-1;	// Select number of conversions between interrupts
	AD1CON2bits.BUFM	= 0;			// Use 1x16-word buffer for conversion sequences
	AD1CON2bits.ALTS	= 0;

	AD1CON3bits.ADRC	= 0;					// ADC Clock is derived from Systems Clock
	AD1CON3bits.SAMC	= 10;
	AD1CON3bits.ADCS	= 40;

	AD1CON4bits.DMABL	= 0;//ADC_NUM_SAMPLES-1;	// Allocates ADC_NUM_SAMPLES words of buffer to each analog input


	// AD1PCFGL: ADC1 PORT CONFIGURATION REGISTER LOW
    // AD1PCFGLBITS.PCFGx=0 defines the Pin as an Analog input.
    // AD1PCFGLBITS.PCFGx=1 defines the Pin as Digital.
	AD1PCFGLbits.PCFG0	= 0;
    AD1PCFGLbits.PCFG1	= 0;
    AD1PCFGLbits.PCFG2	= 0;
    AD1PCFGLbits.PCFG3	= 1;
    AD1PCFGLbits.PCFG4	= 1;
    AD1PCFGLbits.PCFG5	= 1;
    AD1PCFGLbits.PCFG6	= 1;
    AD1PCFGLbits.PCFG7	= 1;
    AD1PCFGLbits.PCFG8	= 1;
	

	AD1CSSLbits.CSS0	= 1;
	AD1CSSLbits.CSS1	= 1;
	AD1CSSLbits.CSS2	= 1;



	//Enable interrupt
    _AD1IF = 0;
	_AD1IE = 0;

	//Turn on ADC
	AD1CON1bits.ADON = 1;		// Turn on the A/D converter
}




void setup_dma1(void)
{
	DMA1CONbits.SIZE	= 0;				// Word - yo
	DMA1CONbits.DIR		= 0;				// Read from Peripheral address, write to DPSRAM address
	DMA1CONbits.HALF	= 0;				// Initiate interrupt when all of the data has been moved
	DMA1CONbits.NULLW	= 0;				// Normal operation
	DMA1CONbits.AMODE	= 0;				// Configure DMA for Register indirect mode
	DMA1CONbits.MODE	= 2;				// Configure DMA for Continuous Ping-Pong mode

	DMA1PAD				=(uint16_t)&ADC1BUF0;

	DMA1REQbits.FORCE	= 0;				// Automatic DMA transfer initiation by DMA Request
	DMA1REQbits.IRQSEL	= 0b0001101;		// ADC1 ? ADC1 Convert done

	DMA1STA				= __builtin_dmaoffset(BufferA);	// Primary DPSRAM Start Address Offset bits (source or destination)
	DMA1STB				= __builtin_dmaoffset(BufferB);	// Secondary DPSRAM Start Address Offset bits (source or destination)

	DMA1CNT				= DMA_BUF_DEPTH - 1;	// DMA Transfer Count Register bits

	//Interrupts
	IFS0bits.DMA1IF = 0;					// Clear the DMA interrupt flag bit
	//IPC3bits.DMA1IP = 5;					// Highest-1

	IEC0bits.DMA1IE = 1;					//Set the DMA interrupt enable bit

	DMA1CONbits.CHEN = 1;					//Channel enabled
}	// end setup_dma1


/*
void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void)
{
	_AD1IF = 0;		//Clear the flag

	adc_raw[0]=ADC1BUF0;
	adc_raw[1]=ADC1BUF1;
	adc_raw[2]=ADC1BUF2;


	adc_buffer[ADC_MOTOR_TEMP][adc_idx]=adc_raw[ADC_MOTOR_TEMP];
	adc_buffer[ADC_AMP_TEMP_A][adc_idx]=adc_raw[ADC_AMP_TEMP_A];
	adc_buffer[ADC_AMP_TEMP_B][adc_idx]=adc_raw[ADC_AMP_TEMP_B];
	adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);

}
*/


void __attribute__((__interrupt__, no_auto_psv)) _DMA1Interrupt(void)
{
	uint16_t	* dma_buf_ptr;


	// attach to dma buffer with updated adc values
	if (DMACS1bits.PPST1)
		dma_buf_ptr = BufferA;
	else
		dma_buf_ptr = BufferB;



	adc_raw[0]=dma_buf_ptr[0];
	adc_raw[1]=dma_buf_ptr[1];
	adc_raw[2]=dma_buf_ptr[2];


	adc_buffer[ADC_MOTOR_TEMP][adc_idx]=adc_raw[ADC_MOTOR_TEMP];
	adc_buffer[ADC_AMP_TEMP_A][adc_idx]=adc_raw[ADC_AMP_TEMP_A];
	adc_buffer[ADC_AMP_TEMP_B][adc_idx]=adc_raw[ADC_AMP_TEMP_B];
	adc_idx = INC_MOD(adc_idx,ADC_NUM_SMOOTH);

	_DMA1IF = 0;		//Clear the flag
}