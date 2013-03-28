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
	//Configure A/D to convert AN0-AN(ADC_NUM_CH-1) using CH0 	
	//Select port pins
	//AD1PCFGH/AD1PCFGL: Port Configuration Register
	ADPCFG = 0; 

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
	AD1CON3bits.ADRC=0;				// ADC Clock is derived from Systems Clock


#if defined PWR_0_2 || defined PWR_0_3 || defined PWR_0_4 || defined PWR_0_5
	AD1CON3bits.SAMC=10; 			// SLOW: Auto Sample Time = 3*Tad		
	AD1CON3bits.ADCS=40;			// SLOW: System clock divider TAD=(ADCS+1)*TCY=5*=50ns (As fast as works...)
#endif

	AD1CON2bits.CHPS  = 0;			// Only convert CH0	in 12-bit mode
	AD1CON1bits.ASAM   = 1;			// Sampling begins immediately after conversion is done
	AD1CON1bits.AD12B  = 1;			// 12-bit ADC operation
	AD1CON1bits.SIMSAM = 0;			// No simultaneous sample for 1CH
	AD1CON2bits.CSCNA=1;			// Enable channel scanning
	AD1CON2bits.SMPI=ADC_NUM_CH-1;	// Select number of conversions between interrupts 



#if defined PWR_0_2 || defined PWR_0_3 || defined PWR_0_4 || defined PWR_0_5
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=1;				// Use 2x8-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;			
#endif



	//Select results format
	AD1CON1bits.FORM   = 0;		// Integer Output Format (0B 0000 dddd dddd dddd )
	//Turn on ADC
	AD1CON1bits.ADON = 1;		// Turn on the A/D converter	
	//Enable interrupt
    _AD1IF = 0;
	_AD1IE = 1;
}


void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void)
{
	_AD1IF = 0;		//Clear the flag



#if defined PWR_0_2 || defined PWR_0_3 || defined PWR_0_4 || defined M3_DAC_0_1 || defined M3_ELMO_RNA_R0 || defined PWR_0_5
	if (AD1CON2bits.BUFS==0) //ADC module filling lower group, read from upper
	{
		adc_raw[0]=ADC1BUF8;
		adc_raw[1]=ADC1BUF9;
		adc_raw[2]=ADC1BUFA;
	}
	else
	{
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
	}
#endif



#if defined PWR_0_2 || defined PWR_0_3 || defined PWR_0_4 || defined PWR_0_5
		adc_buffer[ADC_BUS_VOLTAGE][adc_idx]=adc_raw[ADC_BUS_VOLTAGE];
		adc_buffer[ADC_CURRENT_DIGITAL][adc_idx]=adc_raw[ADC_CURRENT_DIGITAL];
		adc_buffer[ADC_EXT][adc_idx]=adc_raw[ADC_EXT];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif
}

#endif
