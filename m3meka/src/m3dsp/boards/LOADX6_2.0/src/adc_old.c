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



#include "p33Fxxxx.h"
#include "setup.h"
#include "adc.h"


int adc_idx;
int adc_idx_fast;
unsigned int adc_raw[ADC_NUM_CH];
unsigned int volatile adc_buffer[ADC_NUM_CH][ADC_NUM_SMOOTH];

#if defined M3_WMA_0_1 || defined M3_BMA_A1R1 || defined M3_DAC_0_1 || defined M3_MAX2 || \
	defined M3_BMW_A2R1 || defined M3_BMW_A2R2 || defined M3_ELMO_RNA_R0 || defined M3_BMW_A2R3
unsigned int volatile adc_buffer_fast[ADC_NUM_SMOOTH_FAST];
#endif

 
unsigned int get_avg_adc(int ch)
{
	long v;
	int i;
	v=0;
	for (i=0;i<ADC_NUM_SMOOTH;i++)
		v=v+adc_buffer[ch][i];
	return (unsigned int)(v>>ADC_SHIFT_SMOOTH);
}

#if defined M3_DAC_0_1 || defined M3_BMA_A1R1 || defined M3_WMA_0_1 || defined M3_MAX2 \
	|| defined M3_BMW_A2R1 || defined M3_BMW_A2R2|| defined M3_ELMO_RNA_R0 || defined M3_BMW_A2R3
unsigned int get_avg_adc_torque()
{
	int i;
	long v=0;
	for (i=0;i<ADC_NUM_SMOOTH_FAST;i++)
		v=v+adc_buffer_fast[i];
	return (unsigned int)(v>>ADC_SHIFT_SMOOTH_FAST);
}
#endif


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

#if defined M3_BMA_A1R1 || defined M3_WMA_0_1 || defined M3_DAC_0_1 || defined M3_MAX2 || defined M3_ELMO_RNA_R0 || defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
	//TAD=50ns. Sample time = 10TAD. Conversion time 14TAD.
	//Sample-conversion of 6 signals takes (14+10)*6*TAD=144TAD = 7.2us = ~138 cyles/ms
	AD1CON3bits.SAMC=10; 			// FAST: Auto Sample Time (in TAD units)
	AD1CON3bits.ADCS=1;				// FAST: System clock divider TAD=(ADCS+1)*TCY=25*x=50ns (As fast as works...)
#if defined M3_MAX2_BLDC_T2R2 || defined M3_MAX2_BDC_T2R2 || defined M3_MAX2_BDC_T2R3 || defined M3_MAX2_BLDC_T2R3
	//TAD=50ns. Sample time = 10TAD. Conversion time 14TAD.
	//Sample-conversion of 6 signals takes (14+10)*6*TAD=144TAD = 7.2us = ~138 cyles/ms
	AD1CON3bits.SAMC=10; 			// FAST: Auto Sample Time (in TAD units)
	AD1CON3bits.ADCS=1;				// FAST: System clock divider TAD=(ADCS+1)*TCY=25*2=50ns (As fast as works...)
#endif
#endif


#if defined M3_BMW_A2R1 
	//TAD=325ns. Sample time = 6TAD. Conversion time 14TAD.
	//One cycle: 5*(6+14)*325=22.5us=~44Khz
	//Therefore, average 32 samples for the 1khz EtherCAT cycle.
	AD1CON3bits.SAMC=6; 			// Auto Sample Time (in TAD units)
	AD1CON3bits.ADCS=8;				// System clock divider TAD=(ADCS+1)*TCY=25*x=225ns 
#endif

#if defined M3_BMW_A2R2 || defined M3_BMW_A2R3
	AD1CON3bits.SAMC=10; 		
	AD1CON3bits.ADCS=40;			
#endif

#if defined M3_HEX4_S2R1
	AD1CON3bits.SAMC=10; 		
	AD1CON3bits.ADCS=40;			
#endif

#ifdef M3_HMB_H1R1
	//TAD=50ns. Sample time = 3TAD. Conversion time 14TAD.
	//Sample-conversion of 6 signals takes (14+3)*6*TAD=102TAD = 5.1us = ~200 cyles/ms
	AD1CON3bits.SAMC=6; 			// Auto Sample Time (in TAD units)
	AD1CON3bits.ADCS=12;			// System clock divider TAD=(ADCS+1)*TCY=25*x=325ns 
#endif

#if defined M3_HB2_H2R1_J0J1 || defined M3_HB2_H2R1_J2J3J4 || defined M3_HB2_H2R2_J0J1 || defined M3_HB2_H2R2_J2J3J4 \
	|| defined M3_HB2_H2R3_J0J1 || defined M3_HB2_H2R3_J2J3J4
	//TAD=50ns. Sample time = 3TAD. Conversion time 14TAD.
	//Sample-conversion of 6 signals takes (14+3)*6*TAD=102TAD = 5.1us = ~200 cyles/ms
	AD1CON3bits.SAMC=6; 			// Auto Sample Time (in TAD units)
	AD1CON3bits.ADCS=12;			// System clock divider TAD=(ADCS+1)*TCY=25*x=325ns 
#endif

#ifdef M3_HEX2_S1R1
	//TAD=50ns. Sample time = 3TAD. Conversion time 14TAD.
	//Sample-conversion of 6 signals takes (14+3)*6*TAD=102TAD = 5.1us = ~200 cyles/ms
	AD1CON3bits.SAMC=6; 			// Auto Sample Time (in TAD units)
	AD1CON3bits.ADCS=12;			// System clock divider TAD=(ADCS+1)*TCY=25*x=325ns 
#endif

#ifdef M3_GMB_G1R1
	AD1CON3bits.SAMC=6; 			// Auto Sample Time (in TAD units)
	AD1CON3bits.ADCS=12;			// System clock divider TAD=(ADCS+1)*TCY=25*x
#endif

#if defined M3_LEDX2_S1R1 || defined M3_LEDX2XN_S2R1
	AD1CON3bits.SAMC=6; 			// Auto Sample Time (in TAD units)
	AD1CON3bits.ADCS=12;			// System clock divider TAD=(ADCS+1)*TCY=25*x
#endif

#if defined M3_LOADX6_A2R1 || defined M3_LOADX6_A2R2 
	AD1CON3bits.SAMC=6; 			// FAST: Auto Sample Time = 3*Tad	
	AD1CON3bits.ADCS=12;			// FAST: System clock divider TAD=(ADCS+1)*TCY=5*=50ns (As fast as works...)
#endif

#if defined M3_LOADX6_A2R3	
	AD1CON3bits.SAMC=10; 			// FAST: Auto Sample Time = 3*Tad	
	AD1CON3bits.ADCS=10;			// FAST: System clock divider TAD=(ADCS+1)*TCY=5*=50ns (As fast as works...)
#endif

#ifdef M3_DEV
	AD1CON3bits.SAMC=6; 			// FAST: Auto Sample Time = 3*Tad	
	AD1CON3bits.ADCS=12;			// FAST: System clock divider TAD=(ADCS+1)*TCY=5*=50ns (As fast as works...)
#endif

#if defined M3_PWR_0_2 || defined M3_PWR_0_3 || defined M3_PWR_0_4 || defined M3_PWR_0_5
	AD1CON3bits.SAMC=10; 			// SLOW: Auto Sample Time = 3*Tad		
	AD1CON3bits.ADCS=40;			// SLOW: System clock divider TAD=(ADCS+1)*TCY=5*=50ns (As fast as works...)
#endif

	AD1CON2bits.CHPS  = 0;			// Only convert CH0	in 12-bit mode
	AD1CON1bits.ASAM   = 1;			// Sampling begins immediately after conversion is done
	AD1CON1bits.AD12B  = 1;			// 12-bit ADC operation
	AD1CON1bits.SIMSAM = 0;			// No simultaneous sample for 1CH
	AD1CON2bits.CSCNA=1;			// Enable channel scanning
	AD1CON2bits.SMPI=ADC_NUM_CH-1;	// Select number of conversions between interrupts 

#if defined M3_BMA_A1R1 || defined M3_WMA_0_1
	//AD1CON1bits.SSRC = 0b011;		// Sync StartOfConversion with PWM1
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=1;				// Use 2x8-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;		
	AD1CSSLbits.CSS4=1;		
	AD1CSSLbits.CSS5=1;	
#endif


#if defined M3_MAX2_BDC_A2R1 || defined M3_MAX2_BLDC_A2R1 || defined M3_MAX2_BDC_T2R1 || defined M3_MAX2_BLDC_T2R1 || defined M3_BMW_A2R1
	// AD1PCFGL: ADC1 PORT CONFIGURATION REGISTER LOW
    // AD1PCFGLBITS.PCFGx=0 defines the Pin as an Analog input.
    // AD1PCFGLBITS.PCFGx=1 defines the Pin as Digital.
	AD1PCFGLbits.PCFG0=0;
    AD1PCFGLbits.PCFG1=0;
    AD1PCFGLbits.PCFG2=0;
    AD1PCFGLbits.PCFG3=0;
    AD1PCFGLbits.PCFG4=0;
    AD1PCFGLbits.PCFG5=1;
    AD1PCFGLbits.PCFG6=1;
    AD1PCFGLbits.PCFG7=1;
    AD1PCFGLbits.PCFG8=1;

	AD1CON1bits.SSRC = 0b011;		// Sync StartOfConversion with PWM1
	//AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=1;				// Use 2x8-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;		
	AD1CSSLbits.CSS4=1;		
#endif

#if defined M3_MAX2_BDC_A2R2 || defined M3_MAX2_BLDC_A2R2 || defined  M3_MAX2_BDC_S1R1 \
	|| defined  M3_MAX2_BDC_ARMH || defined M3_MAX2_BDC_A2R3 || defined M3_MAX2_BLDC_A2R3
	// AD1PCFGL: ADC1 PORT CONFIGURATION REGISTER LOW
    // AD1PCFGLBITS.PCFGx=0 defines the Pin as an Analog input.
    // AD1PCFGLBITS.PCFGx=1 defines the Pin as Digital.
	AD1PCFGLbits.PCFG0=0;
    AD1PCFGLbits.PCFG1=0;
    AD1PCFGLbits.PCFG2=0;
    AD1PCFGLbits.PCFG3=0;
    AD1PCFGLbits.PCFG4=0;
    AD1PCFGLbits.PCFG5=1;
    AD1PCFGLbits.PCFG6=1;
    AD1PCFGLbits.PCFG7=1;
    AD1PCFGLbits.PCFG8=1;

	//AD1CON1bits.SSRC = 0b011;		// Sync StartOfConversion with PWM1
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=1;				// Use 2x8-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;			
#endif

#if defined M3_MAX2_BLDC_T2R2 || defined M3_MAX2_BDC_T2R2 || defined M3_MAX2_BLDC_T2R3
	// AD1PCFGL: ADC1 PORT CONFIGURATION REGISTER LOW
    // AD1PCFGLBITS.PCFGx=0 defines the Pin as an Analog input.
    // AD1PCFGLBITS.PCFGx=1 defines the Pin as Digital.
	AD1PCFGLbits.PCFG0=0;
    AD1PCFGLbits.PCFG1=0;
    AD1PCFGLbits.PCFG2=0;
    AD1PCFGLbits.PCFG3=0;
    AD1PCFGLbits.PCFG4=0;
    AD1PCFGLbits.PCFG5=1;
    AD1PCFGLbits.PCFG6=0;
    AD1PCFGLbits.PCFG7=1;
    AD1PCFGLbits.PCFG8=1;
#if defined M3_MAX2_BLDC_T2R2 || defined M3_MAX2_BLDC_T2R3
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
#else
	AD1CON1bits.SSRC = 0b011;		// Sync StartOfConversion with PWM1
#endif
	AD1CON2bits.BUFM=1;				// Use 2x8-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;		
	//AD1CSSLbits.CSS4=1;	
	//AD1CSSLbits.CSS5=1;	
	//AD1CSSLbits.CSS6=1;	
#endif

#if defined M3_MAX2_BDC_T2R3
	// AD1PCFGL: ADC1 PORT CONFIGURATION REGISTER LOW
    // AD1PCFGLBITS.PCFGx=0 defines the Pin as an Analog input.
    // AD1PCFGLBITS.PCFGx=1 defines the Pin as Digital.
	AD1PCFGLbits.PCFG0=0;
    AD1PCFGLbits.PCFG1=0;
    AD1PCFGLbits.PCFG2=0;
    AD1PCFGLbits.PCFG3=1;
    AD1PCFGLbits.PCFG4=1;
    AD1PCFGLbits.PCFG5=1;
    AD1PCFGLbits.PCFG6=0;
    AD1PCFGLbits.PCFG7=1;
    AD1PCFGLbits.PCFG8=1;
	AD1CON1bits.SSRC = 0b011;		// Sync StartOfConversion with PWM1
	AD1CON2bits.BUFM=1;				// Use 2x8-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	//AD1CSSLbits.CSS3=1;		
	//AD1CSSLbits.CSS4=1;	
	//AD1CSSLbits.CSS5=1;	
	//AD1CSSLbits.CSS6=1;	
#endif

#if defined M3_DAC_0_1 || defined M3_ELMO_RNA_R0 
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=1;				// Use 2x8-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
#endif

#if defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
	// AD1PCFGL: ADC1 PORT CONFIGURATION REGISTER LOW
    // AD1PCFGLBITS.PCFGx=0 defines the Pin as an Analog input.
    // AD1PCFGLBITS.PCFGx=1 defines the Pin as Digital.
	AD1PCFGLbits.PCFG0=0;
    AD1PCFGLbits.PCFG1=0;
    AD1PCFGLbits.PCFG2=0;
    AD1PCFGLbits.PCFG3=0;
    AD1PCFGLbits.PCFG4=0;
    AD1PCFGLbits.PCFG5=1;
    AD1PCFGLbits.PCFG6=1;
    AD1PCFGLbits.PCFG7=1;// Use AN7 as a digital input for M3_ZTB board (Z-Axis trigger board)
    AD1PCFGLbits.PCFG8=1;// Use AN8 for brake
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=1;				// Use 2x8-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
#endif

#if defined M3_PWR_0_2 || defined M3_PWR_0_3 || defined M3_PWR_0_4 || defined M3_PWR_0_5
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=1;				// Use 2x8-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;			
#endif

#if defined M3_LEDX2_S1R1 || defined M3_LEDX2XN_S2R1
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequencesAD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;	
#endif

#if defined M3_LOADX6_A2R2 || defined M3_LOADX6_A2R3	
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequences
	//AD1CON2bits.BUFM=1;				// Use 2x8-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;		
	AD1CSSLbits.CSS4=1;	
	AD1CSSLbits.CSS5=1;		
	AD1CSSLbits.CSS6=1;	
	AD1CSSLbits.CSS7=1;	
#endif

#if defined M3_LOADX6_A2R1
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequences
	//AD1CON2bits.BUFM=1;				// Use 2x8-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;		
	AD1CSSLbits.CSS4=1;	
	AD1CSSLbits.CSS5=1;
	AD1CSSLbits.CSS6=1;				
	AD1CSSLbits.CSS7=1;		
	AD1CSSLbits.CSS8=1;		
#endif


#ifdef M3_DEV
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequences
	//AD1CON2bits.BUFM=1;				// Use 2x8-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;		
	AD1CSSLbits.CSS4=1;	
	AD1CSSLbits.CSS5=1;
	AD1CSSLbits.CSS6=1;				
	AD1CSSLbits.CSS7=1;		
	AD1CSSLbits.CSS8=1;		
#endif

#ifdef M3_HMB_H1R1
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;		
	AD1CSSLbits.CSS4=1;	
	AD1CSSLbits.CSS5=1;
	AD1CSSLbits.CSS6=1;				
	AD1CSSLbits.CSS7=1;		
	AD1CSSLbits.CSS8=1;		
#endif

#if defined M3_HB2_H2R1_J0J1 || defined M3_HB2_H2R1_J2J3J4
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;		
	AD1CSSLbits.CSS4=1;	
	AD1CSSLbits.CSS5=1;
	AD1CSSLbits.CSS6=1;				
	AD1CSSLbits.CSS7=1;		
#endif

#if defined M3_HB2_H2R2_J0J1 || defined M3_HB2_H2R3_J0J1 
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;		
	AD1CSSLbits.CSS4=1;	
	AD1CSSLbits.CSS5=1;	
#endif

#if defined M3_HB2_H2R2_J2J3J4 || defined M3_HB2_H2R3_J2J3J4
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;		
	AD1CSSLbits.CSS4=1;	
	AD1CSSLbits.CSS5=1;
	AD1CSSLbits.CSS6=1;				
#endif

#ifdef M3_HEX2_S1R1
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;		
	AD1CSSLbits.CSS4=1;	
	AD1CSSLbits.CSS5=1;
	AD1CSSLbits.CSS6=1;				
	AD1CSSLbits.CSS7=1;		
	AD1CSSLbits.CSS8=1;		
#endif

#ifdef M3_GMB_G1R1
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;				
	AD1CSSLbits.CSS2=1;		
	AD1CSSLbits.CSS3=1;		
	AD1CSSLbits.CSS4=1;	
	AD1CSSLbits.CSS5=1;
	AD1CSSLbits.CSS6=1;				
	AD1CSSLbits.CSS7=1;			
	AD1CSSLbits.CSS8=1;		
#endif

#if defined M3_BMW_A2R2 || defined M3_BMW_A2R3
	// AD1PCFGL: ADC1 PORT CONFIGURATION REGISTER LOW
    // AD1PCFGLBITS.PCFGx=0 defines the Pin as an Analog input.
    // AD1PCFGLBITS.PCFGx=1 defines the Pin as Digital.
	AD1PCFGLbits.PCFG0=0;
    AD1PCFGLbits.PCFG1=0;
    AD1PCFGLbits.PCFG2=0;
    AD1PCFGLbits.PCFG3=0;
    AD1PCFGLbits.PCFG4=0;
    AD1PCFGLbits.PCFG5=1;
    AD1PCFGLbits.PCFG6=1;
    AD1PCFGLbits.PCFG7=1;
    AD1PCFGLbits.PCFG8=1;
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequences
	AD1CSSLbits.CSS0=1;
	AD1CSSLbits.CSS1=1;					
#endif

#ifdef M3_HEX4_S2R1
	// AD1PCFGL: ADC1 PORT CONFIGURATION REGISTER LOW
    // AD1PCFGLBITS.PCFGx=0 defines the Pin as an Analog input.
    // AD1PCFGLBITS.PCFGx=1 defines the Pin as Digital.
	AD1PCFGLbits.PCFG0=0;
    AD1PCFGLbits.PCFG1=0;
    AD1PCFGLbits.PCFG2=0;
    AD1PCFGLbits.PCFG3=1;
    AD1PCFGLbits.PCFG4=1;
    AD1PCFGLbits.PCFG5=1;
    AD1PCFGLbits.PCFG6=1;
    AD1PCFGLbits.PCFG7=1;
    AD1PCFGLbits.PCFG8=1;
	AD1CON1bits.SSRC = 0b111;		// Auto StartOfConversion
	AD1CON2bits.BUFM=0;				// Use 1x16-word buffer for conversion sequences
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

#if defined M3_LOADX6_A2R2 || defined M3_LOADX6_A2R3
#ifdef USE_TIMESTAMP_DC
	SetTimestampLatch;
#endif
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
		adc_raw[4]=ADC1BUF4;
		adc_raw[5]=ADC1BUF5;
		adc_raw[6]=ADC1BUF6;
		adc_raw[7]=ADC1BUF7;
#ifdef USE_TIMESTAMP_DC
	ClrTimestampLatch;
#endif
#endif

#if defined M3_LOADX6_A2R1
#ifdef USE_TIMESTAMP_DC
	SetTimestampLatch;
#endif
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
		adc_raw[4]=ADC1BUF4;
		adc_raw[5]=ADC1BUF5;
		adc_raw[6]=ADC1BUF6;
		adc_raw[7]=ADC1BUF7;
		adc_raw[8]=ADC1BUF8;
#ifdef USE_TIMESTAMP_DC
	ClrTimestampLatch;
#endif
#endif

#if defined M3_BMW_A2R2 || defined M3_BMW_A2R3
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
#endif

#ifdef M3_HEX4_S2R1
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
#endif

#if defined M3_LEDX2_S1R1 || defined M3_LEDX2XN_S2R1
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
#endif

#ifdef M3_DEV
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
		adc_raw[4]=ADC1BUF4;
		adc_raw[5]=ADC1BUF5;
		adc_raw[6]=ADC1BUF6;
		adc_raw[7]=ADC1BUF7;
		adc_raw[8]=ADC1BUF8;
#endif

#ifdef M3_HMB_H1R1
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
		adc_raw[4]=ADC1BUF4;
		adc_raw[5]=ADC1BUF5;
		adc_raw[6]=ADC1BUF6;
		adc_raw[7]=ADC1BUF7;
		adc_raw[8]=ADC1BUF8;
#endif
#if defined M3_HB2_H2R1_J0J1 || defined M3_HB2_H2R1_J2J3J4
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
		adc_raw[4]=ADC1BUF4;
		adc_raw[5]=ADC1BUF5;
		adc_raw[6]=ADC1BUF6;
		adc_raw[7]=ADC1BUF7;
#endif
#if defined M3_HB2_H2R2_J0J1  || defined M3_HB2_H2R3_J0J1 
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
		adc_raw[4]=ADC1BUF4;
		adc_raw[5]=ADC1BUF5;
#endif
#if defined M3_HB2_H2R2_J2J3J4 || defined M3_HB2_H2R3_J2J3J4
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
		adc_raw[4]=ADC1BUF4;
		adc_raw[5]=ADC1BUF5;
		adc_raw[6]=ADC1BUF6;
#endif
#ifdef M3_HEX2_S1R1
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
		adc_raw[4]=ADC1BUF4;
		adc_raw[5]=ADC1BUF5;
		adc_raw[6]=ADC1BUF6;
		adc_raw[7]=ADC1BUF7;
		adc_raw[8]=ADC1BUF8;
#endif
#ifdef M3_GMB_G1R1
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
		adc_raw[4]=ADC1BUF4;
		adc_raw[5]=ADC1BUF5;
		adc_raw[6]=ADC1BUF6;
		adc_raw[7]=ADC1BUF7;
		adc_raw[8]=ADC1BUF8;
#endif

#if defined M3_PWR_0_2 || defined M3_PWR_0_3 || defined M3_PWR_0_4 || defined M3_DAC_0_1 || defined M3_ELMO_RNA_R0 || defined M3_PWR_0_5
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
#if defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
	if (AD1CON2bits.BUFS==0) //ADC module filling lower group, read from upper
	{
		adc_raw[0]=ADC1BUF8;
		adc_raw[1]=ADC1BUF9;
	}
	else
	{
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
	}
#endif
#if defined M3_BMA_A1R1 || defined M3_WMA_0_1
	if (AD1CON2bits.BUFS==0) //ADC module filling lower group, read from upper
	{
		adc_raw[0]=ADC1BUF8;
		adc_raw[1]=ADC1BUF9;
		adc_raw[2]=ADC1BUFA;
		adc_raw[3]=ADC1BUFB;
		adc_raw[4]=ADC1BUFC;
		adc_raw[5]=ADC1BUFD;
	}
	else
	{
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
		adc_raw[4]=ADC1BUF4;
		adc_raw[5]=ADC1BUF5;
	}
#endif

#if defined M3_MAX2_BDC_A2R1 || defined M3_MAX2_BLDC_A2R1 || defined M3_MAX2_BDC_T2R1 || defined M3_MAX2_BLDC_T2R1 || defined M3_BMW_A2R1 
	if (AD1CON2bits.BUFS==0) //ADC module filling lower group, read from upper
	{
		adc_raw[0]=ADC1BUF8;
		adc_raw[1]=ADC1BUF9;
		adc_raw[2]=ADC1BUFA;
		adc_raw[3]=ADC1BUFB;
		adc_raw[4]=ADC1BUFC;
	}
	else
	{
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
		adc_raw[4]=ADC1BUF4;
	}
#endif

#if defined M3_MAX2_BDC_A2R2 || defined M3_MAX2_BLDC_A2R2 || defined  M3_MAX2_BDC_S1R1 \
	|| defined  M3_MAX2_BDC_ARMH || defined M3_MAX2_BDC_A2R3 || defined M3_MAX2_BLDC_A2R3
	if (AD1CON2bits.BUFS==0) //ADC module filling lower group, read from upper
	{
		adc_raw[0]=ADC1BUF8;
		adc_raw[1]=ADC1BUF9;
		adc_raw[2]=ADC1BUFA;
		adc_raw[3]=ADC1BUFB;
	}
	else
	{
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
	}
#endif

#if defined M3_MAX2_BLDC_T2R2 || defined M3_MAX2_BDC_T2R2 || defined M3_MAX2_BLDC_T2R3
	if (AD1CON2bits.BUFS==0) //ADC module filling lower group, read from upper
	{
		adc_raw[0]=ADC1BUF8;
		adc_raw[1]=ADC1BUF9;
		adc_raw[2]=ADC1BUFA;
		adc_raw[3]=ADC1BUFB;
		//adc_raw[4]=ADC1BUFC;
		//adc_raw[5]=ADC1BUFD;
		//adc_raw[6]=ADC1BUFE;
	}
	else
	{
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
		//adc_raw[4]=ADC1BUF4;
		//adc_raw[5]=ADC1BUF5;
		//adc_raw[6]=ADC1BUF6;
	}
#endif
#if defined M3_MAX2_BDC_T2R3 
	if (AD1CON2bits.BUFS==0) //ADC module filling lower group, read from upper
	{
		adc_raw[0]=ADC1BUF8;
		adc_raw[1]=ADC1BUF9;
		adc_raw[2]=ADC1BUFA;
		//adc_raw[3]=ADC1BUFB;
		//adc_raw[4]=ADC1BUFC;
		//adc_raw[5]=ADC1BUFD;
		//adc_raw[6]=ADC1BUFE;
	}
	else
	{
		adc_raw[0]=ADC1BUF0;
		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		//adc_raw[3]=ADC1BUF3;
		//adc_raw[4]=ADC1BUF4;
		//adc_raw[5]=ADC1BUF5;
		//adc_raw[6]=ADC1BUF6;
	}
#endif
#if defined M3_DAC_0_1 
		adc_buffer[ADC_NC][adc_idx]=0;//adc_raw[ADC_NC]; //Not connected, read it anyhow to simplify code.
		adc_buffer[ADC_MOTOR_TEMP][adc_idx]=adc_raw[ADC_MOTOR_TEMP];
		adc_buffer[ADC_EXT][adc_idx]=adc_raw[ADC_EXT];
		adc_buffer_fast[adc_idx_fast]=adc_raw[ADC_EXT]; //Used EXT for analog torque feedback
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
		adc_idx_fast=INC_MOD(adc_idx_fast,ADC_NUM_SMOOTH_FAST);
#endif

#if defined M3_ELMO_RNA_R0 
		adc_buffer[ADC_SEAS][adc_idx]=adc_raw[ADC_SEAS]; 
		adc_buffer[ADC_MOTOR_TEMP][adc_idx]=adc_raw[ADC_MOTOR_TEMP];
		adc_buffer[ADC_EXT][adc_idx]=adc_raw[ADC_EXT];
		adc_buffer_fast[adc_idx_fast]=adc_raw[ADC_SEAS]; //Used ADC_SEAS for analog torque feedback
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
		adc_idx_fast=INC_MOD(adc_idx_fast,ADC_NUM_SMOOTH_FAST);
#endif

#if defined M3_ELMO_B1R1 || defined M3_ELMO_Z1R1
		adc_buffer[ADC_MOTOR_TEMP][adc_idx]=adc_raw[ADC_MOTOR_TEMP];
		adc_buffer[ADC_EXT][adc_idx]=adc_raw[ADC_EXT];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif

#if defined M3_BMA_A1R1 || defined M3_WMA_0_1
		adc_buffer[ADC_SEAS][adc_idx]=adc_raw[ADC_SEAS];
		adc_buffer_fast[adc_idx_fast]=adc_raw[ADC_SEAS];
		adc_buffer[ADC_MOTOR_TEMP][adc_idx]=adc_raw[ADC_MOTOR_TEMP];
		adc_buffer[ADC_EXT][adc_idx]=adc_raw[ADC_EXT];
		adc_buffer[ADC_AMP_TEMP][adc_idx]=adc_raw[ADC_AMP_TEMP];
		adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
		adc_buffer[ADC_CURRENT_B][adc_idx]=adc_raw[ADC_CURRENT_B];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
		adc_idx_fast=INC_MOD(adc_idx_fast,ADC_NUM_SMOOTH_FAST);
#endif

#if defined M3_MAX2_BDC_A2R1 || defined M3_MAX2_BLDC_A2R1 || defined M3_BMW_A2R1
		adc_buffer[ADC_MOTOR_TEMP][adc_idx]=adc_raw[ADC_MOTOR_TEMP];
		adc_buffer[ADC_EXT][adc_idx]=adc_raw[ADC_EXT];
		adc_buffer[ADC_AMP_TEMP][adc_idx]=adc_raw[ADC_AMP_TEMP];
		adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
		adc_buffer[ADC_CURRENT_B][adc_idx]=adc_raw[ADC_CURRENT_B];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif

#if defined M3_MAX2_BDC_A2R2 || defined M3_MAX2_BLDC_A2R2 || defined  M3_MAX2_BDC_S1R1 \
	|| defined  M3_MAX2_BDC_ARMH || defined M3_MAX2_BDC_A2R3 || defined M3_MAX2_BLDC_A2R3
		adc_buffer[ADC_MOTOR_TEMP][adc_idx]=adc_raw[ADC_MOTOR_TEMP];
		adc_buffer[ADC_AMP_TEMP][adc_idx]=adc_raw[ADC_AMP_TEMP];
		adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
		adc_buffer[ADC_CURRENT_B][adc_idx]=adc_raw[ADC_CURRENT_B];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif

#if defined M3_BMW_A2R2 || defined M3_BMW_A2R3
		adc_buffer[ADC_EXT_TEMP][adc_idx]=adc_raw[ADC_EXT_TEMP];
		adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif

#if defined M3_HEX4_S2R1
		adc_buffer[ADC_MOTOR_TEMP][adc_idx]=adc_raw[ADC_MOTOR_TEMP];
		adc_buffer[ADC_AMP_TEMP_A][adc_idx]=adc_raw[ADC_AMP_TEMP_A];
		adc_buffer[ADC_AMP_TEMP_B][adc_idx]=adc_raw[ADC_AMP_TEMP_B];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif

#if defined M3_MAX2_BDC_T2R1 || defined M3_MAX2_BLDC_T2R1 || defined M3_MAX2_BLDC_T2R2 || defined M3_MAX2_BDC_T2R2 
		//adc_buffer[ADC_MOTOR_TEMP][adc_idx]=adc_raw[ADC_MOTOR_TEMP];
		adc_buffer[ADC_EXT][adc_idx]=adc_raw[ADC_EXT];
		adc_buffer[ADC_AMP_TEMP][adc_idx]=adc_raw[ADC_AMP_TEMP];
		adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
		adc_buffer[ADC_CURRENT_B][adc_idx]=adc_raw[ADC_CURRENT_B];
		adc_buffer_fast[adc_idx_fast]=adc_raw[ADC_EXT];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
		adc_idx_fast=INC_MOD(adc_idx_fast,ADC_NUM_SMOOTH_FAST);
#endif
#if defined M3_MAX2_BLDC_T2R3
		adc_buffer[ADC_MOTOR_TEMP][adc_idx]=adc_raw[ADC_MOTOR_TEMP];
		adc_buffer[ADC_AMP_TEMP][adc_idx]=adc_raw[ADC_AMP_TEMP];
		adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
		adc_buffer[ADC_CURRENT_B][adc_idx]=adc_raw[ADC_CURRENT_B];
		//adc_buffer_fast[adc_idx_fast]=adc_raw[ADC_EXT];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
		adc_idx_fast=INC_MOD(adc_idx_fast,ADC_NUM_SMOOTH_FAST);
#endif
#if defined M3_MAX2_BDC_T2R3
		adc_buffer[ADC_AMP_TEMP][adc_idx]=adc_raw[ADC_AMP_TEMP];
		adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
		adc_buffer[ADC_CURRENT_B][adc_idx]=adc_raw[ADC_CURRENT_B];
		//adc_buffer_fast[adc_idx_fast]=adc_raw[ADC_EXT];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
		adc_idx_fast=INC_MOD(adc_idx_fast,ADC_NUM_SMOOTH_FAST);
#endif

#ifdef M3_HMB_H1R1
		adc_buffer[ADC_MOTOR_TEMP_A][adc_idx]=adc_raw[ADC_MOTOR_TEMP_A];
		adc_buffer[ADC_EXT_A][adc_idx]=adc_raw[ADC_EXT_A];
		adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
		adc_buffer[ADC_MOTOR_TEMP_B][adc_idx]=adc_raw[ADC_MOTOR_TEMP_B];
		adc_buffer[ADC_EXT_B][adc_idx]=adc_raw[ADC_EXT_B];
		adc_buffer[ADC_CURRENT_B][adc_idx]=adc_raw[ADC_CURRENT_B];
		adc_buffer[ADC_AMP_TEMP][adc_idx]=adc_raw[ADC_AMP_TEMP];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif
#if defined M3_HB2_H2R1_J0J1 
		adc_buffer[ADC_MOTOR_TEMP_A][adc_idx]=adc_raw[ADC_MOTOR_TEMP_A];
		adc_buffer[ADC_MOTOR_TEMP_B][adc_idx]=adc_raw[ADC_MOTOR_TEMP_B];
		adc_buffer[ADC_SEAS_A][adc_idx]=adc_raw[ADC_SEAS_A];
		adc_buffer[ADC_SEAS_B][adc_idx]=adc_raw[ADC_SEAS_B];
		adc_buffer[ADC_AMP_TEMP_A][adc_idx]=adc_raw[ADC_AMP_TEMP_A];
		adc_buffer[ADC_AMP_TEMP_B][adc_idx]=adc_raw[ADC_AMP_TEMP_B];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif
#if defined M3_HB2_H2R2_J0J1  || defined M3_HB2_H2R3_J0J1 
		adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
		adc_buffer[ADC_CURRENT_B][adc_idx]=adc_raw[ADC_CURRENT_B];
		adc_buffer[ADC_SEAS_A][adc_idx]=adc_raw[ADC_SEAS_A];
		adc_buffer[ADC_SEAS_B][adc_idx]=adc_raw[ADC_SEAS_B];
		adc_buffer[ADC_AMP_TEMP_A][adc_idx]=adc_raw[ADC_AMP_TEMP_A];
		adc_buffer[ADC_AMP_TEMP_B][adc_idx]=adc_raw[ADC_AMP_TEMP_B];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif
#if defined M3_HB2_H2R1_J2J3J4 
		adc_buffer[ADC_MOTOR_TEMP_A][adc_idx]=adc_raw[ADC_MOTOR_TEMP_A];
		adc_buffer[ADC_MOTOR_TEMP_B][adc_idx]=adc_raw[ADC_MOTOR_TEMP_B];
		adc_buffer[ADC_MOTOR_TEMP_C][adc_idx]=adc_raw[ADC_MOTOR_TEMP_C];
		adc_buffer[ADC_SEAS_A][adc_idx]=adc_raw[ADC_SEAS_A];
		adc_buffer[ADC_SEAS_B][adc_idx]=adc_raw[ADC_SEAS_B];
		adc_buffer[ADC_SEAS_C][adc_idx]=adc_raw[ADC_SEAS_C];
		adc_buffer[ADC_AMP_TEMP_A][adc_idx]=adc_raw[ADC_AMP_TEMP_A];
		adc_buffer[ADC_AMP_TEMP_B][adc_idx]=adc_raw[ADC_AMP_TEMP_B];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif
#if defined M3_HB2_H2R2_J2J3J4  || defined M3_HB2_H2R3_J2J3J4
		adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
		adc_buffer[ADC_CURRENT_B][adc_idx]=adc_raw[ADC_CURRENT_B];
		adc_buffer[ADC_CURRENT_C][adc_idx]=adc_raw[ADC_CURRENT_C];
		adc_buffer[ADC_SEAS_A][adc_idx]=adc_raw[ADC_SEAS_A];
		adc_buffer[ADC_SEAS_B][adc_idx]=adc_raw[ADC_SEAS_B];
		adc_buffer[ADC_SEAS_C][adc_idx]=adc_raw[ADC_SEAS_C];
		adc_buffer[ADC_AMP_TEMP_A][adc_idx]=adc_raw[ADC_AMP_TEMP_A]; //B=C=A
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif
#ifdef M3_HEX2_S1R1
		adc_buffer[ADC_MOTOR_TEMP_A][adc_idx]=adc_raw[ADC_MOTOR_TEMP_A];
		adc_buffer[ADC_MOTOR_TEMP_B][adc_idx]=adc_raw[ADC_MOTOR_TEMP_B];
		adc_buffer[ADC_EXT_A][adc_idx]=adc_raw[ADC_EXT_A];
		adc_buffer[ADC_EXT_B][adc_idx]=adc_raw[ADC_EXT_B];
		adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
		adc_buffer[ADC_CURRENT_B][adc_idx]=adc_raw[ADC_CURRENT_B];
		adc_buffer[ADC_AMP_TEMP][adc_idx]=adc_raw[ADC_AMP_TEMP];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif
#ifdef M3_GMB_G1R1
		adc_buffer[ADC_SEAS_A][adc_idx]=adc_raw[ADC_SEAS_A];
		adc_buffer[ADC_SEAS_B][adc_idx]=adc_raw[ADC_SEAS_B];
		adc_buffer[ADC_MOTOR_TEMP_A][adc_idx]=adc_raw[ADC_MOTOR_TEMP_A];
		adc_buffer[ADC_MOTOR_TEMP_B][adc_idx]=adc_raw[ADC_MOTOR_TEMP_B];
		adc_buffer[ADC_AMP_TEMP][adc_idx]=adc_raw[ADC_AMP_TEMP];
		adc_buffer[ADC_EXT_A][adc_idx]=adc_raw[ADC_EXT_A];
		adc_buffer[ADC_EXT_B][adc_idx]=adc_raw[ADC_EXT_B];
		adc_buffer[ADC_EXT_C][adc_idx]=adc_raw[ADC_EXT_C];
		adc_buffer[ADC_EXT_D][adc_idx]=adc_raw[ADC_EXT_D];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif

#if defined M3_LEDX2_S1R1 || defined M3_LEDX2XN_S2R1
		adc_buffer[ADC_EXT_A][adc_idx]=adc_raw[ADC_EXT_A];
		adc_buffer[ADC_EXT_B][adc_idx]=adc_raw[ADC_EXT_B];
		adc_buffer[ADC_EXT_C][adc_idx]=adc_raw[ADC_EXT_C];
		adc_buffer[ADC_EXT_D][adc_idx]=adc_raw[ADC_EXT_D];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif

#if defined M3_PWR_0_2 || defined M3_PWR_0_3 || defined M3_PWR_0_4 || defined M3_PWR_0_5
		adc_buffer[ADC_BUS_VOLTAGE][adc_idx]=adc_raw[ADC_BUS_VOLTAGE];
		adc_buffer[ADC_CURRENT_DIGITAL][adc_idx]=adc_raw[ADC_CURRENT_DIGITAL];
		adc_buffer[ADC_EXT][adc_idx]=adc_raw[ADC_EXT];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif


#if defined M3_LOADX6_A2R2  || defined M3_LOADX6_A2R3
		adc_buffer[ADC_LOAD_0][adc_idx]=adc_raw[ADC_LOAD_0];
		adc_buffer[ADC_LOAD_1][adc_idx]=adc_raw[ADC_LOAD_1];
		adc_buffer[ADC_LOAD_2][adc_idx]=adc_raw[ADC_LOAD_2];
		adc_buffer[ADC_LOAD_3][adc_idx]=adc_raw[ADC_LOAD_3];
		adc_buffer[ADC_LOAD_4][adc_idx]=adc_raw[ADC_LOAD_4];
		adc_buffer[ADC_LOAD_5][adc_idx]=adc_raw[ADC_LOAD_5];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif

#if defined M3_LOADX6_A2R1
		adc_buffer[ADC_EXT_0][adc_idx]=adc_raw[ADC_EXT_0];
		adc_buffer[ADC_EXT_1][adc_idx]=adc_raw[ADC_EXT_1];
		adc_buffer[ADC_EXT_2][adc_idx]=adc_raw[ADC_EXT_2];
		adc_buffer[ADC_LOAD_0][adc_idx]=adc_raw[ADC_LOAD_0];
		adc_buffer[ADC_LOAD_1][adc_idx]=adc_raw[ADC_LOAD_1];
		adc_buffer[ADC_LOAD_2][adc_idx]=adc_raw[ADC_LOAD_2];
		adc_buffer[ADC_LOAD_3][adc_idx]=adc_raw[ADC_LOAD_3];
		adc_buffer[ADC_LOAD_4][adc_idx]=adc_raw[ADC_LOAD_4];
		adc_buffer[ADC_LOAD_5][adc_idx]=adc_raw[ADC_LOAD_5];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
#endif

}

#endif
