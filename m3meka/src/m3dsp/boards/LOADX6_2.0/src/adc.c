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

//unsigned int volatile adc_buffer[ADC_NUM_CH][ADC_NUM_SMOOTH];


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


#if 0
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

#if defined M3_LOADX6_A2R3	
	AD1CON3bits.SAMC=10; 			// FAST: Auto Sample Time = 3*Tad	
	AD1CON3bits.ADCS=10;			// FAST: System clock divider TAD=(ADCS+1)*TCY=5*=50ns (As fast as works...)
#endif

	AD1CON2bits.CHPS  = 0;			// Only convert CH0	in 12-bit mode
	AD1CON1bits.ASAM   = 1;			// Sampling begins immediately after conversion is done
	AD1CON1bits.AD12B  = 1;			// 12-bit ADC operation
	AD1CON1bits.SIMSAM = 0;			// No simultaneous sample for 1CH
	AD1CON2bits.CSCNA=1;			// Enable channel scanning
	AD1CON2bits.SMPI=ADC_NUM_CH-1;	// Select number of conversions between interrupts 

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


	//Select results format
	AD1CON1bits.FORM   = 0;		// Integer Output Format (0B 0000 dddd dddd dddd )

        setup_dma2();


	//Turn on ADC
	AD1CON1bits.ADON = 1;		// Turn on the A/D converter	
	//Enable interrupt
    _AD1IF = 0;
	//_AD1IE = 1;
}
#endif


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
	//AD1CON3bits.ADCS=40;			// SLOW: System clock divider TAD=(ADCS+1)*TCY=5*=50ns (As fast as works...)
        AD1CON3bits.ADCS=10;			// SLOW: System clock divider TAD=(ADCS+1)*TCY=5*=50ns (As fast as works...)
								// ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us

	AD1CON1bits.ADDMABM = 0; 	// DMA buffers are built in scatter/gather mode
	AD1CON2bits.SMPI    = (ADC_NUM_CH-1);	// 4 ADC Channel is scanned
	AD1CON4bits.DMABL   = 3;	// Each buffer contains 8 words

	//AD1CSSH/AD1CSSL: A/D Input Scan Selection Register

	AD1CSSLbits.CSS0=1;			// Enable AN4 for channel scan
	AD1CSSLbits.CSS1=1;			// Enable AN5 for channel scan
	AD1CSSLbits.CSS2=1;		// Enable AN10 for channel scan
	AD1CSSLbits.CSS3=1;			// Enable AN4 for channel scan
	AD1CSSLbits.CSS4=1;			// Enable AN5 for channel scan
	AD1CSSLbits.CSS5=1;		// Enable AN10 for channel scan
	AD1CSSLbits.CSS6=1;			// Enable AN4 for channel scan
	AD1CSSLbits.CSS7=1;			// Enable AN5 for channel scan
	AD1CSSLbits.CSS8=1;		// Enable AN10 for channel scan


 	//AD1PCFGH/AD1PCFGL: Port Configuration Register
	AD1PCFGL=0xFFFF;

	AD1PCFGLbits.PCFG0 = 0;		// AN0 as Analog Input
	AD1PCFGLbits.PCFG1 = 0;		// AN1 as Analog Input
        AD1PCFGLbits.PCFG2 = 0;		// AN2 as Analog Input
        AD1PCFGLbits.PCFG3 = 0;		// AN0 as Analog Input
	AD1PCFGLbits.PCFG4 = 0;		// AN1 as Analog Input
        AD1PCFGLbits.PCFG5 = 0;		// AN2 as Analog Input
        AD1PCFGLbits.PCFG6 = 0;		// AN0 as Analog Input
	AD1PCFGLbits.PCFG7 = 0;		// AN1 as Analog Input
        AD1PCFGLbits.PCFG8 = 0;		// AN2 as Analog Input


	IFS0bits.AD1IF   = 0;		// Clear the A/D interrupt flag bit
	IEC0bits.AD1IE   = 0;		// Do Not Enable A/D interrupt

        initDma2();

	AD1CON1bits.ADON = 1;		// Turn on the A/D converter


}

/*
void setup_dma2(void)
{
	DMA2CONbits.SIZE = 0;					//Word
	DMA2CONbits.DIR = 0;					//Read from Peripheral address, write to DPSRAM address
	DMA2CONbits.HALF = 0;					//Initiate interrupt when all of the data has been moved
	DMA2CONbits.NULLW = 0;					//Normal operation
	DMA2CONbits.AMODE = 2;					// Configure DMA for Peripheral indirect mode
	DMA2CONbits.MODE = 2;					// Configure DMA for Continuous Ping-Pong mode

	DMA2PAD=(int)&ADC1BUF0;

	DMA2REQbits.FORCE = 0;					//Automatic DMA transfer initiation by DMA Request
	DMA2REQbits.IRQSEL = 0b0001101; 		//ADC1 ? ADC1 Convert done

	DMA2STA = __builtin_dmaoffset(BufferA);	//Primary DPSRAM Start Address Offset bits (source or destination)
	DMA2STB = __builtin_dmaoffset(BufferB);	//Secondary DPSRAM Start Address Offset bits (source or destination)

	DMA2CNT = DMA_BUF_DEPTH - 1;			//DMA Transfer Count Register bits

	//Interrupts
	IFS1bits.DMA2IF = 0;					//Clear the DMA interrupt flag bit

//XXX
	IEC1bits.DMA2IE = 1;					//Set the DMA interrupt enable bit

	DMA2CONbits.CHEN = 1;					//Channel enabled
}	// end setup_dma1

unsigned int * current_dma_buf()
{
    unsigned int * dma_buf_ptr;

    if (DMACS1bits.PPST2)
        dma_buf_ptr = BufferA;
    else
        dma_buf_ptr = BufferB;
    return(dma_buf_ptr);
}

*/

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
    int j,i;

#ifdef USE_TIMESTAMP_DC
	SetTimestampLatch;
#endif

	if(DmaBuffer == 0)
	{
            for (j = 0; j < 8; j++)
            {
                for (i = 0;  i < ADC_NUM_CH; i++)
                    adc_buffer[i][(adc_idx*8)+j] = BufferA[i][j];
                /*adc_buffer[ADC_BUS_VOLTAGE][(adc_idx*8)+j] = BufferA[0][j];
		adc_buffer[ADC_CURRENT_DIGITAL][(adc_idx*8)+j] = BufferA[1][j];
		adc_buffer[ADC_EXT][(adc_idx*8)+j] = BufferA[2][j];*/

            }
	}
	else
	{
            for (j = 0; j < 8; j++)
            {
                for (i = 0;  i < ADC_NUM_CH; i++)
                    adc_buffer[i][(adc_idx*8)+j] = BufferA[i][j];
                /*adc_buffer[ADC_BUS_VOLTAGE][(adc_idx*8)+j] = BufferB[0][j];
		adc_buffer[ADC_CURRENT_DIGITAL][(adc_idx*8)+j] = BufferB[1][j];
		adc_buffer[ADC_EXT][(adc_idx*8)+j] = BufferB[2][j];*/

            }

	}

	DmaBuffer ^= 1;

        adc_idx=INC_MOD(adc_idx,4);

#ifdef USE_TIMESTAMP_DC
	ClrTimestampLatch;
#endif


	IFS1bits.DMA2IF = 0;		// Clear the DMA0 Interrupt Flag
}


void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void)
{
	_AD1IF = 0;		//Clear the flag

}




#if 0
void __attribute__((__interrupt__, no_auto_psv)) _DMA2Interrupt(void)
{
    unsigned int * dma_buf_ptr;
    int i;
//    int current_reading;
//    int hall_state;
    int tmp;
   // int pwm_current_control;

    ToggleHeartbeatLED();

#ifdef USE_TIMESTAMP_DC
	SetTimestampLatch;
#endif

    dma_buf_ptr = current_dma_buf();


    for(i=0;i<6;i++) {
        adc_raw[i] = dma_buf_ptr[i];
        

    }

    #if defined M3_LOADX6_A2R2  || defined M3_LOADX6_A2R3
		adc_buffer[ADC_LOAD_0][adc_idx]=adc_raw[ADC_LOAD_0];
		adc_buffer[ADC_LOAD_1][adc_idx]=adc_raw[ADC_LOAD_1];
		adc_buffer[ADC_LOAD_2][adc_idx]=adc_raw[ADC_LOAD_2];
		adc_buffer[ADC_LOAD_3][adc_idx]=adc_raw[ADC_LOAD_3];
		adc_buffer[ADC_LOAD_4][adc_idx]=adc_raw[ADC_LOAD_4];
		adc_buffer[ADC_LOAD_5][adc_idx]=adc_raw[ADC_LOAD_5];
		adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
    #endif

    //adc_meas[ADC_AMP_TEMP] = dma_buf_ptr[ADC_AMP_TEMP];


    /*switch (get_dsp_state()) {
        case DSP_PWM:
            set_pwm(0,pwm_desired);
            break;
        case DSP_CURRENT:
            set_pwm(0,pwm_current_control);
            break;
        default:
            set_pwm(0,0);
            break;
    }*/

#ifdef USE_TIMESTAMP_DC
	ClrTimestampLatch;
#endif

    _DMA2IF = 0;		//Clear the flag
}





void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void)
{
	_AD1IF = 0;		//Clear the flag
        ToggleHeartbeatLED();
#if defined M3_LOADX6_A2R2 || defined M3_LOADX6_A2R3
#ifdef USE_TIMESTAMP_DC
	SetTimestampLatch;
#endif
		adc_raw[0]=ADC1BUF0;
/*		adc_raw[1]=ADC1BUF1;
		adc_raw[2]=ADC1BUF2;
		adc_raw[3]=ADC1BUF3;
		adc_raw[4]=ADC1BUF4;
		adc_raw[5]=ADC1BUF5;
		adc_raw[6]=ADC1BUF6;
		adc_raw[7]=ADC1BUF7;*/
#ifdef USE_TIMESTAMP_DC
	ClrTimestampLatch;
#endif
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



}
#endif

#endif
