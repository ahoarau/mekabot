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

#ifdef USE_ADC

#include "p33Fxxxx.h"
#include "setup.h"
#include "adc.h"


int adc_idx;
int adc_idx_fast;

unsigned int adc_raw[ADC_NUM_CH];
int adc_meas[ADC_NUM_CH];
unsigned int adc_zero[ADC_NUM_CH] = {524<<ADC_Q_FORM,524<<ADC_Q_FORM,
                                     TEMP_SENSOR_ZERO, 0};

//unsigned int volatile adc_buffer[ADC_NUM_CH];
//unsigned int volatile adc_buffer_fast[ADC_NUM_SMOOTH_FAST];


int irq_cnt; 
unsigned int wd_cnt = 0, last_status = 0, watchdog_expired;	//Watchdog

//#define MEAS_A &adc_meas[0]
//#define MEAS_B &adc_meas[1]
//const int * current_sensor[8] = {MEAS_A, MEAS_A, MEAS_B, MEAS_B,
//                                         MEAS_B, MEAS_A, MEAS_A, MEAS_A};
// low side table
//const int current_signs[8] = {0,1,1,1,-1,1,-1,0};
// high side table
//const int current_signs[8] = {0,-1,-1,-1,1,-1,1,0};

// Number of locations for ADC buffer = 100 (AN0 only) x BUF_depth (100) = 100 Words
// Align the buffer to 128 words or 256 bytes. This is needed for peripheral indirect mode
//unsigned int BufferA[DMA_BUF_DEPTH] __attribute__((space(dma),aligned(256)));
//unsigned int BufferB[DMA_BUF_DEPTH] __attribute__((space(dma),aligned(256)));
/*typedef struct
{
    unsigned int ch0[8];
    unsigned int ch1[8];
    unsigned int ch2[8];
    unsigned int ch3[8];
} dma_adc_buf;*/

//typedef unsigned int dma_adc_buf[ADC_NUM_SMOOTH][4];

//dma_adc_buf BufferA  __attribute__( (space(dma),aligned(64)) );
//dma_adc_buf BufferB  __attribute__( (space(dma),aligned(64)) );

unsigned int BufferA[DMA_BUF_DEPTH] __attribute__( (space(dma),aligned(8)) );
unsigned int BufferB[DMA_BUF_DEPTH] __attribute__( (space(dma),aligned(8)) );

//dma_adc_buf BufferB __attribute__( (space(dma),aligned(64)) );

//static int16_t an_idx;
//static int16_t an[2];


unsigned int get_avg_adc(int ch)
{
//	long v;
//	int i;
//	v=0;

        return(adc_meas[ch]);
//        dma_adc_buf * buf_ptr;

	//for (i=0;i<ADC_NUM_SMOOTH;i++)
	//	v=v+adc_buffer[ch][i];



   /*     for (i=0;i<1;i++)
            if (DMACS1bits.PPST1)
                v += BufferA[1*ch+i];
            else
                v += BufferB[1*ch+i];*/
/*
        switch (ch) {
            case 0:
                return (unsigned int)(v>>ADC_SHIFT_SMOOTH);
            case 1:
                return buf_ptr->ch0[0];
            case 2:
                v=0;
                for(i=0;i<8;i++)
                    v=v+buf_ptr->ch0[i];
                return v;
            case 3:
                v = 0;
                for(i=0;i<DMA_BUF_DEPTH;i+=4)
                    ;
                return DMACS1bits.PPST1;
        }*/

       /* if (DMACS1bits.PPST1)
            return BufferA[8*ch];
        else
            return BufferB[8*ch];*/
        //return BufferB[ch];
       // return v;
}

unsigned int get_avg_adc_torque()
{
/*	int i;
	long v=0;
	for (i=0;i<ADC_NUM_SMOOTH_FAST;i++)
		v=v+adc_buffer_fast[i];
		
	return (unsigned int)(v>>ADC_SHIFT_SMOOTH_FAST); */
    return 0;
}

int get_temperature_cC(int ch)
{
    return (__builtin_mulsu(adc_meas[ch],TEMP_ADC_CC_MULT)>>TEMP_ADC_CC_SHIFT);
}

void setup_adc(void) 
{
//	adc_idx=0;
//	adc_idx_fast=0;


//        an[0] = 0;
//	an[1] = 1;
//	an_idx = 0;

	//Setup for current sensing
	// System clock divider TAD=(ADCS+1)*TCY==50ns (As fast as works...)
	// Auto Sample Time = 5*Tad, 4 conversions, (14+5)*4*TAD=68*50ns=3.8us

	AD1CON1bits.ADON = 0;			//Turn off ADC

        AD1CON1bits.ADDMABM = 0;                // DMA in order of conversion
        AD1CON1bits.AD12B = 0;			// 10-bit ADC operation
        AD1CON1bits.FORM = 0;			// Select results format Integer Output Format (0B 0000 dddd dddd dddd )
        AD1CON1bits.SSRC = 0b011;		// Manual StartOfConversion 0b000 //PWM: 0b011;
        AD1CON1bits.SIMSAM = 1;                 // Simultaneous sample
        AD1CON1bits.ASAM = 1;			// Sampling begins immediately after conversion is done

	AD1CON2bits.VCFG = 0;                   // Vref AVdd/AVss
	AD1CON2bits.CSCNA = 0;			// Disable channel scanning
        AD1CON2bits.CHPS = 2;			// Convert ch0-3
	AD1CON2bits.SMPI = ADC_NUM_SAMPLES-1;			// Select 4 conversions between interrupts
        AD1CON2bits.BUFM = 0;
        AD1CON2bits.ALTS = 0;

        AD1CON3bits.ADRC = 0;			// ADC Clock is derived from Systems Clock
	AD1CON3bits.SAMC = 5;
	AD1CON3bits.ADCS = 1;

        AD1CON4bits.DMABL	= ADC_NUM_SAMPLES-1;		// Allocates 8 words of buffer to each analog input

        AD1CHS123bits.CH123NB = 0;              // ch1-3 negative input is vrefl
        AD1CHS123bits.CH123SB = 0;              // ch1 -> an0, ch2 -> an1, ch3 -> an2
        AD1CHS123bits.CH123NA = 0;              // ch1-3 negative input is vrefl
        AD1CHS123bits.CH123SA = 0;              // ch1 -> an0, ch2 -> an1, ch3 -> an2

        AD1CHS0bits.CH0NB = 0;                  // ch0 negative input is vrefl
        AD1CHS0bits.CH0SB = ADC_EXT_TEMP;                  // ch0 -> an3
        AD1CHS0bits.CH0NA = 0;                  // ch0 negative input is vrefl
        AD1CHS0bits.CH0SA = ADC_EXT_TEMP;                  // ch0 -> an3

        ADPCFG = 0xFFFF;
        AD1PCFGLbits.PCFG0 = 0;
	AD1PCFGLbits.PCFG1 = 0;
	AD1PCFGLbits.PCFG6 = 0;
        AD1PCFGLbits.PCFG8 = 0;


        ADPCFG = 0xFFFF;
        AD1PCFGLbits.PCFG0 = 0;
	AD1PCFGLbits.PCFG1 = 0;
	AD1PCFGLbits.PCFG2 = 0;
        AD1PCFGLbits.PCFG3 = 0;


        setup_dma2();

	AD1CON1bits.ADON = 1;			// Turn on ADC
	_AD1IF = 0;						// Enable interrupt
	_AD1IE = 0;
}

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


void __attribute__((__interrupt__, no_auto_psv)) _DMA2Interrupt(void)
{
    unsigned int * dma_buf_ptr;
    int i;
//    int current_reading;
//    int hall_state;
    int tmp;
   // int pwm_current_control;


    dma_buf_ptr = current_dma_buf();

    
    for(i=0;i<3;i++) {
        adc_raw[i] = dma_buf_ptr[i];
        tmp = adc_raw[i] - (adc_zero[i]>>ADC_Q_FORM);
        adc_filter((int *)&adc_meas[i], tmp, 0x8 );

    }
    //adc_meas[ADC_AMP_TEMP] = dma_buf_ptr[ADC_AMP_TEMP];
    adc_meas[ADC_EXT_TEMP] = dma_buf_ptr[ADC_EXT_TEMP];
    set_current_ab(adc_meas[ADC_CURRENT_A],adc_meas[ADC_CURRENT_B]);
    //hall_state = get_hall_state();
    //current_reading = *current_sensor[hall_state]*current_signs[hall_state];
    set_pwm_current_desired(current_control());
    update_pwm();

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
    
    _DMA2IF = 0;		//Clear the flag
}



void adc_filter(int *y, int x, int alpha)
{
    // y and alpha are 10Q6, x is 10Q0
    long tmp;
    tmp = __builtin_mulsu(*y,(1<<ADC_Q_FORM)-alpha);
    *y = ((tmp>>ADC_Q_FORM) + x*alpha);
}

void set_adc_zeros()
{
    adc_filter((int *)&adc_zero[ADC_CURRENT_A], adc_raw[ADC_CURRENT_A], 1);
    adc_filter((int *)&adc_zero[ADC_CURRENT_B], adc_raw[ADC_CURRENT_B], 1);
}

int get_adc_zero(int ch)
{
    return adc_zero[ch];
}

#if 0
void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void)
{
	static unsigned int count = 0;
	
	_AD1IF = 0;		//Clear the flag

        if(an_idx == 0)
            adc_raw[0]=ADC1BUF0;
        else if (an_idx == 1)
            adc_raw[1]=ADC1BUF0;

/*	if (AD1CON2bits.BUFS==0) //ADC module filling lower group, read from upper
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
	}*/
	
	#if defined MAX2_BDC_0_3_A2R2 || defined MAX2_BLDC_0_3_A2R2 || defined MAX2_BDC_0_2_A2R3 \
	|| defined  MAX2_BLDC_0_2_A2R3
	adc_buffer[ADC_MOTOR_TEMP][adc_idx/2]=adc_raw[ADC_MOTOR_TEMP];
	adc_buffer[ADC_AMP_TEMP][adc_idx/2]=adc_raw[ADC_AMP_TEMP];
        adc_buffer[ADC_CURRENT_A][adc_idx/2]=adc_raw[ADC_CURRENT_A];
	adc_buffer[ADC_CURRENT_B][adc_idx/2]=adc_raw[ADC_CURRENT_B];
	adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH*2);
	/*adc_buffer[ADC_CURRENT_A][adc_idx>>1]=adc_raw[ADC_CURRENT_A];
	adc_buffer[ADC_CURRENT_B][adc_idx>>1]=adc_raw[ADC_CURRENT_B];
	adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH*2);*/
	#endif
	
	#if defined MAX2_BLDC_0_3_T2R2 || defined MAX2_BDC_0_3_T2R2 
	adc_buffer[ADC_EXT][adc_idx]=adc_raw[ADC_EXT];
	adc_buffer[ADC_AMP_TEMP][adc_idx]=adc_raw[ADC_AMP_TEMP];
	adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
	adc_buffer[ADC_CURRENT_B][adc_idx]=adc_raw[ADC_CURRENT_B];
	adc_buffer_fast[adc_idx_fast]=adc_raw[ADC_EXT];
	adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
	adc_idx_fast=INC_MOD(adc_idx_fast,ADC_NUM_SMOOTH_FAST);
	#endif
	
	#if defined MAX2_BLDC_0_2_T2R3
	adc_buffer[ADC_MOTOR_TEMP][adc_idx]=adc_raw[ADC_MOTOR_TEMP];
	adc_buffer[ADC_AMP_TEMP][adc_idx]=adc_raw[ADC_AMP_TEMP];
	adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
	adc_buffer[ADC_CURRENT_B][adc_idx]=adc_raw[ADC_CURRENT_B];
	//adc_buffer_fast[adc_idx_fast]=adc_raw[ADC_EXT];
	adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
	adc_idx_fast=INC_MOD(adc_idx_fast,ADC_NUM_SMOOTH_FAST);
	#endif	
	
	#if defined MAX2_BDC_0_2_T2R3
	adc_buffer[ADC_AMP_TEMP][adc_idx]=adc_raw[ADC_AMP_TEMP];
	adc_buffer[ADC_CURRENT_A][adc_idx]=adc_raw[ADC_CURRENT_A];
	adc_buffer[ADC_CURRENT_B][adc_idx]=adc_raw[ADC_CURRENT_B];
	//adc_buffer_fast[adc_idx_fast]=adc_raw[ADC_EXT];
	adc_idx=INC_MOD(adc_idx,ADC_NUM_SMOOTH);
	adc_idx_fast=INC_MOD(adc_idx_fast,ADC_NUM_SMOOTH_FAST);
	#endif
	
	//Timed actions - 2kHz
	//Originaly in timer3 ISR
	//======================
	
	count = INC_MOD(count,5);		
	//if(count == 0)
        if(0)
	{
		//Latch encoder timestamp on Rising edge.
		#ifdef USE_TIMESTAMP_DC			//Takes 0.05us to execute
		SetTimestampLatch;	
		ClrTimestampLatch;
		#endif
	
		#if defined USE_ENCODER_VERTX	//Takes 122us to execute
		step_vertx();					
		#endif
	
		#ifdef USE_CURRENT				//Takes 0.475us to execute
		step_current();
		#endif
	
		#ifdef USE_CONTROL				//Takes 2.3us to execute
		step_control();					//(will probably increase in PID mode...)
		#endif
		
		//Torso external ADC
		#ifdef USE_ADC_SPI
		step_adc_spi();
		#endif
		
		irq_cnt++;
		
		
		#ifdef USE_WATCHDOG
		wd_cnt++;
		if ((ec_cmd.command[0].config & 0x4000) != last_status)		// if the WD bit changes, everything is cool
		{
			wd_cnt = 0;
			watchdog_expired = 0;
		}
		else if (wd_cnt > 500)					// if the status doesn't change in 250ms, problem
		{
			watchdog_expired = 1;
		}	
		last_status = (ec_cmd.command[0].config & 0x4000);
		#else
		watchdog_expired = 0;	//Always off
		#endif

		
		//Sum = 125us, 25% of this time slice
	}


        /////////////////////////////
        // ADC MANAGEMENT:
	an_idx ^= 1;
	AD1CON1bits.ADON	= 0;							// turn off the adc
	AD1CHS0bits.CH0SA	= an[an_idx];					// select the analog input
	AD1CON1bits.SSRC	= 0b011;						// set to trigger the sample from the pwm hardware
	AD1CON1bits.ADON	= 1;							// turn on the adc module

}
#endif
#endif
